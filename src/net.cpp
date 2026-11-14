#include "net.h"
#include <algorithm>
#include <cctype>
#include <curl/curl.h>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace arnis::net
{
namespace
{
std::mutex mutex;
std::condition_variable freed;
std::size_t in_flight = 0;
std::atomic<std::size_t> request_limit{MAX_CONCURRENT_REQUESTS};
std::atomic<bool> offline_run{false};
std::atomic<bool> prewarm_run{false};
std::atomic<bool> persistent_offline_mode{false};
std::mutex misses_mutex;
std::map<std::string, std::size_t> misses;
std::mutex uncached_mutex;
std::set<std::string> uncached_resources;
}
RequestPermit::RequestPermit(const std::string &resource)
{
	if (offline_run.load(std::memory_order_relaxed)) {
		{
			std::lock_guard lock(misses_mutex);
			++misses[resource];
		}
		throw std::runtime_error("offline mode: " + resource + " is not in the cache");
	}
	std::unique_lock lock(mutex);
	freed.wait(lock,
			[] { return in_flight < request_limit.load(std::memory_order_relaxed); });
	++in_flight;
	held_ = true;
}
RequestPermit::~RequestPermit()
{
	if (!held_)
		return;
	{
		std::lock_guard lock(mutex);
		if (in_flight)
			--in_flight;
	}
	freed.notify_one();
}
RequestPermit::RequestPermit(RequestPermit &&other) noexcept : held_(other.held_)
{
	other.held_ = false;
}
RequestPermit request_permit(const std::string &resource)
{
	return RequestPermit{resource};
}
void set_max_requests(std::size_t count)
{
	request_limit.store(std::max<std::size_t>(1, count), std::memory_order_relaxed);
	freed.notify_all();
}
std::size_t max_requests()
{
	return request_limit.load(std::memory_order_relaxed);
}
std::size_t in_flight_requests()
{
	std::lock_guard lock(mutex);
	return in_flight;
}
void set_offline(bool enabled)
{
	if (enabled) {
		std::lock_guard lock(misses_mutex);
		misses.clear();
	}
	offline_run.store(enabled, std::memory_order_relaxed);
}
bool offline()
{
	return offline_run.load(std::memory_order_relaxed);
}
std::vector<std::pair<std::string, std::size_t>> offline_misses()
{
	std::lock_guard lock(misses_mutex);
	return {misses.begin(), misses.end()};
}
void clear_offline_misses()
{
	std::lock_guard lock(misses_mutex);
	misses.clear();
}
void set_prewarm(bool enabled)
{
	if (enabled) {
		std::lock_guard lock(uncached_mutex);
		uncached_resources.clear();
	}
	prewarm_run.store(enabled, std::memory_order_relaxed);
}
bool prewarm()
{
	return prewarm_run.load(std::memory_order_relaxed);
}
void note_uncached(const std::string &resource)
{
	if (!prewarm())
		return;
	std::lock_guard lock(uncached_mutex);
	uncached_resources.insert(resource);
}
std::vector<std::string> uncached()
{
	std::lock_guard lock(uncached_mutex);
	return {uncached_resources.begin(), uncached_resources.end()};
}
void set_offline_mode(bool enabled)
{
	persistent_offline_mode.store(enabled, std::memory_order_relaxed);
}
bool offline_mode()
{
	return offline() || persistent_offline_mode.load(std::memory_order_relaxed);
}
RunScope::RunScope(bool offline_run_requested, bool prewarm_run_requested,
		std::optional<std::size_t> requested_limit) : active_(true)
{
	if (requested_limit) {
		previous_request_limit_ = max_requests();
		set_max_requests(*requested_limit);
	}
	set_prewarm(prewarm_run_requested);
	set_offline(offline_run_requested);
}
RunScope::~RunScope()
{
	if (active_) {
		set_offline(false);
		set_prewarm(false);
		if (previous_request_limit_)
			set_max_requests(*previous_request_limit_);
	}
}
std::string failure_kind(int transport_error, long http_status)
{
	if (http_status >= 100 && http_status <= 599)
		return "http_" + std::to_string(http_status);
	const auto code = static_cast<CURLcode>(transport_error);
	if (code == CURLE_OPERATION_TIMEDOUT)
		return "timeout";
	if (code == CURLE_COULDNT_RESOLVE_HOST || code == CURLE_COULDNT_RESOLVE_PROXY)
		return "dns";
	if (code == CURLE_SSL_CONNECT_ERROR || code == CURLE_PEER_FAILED_VERIFICATION ||
			code == CURLE_SSL_CERTPROBLEM || code == CURLE_SSL_CIPHER)
		return "tls";
	if (code == CURLE_RECV_ERROR || code == CURLE_SEND_ERROR ||
			code == CURLE_GOT_NOTHING || code == CURLE_PARTIAL_FILE)
		return "reset";
	if (code == CURLE_COULDNT_CONNECT)
		return "connect";
	if (code == CURLE_BAD_CONTENT_ENCODING)
		return "body";
	return "other";
}
bool is_failure_kind(const std::string &tag)
{
	if (tag == "timeout" || tag == "dns" || tag == "tls" || tag == "reset" ||
			tag == "connect" || tag == "body" || tag == "other")
		return true;
	if (tag.size() != 8 || tag.compare(0, 5, "http_") != 0)
		return false;
	return std::all_of(
			tag.begin() + 5, tag.end(), [](unsigned char c) { return std::isdigit(c); });
}
}
