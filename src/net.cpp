#include "net.h"
#include <curl/curl.h>
#include <condition_variable>
#include <mutex>
#include <string>
namespace arnis::net
{
namespace
{
std::mutex mutex;
std::condition_variable freed;
std::size_t in_flight = 0;
}
RequestPermit::RequestPermit()
{
	std::unique_lock lock(mutex);
	freed.wait(lock, [] { return in_flight < MAX_CONCURRENT_REQUESTS; });
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
RequestPermit request_permit()
{
	return RequestPermit{};
}
std::size_t in_flight_requests()
{
	std::lock_guard lock(mutex);
	return in_flight;
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
	return "other";
}
}
