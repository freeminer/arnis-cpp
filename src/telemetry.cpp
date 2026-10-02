#include "telemetry.h"

#include "version.h"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <exception>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>
#include <utility>

namespace arnis::telemetry
{
namespace
{
constexpr const char *TELEMETRY_URL =
		"https://arnismc.com/telemetry/report_telemetry.php";
constexpr unsigned MAX_SENDS_IN_FLIGHT = 4;
std::atomic<bool> enabled{false};
std::atomic<unsigned> sends_in_flight{0};
std::once_flag curl_init_once;
std::once_flag shared_init_once;
CURLSH *shared = nullptr;
std::array<std::mutex, CURL_LOCK_DATA_LAST> share_mutexes;

void share_lock(CURL *, curl_lock_data data, curl_lock_access, void *)
{
	if (data >= 0 && data < CURL_LOCK_DATA_LAST)
		share_mutexes[static_cast<std::size_t>(data)].lock();
}

void share_unlock(CURL *, curl_lock_data data, void *)
{
	if (data >= 0 && data < CURL_LOCK_DATA_LAST)
		share_mutexes[static_cast<std::size_t>(data)].unlock();
}

CURLSH *shared_client()
{
	std::call_once(curl_init_once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
	std::call_once(shared_init_once, [] {
		shared = curl_share_init();
		if (!shared)
			return;
		curl_share_setopt(shared, CURLSHOPT_LOCKFUNC, share_lock);
		curl_share_setopt(shared, CURLSHOPT_UNLOCKFUNC, share_unlock);
		curl_share_setopt(shared, CURLSHOPT_SHARE, CURL_LOCK_DATA_DNS);
		curl_share_setopt(shared, CURLSHOPT_SHARE, CURL_LOCK_DATA_SSL_SESSION);
		curl_share_setopt(shared, CURLSHOPT_SHARE, CURL_LOCK_DATA_CONNECT);
	});
	return shared;
}

std::size_t discard_response(char *, std::size_t size, std::size_t count, void *)
{
	return size * count;
}

bool post_payload(const nlohmann::json &payload, long timeout_seconds, bool reuse_client)
{
	CURL *raw = curl_easy_init();
	if (!raw)
		return false;
	const auto cleanup = [](CURL *curl) { curl_easy_cleanup(curl); };
	std::unique_ptr<CURL, decltype(cleanup)> curl(raw, cleanup);
	if (reuse_client)
		if (auto *share = shared_client())
			curl_easy_setopt(raw, CURLOPT_SHARE, share);
	const std::string body = payload.dump();
	struct curl_slist *headers = nullptr;
	headers = curl_slist_append(headers, "Content-Type: application/json");
	curl_easy_setopt(raw, CURLOPT_URL, TELEMETRY_URL);
	curl_easy_setopt(raw, CURLOPT_POST, 1L);
	curl_easy_setopt(raw, CURLOPT_POSTFIELDS, body.data());
	curl_easy_setopt(
			raw, CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(body.size()));
	curl_easy_setopt(raw, CURLOPT_HTTPHEADER, headers);
	curl_easy_setopt(raw, CURLOPT_WRITEFUNCTION, discard_response);
	curl_easy_setopt(raw, CURLOPT_TIMEOUT, timeout_seconds);
	curl_easy_setopt(raw, CURLOPT_CONNECTTIMEOUT, 5L);
	curl_easy_setopt(raw, CURLOPT_NOSIGNAL, 1L);
	curl_easy_setopt(
			raw, CURLOPT_USERAGENT, "Arnis/Cpp (+https://github.com/louis-e/arnis)");
	const auto result = curl_easy_perform(raw);
	curl_slist_free_all(headers);
	return result == CURLE_OK;
}

const char *level_name(LogLevel level)
{
	switch (level) {
	case LogLevel::Debug:
		return "debug";
	case LogLevel::Info:
		return "info";
	case LogLevel::Warning:
		return "warning";
	case LogLevel::Error:
		return "error";
	}
	return "info";
}

class SendSlot
{
	bool held = false;

public:
	static SendSlot acquire()
	{
		unsigned count = sends_in_flight.load(std::memory_order_acquire);
		while (count < MAX_SENDS_IN_FLIGHT) {
			if (sends_in_flight.compare_exchange_weak(count, count + 1,
						std::memory_order_acq_rel, std::memory_order_acquire))
				return SendSlot(true);
		}
		return {};
	}
	SendSlot() = default;
	SendSlot(const SendSlot &) = delete;
	SendSlot &operator=(const SendSlot &) = delete;
	SendSlot(SendSlot &&other) noexcept : held(std::exchange(other.held, false)) {}
	SendSlot &operator=(SendSlot &&other) noexcept
	{
		if (this != &other) {
			release();
			held = std::exchange(other.held, false);
		}
		return *this;
	}
	~SendSlot() { release(); }
	bool valid() const { return held; }

private:
	explicit SendSlot(bool value) : held(value) {}
	void release()
	{
		if (held) {
			sends_in_flight.fetch_sub(1, std::memory_order_acq_rel);
			held = false;
		}
	}
};

bool release_build()
{
#ifdef NDEBUG
	return true;
#else
	return false;
#endif
}

void truncate_characters(std::string &value, std::size_t max_characters)
{
	std::size_t i = 0;
	std::size_t characters = 0;
	while (i < value.size() && characters < max_characters) {
		const auto lead = static_cast<unsigned char>(value[i]);
		std::size_t width = lead < 0x80				? 1
							: (lead & 0xE0) == 0xC0 ? 2
							: (lead & 0xF0) == 0xE0 ? 3
							: (lead & 0xF8) == 0xF0 ? 4
													: 1;
		if (i + width > value.size())
			width = 1;
		i += width;
		++characters;
	}
	if (i < value.size())
		value.resize(i);
}

} // namespace

void set_consent(bool value)
{
	enabled.store(value, std::memory_order_relaxed);
}

bool consent()
{
	return enabled.load(std::memory_order_relaxed);
}

const char *platform()
{
#if defined(_WIN32)
	return "windows";
#elif defined(__APPLE__)
	return "macos";
#elif defined(__linux__)
	return "linux";
#else
	return "unknown";
#endif
}

const char *event_name_generation_click()
{
	return "generation_click";
}

std::string redact_url_queries(const std::string &message)
{
	constexpr std::array<const char *, 2> tile_hosts = {
			"tiles.mapterhorn.com", "s3.amazonaws.com/elevation-tiles-prod"};
	std::string out;
	std::string_view input(message);
	out.reserve(message.size());
	for (;;) {
		const auto scheme = input.find("://");
		if (scheme == std::string_view::npos)
			break;
		out.append(input.substr(0, scheme + 3));
		input.remove_prefix(scheme + 3);
		const auto end = input.find_first_of(" \t\r\n)");
		const auto url = input.substr(0, end);
		const auto host = std::find_if(tile_hosts.begin(), tile_hosts.end(),
				[&](const char *prefix) { return url.starts_with(prefix); });
		if (host != tile_hosts.end()) {
			out += *host;
		} else if (const auto query = url.find('?'); query != std::string_view::npos) {
			out.append(url.substr(0, query));
		} else {
			out.append(url);
		}
		if (end == std::string_view::npos) {
			input = {};
			break;
		}
		input.remove_prefix(end);
	}
	out.append(input);
	return out;
}

void send_log(LogLevel level, const std::string &message)
{
	if (!consent() || !release_build())
		return;
	auto slot = SendSlot::acquire();
	if (!slot.valid())
		return;
	auto redacted = redact_url_queries(message);
	truncate_characters(redacted, 1000);
	try {
		std::thread([slot = std::move(slot), level,
							message = std::move(redacted)]() mutable {
			try {
				(void)post_payload(
						{{"type", "log"}, {"log_level", level_name(level)},
								{"log_message", message}, {"platform", platform()},
								{"app_version",
										g_version_string ? g_version_string : "unknown"}},
						15, true);
			} catch (...) {
			}
		}).detach();
	} catch (...) {
		// Telemetry must never make a resource-exhaustion condition worse.
	}
}

void send_generation_click()
{
	if (!consent() || !release_build())
		return;
	try {
		std::thread([] {
			try {
				(void)post_payload({{"type", "generation_click"}}, 15, true);
			} catch (...) {
			}
		}).detach();
	} catch (...) {
	}
}

void send_crash_report(const std::string &message)
{
	if (!consent() || !release_build())
		return;
	auto redacted = redact_url_queries(message);
	truncate_characters(redacted, 500);
	try {
		(void)post_payload(
				{{"type", "crash"}, {"error_message", redacted}, {"platform", platform()},
						{"app_version", g_version_string ? g_version_string : "unknown"}},
				10, false);
	} catch (...) {
	}
}

} // namespace arnis::telemetry
