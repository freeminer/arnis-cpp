#include "fetch.h"

#include "../net.h"

#include <curl/curl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <limits>
#include <mutex>
#include <string_view>
#include <thread>

namespace arnis::survey::fetch
{
namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t MAX_BODY = 256ULL << 20;
std::atomic<std::uint64_t> laser_total{0}, photo_total{0}, laser_area{0};
std::atomic<std::uint64_t> laser_limit{std::numeric_limits<std::uint64_t>::max()};
thread_local bool photo_thread = false;
std::mutex pacing_mutex;
std::optional<Clock::time_point> next_start;

struct Response
{
	std::vector<std::uint8_t> body;
	std::string retry_after;
	long status = 0;
	std::size_t body_limit = static_cast<std::size_t>(MAX_BODY);
	bool too_large = false;
};

size_t write_body(char *data, size_t size, size_t count, void *opaque)
{
	const auto bytes = size * count;
	auto &response = *static_cast<Response *>(opaque);
	const auto max_size = response.body_limit;
	if (bytes > max_size - std::min(max_size, response.body.size())) {
		response.too_large = true;
		return 0;
	}
	const auto *first = reinterpret_cast<const std::uint8_t *>(data);
	response.body.insert(response.body.end(), first, first + bytes);
	return bytes;
}

size_t read_header(char *data, size_t size, size_t count, void *opaque)
{
	const auto bytes = size * count;
	std::string_view line(data, bytes);
	constexpr std::string_view prefix = "retry-after:";
	if (line.size() >= prefix.size()) {
		auto key = line.substr(0, prefix.size());
		std::string lower(key);
		std::transform(lower.begin(), lower.end(), lower.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		if (lower == prefix) {
			auto value = line.substr(prefix.size());
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.front())))
				value.remove_prefix(1);
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.back())))
				value.remove_suffix(1);
			static_cast<Response *>(opaque)->retry_after.assign(value);
		}
	}
	return bytes;
}

std::optional<unsigned> paced_host(const std::string &url)
{
	auto begin = url.find("://");
	if (begin == std::string::npos)
		return std::nullopt;
	begin += 3;
	const auto end = url.find('/', begin);
	return url.substr(begin, end == std::string::npos ? end : end - begin) ==
						   "data.geopf.fr"
				   ? std::optional<unsigned>(8)
				   : std::nullopt;
}

void pace(unsigned rate)
{
	const auto gap = std::chrono::seconds(1) / rate;
	Clock::time_point start;
	{
		std::lock_guard lock(pacing_mutex);
		const auto now = Clock::now();
		start = next_start ? std::max(*next_start, now) : now;
		next_start = start + gap;
	}
	if (start > Clock::now())
		std::this_thread::sleep_until(start);
}

void back_off(std::chrono::milliseconds delay)
{
	std::lock_guard lock(pacing_mutex);
	const auto until = Clock::now() + delay;
	if (!next_start || *next_start < until)
		next_start = until;
}

std::optional<unsigned long> retry_after(const std::string &value)
{
	try {
		std::size_t consumed = 0;
		const auto seconds = std::stoul(value, &consumed);
		if (consumed == value.size())
			return static_cast<unsigned long>(std::clamp(seconds, 1UL, 15UL) * 1000UL);
	} catch (...) {
	}
	return std::nullopt;
}

std::string transport_kind(CURLcode code)
{
	switch (code) {
	case CURLE_OPERATION_TIMEDOUT:
		return "timeout";
	case CURLE_COULDNT_CONNECT:
		return "connect";
	case CURLE_COULDNT_RESOLVE_HOST:
	case CURLE_COULDNT_RESOLVE_PROXY:
		return "dns";
	case CURLE_SSL_CONNECT_ERROR:
	case CURLE_PEER_FAILED_VERIFICATION:
		return "tls";
	default:
		return "other";
	}
}

Result<std::vector<std::uint8_t>> get(const std::string &url,
		std::optional<std::pair<std::uint64_t, std::uint64_t>> from,
		std::optional<std::uint64_t> tail)
{
	std::uint64_t cap = MAX_BODY;
	if (from) {
		if (from->second == 0)
			return std::vector<std::uint8_t>{};
		cap = from->second;
	} else if (tail) {
		cap = *tail;
	}
	Response response;
	response.body_limit = static_cast<std::size_t>(
			std::min<std::uint64_t>(cap, std::numeric_limits<std::size_t>::max()));
	if (photo_thread && past_laser_limit())
		return Failure{Failure::Kind::Other, PAST_LIMIT};
	const auto rate = paced_host(url);
	unsigned wait_ms = 1000;
	for (int attempt = 0; attempt < 8; ++attempt) {
		if (rate)
			pace(*rate);
		response.body.clear();
		response.retry_after.clear();
		response.status = 0;
		response.too_large = false;
		CURLcode result = CURLE_FAILED_INIT;
		try {
			auto permit = net::request_permit("survey " + url);
			CURL *curl = curl_easy_init();
			if (!curl)
				return Failure{Failure::Kind::Other, "curl initialization failed"};
			curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_body);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
			curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, read_header);
			curl_easy_setopt(curl, CURLOPT_HEADERDATA, &response);
			curl_easy_setopt(curl, CURLOPT_USERAGENT, "arnis/1.0");
			curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
			curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
			curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 15L);
			curl_easy_setopt(curl, CURLOPT_TIMEOUT, 60L);
			curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
			std::string range;
			if (from) {
				const auto end = from->first > std::numeric_limits<std::uint64_t>::max() -
														 from->second + 1
										 ? std::numeric_limits<std::uint64_t>::max()
										 : from->first + from->second - 1;
				range = std::to_string(from->first) + "-" + std::to_string(end);
				curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
			} else if (tail) {
				range = "-" + std::to_string(*tail);
				curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
			}
			result = curl_easy_perform(curl);
			curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response.status);
			curl_easy_cleanup(curl);
		} catch (const std::exception &e) {
			return Failure{Failure::Kind::Other, e.what()};
		}
		if (result == CURLE_OK || response.too_large) {
			if ((response.status == 429 || response.status == 502 ||
						response.status == 503 || response.status == 504) &&
					attempt < 7) {
				const auto delay = std::chrono::milliseconds(
						retry_after(response.retry_after).value_or(wait_ms));
				if (rate)
					back_off(delay);
				else
					std::this_thread::sleep_for(delay);
				wait_ms = std::min(wait_ms * 2, 15000U);
				continue;
			}
			if (response.status == 403 || response.status == 404)
				return Failure{Failure::Kind::Missing, "not found"};
			if ((from || tail) && response.status != 206)
				return Failure{Failure::Kind::Other,
						"HTTP " + std::to_string(response.status) + " (expected 206)"};
			if (response.status < 200 || response.status >= 300)
				return Failure{
						Failure::Kind::Other, "HTTP " + std::to_string(response.status)};
			if (response.too_large || response.body.size() > cap)
				return Failure{Failure::Kind::Other, "answer larger than asked for"};
			if (photo_thread)
				photo_total.fetch_add(response.body.size(), std::memory_order_relaxed);
			return std::move(response.body);
		}
		const auto kind = transport_kind(result);
		if (attempt < 2 && kind != "dns" && kind != "timeout") {
			std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
			wait_ms *= 2;
			continue;
		}
		return Failure{Failure::Kind::Other, kind + ": " + curl_easy_strerror(result)};
	}
	return Failure{Failure::Kind::Other, "server busy"};
}
} // namespace

std::string Failure::to_string() const
{
	return message;
}
Failure Failure::of_part() const
{
	return kind == Kind::Missing ? Failure{Kind::Other, "part of the file is missing"}
								 : *this;
}
std::uint64_t laser_bytes()
{
	return laser_total.load(std::memory_order_relaxed);
}
void count_laser_bytes(std::uint64_t bytes)
{
	laser_total.fetch_add(bytes, std::memory_order_relaxed);
}
std::uint64_t photo_bytes()
{
	return photo_total.load(std::memory_order_relaxed);
}
void for_photos()
{
	photo_thread = true;
}
void limit_laser_bytes(std::uint64_t bytes)
{
	const auto used = laser_bytes() + photo_bytes();
	laser_limit.store(used > std::numeric_limits<std::uint64_t>::max() - bytes
							  ? std::numeric_limits<std::uint64_t>::max()
							  : used + bytes,
			std::memory_order_relaxed);
}
bool past_laser_limit()
{
	return laser_bytes() + photo_bytes() > laser_limit.load(std::memory_order_relaxed);
}
Result<bool> within_laser_limit()
{
	if (past_laser_limit())
		return Failure{Failure::Kind::Other, PAST_LIMIT};
	return true;
}
std::uint64_t laser_area_m2()
{
	return laser_area.load(std::memory_order_relaxed);
}
void count_laser_area(double area_m2)
{
	if (area_m2 > 0) {
		const auto bounded = std::min(
				area_m2, static_cast<double>(std::numeric_limits<std::uint64_t>::max()));
		laser_area.fetch_add(
				static_cast<std::uint64_t>(bounded), std::memory_order_relaxed);
	}
}
bool unreachable(const std::string &kind)
{
	return kind == "timeout" || kind == "connect" || kind == "dns" || kind == "tls" ||
		   kind == PAST_LIMIT;
}
Result<std::vector<std::uint8_t>> get_range(
		const std::string &url, std::uint64_t start, std::uint64_t length)
{
	auto path = url;
	const auto marker = path.rfind("#at=");
	if (marker != std::string::npos) {
		try {
			const auto base = std::stoull(path.substr(marker + 4));
			start = base > std::numeric_limits<std::uint64_t>::max() - start
							? std::numeric_limits<std::uint64_t>::max()
							: base + start;
			path.resize(marker);
		} catch (...) {
		}
	}
	return get(path, std::pair{start, length}, std::nullopt);
}
Result<std::vector<std::uint8_t>> get_tail(const std::string &url, std::uint64_t length)
{
	return get(url, std::nullopt, length);
}
Result<std::vector<std::uint8_t>> get_bytes(const std::string &url)
{
	return get(url, std::nullopt, std::nullopt);
}
std::optional<std::uint64_t> content_length(const std::string &url)
{
	try {
		auto permit = net::request_permit("survey HEAD " + url);
		CURL *curl = curl_easy_init();
		if (!curl)
			return std::nullopt;
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, "arnis/1.0");
		curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "identity");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 15L);
		curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
		curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
		const auto result = curl_easy_perform(curl);
		long status = 0;
		curl_off_t length = -1;
		curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
		curl_easy_getinfo(curl, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &length);
		curl_easy_cleanup(curl);
		if (result != CURLE_OK)
			return std::nullopt;
		if (status == 403 || status == 404)
			return 0;
		if (status < 200 || status >= 300 || length < 0)
			return std::nullopt;
		return static_cast<std::uint64_t>(length);
	} catch (...) {
		return std::nullopt;
	}
}
Result<nlohmann::json> get_json(const std::string &url)
{
	auto response = get_bytes(url);
	if (const auto *failure = std::get_if<Failure>(&response))
		return Failure{Failure::Kind::Other, url + ": " + failure->to_string()};
	try {
		const auto &bytes = std::get<std::vector<std::uint8_t>>(response);
		return nlohmann::json::parse(bytes.begin(), bytes.end());
	} catch (const std::exception &e) {
		return Failure{Failure::Kind::Other, url + ": " + e.what()};
	}
}
} // namespace arnis::survey::fetch
