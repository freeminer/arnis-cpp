#include "osm_pbf.h"
#include "cache_root.h"
#include "net.h"
#include "progress.h"
#include "retrieve_data.h"
#include "transfer.h"
#include "world_utils.h"
#include "util/hashing.h"

#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <cctype>

namespace arnis::osm_pbf
{
namespace
{
using Point = std::pair<double, double>; // longitude, latitude
using Ring = std::vector<Point>;
using Rings = std::vector<Ring>;
constexpr const char *GEOFABRIK = "geofabrik";
constexpr const char *GEOFABRIK_INDEX_URL = "https://download.geofabrik.de/index-v1.json";
constexpr std::uint64_t AUTO_EXTRACT_MAX_BYTES = 1'500'000'000;
constexpr std::uint64_t INDEX_MAX_BYTES = 16 * 1024 * 1024;

std::string url_key(const std::string &url)
{
	// Match Rust's cache key: the first six SHA-256 bytes as twelve hex digits.
	const auto digest = hashing::sha256(url);
	std::ostringstream out;
	for (std::size_t i = 0; i < 6; ++i)
		out << std::hex << std::setfill('0') << std::setw(2)
			<< static_cast<unsigned>(static_cast<unsigned char>(digest[i]));
	return out.str();
}

std::optional<std::uint64_t> recorded_size(
		const std::filesystem::path &directory, const std::string &name)
{
	std::ifstream input(directory / "sizes.json", std::ios::binary);
	if (!input)
		return std::nullopt;
	const auto sizes = nlohmann::json::parse(input, nullptr, false);
	if (!sizes.is_object())
		return std::nullopt;
	const auto found = sizes.find(name);
	if (found == sizes.end() || !found->is_number_unsigned())
		return std::nullopt;
	return found->get<std::uint64_t>();
}

void remember_size(const std::filesystem::path &directory, const std::string &name,
		std::uint64_t size)
{
	const auto path = directory / "sizes.json";
	nlohmann::json sizes = nlohmann::json::object();
	{
		std::ifstream input(path, std::ios::binary);
		if (input) {
			auto existing = nlohmann::json::parse(input, nullptr, false);
			if (existing.is_object())
				sizes = std::move(existing);
		}
	}
	const auto previous = sizes.find(name);
	if (previous != sizes.end() && previous->is_number_unsigned() &&
			previous->get<std::uint64_t>() == size)
		return;
	sizes[name] = size;
	const auto body = sizes.dump();
	world_utils::replace_file_atomically(
			path, std::vector<std::uint8_t>(body.begin(), body.end()));
}

std::optional<std::string> pbf_filename(const std::string &url)
{
	const auto slash = url.rfind('/');
	if (slash == std::string::npos || slash + 1 == url.size())
		return std::nullopt;
	const auto name = url.substr(slash + 1);
	if (name.size() <= 4 || name.front() == '.' ||
			name.find_first_of("\\:/?#") != std::string::npos ||
			name.substr(name.size() - 4) != ".pbf")
		return std::nullopt;
	return name;
}

std::shared_ptr<std::mutex> url_mutex(const std::filesystem::path &path)
{
	static std::mutex mutex;
	static std::unordered_map<std::string, std::weak_ptr<std::mutex>> locks;
	const auto key = path.string();
	const std::lock_guard lock(mutex);
	auto &entry = locks[key];
	auto result = entry.lock();
	if (!result) {
		result = std::make_shared<std::mutex>();
		entry = result;
	}
	return result;
}

struct CurlBody
{
	std::string *memory = nullptr;
	std::ofstream *file = nullptr;
	std::mutex *file_mutex = nullptr;
	std::uint64_t file_offset = 0;
	std::uint64_t bytes = 0;
	std::uint64_t max_bytes = 0;
	std::atomic_uint64_t *downloaded = nullptr;
	bool exceeded = false;
	bool write_failed = false;
};

struct CurlHeaders
{
	std::unordered_map<std::string, std::string> values;
	std::uint64_t content_length = 0;
};

std::string trim(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}

std::size_t curl_header(char *data, std::size_t size, std::size_t count, void *opaque)
{
	if (size != 0 && count > std::numeric_limits<std::size_t>::max() / size)
		return 0;
	const auto bytes = size * count;
	auto &headers = *static_cast<CurlHeaders *>(opaque);
	const std::string line(data, bytes);
	if (line.rfind("HTTP/", 0) == 0) {
		headers.values.clear();
		headers.content_length = 0;
		return bytes;
	}
	const auto colon = line.find(':');
	if (colon != std::string::npos) {
		std::string key = line.substr(0, colon);
		std::transform(key.begin(), key.end(), key.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		const auto value = trim(line.substr(colon + 1));
		headers.values[key] = value;
		if (key == "content-length") {
			try {
				headers.content_length = std::stoull(value);
			} catch (...) {
				headers.content_length = 0;
			}
		}
	}
	return bytes;
}

std::size_t curl_write(void *data, std::size_t size, std::size_t count, void *opaque)
{
	auto &body = *static_cast<CurlBody *>(opaque);
	if (size != 0 && count > std::numeric_limits<std::size_t>::max() / size)
		return 0;
	const auto bytes = size * count;
	if (body.max_bytes && bytes > body.max_bytes - std::min(body.bytes, body.max_bytes)) {
		body.exceeded = true;
		return 0;
	}
	if (body.memory)
		body.memory->append(static_cast<const char *>(data), bytes);
	else if (body.file) {
		if (body.file_mutex) {
			std::lock_guard lock(*body.file_mutex);
			body.file->seekp(static_cast<std::streamoff>(body.file_offset));
			body.file->write(
					static_cast<const char *>(data), static_cast<std::streamsize>(bytes));
		} else {
			body.file->write(
					static_cast<const char *>(data), static_cast<std::streamsize>(bytes));
		}
		if (!*body.file) {
			body.write_failed = true;
			return 0;
		}
		body.file_offset += bytes;
	}
	if (body.file && !*body.file)
		return 0;
	body.bytes += bytes;
	if (body.downloaded)
		body.downloaded->fetch_add(bytes, std::memory_order_relaxed);
	return bytes;
}

bool is_byte_span(const std::string &header, std::uint64_t start, std::uint64_t end,
		std::uint64_t total)
{
	const std::string prefix = "bytes ";
	if (header.compare(0, prefix.size(), prefix) != 0)
		return false;
	const auto slash = header.find('/');
	const auto dash = header.find('-', prefix.size());
	if (slash == std::string::npos || dash == std::string::npos || dash > slash)
		return false;
	try {
		const auto got_start =
				std::stoull(header.substr(prefix.size(), dash - prefix.size()));
		const auto got_end = std::stoull(header.substr(dash + 1, slash - dash - 1));
		const auto got_total = header.substr(slash + 1);
		return got_start == start && got_end == end &&
			   (got_total == "*" || std::stoull(got_total) == total);
	} catch (...) {
		return false;
	}
}

struct ByteSpan
{
	std::uint64_t begin;
	std::uint64_t end;
};

std::size_t segment_count(std::uint64_t total, bool ranges)
{
	if (!ranges || !total)
		return 1;
	constexpr std::uint64_t MIN_SEGMENT = 8ULL << 20;
	return std::clamp<std::size_t>(static_cast<std::size_t>(total / MIN_SEGMENT), 1,
			std::min<std::size_t>(net::max_requests(), 8));
}

std::optional<std::string> fetch_index(std::string *error)
{
	const auto root = cache_root();
	const auto path = root / "geofabrik-index.json";
	std::error_code ec;
	const auto modified = std::filesystem::last_write_time(path, ec);
	const bool cached = !ec && std::filesystem::is_regular_file(path, ec);
	const auto now = std::filesystem::file_time_type::clock::now();
	const bool fresh =
			cached && now >= modified && now - modified < std::chrono::hours(24 * 7);
	if (cached && (fresh || net::offline())) {
		std::ifstream in(path, std::ios::binary);
		std::ostringstream bytes;
		bytes << in.rdbuf();
		if (in.good() || in.eof()) {
			auto body = bytes.str();
			const auto parsed = nlohmann::json::parse(body, nullptr, false);
			if (!parsed.is_discarded() && parsed.contains("features") &&
					parsed["features"].is_array())
				return body;
		}
	}
	std::unique_ptr<net::RequestPermit> permit;
	try {
		permit = std::make_unique<net::RequestPermit>(
				net::request_permit("Geofabrik extract index"));
	} catch (const std::exception &e) {
		if (error)
			*error = e.what();
		return std::nullopt;
	}
	CURL *curl = curl_easy_init();
	if (!curl) {
		if (error)
			*error = "could not initialize Geofabrik index request";
		return std::nullopt;
	}
	std::string bytes;
	CurlBody sink;
	sink.memory = &bytes;
	sink.max_bytes = INDEX_MAX_BYTES;
	curl_easy_setopt(curl, CURLOPT_URL, GEOFABRIK_INDEX_URL);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sink);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
	curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 30L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);
	long status = 0;
	const auto result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300) {
		if (error)
			*error = std::string("Geofabrik index request failed: ") +
					 (result == CURLE_OK ? "HTTP " + std::to_string(status)
										 : curl_easy_strerror(result));
		return std::nullopt;
	}
	if (sink.exceeded) {
		if (error)
			*error = "Geofabrik index exceeded the size limit";
		return std::nullopt;
	}
	const auto parsed = nlohmann::json::parse(bytes, nullptr, false);
	if (parsed.is_discarded() || !parsed.contains("features") ||
			!parsed["features"].is_array()) {
		if (error)
			*error = "bad Geofabrik index";
		return std::nullopt;
	}
	std::error_code dir_error;
	std::filesystem::create_directories(root, dir_error);
	if (!dir_error) {
		const std::vector<std::uint8_t> data(bytes.begin(), bytes.end());
		world_utils::replace_file_atomically(path, data);
	}
	return bytes;
}

std::optional<std::filesystem::path> download_extract(const std::string &url,
		std::optional<std::uint64_t> max_bytes, std::string *error)
{
	const auto name = pbf_filename(url);
	if (!name) {
		if (error)
			*error = "not a .pbf URL: " + url;
		return std::nullopt;
	}
	const auto root = cache_root() / "downloads" / url_key(url);
	const auto destination = root / *name;
	const auto per_url = url_mutex(destination);
	const std::lock_guard request_lock(*per_url);
	std::error_code ec;
	if (std::filesystem::is_regular_file(destination, ec)) {
		const auto size = std::filesystem::file_size(destination, ec);
		const auto declared = recorded_size(root, *name);
		if (!ec && (!declared || size == *declared) && (!max_bytes || size <= *max_bytes))
			return destination;
		std::filesystem::remove(destination, ec);
	}
	std::filesystem::create_directories(root, ec);
	if (ec) {
		if (error)
			*error = root.string() + ": " + ec.message();
		return std::nullopt;
	}
	static std::atomic_uint64_t sequence{0};
	auto part = destination;
	part += "." +
			std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) +
			"-" + std::to_string(sequence.fetch_add(1, std::memory_order_relaxed)) +
			".part";
	// HEAD fixes the target version and determines whether byte-range workers are
	// safe. A -latest redirect is resolved once so every worker hits the same URL.
	CurlHeaders remote_headers;
	std::string effective_url = url;
	long head_status = 0;
	CURLcode head_result = CURLE_OK;
	{
		auto permit = net::request_permit("Geofabrik extract metadata " + *name);
		CURL *head = curl_easy_init();
		if (!head) {
			if (error)
				*error = "could not initialize Geofabrik extract metadata request";
			return std::nullopt;
		}
		curl_easy_setopt(head, CURLOPT_URL, url.c_str());
		curl_easy_setopt(head, CURLOPT_NOBODY, 1L);
		curl_easy_setopt(head, CURLOPT_HEADERFUNCTION, curl_header);
		curl_easy_setopt(head, CURLOPT_HEADERDATA, &remote_headers);
		curl_easy_setopt(head, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
		curl_easy_setopt(head, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(head, CURLOPT_CONNECTTIMEOUT, 30L);
		curl_easy_setopt(head, CURLOPT_TIMEOUT, 120L);
		head_result = curl_easy_perform(head);
		curl_easy_getinfo(head, CURLINFO_RESPONSE_CODE, &head_status);
		char *resolved = nullptr;
		curl_easy_getinfo(head, CURLINFO_EFFECTIVE_URL, &resolved);
		if (resolved)
			effective_url = resolved;
		curl_off_t length = -1;
		curl_easy_getinfo(head, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &length);
		if (length > 0)
			remote_headers.content_length = static_cast<std::uint64_t>(length);
		curl_easy_cleanup(head);
	}
	if (head_result != CURLE_OK || head_status < 200 || head_status >= 300) {
		if (error)
			*error = "Geofabrik extract metadata request failed: " +
					 (head_result == CURLE_OK ? "HTTP " + std::to_string(head_status)
											  : curl_easy_strerror(head_result));
		return std::nullopt;
	}
	const auto total = remote_headers.content_length;
	if (max_bytes && total > *max_bytes) {
		if (error)
			*error = "Geofabrik extract exceeds the automatic selection size limit";
		return std::nullopt;
	}
	if (total > 0)
		remember_size(root, *name, total);
	const auto accept_ranges = remote_headers.values.find("accept-ranges");
	const bool supports_ranges = accept_ranges != remote_headers.values.end() &&
								 accept_ranges->second == "bytes" && total > 0;
	const auto worker_count = segment_count(total, supports_ranges);
	std::vector<ByteSpan> spans;
	if (supports_ranges && worker_count > 1) {
		for (std::size_t i = 0; i < worker_count; ++i) {
			const auto begin = total * i / worker_count;
			const auto end_exclusive = total * (i + 1) / worker_count;
			if (begin < end_exclusive)
				spans.push_back({begin, end_exclusive - 1});
		}
	} else if (supports_ranges) {
		spans.push_back({0, total - 1});
	} else {
		spans.push_back({0, std::numeric_limits<std::uint64_t>::max()});
	}
	std::ofstream output(part, std::ios::binary | std::ios::trunc);
	if (!output) {
		if (error)
			*error = "cannot create " + part.string();
		return std::nullopt;
	}
	if (total > 0) {
		output.seekp(static_cast<std::streamoff>(total - 1));
		output.put('\0');
		output.flush();
		output.seekp(0);
	}
	std::mutex output_mutex;
	std::atomic_uint64_t downloaded{0};
	std::atomic_bool failed{false};
	std::atomic_size_t active_workers{spans.size()};
	std::mutex error_mutex;
	std::string worker_error;
	const auto started = std::chrono::steady_clock::now();
	transfer::RateMeter rate;
	auto emit_transfer = [&](std::uint64_t done, double speed) {
		transfer::Transfer event;
		event.name = *name;
		event.item = 1;
		event.items = 1;
		event.done_bytes = done;
		event.total_bytes = total;
		event.rate_bps = speed;
		event.downloads = static_cast<std::uint32_t>(spans.size());
		event.percent = total ? 40.0 * static_cast<double>(done) / total : 0.0;
		const auto text =
				total ? "Downloading " + *name + ": " +
								std::to_string(static_cast<int>(100.0 * done / total)) +
								"%"
					  : "Downloading " + *name;
		progress::emit_gui_transfer(event.percent, text, event);
	};
	emit_transfer(0, 0.0);
	auto fetch = [&](const ByteSpan span) {
		try {
			auto permit = net::request_permit("Geofabrik extract " + *name);
			CURL *curl = curl_easy_init();
			if (!curl)
				throw std::runtime_error(
						"could not initialize Geofabrik extract request");
			CurlBody sink;
			sink.file = &output;
			sink.file_mutex = &output_mutex;
			sink.file_offset = span.begin;
			sink.max_bytes = max_bytes.value_or(0);
			sink.downloaded = &downloaded;
			CurlHeaders response_headers;
			struct curl_slist *request_headers = nullptr;
			curl_easy_setopt(curl, CURLOPT_URL, effective_url.c_str());
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &sink);
			curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, curl_header);
			curl_easy_setopt(curl, CURLOPT_HEADERDATA, &response_headers);
			curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
			curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "identity");
			curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
			curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 30L);
			curl_easy_setopt(curl, CURLOPT_TIMEOUT, 0L);
			if (span.end != std::numeric_limits<std::uint64_t>::max()) {
				const auto range =
						std::to_string(span.begin) + "-" + std::to_string(span.end);
				curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
				const auto etag = remote_headers.values.find("etag");
				const auto modified = remote_headers.values.find("last-modified");
				std::string if_range;
				if (etag != remote_headers.values.end() &&
						etag->second.rfind("W/", 0) != 0)
					if_range = etag->second;
				else if (modified != remote_headers.values.end())
					if_range = modified->second;
				if (!if_range.empty()) {
					const auto header = "If-Range: " + if_range;
					request_headers = curl_slist_append(nullptr, header.c_str());
					curl_easy_setopt(curl, CURLOPT_HTTPHEADER, request_headers);
				}
			}
			const auto result = curl_easy_perform(curl);
			long status = 0;
			curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
			curl_slist_free_all(request_headers);
			curl_easy_cleanup(curl);
			if (result != CURLE_OK || status < 200 || status >= 300 || sink.exceeded ||
					sink.write_failed) {
				throw std::runtime_error(
						sink.exceeded
								? "Geofabrik extract exceeds the automatic selection size limit"
								: "Geofabrik extract download failed: " +
										  (result == CURLE_OK
														  ? "HTTP " +
																	std::to_string(status)
														  : curl_easy_strerror(result)));
			}
			if (span.end != std::numeric_limits<std::uint64_t>::max()) {
				const auto range = response_headers.values.find("content-range");
				const auto expected = span.end - span.begin + 1;
				if (status != 206 || range == response_headers.values.end() ||
						!is_byte_span(range->second, span.begin, span.end, total) ||
						sink.bytes != expected)
					throw std::runtime_error(
							"server returned an invalid byte range; extract may have changed");
			}
		} catch (const std::exception &e) {
			failed.store(true, std::memory_order_release);
			std::lock_guard lock(error_mutex);
			if (worker_error.empty())
				worker_error = e.what();
		}
		active_workers.fetch_sub(1, std::memory_order_release);
	};
	std::vector<std::thread> workers;
	workers.reserve(spans.size());
	for (const auto span : spans)
		workers.emplace_back(fetch, span);
	while (active_workers.load(std::memory_order_acquire) != 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		const auto done = downloaded.load(std::memory_order_relaxed);
		const auto elapsed =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		emit_transfer(done, rate.add(elapsed, done));
		if (failed.load(std::memory_order_acquire))
			break;
	}
	// Joining is also the lifetime barrier for the shared output stream/callbacks.
	for (auto &worker : workers)
		if (worker.joinable())
			worker.join();
	const auto final_done = downloaded.load(std::memory_order_relaxed);
	const auto elapsed =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
					.count();
	emit_transfer(final_done, rate.add(elapsed, final_done));
	output.close();
	const auto expected_bytes = total;
	if (failed.load(std::memory_order_acquire) ||
			(expected_bytes &&
					downloaded.load(std::memory_order_relaxed) != expected_bytes) ||
			!output) {
		std::filesystem::remove(part, ec);
		if (error)
			*error = worker_error.empty() ? "Geofabrik extract download was incomplete"
										  : worker_error;
		return std::nullopt;
	}
	std::filesystem::rename(part, destination, ec);
	if (ec) {
		// A different process may have completed the same cache key meanwhile.
		if (!std::filesystem::is_regular_file(destination)) {
			std::filesystem::remove(part, ec);
			if (error)
				*error = "cannot install Geofabrik extract: " + ec.message();
			return std::nullopt;
		}
		std::filesystem::remove(part, ec);
	}
	if (error)
		error->clear();
	return destination;
}

void append_ring(const nlohmann::json &value, Rings &out)
{
	if (!value.is_array())
		return;
	Ring ring;
	for (const auto &point : value) {
		if (!point.is_array() || point.size() < 2 || !point[0].is_number() ||
				!point[1].is_number())
			return;
		const double lon = point[0].get<double>();
		const double lat = point[1].get<double>();
		if (!std::isfinite(lon) || !std::isfinite(lat))
			return;
		ring.emplace_back(lon, lat);
	}
	if (ring.size() >= 3)
		out.push_back(std::move(ring));
}

Rings geometry_rings(const nlohmann::json &geometry)
{
	Rings rings;
	const auto type = geometry.value("type", std::string{});
	const auto coordinates = geometry.find("coordinates");
	if (coordinates == geometry.end() || !coordinates->is_array())
		return rings;
	if (type == "Polygon") {
		for (const auto &ring : *coordinates)
			append_ring(ring, rings);
	} else if (type == "MultiPolygon") {
		for (const auto &polygon : *coordinates) {
			if (!polygon.is_array())
				continue;
			for (const auto &ring : polygon)
				append_ring(ring, rings);
		}
	}
	return rings;
}

bool inside_rings(double x, double y, const Rings &rings)
{
	bool inside = false;
	for (const auto &ring : rings) {
		if (ring.size() < 3)
			continue;
		std::size_t j = ring.size() - 1;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const auto [xi, yi] = ring[i];
			const auto [xj, yj] = ring[j];
			if ((yi > y) != (yj > y) && x < xj + (y - yj) * (xi - xj) / (yi - yj))
				inside = !inside;
			j = i;
		}
	}
	return inside;
}

double area(const Rings &rings)
{
	double result = 0.0;
	for (const auto &ring : rings) {
		if (ring.size() < 3)
			continue;
		double twice_area = 0.0;
		std::size_t j = ring.size() - 1;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			twice_area +=
					(ring[j].first + ring[i].first) * (ring[j].second - ring[i].second);
			j = i;
		}
		result += std::abs(twice_area) * 0.5;
	}
	return result;
}
} // namespace

std::optional<GeofabrikExtract> select_geofabrik_extract(const nlohmann::json &index,
		const geographic::LLBBox &selection, std::string *error)
{
	const auto fail = [error](const char *message) -> std::optional<GeofabrikExtract> {
		if (error)
			*error = message;
		return std::nullopt;
	};
	const auto features = index.find("features");
	if (features == index.end() || !features->is_array())
		return fail("bad Geofabrik index: missing features array");

	constexpr int samples_per_axis = 5;
	std::array<Point, samples_per_axis * samples_per_axis> samples{};
	for (int row = 0; row < samples_per_axis; ++row) {
		const double lat =
				selection.min().lat() + (selection.max().lat() - selection.min().lat()) *
												row / (samples_per_axis - 1);
		for (int col = 0; col < samples_per_axis; ++col) {
			const double lon = selection.min().lng() +
							   (selection.max().lng() - selection.min().lng()) * col /
									   (samples_per_axis - 1);
			samples[row * samples_per_axis + col] = {lon, lat};
		}
	}

	std::optional<GeofabrikExtract> best;
	for (const auto &feature : *features) {
		if (!feature.is_object())
			continue;
		const auto &properties = feature.value("properties", nlohmann::json::object());
		const auto urls = properties.find("urls");
		if (urls == properties.end() || !urls->is_object() || !urls->contains("pbf") ||
				!(*urls)["pbf"].is_string())
			continue;
		const auto rings = geometry_rings(feature.value("geometry", nlohmann::json{}));
		if (rings.empty() || !std::all_of(samples.begin(), samples.end(),
									 [&rings](const Point &point) {
										 return inside_rings(
												 point.first, point.second, rings);
									 }))
			continue;
		const double polygon_area = area(rings);
		if (!std::isfinite(polygon_area) || polygon_area <= 0.0)
			continue;
		const std::string url = (*urls)["pbf"].get<std::string>();
		if (url.empty())
			continue;
		const std::string name = properties.value("name", std::string{});
		if (!best || polygon_area < best->polygon_area_degrees_squared)
			best = GeofabrikExtract{name, url, polygon_area};
	}
	if (!best)
		return fail("no single Geofabrik extract holds the selection");
	if (error)
		error->clear();
	return best;
}

std::filesystem::path cache_root()
{
	return cache::base_directory() / "osm-pbf";
}

std::optional<std::filesystem::path> locate_extract(const std::string &spec,
		const std::optional<std::string> &explicit_url,
		const geographic::LLBBox &selection, std::string *error)
{
	if (spec != GEOFABRIK) {
		const std::filesystem::path path(spec);
		if (std::filesystem::is_regular_file(path) && path.extension() == ".pbf") {
			if (error)
				error->clear();
			return path;
		}
		if (error)
			*error = "--osm-pbf: " + path.string() + " is not a .pbf file";
		return std::nullopt;
	}
	std::string url;
	std::optional<std::uint64_t> automatic_limit;
	if (explicit_url) {
		url = *explicit_url;
	} else {
		const auto index_bytes = fetch_index(error);
		if (!index_bytes)
			return std::nullopt;
		const auto index = nlohmann::json::parse(*index_bytes, nullptr, false);
		if (index.is_discarded()) {
			if (error)
				*error = "bad Geofabrik index";
			return std::nullopt;
		}
		const auto extract = select_geofabrik_extract(index, selection, error);
		if (!extract)
			return std::nullopt;
		url = extract->url;
		automatic_limit = AUTO_EXTRACT_MAX_BYTES;
	}
	return download_extract(url, automatic_limit, error);
}
} // namespace arnis::osm_pbf
