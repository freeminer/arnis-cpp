#include "overture.h"
#include "clipping.h"
#include "coordinate_system/transformation.h"
#include "overture/mvt.h"
#include "overture/cache.h"
#include "overture/pmtiles.h"
#include "retrieve_data.h"
#include "net.h"
#include "../../http.h"

#if defined(USE_ARROW) && USE_ARROW
#include <arrow/api.h>
#include <arrow/io/file.h>
#include <arrow/io/memory.h>
#include <parquet/arrow/reader.h>
#include <parquet/metadata.h>
#include <parquet/statistics.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cctype>
#include <curl/curl.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <condition_variable>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <regex>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#if __EMSCRIPTEN__
#undef read
#endif

namespace arnis::overture
{

elevation::CacheClearStats clear_overture_cache()
{
	return cache::clear_overture_cache();
}

namespace
{
constexpr std::uint8_t OVERTURE_TILE_ZOOM = 14;
constexpr std::uint64_t OVERTURE_MAX_TILES = 4096;
constexpr std::size_t OVERTURE_MAX_RELEASE_ATTEMPTS = 4;
constexpr std::size_t OVERTURE_RELEASE_FALLBACK_SLOTS = 2;
constexpr unsigned OVERTURE_RANGE_ATTEMPTS = 3;
constexpr std::size_t OVERTURE_UNHEALTHY_FAILURES = 3;
constexpr const char *OVERTURE_STAC_RELEASE_FALLBACK = "2026-08-19.0";

constexpr unsigned char ascii_lower(unsigned char c)
{
	return c >= 'A' && c <= 'Z' ? static_cast<unsigned char>(c + ('a' - 'A')) : c;
}

bool ascii_iequals(std::string_view a, std::string_view b)
{
	return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
										   [](unsigned char x, unsigned char y) {
											   return ascii_lower(x) == ascii_lower(y);
										   });
}

// Keep the known transient ML false positives out of both transports before
// they can consume an in-bbox footprint budget slot.
bool is_skipped_footprint(std::string_view id)
{
	// Königsplatz, Munich: a temporary stage caught by one imagery pass.
	// Corcovado, Rio: boxes on the summit terrace where OSM maps the buildings.
	static constexpr std::array<std::string_view, 3> skipped{
			"f8c0757e-c059-49e4-9757-7e278751926f",
			"300e8ad6-8834-4704-9f3c-f0f082439da5",
			"e1286a9a-92bb-4d99-9f66-61d383c399f4",
	};
	return std::find(skipped.begin(), skipped.end(), id) != skipped.end();
}

// Keep blocking tile fetches on a small, dedicated process-wide pool. Creating
// and joining a new set of workers for every 64-tile batch needlessly churns
// threads on large areas, and using the generation pool would let slow HTTP
// requests starve world-generation work.
class TileFetchPool
{
	struct Group
	{
		explicit Group(std::size_t count) : remaining(count) {}
		std::atomic_size_t remaining;
		std::mutex mutex;
		std::condition_variable finished;
		std::exception_ptr failure;
	};

	std::vector<std::thread> workers_;
	std::queue<std::function<void()>> tasks_;
	std::mutex mutex_;
	std::condition_variable ready_;
	bool stopping_{false};

	void enqueue(std::function<void()> task)
	{
		{
			std::lock_guard lock(mutex_);
			tasks_.push(std::move(task));
		}
		ready_.notify_one();
	}

public:
	TileFetchPool()
	{
		const auto hardware = std::thread::hardware_concurrency();
		const auto count = std::clamp<std::size_t>(hardware == 0 ? 2 : hardware, 2, 8);
		workers_.reserve(count);
		for (std::size_t i = 0; i < count; ++i) {
			workers_.emplace_back([this] {
				for (;;) {
					std::function<void()> task;
					{
						std::unique_lock lock(mutex_);
						ready_.wait(
								lock, [this] { return stopping_ || !tasks_.empty(); });
						if (stopping_ && tasks_.empty())
							return;
						task = std::move(tasks_.front());
						tasks_.pop();
					}
					task();
				}
			});
		}
	}

	~TileFetchPool()
	{
		{
			std::lock_guard lock(mutex_);
			stopping_ = true;
		}
		ready_.notify_all();
		for (auto &worker : workers_)
			if (worker.joinable())
				worker.join();
	}

	template <typename Function>
	void parallel_for(std::size_t count, Function &&function)
	{
		if (count == 0)
			return;
		auto group = std::make_shared<Group>(count);
		auto work = std::make_shared<std::decay_t<Function>>(
				std::forward<Function>(function));
		for (std::size_t i = 0; i < count; ++i) {
			enqueue([group, work, i] {
				try {
					(*work)(i);
				} catch (...) {
					std::lock_guard lock(group->mutex);
					if (!group->failure)
						group->failure = std::current_exception();
				}
				if (group->remaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
					group->finished.notify_one();
			});
		}
		{
			std::unique_lock lock(group->mutex);
			group->finished.wait(lock, [&] {
				return group->remaining.load(std::memory_order_acquire) == 0;
			});
			if (group->failure)
				std::rethrow_exception(group->failure);
		}
	}
};

TileFetchPool &tile_fetch_pool()
{
	static TileFetchPool pool;
	return pool;
}

bool overture_tiles_cover(const geographic::LLBBox &bbox)
{
	return pmtiles::tile_count(bbox, OVERTURE_TILE_ZOOM) <= OVERTURE_MAX_TILES;
}

size_t release_listing_write(void *data, size_t size, size_t count, void *opaque)
{
	auto *body = static_cast<std::string *>(opaque);
	body->append(static_cast<const char *>(data), size * count);
	return size * count;
}

std::vector<std::string> discover_overture_releases()
{
	constexpr const char *listing_url =
			"https://overturemaps-us-west-2.s3.amazonaws.com/?delimiter=/&prefix=release/";
	CURL *curl = curl_easy_init();
	if (!curl)
		return std::vector<std::string>{};
	std::string body;
	curl_easy_setopt(curl, CURLOPT_URL, listing_url);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, release_listing_write);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
	long status = 0;
	[[maybe_unused]] auto permit = net::request_permit();
	const auto result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300)
		return std::vector<std::string>{};
	std::vector<std::string> releases;
	const std::regex prefix(
			R"(<Prefix>release/([0-9]{4}-[0-9]{2}-[0-9]{2}\.[0-9]+)/</Prefix>)");
	for (std::sregex_iterator it(body.begin(), body.end(), prefix), end; it != end; ++it)
		releases.push_back((*it)[1].str());
	std::sort(releases.begin(), releases.end(), [](const auto &a, const auto &b) {
		const auto pa = a.rfind('.');
		const auto pb = b.rfind('.');
		const auto da = pa == std::string::npos ? a : a.substr(0, pa);
		const auto db = pb == std::string::npos ? b : b.substr(0, pb);
		if (da != db)
			return da > db;
		try {
			return std::stoull(pa == std::string::npos ? "0" : a.substr(pa + 1)) >
				   std::stoull(pb == std::string::npos ? "0" : b.substr(pb + 1));
		} catch (...) {
			return a > b;
		}
	});
	releases.erase(std::unique(releases.begin(), releases.end()), releases.end());
	return releases;
}

// Keep release selection aligned with Rust: try the two newest discovered
// releases, then reserve slots for this installation's last successful
// release and the bundled fallback. A remembered release that is present in
// the listing but older than the newest two must still occupy its fallback
// slot instead of being displaced by another discovered release.
std::vector<std::string> overture_release_candidates(
		std::vector<std::string> discovered, const std::optional<std::string> &last_good)
{
	const auto discovered_slots =
			OVERTURE_MAX_RELEASE_ATTEMPTS - OVERTURE_RELEASE_FALLBACK_SLOTS;
	std::vector<std::string> candidates;
	candidates.reserve(OVERTURE_MAX_RELEASE_ATTEMPTS);
	for (const auto &release : discovered) {
		if (!cache::valid_release(release))
			continue;
		candidates.push_back(release);
		if (candidates.size() == discovered_slots)
			break;
	}
	for (const auto *fallback :
			{last_good ? last_good->c_str() : nullptr, OVERTURE_STAC_RELEASE_FALLBACK}) {
		if (!fallback)
			continue;
		const std::string release(fallback);
		if (cache::valid_release(release) &&
				std::find(candidates.begin(), candidates.end(), release) ==
						candidates.end())
			candidates.push_back(release);
	}
	return candidates;
}

// Keep the closed Overture enums aligned between the Parquet and tile
// transports.  Unknown values are deliberately discarded, matching the Rust
// interning helpers: a newly introduced provider value must not leak through
// as an unrecognised material/shape in the generated world.
std::optional<std::string> intern_roof_shape(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	const auto &v = *value;
	if (v == "gabled" || v == "gable")
		return "gabled";
	if (v == "hipped" || v == "hip")
		return "hipped";
	if (v == "flat")
		return "flat";
	if (v == "pyramidal")
		return "pyramidal";
	if (v == "dome" || v == "onion")
		return "dome";
	if (v == "skillion" || v == "shed")
		return "skillion";
	if (v == "gambrel" || v == "mansard" || v == "round" || v == "half_hipped" ||
			v == "saltbox" || v == "sawtooth" || v == "spherical")
		return v;
	return std::nullopt;
}

std::optional<std::string> intern_roof_material(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	static constexpr std::array<const char *, 14> values = {"concrete", "copper",
			"eternit", "glass", "grass", "gravel", "metal", "plastic", "roof_tiles",
			"slate", "solar_panels", "thatch", "tar_paper", "wood"};
	if (std::find(values.begin(), values.end(), *value) == values.end())
		return std::nullopt;
	return value;
}

std::optional<std::string> intern_roof_orientation(
		const std::optional<std::string> &value)
{
	if (value && (*value == "along" || *value == "across"))
		return value;
	return std::nullopt;
}

std::optional<std::string> intern_facade_material(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	static constexpr std::array<const char *, 11> values = {"brick", "cement_block",
			"clay", "concrete", "glass", "metal", "plaster", "plastic", "stone",
			"timber_framing", "wood"};
	if (std::find(values.begin(), values.end(), *value) == values.end())
		return std::nullopt;
	return value;
}

uint32_t read_u32(
		const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
	if (offset + 4 > bytes.size())
		return 0;
	if (little_endian) {
		return static_cast<uint32_t>(bytes[offset]) |
			   (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
			   (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
			   (static_cast<uint32_t>(bytes[offset + 3]) << 24);
	}
	return (static_cast<uint32_t>(bytes[offset]) << 24) |
		   (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
		   (static_cast<uint32_t>(bytes[offset + 2]) << 8) |
		   static_cast<uint32_t>(bytes[offset + 3]);
}

double read_f64(const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
	std::array<uint8_t, 8> raw{};
	if (offset + 8 > bytes.size())
		return 0.0;
	if (little_endian) {
		std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset), 8, raw.begin());
	} else {
		std::reverse_copy(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
				bytes.begin() + static_cast<std::ptrdiff_t>(offset + 8), raw.begin());
	}
	double out = 0.0;
	std::memcpy(&out, raw.data(), sizeof(out));
	return out;
}

std::optional<std::vector<std::pair<double, double>>> parse_overture_wkb_polygon_impl(
		const std::vector<uint8_t> &wkb)
{
	if (wkb.size() < 13)
		return std::nullopt;

	const uint8_t byte_order = wkb[0];
	if (byte_order > 1)
		return std::nullopt;
	const bool little_endian = byte_order == 1;

	const uint32_t geom_type = read_u32(wkb, 1, little_endian);
	if ((geom_type % 1000) != 3)
		return std::nullopt;

	const uint32_t num_rings = read_u32(wkb, 5, little_endian);
	if (num_rings == 0)
		return std::nullopt;

	std::size_t offset = 9;
	if (offset + 4 > wkb.size())
		return std::nullopt;
	const uint32_t num_points = read_u32(wkb, offset, little_endian);
	offset += 4;

	const uint32_t dimension = geom_type / 1000;
	const bool has_z = dimension == 1 || dimension == 3;
	const bool has_m = dimension == 2 || dimension == 3;
	const std::size_t point_size = 16 + (has_z ? 8 : 0) + (has_m ? 8 : 0);
	if (offset + static_cast<std::size_t>(num_points) * point_size > wkb.size())
		return std::nullopt;

	std::vector<std::pair<double, double>> coords;
	coords.reserve(num_points);
	for (uint32_t i = 0; i < num_points; ++i) {
		const double lng = read_f64(wkb, offset, little_endian);
		const double lat = read_f64(wkb, offset + 8, little_endian);
		coords.emplace_back(lng, lat);
		offset += point_size;
	}
	return coords;
}

uint64_t gers_id_to_u64_impl(const std::string &gers_id)
{
	constexpr uint64_t FNV_OFFSET = 0xcbf29ce484222325ULL;
	constexpr uint64_t FNV_PRIME = 0x100000001b3ULL;

	uint64_t hash = FNV_OFFSET;
	for (unsigned char byte : gers_id) {
		hash ^= static_cast<uint64_t>(byte);
		hash *= FNV_PRIME;
	}
	return hash | OVERTURE_ID_HIGH_BIT;
}

std::string overture_class_to_osm_building_impl(const std::optional<std::string> &subtype,
		const std::optional<std::string> &clazz)
{
	if (clazz) {
		const auto &c = *clazz;
		if (c == "house" || c == "detached")
			return "house";
		if (c == "apartments" || c == "apartment")
			return "apartments";
		if (c == "residential")
			return "residential";
		if (c == "commercial")
			return "commercial";
		if (c == "retail")
			return "retail";
		if (c == "office")
			return "office";
		if (c == "industrial")
			return "industrial";
		if (c == "warehouse")
			return "warehouse";
		if (c == "garage" || c == "garages")
			return "garage";
		if (c == "shed")
			return "shed";
		if (c == "school")
			return "school";
		if (c == "hospital")
			return "hospital";
		if (c == "church" || c == "mosque" || c == "temple" || c == "synagogue")
			return "church";
		if (c == "hotel")
			return "hotel";
		if (c == "farm" || c == "barn")
			return "farm";
	}

	if (subtype) {
		const auto &s = *subtype;
		if (s == "residential")
			return "residential";
		if (s == "commercial")
			return "commercial";
		if (s == "industrial")
			return "industrial";
		if (s == "agricultural")
			return "farm";
		if (s == "civic" || s == "government" || s == "education")
			return "public";
		if (s == "medical")
			return "hospital";
		if (s == "religious")
			return "church";
		if (s == "transportation")
			return "transportation";
		if (s == "outbuilding")
			return "shed";
	}
	return "yes";
}

std::optional<std::string> attribute_string(
		const mvt::Layer &layer, const mvt::Feature &feature, const std::string &name)
{
	const auto attribute = layer.attribute(feature, name);
	if (!attribute)
		return std::nullopt;
	if (const auto value = std::get_if<std::string>(&*attribute))
		return *value;
	return std::nullopt;
}

std::optional<double> attribute_number(
		const mvt::Layer &layer, const mvt::Feature &feature, const std::string &name)
{
	const auto attribute = layer.attribute(feature, name);
	if (!attribute)
		return std::nullopt;
	if (const auto value = std::get_if<double>(&*attribute))
		return *value;
	if (const auto value = std::get_if<std::int64_t>(&*attribute))
		return static_cast<double>(*value);
	if (const auto value = std::get_if<std::uint64_t>(&*attribute))
		return static_cast<double>(*value);
	if (const auto value = std::get_if<std::string>(&*attribute)) {
		constexpr std::string_view whitespace = " \t\n\r\f\v";
		const auto first = value->find_first_not_of(whitespace);
		if (first == std::string::npos)
			return std::nullopt;
		const auto last = value->find_last_not_of(whitespace);
		const char *begin = value->data() + first;
		const char *end = value->data() + last + 1;
		// Rust's Value::as_f64 parses trimmed numeric strings as well as numeric
		// protobuf encodings. std::from_chars does not accept a leading '+', which
		// Rust does, so skip that one character explicitly.
		if (*begin == '+')
			++begin;
		if (begin == end)
			return std::nullopt;
		double parsed = 0.0;
		const auto result =
				std::from_chars(begin, end, parsed, std::chars_format::general);
		if (result.ec == std::errc{} && result.ptr == end)
			return parsed;
	}
	return std::nullopt;
}

bool source_is_osm(const std::optional<std::string> &geometry_source,
		const std::optional<std::string> &sources)
{
	const bool primary_source_is_osm =
			geometry_source && ascii_iequals(*geometry_source, "openstreetmap");
	return primary_source_is_osm ||
		   (sources && sources->find("OpenStreetMap") != std::string::npos);
}

std::optional<OvertureOsmRef> parse_osm_reference(const std::string &sources)
{
	const auto parse_record_id =
			[](const std::string &record_id) -> std::optional<OvertureOsmRef> {
		const auto first = record_id.find_first_not_of(" \t\n\r\f\v");
		if (first == std::string::npos)
			return std::nullopt;
		const auto last = record_id.find_last_not_of(" \t\n\r\f\v");
		const auto value = record_id.substr(first, last - first + 1);
		if (value.size() < 2)
			return std::nullopt;
		const char kind = static_cast<char>(ascii_lower(value.front()));
		if (kind != 'w' && kind != 'r')
			return std::nullopt;
		const auto at = value.find('@', 1);
		const auto digits = value.substr(1, at == std::string::npos ? at : at - 1);
		if (digits.empty() || !std::all_of(digits.begin(), digits.end(),
									  [](unsigned char c) { return std::isdigit(c); }))
			return std::nullopt;
		try {
			return OvertureOsmRef{std::stoull(digits), kind == 'r'};
		} catch (...) {
			return std::nullopt;
		}
	};

	const auto document = nlohmann::json::parse(sources, nullptr, false);
	if (document.is_discarded())
		return std::nullopt;
	const auto is_osm_dataset = [](const std::string &dataset) {
		return ascii_iequals(dataset, "openstreetmap");
	};

	std::function<std::optional<OvertureOsmRef>(const nlohmann::json &)> find_reference;
	find_reference = [&](const nlohmann::json &value) -> std::optional<OvertureOsmRef> {
		if (value.is_array()) {
			for (const auto &entry : value)
				if (auto reference = find_reference(entry))
					return reference;
		} else if (value.is_object()) {
			const auto dataset = value.find("dataset");
			const auto record_id = value.find("record_id");
			if (dataset != value.end() && dataset->is_string() &&
					is_osm_dataset(dataset->get_ref<const std::string &>()) &&
					record_id != value.end() && record_id->is_string()) {
				if (auto reference = parse_record_id(
							record_id->get_ref<const std::string &>()))
					return reference;
			}
			for (auto it = value.begin(); it != value.end(); ++it) {
				if (it.key() == "dataset" || it.key() == "record_id")
					continue;
				if (auto reference = find_reference(it.value()))
					return reference;
			}
		}
		return std::nullopt;
	};
	return find_reference(document);
}

double ring_area2(const std::vector<std::pair<double, double>> &ring)
{
	double sum = 0.0;
	for (std::size_t index = 0; index < ring.size(); ++index) {
		const auto &[x1, y1] = ring[index];
		const auto &[x2, y2] = ring[(index + 1) % ring.size()];
		sum += x1 * y2 - x2 * y1;
	}
	return std::abs(sum);
}

std::string format_one_decimal(double value)
{
	std::ostringstream formatted;
	formatted << std::fixed << std::setprecision(1) << value;
	return formatted.str();
}

bool ring_overlaps_bbox(const std::vector<std::pair<double, double>> &ring,
		const geographic::LLBBox &bbox)
{
	if (ring.empty())
		return false;
	double min_lng = std::numeric_limits<double>::infinity();
	double max_lng = -min_lng, min_lat = min_lng, max_lat = -min_lng;
	for (const auto &[lng, lat] : ring) {
		min_lng = std::min(min_lng, lng);
		max_lng = std::max(max_lng, lng);
		min_lat = std::min(min_lat, lat);
		max_lat = std::max(max_lat, lat);
	}
	return max_lng >= bbox.min().lng() && min_lng <= bbox.max().lng() &&
		   max_lat >= bbox.min().lat() && min_lat <= bbox.max().lat();
}

#if defined(USE_ARROW) && USE_ARROW
std::optional<std::string> string_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		auto scalar = chunk->GetScalar(row);
		if (!scalar.ok())
			return std::nullopt;
		if ((*scalar)->type->id() == arrow::Type::STRING)
			return std::string(
					std::static_pointer_cast<arrow::StringScalar>(*scalar)->view());
		if ((*scalar)->type->id() == arrow::Type::LARGE_STRING)
			return std::string(
					std::static_pointer_cast<arrow::LargeStringScalar>(*scalar)->view());
		return std::nullopt;
	}
	return std::nullopt;
}

// Rust formats the nested `sources` field for its dataset fallback check; keep
// that conversion separate so ordinary string fields retain strict typing.
std::optional<std::string> display_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		auto scalar = chunk->GetScalar(row);
		return scalar.ok() ? std::optional<std::string>((*scalar)->ToString())
						   : std::nullopt;
	}
	return std::nullopt;
}

std::optional<double> number_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		auto scalar = chunk->GetScalar(row);
		if (!scalar.ok())
			return std::nullopt;
		// Rust's Parquet reader accepts only Field::Float / Field::Double for
		// height-like attributes; strings and integral columns are not coerced.
		if ((*scalar)->type->id() == arrow::Type::DOUBLE)
			return std::static_pointer_cast<arrow::DoubleScalar>(*scalar)->value;
		if ((*scalar)->type->id() == arrow::Type::FLOAT)
			return static_cast<double>(
					std::static_pointer_cast<arrow::FloatScalar>(*scalar)->value);
		return std::nullopt;
	}
	return std::nullopt;
}

std::optional<int> integer_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		auto scalar = chunk->GetScalar(row);
		if (!scalar.ok() || (*scalar)->type->id() != arrow::Type::INT32)
			return std::nullopt;
		return std::static_pointer_cast<arrow::Int32Scalar>(*scalar)->value;
	}
	return std::nullopt;
}

std::optional<std::vector<std::uint8_t>> binary_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		if (const auto binary = std::dynamic_pointer_cast<arrow::BinaryArray>(chunk)) {
			const auto value = binary->GetView(row);
			return std::vector<std::uint8_t>(value.begin(), value.end());
		}
		if (const auto binary =
						std::dynamic_pointer_cast<arrow::LargeBinaryArray>(chunk)) {
			const auto value = binary->GetView(row);
			return std::vector<std::uint8_t>(value.begin(), value.end());
		}
		return std::nullopt;
	}
	return std::nullopt;
}
#endif

}

std::size_t overture_building_budget(const geographic::LLBBox &bbox)
{
	return std::clamp(
			static_cast<std::size_t>(bbox.area_km2() * OVERTURE_BUILDINGS_PER_KM2),
			MIN_OVERTURE_BUILDINGS, MAX_OVERTURE_BUILDINGS);
}

std::optional<std::vector<std::pair<double, double>>> parse_overture_wkb_polygon(
		const std::vector<std::uint8_t> &wkb)
{
	return parse_overture_wkb_polygon_impl(wkb);
}
std::uint64_t gers_id_to_u64(const std::string &gers_id)
{
	return gers_id_to_u64_impl(gers_id);
}

std::optional<OvertureOsmRef> parse_overture_osm_reference(const std::string &sources)
{
	return parse_osm_reference(sources);
}

void OvertureHints::insert(OvertureOsmRef key, OsmAttributeHint hint)
{
	if (hint.height_m && (!std::isfinite(*hint.height_m) || *hint.height_m < 2.5 ||
								 *hint.height_m > 500.0))
		hint.height_m.reset();
	if (hint.num_floors && (*hint.num_floors < 2 || *hint.num_floors >= 200))
		hint.num_floors.reset();
	if (!hint.height_m && !hint.num_floors)
		return;
	const auto found = hints_.find(key);
	if (found != hints_.end()) {
		if (!found->second.height_m)
			found->second.height_m = hint.height_m;
		if (!found->second.num_floors)
			found->second.num_floors = hint.num_floors;
	} else if (hints_.size() < limit_) {
		hints_.emplace(key, hint);
	}
}

void OvertureHints::merge(const OvertureHints &other)
{
	for (const auto &[key, hint] : other.hints_)
		insert(key, hint);
}

std::size_t OvertureHints::apply(std::vector<ProcessedElement> &elements) const
{
	std::size_t enriched = 0;
	for (auto &element : elements) {
		if (element.is_node())
			continue;
		const auto hint = hints_.find({element.id(), element.is_relation()});
		if (hint == hints_.end())
			continue;
		tags_t *tags = nullptr;
		if (element.is_way())
			tags = &std::get<ProcessedWay>(static_cast<variant_t &>(element)).tags;
		else
			tags = &std::get<ProcessedRelation>(static_cast<variant_t &>(element)).tags;
		if (!(tags->contains("building") || tags->contains("building:part")) ||
				tags->contains("height") || tags->contains("building:levels"))
			continue;
		if (hint->second.num_floors)
			(*tags)["building:levels"] = std::to_string(*hint->second.num_floors);
		if (hint->second.height_m) {
			(*tags)["height"] = format_one_decimal(*hint->second.height_m);
		}
		(*tags)["arnis:height_source"] = "overture_maps";
		++enriched;
	}
	return enriched;
}

std::size_t enrich_osm_buildings_from_overture(
		const std::vector<OvertureBuilding> &buildings,
		std::vector<ProcessedElement> &elements)
{
	OvertureHints hints;
	for (const auto &building : buildings)
		if (building.is_osm_sourced && building.osm_ref)
			hints.insert(*building.osm_ref,
					OsmAttributeHint{building.height, building.num_floors});
	return hints.apply(elements);
}

std::size_t OvertureData::apply_hints(std::vector<ProcessedElement> &osm_elements) const
{
	return hints.apply(osm_elements);
}

std::pair<std::size_t, std::size_t> OvertureData::merge_into(
		std::vector<ProcessedElement> &osm_elements) const
{
	const auto enriched = apply_hints(osm_elements);
	auto unique = deduplicate_against_osm(elements, osm_elements);
	const auto added = unique.size();
	osm_elements.insert(osm_elements.end(), std::make_move_iterator(unique.begin()),
			std::make_move_iterator(unique.end()));
	return {enriched, added};
}
std::string overture_class_to_osm_building(const std::optional<std::string> &subtype,
		const std::optional<std::string> &clazz)
{
	return overture_class_to_osm_building_impl(subtype, clazz);
}

std::vector<OvertureBuilding> decode_overture_building_tile(
		const std::vector<std::uint8_t> &tile, std::uint32_t tile_x, std::uint32_t tile_y,
		std::uint8_t zoom, std::size_t maximum)
{
	auto decoded = try_decode_overture_building_tile(tile, tile_x, tile_y, zoom, maximum);
	return decoded ? std::move(*decoded) : std::vector<OvertureBuilding>{};
}

std::optional<std::vector<OvertureBuilding>> try_decode_overture_building_tile(
		const std::vector<std::uint8_t> &tile, std::uint32_t tile_x, std::uint32_t tile_y,
		std::uint8_t zoom, std::size_t maximum, bool include_osm_sourced)
{
	std::vector<OvertureBuilding> buildings;
	if (maximum == 0)
		return buildings;
	const auto layers = mvt::decode(tile);
	if (!layers)
		return std::nullopt;
	for (const auto &layer : *layers) {
		if (layer.name != "building" || layer.extent == 0)
			continue;
		for (const auto &feature : layer.features) {
			if (buildings.size() == maximum)
				return buildings;
			if (feature.geom_type != mvt::GEOM_POLYGON)
				continue;
			const auto id = attribute_string(layer, feature, "id");
			if (!id)
				continue;
			// Keep the tile transport's pre-budget filtering identical to the
			// Rust PMTiles path: known transient ML false positives must not
			// consume a slot that could be used by a real footprint.
			if (is_skipped_footprint(*id))
				continue;
			const mvt::Ring *ring = nullptr;
			for (const auto &candidate : feature.rings)
				if (candidate.exterior() && candidate.points.size() >= 3 &&
						(!ring || candidate.area2_abs() >= ring->area2_abs()))
					ring = &candidate;
			if (!ring)
				continue;
			OvertureBuilding building;
			building.id = *id;
			const auto extent = double(layer.extent);
			building.exterior_ring.reserve(ring->points.size());
			for (const auto &[x, y] : ring->points)
				building.exterior_ring.emplace_back(
						pmtiles::tile_x_to_longitude(zoom, tile_x, double(x) / extent),
						pmtiles::tile_y_to_latitude(zoom, tile_y, double(y) / extent));
			const auto sources = attribute_string(layer, feature, "sources");
			building.is_osm_sourced = source_is_osm(
					attribute_string(layer, feature, "@geometry_source"), sources);
			// Rust excludes OSM-sourced duplicate footprints before applying the
			// building budget, so they cannot crowd out Overture-only geometry.
			if (building.is_osm_sourced && !include_osm_sourced)
				continue;
			if (sources)
				building.osm_ref = parse_osm_reference(*sources);
			building.height = attribute_number(layer, feature, "height");
			building.min_height = attribute_number(layer, feature, "min_height");
			if (const auto floors = attribute_number(layer, feature, "num_floors");
					floors && *floors >= std::numeric_limits<int>::min() &&
					*floors <= std::numeric_limits<int>::max())
				building.num_floors = static_cast<int>(*floors);
			building.subtype = attribute_string(layer, feature, "subtype");
			building.clazz = attribute_string(layer, feature, "class");
			building.roof_shape =
					intern_roof_shape(attribute_string(layer, feature, "roof_shape"));
			building.roof_material = intern_roof_material(
					attribute_string(layer, feature, "roof_material"));
			building.roof_orientation = intern_roof_orientation(
					attribute_string(layer, feature, "roof_orientation"));
			building.facade_color = attribute_string(layer, feature, "facade_color");
			building.roof_color = attribute_string(layer, feature, "roof_color");
			building.roof_height = attribute_number(layer, feature, "roof_height");
			building.facade_material = intern_facade_material(
					attribute_string(layer, feature, "facade_material"));
			buildings.push_back(std::move(building));
		}
	}
	return buildings;
}

static BuildingTransportSource pmtiles_building_transport_source_impl(
		pmtiles::Header header, std::vector<std::uint8_t> root_directory,
		pmtiles::RangeReader read_range, std::uint8_t zoom, bool debug,
		bool include_osm_sourced, std::size_t footprint_budget,
		std::size_t osm_hint_budget, bool report_gaps)
{
	return [header, root_directory = std::move(root_directory),
				   read_range = std::move(read_range), zoom, debug, include_osm_sourced,
				   footprint_budget, osm_hint_budget,
				   report_gaps](const geographic::LLBBox &bbox,
				   std::size_t maximum) -> std::optional<OvertureCollection> {
		std::vector<OvertureBuilding> buildings;
		OvertureHints hints(osm_hint_budget);
		std::unordered_map<std::string, std::size_t> by_id;
		std::size_t footprint_count = 0;
		if ((!maximum && !footprint_budget && !osm_hint_budget) || !read_range ||
				zoom < header.min_zoom || zoom > header.max_zoom)
			return std::nullopt;
		const auto [min_x, min_y] =
				pmtiles::lonlat_to_tile(bbox.min().lng(), bbox.max().lat(), zoom);
		const auto [max_x, max_y] =
				pmtiles::lonlat_to_tile(bbox.max().lng(), bbox.min().lat(), zoom);
		constexpr std::uint64_t max_tiles = OVERTURE_MAX_TILES;
		const auto width = std::uint64_t(max_x) - min_x + 1;
		const auto height = std::uint64_t(max_y) - min_y + 1;
		if (width > max_tiles || height > max_tiles || width * height > max_tiles)
			// Rust returns an error here so Auto can choose GeoParquet; an empty
			// success would incorrectly bless the tile source and drop the area.
			return std::nullopt;
		struct TileResult
		{
			pmtiles::TileReadStatus status = pmtiles::TileReadStatus::Error;
			std::vector<OvertureBuilding> buildings;
		};
		struct LocatedTile
		{
			std::uint32_t x, y;
			pmtiles::TileLocation location;
		};
		std::vector<std::pair<std::uint32_t, std::uint32_t>> tiles;
		tiles.reserve(static_cast<std::size_t>(width * height));
		for (std::uint32_t x = min_x; x <= max_x; ++x)
			for (std::uint32_t y = min_y; y <= max_y; ++y)
				tiles.emplace_back(x, y);
		std::vector<LocatedTile> located;
		located.reserve(tiles.size());
		pmtiles::LeafDirectoryCache leaf_cache;
		std::size_t missing_tiles = 0;
		std::size_t locate_failures = 0;
		for (const auto &[x, y] : tiles) {
			const auto result = pmtiles::locate_tile_checked(
					header, root_directory, zoom, x, y, read_range, &leaf_cache);
			if (result.status == pmtiles::TileReadStatus::Missing) {
				++missing_tiles;
			} else if (result.status == pmtiles::TileReadStatus::Error) {
				++locate_failures;
			} else {
				located.push_back({x, y, result.location});
			}
		}
		// Rust aborts the tile attempt when any directory lookup fails. Missing
		// tiles are valid empty coverage, but a failed range read is not.
		if (locate_failures > 0)
			return std::nullopt;
		if (debug)
			std::cerr << "Overture tiles: " << tiles.size() << " cover the bbox, "
					  << located.size() << " hold data, " << missing_tiles
					  << " are empty.\n";
		constexpr std::size_t tile_batch_size = 64;
		std::size_t lost_tiles = locate_failures;
		std::size_t attempted_tiles = 0;
		bool capped = false;
		for (std::size_t batch_start = 0; batch_start < located.size();
				batch_start += tile_batch_size) {
			const auto batch_count =
					std::min(tile_batch_size, located.size() - batch_start);
			attempted_tiles += batch_count;
			std::vector<TileResult> results(batch_count);
			tile_fetch_pool().parallel_for(batch_count, [&](std::size_t index) {
				const auto &located_tile = located[batch_start + index];
				auto &result = results[index];
				auto tile = pmtiles::read_tile_payload_checked(
						header, located_tile.location, read_range);
				result.status = tile.status;
				if (tile.status == pmtiles::TileReadStatus::Found &&
						!tile.bytes.empty()) {
					// The Rust reader applies the bbox test before its global building
					// budget. A per-tile decode cap here would let buffered polygons
					// outside the requested area hide later in-bounds buildings, so
					// keep each bounded batch complete and cap only after bbox filtering
					// and cross-tile deduplication below.
					auto decoded = try_decode_overture_building_tile(tile.bytes,
							located_tile.x, located_tile.y, zoom,
							std::numeric_limits<std::size_t>::max(),
							include_osm_sourced || footprint_budget > 0 ||
									osm_hint_budget > 0);
					if (decoded)
						result.buildings = std::move(*decoded);
					else
						result.status = pmtiles::TileReadStatus::Error;
				}
			});

			// Merge in tile order, independent of worker completion order. Keep
			// the largest buffered footprint for each GERS id; tile-edge copies
			// are clipped, while the full copy preserves more geometry.
			for (auto &result : results) {
				if (result.status != pmtiles::TileReadStatus::Found) {
					++lost_tiles;
					continue;
				}
				for (auto &building : result.buildings) {
					if (is_skipped_footprint(building.id))
						continue;
					if (!ring_overlaps_bbox(building.exterior_ring, bbox))
						continue;
					if ((footprint_budget > 0 || osm_hint_budget > 0) &&
							building.is_osm_sourced) {
						const bool usable_height = building.height &&
												   *building.height >= 2.5 &&
												   *building.height <= 500.0;
						const bool usable_floors = building.num_floors &&
												   *building.num_floors >= 2 &&
												   *building.num_floors < 200;
						if (osm_hint_budget > 0 && building.osm_ref &&
								(usable_height || usable_floors))
							hints.insert(*building.osm_ref,
									OsmAttributeHint{usable_height ? building.height
																   : std::nullopt,
											usable_floors ? building.num_floors
														  : std::nullopt});
						// Rust returns both the hint and footprint when requested. The
						// hint-only mode drops the geometry after recording its metadata.
						if (!include_osm_sourced)
							continue;
					}
					const auto previous = by_id.find(building.id);
					if (previous == by_id.end()) {
						by_id.emplace(building.id, buildings.size());
						buildings.push_back(std::move(building));
						++footprint_count;
					} else if (ring_area2(building.exterior_ring) >
							   ring_area2(buildings[previous->second].exterior_ring)) {
						buildings[previous->second] = std::move(building);
					}
				}
			}
			// Check the cap between bounded rounds, after every tile in this
			// round had a chance to replace a clipped duplicate with its full ring.
			const auto footprint_limit =
					footprint_budget > 0 ? footprint_budget : maximum;
			if (footprint_count >= footprint_limit) {
				capped = true;
				break;
			}
		}
		// Returning empty success here would prevent Auto from trying the
		// GeoParquet fallback, even though every tile containing data failed.
		if (attempted_tiles > 0 && lost_tiles == attempted_tiles)
			return std::nullopt;
		if (report_gaps && lost_tiles > 0)
			std::cerr << "Warning: Overture tile data incomplete: " << lost_tiles
					  << " tile read(s) failed.\n";
		if (report_gaps && capped)
			std::cerr
					<< "Warning: Overture Maps reached the building limit ("
					<< (footprint_budget > 0 ? footprint_budget : maximum)
					<< "); footprints beyond it were dropped, which leaves whole districts "
					   "without them. Use a smaller area for full coverage.\n";
		std::sort(buildings.begin(), buildings.end(),
				[](const OvertureBuilding &a, const OvertureBuilding &b) {
					return a.id < b.id;
				});
		const auto footprint_limit = footprint_budget > 0 ? footprint_budget : maximum;
		if (buildings.size() > footprint_limit)
			buildings.resize(footprint_limit);
		return OvertureCollection{std::move(buildings), std::move(hints)};
	};
}

BuildingTransportSource pmtiles_building_transport_source(pmtiles::Header header,
		std::vector<std::uint8_t> root_directory, pmtiles::RangeReader read_range,
		std::uint8_t zoom, bool debug, bool include_osm_sourced,
		std::size_t osm_hint_budget)
{
	return pmtiles_building_transport_source_impl(header, std::move(root_directory),
			std::move(read_range), zoom, debug, include_osm_sourced, 0, osm_hint_budget,
			false);
}

BuildingSource pmtiles_building_source(pmtiles::Header header,
		std::vector<std::uint8_t> root_directory, pmtiles::RangeReader read_range,
		std::uint8_t zoom, bool debug, bool include_osm_sourced)
{
	auto transport = pmtiles_building_transport_source(header, std::move(root_directory),
			std::move(read_range), zoom, debug, include_osm_sourced);
	return [transport = std::move(transport)](
				   const geographic::LLBBox &bbox, std::size_t maximum) {
		auto collection = transport(bbox, maximum);
		return collection.value_or(OvertureCollection{});
	};
}

static BuildingTransportSource http_pmtiles_building_transport_source_impl(
		std::string archive_url, std::uint8_t zoom, std::filesystem::path cache_directory,
		bool debug, bool include_osm_sourced, std::size_t footprint_budget,
		std::size_t osm_hint_budget, bool report_gaps)
{
	struct State
	{
		std::mutex mutex;
		std::optional<pmtiles::Archive> archive;
	};
	auto state = std::make_shared<State>();
	return [url = std::move(archive_url), state, zoom,
				   cache_directory = std::move(cache_directory), debug,
				   include_osm_sourced, footprint_budget, osm_hint_budget,
				   report_gaps](const geographic::LLBBox &bbox,
				   std::size_t maximum) -> std::optional<OvertureCollection> {
		const pmtiles::RangeReader range = [&url, &cache_directory](std::uint64_t offset,
												   std::uint64_t length)
				-> std::optional<std::vector<std::uint8_t>> {
			const auto cache_file =
					cache_directory.empty()
							? std::filesystem::path{}
							: cache_directory / "ranges" /
									  (std::to_string(offset) + "_" +
											  std::to_string(length) + ".bin");
			if (!cache_file.empty()) {
				if (const auto cached = cache::read(cache_file);
						cached && cached->size() == length)
					return cached;
			}
			std::string bytes;
			for (unsigned attempt = 0; attempt < 3; ++attempt) {
				if (attempt > 0)
					std::this_thread::sleep_for(
							std::chrono::milliseconds(500U << (attempt - 1)));
				[[maybe_unused]] auto permit = net::request_permit();
				bytes = http_get_range(url, offset, length);
				if (bytes.size() == length)
					break;
			}
			if (bytes.size() != length)
				return std::nullopt;
			auto data = std::vector<std::uint8_t>(bytes.begin(), bytes.end());
			if (!cache_file.empty())
				cache::write_atomic(cache_file, data);
			return data;
		};
		pmtiles::Archive archive;
		{
			std::lock_guard lock(state->mutex);
			if (!state->archive)
				state->archive = pmtiles::open_archive(range);
			if (!state->archive)
				return std::nullopt;
			archive = *state->archive;
		}
		if (archive.root_directory.empty())
			return std::nullopt;
		return pmtiles_building_transport_source_impl(archive.header,
				std::move(archive.root_directory), range, zoom, debug,
				include_osm_sourced, footprint_budget, osm_hint_budget,
				report_gaps)(bbox, maximum);
	};
}

BuildingTransportSource http_pmtiles_building_transport_source(std::string archive_url,
		std::uint8_t zoom, std::filesystem::path cache_directory, bool debug,
		bool include_osm_sourced, std::size_t osm_hint_budget)
{
	return http_pmtiles_building_transport_source_impl(std::move(archive_url), zoom,
			std::move(cache_directory), debug, include_osm_sourced, 0, osm_hint_budget,
			false);
}

BuildingSource http_pmtiles_building_source(std::string archive_url, std::uint8_t zoom,
		std::filesystem::path cache_directory, bool debug, bool include_osm_sourced)
{
	auto transport = http_pmtiles_building_transport_source(std::move(archive_url), zoom,
			std::move(cache_directory), debug, include_osm_sourced);
	return [transport = std::move(transport)](
				   const geographic::LLBBox &bbox, std::size_t maximum) {
		auto collection = transport(bbox, maximum);
		return collection.value_or(OvertureCollection{});
	};
}

#if defined(USE_ARROW) && USE_ARROW
namespace
{
constexpr const char *OVERTURE_STAC_ROOT = "https://stac.overturemaps.org";

std::size_t binary_write(void *data, std::size_t size, std::size_t count, void *opaque)
{
	auto *body = static_cast<std::string *>(opaque);
	body->append(static_cast<const char *>(data), size * count);
	return size * count;
}

std::optional<std::string> http_get_bytes(const std::string &url)
{
	CURL *curl = curl_easy_init();
	if (!curl)
		return std::nullopt;
	std::string body;
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, binary_write);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 20L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 90L);
	long status = 0;
	[[maybe_unused]] auto permit = net::request_permit();
	const auto result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300)
		return std::nullopt;
	return body;
}

std::optional<std::uint64_t> parquet_file_size(
		const std::string &url, const std::filesystem::path &cache_dir)
{
	if (const auto cached = cache::read(cache_dir / "size")) {
		const std::string value(cached->begin(), cached->end());
		std::uint64_t size = 0;
		const auto [end, error] =
				std::from_chars(value.data(), value.data() + value.size(), size);
		if (error == std::errc{} && end == value.data() + value.size() && size >= 12)
			return size;
	}
	CURL *curl = curl_easy_init();
	if (!curl)
		return std::nullopt;
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_NOBODY, 1L);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 20L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 45L);
	long status = 0;
	[[maybe_unused]] auto permit = net::request_permit();
	const auto result = curl_easy_perform(curl);
	curl_off_t length = -1;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_getinfo(curl, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &length);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300 || length < 12)
		return std::nullopt;
	const auto size = static_cast<std::uint64_t>(length);
	const auto encoded = std::to_string(size);
	cache::write_atomic(cache_dir / "size",
			std::vector<std::uint8_t>(encoded.begin(), encoded.end()));
	return size;
}

std::optional<std::vector<std::uint8_t>> fetch_parquet_range(const std::string &url,
		std::uint64_t start, std::uint64_t length, const std::filesystem::path &cache_dir,
		unsigned attempts = OVERTURE_RANGE_ATTEMPTS)
{
	if (length == 0 || length > std::numeric_limits<std::size_t>::max() ||
			start > std::numeric_limits<std::uint64_t>::max() - (length - 1))
		return std::nullopt;
	const auto cache_file =
			cache_dir / (std::to_string(start) + "_" + std::to_string(length) + ".bin");
	if (const auto bytes = cache::read(cache_file); bytes && bytes->size() == length)
		return bytes;
	std::string body;
	for (unsigned attempt = 0; attempt < attempts; ++attempt) {
		if (attempt)
			std::this_thread::sleep_for(std::chrono::milliseconds(500U << (attempt - 1)));
		CURL *curl = curl_easy_init();
		if (!curl)
			continue;
		body.clear();
		const auto range =
				std::to_string(start) + "-" + std::to_string(start + length - 1);
		curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, binary_write);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 20L);
		curl_easy_setopt(curl, CURLOPT_TIMEOUT, 90L);
		long status = 0;
		[[maybe_unused]] auto permit = net::request_permit();
		const auto result = curl_easy_perform(curl);
		curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
		curl_easy_cleanup(curl);
		if (result == CURLE_OK && status == 206 && body.size() == length)
			break;
		body.clear();
	}
	if (body.size() != length)
		return std::nullopt;
	std::vector<std::uint8_t> bytes(body.begin(), body.end());
	cache::write_atomic(cache_file, bytes);
	return bytes;
}

bool write_sparse_range(const std::filesystem::path &path, std::uint64_t offset,
		const std::vector<std::uint8_t> &bytes)
{
	if (offset > static_cast<std::uint64_t>(std::numeric_limits<std::streamoff>::max()))
		return false;
	std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
	if (!file)
		return false;
	file.seekp(static_cast<std::streamoff>(offset));
	file.write(reinterpret_cast<const char *>(bytes.data()),
			static_cast<std::streamsize>(bytes.size()));
	return static_cast<bool>(file);
}

struct SparseFileCleanup
{
	std::filesystem::path path;
	~SparseFileCleanup()
	{
		std::error_code error;
		std::filesystem::remove(path, error);
	}
};

std::shared_ptr<arrow::Scalar> struct_value(
		const std::shared_ptr<arrow::Scalar> &value, const std::string &name)
{
	if (!value || !value->is_valid || value->type->id() != arrow::Type::STRUCT)
		return std::shared_ptr<arrow::Scalar>{};
	const auto structure = std::static_pointer_cast<arrow::StructScalar>(value);
	const auto type = std::static_pointer_cast<arrow::StructType>(value->type);
	const auto index = type->GetFieldIndex(name);
	return index >= 0 && static_cast<std::size_t>(index) < structure->value.size()
				   ? structure->value[index]
				   : std::shared_ptr<arrow::Scalar>{};
}

struct OvertureSourceInfo
{
	bool is_osm = false;
	std::optional<OvertureOsmRef> osm_ref;
};

std::optional<std::string> scalar_text(const std::shared_ptr<arrow::Scalar> &value);

void scan_overture_sources(
		const std::shared_ptr<arrow::Scalar> &value, OvertureSourceInfo &out)
{
	if (!value || !value->is_valid)
		return;
	if (value->type->id() == arrow::Type::STRING ||
			value->type->id() == arrow::Type::LARGE_STRING) {
		const auto text = scalar_text(value);
		if (text && text->find("OpenStreetMap") != std::string::npos)
			out.is_osm = true;
		return;
	}
	if (value->type->id() == arrow::Type::STRUCT) {
		const auto structure = std::static_pointer_cast<arrow::StructScalar>(value);
		const auto type = std::static_pointer_cast<arrow::StructType>(value->type);
		const auto dataset_index = type->GetFieldIndex("dataset");
		const auto record_index = type->GetFieldIndex("record_id");
		if (dataset_index >= 0 && record_index >= 0 &&
				static_cast<std::size_t>(dataset_index) < structure->value.size() &&
				static_cast<std::size_t>(record_index) < structure->value.size()) {
			const auto dataset = scalar_text(structure->value[dataset_index]);
			const auto record_id = scalar_text(structure->value[record_index]);
			if (dataset && ascii_iequals(*dataset, "OpenStreetMap")) {
				out.is_osm = true;
				if (!out.osm_ref && record_id) {
					const auto encoded = nlohmann::json{{"dataset", *dataset},
							{"record_id",
									*record_id}}.dump();
					out.osm_ref = parse_overture_osm_reference(encoded);
				}
			}
		}
		for (const auto &child : structure->value)
			scan_overture_sources(child, out);
		return;
	}
	if (value->type->id() == arrow::Type::LIST ||
			value->type->id() == arrow::Type::LARGE_LIST ||
			value->type->id() == arrow::Type::LIST_VIEW ||
			value->type->id() == arrow::Type::LARGE_LIST_VIEW ||
			value->type->id() == arrow::Type::FIXED_SIZE_LIST ||
			value->type->id() == arrow::Type::MAP) {
		const auto list = std::static_pointer_cast<arrow::BaseListScalar>(value);
		for (int64_t index = 0; index < list->value->length(); ++index) {
			const auto child = list->value->GetScalar(index);
			if (child.ok())
				scan_overture_sources(*child, out);
		}
	}
}

std::optional<std::string> scalar_text(const std::shared_ptr<arrow::Scalar> &value)
{
	if (!value || !value->is_valid)
		return std::nullopt;
	if (value->type->id() == arrow::Type::STRING)
		return std::static_pointer_cast<arrow::StringScalar>(value)->value->ToString();
	if (value->type->id() == arrow::Type::LARGE_STRING)
		return std::static_pointer_cast<arrow::LargeStringScalar>(value)
				->value->ToString();
	return std::nullopt;
}

std::optional<double> scalar_number(const std::shared_ptr<arrow::Scalar> &value)
{
	if (!value || !value->is_valid)
		return std::nullopt;
	switch (value->type->id()) {
	case arrow::Type::DOUBLE:
		return std::static_pointer_cast<arrow::DoubleScalar>(value)->value;
	case arrow::Type::FLOAT:
		return std::static_pointer_cast<arrow::FloatScalar>(value)->value;
	case arrow::Type::INT64:
		return static_cast<double>(
				std::static_pointer_cast<arrow::Int64Scalar>(value)->value);
	case arrow::Type::INT32:
		return static_cast<double>(
				std::static_pointer_cast<arrow::Int32Scalar>(value)->value);
	default:
		return std::nullopt;
	}
}

std::shared_ptr<arrow::Scalar> scalar_at(
		const std::shared_ptr<arrow::Table> &table, const char *column, int64_t row)
{
	const auto values = table ? table->GetColumnByName(column) : nullptr;
	if (!values || row < 0 || row >= values->length())
		return std::shared_ptr<arrow::Scalar>{};
	auto result = values->GetScalar(row);
	return result.ok() ? *result : std::shared_ptr<arrow::Scalar>{};
}

std::optional<std::vector<std::string>> stac_partition_urls(
		const std::filesystem::path &root, const std::string &release,
		const geographic::LLBBox &bbox, bool debug)
{
	const auto index_path = root / release / "stac_index.parquet";
	auto index_bytes = cache::read(index_path);
	if (!index_bytes) {
		const auto downloaded = http_get_bytes(
				std::string(OVERTURE_STAC_ROOT) + "/" + release + "/collections.parquet");
		if (!downloaded)
			return std::nullopt;
		index_bytes.emplace(downloaded->begin(), downloaded->end());
		cache::write_atomic(index_path, *index_bytes);
	}
	const auto input = arrow::io::ReadableFile::Open(index_path.string());
	if (!input.ok())
		return std::nullopt;
	auto reader_result = parquet::arrow::OpenFile(*input, arrow::default_memory_pool());
	if (!reader_result.ok())
		return std::nullopt;
	std::shared_ptr<arrow::Table> table;
	if (!(*reader_result)->ReadTable(&table).ok() || !table)
		return std::nullopt;
	auto compact = table->CombineChunks(arrow::default_memory_pool());
	if (!compact.ok())
		return std::nullopt;
	table = *compact;
	std::vector<std::string> urls;
	for (int64_t row = 0; row < table->num_rows(); ++row) {
		if (scalar_text(scalar_at(table, "collection", row)) != "building")
			continue;
		const auto bounds = scalar_at(table, "bbox", row);
		const auto west = scalar_number(struct_value(bounds, "xmin"));
		const auto south = scalar_number(struct_value(bounds, "ymin"));
		const auto east = scalar_number(struct_value(bounds, "xmax"));
		const auto north = scalar_number(struct_value(bounds, "ymax"));
		if (!west || !south || !east || !north || *west > bbox.max().lng() ||
				*east < bbox.min().lng() || *south > bbox.max().lat() ||
				*north < bbox.min().lat())
			continue;
		const auto assets = scalar_at(table, "assets", row);
		auto href = scalar_text(struct_value(struct_value(assets, "azure"), "href"));
		if (!href)
			href = scalar_text(struct_value(struct_value(assets, "aws"), "href"));
		if (href)
			urls.push_back(std::move(*href));
	}
	if (debug)
		std::cerr << "Overture STAC: " << urls.size()
				  << " Parquet partition(s) overlap the requested bounds.\n";
	return urls;
}

std::optional<std::vector<OvertureBuilding>> read_remote_parquet_partition(
		const std::string &url, const std::filesystem::path &range_cache,
		const geographic::LLBBox &bbox, bool debug, std::size_t &failed_row_groups)
{
	failed_row_groups = 0;
	const auto size = parquet_file_size(url, range_cache);
	if (!size || *size < 12)
		return std::nullopt;
	const auto tail = fetch_parquet_range(url, *size - 8, 8, range_cache);
	if (!tail || (*tail)[4] != 'P' || (*tail)[5] != 'A' || (*tail)[6] != 'R' ||
			(*tail)[7] != '1')
		return std::nullopt;
	const std::uint32_t footer_size =
			std::uint32_t((*tail)[0]) | (std::uint32_t((*tail)[1]) << 8) |
			(std::uint32_t((*tail)[2]) << 16) | (std::uint32_t((*tail)[3]) << 24);
	if (footer_size > *size - 8)
		return std::nullopt;
	const auto footer_offset = *size - footer_size - 8;
	const auto footer = fetch_parquet_range(url, footer_offset, footer_size, range_cache);
	const auto magic = fetch_parquet_range(url, 0, 4, range_cache);
	if (!footer || !magic)
		return std::nullopt;
	static std::atomic_uint64_t sparse_sequence{0};
	const auto sparse_path =
			range_cache /
			(".sparse-" + std::to_string(sparse_sequence.fetch_add(1)) + ".parquet");
	const SparseFileCleanup sparse_cleanup{sparse_path};
	std::error_code filesystem_error;
	std::filesystem::create_directories(range_cache, filesystem_error);
	if (filesystem_error)
		return std::nullopt;
	{
		std::ofstream create(sparse_path, std::ios::binary | std::ios::trunc);
		if (!create)
			return std::nullopt;
	}
	std::filesystem::resize_file(sparse_path, *size, filesystem_error);
	if (filesystem_error || !write_sparse_range(sparse_path, 0, *magic) ||
			!write_sparse_range(sparse_path, footer_offset, *footer) ||
			!write_sparse_range(sparse_path, *size - 8, *tail)) {
		std::filesystem::remove(sparse_path, filesystem_error);
		return std::nullopt;
	}
	const auto remove_sparse = [&] {
		std::filesystem::remove(sparse_path, filesystem_error);
	};
	auto input = arrow::io::ReadableFile::Open(sparse_path.string());
	if (!input.ok()) {
		remove_sparse();
		return std::nullopt;
	}
	auto reader_result = parquet::arrow::OpenFile(*input, arrow::default_memory_pool());
	if (!reader_result.ok()) {
		remove_sparse();
		return std::nullopt;
	}
	auto reader = std::move(*reader_result);
	const auto metadata = reader->parquet_reader()->metadata();
	if (!metadata) {
		remove_sparse();
		return std::nullopt;
	}
	std::array<int, 4> bbox_columns{-1, -1, -1, -1};
	const std::array<std::string, 4> bbox_paths{
			"bbox.xmin", "bbox.ymin", "bbox.xmax", "bbox.ymax"};
	for (int column = 0; column < metadata->schema()->num_columns(); ++column) {
		const auto path = metadata->schema()->Column(column)->path()->ToDotString();
		for (std::size_t i = 0; i < bbox_paths.size(); ++i)
			if (path == bbox_paths[i])
				bbox_columns[i] = column;
	}
	std::vector<int> matching_groups;
	for (int group = 0; group < metadata->num_row_groups(); ++group) {
		bool overlaps = true;
		if (std::all_of(bbox_columns.begin(), bbox_columns.end(),
					[](int i) { return i >= 0; })) {
			const auto row_group = metadata->RowGroup(group);
			std::array<std::optional<double>, 4> minimum, maximum;
			for (std::size_t i = 0; i < bbox_columns.size(); ++i) {
				const auto column = row_group->ColumnChunk(bbox_columns[i]);
				const auto stats = column->statistics();
				if (!stats)
					continue;
				std::shared_ptr<arrow::Scalar> low, high;
				if (!parquet::arrow::StatisticsAsScalars(*stats, &low, &high).ok())
					continue;
				minimum[i] = scalar_number(low);
				maximum[i] = scalar_number(high);
			}
			if (minimum[0] && minimum[1] && maximum[2] && maximum[3])
				overlaps = *maximum[2] >= bbox.min().lng() &&
						   *minimum[0] <= bbox.max().lng() &&
						   *maximum[3] >= bbox.min().lat() &&
						   *minimum[1] <= bbox.max().lat();
		}
		if (overlaps)
			matching_groups.push_back(group);
	}
	if (matching_groups.empty()) {
		reader.reset();
		remove_sparse();
		return std::vector<OvertureBuilding>{};
	}
	// Keep only the footer, metadata and matching groups in the sparse file.
	std::unordered_set<int> unavailable_groups;
	std::size_t failed_downloads = 0;
	for (const int group_index : matching_groups) {
		const auto group = metadata->RowGroup(group_index);
		std::uint64_t first = std::numeric_limits<std::uint64_t>::max();
		std::uint64_t last = 0;
		for (int column_index = 0; column_index < group->num_columns(); ++column_index) {
			const auto column = group->ColumnChunk(column_index);
			const auto dictionary = column->dictionary_page_offset();
			const auto start = dictionary >= 0 ? dictionary : column->data_page_offset();
			if (start < 0 || column->total_compressed_size() < 0)
				continue;
			first = std::min(first, static_cast<std::uint64_t>(start));
			last = std::max(last,
					static_cast<std::uint64_t>(start + column->total_compressed_size()));
		}
		if (first >= last || last > *size) {
			unavailable_groups.insert(group_index);
			continue;
		}
		const auto attempts = failed_downloads >= OVERTURE_UNHEALTHY_FAILURES
									  ? 1U
									  : OVERTURE_RANGE_ATTEMPTS;
		const auto bytes =
				fetch_parquet_range(url, first, last - first, range_cache, attempts);
		if (!bytes || !write_sparse_range(sparse_path, first, *bytes)) {
			unavailable_groups.insert(group_index);
			++failed_downloads;
		}
	}
	reader.reset();
	input = arrow::io::ReadableFile::Open(sparse_path.string());
	if (!input.ok()) {
		remove_sparse();
		return std::nullopt;
	}
	reader_result = parquet::arrow::OpenFile(*input, arrow::default_memory_pool());
	if (!reader_result.ok()) {
		remove_sparse();
		return std::nullopt;
	}
	reader = std::move(*reader_result);
	std::vector<OvertureBuilding> buildings;
	for (const int group : matching_groups) {
		if (unavailable_groups.contains(group))
			continue;
		std::shared_ptr<arrow::Table> table;
		if (!reader->ReadRowGroup(group, &table).ok() || !table) {
			unavailable_groups.insert(group);
			continue;
		}
		auto compact = table->CombineChunks(arrow::default_memory_pool());
		if (!compact.ok()) {
			unavailable_groups.insert(group);
			continue;
		}
		table = *compact;
		const auto ids = table->GetColumnByName("id");
		const auto geometries = table->GetColumnByName("geometry");
		if (!ids || !geometries) {
			unavailable_groups.insert(group);
			continue;
		}
		const auto sources = table->GetColumnByName("sources");
		const auto heights = table->GetColumnByName("height");
		const auto min_heights = table->GetColumnByName("min_height");
		const auto floors = table->GetColumnByName("num_floors");
		const auto subtypes = table->GetColumnByName("subtype");
		const auto classes = table->GetColumnByName("class");
		const auto roof_shapes = table->GetColumnByName("roof_shape");
		const auto roof_materials = table->GetColumnByName("roof_material");
		const auto roof_orientations = table->GetColumnByName("roof_orientation");
		const auto facade_colors = table->GetColumnByName("facade_color");
		const auto roof_colors = table->GetColumnByName("roof_color");
		const auto roof_heights = table->GetColumnByName("roof_height");
		const auto facade_materials = table->GetColumnByName("facade_material");
		for (int64_t row = 0; row < table->num_rows(); ++row) {
			auto id_scalar = ids->GetScalar(row);
			auto geometry_scalar = geometries->GetScalar(row);
			if (!id_scalar.ok() || !geometry_scalar.ok())
				continue;
			const auto id = scalar_text(*id_scalar);
			if (!id || is_skipped_footprint(*id))
				continue;
			std::vector<std::uint8_t> wkb;
			if ((*geometry_scalar)->type->id() == arrow::Type::BINARY) {
				const auto binary =
						std::static_pointer_cast<arrow::BinaryScalar>(*geometry_scalar);
				if (binary->value)
					wkb.assign(binary->value->data(),
							binary->value->data() + binary->value->size());
			} else if ((*geometry_scalar)->type->id() == arrow::Type::LARGE_BINARY) {
				const auto binary = std::static_pointer_cast<arrow::LargeBinaryScalar>(
						*geometry_scalar);
				if (binary->value)
					wkb.assign(binary->value->data(),
							binary->value->data() + binary->value->size());
			}
			auto ring = parse_overture_wkb_polygon(wkb);
			if (!ring || ring->size() < 3)
				continue;
			const auto row_bbox = scalar_at(table, "bbox", row);
			const auto xmin = scalar_number(struct_value(row_bbox, "xmin"));
			const auto ymin = scalar_number(struct_value(row_bbox, "ymin"));
			const auto xmax = scalar_number(struct_value(row_bbox, "xmax"));
			const auto ymax = scalar_number(struct_value(row_bbox, "ymax"));
			if (xmin && ymin && xmax && ymax &&
					(*xmax < bbox.min().lng() || *xmin > bbox.max().lng() ||
							*ymax < bbox.min().lat() || *ymin > bbox.max().lat()))
				continue;
			if (!(xmin && ymin && xmax && ymax)) {
				double ring_min_lng = std::numeric_limits<double>::infinity();
				double ring_min_lat = std::numeric_limits<double>::infinity();
				double ring_max_lng = -std::numeric_limits<double>::infinity();
				double ring_max_lat = -std::numeric_limits<double>::infinity();
				for (const auto &[lng, lat] : *ring) {
					ring_min_lng = std::min(ring_min_lng, lng);
					ring_min_lat = std::min(ring_min_lat, lat);
					ring_max_lng = std::max(ring_max_lng, lng);
					ring_max_lat = std::max(ring_max_lat, lat);
				}
				if (ring_max_lng < bbox.min().lng() || ring_min_lng > bbox.max().lng() ||
						ring_max_lat < bbox.min().lat() ||
						ring_min_lat > bbox.max().lat())
					continue;
			}
			const auto text_at = [&](const std::shared_ptr<arrow::ChunkedArray> &column) {
				if (!column)
					return std::optional<std::string>{};
				auto scalar = column->GetScalar(row);
				return scalar.ok() ? scalar_text(*scalar) : std::optional<std::string>{};
			};
			const auto number_at =
					[&](const std::shared_ptr<arrow::ChunkedArray> &column) {
						if (!column)
							return std::optional<double>{};
						auto scalar = column->GetScalar(row);
						return scalar.ok() ? scalar_number(*scalar)
										   : std::optional<double>{};
					};
			OvertureSourceInfo source_info;
			if (sources) {
				auto source_scalar = sources->GetScalar(row);
				if (source_scalar.ok())
					scan_overture_sources(*source_scalar, source_info);
			}
			OvertureBuilding building;
			building.id = *id;
			building.exterior_ring = std::move(*ring);
			building.is_osm_sourced = source_info.is_osm;
			building.osm_ref = source_info.osm_ref;
			building.height = number_at(heights);
			building.min_height = number_at(min_heights);
			if (const auto value = number_at(floors); value && std::isfinite(*value))
				building.num_floors = static_cast<int>(*value);
			building.subtype = text_at(subtypes);
			building.clazz = text_at(classes);
			building.roof_shape = text_at(roof_shapes);
			building.roof_material = text_at(roof_materials);
			building.roof_orientation = text_at(roof_orientations);
			building.facade_color = text_at(facade_colors);
			building.roof_color = text_at(roof_colors);
			building.roof_height = number_at(roof_heights);
			building.facade_material = text_at(facade_materials);
			buildings.push_back(std::move(building));
		}
	}
	failed_row_groups = unavailable_groups.size();
	reader.reset();
	remove_sparse();
	if (debug)
		std::cerr << "Overture Parquet partition yielded " << buildings.size()
				  << " in-bounds building(s).\n";
	return buildings;
}

std::optional<OvertureCollection> collect_overture_parquet_release(
		const std::filesystem::path &root, const std::string &release,
		const geographic::LLBBox &bbox, std::size_t maximum, bool debug)
{
	const auto urls = stac_partition_urls(root, release, bbox, debug);
	if (!urls)
		return std::nullopt;
	OvertureCollection collection;
	std::size_t lost_partitions = 0;
	std::size_t lost_row_groups = 0;
	bool capped = false;
	for (const auto &url : *urls) {
		const auto path_end = url.find_first_of("?#");
		const auto path = url.substr(0, path_end);
		const auto slash = path.find_last_of('/');
		auto filename = slash == std::string::npos ? path : path.substr(slash + 1);
		const bool safe_name =
				!filename.empty() && filename.size() <= 128 && filename != "." &&
				filename != ".." && filename.front() != '.' &&
				std::all_of(filename.begin(), filename.end(), [](unsigned char c) {
					return std::isalnum(c) || c == '-' || c == '_' || c == '.';
				});
		if (!safe_name) {
			std::uint64_t hash = 1469598103934665603ULL;
			for (const unsigned char c : url) {
				hash ^= c;
				hash *= 1099511628211ULL;
			}
			filename = "partition-" + std::to_string(hash);
		}
		std::size_t failed_row_groups = 0;
		auto buildings = read_remote_parquet_partition(url,
				root / release / "parquet" / filename, bbox, debug, failed_row_groups);
		if (!buildings) {
			++lost_partitions;
			if (debug)
				std::cerr << "Warning: failed to process Overture partition " << url
						  << ".\n";
			continue;
		}
		lost_row_groups += failed_row_groups;
		for (auto &building : *buildings) {
			if (building.is_osm_sourced && building.osm_ref &&
					collection.hints.size() < MAX_OVERTURE_OSM_HINTS)
				collection.hints.insert(*building.osm_ref,
						OsmAttributeHint{building.height, building.num_floors});
			if (building.is_osm_sourced || is_skipped_footprint(building.id))
				continue;
			if (collection.buildings.size() >= maximum) {
				capped = true;
				break;
			}
			collection.buildings.push_back(std::move(building));
		}
		if (capped)
			break;
	}
	if (lost_partitions || lost_row_groups)
		std::cerr << "Warning: Overture Maps data incomplete: " << lost_partitions
				  << " partition(s) and " << lost_row_groups
				  << " row group(s) could not be read.\n";
	if (capped)
		std::cerr
				<< "Warning: Overture Maps reached the building limit (" << maximum
				<< "); later footprints may be missing. Use a smaller area for full coverage.\n";
	return collection;
}

std::optional<OvertureData> fetch_overture_parquet_candidates(
		const std::vector<std::string> &candidates, const geographic::LLBBox &bbox,
		const projection::ProjectionSpec &projection_spec, std::size_t maximum,
		bool debug, bool pinned_release)
{
	for (const auto &release : candidates) {
		auto collection = collect_overture_parquet_release(
				cache::cache_root(), release, bbox, maximum, debug);
		if (!collection) {
			if (debug)
				std::cerr << "Overture Parquet unavailable for release " << release
						  << "; trying the next release.\n";
			continue;
		}
		const BuildingSource collection_source =
				[collection = std::move(*collection)](const geographic::LLBBox &,
						std::size_t) mutable { return std::move(collection); };
		auto result = fetch_overture_data_from(
				collection_source, bbox, projection_spec, false, maximum);
		if (!pinned_release) {
			cache::set_last_good_release(cache::cache_root(), release);
			cache::prune_releases_older_than(cache::cache_root(), release);
		}
		return result;
	}
	return std::nullopt;
}
} // namespace

std::vector<OvertureBuilding> read_overture_geoparquet(const std::filesystem::path &path,
		const geographic::LLBBox &bbox, std::size_t maximum)
{
	std::vector<OvertureBuilding> buildings;
	if (maximum == 0 || path.empty())
		return buildings;

	auto input_result = arrow::io::ReadableFile::Open(path.string());
	if (!input_result.ok())
		return buildings;
	auto reader_result =
			parquet::arrow::OpenFile(*input_result, arrow::default_memory_pool());
	if (!reader_result.ok())
		return buildings;
	auto reader = std::move(*reader_result);
	std::shared_ptr<arrow::Table> table;
	if (!reader->ReadTable(&table).ok() || !table)
		return buildings;
	auto compact = table->CombineChunks(arrow::default_memory_pool());
	if (!compact.ok())
		return buildings;
	table = *compact;

	const auto id = table->GetColumnByName("id");
	const auto geometry = table->GetColumnByName("geometry");
	if (!id || !geometry)
		return buildings;
	const auto sources = table->GetColumnByName("sources");
	const auto height = table->GetColumnByName("height");
	const auto min_height = table->GetColumnByName("min_height");
	const auto floors = table->GetColumnByName("num_floors");
	const auto subtype = table->GetColumnByName("subtype");
	const auto clazz = table->GetColumnByName("class");
	const auto roof_shape = table->GetColumnByName("roof_shape");
	const auto roof_material = table->GetColumnByName("roof_material");
	const auto roof_orientation = table->GetColumnByName("roof_orientation");
	const auto facade_color = table->GetColumnByName("facade_color");
	const auto roof_color = table->GetColumnByName("roof_color");
	const auto roof_height = table->GetColumnByName("roof_height");
	const auto facade_material = table->GetColumnByName("facade_material");

	for (std::int64_t row = 0; row < table->num_rows() && buildings.size() < maximum;
			++row) {
		auto value_id = string_at(id, row);
		auto value_geometry = binary_at(geometry, row);
		if (!value_id || !value_geometry)
			continue;
		// Filter before the source budget cap, matching Rust's Parquet path.
		if (is_skipped_footprint(*value_id))
			continue;
		auto ring = parse_overture_wkb_polygon(*value_geometry);
		if (!ring || ring->size() < 3)
			continue;
		if (!ring_overlaps_bbox(*ring, bbox))
			continue;
		OvertureSourceInfo source_info;
		if (sources) {
			auto source_scalar = sources->GetScalar(row);
			if (source_scalar.ok())
				scan_overture_sources(*source_scalar, source_info);
		}
		buildings.push_back({*value_id, std::move(*ring), source_info.is_osm,
				source_info.osm_ref, number_at(height, row), number_at(min_height, row),
				integer_at(floors, row), string_at(subtype, row), string_at(clazz, row),
				intern_roof_shape(string_at(roof_shape, row)),
				intern_roof_material(string_at(roof_material, row)),
				intern_roof_orientation(string_at(roof_orientation, row)),
				string_at(facade_color, row), string_at(roof_color, row),
				intern_facade_material(string_at(facade_material, row)),
				number_at(roof_height, row)});
	}
	return buildings;
}

BuildingSource parquet_building_source(std::filesystem::path path)
{
	return [path = std::move(path)](const geographic::LLBBox &bbox, std::size_t maximum) {
		OvertureCollection collection;
		auto buildings = read_overture_geoparquet(path, bbox, maximum);
		collection.buildings.reserve(buildings.size());
		for (auto &building : buildings) {
			if (building.is_osm_sourced && building.osm_ref)
				collection.hints.insert(*building.osm_ref,
						OsmAttributeHint{building.height, building.num_floors});
			collection.buildings.push_back(std::move(building));
		}
		return collection;
	};
}
#endif

std::optional<ProcessedWay> overture_building_to_way(
		const OvertureBuilding &building, const geographic::LLBBox &bbox, double scale)
{
	return overture_building_to_way(building, bbox,
			projection::ProjectionSpec{
					projection::ProjectionKind::Local, std::nullopt, scale, 0});
}

std::optional<ProcessedWay> overture_building_to_way(const OvertureBuilding &building,
		const geographic::LLBBox &bbox, const projection::ProjectionSpec &projection_spec)
{
	if (building.exterior_ring.size() < 3)
		return std::nullopt;
	const auto [transformer, rectangle] = projection_spec.transformer(bbox);
	const auto base = gers_id_to_u64(building.id);
	ProcessedWay way;
	way.id = base;
	double min_lat = std::numeric_limits<double>::infinity(), max_lat = -min_lat;
	double min_lng = min_lat, max_lng = -min_lat;
	for (std::size_t i = 0; i < building.exterior_ring.size(); ++i) {
		const auto [lng, lat] = building.exterior_ring[i];
		if (!std::isfinite(lat) || !std::isfinite(lng) || lat < -90. || lat > 90. ||
				lng < -180. || lng > 180.)
			continue;
		min_lat = std::min(min_lat, lat);
		max_lat = std::max(max_lat, lat);
		min_lng = std::min(min_lng, lng);
		max_lng = std::max(max_lng, lng);
		const auto point = transformer.transform_point(geographic::LLPoint(lat, lng));
		ProcessedNode node{base + i, tags_t{}, point.x, point.z};
		node.latitude = lat;
		node.longitude = lng;
		way.nodes.push_back(std::move(node));
	}
	if (way.nodes.size() < 3 || max_lng < bbox.min().lng() ||
			min_lng > bbox.max().lng() || max_lat < bbox.min().lat() ||
			min_lat > bbox.max().lat())
		return std::nullopt;
	if (way.nodes.front().x != way.nodes.back().x ||
			way.nodes.front().z != way.nodes.back().z) {
		auto closing_node = way.nodes.front();
		closing_node.id = base + building.exterior_ring.size();
		way.nodes.push_back(std::move(closing_node));
	}
	const auto clip_rectangle = projection_spec.clip_bbox(rectangle);
	XZBBox clipbox(clip_rectangle.min().x, clip_rectangle.min().z, clip_rectangle.max().x,
			clip_rectangle.max().z);
	const auto source_nodes = way.nodes;
	way.nodes = clipping::clip_way_to_bbox_owned(std::move(way.nodes), clipbox);
	// The generic clipper emits projected nodes. Restore their geographic
	// coordinates by interpolating along the source ring so host mapgens can
	// reproject Overture geometry into their own world coordinate frame.
	for (auto &node : way.nodes) {
		double nearest_distance = std::numeric_limits<double>::infinity();
		for (std::size_t i = 1; i < source_nodes.size(); ++i) {
			const auto &a = source_nodes[i - 1];
			const auto &b = source_nodes[i];
			const double dx = static_cast<double>(b.x) - a.x;
			const double dz = static_cast<double>(b.z) - a.z;
			const double length_squared = dx * dx + dz * dz;
			const double t =
					length_squared > 0.0
							? std::clamp(((node.x - a.x) * dx + (node.z - a.z) * dz) /
												 length_squared,
									  0.0, 1.0)
							: 0.0;
			const double nearest_x = a.x + t * dx;
			const double nearest_z = a.z + t * dz;
			const double distance = (node.x - nearest_x) * (node.x - nearest_x) +
									(node.z - nearest_z) * (node.z - nearest_z);
			if (distance < nearest_distance) {
				nearest_distance = distance;
				node.latitude = a.latitude + t * (b.latitude - a.latitude);
				node.longitude = a.longitude + t * (b.longitude - a.longitude);
			}
		}
	}
	// In padded runs, retain only ways that can contribute blocks inside the
	// requested world rectangle. The padding keeps crossing footprints whole
	// for clipping/rendering without admitting detached geometry.
	if (projection_spec.clip_pad > 0 && !way.nodes.empty()) {
		const auto [min_x, max_x] = std::minmax_element(way.nodes.begin(),
				way.nodes.end(), [](const auto &a, const auto &b) { return a.x < b.x; });
		const auto [min_z, max_z] = std::minmax_element(way.nodes.begin(),
				way.nodes.end(), [](const auto &a, const auto &b) { return a.z < b.z; });
		if (max_x->x < rectangle.min().x || min_x->x > rectangle.max().x ||
				max_z->z < rectangle.min().z || min_z->z > rectangle.max().z)
			return std::nullopt;
	}
	if (way.nodes.size() < 4)
		return std::nullopt;
	way.tags["building"] =
			overture_class_to_osm_building(building.subtype, building.clazz);
	const bool useful_floors = building.num_floors && *building.num_floors >= 2;
	if (building.height && *building.height > 0. && *building.height < 1000. &&
			((useful_floors && *building.height > 28.) ||
					(!useful_floors && *building.height >= 10.)))
		way.tags["height"] = format_one_decimal(*building.height);
	if (building.min_height && *building.min_height > 0. && *building.min_height < 1000.)
		way.tags["min_height"] = format_one_decimal(*building.min_height);
	if (building.num_floors && *building.num_floors >= 2 && *building.num_floors < 200)
		way.tags["building:levels"] = std::to_string(*building.num_floors);
	if (building.roof_shape) {
		static const std::unordered_map<std::string, std::string> shapes{
				{"gable", "gabled"}, {"gabled", "gabled"}, {"hip", "hipped"},
				{"hipped", "hipped"}, {"flat", "flat"}, {"pyramidal", "pyramidal"},
				{"dome", "dome"}, {"onion", "dome"}, {"skillion", "skillion"},
				{"shed", "skillion"}, {"gambrel", "gambrel"}, {"mansard", "mansard"},
				{"round", "round"}};
		way.tags["roof:shape"] = shapes.contains(*building.roof_shape)
										 ? shapes.at(*building.roof_shape)
										 : *building.roof_shape;
	}
	if (building.roof_material)
		way.tags["roof:material"] = *building.roof_material;
	if (building.roof_orientation)
		way.tags["roof:orientation"] = *building.roof_orientation;
	if (building.facade_color)
		way.tags["building:colour"] = *building.facade_color;
	if (building.roof_color)
		way.tags["roof:colour"] = *building.roof_color;
	if (building.facade_material) {
		static const std::unordered_set<std::string> materials{"brick", "cement_block",
				"clay", "concrete", "glass", "metal", "plaster", "plastic", "stone",
				"timber_framing", "wood"};
		if (materials.contains(*building.facade_material))
			way.tags["building:material"] = *building.facade_material;
	}
	if (building.roof_height && *building.roof_height >= 0.5 &&
			*building.roof_height < 100.0 &&
			(!building.height || *building.roof_height < *building.height * 0.75))
		way.tags["roof:height"] = format_one_decimal(*building.roof_height);
	way.tags["source"] = "overture_maps";
	return way;
}

std::vector<ProcessedElement> convert_overture_buildings(
		const std::vector<OvertureBuilding> &buildings, const geographic::LLBBox &bbox,
		double scale, bool include_osm_sourced, std::size_t maximum)
{
	return convert_overture_buildings(buildings, bbox,
			projection::ProjectionSpec{
					projection::ProjectionKind::Local, std::nullopt, scale, 0},
			include_osm_sourced, maximum);
}

std::vector<ProcessedElement> convert_overture_buildings(
		const std::vector<OvertureBuilding> &buildings, const geographic::LLBBox &bbox,
		const projection::ProjectionSpec &projection_spec, bool include_osm_sourced,
		std::size_t maximum)
{
	std::vector<ProcessedElement> out;
	out.reserve(std::min(maximum, buildings.size()));
	for (const auto &building : buildings) {
		if (out.size() >= maximum)
			break;
		// Known transient ML false positive (temporary stage/tent at
		// Königsplatz, Munich).  Rust filters this stable GERS id before
		// converting it into a generated building.
		if (is_skipped_footprint(building.id))
			continue;
		if (!include_osm_sourced && building.is_osm_sourced)
			continue;
		if (auto way = overture_building_to_way(building, bbox, projection_spec))
			out.push_back(ProcessedElement::FromWay(*way));
	}
	return out;
}

std::vector<ProcessedElement> fetch_overture_buildings_from(const BuildingSource &source,
		const geographic::LLBBox &bbox, double scale, bool include_osm_sourced,
		std::size_t maximum)
{
	return fetch_overture_data_from(source, bbox, scale, include_osm_sourced, maximum)
			.elements;
}

std::vector<ProcessedElement> fetch_overture_buildings_from(const BuildingSource &source,
		const geographic::LLBBox &bbox, const projection::ProjectionSpec &projection_spec,
		bool include_osm_sourced, std::size_t maximum)
{
	return fetch_overture_data_from(
			source, bbox, projection_spec, include_osm_sourced, maximum)
			.elements;
}

OvertureData fetch_overture_data_from(const BuildingSource &source,
		const geographic::LLBBox &bbox, double scale, bool include_osm_sourced,
		std::size_t maximum)
{
	return fetch_overture_data_from(source, bbox,
			projection::ProjectionSpec{
					projection::ProjectionKind::Local, std::nullopt, scale, 0},
			include_osm_sourced, maximum);
}

OvertureData fetch_overture_data_from(const BuildingSource &source,
		const geographic::LLBBox &bbox, const projection::ProjectionSpec &projection_spec,
		bool include_osm_sourced, std::size_t maximum)
{
	OvertureData out;
	if (!source || projection_spec.scale <= 0)
		return out;
	if (maximum == 0)
		maximum = overture_building_budget(bbox);
	maximum = std::min(maximum, MAX_OVERTURE_BUILDINGS);
	// Leave room for OSM-sourced rows when a provider collects hints alongside
	// footprints; the footprint limit is applied only after conversion.
	const auto raw_limit = std::min(MAX_OVERTURE_BUILDINGS + MAX_OVERTURE_OSM_HINTS,
			maximum + MAX_OVERTURE_OSM_HINTS);
	try {
		auto collection = source(bbox, raw_limit);
		for (const auto &building : collection.buildings) {
			if (building.is_osm_sourced) {
				if (building.osm_ref)
					out.hints.insert(*building.osm_ref,
							OsmAttributeHint{building.height, building.num_floors});
				if (!include_osm_sourced)
					continue;
			}
			if (is_skipped_footprint(building.id))
				continue;
			if (out.elements.size() >= maximum)
				continue;
			if (auto way = overture_building_to_way(building, bbox, projection_spec))
				out.elements.push_back(ProcessedElement::FromWay(*way));
		}
		out.hints.merge(collection.hints);
	} catch (...) {
		// Rust treats STAC/Parquet failures as optional-data failures and
		// continues with OSM.  Keep that property for host callbacks as well.
		return OvertureData{};
	}
	return out;
}

OvertureData fetch_overture_data_from_transports(
		const BuildingTransportSource &tile_source,
		const BuildingTransportSource &parquet_source, const geographic::LLBBox &bbox,
		double scale, OvertureSource source, bool include_osm_sourced,
		std::size_t maximum)
{
	return fetch_overture_data_from_transports(tile_source, parquet_source, bbox,
			projection::ProjectionSpec{
					projection::ProjectionKind::Local, std::nullopt, scale, 0},
			source, include_osm_sourced, maximum);
}

OvertureData fetch_overture_data_from_transports(
		const BuildingTransportSource &tile_source,
		const BuildingTransportSource &parquet_source, const geographic::LLBBox &bbox,
		const projection::ProjectionSpec &projection_spec, OvertureSource source,
		bool include_osm_sourced, std::size_t maximum)
{
	const auto collect =
			[&](const BuildingTransportSource &transport) -> std::optional<OvertureData> {
		if (!transport)
			return std::nullopt;
		auto footprint_limit = maximum;
		if (footprint_limit == 0)
			footprint_limit = overture_building_budget(bbox);
		footprint_limit = std::min(footprint_limit, MAX_OVERTURE_BUILDINGS);
		std::optional<OvertureCollection> collection;
		try {
			collection = transport(bbox, footprint_limit);
		} catch (...) {
			return std::nullopt;
		}
		if (!collection)
			return std::nullopt;
		// Reuse the conversion and hint application path without asking the
		// legacy row-limited adapter for an inflated raw-row budget.
		const BuildingSource fixed_collection =
				[collection = std::move(*collection)](const geographic::LLBBox &,
						std::size_t) mutable { return std::move(collection); };
		return fetch_overture_data_from(fixed_collection, bbox, projection_spec,
				include_osm_sourced, footprint_limit);
	};

	if (source == OvertureSource::Parquet)
		return collect(parquet_source).value_or(OvertureData{});
	const bool tile_area = overture_tiles_cover(bbox);
	if (source == OvertureSource::Tiles) {
		if (!tile_area)
			return OvertureData{};
		return collect(tile_source).value_or(OvertureData{});
	}
	if (tile_area)
		if (auto tiled = collect(tile_source))
			return std::move(*tiled);
	return collect(parquet_source).value_or(OvertureData{});
}

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug)
{
	return fetch_overture_data(
			min_lat, min_lng, max_lat, max_lng, scale, debug, OvertureSource::Auto)
			.elements;
}

OvertureData fetch_overture_data(double min_lat, double min_lng, double max_lat,
		double max_lng, double scale, bool debug, OvertureSource source)
{
	const geographic::LLBBox bbox(min_lat, min_lng, max_lat, max_lng);
	return fetch_overture_data(bbox,
			projection::ProjectionSpec{
					projection::ProjectionKind::Local, std::nullopt, scale, 0},
			debug, source);
}

OvertureData fetch_overture_data(const geographic::LLBBox &bbox,
		const projection::ProjectionSpec &projection_spec, bool debug,
		OvertureSource source)
{
	if (projection_spec.scale <= 0.0 || bbox.min().lat() > bbox.max().lat() ||
			bbox.min().lng() > bbox.max().lng())
		return OvertureData{};
	const bool tiles_cover = overture_tiles_cover(bbox);
	if (source == OvertureSource::Tiles && !tiles_cover) {
		if (debug)
			std::cerr << "Overture Maps: tile source is limited to " << OVERTURE_MAX_TILES
					  << " z14 tiles; use auto or parquet for this area.\n";
		return OvertureData{};
	}
	const char *release_env = std::getenv("ARNIS_OVERTURE_RELEASE");
	const bool pinned_release = release_env && *release_env;
	std::vector<std::string> candidates;
	if (pinned_release) {
		if (cache::valid_release(release_env))
			candidates.emplace_back(release_env);
	} else {
		candidates = overture_release_candidates(discover_overture_releases(),
				cache::last_good_release(cache::cache_root()));
	}
	const auto maximum = std::min(overture_building_budget(bbox), MAX_OVERTURE_BUILDINGS);
	if (source == OvertureSource::Parquet ||
			(source == OvertureSource::Auto && !tiles_cover)) {
#if defined(USE_ARROW) && USE_ARROW
		if (debug && source == OvertureSource::Auto)
			std::cerr << "Overture Maps: area exceeds the vector-tile threshold; "
					  << "using GeoParquet.\n";
		if (auto result = fetch_overture_parquet_candidates(
					candidates, bbox, projection_spec, maximum, debug, pinned_release))
			return std::move(*result);
		if (debug)
			std::cerr << "Overture Maps: no GeoParquet release could be read.\n";
		return OvertureData{};
#else
		if (debug)
			std::cerr
					<< "Overture Maps: GeoParquet requested but Arrow support is disabled.\n";
		return OvertureData{};
#endif
	}
	// For tile-sized requests, try the release's PMTiles archive first. The same
	// candidate list feeds the GeoParquet fallback below, matching Rust's
	// release-aware transport dispatch.
	std::vector<std::string> tile_errors;
	for (const auto &release : candidates) {
		const std::string archive =
				"https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com/tiles/" +
				release + "/buildings.pmtiles";
		if (debug)
			std::cerr << "Overture Maps: trying release " << release << " for " << maximum
					  << " buildings.\n";
		auto transport = http_pmtiles_building_transport_source_impl(archive,
				OVERTURE_TILE_ZOOM, cache::cache_root() / release / "tiles" / "buildings",
				debug, false, maximum, MAX_OVERTURE_OSM_HINTS, true);
		auto collection = transport(bbox, maximum);
		if (!collection) {
			tile_errors.push_back(release + ": tile archive read failed");
			if (debug)
				std::cerr << "Overture tiles unavailable for release " << release
						  << "; trying the next source.\n";
			continue;
		}
		// A successfully read empty collection is still a valid release. Only
		// transport errors advance to another release or GeoParquet fallback.
		const BuildingSource collection_source =
				[collection = std::move(*collection)](const geographic::LLBBox &,
						std::size_t) mutable { return std::move(collection); };
		auto result = fetch_overture_data_from(
				collection_source, bbox, projection_spec, false, maximum);
		if (!pinned_release) {
			cache::set_last_good_release(cache::cache_root(), release);
			cache::prune_releases_older_than(cache::cache_root(), release);
		}
		return result;
	}
	if (source == OvertureSource::Auto) {
#if defined(USE_ARROW) && USE_ARROW
		std::cerr << "Warning: Overture vector tiles unavailable ("
				  << (tile_errors.empty() ? "no usable release" : tile_errors.front())
				  << "); falling back to the GeoParquet source.\n";
		if (auto result = fetch_overture_parquet_candidates(
					candidates, bbox, projection_spec, maximum, debug, pinned_release))
			return std::move(*result);
		std::cerr << "Warning: Overture GeoParquet releases were unavailable.\n";
#else
		std::cerr << "Warning: Overture vector tiles unavailable and Arrow/Parquet "
				  << "fallback is disabled.\n";
#endif
	}
	return OvertureData{};
}

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug, OvertureSource source)
{
	return fetch_overture_data(min_lat, min_lng, max_lat, max_lng, scale, debug, source)
			.elements;
}

std::vector<ProcessedElement> deduplicate_against_osm(
		std::vector<ProcessedElement> overture_elements,
		const std::vector<ProcessedElement> &osm_elements)
{
	struct BBox
	{
		int min_x;
		int min_z;
		int max_x;
		int max_z;
	};

	std::vector<BBox> osm_buildings;
	for (const auto &element : osm_elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (!(way.tags.contains("building") || way.tags.contains("building:part")) ||
				way.nodes.size() < 3)
			continue;
		BBox b{way.nodes.front().x, way.nodes.front().z, way.nodes.front().x,
				way.nodes.front().z};
		for (const auto &node : way.nodes) {
			b.min_x = std::min(b.min_x, node.x);
			b.min_z = std::min(b.min_z, node.z);
			b.max_x = std::max(b.max_x, node.x);
			b.max_z = std::max(b.max_z, node.z);
		}
		osm_buildings.push_back(b);
	}

	if (osm_buildings.empty())
		return overture_elements;

	constexpr int CELL_SIZE = 64;
	int grid_min_x = osm_buildings.front().min_x;
	int grid_min_z = osm_buildings.front().min_z;
	for (const auto &b : osm_buildings) {
		grid_min_x = std::min(grid_min_x, b.min_x);
		grid_min_z = std::min(grid_min_z, b.min_z);
	}

	struct PairHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept
		{
			return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
										  static_cast<unsigned int>(p.second));
		}
	};
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, PairHash> grid;
	for (std::size_t i = 0; i < osm_buildings.size(); ++i) {
		const auto &b = osm_buildings[i];
		const int cell_x0 = (b.min_x - grid_min_x) / CELL_SIZE;
		const int cell_z0 = (b.min_z - grid_min_z) / CELL_SIZE;
		const int cell_x1 = (b.max_x - grid_min_x) / CELL_SIZE;
		const int cell_z1 = (b.max_z - grid_min_z) / CELL_SIZE;
		for (int cx = cell_x0; cx <= cell_x1; ++cx)
			for (int cz = cell_z0; cz <= cell_z1; ++cz)
				grid[{cx, cz}].push_back(i);
	}

	std::vector<ProcessedElement> out;
	out.reserve(overture_elements.size());
	for (auto &element : overture_elements) {
		if (!element.is_way()) {
			out.push_back(std::move(element));
			continue;
		}
		const auto &way = element.as_way();
		if (way.nodes.empty())
			continue;
		long long sum_x = 0;
		long long sum_z = 0;
		for (const auto &node : way.nodes) {
			sum_x += node.x;
			sum_z += node.z;
		}
		const int cx = static_cast<int>(sum_x / static_cast<long long>(way.nodes.size()));
		const int cz = static_cast<int>(sum_z / static_cast<long long>(way.nodes.size()));
		const auto key = std::make_pair(
				(cx - grid_min_x) / CELL_SIZE, (cz - grid_min_z) / CELL_SIZE);

		bool duplicate = false;
		if (auto it = grid.find(key); it != grid.end()) {
			for (std::size_t idx : it->second) {
				const auto &b = osm_buildings[idx];
				if (cx >= b.min_x && cx <= b.max_x && cz >= b.min_z && cz <= b.max_z) {
					duplicate = true;
					break;
				}
			}
		}
		if (!duplicate)
			out.push_back(std::move(element));
	}
	return out;
}

}
