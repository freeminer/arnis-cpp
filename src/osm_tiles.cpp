#include "osm_tiles.h"
#include "cache_root.h"
#include "overture/pmtiles.h"
#include "retrieve_data.h"
#include "world_utils.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <curl/curl.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <zstd.h>

namespace arnis::osm_tiles
{
namespace
{
constexpr std::int64_t EDGE_MARGIN_E7 = 10'000;
constexpr std::int64_t MAX_LAT_E7 = 900'000'000;
constexpr std::int64_t MAX_LON_E7 = 1'800'000'000;
constexpr std::uint64_t SPILL_LAT = (std::uint64_t{1} << 31) - 1;
constexpr std::uint64_t RELATION_TILE_BASE = ((std::uint64_t{1} << 40) - 1) / 3;
constexpr std::uint64_t MAX_RECORD_BYTES = 16 * 1024 * 1024;

bool building_tags(const tags_t &tags)
{
	return tags.contains("building") || tags.contains("building:part") ||
		   (tags.get("type") == "building");
}

struct TileExtent
{
	std::int32_t min_lat = 0, min_lon = 0, max_lat = 0, max_lon = 0;
	bool valid = false;
};

TileExtent way_extent(const DecodedWay &way)
{
	TileExtent e;
	for (const auto &[lat, lon] : way.points) {
		e.min_lat = e.valid ? std::min(e.min_lat, lat) : lat;
		e.max_lat = e.valid ? std::max(e.max_lat, lat) : lat;
		e.min_lon = e.valid ? std::min(e.min_lon, lon) : lon;
		e.max_lon = e.valid ? std::max(e.max_lon, lon) : lon;
		e.valid = true;
	}
	return e;
}

bool intersects(const TileExtent &a, const TileExtent &b)
{
	return a.valid && b.valid && a.max_lat >= b.min_lat && a.min_lat <= b.max_lat &&
		   a.max_lon >= b.min_lon && a.min_lon <= b.max_lon;
}

TileExtent extent_union(const TileExtent &a, const TileExtent &b)
{
	if (!a.valid)
		return b;
	if (!b.valid)
		return a;
	return {std::min(a.min_lat, b.min_lat), std::min(a.min_lon, b.min_lon),
			std::max(a.max_lat, b.max_lat), std::max(a.max_lon, b.max_lon), true};
}

TileExtent relation_extent(const DecodedRelation &relation,
		const std::unordered_map<std::uint64_t, DecodedWay> &ways)
{
	TileExtent extent;
	for (const auto &member : relation.members) {
		const auto it = ways.find(member.ref);
		if (it != ways.end())
			extent = extent_union(extent, way_extent(it->second));
	}
	return extent;
}

std::vector<DecodedTile> select_for_bbox(
		const std::vector<DecodedTile> &input, const geographic::LLBBox &bbox)
{
	// Match Rust's extent exactly: outward rounding plus the small margin keeps
	// ways whose projected vertices land on the edge row/column available to the
	// downstream bbox clipper.
	const auto lower_e7 = [](double value, std::int64_t limit) {
		return std::clamp(static_cast<std::int64_t>(std::floor(value * COORD_SCALE)) -
								  EDGE_MARGIN_E7,
				-limit, limit);
	};
	const auto upper_e7 = [](double value, std::int64_t limit) {
		return std::clamp(static_cast<std::int64_t>(std::ceil(value * COORD_SCALE)) +
								  EDGE_MARGIN_E7,
				-limit, limit);
	};
	const TileExtent area{
			static_cast<std::int32_t>(lower_e7(bbox.min().lat(), MAX_LAT_E7)),
			static_cast<std::int32_t>(lower_e7(bbox.min().lng(), MAX_LON_E7)),
			static_cast<std::int32_t>(upper_e7(bbox.max().lat(), MAX_LAT_E7)),
			static_cast<std::int32_t>(upper_e7(bbox.max().lng(), MAX_LON_E7)), true};
	std::unordered_map<std::uint64_t, DecodedWay> ways;
	std::unordered_map<std::uint64_t, DecodedRelation> relations;
	for (const auto &tile : input) {
		for (const auto &way : tile.ways)
			ways.emplace(way.id, way);
		for (const auto &relation : tile.relations)
			relations.emplace(relation.id, relation);
	}
	std::unordered_set<std::uint64_t> keep_ways, keep_relations;
	TileExtent building_extent = area;
	for (const auto &[id, way] : ways)
		if (intersects(way_extent(way), area)) {
			keep_ways.insert(id);
			if (building_tags(way.tags))
				building_extent = extent_union(building_extent, way_extent(way));
		}
	for (const auto &[id, relation] : relations) {
		const auto extent = relation_extent(relation, ways);
		if (intersects(extent, area)) {
			keep_relations.insert(id);
			for (const auto &member : relation.members)
				keep_ways.insert(member.ref);
			if (building_tags(relation.tags) && extent.valid)
				building_extent = extent_union(building_extent, extent);
		}
	}
	// Rust unions the outlines of every selected way (including relation
	// members) before selecting adjacent building parts.
	for (const auto id : keep_ways)
		if (const auto it = ways.find(id);
				it != ways.end() && building_tags(it->second.tags))
			building_extent = extent_union(building_extent, way_extent(it->second));
	// Keep building parts intersecting the bbox or the footprint of a selected
	// building. This is Rust's union of the requested area and selected outline
	// extents; unrelated building relations are deliberately not pulled in.
	for (const auto &[id, way] : ways)
		if (!keep_ways.contains(id) && building_tags(way.tags)) {
			const auto extent = way_extent(way);
			if (intersects(extent, building_extent))
				keep_ways.insert(id);
		}
	// Building relations can contain parts extending beyond their outer relation's
	// initial selected extent. Rust includes those relations when any member
	// overlaps the accumulated requested/selected-building extent.
	for (const auto &[id, relation] : relations)
		if (!keep_relations.contains(id) && building_tags(relation.tags) &&
				intersects(relation_extent(relation, ways), building_extent)) {
			keep_relations.insert(id);
			for (const auto &member : relation.members)
				keep_ways.insert(member.ref);
		}
	std::vector<DecodedTile> selected;
	for (const auto &tile : input) {
		DecodedTile out;
		for (const auto &way : tile.ways)
			if (keep_ways.contains(way.id)) {
				out.ways.push_back(way);
			}
		for (const auto &relation : tile.relations)
			if (keep_relations.contains(relation.id))
				out.relations.push_back(relation);
		std::set<std::pair<std::int32_t, std::int32_t>> vertices;
		for (const auto &way : out.ways)
			for (const auto &point : way.points)
				vertices.insert(point);
		for (const auto &node : tile.nodes) {
			const bool in_area = node.lat >= area.min_lat && node.lat <= area.max_lat &&
								 node.lon >= area.min_lon && node.lon <= area.max_lon;
			if (in_area || vertices.contains({node.lat, node.lon}))
				out.nodes.push_back(node);
		}
		// Rust's assembler keeps in-bbox standalone nodes as well as ways and
		// relations. Dropping a node-only tile here loses mapped POIs and other
		// point features when no selected way shares their archive tile.
		if (!out.nodes.empty() || !out.ways.empty() || !out.relations.empty())
			selected.push_back(std::move(out));
	}
	return selected;
}
}

std::filesystem::path cache_root()
{
	return arnis::cache::provider_cache_root("osm-tiles");
}

elevation::CacheClearStats clear_osm_tiles_cache()
{
	const auto root = cache_root();
	return root.empty() ? elevation::CacheClearStats{} : elevation::clear_cache_dir(root);
}

namespace
{
size_t append_bytes(void *ptr, size_t size, size_t count, void *userdata)
{
	auto *out = static_cast<std::vector<std::uint8_t> *>(userdata);
	const auto *first = static_cast<const std::uint8_t *>(ptr);
	out->insert(out->end(), first, first + size * count);
	return size * count;
}
std::string cache_key(const std::string &value);
std::optional<std::vector<std::uint8_t>> http_range(const std::string &url,
		const std::filesystem::path &archive_cache, std::uint64_t offset,
		std::uint64_t length, std::string *error)
{
	if (!length || length > 256 * 1024 * 1024) {
		if (error)
			*error = "invalid archive range";
		return std::nullopt;
	}
	const auto cached = archive_cache.empty()
								? std::filesystem::path{}
								: archive_cache / cache_key(url) /
										  (std::to_string(offset) + "-" +
												  std::to_string(length) + ".bin");
	if (!cached.empty()) {
		std::error_code ec;
		if (std::filesystem::exists(cached, ec)) {
			std::ifstream in(cached, std::ios::binary);
			std::vector<std::uint8_t> bytes;
			bytes.assign(std::istreambuf_iterator<char>(in), {});
			if (bytes.size() == length)
				return bytes;
		}
	}
	CURL *curl = curl_easy_init();
	if (!curl) {
		if (error)
			*error = "curl initialization failed";
		return std::nullopt;
	}
	std::vector<std::uint8_t> body;
	const auto range = std::to_string(offset) + "-" + std::to_string(offset + length - 1);
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_RANGE, range.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_bytes);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
	curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);
	const auto rc = curl_easy_perform(curl);
	long status = 0;
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (rc != CURLE_OK || (status != 206 && !(offset == 0 && status == 200))) {
		if (error)
			*error = "range request failed";
		return std::nullopt;
	}
	if (status == 206 && body.size() != length) {
		if (error)
			*error = "range response has unexpected length";
		return std::nullopt;
	}
	if (!cached.empty() && body.size() == length) {
		std::error_code ec;
		std::filesystem::create_directories(cached.parent_path(), ec);
		arnis::world_utils::replace_file_atomically(cached, body);
	}
	return body;
}
std::optional<std::vector<std::uint8_t>> zstd_decode(
		const std::vector<std::uint8_t> &input, std::string *error)
{
	const auto size = ZSTD_getFrameContentSize(input.data(), input.size());
	if (size == ZSTD_CONTENTSIZE_ERROR || size > 256 * 1024 * 1024) {
		if (error)
			*error = "tile has invalid zstd frame";
		return std::nullopt;
	}
	if (size != ZSTD_CONTENTSIZE_UNKNOWN) {
		std::vector<std::uint8_t> output(static_cast<size_t>(size));
		const auto result =
				ZSTD_decompress(output.data(), output.size(), input.data(), input.size());
		if (!ZSTD_isError(result) && result == output.size())
			return output;
	}
	ZSTD_DStream *stream = ZSTD_createDStream();
	if (!stream || ZSTD_isError(ZSTD_initDStream(stream))) {
		ZSTD_freeDStream(stream);
		if (error)
			*error = "tile zstd decompression failed";
		return std::nullopt;
	}
	ZSTD_inBuffer in{input.data(), input.size(), 0};
	std::vector<std::uint8_t> output;
	std::array<std::uint8_t, 64 * 1024> buffer{};
	std::size_t remaining = 1;
	while (in.pos < in.size || remaining != 0) {
		ZSTD_outBuffer out{buffer.data(), buffer.size(), 0};
		remaining = ZSTD_decompressStream(stream, &out, &in);
		if (ZSTD_isError(remaining) || output.size() > 256 * 1024 * 1024 - out.pos) {
			ZSTD_freeDStream(stream);
			if (error)
				*error = "tile zstd decompression failed";
			return std::nullopt;
		}
		output.insert(output.end(), buffer.begin(), buffer.begin() + out.pos);
		if (out.pos == 0 && in.pos == in.size && remaining != 0) {
			ZSTD_freeDStream(stream);
			if (error)
				*error = "tile zstd frame is truncated";
			return std::nullopt;
		}
	}
	ZSTD_freeDStream(stream);
	return output;
}
std::string cache_key(const std::string &value)
{
	std::uint64_t hash = 0xcbf29ce484222325ULL;
	for (const auto &byte : value) {
		hash ^= static_cast<std::uint8_t>(byte);
		hash *= 0x100000001b3ULL;
	}
	std::ostringstream out;
	out << std::hex << std::setw(16) << std::setfill('0') << hash;
	return out.str();
}
struct Reader
{
	const std::vector<std::uint8_t> &b;
	size_t p = 0;
	bool uvar(std::uint64_t &v)
	{
		v = 0;
		for (unsigned shift = 0; shift < 64; shift += 7) {
			if (p >= b.size())
				return false;
			const auto c = b[p++];
			if (shift == 63 && c > 1)
				return false;
			v |= std::uint64_t(c & 0x7f) << shift;
			if (!(c & 0x80))
				return true;
		}
		return false;
	}
	bool svar(std::int64_t &v)
	{
		std::uint64_t n;
		if (!uvar(n))
			return false;
		// Zig-zag decode without relying on a signed right shift.  Keep the
		// unsigned sign mask separate: this is the exact inverse of Rust's
		// `(value >> 1) ^ -(value & 1)` and avoids signed-overflow concerns.
		const auto sign_mask = std::uint64_t{0} - (n & std::uint64_t{1});
		v = static_cast<std::int64_t>((n >> 1) ^ sign_mask);
		return true;
	}
	bool byte(std::uint8_t &v)
	{
		if (p >= b.size())
			return false;
		v = b[p++];
		return true;
	}
};
bool fail(std::string *e, const char *message)
{
	if (e)
		*e = message;
	return false;
}
bool step(Reader &r, std::int64_t &acc, std::int64_t &value, std::string *e)
{
	std::int64_t delta;
	if (!r.svar(delta) ||
			((delta > 0 && acc > std::numeric_limits<std::int64_t>::max() - delta) ||
					(delta < 0 &&
							acc < std::numeric_limits<std::int64_t>::min() - delta)))
		return fail(e, "tile delta overflows");
	acc += delta;
	value = acc;
	return true;
}
bool coord(std::int64_t v, std::int64_t limit, std::int32_t &out, std::string *e)
{
	if (v < -limit || v > limit)
		return fail(e, "tile coordinate out of range");
	out = static_cast<std::int32_t>(v);
	return true;
}
bool scaled_coord(std::int64_t value, std::int64_t multiplier, std::int64_t limit,
		std::int32_t &out, std::string *error)
{
	if (value < -limit / multiplier || value > limit / multiplier)
		return fail(error, "tile coordinate out of range");
	return coord(value * multiplier, limit, out, error);
}
bool oid(std::int64_t v, std::uint64_t &out, std::string *e)
{
	if (v < 0 || static_cast<std::uint64_t>(v) >= SYNTHETIC_ID_BASE)
		return fail(e, "tile id out of range");
	out = static_cast<std::uint64_t>(v);
	return true;
}

std::uint64_t coordinate_node_id(const std::pair<std::int32_t, std::int32_t> &coordinate)
{
	const auto lat = static_cast<std::uint64_t>(
			static_cast<std::int64_t>(coordinate.first) + MAX_LAT_E7);
	const auto lon = static_cast<std::uint64_t>(coordinate.second) &
					 ((std::uint64_t{1} << 30) - 1);
	return SYNTHETIC_ID_BASE | (lat << 30) | lon;
}

struct CoordinateNodeIds
{
	struct PairHash
	{
		std::size_t operator()(
				const std::pair<std::int32_t, std::int32_t> &p) const noexcept
		{
			return (std::uint64_t(static_cast<std::uint32_t>(p.first)) << 32) ^
				   static_cast<std::uint32_t>(p.second);
		}
	};
	std::unordered_map<std::uint64_t, std::pair<std::int32_t, std::int32_t>> owners;
	std::unordered_map<std::pair<std::int32_t, std::int32_t>, std::uint64_t, PairHash>
			spills;

	std::pair<std::uint64_t, bool> get(const std::pair<std::int32_t, std::int32_t> &point)
	{
		const auto id = coordinate_node_id(point);
		const auto found = owners.find(id);
		if (found == owners.end()) {
			owners.emplace(id, point);
			return {id, true};
		}
		if (found->second == point)
			return {id, false};
		if (const auto spilled = spills.find(point); spilled != spills.end())
			return {spilled->second, false};
		const auto spill_id = SYNTHETIC_ID_BASE | (SPILL_LAT << 30) | spills.size();
		spills.emplace(point, spill_id);
		return {spill_id, true};
	}
};
}

bool decode(const std::vector<std::uint8_t> &b, DecodedTile &out, std::string *error)
{
	out = {};
	if (b.size() < 4)
		return fail(error, "not an Arnis tile payload");
	const std::int64_t coordinate_multiplier =
			std::equal(b.begin(), b.begin() + 4, "AOT2")   ? 1
			: std::equal(b.begin(), b.begin() + 4, "AOT1") ? 10
														   : 0;
	if (!coordinate_multiplier)
		return fail(error, "not an Arnis tile payload");
	Reader r{b};
	r.p = 4;
	std::uint64_t nstrings;
	if (!r.uvar(nstrings) || nstrings > (1u << 24))
		return fail(error, "implausible string table");
	std::vector<std::string> strings;
	strings.reserve(static_cast<size_t>(std::min<std::uint64_t>(nstrings, 4096)));
	for (std::uint64_t i = 0; i < nstrings; ++i) {
		std::uint64_t len;
		if (!r.uvar(len) || len > b.size() - std::min(r.p, b.size()) || len > (1u << 28))
			return fail(error, "truncated string");
		strings.emplace_back(
				reinterpret_cast<const char *>(b.data() + r.p), static_cast<size_t>(len));
		r.p += static_cast<size_t>(len);
	}
	auto string_at = [&](std::uint64_t i, std::string &s) {
		if (i >= strings.size())
			return false;
		s = strings[static_cast<size_t>(i)];
		return true;
	};
	auto read_tags = [&](tags_t &tags) {
		std::uint64_t n;
		if (!r.uvar(n) || n > (1u << 16))
			return false;
		for (std::uint64_t i = 0; i < n; ++i) {
			std::uint64_t k, v;
			std::string ks, vs;
			if (!r.uvar(k) || !r.uvar(v) || !string_at(k, ks) || !string_at(v, vs))
				return false;
			tags[std::move(ks)] = std::move(vs);
		}
		return true;
	};
	std::uint64_t count;
	if (!r.uvar(count) || count > (1u << 24))
		return fail(error, "implausible node count");
	std::int64_t id = 0, lat = 0, lon = 0, v;
	for (std::uint64_t i = 0; i < count; ++i) {
		DecodedNode n;
		if (!step(r, id, v, error) || !oid(v, n.id, error) || !step(r, lat, v, error) ||
				!scaled_coord(v, coordinate_multiplier, MAX_LAT_E7, n.lat, error) ||
				!step(r, lon, v, error) ||
				!scaled_coord(v, coordinate_multiplier, MAX_LON_E7, n.lon, error) ||
				!read_tags(n.tags))
			return false;
		out.nodes.push_back(std::move(n));
	}
	if (!r.uvar(count) || count > (1u << 24))
		return fail(error, "implausible way count");
	id = 0;
	for (std::uint64_t i = 0; i < count; ++i) {
		DecodedWay w;
		std::uint8_t closed;
		if (!step(r, id, v, error) || !oid(v, w.id, error) || !r.byte(closed) ||
				!read_tags(w.tags))
			return fail(error, "truncated way");
		w.closed = closed != 0;
		std::uint64_t np;
		if (!r.uvar(np) || np > (1u << 22))
			return fail(error, "implausible vertex count");
		std::int64_t y = 0, x = 0;
		for (std::uint64_t j = 0; j < np; ++j) {
			std::int32_t yy, xx;
			if (!step(r, y, v, error) ||
					!scaled_coord(v, coordinate_multiplier, MAX_LAT_E7, yy, error) ||
					!step(r, x, v, error) ||
					!scaled_coord(v, coordinate_multiplier, MAX_LON_E7, xx, error))
				return false;
			w.points.emplace_back(yy, xx);
		}
		out.ways.push_back(std::move(w));
	}
	if (!r.uvar(count) || count > (1u << 24))
		return fail(error, "implausible relation count");
	id = 0;
	for (std::uint64_t i = 0; i < count; ++i) {
		DecodedRelation rel;
		if (!step(r, id, v, error) || !oid(v, rel.id, error) || !read_tags(rel.tags))
			return fail(error, "truncated relation");
		std::uint64_t nm;
		if (!r.uvar(nm) || nm > (1u << 20))
			return fail(error, "implausible member count");
		std::int64_t ref = 0;
		for (std::uint64_t j = 0; j < nm; ++j) {
			DecodedRelationMember m;
			if (!step(r, ref, v, error) || !oid(v, m.ref, error))
				return false;
			std::uint64_t role;
			if (!r.uvar(role) || !string_at(role, m.role))
				return fail(error, "relation role out of range");
			rel.members.push_back(std::move(m));
		}
		out.relations.push_back(std::move(rel));
	}
	return true;
}

osm_parser::RawOsmDocument assemble(const std::vector<DecodedTile> &tiles)
{
	std::unordered_map<std::uint64_t, DecodedNode> nodes;
	std::unordered_map<std::uint64_t, DecodedWay> ways;
	std::unordered_map<std::uint64_t, DecodedRelation> relations;
	for (const auto &tile : tiles) {
		for (const auto &n : tile.nodes)
			nodes.emplace(n.id, n);
		for (const auto &w : tile.ways)
			ways.emplace(w.id, w);
		for (const auto &r : tile.relations)
			relations.emplace(r.id, r);
	}
	osm_parser::RawOsmDocument out;
	using Coordinate = std::pair<std::int32_t, std::int32_t>;
	std::map<Coordinate, std::uint64_t> lowest_source_id;
	// Rust sorts archive nodes by ID before exposing them to the parser.  Do
	// the same here: unordered_map iteration otherwise changes element order
	// between processes, which changes seeded tree/structure decisions.
	std::vector<std::uint64_t> ni;
	ni.reserve(nodes.size());
	for (const auto &[id, _] : nodes)
		ni.push_back(id);
	std::sort(ni.begin(), ni.end());
	for (const auto &id : ni) {
		const auto &node = nodes.at(id);
		const Coordinate coordinate{node.lat, node.lon};
		lowest_source_id.emplace(coordinate, id);
	}
	std::vector<std::uint64_t> wi;
	for (const auto &[id, _] : ways)
		wi.push_back(id);
	std::sort(wi.begin(), wi.end());
	CoordinateNodeIds node_ids;
	std::map<Coordinate, std::uint64_t> vertex_ids;
	std::vector<Coordinate> synthetic;
	for (const auto id : wi)
		for (const auto &coordinate : ways.at(id).points) {
			if (vertex_ids.contains(coordinate))
				continue;
			const auto [node_id, first] = node_ids.get(coordinate);
			vertex_ids.emplace(coordinate, node_id);
			if (first && !lowest_source_id.contains(coordinate))
				synthetic.push_back(coordinate);
		}
	for (const auto &id : ni) {
		const auto &node = nodes.at(id);
		const Coordinate coordinate{node.lat, node.lon};
		const auto source = lowest_source_id.find(coordinate);
		const bool promoted = vertex_ids.contains(coordinate) &&
							  (source == lowest_source_id.end() || source->second == id);
		out.nodes.push_back({promoted ? vertex_ids.at(coordinate) : id,
				double(node.lat) / COORD_SCALE, double(node.lon) / COORD_SCALE,
				node.tags});
	}
	for (auto id : wi) {
		const auto &w = ways.at(id);
		osm_parser::RawWay rw;
		rw.id = id;
		rw.tags = w.tags;
		for (const auto &p : w.points)
			rw.node_refs.push_back(vertex_ids.at(p));
		if (w.closed && rw.node_refs.size() > 1 &&
				rw.node_refs.front() != rw.node_refs.back())
			rw.node_refs.push_back(rw.node_refs.front());
		out.ways.push_back(std::move(rw));
	}
	for (const auto &coordinate : synthetic)
		out.nodes.push_back(
				{vertex_ids.at(coordinate), double(coordinate.first) / COORD_SCALE,
						double(coordinate.second) / COORD_SCALE, {}});
	std::vector<std::uint64_t> ri;
	for (const auto &[id, _] : relations)
		ri.push_back(id);
	std::sort(ri.begin(), ri.end());
	for (auto id : ri) {
		const auto &r = relations.at(id);
		osm_parser::RawRelation rr;
		rr.id = id;
		rr.tags = r.tags;
		for (const auto &m : r.members)
			rr.members.push_back({"way", m.ref, m.role});
		out.relations.push_back(std::move(rr));
	}
	return out;
}

std::optional<osm_parser::RawOsmDocument> fetch_data_from_tiles_quietly(
		const geographic::LLBBox &bbox, const std::string &base_url, std::string *error)
{
	return fetch_data_from_tiles(bbox, base_url, error);
}

std::optional<osm_parser::RawOsmDocument> fetch_data_from_tiles(
		const geographic::LLBBox &bbox, const std::string &base_url, std::string *error)
{
	if (base_url.empty()) {
		if (error)
			*error = "tile archive URL is empty";
		return std::nullopt;
	}
	const auto root =
			base_url.back() == '/' ? base_url.substr(0, base_url.size() - 1) : base_url;
	std::string local_error;
	std::vector<std::uint8_t> manifest_bytes;
	const auto cache = cache_root();
	const auto archive_cache =
			cache.empty() ? std::filesystem::path{} : cache / cache_key(root);
	bool fetched_manifest = false;
	const auto cache_manifest =
			cache.empty() ? std::filesystem::path{}
						  : cache / ("manifest-" + cache_key(root) + ".json");
	if (!cache_manifest.empty()) {
		std::error_code ec;
		const auto age = std::filesystem::file_time_type::clock::now() -
						 std::filesystem::last_write_time(cache_manifest, ec);
		if (!ec && age < std::chrono::hours(24)) {
			std::ifstream in(cache_manifest, std::ios::binary);
			manifest_bytes.assign(std::istreambuf_iterator<char>(in), {});
		}
	}
	if (!manifest_bytes.empty()) {
		const auto cached_json = nlohmann::json::parse(manifest_bytes, nullptr, false);
		if (cached_json.is_discarded() || !cached_json.is_object() ||
				cached_json.value("zoom", 0) != ZOOM ||
				!cached_json.contains("archives") || !cached_json["archives"].is_array())
			manifest_bytes.clear();
	}
	if (manifest_bytes.empty()) {
		CURL *curl = curl_easy_init();
		if (!curl) {
			if (error)
				*error = "curl initialization failed";
			return std::nullopt;
		}
		const auto manifest_url = root + "/archives.json";
		curl_easy_setopt(curl, CURLOPT_URL, manifest_url.c_str());
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, append_bytes);
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, &manifest_bytes);
		curl_easy_setopt(curl, CURLOPT_USERAGENT, retrieve_data::OSM_USER_AGENT);
		curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
		curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
		curl_easy_setopt(curl, CURLOPT_TIMEOUT, 120L);
		const auto manifest_rc = curl_easy_perform(curl);
		long manifest_status = 0;
		curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &manifest_status);
		curl_easy_cleanup(curl);
		if (manifest_rc != CURLE_OK || manifest_status < 200 || manifest_status >= 300) {
			if (error)
				*error = "tile archive index unreachable";
			return std::nullopt;
		}
		fetched_manifest = true;
		if (!cache_manifest.empty()) {
			std::error_code ec;
			std::filesystem::create_directories(cache_manifest.parent_path(), ec);
			arnis::world_utils::replace_file_atomically(cache_manifest, manifest_bytes);
		}
	}
	const auto json = nlohmann::json::parse(manifest_bytes, nullptr, false);
	if (json.is_discarded() || !json.is_object() || json.value("zoom", 0) != ZOOM ||
			!json.contains("archives") || !json["archives"].is_array()) {
		if (error)
			*error = "bad archive index";
		return std::nullopt;
	}
	// Keep each source's range cache separate, then discard archive ranges no
	// longer named by its refreshed manifest (rebakes publish new archive files).
	// Restrict cleanup to the exact source-scoped cache directory and hash-named
	// archive subdirectories; manifest files and unknown cache entries are left
	// untouched.
	if (fetched_manifest && !archive_cache.empty()) {
		std::unordered_set<std::string> keep;
		for (const auto &entry : json["archives"]) {
			const auto file = entry.value("file", std::string{});
			if (!file.empty() && file.size() <= 96 &&
					file.find("..") == std::string::npos &&
					file.find_first_not_of(
							"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.") ==
							std::string::npos)
				keep.insert(cache_key(root + "/" + file));
		}
		std::error_code ec;
		std::vector<std::filesystem::path> stale;
		for (std::filesystem::directory_iterator it(archive_cache, ec), end;
				!ec && it != end; it.increment(ec)) {
			std::error_code type_error;
			if (!it->is_directory(type_error))
				continue;
			const auto name = it->path().filename().string();
			const bool hash_name =
					name.size() == 16 &&
					name.find_first_not_of("0123456789abcdef") == std::string::npos;
			if (hash_name && !keep.contains(name))
				stale.push_back(it->path());
		}
		for (const auto &path : stale) {
			std::error_code remove_error;
			std::filesystem::remove_all(path, remove_error);
		}
	}
	const auto a =
			overture::pmtiles::lonlat_to_tile(bbox.min().lng(), bbox.max().lat(), ZOOM);
	const auto b =
			overture::pmtiles::lonlat_to_tile(bbox.max().lng(), bbox.min().lat(), ZOOM);
	const auto xs = std::min(a.first, b.first), xe = std::max(a.first, b.first);
	const auto ys = std::min(a.second, b.second), ye = std::max(a.second, b.second);
	const auto needed = (std::uint64_t(xe) - xs + 1) * (std::uint64_t(ye) - ys + 1);
	if (needed > 4096) {
		if (error)
			*error = "that area needs too many tiles";
		return std::nullopt;
	}
	std::vector<std::pair<std::uint32_t, std::uint32_t>> wanted;
	for (auto x = xs; x <= xe; ++x)
		for (auto y = ys; y <= ye; ++y)
			wanted.emplace_back(x, y);
	const auto cell_zoom = json.value("cell_zoom", 6u);
	if (cell_zoom > ZOOM) {
		if (error)
			*error = "archive index has cell zoom above archive zoom";
		return std::nullopt;
	}
	const auto shift = unsigned(ZOOM - cell_zoom);
	const std::uint32_t side = std::uint32_t{1} << cell_zoom;
	std::unordered_set<std::uint32_t> wanted_cells;
	for (const auto &[x, y] : wanted)
		wanted_cells.insert((y >> shift) * side + (x >> shift));
	std::vector<DecodedTile> decoded;
	struct OpenArchive
	{
		overture::pmtiles::Archive archive;
		overture::pmtiles::RangeReader read_range;
		std::unordered_set<std::uint64_t> relation_ids;
		bool aot2 = false;
	};
	std::vector<OpenArchive> opened;
	for (const auto &entry : json["archives"]) {
		const auto file = entry.value("file", std::string{});
		if (file.empty() || file.size() > 96 || file.find("..") != std::string::npos ||
				file.find_first_not_of(
						"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.") !=
						std::string::npos) {
			if (error)
				*error = "archive index has an unusable name";
			return std::nullopt;
		}
		// Legacy manifests have no coarse cells; retain their bbox coverage
		// check so unrelated archives are not opened in that format.
		if (!entry.contains("cells") || !entry["cells"].is_array() ||
				entry["cells"].empty()) {
			const auto min_lat = entry.value("min_lat", -90.0);
			const auto max_lat = entry.value("max_lat", 90.0);
			const auto min_lon = entry.value("min_lon", -180.0);
			const auto max_lon = entry.value("max_lon", 180.0);
			if (max_lat < bbox.min().lat() || min_lat > bbox.max().lat() ||
					max_lon < bbox.min().lng() || min_lon > bbox.max().lng())
				continue;
		}
		bool covers = true;
		if (entry.contains("cells") && entry["cells"].is_array() &&
				!entry["cells"].empty()) {
			covers = false;
			for (const auto &cell : entry["cells"])
				if (cell.is_number_unsigned() &&
						wanted_cells.contains(cell.get<std::uint32_t>())) {
					covers = true;
					break;
				}
		}
		if (!covers)
			continue;
		const auto archive_url = root + "/" + file;
		auto range = [archive_url, archive_cache](std::uint64_t off, std::uint64_t len) {
			std::string ignored_error;
			return http_range(archive_url, archive_cache, off, len, &ignored_error);
		};
		auto archive = overture::pmtiles::open_archive(range);
		if (!archive)
			continue;
		OpenArchive archive_state{*archive, range, {}, false};
		for (const auto &[x, y] : wanted) {
			auto raw = overture::pmtiles::read_tile(
					archive->header, archive->root_directory, ZOOM, x, y, range);
			if (!raw || raw->empty())
				continue;
			auto plain = zstd_decode(*raw, &local_error);
			if (!plain)
				continue;
			DecodedTile tile;
			if (decode(*plain, tile, &local_error)) {
				archive_state.aot2 |=
						std::equal(plain->begin(), plain->begin() + 4, "AOT2");
				for (const auto &relation : tile.relations)
					archive_state.relation_ids.insert(relation.id);
				decoded.push_back(std::move(tile));
			}
		}
		opened.push_back(std::move(archive_state));
	}
	if (decoded.empty()) {
		if (error)
			*error = "the tile archive has no data for this area";
		return std::nullopt;
	}
	// AOT2 tiles contain only relation members which touch that tile. If a
	// selected relation has missing ways, retrieve its whole-record entry from
	// each AOT2 archive that supplied one of its fragments before assembling.
	std::unordered_map<std::uint64_t, DecodedWay> available_ways;
	std::unordered_map<std::uint64_t, DecodedRelation> available_relations;
	for (const auto &tile : decoded) {
		for (const auto &way : tile.ways)
			available_ways.emplace(way.id, way);
		for (const auto &relation : tile.relations)
			available_relations.emplace(relation.id, relation);
	}
	const auto lower = [](double v, std::int64_t limit) {
		return static_cast<std::int32_t>(std::clamp(
				static_cast<std::int64_t>(std::floor(v * COORD_SCALE)) - EDGE_MARGIN_E7,
				-limit, limit));
	};
	const auto upper = [](double v, std::int64_t limit) {
		return static_cast<std::int32_t>(std::clamp(
				static_cast<std::int64_t>(std::ceil(v * COORD_SCALE)) + EDGE_MARGIN_E7,
				-limit, limit));
	};
	const TileExtent area{lower(bbox.min().lat(), MAX_LAT_E7),
			lower(bbox.min().lng(), MAX_LON_E7), upper(bbox.max().lat(), MAX_LAT_E7),
			upper(bbox.max().lng(), MAX_LON_E7), true};
	std::unordered_set<std::uint64_t> partial_relations;
	for (const auto &[id, relation] : available_relations) {
		bool missing_way = false;
		for (const auto &member : relation.members)
			missing_way |= !available_ways.contains(member.ref);
		if (missing_way && intersects(relation_extent(relation, available_ways), area))
			partial_relations.insert(id);
	}
	for (auto &archive : opened) {
		if (!archive.aot2)
			continue;
		overture::pmtiles::LeafDirectoryCache leaves;
		for (const auto id : partial_relations) {
			if (!archive.relation_ids.contains(id))
				continue;
			const auto location = overture::pmtiles::locate_id_checked(
					archive.archive.header, archive.archive.root_directory,
					RELATION_TILE_BASE + id, archive.read_range, &leaves);
			if (location.status != overture::pmtiles::TileReadStatus::Found ||
					location.location.length > MAX_RECORD_BYTES)
				continue;
			const auto record = overture::pmtiles::read_tile_payload_checked(
					archive.archive.header, location.location, archive.read_range);
			if (record.status != overture::pmtiles::TileReadStatus::Found ||
					record.bytes.empty())
				continue;
			const auto plain = zstd_decode(record.bytes, &local_error);
			if (!plain)
				continue;
			DecodedTile whole_relation;
			if (decode(*plain, whole_relation, &local_error)) {
				std::unordered_set<std::uint64_t> completed_ids;
				for (const auto &relation : whole_relation.relations)
					completed_ids.insert(relation.id);
				// Replace the tile's clipped member list with the authoritative record;
				// retaining the first fragment would still leave multipolygons open.
				for (auto &tile : decoded)
					std::erase_if(tile.relations, [&](const DecodedRelation &relation) {
						return completed_ids.contains(relation.id);
					});
				decoded.push_back(std::move(whole_relation));
			}
		}
	}
	const auto selected = select_for_bbox(decoded, bbox);
	if (selected.empty()) {
		if (error)
			*error = "the tile archive has no intersecting OSM elements";
		return std::nullopt;
	}
	return assemble(selected);
}
} // namespace arnis::osm_tiles
