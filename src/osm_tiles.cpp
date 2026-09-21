#include "osm_tiles.h"
#include "overture/pmtiles.h"
#include "retrieve_data.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <curl/curl.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <zstd.h>

namespace arnis::osm_tiles
{
std::filesystem::path cache_root()
{
#if defined(_WIN32)
	if (const char *local = std::getenv("LOCALAPPDATA"); local && *local)
		return std::filesystem::path(local) / "arnis" / "osm-tiles";
#elif defined(__APPLE__)
	if (const char *home = std::getenv("HOME"); home && *home)
		return std::filesystem::path(home) / "Library" / "Caches" / "arnis" / "osm-tiles";
#endif
	if (const char *xdg = std::getenv("XDG_CACHE_HOME"); xdg && *xdg)
		return std::filesystem::path(xdg) / "arnis" / "osm-tiles";
	if (const char *home = std::getenv("HOME"); home && *home)
		return std::filesystem::path(home) / ".cache" / "arnis" / "osm-tiles";
	return {};
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
		std::uint64_t offset, std::uint64_t length, std::string *error)
{
	if (!length || length > 256 * 1024 * 1024) {
		if (error)
			*error = "invalid archive range";
		return std::nullopt;
	}
	const auto root = cache_root();
	const auto cached = root.empty() ? std::filesystem::path{}
									 : root / cache_key(url) /
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
		std::ofstream out(cached, std::ios::binary | std::ios::trunc);
		if (out)
			out.write(reinterpret_cast<const char *>(body.data()),
					static_cast<std::streamsize>(body.size()));
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
	for (const auto byte : value) {
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
bool oid(std::int64_t v, std::uint64_t &out, std::string *e)
{
	if (v < 0 || static_cast<std::uint64_t>(v) >= SYNTHETIC_ID_BASE)
		return fail(e, "tile id out of range");
	out = static_cast<std::uint64_t>(v);
	return true;
}
}

bool decode(const std::vector<std::uint8_t> &b, DecodedTile &out, std::string *error)
{
	out = {};
	if (b.size() < 4 || !std::equal(b.begin(), b.begin() + 4, "AOT1"))
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
				!coord(v, 90000000, n.lat, error) || !step(r, lon, v, error) ||
				!coord(v, 180000000, n.lon, error) || !read_tags(n.tags))
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
			if (!step(r, y, v, error) || !coord(v, 90000000, yy, error) ||
					!step(r, x, v, error) || !coord(v, 180000000, xx, error))
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
	std::map<std::pair<std::int32_t, std::int32_t>, std::uint64_t> coord_ids;
	std::uint64_t next = SYNTHETIC_ID_BASE;
	std::vector<DecodedNode> synthetic;
	for (const auto &[id, n] : nodes) {
		coord_ids.emplace(std::make_pair(n.lat, n.lon), id);
		out.nodes.push_back(
				{id, double(n.lat) / COORD_SCALE, double(n.lon) / COORD_SCALE, n.tags});
	}
	std::vector<std::uint64_t> wi;
	for (const auto &[id, _] : ways)
		wi.push_back(id);
	std::sort(wi.begin(), wi.end());
	for (auto id : wi) {
		const auto &w = ways.at(id);
		osm_parser::RawWay rw;
		rw.id = id;
		rw.tags = w.tags;
		for (const auto &p : w.points) {
			auto it = coord_ids.find(p);
			if (it == coord_ids.end()) {
				it = coord_ids.emplace(p, next).first;
				synthetic.push_back({next, p.first, p.second, {}});
				++next;
			}
			rw.node_refs.push_back(it->second);
		}
		if (w.closed && rw.node_refs.size() > 1 &&
				rw.node_refs.front() != rw.node_refs.back())
			rw.node_refs.push_back(rw.node_refs.front());
		out.ways.push_back(std::move(rw));
	}
	for (const auto &n : synthetic)
		out.nodes.push_back(
				{n.id, double(n.lat) / COORD_SCALE, double(n.lon) / COORD_SCALE, {}});
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
		if (!cache_manifest.empty()) {
			std::error_code ec;
			std::filesystem::create_directories(cache_manifest.parent_path(), ec);
			std::ofstream out(cache_manifest, std::ios::binary | std::ios::trunc);
			if (out)
				out.write(reinterpret_cast<const char *>(manifest_bytes.data()),
						static_cast<std::streamsize>(manifest_bytes.size()));
		}
	}
	const auto json = nlohmann::json::parse(manifest_bytes, nullptr, false);
	if (json.is_discarded() || !json.is_object() || json.value("zoom", 0) != ZOOM ||
			!json.contains("archives") || !json["archives"].is_array()) {
		if (error)
			*error = "bad archive index";
		return std::nullopt;
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
		auto range = [&](std::uint64_t off, std::uint64_t len) {
			return http_range(root + "/" + file, off, len, &local_error);
		};
		auto archive = overture::pmtiles::open_archive(range);
		if (!archive)
			continue;
		for (const auto &[x, y] : wanted) {
			auto raw = overture::pmtiles::read_tile(
					archive->header, archive->root_directory, ZOOM, x, y, range);
			if (!raw || raw->empty())
				continue;
			auto plain = zstd_decode(*raw, &local_error);
			if (!plain)
				continue;
			DecodedTile tile;
			if (decode(*plain, tile, &local_error))
				decoded.push_back(std::move(tile));
		}
	}
	if (decoded.empty()) {
		if (error)
			*error = "the tile archive has no data for this area";
		return std::nullopt;
	}
	return assemble(decoded);
}
} // namespace arnis::osm_tiles
