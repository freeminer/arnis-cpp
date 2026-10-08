#include "canopy.h"
#include "../grid_ops.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <zlib.h>
#include <fstream>
#include <chrono>
#include <memory>
#include <mutex>
#include <limits>
#include <list>
#include <unordered_map>
#include "../../../http.h"
#include "../world_utils.h"

namespace arnis::canopy
{
namespace
{
constexpr double MERCATOR_WORLD = 20037508.342789244;
constexpr double COLUMNS_PER_TREE = 52.0;
constexpr double COLUMNS_PER_TREE_LEGACY = 25.0;
constexpr double SLOT_P_MAX = 0.85;
constexpr std::size_t ROW_BLOCK = 128;
constexpr std::uint64_t FETCH_BATCH_BYTES = 16ULL << 20;
constexpr auto ROW_CACHE_MAX_AGE = std::chrono::hours(24 * 30);
constexpr auto TILE_FAILURE_RETRY = std::chrono::minutes(5);
constexpr auto FAILURE_WARNING_RETRY = std::chrono::minutes(5);

// Immutable decoded rows shared across chunk workers. Disk strips remain the
// authoritative backing store; LRU eviction bounds retained payload to 64 MiB.
struct DecodedRows
{
	using Row = std::shared_ptr<const std::vector<std::uint8_t>>;
	struct Entry {
		Row row;
		std::list<std::string>::iterator position;
	};
	std::mutex mutex;
	std::list<std::string> lru;
	std::unordered_map<std::string, Entry> entries;
	std::size_t bytes{};
	Row get(const std::string &key)
	{
		std::lock_guard lock(mutex);
		const auto it = entries.find(key);
		if (it == entries.end())
			return {};
		lru.splice(lru.begin(), lru, it->second.position);
		return it->second.row;
	}
	Row put(const std::string &key, std::vector<std::uint8_t> values)
	{
		auto row = std::make_shared<const std::vector<std::uint8_t>>(std::move(values));
		std::lock_guard lock(mutex);
		if (const auto it = entries.find(key); it != entries.end())
			return it->second.row;
		while (bytes + row->size() > (64ULL << 20) && !lru.empty()) {
			const auto it = entries.find(lru.back());
			bytes -= it->second.row->size();
			entries.erase(it);
			lru.pop_back();
		}
		lru.push_front(key);
		entries.emplace(key, Entry{row, lru.begin()});
		bytes += row->size();
		return row;
	}
	void clear()
	{
		std::lock_guard lock(mutex);
		entries.clear();
		lru.clear();
		bytes = 0;
	}
};

DecodedRows &decoded_rows()
{
	static DecodedRows cache;
	return cache;
}

struct TileFetchState
{
	std::mutex mutex;
	std::chrono::steady_clock::time_point unavailable_until{};
};

std::shared_ptr<TileFetchState> tile_fetch_state(
		const std::filesystem::path &base, int xt, int yt)
{
	static std::mutex states_mutex;
	static std::unordered_map<std::string, std::shared_ptr<TileFetchState>> states;
	const auto key = cache_dir(base).string() + "/" + quadkey_of(xt, yt);
	std::lock_guard lock(states_mutex);
	if (const auto it = states.find(key); it != states.end())
		return it->second;
	auto created = std::make_shared<TileFetchState>();
	states.emplace(key, created);
	if (states.size() > 512) {
		const auto now = std::chrono::steady_clock::now();
		for (auto it = states.begin(); it != states.end();) {
			const auto &candidate = it->second;
			// An unshared state cannot be modified by an active tile request.
			if (candidate.use_count() == 1 && candidate->unavailable_until <= now)
				it = states.erase(it);
			else
				++it;
		}
	}
	return created;
}

bool should_log_canopy_failure()
{
	// Chunk generation asks for overlapping regions. A large area may touch
	// many unavailable source tiles, so per-tile throttling still permits a
	// warning storm on every new chunk. Keep retry state per tile, but rate-limit
	// canopy failure diagnostics for the process.
	static std::mutex warning_mutex;
	static std::chrono::steady_clock::time_point last_warning{};
	std::lock_guard lock(warning_mutex);
	const auto now = std::chrono::steady_clock::now();
	if (last_warning != std::chrono::steady_clock::time_point{} &&
			now - last_warning < FAILURE_WARNING_RETRY)
		return false;
	last_warning = now;
	return true;
}

void print_canopy_fetch_message_once()
{
	static std::once_flag once;
	std::call_once(once, [] {
		std::cout
				<< "Fetching canopy height data (Meta/WRI 1m global canopy height)...\n";
	});
}
}

CanopyData::CanopyData(std::vector<std::uint8_t> grid, std::size_t w, std::size_t h) :
		grid_(std::move(grid)), width(w), height(h)
{
}

std::uint8_t CanopyData::at(std::size_t gx, std::size_t gz) const
{
	return gx >= width || gz >= height || gz * width + gx >= grid_.size()
				   ? CANOPY_NODATA
				   : grid_[gz * width + gx];
}

void CanopyData::remap_rows_to_mercator(double lat_top, double lat_bottom)
{
	grid_ops::remap_flat_rows_nearest(grid_, width, [=](std::size_t z) {
		return grid_ops::mercator_source_row(lat_top, lat_bottom, height, z);
	});
}

void CanopyData::crop(
		std::size_t x0, std::size_t z0, std::size_t new_width, std::size_t new_height)
{
	grid_ops::crop_flat(grid_, width, x0, z0, new_width, new_height);
	width = new_width;
	height = new_height;
}

std::optional<std::uint8_t> CanopyData::canopy_height_m(
		std::size_t gx, std::size_t gz) const
{
	const auto v = at(gx, gz);
	return v == CANOPY_NODATA ? std::nullopt : std::optional<std::uint8_t>(v);
}
std::optional<double> CanopyData::canopy_fraction(
		std::size_t gx, std::size_t gz, int spacing) const
{
	if (spacing <= 0)
		return std::nullopt;
	std::size_t covered = 0, total = 0;
	for (int z = 0; z < spacing; ++z)
		for (int x = 0; x < spacing; ++x) {
			const auto v = at(gx + std::size_t(x), gz + std::size_t(z));
			if (v != CANOPY_NODATA) {
				++total;
				if (v >= CANOPY_MIN_M)
					++covered;
			}
		}
	return total ? std::optional<double>(double(covered) / double(total)) : std::nullopt;
}

std::tuple<std::size_t, std::size_t, double, std::uint8_t> CanopyData::stats() const
{
	std::size_t covered = 0, canopy = 0;
	std::uint64_t sum = 0;
	std::uint8_t maximum = 0;
	for (const auto &h : grid_) {
		if (h == CANOPY_NODATA)
			continue;
		++covered;
		if (h >= CANOPY_MIN_M) {
			++canopy;
			sum += h;
			maximum = std::max(maximum, h);
		}
	}
	return {covered, canopy, canopy ? double(sum) / canopy : 0.0, maximum};
}

double slot_probability(double fraction, int spacing, bool schematic_pack)
{
	const auto per_tree = schematic_pack ? COLUMNS_PER_TREE : COLUMNS_PER_TREE_LEGACY;
	return std::clamp(fraction * std::max(0, spacing) * std::max(0, spacing) / per_tree,
			0.0, SLOT_P_MAX);
}

std::pair<int, int> tile_xy(double lat, double lon)
{
	const auto n = double(1u << TILE_ZOOM);
	const auto x = int(std::floor((lon + 180.0) / 360.0 * n));
	const auto rad = std::clamp(lat, -85.05112878, 85.05112878) * std::acos(-1.0) / 180.0;
	const auto y = int(std::floor(
			(1.0 - std::log(std::tan(rad) + 1.0 / std::cos(rad)) / std::acos(-1.0)) *
			0.5 * n));
	return {std::clamp(x, 0, int(n) - 1), std::clamp(y, 0, int(n) - 1)};
}

std::string quadkey_of(int xt, int yt)
{
	std::string out;
	out.reserve(TILE_ZOOM);
	for (int bit = int(TILE_ZOOM) - 1; bit >= 0; --bit)
		out.push_back(char('0' + (((xt >> bit) & 1) | (((yt >> bit) & 1) << 1))));
	return out;
}

double merc_x(double lon)
{
	return MERCATOR_WORLD * lon / 180.0;
}
double merc_y(double lat)
{
	lat = std::clamp(lat, -85.05112878, 85.05112878);
	return MERCATOR_WORLD *
		   std::log(std::tan(std::acos(-1.0) / 4.0 + lat * std::acos(-1.0) / 360.0)) /
		   std::acos(-1.0);
}

namespace
{
std::uint16_t read_u16(const std::vector<std::uint8_t> &b, std::size_t at)
{
	return std::uint16_t(b[at]) | (std::uint16_t(b[at + 1]) << 8);
}
std::uint64_t read_u64(const std::vector<std::uint8_t> &b, std::size_t at)
{
	std::uint64_t value = 0;
	for (unsigned i = 0; i < 8; ++i)
		value |= std::uint64_t(b[at + i]) << (i * 8);
	return value;
}
}

std::vector<std::uint8_t> StripIndex::encode() const
{
	if (offsets.size() != TILE_PX || counts.size() != TILE_PX)
		return {};
	std::vector<std::uint8_t> out(TILE_PX * 16);
	auto put = [&](std::size_t pos, std::uint64_t value) {
		for (unsigned i = 0; i < 8; ++i)
			out[pos + i] = std::uint8_t(value >> (i * 8));
	};
	for (std::size_t i = 0; i < TILE_PX; ++i) {
		put(i * 8, offsets[i]);
		put((TILE_PX + i) * 8, counts[i]);
	}
	return out;
}

std::optional<StripIndex> StripIndex::decode(const std::vector<std::uint8_t> &bytes)
{
	if (bytes.size() != TILE_PX * 16)
		return std::nullopt;
	StripIndex out;
	out.offsets.resize(TILE_PX);
	out.counts.resize(TILE_PX);
	for (std::size_t i = 0; i < TILE_PX; ++i) {
		out.offsets[i] = read_u64(bytes, i * 8);
		out.counts[i] = read_u64(bytes, (TILE_PX + i) * 8);
	}
	return out;
}

std::optional<StripTableLocation> parse_bigtiff_header(const std::vector<std::uint8_t> &b)
{
	if (b.size() < 16 || b[0] != 'I' || b[1] != 'I' || read_u16(b, 2) != 43)
		return std::nullopt;
	const auto ifd = read_u64(b, 8);
	if (ifd + 8 > b.size())
		return std::nullopt;
	const auto entries = read_u64(b, std::size_t(ifd));
	struct Value
	{
		std::uint64_t count, value;
	};
	std::vector<std::pair<std::uint16_t, Value>> tags;
	for (std::uint64_t i = 0; i < entries; ++i) {
		const auto at = std::size_t(ifd + 8 + i * 20);
		if (at + 20 > b.size())
			return std::nullopt;
		tags.push_back({read_u16(b, at), {read_u64(b, at + 4), read_u64(b, at + 12)}});
	}
	auto tag = [&](std::uint16_t id) -> std::optional<Value> {
		for (const auto &[key, v] : tags)
			if (key == id)
				return v;
		return std::nullopt;
	};
	if (!tag(256) || !tag(257) || tag(256)->value != TILE_PX ||
			tag(257)->value != TILE_PX || !tag(258) || tag(258)->value != 8 ||
			!tag(259) || tag(259)->value != 8 || !tag(277) || tag(277)->value != 1 ||
			!tag(278) || tag(278)->value != 1 || !tag(317) || tag(317)->value != 2)
		return std::nullopt;
	auto offsets = tag(273), counts = tag(279);
	if (!offsets || !counts || offsets->count != TILE_PX || counts->count != TILE_PX)
		return std::nullopt;
	return StripTableLocation{offsets->value, counts->value};
}

std::optional<StripIndex> make_strip_index(
		const std::vector<std::uint8_t> &a, const std::vector<std::uint8_t> &b)
{
	if (a.size() < TILE_PX * 8 || b.size() < TILE_PX * 8)
		return std::nullopt;
	StripIndex out;
	out.offsets.resize(TILE_PX);
	out.counts.resize(TILE_PX);
	for (std::size_t i = 0; i < TILE_PX; ++i) {
		out.offsets[i] = read_u64(a, i * 8);
		out.counts[i] = read_u64(b, i * 8);
	}
	return out;
}
std::filesystem::path strip_index_cache_path(
		const std::filesystem::path &base, int xt, int yt)
{
	return cache_dir(base) / (quadkey_of(xt, yt) + ".idx");
}
bool save_strip_index_cache(const std::filesystem::path &path, const StripIndex &index)
{
	if (!path.parent_path().empty())
		std::filesystem::create_directories(path.parent_path());
	const auto bytes = index.encode();
	std::ofstream out(path, std::ios::binary);
	if (!out)
		return false;
	out.write(
			reinterpret_cast<const char *>(bytes.data()), std::streamsize(bytes.size()));
	return bool(out);
}
std::optional<StripIndex> load_strip_index_cache(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return std::nullopt;
	in.seekg(0, std::ios::end);
	const auto n = in.tellg();
	if (n <= 0 || n > std::streamoff(2 * 1024 * 1024))
		return std::nullopt;
	in.seekg(0);
	std::vector<std::uint8_t> bytes(static_cast<std::size_t>(n));
	in.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size()));
	if (!in)
		return std::nullopt;
	return StripIndex::decode(bytes);
}

bool inflate_sampled_row(const std::vector<std::uint8_t> &compressed,
		const std::vector<std::size_t> &columns, std::vector<std::uint8_t> &scratch,
		std::vector<std::uint8_t> &out)
{
	if (columns.empty()) {
		out.clear();
		return true;
	}
	if (!std::is_sorted(columns.begin(), columns.end()))
		return false;
	const auto last = columns.back();
	// Predictor 2 requires every byte up to the rightmost requested column;
	// inflate the complete 65,536-byte strip so zlib can finish the stream.
	scratch.assign(TILE_PX, 0);
	z_stream stream{};
	stream.next_in =
			const_cast<Bytef *>(reinterpret_cast<const Bytef *>(compressed.data()));
	stream.avail_in = uInt(compressed.size());
	stream.next_out = reinterpret_cast<Bytef *>(scratch.data());
	stream.avail_out = uInt(scratch.size());
	if (inflateInit(&stream) != Z_OK)
		return false;
	const auto result = inflate(&stream, Z_FINISH);
	inflateEnd(&stream);
	if (result != Z_STREAM_END || stream.total_out <= last)
		return false;
	out.assign(columns.size(), 0);
	std::uint8_t accumulated = 0;
	std::size_t next = 0;
	for (std::size_t i = 0; i <= last; ++i) {
		accumulated = std::uint8_t(accumulated + scratch[i]);
		while (next < columns.size() && columns[next] == i)
			out[next++] = accumulated;
	}
	return next == columns.size();
}
bool read_strip_rows(const std::filesystem::path &path, const StripIndex &index,
		const std::vector<std::size_t> &rows,
		std::vector<std::vector<std::uint8_t>> &compressed_rows)
{
	if (index.offsets.size() != TILE_PX || index.counts.size() != TILE_PX)
		return false;
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return false;
	compressed_rows.clear();
	compressed_rows.reserve(rows.size());
	for (const auto &row : rows) {
		if (row >= TILE_PX || index.counts[row] == 0 ||
				index.counts[row] > 64 * 1024 * 1024)
			return false;
		in.seekg(std::streamoff(index.offsets[row]), std::ios::beg);
		if (!in)
			return false;
		std::vector<std::uint8_t> bytes(static_cast<std::size_t>(index.counts[row]));
		in.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size()));
		if (!in)
			return false;
		compressed_rows.push_back(std::move(bytes));
	}
	return true;
}

TileGeometry tile_geometry(int xt, int yt)
{
	const auto tile_m = 2.0 * MERCATOR_WORLD / double(1u << TILE_ZOOM);
	return {-MERCATOR_WORLD + xt * tile_m, MERCATOR_WORLD - yt * tile_m,
			tile_m / double(TILE_PX)};
}

std::pair<std::size_t, std::vector<std::size_t>> axis_mapping(
		std::size_t count, double low, double step, double origin, double resolution)
{
	std::optional<std::size_t> first;
	std::vector<std::size_t> indices;
	for (std::size_t i = 0; i < count; ++i) {
		const auto p = (low + step * i - origin) / resolution;
		if (p < 0.0 || p >= double(TILE_PX)) {
			if (first)
				break;
			continue;
		}
		if (!first)
			first = i;
		indices.push_back(std::size_t(p));
	}
	return {first.value_or(0), std::move(indices)};
}

TileWindow tile_window(int xt, int yt, double min_lat, double min_lon, double max_lat,
		double max_lon, std::size_t width, std::size_t height)
{
	TileWindow out;
	if (!width || !height)
		return out;
	const auto geometry = tile_geometry(xt, yt);
	const auto x_lo = merc_x(min_lon), x_hi = merc_x(max_lon);
	const auto x_step = width > 1 ? (x_hi - x_lo) / double(width - 1) : 0.0;
	auto xmap = axis_mapping(width, x_lo, x_step, geometry.min_x, geometry.resolution);
	out.grid_x0 = xmap.first;
	out.columns = std::move(xmap.second);
	const auto lat_hi = std::clamp(max_lat, -85.05112878, 85.05112878);
	const auto lat_lo = std::clamp(min_lat, -85.05112878, 85.05112878);
	const auto lat_step = height > 1 ? (lat_lo - lat_hi) / double(height - 1) : 0.0;
	std::optional<std::size_t> first;
	for (std::size_t gz = 0; gz < height; ++gz) {
		const auto p =
				(geometry.max_y - merc_y(lat_hi + lat_step * gz)) / geometry.resolution;
		if (p < 0.0 || p >= double(TILE_PX)) {
			if (first)
				break;
			continue;
		}
		if (!first)
			first = gz;
		out.rows.push_back(std::size_t(p));
	}
	out.grid_z0 = first.value_or(0);
	return out;
}

bool write_sampled_rows(const StripIndex &index, const std::vector<std::size_t> &rows,
		std::size_t grid_x0, std::size_t grid_z0,
		const std::vector<std::vector<std::uint8_t>> &compressed_rows, std::size_t width,
		std::size_t height, std::vector<std::uint8_t> &grid)
{
	if (index.offsets.size() != TILE_PX || index.counts.size() != TILE_PX ||
			grid.size() < width * height || compressed_rows.size() != rows.size())
		return false;
	std::vector<std::size_t> columns;
	// The caller supplies compressed rows already selected by the strip index;
	// columns are reconstructed from the sampled row width.
	for (std::size_t i = 0; i < compressed_rows.size(); ++i) {
		if (grid_z0 + i >= height)
			break;
		if (compressed_rows[i].empty())
			continue;
		columns.resize(width - grid_x0);
		for (std::size_t x = 0; x < columns.size(); ++x)
			columns[x] = x;
		std::vector<std::uint8_t> scratch, values;
		if (!inflate_sampled_row(compressed_rows[i], columns, scratch, values))
			return false;
		for (std::size_t x = 0; x < values.size() && grid_x0 + x < width; ++x)
			grid[(grid_z0 + i) * width + grid_x0 + x] = values[x];
	}
	return true;
}
bool fill_from_cached_tile(const std::filesystem::path &base, int xt, int yt,
		double min_lat, double min_lon, double max_lat, double max_lon, std::size_t width,
		std::size_t height, std::vector<std::uint8_t> &grid)
{
	if (width == 0 || height == 0 || grid.size() < width * height)
		return false;
	const auto path = tile_cache_path(base, xt, yt);
	const auto idxpath = strip_index_cache_path(base, xt, yt);
	auto index = load_strip_index_cache(idxpath);
	if (!index) {
		std::error_code ec;
		std::filesystem::remove(idxpath, ec);
		return false;
	}
	const auto win =
			tile_window(xt, yt, min_lat, min_lon, max_lat, max_lon, width, height);
	if (win.columns.empty() || win.rows.empty())
		return false;
	std::vector<std::vector<std::uint8_t>> rows;
	if (!read_strip_rows(path, *index, win.rows, rows))
		return false;
	for (std::size_t i = 0; i < rows.size(); ++i) {
		std::vector<std::uint8_t> scratch, values;
		if (!inflate_sampled_row(rows[i], win.columns, scratch, values))
			return false;
		const auto gz = win.grid_z0 + i;
		if (gz >= height)
			break;
		for (std::size_t x = 0; x < values.size() && win.grid_x0 + x < width; ++x)
			grid[gz * width + win.grid_x0 + x] = values[x];
	}
	return true;
}
std::optional<CanopyData> assemble_cached_canopy(const std::filesystem::path &base,
		double min_lat, double min_lon, double max_lat, double max_lon, std::size_t width,
		std::size_t height)
{
	if (width == 0 || height == 0)
		return std::nullopt;
	std::vector<std::uint8_t> grid(width * height, CANOPY_NODATA);
	bool any = false;
	for (const auto &[xt, yt] : tiles_for_bbox(min_lat, min_lon, max_lat, max_lon))
		if (fill_from_cached_tile(base, xt, yt, min_lat, min_lon, max_lat, max_lon, width,
					height, grid))
			any = true;
	if (!any)
		return std::nullopt;
	return CanopyData(std::move(grid), width, height);
}

namespace
{
std::optional<StripIndex> cached_or_remote_strip_index(const std::filesystem::path &base,
		int xt, int yt, const RangeFetcher &fetch_range)
{
	const auto cached = load_strip_index_cache(strip_index_cache_path(base, xt, yt));
	if (cached)
		return cached;
	const auto url = tile_url(xt, yt);
	auto header = fetch_range(url, 0, 4096);
	if (!header || header->size() < 16)
		return std::nullopt;
	const auto tables = parse_bigtiff_header(*header);
	if (!tables)
		return std::nullopt;
	auto offsets = fetch_range(url, tables->offsets_at, TILE_PX * 8);
	auto counts = fetch_range(url, tables->counts_at, TILE_PX * 8);
	if (!offsets || !counts)
		return std::nullopt;
	auto index = make_strip_index(*offsets, *counts);
	if (index)
		save_strip_index_cache(strip_index_cache_path(base, xt, yt), *index);
	return index;
}

bool fill_from_remote_tile_unlocked(const std::filesystem::path &base,
		const RangeFetcher &fetch_range, int xt, int yt, double min_lat, double min_lon,
		double max_lat, double max_lon, std::size_t width, std::size_t height,
		std::vector<std::uint8_t> &grid)
{
	const auto win =
			tile_window(xt, yt, min_lat, min_lon, max_lat, max_lon, width, height);
	if (win.columns.empty() || win.rows.empty())
		return false;
	const auto row_prefix = std::filesystem::absolute(cache_dir(base))
								   .lexically_normal().string() + "/" + quadkey_of(xt, yt) + "/";
	// A complete hit avoids index reads, compressed row reads and decompression.
	bool complete = true;
	for (std::size_t i = 0; i < win.rows.size(); ++i) {
		const auto row = decoded_rows().get(row_prefix + std::to_string(win.rows[i]));
		if (!row) {
			complete = false;
			break;
		}
		for (std::size_t x = 0; x < win.columns.size(); ++x)
			grid[(win.grid_z0 + i) * width + win.grid_x0 + x] = (*row)[win.columns[x]];
	}
	if (complete)
		return true;
	const auto index = cached_or_remote_strip_index(base, xt, yt, fetch_range);
	if (!index)
		return false;
	const auto url = tile_url(xt, yt);
	std::vector<std::size_t> unique_rows = win.rows;
	unique_rows.erase(
			std::unique(unique_rows.begin(), unique_rows.end()), unique_rows.end());
	const auto key = quadkey_of(xt, yt);
	auto block_rows = [](std::size_t block) {
		const auto first = block * ROW_BLOCK;
		return std::pair{first, std::min(first + ROW_BLOCK, TILE_PX) - 1};
	};
	auto strip_span = [&](std::size_t first, std::size_t last, std::uint64_t &lo,
							  std::uint64_t &hi) {
		if (first > last || last >= index->offsets.size() || last >= index->counts.size())
			return false;
		lo = index->offsets[first];
		if (index->counts[last] >
				std::numeric_limits<std::uint64_t>::max() - index->offsets[last])
			return false;
		hi = index->offsets[last] + index->counts[last];
		return hi > lo;
	};
	auto row_block_path = [&](std::size_t block) {
		return cache_dir(base) / "rows" / key / (std::to_string(block) + ".strips");
	};
	auto store_row_block = [&](std::size_t block,
								   const std::vector<std::uint8_t> &bytes) {
		const auto path = row_block_path(block);
		std::error_code ec;
		std::filesystem::create_directories(path.parent_path(), ec);
		if (!ec)
			arnis::world_utils::replace_file_atomically(path, bytes);
	};
	auto read_whole = [](const std::filesystem::path &path, std::uint64_t expected_size)
			-> std::optional<std::vector<std::uint8_t>> {
		std::ifstream in(path, std::ios::binary);
		if (!in)
			return std::nullopt;
		in.seekg(0, std::ios::end);
		const auto length = in.tellg();
		if (length < 0 || std::uint64_t(length) != expected_size)
			return std::nullopt;
		in.seekg(0, std::ios::beg);
		std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
		in.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size()));
		return in ? std::optional<std::vector<std::uint8_t>>(std::move(bytes))
				  : std::nullopt;
	};
	auto block_cache_valid = [&](std::size_t block) {
		const auto [first, last] = block_rows(block);
		std::uint64_t lo = 0, hi = 0;
		if (!strip_span(first, last, lo, hi))
			return false;
		std::error_code ec;
		return std::filesystem::file_size(row_block_path(block), ec) == hi - lo && !ec;
	};
	std::vector<std::size_t> blocks;
	for (const auto row : unique_rows) {
		if (row >= TILE_PX)
			return false;
		const auto block = row / ROW_BLOCK;
		if (blocks.empty() || blocks.back() != block)
			blocks.push_back(block);
	}
	std::size_t first = 0, out_i = 0, bi = 0;
	while (bi < blocks.size()) {
		const auto block = blocks[bi];
		const auto [r0, r1] = block_rows(block);
		std::uint64_t lo = 0, hi = 0;
		if (!strip_span(r0, r1, lo, hi))
			return false;
		std::vector<std::uint8_t> blob;
		std::size_t count = 1;
		bool from_cache = false;
		if (block_cache_valid(block)) {
			auto cached = read_whole(row_block_path(block), hi - lo);
			if (cached) {
				blob = std::move(*cached);
				from_cache = true;
			} else {
				std::error_code remove_error;
				std::filesystem::remove(row_block_path(block), remove_error);
			}
		}
		if (!from_cache) {
			const auto range_lo = lo;
			auto range_hi = hi;
			while (bi + count < blocks.size() && blocks[bi + count] == block + count &&
					!block_cache_valid(blocks[bi + count])) {
				const auto [next_first, next_last] = block_rows(blocks[bi + count]);
				std::uint64_t next_lo = 0, next_hi = 0;
				if (!strip_span(next_first, next_last, next_lo, next_hi) ||
						next_hi <= range_hi || next_hi - range_lo > FETCH_BATCH_BYTES)
					break;
				range_hi = next_hi;
				++count;
			}
			auto fetched = fetch_range(url, range_lo, range_hi - range_lo);
			if (!fetched || fetched->size() != range_hi - range_lo)
				return false;
			for (std::size_t n = 0; n < count; ++n) {
				const auto current = blocks[bi + n];
				const auto [first_row, last_row] = block_rows(current);
				std::uint64_t block_lo = 0, block_hi = 0;
				if (!strip_span(first_row, last_row, block_lo, block_hi) ||
						block_lo < range_lo || block_hi > range_hi)
					return false;
				const auto begin = static_cast<std::size_t>(block_lo - range_lo);
				const auto end = static_cast<std::size_t>(block_hi - range_lo);
				const auto block_bytes = std::vector<std::uint8_t>(
						fetched->begin() + begin, fetched->begin() + end);
				store_row_block(current, block_bytes);
			}
			blob = std::move(*fetched);
			lo = range_lo;
		}
		const auto last = block_rows(blocks[bi + count - 1]).second;
		std::size_t end = first;
		while (end < unique_rows.size() && unique_rows[end] <= last)
			++end;
		if (end == first)
			return false;
		auto inflate_batch = [&](const std::vector<std::uint8_t> &bytes)
				-> std::optional<std::vector<std::uint8_t>> {
			std::vector<std::uint8_t> output;
			output.reserve((end - first) * win.columns.size());
			for (std::size_t n = first; n < end; ++n) {
				const auto row = unique_rows[n];
				const auto row_key = row_prefix + std::to_string(row);
				if (const auto cached = decoded_rows().get(row_key)) {
					for (const auto column : win.columns)
						output.push_back((*cached)[column]);
					continue;
				}
				if (index->counts[row] == 0 || index->counts[row] > 64 * 1024 * 1024 ||
						index->offsets[row] < lo)
					return std::nullopt;
				const auto offset = static_cast<std::size_t>(index->offsets[row] - lo);
				if (offset > bytes.size() || index->counts[row] > bytes.size() - offset)
					return std::nullopt;
				const auto length = static_cast<std::size_t>(index->counts[row]);
				std::vector<std::uint8_t> compressed(
						bytes.begin() + offset, bytes.begin() + offset + length);
				std::vector<std::uint8_t> scratch, values;
				if (!inflate_sampled_row(compressed, {TILE_PX - 1}, scratch, values))
					return std::nullopt;
				// TIFF horizontal predictor must be reversed for the entire shared row.
				for (std::size_t x = 1; x < scratch.size(); ++x)
					scratch[x] = std::uint8_t(scratch[x] + scratch[x - 1]);
				const auto decoded = decoded_rows().put(row_key, std::move(scratch));
				for (const auto column : win.columns)
					output.push_back((*decoded)[column]);
			}
			return output;
		};
		auto batch = inflate_batch(blob);
		if (!batch && from_cache) {
			std::error_code remove_error;
			std::filesystem::remove(row_block_path(block), remove_error);
			count = 1;
			const auto [first_row, last_row] = block_rows(block);
			if (!strip_span(first_row, last_row, lo, hi))
				return false;
			auto replacement = fetch_range(url, lo, hi - lo);
			if (!replacement || replacement->size() != hi - lo)
				return false;
			blob = std::move(*replacement);
			end = first;
			while (end < unique_rows.size() && unique_rows[end] <= last_row)
				++end;
			batch = inflate_batch(blob);
			if (!batch)
				return false;
			store_row_block(block, blob);
		}
		if (!batch)
			return false;
		while (out_i < win.rows.size() && win.rows[out_i] <= last) {
			const auto source = std::lower_bound(unique_rows.begin() + first,
					unique_rows.begin() + end, win.rows[out_i]);
			if (source == unique_rows.begin() + end || *source != win.rows[out_i])
				return false;
			const auto row_offset = std::size_t(source - (unique_rows.begin() + first)) *
									win.columns.size();
			const auto gz = win.grid_z0 + out_i;
			if (gz >= height)
				break;
			std::copy_n(batch->begin() + row_offset, win.columns.size(),
					grid.begin() + gz * width + win.grid_x0);
			++out_i;
		}
		first = end;
		bi += count;
	}
	return true;
}

bool fill_from_remote_tile(const std::filesystem::path &base,
		const RangeFetcher &fetch_range, int xt, int yt, double min_lat, double min_lon,
		double max_lat, double max_lon, std::size_t width, std::size_t height,
		std::vector<std::uint8_t> &grid)
{
	const auto window =
			tile_window(xt, yt, min_lat, min_lon, max_lat, max_lon, width, height);
	if (window.columns.empty() || window.rows.empty())
		return false;

	// Concurrent mapgen calls often ask for the same source tile. Serialize only
	// requests for that tile; a successful first caller populates the strip/row
	// caches, while a failed one suppresses repeated HTTP retries briefly.
	const auto state = tile_fetch_state(base, xt, yt);
	std::lock_guard lock(state->mutex);
	const auto now = std::chrono::steady_clock::now();
	if (state->unavailable_until > now)
		return false;
	const bool loaded = fill_from_remote_tile_unlocked(base, fetch_range, xt, yt, min_lat,
			min_lon, max_lat, max_lon, width, height, grid);
	state->unavailable_until =
			loaded ? std::chrono::steady_clock::time_point{} : now + TILE_FAILURE_RETRY;
	return loaded;
}
} // namespace

namespace
{
void prune_row_cache(const std::filesystem::path &base)
{
	const auto root = cache_dir(base) / "rows";
	std::error_code ec;
	const auto root_status = std::filesystem::symlink_status(root, ec);
	if (ec || !std::filesystem::is_directory(root_status))
		return;
	std::filesystem::directory_iterator tile_it(root, ec), end;
	if (ec)
		return;
	const auto now = std::filesystem::file_time_type::clock::now();
	for (; tile_it != end; tile_it.increment(ec)) {
		if (ec) {
			ec.clear();
			continue;
		}
		const auto tile_status = tile_it->symlink_status(ec);
		if (ec || !std::filesystem::is_directory(tile_status)) {
			ec.clear();
			continue;
		}
		std::filesystem::directory_iterator file_it(tile_it->path(), ec), file_end;
		if (ec) {
			ec.clear();
			continue;
		}
		for (; file_it != file_end; file_it.increment(ec)) {
			if (ec) {
				ec.clear();
				continue;
			}
			const auto status = file_it->symlink_status(ec);
			if (ec || !std::filesystem::is_regular_file(status)) {
				ec.clear();
				continue;
			}
			const auto modified = file_it->last_write_time(ec);
			if (!ec && now - modified > ROW_CACHE_MAX_AGE)
				std::filesystem::remove(file_it->path(), ec);
			ec.clear();
		}
	}
}

std::size_t clear_cache_tree(const std::filesystem::path &directory)
{
	std::error_code ec;
	const auto root_status = std::filesystem::symlink_status(directory, ec);
	if (ec || std::filesystem::is_symlink(root_status) ||
			!std::filesystem::is_directory(root_status))
		return 0;
	std::filesystem::directory_iterator it(directory, ec), end;
	if (ec)
		return 0;
	std::size_t removed = 0;
	for (; it != end; it.increment(ec)) {
		if (ec) {
			ec.clear();
			continue;
		}
		const auto path = it->path();
		const auto status = it->symlink_status(ec);
		if (ec) {
			ec.clear();
			continue;
		}
		if (std::filesystem::is_symlink(status)) {
			if (std::filesystem::remove(path, ec))
				++removed;
		} else if (std::filesystem::is_directory(status)) {
			removed += clear_cache_tree(path);
			std::filesystem::remove(path, ec);
		} else if (std::filesystem::is_regular_file(status) &&
				   std::filesystem::remove(path, ec)) {
			++removed;
		}
		ec.clear();
	}
	return removed;
}
} // namespace

bool save_canopy_cache(const std::filesystem::path &path, const CanopyData &data)
{
	if (!path.parent_path().empty())
		std::filesystem::create_directories(path.parent_path());
	auto tmp = path;
	tmp += ".tmp";
	std::ofstream out(tmp, std::ios::binary);
	if (!out)
		return false;
	out.write(reinterpret_cast<const char *>(&data.width), sizeof(data.width));
	out.write(reinterpret_cast<const char *>(&data.height), sizeof(data.height));
	std::vector<std::uint8_t> bytes;
	bytes.reserve(data.width * data.height);
	for (std::size_t z = 0; z < data.height; ++z)
		for (std::size_t x = 0; x < data.width; ++x)
			bytes.push_back(data.at(x, z));
	out.write(
			reinterpret_cast<const char *>(bytes.data()), std::streamsize(bytes.size()));
	if (!out)
		return false;
	out.close();
	std::error_code ec;
	std::filesystem::rename(tmp, path, ec);
	if (ec) {
		std::filesystem::remove(tmp);
		return false;
	}
	return true;
}

std::optional<CanopyData> load_canopy_cache(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return std::nullopt;
	std::size_t width = 0, height = 0;
	in.read(reinterpret_cast<char *>(&width), sizeof(width));
	in.read(reinterpret_cast<char *>(&height), sizeof(height));
	if (!in || width == 0 || height == 0 || width > 4096 || height > 4096)
		return std::nullopt;
	in.seekg(0, std::ios::end);
	const auto file_size = in.tellg();
	const auto expected = std::streamoff(sizeof(width) + sizeof(height)) +
						  std::streamoff(width * height);
	if (file_size != expected)
		return std::nullopt;
	in.seekg(std::streamoff(sizeof(width) + sizeof(height)), std::ios::beg);
	std::vector<std::uint8_t> bytes(width * height);
	in.read(reinterpret_cast<char *>(bytes.data()), std::streamsize(bytes.size()));
	if (!in)
		return std::nullopt;
	return CanopyData(std::move(bytes), width, height);
}
std::filesystem::path cache_dir(const std::filesystem::path &base)
{
	return base.empty() ? std::filesystem::path("./arnis-canopy-cache")
						: base / "arnis-canopy-cache";
}
std::filesystem::path tile_cache_path(const std::filesystem::path &base, int xt, int yt)
{
	return cache_dir(base) / (quadkey_of(xt, yt) + ".canopy");
}
std::string tile_url(int xt, int yt)
{
	return "https://dataforgood-fb-data.s3.amazonaws.com/forests/v1/alsgedi_global_v6_float/chm/" +
		   quadkey_of(xt, yt) + ".tif";
}
std::vector<std::pair<int, int>> tiles_for_bbox(double a, double b, double c, double d)
{
	a = std::clamp(a, -85.05112878, 85.05112878);
	c = std::clamp(c, -85.05112878, 85.05112878);
	auto lo = tile_xy(a, b), hi = tile_xy(c, d);
	std::vector<std::pair<int, int>> out;
	for (int y = std::min(lo.second, hi.second); y <= std::max(lo.second, hi.second); ++y)
		for (int x = std::min(lo.first, hi.first); x <= std::max(lo.first, hi.first); ++x)
			out.emplace_back(x, y);
	return out;
}
bool validate_tile_header(const std::filesystem::path &path)
{
	std::ifstream in(path, std::ios::binary);
	if (!in)
		return false;
	std::vector<std::uint8_t> header(4096);
	in.read(reinterpret_cast<char *>(header.data()), std::streamsize(header.size()));
	header.resize(static_cast<std::size_t>(in.gcount()));
	return parse_bigtiff_header(header).has_value();
}
bool fetch_tile(const std::filesystem::path &base, int xt, int yt)
{
	const auto dst = tile_cache_path(base, xt, yt);
	if (std::filesystem::exists(dst) && std::filesystem::file_size(dst) > 0) {
		if (validate_tile_header(dst))
			return true;
		std::error_code stale_ec;
		std::filesystem::remove(dst, stale_ec);
	}
	if (!dst.parent_path().empty())
		std::filesystem::create_directories(dst.parent_path());
	auto tmp = dst;
	tmp += ".tmp";
	std::error_code ec;
	std::filesystem::remove(tmp, ec);
	if (http_to_file(tile_url(xt, yt), tmp.string()) == 0)
		return false;
	auto size = std::filesystem::file_size(tmp, ec);
	if (ec || size < 16 || !validate_tile_header(tmp)) {
		std::filesystem::remove(tmp);
		return false;
	}
	std::filesystem::rename(tmp, dst, ec);
	if (ec) {
		std::filesystem::remove(tmp);
		return false;
	}
	return true;
}
std::vector<std::pair<int, int>> fetch_tiles_for_bbox(
		const std::filesystem::path &base, double a, double b, double c, double d)
{
	std::vector<std::pair<int, int>> available;
	for (const auto &tile : tiles_for_bbox(a, b, c, d))
		if (fetch_tile(base, tile.first, tile.second))
			available.push_back(tile);
	return available;
}
std::optional<CanopyData> fetch_canopy_data(const std::filesystem::path &base,
		double min_lat, double min_lon, double max_lat, double max_lon,
		std::size_t grid_width, std::size_t grid_height)
{
	// Freeminer's native HTTP helper provides the same strict-206 semantics as
	// Rust's reqwest client.  Keep this convenience overload, but route it
	// through the range-only algorithm so it never falls back to downloading a
	// whole 450 MB canopy TIFF.  Embedders can use fetch_canopy_data_ranges()
	// directly with their own transport.
	const RangeFetcher native_range =
			[](const std::string &url, std::uint64_t offset,
					std::uint64_t length) -> std::optional<std::vector<std::uint8_t>> {
		if (length == 0 ||
				length > std::uint64_t(std::numeric_limits<std::size_t>::max()))
			return std::nullopt;
		const auto body = http_get_range(url, offset, length);
		if (body.size() != length)
			return std::nullopt;
		return std::vector<std::uint8_t>(body.begin(), body.end());
	};
	return fetch_canopy_data_ranges(base, native_range, min_lat, min_lon, max_lat,
			max_lon, grid_width, grid_height);
}
std::optional<CanopyData> fetch_canopy_data_ranges(const std::filesystem::path &base,
		const RangeFetcher &fetch_range, double min_lat, double min_lon, double max_lat,
		double max_lon, std::size_t grid_width, std::size_t grid_height)
{
	if (!fetch_range || grid_width == 0 || grid_height == 0 || !std::isfinite(min_lat) ||
			!std::isfinite(min_lon) || !std::isfinite(max_lat) || !std::isfinite(max_lon))
		return std::nullopt;
	print_canopy_fetch_message_once();
	prune_row_cache(base);
	std::vector<std::uint8_t> grid(grid_width * grid_height, CANOPY_NODATA);
	std::size_t loaded_tiles = 0;
	std::vector<std::string> failures;
	bool log_failure = false;
	for (const auto &[xt, yt] : tiles_for_bbox(min_lat, min_lon, max_lat, max_lon)) {
		if (fill_from_remote_tile(base, fetch_range, xt, yt, min_lat, min_lon, max_lat,
					max_lon, grid_width, grid_height, grid))
			++loaded_tiles;
		else {
			failures.push_back(quadkey_of(xt, yt) + ": canopy tile unavailable");
			log_failure = should_log_canopy_failure() || log_failure;
		}
	}
	CanopyData data(std::move(grid), grid_width, grid_height);
	const auto [covered, canopy, mean, maximum] = data.stats();
	if (covered == 0) {
		const auto why = failures.empty() ? std::string("no tiles") : failures.front();
		if (log_failure)
			std::clog << "Warning: Canopy height data unavailable (" << why.substr(0, 200)
					  << "). Falling back to land cover for trees.\n";
		return std::nullopt;
	}
	const auto cout_flags = std::cout.flags();
	const auto cout_precision = std::cout.precision();
	std::cout << "Canopy height data loaded: " << loaded_tiles << " tile"
			  << (loaded_tiles == 1 ? "" : "s") << ", canopy over "
			  << static_cast<unsigned>(CANOPY_MIN_M) << "m on " << std::fixed
			  << std::setprecision(1) << (100.0 * static_cast<double>(canopy) / covered)
			  << "% of the area, mean " << mean << "m, tallest "
			  << static_cast<unsigned>(maximum) << "m\n";
	std::cout.flags(cout_flags);
	std::cout.precision(cout_precision);
	if (!failures.empty() && log_failure)
		std::clog
				<< "Warning: " << failures.size()
				<< " canopy tile(s) unavailable; those areas fall back to land cover.\n";
	return std::optional<CanopyData>(std::move(data));
}
std::size_t clear_canopy_cache(const std::filesystem::path &base)
{
	decoded_rows().clear();
	const auto dir = cache_dir(base);
	return clear_cache_tree(dir);
}
bool canopy_cache_fresh(const std::filesystem::path &path, std::chrono::seconds age)
{
	if (age.count() <= 0 || !std::filesystem::exists(path))
		return false;
	return std::filesystem::file_time_type::clock::now() -
				   std::filesystem::last_write_time(path) <=
		   age;
}
} // namespace arnis::canopy
