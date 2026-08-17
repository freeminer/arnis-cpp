#include "canopy.h"

#include <algorithm>
#include <cmath>
#include <zlib.h>
#include <fstream>
#include <chrono>
#include "../../../http.h"

namespace arnis::canopy
{
namespace
{
constexpr double MERCATOR_WORLD = 20037508.342789244;
constexpr double COLUMNS_PER_TREE = 52.0;
constexpr double COLUMNS_PER_TREE_LEGACY = 25.0;
constexpr double SLOT_P_MAX = 0.85;
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
	for (const auto h : grid_) {
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
	for (const auto row : rows) {
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
	for (const auto [xt, yt] : tiles_for_bbox(min_lat, min_lon, max_lat, max_lon))
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

bool fill_from_remote_tile(const std::filesystem::path &base,
		const RangeFetcher &fetch_range, int xt, int yt, double min_lat, double min_lon,
		double max_lat, double max_lon, std::size_t width, std::size_t height,
		std::vector<std::uint8_t> &grid)
{
	const auto win =
			tile_window(xt, yt, min_lat, min_lon, max_lat, max_lon, width, height);
	if (win.columns.empty() || win.rows.empty())
		return false;
	const auto index = cached_or_remote_strip_index(base, xt, yt, fetch_range);
	if (!index)
		return false;
	const auto url = tile_url(xt, yt);
	std::vector<std::size_t> unique_rows = win.rows;
	unique_rows.erase(
			std::unique(unique_rows.begin(), unique_rows.end()), unique_rows.end());
	std::vector<std::vector<std::uint8_t>> values(unique_rows.size());
	for (std::size_t i = 0; i < unique_rows.size(); ++i) {
		const auto row = unique_rows[i];
		if (row >= TILE_PX || index->counts[row] == 0 ||
				index->counts[row] > 64 * 1024 * 1024)
			return false;
		auto compressed = fetch_range(url, index->offsets[row], index->counts[row]);
		if (!compressed || compressed->size() != index->counts[row])
			return false;
		std::vector<std::uint8_t> scratch;
		if (!inflate_sampled_row(*compressed, win.columns, scratch, values[i]))
			return false;
	}
	for (std::size_t out_row = 0; out_row < win.rows.size(); ++out_row) {
		const auto found = std::lower_bound(
				unique_rows.begin(), unique_rows.end(), win.rows[out_row]);
		if (found == unique_rows.end() || *found != win.rows[out_row])
			return false;
		const auto gz = win.grid_z0 + out_row;
		if (gz >= height)
			break;
		const auto &row = values[std::size_t(found - unique_rows.begin())];
		for (std::size_t x = 0; x < row.size() && win.grid_x0 + x < width; ++x)
			grid[gz * width + win.grid_x0 + x] = row[x];
	}
	return true;
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
	for (const auto tile : tiles_for_bbox(a, b, c, d))
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
	std::vector<std::uint8_t> grid(grid_width * grid_height, CANOPY_NODATA);
	for (const auto [xt, yt] : tiles_for_bbox(min_lat, min_lon, max_lat, max_lon))
		fill_from_remote_tile(base, fetch_range, xt, yt, min_lat, min_lon, max_lat,
				max_lon, grid_width, grid_height, grid);
	CanopyData data(std::move(grid), grid_width, grid_height);
	const auto [covered, canopy, mean, maximum] = data.stats();
	(void)canopy;
	(void)mean;
	(void)maximum;
	return covered ? std::optional<CanopyData>(std::move(data)) : std::nullopt;
}
std::size_t clear_canopy_cache(const std::filesystem::path &base)
{
	const auto dir = cache_dir(base);
	if (!std::filesystem::exists(dir))
		return 0;
	std::size_t removed = 0;
	for (const auto &entry : std::filesystem::directory_iterator(dir)) {
		const auto ext = entry.path().extension();
		if (ext == ".canopy" || ext == ".idx" || ext == ".tmp") {
			std::error_code ec;
			std::filesystem::remove(entry.path(), ec);
			if (!ec)
				++removed;
		}
	}
	return removed;
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
