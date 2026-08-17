#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <optional>
#include <vector>
#include <filesystem>
#include <chrono>
#include <functional>

namespace arnis::canopy
{

inline constexpr std::uint8_t CANOPY_NODATA = 255;
inline constexpr std::uint8_t CANOPY_MIN_M = 3;
inline constexpr unsigned TILE_ZOOM = 9;
inline constexpr std::size_t TILE_PX = 65536;

class CanopyData
{
	std::vector<std::uint8_t> grid_;

public:
	std::size_t width = 0, height = 0;
	CanopyData() = default;
	CanopyData(std::vector<std::uint8_t> grid, std::size_t width, std::size_t height);
	std::uint8_t at(std::size_t gx, std::size_t gz) const;
	std::optional<std::uint8_t> canopy_height_m(std::size_t gx, std::size_t gz) const;
	std::optional<double> canopy_fraction(
			std::size_t gx, std::size_t gz, int spacing) const;
	std::tuple<std::size_t, std::size_t, double, std::uint8_t> stats() const;
};
bool save_canopy_cache(const std::filesystem::path &, const CanopyData &);
std::optional<CanopyData> load_canopy_cache(const std::filesystem::path &);
std::filesystem::path cache_dir(const std::filesystem::path &base = {});
std::filesystem::path tile_cache_path(const std::filesystem::path &base, int xt, int yt);
std::string tile_url(int xt, int yt);
std::vector<std::pair<int, int>> tiles_for_bbox(
		double min_lat, double min_lon, double max_lat, double max_lon);
bool fetch_tile(const std::filesystem::path &base, int xt, int yt);
std::vector<std::pair<int, int>> fetch_tiles_for_bbox(const std::filesystem::path &base,
		double min_lat, double min_lon, double max_lat, double max_lon);
// Cache-first canopy acquisition matching Rust's fetch_canopy_data contract.
// Callers opt in explicitly; failure or an all-no-data response returns nullopt
// so terrain generation falls back to land-cover tree density.
std::optional<CanopyData> fetch_canopy_data(const std::filesystem::path &base,
		double min_lat, double min_lon, double max_lat, double max_lon,
		std::size_t grid_width, std::size_t grid_height);
// The embedding application supplies HTTP range requests; this avoids loading
// complete canopy GeoTIFFs and leaves networking outside the mapgen library.
using RangeFetcher = std::function<std::optional<std::vector<std::uint8_t>>(
		const std::string &url, std::uint64_t offset, std::uint64_t length)>;
std::optional<CanopyData> fetch_canopy_data_ranges(const std::filesystem::path &base,
		const RangeFetcher &fetch_range, double min_lat, double min_lon, double max_lat,
		double max_lon, std::size_t grid_width, std::size_t grid_height);
std::size_t clear_canopy_cache(const std::filesystem::path &base = {});
bool canopy_cache_fresh(const std::filesystem::path &, std::chrono::seconds max_age);

double slot_probability(double canopy_fraction, int spacing, bool schematic_pack);
std::pair<int, int> tile_xy(double lat, double lon);
std::string quadkey_of(int xt, int yt);
double merc_x(double lon);
double merc_y(double lat);

struct StripIndex
{
	std::vector<std::uint64_t> offsets, counts;
	std::vector<std::uint8_t> encode() const;
	static std::optional<StripIndex> decode(const std::vector<std::uint8_t> &);
};
std::filesystem::path strip_index_cache_path(
		const std::filesystem::path &, int xt, int yt);
bool save_strip_index_cache(const std::filesystem::path &, const StripIndex &);
std::optional<StripIndex> load_strip_index_cache(const std::filesystem::path &);
struct StripTableLocation
{
	std::uint64_t offsets_at = 0, counts_at = 0;
};
std::optional<StripTableLocation> parse_bigtiff_header(
		const std::vector<std::uint8_t> &header);
bool validate_tile_header(const std::filesystem::path &path);
std::optional<StripIndex> make_strip_index(const std::vector<std::uint8_t> &offsets,
		const std::vector<std::uint8_t> &counts);
bool inflate_sampled_row(const std::vector<std::uint8_t> &compressed,
		const std::vector<std::size_t> &columns, std::vector<std::uint8_t> &scratch,
		std::vector<std::uint8_t> &out);
bool read_strip_rows(const std::filesystem::path &, const StripIndex &,
		const std::vector<std::size_t> &rows,
		std::vector<std::vector<std::uint8_t>> &compressed_rows);

struct TileGeometry
{
	double min_x = 0.0, max_y = 0.0, resolution = 0.0;
};
TileGeometry tile_geometry(int xt, int yt);
std::pair<std::size_t, std::vector<std::size_t>> axis_mapping(
		std::size_t count, double low, double step, double origin, double resolution);
struct TileWindow
{
	std::size_t grid_x0 = 0, grid_z0 = 0;
	std::vector<std::size_t> columns, rows;
};
TileWindow tile_window(int xt, int yt, double min_lat, double min_lon, double max_lat,
		double max_lon, std::size_t grid_width, std::size_t grid_height);
bool write_sampled_rows(const StripIndex &, const std::vector<std::size_t> &rows,
		std::size_t grid_x0, std::size_t grid_z0,
		const std::vector<std::vector<std::uint8_t>> &compressed_rows,
		std::size_t grid_width, std::size_t grid_height, std::vector<std::uint8_t> &grid);
bool fill_from_cached_tile(const std::filesystem::path &, int xt, int yt, double min_lat,
		double min_lon, double max_lat, double max_lon, std::size_t grid_width,
		std::size_t grid_height, std::vector<std::uint8_t> &grid);
std::optional<CanopyData> assemble_cached_canopy(const std::filesystem::path &,
		double min_lat, double min_lon, double max_lat, double max_lon,
		std::size_t grid_width, std::size_t grid_height);

} // namespace arnis::canopy
