#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <tuple>
#include <filesystem>
#include <functional>
namespace arnis
{
class ProcessedElement;
}
struct XZBBox;

namespace arnis::land_cover
{
enum class Source
{
	OSMOverride,
	ESAWorldCover
};
std::string esa_tile_url(int tile_lat, int tile_lng);
std::vector<std::tuple<int, int, std::string>> esa_tiles_for_bbox(
		double min_lat, double min_lng, double max_lat, double max_lng);
bool fetch_esa_tile(const std::string &url, const std::filesystem::path &cache_file);
// Removes cached ESA tiles and returns the number of files removed.
std::size_t clear_land_cover_cache(const std::filesystem::path &cache_dir);

inline constexpr uint8_t LC_TREE_COVER = 10;
inline constexpr uint8_t LC_SHRUBLAND = 20;
inline constexpr uint8_t LC_GRASSLAND = 30;
inline constexpr uint8_t LC_CROPLAND = 40;
inline constexpr uint8_t LC_BUILT_UP = 50;
inline constexpr uint8_t LC_BARE = 60;
inline constexpr uint8_t LC_SNOW_ICE = 70;
inline constexpr uint8_t LC_WATER = 80;
inline constexpr uint8_t LC_WETLAND = 90;
inline constexpr uint8_t LC_MANGROVES = 95;
inline constexpr uint8_t LC_MOSS = 100;

struct LandCoverData
{
	std::vector<std::vector<uint8_t>> grid;
	std::vector<std::vector<uint8_t>> water_distance;
	std::vector<std::vector<float>> water_blend_grid;
	std::size_t width{0};
	std::size_t height{0};

	void refresh_water_blend_grid();
};

// A decoded ESA WorldCover tile.  Pixels are row-major, with (0, 0) at the
// north-west corner, exactly as in the COG.  Keeping decoding separate from
// assembly lets embedders use their own HTTP/cache/TIFF implementation.
struct EsaRasterTile
{
	int south_lat{0};
	int west_lng{0};
	std::size_t width{0};
	std::size_t height{0};
	std::vector<uint8_t> pixels;

	bool valid() const
	{
		return width != 0 && height != 0 && pixels.size() >= width * height;
	}
	bool contains(double lat, double lng) const
	{
		return lat >= south_lat && lat <= south_lat + 3 && lng >= west_lng &&
			   lng <= west_lng + 3;
	}
	uint8_t sample(double lat, double lng) const;
};

// Geographic extent used by the library-facing ESA assembler.  Coordinates
// are WGS84 degrees and must be ordered south/west to north/east.
struct GeographicBounds
{
	double min_lat{0.0};
	double min_lng{0.0};
	double max_lat{0.0};
	double max_lng{0.0};
	bool valid() const { return min_lat <= max_lat && min_lng <= max_lng; }
};

// Assemble an elevation-aligned classification grid from decoded ESA rasters.
// Missing/no-data pixels remain zero until `fill_land_cover_gaps`; an empty
// `LandCoverData` is returned when no valid ESA class covered the extent.
LandCoverData assemble_land_cover_data(const GeographicBounds &bbox,
		std::size_t grid_width, std::size_t grid_height,
		const std::vector<EsaRasterTile> &tiles, bool smooth_boundaries = true);

// Rust-equivalent remote ESA provider. It uses HTTP ranges and therefore never
// downloads an entire WorldCover COG; failures of individual tiles are
// tolerated and yield an empty result only when no tile contributes a class.
LandCoverData fetch_land_cover_data(const GeographicBounds &bbox, std::size_t grid_width,
		std::size_t grid_height, bool smooth_boundaries = true);

// Public equivalents of the Rust post-read cleanup stages.  They are useful
// when a provider streams or incrementally updates a classification grid.
void fill_land_cover_gaps(
		std::vector<std::vector<uint8_t>> &grid, std::size_t width, std::size_t height);
void smooth_land_cover_boundaries(
		std::vector<std::vector<uint8_t>> &grid, std::size_t width, std::size_t height);

uint64_t coord_hash(int32_t x, int32_t z);

std::vector<std::vector<uint8_t>> compute_water_distance(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height);

std::vector<std::vector<float>> compute_water_blend_smooth(
		const std::vector<std::vector<uint8_t>> &grid, std::size_t width,
		std::size_t height);

// Elevation-aware OSM-water guard, ported from osm_water_override.rs.  A
// compact/non-linear land component is protected from line/polygon water
// overrides; the nearest-water elevation grid rejects implausible uphill
// flooding around genuine waterways.
struct WaterOverrideGuard
{
	std::vector<std::uint64_t> protected_mask;
	std::vector<std::vector<float>> nearest_water_y;
	std::size_t width{0};
	bool has_elevation{false};
	bool allows(const std::vector<std::vector<float>> &heights, std::size_t x,
			std::size_t z, float tolerance = 1.5f) const;
};
WaterOverrideGuard build_water_override_guard(
		const std::vector<std::vector<uint8_t>> &grid,
		const std::vector<std::vector<float>> &heights, double cells_per_m2);

void apply_bridge_land_cover_repair(LandCoverData &data,
		const std::vector<ProcessedElement> &elements, const XZBBox &bbox, double scale);
void apply_osm_water_override(LandCoverData &data,
		const std::vector<ProcessedElement> &elements, const XZBBox &bbox);

}
