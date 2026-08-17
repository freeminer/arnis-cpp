#pragma once
#include <string>
#include <filesystem>
#include <cstdint>
#include <optional>
#include <vector>
#include <array>
namespace arnis::elevation::providers
{
enum class Source;
struct GeoBBox
{
	double min_lat = 0, min_lon = 0, max_lat = 0, max_lon = 0;
};
bool bboxes_overlap(const GeoBBox &, const GeoBBox &);
bool usgs_3dep_covers(const GeoBBox &);
enum class SourceMode
{
	Auto,
	GlobalOnly,
	AwsOnly
};
// Ordered fetch plan.  Auto deliberately exposes both the regional primary
// and global fallbacks, mirroring Rust's fetch-time fallback chain.
std::vector<Source> select_sources(const GeoBBox &, SourceMode = SourceMode::Auto);
constexpr double MERCATOR_LIMIT = 20037508.342789244;
constexpr double MERCATOR_LAT_LIMIT = 85.05112878;
constexpr double MERCATOR_RADIUS_M = 6378137.0;
double lon_to_mercator_x(double lon);
double lat_to_mercator_y(double lat);
double mercator_x_to_lon(double x);
double mercator_y_to_lat(double y);
struct FixedTileKey
{
	int level_m = 1, tile_x = 0, tile_y = 0;
	double span_m() const;
	double min_mx() const;
	double max_mx() const;
	double min_my() const;
	double max_my() const;
	std::filesystem::path cache_path(const std::filesystem::path &) const;
};
// USGS publishes these four native pixel resolutions.  Keep the named
// variants rather than treating them as arbitrary integers: the names are
// part of the stable on-disk cache layout used by the Rust implementation.
enum class UsgsResolution
{
	M1,
	M3,
	M10,
	M30
};
double meters_per_pixel(UsgsResolution);
const char *resolution_id(UsgsResolution);
struct UsgsFixedTileKey
{
	UsgsResolution level{UsgsResolution::M1};
	int tile_x = 0, tile_y = 0;
	double span_m() const;
	double min_mx() const;
	double max_mx() const;
	double max_my() const;
	double min_my() const;
	std::filesystem::path cache_path(const std::filesystem::path &) const;
};
UsgsFixedTileKey usgs_tile_for_mercator(UsgsResolution, double mx, double my);
std::vector<UsgsFixedTileKey> covering_usgs_tiles(const GeoBBox &, UsgsResolution);
FixedTileKey fixed_tile_for_mercator(UsgsResolution, double mx, double my);
std::vector<FixedTileKey> covering_fixed_tiles(const GeoBBox &, UsgsResolution);
UsgsResolution choose_usgs_resolution(
		const GeoBBox &, std::size_t grid_width, std::size_t grid_height);
std::string usgs_3dep_tile_url(const FixedTileKey &);
std::string usgs_3dep_tile_url(const UsgsFixedTileKey &);
FixedTileKey fixed_tile_for_mercator(int level_m, double mx, double my);
std::vector<FixedTileKey> covering_fixed_tiles(const GeoBBox &, int level_m);
struct XyzTileKey
{
	unsigned zoom = 0, x = 0, y = 0;
};
XyzTileKey xyz_tile_at(double lat, double lon, unsigned zoom);
std::vector<XyzTileKey> covering_xyz_tiles(const GeoBBox &, unsigned zoom);
std::optional<XyzTileKey> xyz_parent(XyzTileKey);
std::string aws_terrarium_tile_url(const XyzTileKey &);
std::string mapterhorn_tile_url(const XyzTileKey &);
// Mapterhorn's 512px pyramid level selection: preserves the Rust tile budget
// and latitude-aware ground-resolution calculation for any image client.
unsigned choose_mapterhorn_zoom(const GeoBBox &, std::size_t grid_width,
		std::size_t grid_height, unsigned min_zoom = 6, unsigned max_zoom = 17,
		std::size_t tile_budget = 2048);
// Provider-neutral pieces of AWS/Mapterhorn Terrarium decoding.  Image and
// HTTP backends only need to supply decoded RGB pixels to these routines.
struct RgbRaster
{
	std::size_t width = 0, height = 0;
	std::vector<std::array<std::uint8_t, 3>> pixels;
};
std::optional<double> terrarium_height(std::array<std::uint8_t, 3> rgb);
std::optional<double> sample_terrarium_pixel(const RgbRaster &, int x, int y);
std::optional<double> sample_terrarium_bilinear(const RgbRaster &, double x, double y);
// Equivalent to fixed_tile.rs::blend_finite_samples: preserve available
// neighbours instead of turning a partly missing raster into a NaN seam.
double blend_finite_samples(
		double v00, double v10, double v01, double v11, double dx, double dy);
std::optional<double> sample_fixed_tile_bilinear(const std::vector<std::vector<double>> &,
		const FixedTileKey &, double mercator_x, double mercator_y);
std::optional<double> sample_usgs_tile_bilinear(const std::vector<std::vector<double>> &,
		const UsgsFixedTileKey &, double mercator_x, double mercator_y);
std::vector<std::vector<double>> resample_raster_nearest(
		const std::vector<double> &source, std::size_t source_width,
		std::size_t target_width, std::size_t target_height, double nodata = -9999.0);
std::string aws_terrain_url(int lat, int lon);
std::string usgs_3dep_url(double lat, double lon);
std::string mapterhorn_url(double lat, double lon);
bool download_tile(const std::string &url, const std::filesystem::path &file);
bool fetch_with_fallback(int lat, int lon, const std::filesystem::path &file);
void set_download_retries(unsigned retries);
void enable_aws(bool enabled);
void enable_mapterhorn(bool enabled);
void enable_usgs(bool enabled);
struct ProviderConfig
{
	bool aws, usgs, mapterhorn;
	unsigned retries;
};
ProviderConfig config();
enum class Source
{
	AWS,
	USGS,
	Mapterhorn,
	Fixed,
	None
};
Source select_source(int lat, int lon, const std::filesystem::path &file);
const char *source_name(Source);
struct ProviderStats
{
	std::uint64_t attempts = 0, successes = 0, aws_successes = 0, usgs_successes = 0,
				  mapterhorn_successes = 0;
};
ProviderStats stats();
void reset_stats();
void set_fixed_height(std::optional<double> height);
void enable_fixed(bool enabled);
void reset_config();
}
