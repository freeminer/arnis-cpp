#pragma once
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <array>
#include <utility>
#include "elevation.h"
#include "providers.h"
namespace arnis::land_cover
{
struct LandCoverData;
}
namespace arnis::elevation
{
struct LandCoverRepairConfig
{
	land_cover::LandCoverData *data{nullptr};
	double bbox_width_m{0.0};
	double bbox_height_m{0.0};
	double built_up_sigma_m{30.0};
	double coastal_pull_m{25.0};
	std::function<void(double)> report;
	explicit operator bool() const { return data != nullptr; }
};
struct Tile
{
	int lat = 0, lon = 0;
	std::filesystem::path file;
};
struct TileResult
{
	Tile tile;
	providers::Source source{providers::Source::None};
};
class CachedProvider;
std::optional<std::vector<std::int16_t>> read_hgt(const Tile &tile);
double sample_hgt(const std::vector<std::int16_t> &, std::size_t side, double lat,
		double lon, const Tile &tile);
std::optional<double> sample_tile(const Tile &, double lat, double lon);
struct SampleResult
{
	double height = 0.0;
	providers::Source source{providers::Source::None};
};
std::optional<SampleResult> sample_with_source(CachedProvider &, double lat, double lon);
struct SourceGrid
{
	ElevationData data;
	std::vector<std::vector<providers::Source>> sources;
};
SourceGrid build_source_grid(CachedProvider &, double min_lat, double min_lon,
		double max_lat, double max_lon, std::size_t width, std::size_t height);
std::array<std::size_t, 5> source_counts(const SourceGrid &);
double source_coverage(const SourceGrid &);
bool acceptable_source_coverage(const SourceGrid &, double minimum);
void fill_source_gaps(SourceGrid &, double fallback = 0.0);
void process_source_grid(SourceGrid &, double sigma, double fallback = 0.0);
void normalize_source_grid(SourceGrid &, double sea, double scale, double lo, double hi);
SourceGrid resample_source_grid(
		const SourceGrid &, std::size_t width, std::size_t height);
std::optional<double> sample_tiles(const std::vector<Tile> &, double lat, double lon);
ElevationData build_grid(const std::vector<Tile> &, double min_lat, double min_lon,
		double max_lat, double max_lon, std::size_t width, std::size_t height);
ElevationData build_processed_grid(const std::vector<Tile> &, double min_lat,
		double min_lon, double max_lat, double max_lon, std::size_t width,
		std::size_t height, const LandCoverRepairConfig &repair = {});
void normalize_grid(ElevationData &, double sea_level, double scale, double min_height,
		double max_height);
ElevationData resample_grid(const ElevationData &, std::size_t width, std::size_t height);
ElevationData smooth_grid(const ElevationData &, double sigma);
std::vector<std::vector<double>> gradient_grid(const ElevationData &);
std::pair<double, double> elevation_range(const ElevationData &);
ElevationData slope_normalized(const ElevationData &, double max_slope);
enum class TerrainClass
{
	Flat,
	Rolling,
	Steep,
	Cliff
};
std::vector<std::vector<TerrainClass>> classify_terrain(
		const ElevationData &, double rolling, double steep, double cliff);
std::vector<std::vector<std::uint8_t>> basin_mask(const ElevationData &, double drop);
std::vector<std::vector<std::uint8_t>> ridge_mask(const ElevationData &, double rise);
std::vector<std::pair<std::size_t, std::size_t>> local_minima(
		const ElevationData &, double prominence);
std::vector<std::pair<std::size_t, std::size_t>> local_maxima(
		const ElevationData &, double prominence);
std::vector<std::vector<int>> watershed_labels(const ElevationData &, double prominence);
std::vector<std::vector<std::pair<std::size_t, std::size_t>>> contours(
		const ElevationData &, double interval);
std::vector<std::int16_t> encode_heightmap(
		const ElevationData &, double scale = 1.0, double offset = 0.0);
ElevationData decode_heightmap(const std::vector<std::int16_t> &, std::size_t width,
		std::size_t height, double scale = 1.0, double offset = 0.0);
class Provider
{
public:
	virtual ~Provider() = default;
	virtual std::optional<Tile> tile_for(double lat, double lon) = 0;
};
class CachedProvider : public Provider
{
	std::filesystem::path root_;

public:
	explicit CachedProvider(std::filesystem::path root) : root_(std::move(root)) {}
	const std::filesystem::path &root() const { return root_; }
	std::optional<Tile> tile_for(double lat, double lon) override;
	providers::Source source_for(double lat, double lon) const;
	std::optional<TileResult> fetch_tile(double lat, double lon);
};
std::vector<Tile> tiles_for_bbox(double min_lat, double min_lon, double max_lat,
		double max_lon, const std::filesystem::path &root);
}
