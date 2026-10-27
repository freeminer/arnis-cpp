#pragma once

#include "elevation.h"
#include "peaks.h"
#include "postprocess.h"

#include <functional>
#include <memory>
#include <optional>
#include <string_view>

namespace arnis::geographic
{
class LLBBox;
}

namespace arnis::land_cover
{
struct LandCoverData;
}

namespace arnis::elevation
{

class Selector;

struct HeightBand
{
	int ground_level{};
	int min_ground_level{};
	int extended_max_y{};
	bool extended{};
};

// Terrain samples kept outside a cropped One World area so geometry crossing
// its edge queries the same elevation as the neighboring area.
struct ElevationHalo
{
	std::size_t x0{}, z0{};
	std::size_t inner_width{}, inner_height{};
	std::size_t outer_width{}, outer_height{};
	std::vector<std::vector<float>> rows;

	static ElevationHalo cut(const std::vector<std::vector<float>> &grid, std::size_t x0,
			std::size_t z0, std::size_t width, std::size_t height);
	std::optional<float> at(std::int32_t x, std::int32_t z) const;
};

struct ProcessedElevationData
{
	std::vector<std::vector<float>> heights;
	std::size_t width{0};
	std::size_t height{0};
	std::size_t world_width{0};
	std::size_t world_height{0};
	double min_height_m{0.0};
	double blocks_per_meter{0.0};
	double slope_correction{1.0};
	int ground_level{0};
	std::optional<SoftTop> soft_top;
	bool extended_ceiling{false};
	std::shared_ptr<const ElevationHalo> halo;

	ElevationAffine affine() const
	{
		return {min_height_m, blocks_per_meter, ground_level, soft_top};
	}
	std::optional<int> lowest_y() const;
};

using ProgressCallback = std::function<void(double, std::string_view)>;

// Rust-equivalent shared post-processing stage. `raw` is in metres and retains
// non-finite sentinels until filtering/filling is complete. Land cover is
// repaired in place alongside its elevation-dependent water classification.
ProcessedElevationData process_elevation_data(const geographic::LLBBox &bbox,
		ElevationData raw, double scale, double height_multiplier, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		bool apply_earth_repairs, land_cover::LandCoverData *land_cover,
		const AffinePolicy &affine = {}, const ProgressCallback &progress = {},
		TerrainHints terrain_hints = {}, bool auto_height = false,
		std::optional<HeightBand> taller_band = std::nullopt,
		bool force_extended_height = false,
		std::optional<std::size_t> seam_pad = std::nullopt,
		const std::function<std::vector<TallModel>()> &late_overture = {});

ProcessedElevationData fetch_elevation_data(Selector &selector,
		const geographic::LLBBox &bbox, std::size_t world_width, std::size_t world_height,
		std::size_t grid_width, std::size_t grid_height, double scale,
		double height_multiplier, int ground_level, int min_ground_level,
		bool disable_height_limit, int extended_max_y, bool apply_earth_repairs,
		land_cover::LandCoverData *land_cover, const AffinePolicy &affine = {},
		const ProgressCallback &progress = {}, TerrainHints terrain_hints = {},
		bool auto_height = false, std::optional<HeightBand> taller_band = std::nullopt,
		bool force_extended_height = false,
		std::optional<std::size_t> seam_pad = std::nullopt,
		const std::function<std::vector<TallModel>()> &late_overture = {});

} // namespace arnis::elevation
