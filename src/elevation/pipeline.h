#pragma once

#include "elevation.h"
#include "postprocess.h"

#include <functional>
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
		const AffinePolicy &affine = {}, const ProgressCallback &progress = {});

ProcessedElevationData fetch_elevation_data(Selector &selector,
		const geographic::LLBBox &bbox, std::size_t world_width, std::size_t world_height,
		std::size_t grid_width, std::size_t grid_height, double scale,
		double height_multiplier, int ground_level, int min_ground_level,
		bool disable_height_limit, int extended_max_y, bool apply_earth_repairs,
		land_cover::LandCoverData *land_cover, const AffinePolicy &affine = {},
		const ProgressCallback &progress = {});

} // namespace arnis::elevation
