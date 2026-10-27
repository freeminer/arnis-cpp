#include "pipeline.h"

#include "../coordinate_system/geographic/llbbox.h"
#include "../coordinate_system/transformation.h"
#include "../land_cover/land_cover.h"
#include "../world_editor/floor_state.h"
#include "selector.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace arnis::elevation
{
namespace
{
constexpr double BUILT_UP_SIGMA_M = 30.0;
constexpr double COASTAL_PULL_M = 25.0;

void report(const ProgressCallback &callback, double percent, std::string_view message)
{
	if (callback)
		callback(percent, message);
}

std::uint32_t saturating_u32(double value)
{
	if (!std::isfinite(value) || value <= 0.0)
		return 0;
	if (value >= std::numeric_limits<std::uint32_t>::max())
		return std::numeric_limits<std::uint32_t>::max();
	return static_cast<std::uint32_t>(value);
}
} // namespace

std::optional<int> ProcessedElevationData::lowest_y() const
{
	std::optional<int> lowest;
	for (const auto &row : heights)
		for (const float value : row)
			if (std::isfinite(value)) {
				const double floored = std::floor(static_cast<double>(value));
				const int y = floored <= std::numeric_limits<int>::min()
									  ? std::numeric_limits<int>::min()
							  : floored >= std::numeric_limits<int>::max()
									  ? std::numeric_limits<int>::max()
									  : static_cast<int>(floored);
				lowest = lowest ? std::min(*lowest, y) : y;
			}
	return lowest;
}

ProcessedElevationData process_elevation_data(const geographic::LLBBox &bbox,
		ElevationData raw, double scale, double height_multiplier, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		bool apply_earth_repairs, land_cover::LandCoverData *land_cover,
		const AffinePolicy &policy, const ProgressCallback &progress,
		const TerrainHints &terrain_hints)
{
	const auto grid_height = raw.heights.size();
	const auto grid_width = grid_height ? raw.heights.front().size() : 0;
	const auto world_width = raw.world_width ? raw.world_width : grid_width;
	const auto world_height = raw.world_height ? raw.world_height : grid_height;
	raw.width = grid_width;
	raw.height = grid_height;
	raw.world_width = world_width;
	raw.world_height = world_height;

	const auto [bbox_height_m, bbox_width_m] =
			coordinate_system::geo_distance(bbox.min(), bbox.max());
	const double meters_per_cell =
			grid_width && grid_height
					? (bbox_width_m / static_cast<double>(grid_width) +
							  bbox_height_m / static_cast<double>(grid_height)) *
							  0.5
					: 0.0;

	report(progress, 12.0, "Processing elevation...");
	if (apply_earth_repairs) {
		filter_elevation_outliers(raw.heights);
		repair_terrain_anomalies(raw.heights, meters_per_cell);
	}
	fill_nan_values(raw.heights);
	report(progress, 14.0, "Processing elevation...");

	if (land_cover) {
		const double sigma_cells =
				meters_per_cell > 0.0 ? BUILT_UP_SIGMA_M / meters_per_cell : 0.0;
		const auto coastal_cells =
				meters_per_cell > 0.0
						? saturating_u32(std::round(COASTAL_PULL_M / meters_per_cell))
						: 0;
		const std::function<void(double)> subprogress =
				progress ? std::function<void(double)>([&](double fraction) {
					report(progress, 14.0 + std::clamp(fraction, 0.0, 1.0) * 2.0,
							"Processing elevation...");
				})
						 : std::function<void(double)>{};
		apply_land_cover_repair(raw.heights, *land_cover, sigma_cells, coastal_cells,
				meters_per_cell, subprogress);
	}
	report(progress, 16.0, "Processing elevation...");
	// Restore surveyed natural summits after smoothing, then fit terrain with
	// room for tall landmark models at their actual terrain elevations.
	// Match Rust: surveyed summits are restored after smoothing for every body;
	// only the Earth-specific DSM outlier/anomaly repairs above are conditional.
	raise_peaks(raw.heights, bbox, terrain_hints.peaks, meters_per_cell);
	auto tall_cells = model_cells(raw.heights, bbox, terrain_hints.tall_models);
	const auto structure_cells =
			model_cells(raw.heights, bbox, terrain_hints.tall_structures);
	tall_cells.insert(tall_cells.end(), structure_cells.begin(), structure_cells.end());
	const int top_reserve =
			policy.kind == AffinePolicy::Kind::Fixed
					? 0
					: top_reserve_for(raw.heights, tall_cells, scale * height_multiplier,
							  scale, ground_level, min_ground_level, disable_height_limit,
							  extended_max_y,
							  policy.kind == AffinePolicy::Kind::FitWithHeadroom);

	auto [minecraft_heights, mapping] = scale_to_minecraft_reserving(raw.heights,
			scale * height_multiplier, ground_level, min_ground_level,
			disable_height_limit, extended_max_y, policy, top_reserve,
			[&](std::string_view message) { report(progress, -1.0, message); });
	// The metre grid is no longer needed. Release it before allocating the f32
	// output so large runs do not keep raw, scaled-f64, and output grids at once.
	std::vector<std::vector<double>>().swap(raw.heights);
	ProcessedElevationData out;
	out.height = minecraft_heights.size();
	out.width = out.height ? minecraft_heights.front().size() : 0;
	out.world_width = world_width;
	out.world_height = world_height;
	out.min_height_m = mapping.min_height_m;
	out.blocks_per_meter = mapping.blocks_per_meter;
	out.slope_correction =
			mapping.blocks_per_meter > 0.0 ? scale / mapping.blocks_per_meter : 1.0;
	out.ground_level = mapping.ground_level;
	out.soft_top = mapping.soft_top;
	out.heights.resize(out.height);
	for (std::size_t z = 0; z < out.height; ++z) {
		out.heights[z].reserve(minecraft_heights[z].size());
		for (const double value : minecraft_heights[z])
			out.heights[z].push_back(static_cast<float>(value));
	}
	report(progress, 18.0, "Processing elevation...");
	return out;
}

ProcessedElevationData fetch_elevation_data(Selector &selector,
		const geographic::LLBBox &bbox, std::size_t world_width, std::size_t world_height,
		std::size_t grid_width, std::size_t grid_height, double scale,
		double height_multiplier, int ground_level, int min_ground_level,
		bool disable_height_limit, int extended_max_y, bool apply_earth_repairs,
		land_cover::LandCoverData *land_cover, const AffinePolicy &affine,
		const ProgressCallback &progress, const TerrainHints &terrain_hints,
		bool auto_height, std::optional<HeightBand> taller_band,
		bool force_extended_height)
{
	if (selector.source_mode() == providers::SourceMode::Planetary)
		throw std::invalid_argument(
				"Planetary elevation must be fetched with fetch_planetary_elevation");
	report(progress, 10.0, "Downloading elevation data...");
	const auto &minimum = bbox.min();
	const auto &maximum = bbox.max();
	auto raw = selector.raw_grid(minimum.lat(), minimum.lng(), maximum.lat(),
			maximum.lng(), grid_width, grid_height);
	raw.world_width = world_width;
	raw.world_height = world_height;
	bool use_extended = disable_height_limit || (auto_height && force_extended_height);
	if (auto_height && taller_band) {
		constexpr int vanilla_min_ground = world_editor::DEFAULT_MIN_Y + 2;
		constexpr int vanilla_max_ground =
				world_editor::DEFAULT_MAX_Y - TERRAIN_HEIGHT_BUFFER;
		if (force_extended_height || ground_level < vanilla_min_ground ||
				ground_level > vanilla_max_ground) {
			use_extended = true;
		} else {
			auto tops = model_cells(raw.heights, bbox, terrain_hints.tall_models);
			const auto structures =
					model_cells(raw.heights, bbox, terrain_hints.tall_structures);
			tops.insert(tops.end(), structures.begin(), structures.end());
			const auto overflow = vanilla_overflow(
					raw.heights, tops, scale * height_multiplier, scale, ground_level);
			if (overflow) {
				std::clog << "Extended build height: " << *overflow << '\n';
				use_extended = true;
			}
		}
	}
	if (use_extended && taller_band && auto_height) {
		ground_level = taller_band->ground_level;
		min_ground_level = taller_band->min_ground_level;
		extended_max_y = taller_band->extended_max_y;
	}
	auto out = process_elevation_data(bbox, std::move(raw), scale, height_multiplier,
			ground_level, min_ground_level, use_extended, extended_max_y,
			apply_earth_repairs, land_cover, affine, progress, terrain_hints);
	out.extended_ceiling = use_extended && extended_max_y > 319;
	return out;
}

} // namespace arnis::elevation
