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

ElevationHalo ElevationHalo::cut(const std::vector<std::vector<float>> &grid,
		std::size_t x0, std::size_t z0, std::size_t width, std::size_t height)
{
	ElevationHalo halo;
	halo.x0 = x0;
	halo.z0 = z0;
	halo.inner_width = width;
	halo.inner_height = height;
	halo.outer_width = grid.empty() ? 0 : grid.front().size();
	halo.outer_height = grid.size();
	halo.rows.reserve(grid.size());
	for (std::size_t z = 0; z < grid.size(); ++z) {
		if (z >= z0 && z - z0 < height) {
			const auto &row = grid[z];
			const auto left_end = std::min(x0, row.size());
			const auto right_begin =
					left_end == row.size()
							? row.size()
							: left_end + std::min(width, row.size() - left_end);
			std::vector<float> margins;
			margins.reserve(left_end + row.size() - right_begin);
			margins.insert(margins.end(), row.begin(),
					row.begin() + static_cast<std::ptrdiff_t>(left_end));
			margins.insert(margins.end(),
					row.begin() + static_cast<std::ptrdiff_t>(right_begin), row.end());
			halo.rows.push_back(std::move(margins));
		} else {
			halo.rows.push_back(grid[z]);
		}
	}
	return halo;
}

std::optional<float> ElevationHalo::at(std::int32_t x, std::int32_t z) const
{
	if (outer_width == 0 || outer_height == 0)
		return std::nullopt;
	const auto x64 = static_cast<std::int64_t>(x);
	const auto z64 = static_cast<std::int64_t>(z);
	if (x64 >= 0 && z64 >= 0 && static_cast<std::uint64_t>(x64) < inner_width &&
			static_cast<std::uint64_t>(z64) < inner_height)
		return std::nullopt;
	const auto clamp_index = [](std::int64_t coordinate, std::size_t offset,
									 std::size_t extent) {
		const auto value = coordinate + static_cast<std::int64_t>(offset);
		return static_cast<std::size_t>(std::clamp<std::int64_t>(
				value, 0, static_cast<std::int64_t>(extent - 1)));
	};
	const auto px = clamp_index(x64, x0, outer_width);
	const auto pz = clamp_index(z64, z0, outer_height);
	if (pz >= rows.size())
		return std::nullopt;
	const auto &row = rows[pz];
	if (row.size() == outer_width || px < x0)
		return px < row.size() ? std::optional<float>(row[px]) : std::nullopt;
	if (px >= x0 + inner_width) {
		const auto index = px - inner_width;
		return index < row.size() ? std::optional<float>(row[index]) : std::nullopt;
	}
	return std::nullopt;
}

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
		TerrainHints terrain_hints, bool auto_height,
		std::optional<HeightBand> taller_band, bool force_extended_height,
		std::optional<std::size_t> seam_pad,
		const std::function<std::vector<TallModel>()> &late_overture)
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
	raise_peaks(raw.heights, bbox, terrain_hints.peaks, meters_per_cell, seam_pad);
	// Rust chooses the height band only after repairs and surveyed peaks have
	// been applied. Checking the provider's raw grid in fetch_elevation_data can
	// miss a peak or a repair-induced overflow and leave the generated terrain
	// and structures clipped at the vanilla ceiling.
	bool use_extended = disable_height_limit || (auto_height && force_extended_height);
	if (auto_height && taller_band) {
		// Rust's vanilla_overflow is authoritative here: a low requested base alone
		// does not require an extended band; only an actual fit/structure overflow does.
		if (force_extended_height) {
			use_extended = true;
		} else {
			auto tops = model_cells(raw.heights, bbox, terrain_hints.tall_models);
			const auto structures =
					model_cells(raw.heights, bbox, terrain_hints.tall_structures);
			tops.insert(tops.end(), structures.begin(), structures.end());
			auto overflow = vanilla_overflow(
					raw.heights, tops, scale * height_multiplier, scale, ground_level);
			// Match Rust's two-stage hint channel: Overture is awaited only if OSM
			// geometry still fits within vanilla height and Overture hints follow.
			if (!overflow && terrain_hints.overture_follows && late_overture) {
				const auto overture_tops =
						model_cells(raw.heights, bbox, late_overture());
				tops.insert(tops.end(), overture_tops.begin(), overture_tops.end());
				overflow = vanilla_overflow(raw.heights, tops, scale * height_multiplier,
						scale, ground_level);
			}
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
	// Tall structures only decide whether the vanilla band is sufficient. Rust
	// releases this potentially large OSM-derived list before the terrain fit;
	// only placed landmark models need per-cell headroom in the scaled terrain.
	std::vector<TallModel>().swap(terrain_hints.tall_structures);
	const auto tall_cells = model_cells(raw.heights, bbox, terrain_hints.tall_models);
	const int top_reserve =
			policy.kind == AffinePolicy::Kind::Fixed
					? 0
					: top_reserve_for(raw.heights, tall_cells, scale * height_multiplier,
							  scale, ground_level, min_ground_level, use_extended,
							  extended_max_y,
							  policy.kind == AffinePolicy::Kind::FitWithHeadroom);

	auto [minecraft_heights, mapping] = scale_to_minecraft_reserving(raw.heights,
			scale * height_multiplier, ground_level, min_ground_level, use_extended,
			extended_max_y, policy, top_reserve,
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
	out.extended_ceiling = use_extended && extended_max_y > 319;
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
		const ProgressCallback &progress, TerrainHints terrain_hints, bool auto_height,
		std::optional<HeightBand> taller_band, bool force_extended_height,
		std::optional<std::size_t> seam_pad,
		const std::function<std::vector<TallModel>()> &late_overture)
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
	auto out = process_elevation_data(bbox, std::move(raw), scale, height_multiplier,
			ground_level, min_ground_level, disable_height_limit, extended_max_y,
			apply_earth_repairs, land_cover, affine, progress, std::move(terrain_hints),
			auto_height, taller_band, force_extended_height, seam_pad, late_overture);
	return out;
}

} // namespace arnis::elevation
