#include "pipeline.h"

#include "../coordinate_system/geographic/llbbox.h"
#include "../coordinate_system/transformation.h"
#include "../land_cover/land_cover.h"
#include "../water_depth.h"
#include "../world_editor/floor_state.h"
#include "selector.h"
#include "detail.h"

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

std::vector<water_depth::BoreCrossing> station_crossings(
		const std::vector<StationHint> &stations, const geographic::LLBBox &bbox,
		const std::vector<std::vector<double>> &heights)
{
	const std::size_t gh = heights.size();
	const std::size_t gw = gh ? heights.front().size() : 0;
	if (gw < 2 || gh < 2)
		return {};
	const double min_lat = bbox.min().lat(), max_lat = bbox.max().lat();
	const double min_lon = bbox.min().lng(), max_lon = bbox.max().lng();
	if (!(max_lat > min_lat) || !(max_lon > min_lon))
		return {};
	std::vector<water_depth::BoreCrossing> result;
	for (const auto &station : stations) {
		if (!std::isfinite(station.lat) || !std::isfinite(station.lon) ||
				station.lat < min_lat || station.lat > max_lat || station.lon < min_lon ||
				station.lon > max_lon)
			continue;
		const auto x = static_cast<std::size_t>(
				std::llround((station.lon - min_lon) / (max_lon - min_lon) * (gw - 1)));
		const auto z = static_cast<std::size_t>(
				std::llround((max_lat - station.lat) / (max_lat - min_lat) * (gh - 1)));
		if (z >= gh || x >= heights[z].size())
			continue;
		const double ground = heights[z][x];
		if (std::isfinite(ground))
			result.push_back({station.depth - 1 - water_depth::BORE_ROOM, ground});
	}
	return result;
}

HeightBand with_bore_room(HeightBand band,
		const std::vector<water_depth::BoreCrossing> &crossings, double lowest_m,
		double range_m, double blocks_per_metre)
{
	if (crossings.empty())
		return band;
	const auto room = [&](const water_depth::BoreCrossing &crossing, int over_base) {
		return band.floor_y + crossing.depth + 2 + water_depth::BORE_ROOM - over_base;
	};
	int worst = std::numeric_limits<int>::min();
	for (const auto &crossing : crossings)
		worst = std::max(worst, room(crossing, 0));
	const int ceiling =
			(band.extended ? band.extended_max_y : MAX_Y) - TERRAIN_HEIGHT_BUFFER;
	const bool exact = range_m * blocks_per_metre <=
					   double(ceiling - std::max(worst, band.ground_level)) / 2.0;
	int needed = std::numeric_limits<int>::min();
	for (const auto &crossing : crossings) {
		int over_base = 0;
		if (exact && std::isfinite(crossing.lowest_m)) {
			const double blocks =
					std::floor((crossing.lowest_m - lowest_m) * blocks_per_metre);
			over_base = blocks <= 2.0 ? 0
						: blocks >= std::numeric_limits<int>::max()
								? std::numeric_limits<int>::max()
								: static_cast<int>(blocks) - 2;
		}
		needed = std::max(needed, room(crossing, over_base));
	}
	if (needed == std::numeric_limits<int>::min())
		needed = worst;
	band.ground_level = std::max(band.ground_level, needed);
	band.min_ground_level =
			std::min(band.ground_level, std::max(band.min_ground_level, needed));
	return band;
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
		const std::function<std::vector<TallModel>()> &late_overture,
		std::optional<std::pair<std::int64_t, std::int64_t>> one_world_origin)
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
			seam_pad && *seam_pad > 0 ? 1.0 / scale
			: grid_width && grid_height
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
	// The graded OSM ways are needed only while steep-detail strength is
	// rasterized. Rust moves them out of TerrainHints and drops them here; do the
	// same so large areas do not retain this geometry through peak/height fitting.
	auto graded = std::move(terrain_hints.graded);
	auto source_pixels = std::move(raw.source_pixels);
	raw.source_pixels.reset();
	if (source_pixels) {
		const auto changed = detail::add_steep_detail(raw.heights, bbox, *source_pixels,
				scale, one_world_origin, land_cover, graded);
		if (changed > 0) {
			const auto total = grid_width * grid_height;
			std::clog
					<< "Steep detail: roughened "
					<< (total ? static_cast<double>(changed) * 100.0 / total : 0.0)
					<< "% of the terrain, steep ground with only coarse elevation data\n";
		}
	}
	source_pixels.reset();
	std::vector<std::vector<std::pair<double, double>>>().swap(graded);
	// Detail runs after land-cover repair, which would otherwise smooth it away,
	// and before surveyed peak restoration, matching the Rust elevation order.
	// Restore surveyed natural summits after smoothing, then fit terrain with
	// room for tall landmark models at their actual terrain elevations.
	// Match Rust: surveyed summits are restored after smoothing for every body;
	// only the Earth-specific DSM outlier/anomaly repairs above are conditional.
	raise_peaks(raw.heights, bbox, terrain_hints.peaks, meters_per_cell, seam_pad);
	// A water carve can cut six blocks into an LC water body, while a station
	// hall needs a similar reserve below its platform. Lift the terrain base
	// before fitting it so both cuts retain their cover above the world floor.
	std::vector<water_depth::BoreCrossing> crossings;
	if (land_cover) {
		const auto cells = bore_water_cells(*land_cover, bbox, terrain_hints.bores);
		auto water_crossings = water_depth::bore_crossings(land_cover->grid, world_width,
				world_height, cells, [&](std::size_t x, std::size_t z) {
					return z < raw.heights.size() && x < raw.heights[z].size()
								   ? raw.heights[z][x]
								   : std::numeric_limits<double>::quiet_NaN();
				});
		crossings.insert(crossings.end(), water_crossings.begin(), water_crossings.end());
	}
	auto station_water_crossings =
			station_crossings(terrain_hints.stations, bbox, raw.heights);
	crossings.insert(crossings.end(), station_water_crossings.begin(),
			station_water_crossings.end());
	if (!crossings.empty()) {
		double lowest = std::numeric_limits<double>::infinity();
		double highest = -std::numeric_limits<double>::infinity();
		for (const auto &row : raw.heights)
			for (const double height : row)
				if (std::isfinite(height)) {
					lowest = std::min(lowest, height);
					highest = std::max(highest, height);
				}
		if (lowest <= highest) {
			HeightBand band{ground_level, min_ground_level, extended_max_y,
					disable_height_limit, world_editor::min_y()};
			band = with_bore_room(
					band, crossings, lowest, highest - lowest, scale * height_multiplier);
			ground_level = band.ground_level;
			min_ground_level = band.min_ground_level;
			if (taller_band)
				*taller_band = with_bore_room(*taller_band, crossings, lowest,
						highest - lowest, scale * height_multiplier);
		}
	}
	// Rust chooses the height band only after repairs and surveyed peaks have
	// been applied. Checking the provider's raw grid in fetch_elevation_data can
	// miss a peak or a repair-induced overflow and leave the generated terrain
	// and structures clipped at the vanilla ceiling.
	bool use_extended = disable_height_limit || (auto_height && force_extended_height);
	std::optional<double> overture_tallest;
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
				const auto overture_models = late_overture();
				double tallest = 0.0;
				for (const auto &model : overture_models)
					tallest = std::max(tallest, model.height_m);
				overture_tallest = tallest;
				const auto overture_tops =
						model_cells(raw.heights, bbox, overture_models);
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
	const bool raised_for_tall_structure = use_extended && taller_band && auto_height;
	const bool overture_pending = terrain_hints.overture_follows && !overture_tallest;
	double tallest_structure_m = 0.0;
	if (raised_for_tall_structure) {
		for (const auto &model : terrain_hints.tall_models)
			tallest_structure_m = std::max(tallest_structure_m, model.height_m);
		for (const auto &model : terrain_hints.tall_structures)
			tallest_structure_m = std::max(tallest_structure_m, model.height_m);
		if (overture_tallest)
			tallest_structure_m = std::max(tallest_structure_m, *overture_tallest);
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
	if (raised_for_tall_structure) {
		double max_height = -std::numeric_limits<double>::infinity();
		for (const auto &row : minecraft_heights)
			for (const double value : row)
				if (std::isfinite(value))
					max_height = std::max(max_height, value);
		out.raised_for = ProcessedElevationData::RaisedFor{
				std::isfinite(max_height) ? static_cast<int>(std::ceil(max_height))
										  : mapping.ground_level,
				static_cast<int>(std::ceil(tallest_structure_m * scale)),
				overture_pending};
	}
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
		const std::function<std::vector<TallModel>()> &late_overture,
		std::optional<std::pair<std::int64_t, std::int64_t>> one_world_origin)
{
	report(progress, 10.0, "Downloading elevation data...");
	const auto &minimum = bbox.min();
	const auto &maximum = bbox.max();
	auto raw = selector.raw_grid(minimum.lat(), minimum.lng(), maximum.lat(),
			maximum.lng(), grid_width, grid_height);
	raw.world_width = world_width;
	raw.world_height = world_height;
	// Rust gates Earth-specific outlier and anomaly repair from SourceMode itself.
	// Keep the explicit C++ opt-out for callers, but never apply those repairs to
	// planetary DEMs even if an embedding caller accidentally leaves it enabled.
	const bool repair_earth =
			apply_earth_repairs && selector.source_mode().allows_earth_fallback();
	auto out = process_elevation_data(bbox, std::move(raw), scale, height_multiplier,
			ground_level, min_ground_level, disable_height_limit, extended_max_y,
			repair_earth, land_cover, affine, progress, std::move(terrain_hints),
			auto_height, taller_band, force_extended_height, seam_pad, late_overture,
			one_world_origin);
	return out;
}

} // namespace arnis::elevation
