#pragma once

#include "floodfill_cache.h"

#include <cstddef>
#include <limits>

namespace arnis::railways
{

// Shared centerline predicates and rasterization from the rail renderer.
bool is_at_grade_track(const ProcessedWay &way);
std::vector<std::pair<int, int>> build_smoothed_centerline(const ProcessedWay &way);

enum class RailThrough
{
	None,
	Clips,
	Into,
	Under,
};

/// Ground-level track space that buildings should leave open, together with
/// whether the track passes under a building or runs into it as a hall.
struct RailPassages
{
	CoordinateBitmap cells;
	CoordinateBitmap under;
	CoordinateBitmap track;
	int height{0};
	std::size_t runs_in{std::numeric_limits<std::size_t>::max()};

	static const RailPassages &none();
	RailThrough through(const std::vector<std::pair<int, int>> &area) const;
};

RailPassages collect_rail_passages(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, double scale,
		const BuildingFootprintBitmap &building_footprints);

} // namespace arnis::railways
