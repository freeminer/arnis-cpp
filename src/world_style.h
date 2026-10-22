#pragma once

#include "args.h"
#include "floodfill_cache.h"

#include <set>
#include <utility>

namespace arnis::world_style
{
// Apply optional final-world weathering after all structure/model placement and
// before the backend commits generated blocks.
void apply(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints);
// Style one completed 512x512 region after all generation and deferred placements
// that can write to it have finished.
void apply_region(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints, int region_x, int region_z);
// Finalize regions that were not styled during the streaming tile pass.
void apply_remaining(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints,
		const std::set<std::pair<int, int>> &already_styled);
}
