#pragma once

#include "args.h"
#include "floodfill_cache.h"

namespace arnis::world_style
{
// Apply optional final-world weathering after all structure/model placement and
// before the backend commits generated blocks.
void apply(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints);
}
