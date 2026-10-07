#pragma once

#include "../floodfill_cache.h"
#include "../osm_parser.h"
#include "../../../arnis_adapter.h"

#include <utility>
#include <vector>

namespace arnis::construction_site
{
Block ground_block(int x, int z, bool arid);
bool is_arid(biome::Climate climate);
void furnish(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &floor_area,
		const BuildingFootprintBitmap &footprints);
} // namespace arnis::construction_site
