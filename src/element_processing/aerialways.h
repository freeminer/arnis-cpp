#pragma once

#include "../floodfill_cache.h"
#include "../osm_parser.h"

#include <vector>

namespace arnis::aerialways
{

bool is_lift(const tags_t &tags);
CoordinateBitmap clear_corridors(
		const std::vector<ProcessedElement> &elements, const XZBBox &bounds);
void generate_aerialway(WorldEditor &editor, const ProcessedWay &way,
		const CoordinateBitmap &building_footprints, const RoadMaskBitmap &road_mask);

} // namespace arnis::aerialways
