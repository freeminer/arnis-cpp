#pragma once
#include "../../arnis_adapter.h"
#include "floodfill_cache.h"

namespace arnis
{

bool generate_world(
		WorldEditor &editor, const std::vector<ProcessedElement> &elements,
		const Args &args, FloodFillCache const & flood_fill_cache, 
        BuildingFootprintBitmap const & building_footprints);
}