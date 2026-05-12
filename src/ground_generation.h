#pragma once

//#include "../../../arnis_adapter.h"
#include "args.h"
#include "floodfill_cache.h"

namespace arnis::ground_generation
{

void generate_ground_layer(
        WorldEditor &editor,
        const Args &args,
        const XZBBox &xzbbox,
        const BuildingFootprintBitmap &building_footprints);

}
