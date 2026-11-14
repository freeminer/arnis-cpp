#pragma once

#include "args.h"
#include "floodfill_cache.h"

namespace arnis::ground_scatter
{

void scatter_region(WorldEditor &editor, const Args &args, const XZBBox &bbox, int min_x,
		int max_x, int min_z, int max_z);

} // namespace arnis::ground_scatter
