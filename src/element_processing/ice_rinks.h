#pragma once

#include "../floodfill_cache.h"

namespace arnis::ice_rinks
{
void generate_ice_rink(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const Holes *holes);
}
