#pragma once

#include "../floodfill_cache.h"
#include "../../../arnis_adapter.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace arnis::parking_garage
{
void generate(WorldEditor &editor, const std::vector<std::pair<int, int>> &floor_area,
		int building_height, int base_y, std::uint64_t seed);
} // namespace arnis::parking_garage
