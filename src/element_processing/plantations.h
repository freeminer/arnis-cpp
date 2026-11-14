#pragma once

#include "../floodfill_cache.h"

namespace arnis::plantations
{

void vineyard_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes);
void orchard_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes);

} // namespace arnis::plantations
