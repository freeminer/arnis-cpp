#pragma once

#include "../floodfill_cache.h"

namespace arnis::allotments
{

void prepare(std::vector<ProcessedElement> &elements, double scale);
void generate_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes);

} // namespace arnis::allotments
