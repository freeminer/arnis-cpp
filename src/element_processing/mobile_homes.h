#pragma once

#include "../floodfill_cache.h"
#include "../args.h"

namespace arnis::mobile_homes
{

bool claims(const tags_t &tags);
bool is_trailer_park(const tags_t &tags);
void prepare(std::vector<ProcessedElement> &elements, double scale);
bool generate(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, int ground,
		const Args &args, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints);

} // namespace arnis::mobile_homes
