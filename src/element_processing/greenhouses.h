#pragma once

#include "../floodfill_cache.h"

namespace arnis::greenhouses
{

bool is_greenhouse(const tags_t &tags);
bool wants_venlo_roof(const tags_t &tags);
void venlo_roof(WorldEditor &editor, const std::vector<std::pair<int, int>> &area,
		bool along_x, int roof_y);
void plant_rows(WorldEditor &editor, const std::vector<std::pair<int, int>> &area,
		bool along_x, int floor_y, std::uint64_t seed);
void horticulture_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes);

} // namespace arnis::greenhouses
