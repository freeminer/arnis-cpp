#pragma once

#include "../floodfill_cache.h"
#include "../../../arnis_adapter.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace arnis::street_furniture
{

void generate_bicycle_parking_area(world_editor::WorldEditor &editor,
		const ProcessedElement &element,
		const std::vector<std::pair<int32_t, int32_t>> &area);
void generate_shelter_node(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);
void picnic_table(
		world_editor::WorldEditor &editor, int x, int z, int base, bool along_x);

} // namespace arnis::street_furniture
