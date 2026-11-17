#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis
{
namespace emergency
{
void generate_emergency(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask);
void generate_fire_hydrant(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask);

}
}
