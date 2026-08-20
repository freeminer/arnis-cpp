#pragma once

#include "../../../arnis_adapter.h"
#include <optional>

namespace arnis
{
struct RoadMaskBitmap;

namespace tourisms
{

void generate_tourisms(WorldEditor &editor, const ProcessedNode &element,
		const RoadMaskBitmap &road_mask);

}
}