#pragma once

#include "../../../arnis_adapter.h"
#include <optional>

namespace arnis
{
struct RoadMaskBitmap;

namespace advertising
{
void generate_advertising(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);
void generate_advertising_column(WorldEditor &editor, const ProcessedNode &node);
void generate_advertising_flag(WorldEditor &editor, const ProcessedNode &node);
void generate_poster_box(WorldEditor &editor, const ProcessedNode &node);

}
}