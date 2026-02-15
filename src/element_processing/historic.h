#pragma once

#include "../../../arnis_adapter.h"

namespace arnis {
namespace historic {
void generate_historic(WorldEditor &editor, const ProcessedNode &node);
void generate_memorial(WorldEditor &editor, const ProcessedNode &node);
void generate_monument(WorldEditor &editor, const ProcessedNode &node);
void generate_wayside_cross(WorldEditor &editor, const ProcessedNode &node);
void generate_cross(WorldEditor &editor, int x, int z, int height);
void generate_pyramid(WorldEditor &editor, const ProcessedWay &element, const Args &args);

}
}