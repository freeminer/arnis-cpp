#pragma once

#include "../../../arnis_adapter.h"

namespace arnis {
namespace power {

void generate_power(WorldEditor &editor, const ProcessedElement &element);
void generate_power_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_power_tower(WorldEditor &editor, const ProcessedElement &element);
void generate_power_tower_from_node(WorldEditor &editor, const ProcessedNode &node);
void generate_power_tower_impl(WorldEditor &editor, int x, int z, int height);
void generate_power_pole(WorldEditor &editor, const ProcessedElement &element);
void generate_power_pole_from_node(WorldEditor &editor, const ProcessedNode &node);
void generate_power_pole_impl(WorldEditor &editor, int x, int z, int height, const std::string& pole_material);
void generate_power_line(WorldEditor &editor, const ProcessedWay &way);

}
}