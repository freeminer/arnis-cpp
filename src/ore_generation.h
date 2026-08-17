#pragma once
#include <array>
#include "../../arnis_block.h"
namespace arnis::world_editor { struct WorldEditor; }
namespace arnis::ore_generation
{
struct OreRule
{
	Block block;
	int depth_min, depth_max;
	unsigned vein_min, vein_max, avg_veins_per_chunk;
};
const std::array<OreRule, 6> &rules();
void generate_ores(world_editor::WorldEditor &, int, int, int, int);
}
