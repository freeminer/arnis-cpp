#pragma once

#include "block_definitions.h"

#include <array>
#include <cstdint>
#include <vector>

namespace arnis::surface
{

// Separate Minecraft placement rules from Arnis scatter rules. A surface can
// support a vanilla plant without being a substrate our land-cover pass owns.
bool supports_vegetation(Block block);
bool supports_dead_bush(Block block);
bool takes_wild_plants(Block block);
bool takes_wild_trees(Block block);
bool takes_wild_dead_bush(Block block);
bool supports_crops(Block block);
bool supports_sugar_cane(Block block);
bool is_natural_ground(Block block);

const std::array<Block, 10> &soil_plants();
const std::array<Block, 3> &crops();
const std::vector<Block> &clearable_plants();
const std::vector<Block> &soil_plant_tops();
inline constexpr int SUGAR_CANE_MAX_HEIGHT = 3;

} // namespace arnis::surface
