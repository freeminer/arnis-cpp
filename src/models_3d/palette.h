#pragma once
#include "../colors.h"
#include "../../../arnis_block.h"
#include <cstddef>
#include <vector>
namespace arnis::models_3d
{
Block closest_block(RGBTuple color);
std::vector<Block> closest_blocks(RGBTuple color, std::size_t k);
}
