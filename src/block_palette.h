#pragma once

#include "colors.h"
#include "deterministic_rng.h"
#include "../../arnis_block.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace arnis::block_palette
{
inline constexpr std::uint8_t USE_MODEL = 1;
inline constexpr std::uint8_t USE_WALL = 2;
inline constexpr std::uint8_t USE_ROOF = 4;

Block closest_block(RGBTuple color);
std::vector<Block> closest_blocks(RGBTuple color, std::size_t k);
Block wall_block_for_color(RGBTuple color, ChaCha8Rng &rng);
Block roof_block_for_color(RGBTuple color, ChaCha8Rng &rng);
std::vector<Block> all_building_palette_blocks();
}
