#pragma once
#include "../colors.h"
#include "../../../arnis_block.h"
#include <cstddef>
#include <vector>
#include "../deterministic_rng.h"
namespace arnis::models_3d
{
inline constexpr std::uint8_t USE_MODEL = 1, USE_WALL = 2, USE_ROOF = 4;
Block closest_block(RGBTuple color);
std::vector<Block> closest_blocks(RGBTuple color, std::size_t k);
std::vector<Block> closest_blocks_for_usage(
		RGBTuple color, std::size_t k, std::uint8_t usage);
std::vector<Block> all_blocks_for_usage(std::uint8_t usage);
Block block_for_usage(
		RGBTuple color, std::uint8_t usage, std::uint64_t deterministic_seed);
Block block_for_usage(RGBTuple color, std::uint8_t usage, ChaCha8Rng &rng);
}
