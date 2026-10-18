#include "block_palette.h"
#include "block_definitions.h"
#include "models_3d/palette.h"

#include <algorithm>
#include <limits>

namespace arnis::block_palette
{
Block closest_block(RGBTuple color)
{
	return models_3d::closest_block(color);
}

Block closest_of(const std::vector<std::pair<RGBTuple, Block>> &choices, RGBTuple color)
{
	if (choices.empty())
		return STONE_BRICKS;

	const auto target = oklab_components(color);
	Block result = STONE_BRICKS;
	float best_distance = std::numeric_limits<float>::infinity();
	for (const auto &[candidate_color, block] : choices) {
		const auto candidate = oklab_components(candidate_color);
		const float dl = target[0] - candidate[0];
		const float da = target[1] - candidate[1];
		const float db = target[2] - candidate[2];
		const float distance = dl * dl + da * da + db * db;
		if (distance < best_distance) {
			best_distance = distance;
			result = block;
		}
	}
	return result;
}

Block paint_block_for_color(RGBTuple color)
{
	return models_3d::paint_block_for_color(color);
}

Block facade_block_for_color(RGBTuple color)
{
	return models_3d::facade_block_for_color(color);
}

std::vector<Block> closest_blocks(RGBTuple color, std::size_t k)
{
	return models_3d::closest_blocks(color, k);
}

Block wall_block_for_color(RGBTuple color, ChaCha8Rng &rng)
{
	return models_3d::wall_block_for_color(color, rng);
}

Block roof_block_for_color(RGBTuple color, ChaCha8Rng &rng)
{
	return models_3d::block_for_usage(color, USE_ROOF, rng);
}

std::vector<Block> all_building_palette_blocks()
{
	auto walls = models_3d::all_blocks_for_usage(USE_WALL);
	auto roofs = models_3d::all_blocks_for_usage(USE_ROOF);
	auto facade = models_3d::all_blocks_for_usage(USE_FACADE);
	for (const Block block : roofs)
		if (std::find(walls.begin(), walls.end(), block) == walls.end())
			walls.push_back(block);
	for (const Block block : facade)
		if (std::find(walls.begin(), walls.end(), block) == walls.end())
			walls.push_back(block);
	return walls;
}
}
