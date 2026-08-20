#include "block_palette.h"
#include "models_3d/palette.h"

#include <algorithm>

namespace arnis::block_palette
{
Block closest_block(RGBTuple color)
{
	return models_3d::closest_block(color);
}

std::vector<Block> closest_blocks(RGBTuple color, std::size_t k)
{
	return models_3d::closest_blocks(color, k);
}

Block wall_block_for_color(RGBTuple color, ChaCha8Rng &rng)
{
	return models_3d::block_for_usage(color, USE_WALL, rng);
}

Block roof_block_for_color(RGBTuple color, ChaCha8Rng &rng)
{
	return models_3d::block_for_usage(color, USE_ROOF, rng);
}

std::vector<Block> all_building_palette_blocks()
{
	auto walls = models_3d::all_blocks_for_usage(USE_WALL);
	auto roofs = models_3d::all_blocks_for_usage(USE_ROOF);
	for (const Block block : roofs)
		if (std::find(walls.begin(), walls.end(), block) == walls.end())
			walls.push_back(block);
	return walls;
}
}
