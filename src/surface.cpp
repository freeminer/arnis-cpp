#include "surface.h"
#include "mapgen/earth/arnis_block.h"

namespace arnis::surface
{

const std::array<Block, 10> &soil_plants()
{
	static const std::array<Block, 10> blocks{GRASS, FERN, TALL_GRASS_BOTTOM,
			TALL_GRASS_TOP, LARGE_FERN_LOWER, LARGE_FERN_UPPER, RED_FLOWER, BLUE_FLOWER,
			YELLOW_FLOWER, WHITE_FLOWER};
	return blocks;
}

const std::array<Block, 3> &crops()
{
	static const std::array<Block, 3> blocks{WHEAT, CARROTS, POTATOES};
	return blocks;
}

const std::vector<Block> &clearable_plants()
{
	static const std::vector<Block> blocks{GRASS, FERN, TALL_GRASS_BOTTOM, TALL_GRASS_TOP,
			LARGE_FERN_LOWER, LARGE_FERN_UPPER, RED_FLOWER, BLUE_FLOWER, YELLOW_FLOWER,
			WHITE_FLOWER, DEAD_BUSH, WHEAT, CARROTS, POTATOES, SUGAR_CANE};
	return blocks;
}

const std::vector<Block> &soil_plant_tops()
{
	static const std::vector<Block> blocks{TALL_GRASS_TOP, LARGE_FERN_UPPER};
	return blocks;
}

bool supports_vegetation(Block b)
{
	return b == GRASS_BLOCK || b == DIRT || b == COARSE_DIRT || b == PODZOL ||
		   b == MOSS_BLOCK || b == MUD || b == FARMLAND;
}

bool supports_dead_bush(Block b)
{
	return supports_vegetation(b) || b == TERRACOTTA || b == ORANGE_TERRACOTTA ||
		   b == RED_TERRACOTTA || b == BROWN_TERRACOTTA || b == GRAY_TERRACOTTA ||
		   b == LIGHT_GRAY_TERRACOTTA || b == WHITE_TERRACOTTA || b == SAND ||
		   b == RED_SAND;
}

bool takes_wild_plants(Block b)
{
	// Matches ground_generation.rs::natural_tree_ground: these are the
	// substrates where the land-cover pass may place wild plants.
	return b == GRASS_BLOCK || b == COARSE_DIRT || b == DIRT || b == MUD ||
		   b == FARMLAND || b == PODZOL || b == MOSS_BLOCK;
}

bool takes_wild_trees(Block b)
{
	return takes_wild_plants(b) || b == SMOOTH_STONE || b == STONE_BRICKS ||
		   b == CRACKED_STONE_BRICKS;
}

bool takes_wild_dead_bush(Block b)
{
	return takes_wild_plants(b) || b == TERRACOTTA || b == ORANGE_TERRACOTTA ||
		   b == RED_TERRACOTTA || b == BROWN_TERRACOTTA || b == GRAY_TERRACOTTA ||
		   b == LIGHT_GRAY_TERRACOTTA || b == WHITE_TERRACOTTA || b == RED_SAND;
}

bool supports_crops(Block b)
{
	return b == FARMLAND;
}

bool supports_sugar_cane(Block b)
{
	return b == GRASS_BLOCK || b == DIRT || b == COARSE_DIRT || b == PODZOL ||
		   b == MOSS_BLOCK || b == MUD || b == SAND || b == RED_SAND;
}

bool is_natural_ground(Block b)
{
	return supports_vegetation(b) || b == STONE || b == COBBLESTONE || b == TERRACOTTA ||
		   b == ORANGE_TERRACOTTA || b == RED_TERRACOTTA || b == BROWN_TERRACOTTA ||
		   b == GRAY_TERRACOTTA || b == LIGHT_GRAY_TERRACOTTA || b == WHITE_TERRACOTTA ||
		   b == SANDSTONE || b == SMOOTH_SANDSTONE || b == ANDESITE || b == TUFF ||
		   b == GRAVEL || b == SAND || b == RED_SAND || b == DEEPSLATE ||
		   b == COBBLED_DEEPSLATE || b == GRANITE || b == BLACKSTONE || b == CLAY ||
		   b == SNOW_BLOCK || b == PACKED_ICE;
}

} // namespace arnis::surface
