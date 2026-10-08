#pragma once
#include "colors.h"
#include "deterministic_rng.h"

namespace arnis
{

enum class StairFacing
{
	North,
	East,
	South,
	West,
};

enum class StairShape
{
	Straight,
	InnerLeft,
	InnerRight,
	OuterLeft,
	OuterRight,
};

inline const char *StairFacing_as_str(StairFacing f) noexcept
{
	switch (f) {
	case StairFacing::North:
		return "north";
	case StairFacing::East:
		return "east";
	case StairFacing::South:
		return "south";
	case StairFacing::West:
		return "west";
	}
	return "";
}

inline const char *StairShape_as_str(StairShape s) noexcept
{
	switch (s) {
	case StairShape::Straight:
		return "straight";
	case StairShape::InnerLeft:
		return "inner_left";
	case StairShape::InnerRight:
		return "inner_right";
	case StairShape::OuterLeft:
		return "outer_left";
	case StairShape::OuterRight:
		return "outer_right";
	}
	return "";
}

class Block;
class BlockWithProperties;

Block get_building_wall_block_for_color(const RGB &color);
Block get_fallback_building_block();
Block get_castle_wall_block();
Block get_building_wall_block_for_color(const RGB &color, ChaCha8Rng &rng);
Block get_fallback_building_block(ChaCha8Rng &rng);
Block get_castle_wall_block(ChaCha8Rng &rng);
Block get_random_floor_block();
// Seeded forms are used by the Rust-parity generators.  The legacy overload
// remains for older callers that deliberately use the host-global palette RNG.
Block get_random_floor_block(ChaCha8Rng &rng);
Block get_window_block_for_building_type(const std::string &building_type);
Block get_window_block_for_building_type(
		const std::string &building_type, ChaCha8Rng &rng);
Block get_stair_block_for_material(const Block &material);
BlockWithProperties create_stair_with_properties(
		const Block &base_stair_block, StairFacing facing, StairShape shape);
BlockWithProperties top_stair(BlockWithProperties stair);
Block get_slab_block_for_material(const Block &material);
Block get_wall_piece_for_material(const Block &material);
// Java snow layers use one node plus a `layers` state.  Keep the Rust
// SNOW_LAYERS[0..7] concept available to terrain and structure code without
// inventing separate node IDs in the Luanti palette.
BlockWithProperties snow_layer_with_depth(unsigned layers);

namespace block_definitions
{

// Example constants to match the Rust block references
// You’ll need to define these properly in your code.

extern Block ACACIA_PLANKS;
extern Block AIR;
extern Block ANDESITE;
extern Block ANDESITE_SLAB;
extern Block BAMBOO_SLAB;
extern Block BAMBOO_STAIRS;
extern Block BIRCH_LEAVES;
extern Block BIRCH_LOG;
extern Block BIRCH_BUTTON;
extern Block BIRCH_DOOR;
extern Block BIRCH_FENCE;
extern Block BIRCH_PRESSURE_PLATE;
extern Block BLACK_CONCRETE;
extern Block BLACKSTONE;
extern Block BLACKSTONE_STAIRS;
extern Block BLACKSTONE_WALL;
extern Block BLUE_ICE;
extern Block BLUE_FLOWER;
extern Block BLUE_TERRACOTTA;
extern Block BRICK;
extern Block CAULDRON;
extern Block CHISELED_STONE_BRICKS;
extern Block COBBLESTONE_WALL;
extern Block COBBLESTONE;
extern Block CORNFLOWER;
extern Block POLISHED_BLACKSTONE_BRICKS;
extern Block CHISELED_POLISHED_BLACKSTONE;
extern Block CHISELED_DEEPSLATE;
extern Block COAL_BLOCK;
extern Block COBBLESTONE_SLAB;
extern Block DARK_PRISMARINE;
extern Block DARK_PRISMARINE_SLAB;
extern Block DARK_PRISMARINE_STAIRS;
extern Block CRACKED_STONE_BRICKS;
extern Block CRIMSON_PLANKS;
extern Block CRIMSON_SLAB;
extern Block CRIMSON_STAIRS;
extern Block CUT_SANDSTONE;
extern Block CUT_SANDSTONE_SLAB;
extern Block CYAN_CARPET;
extern Block CYAN_CONCRETE;
extern Block DARK_OAK_PLANKS;
extern Block DARK_OAK_SLAB;
extern Block DARK_OAK_FENCE;
extern Block DEEPSLATE_BRICKS;
extern Block DEEPSLATE_TILES;
extern Block DEEPSLATE_TILE_SLAB;
extern Block DEEPSLATE_TILE_WALL;
extern Block DIORITE;
extern Block DIORITE_STAIRS;
extern Block DIORITE_WALL;
extern Block DIRT;
extern Block END_STONE_BRICKS;
extern Block END_STONE;
extern Block END_STONE_BRICK_SLAB;
extern Block END_STONE_BRICK_WALL;
extern Block FARMLAND;
extern Block GLASS;
extern Block GLOWSTONE;
extern Block GRANITE;
extern Block GRASS_BLOCK;
extern Block GRASS;
extern Block SNOWY_GRASS_BLOCK;
extern Block GRAVEL;
extern Block GRAY_CONCRETE;
extern Block GRAY_TERRACOTTA;
extern Block GREEN_STAINED_HARDENED_CLAY;
extern Block GREEN_WOOL;
extern Block HAY_BALE;
extern Block LANTERN;
extern Block SOUL_LANTERN;
extern Block SMOKER;
extern Block EMPTY_FLOWER_POT;
extern Block COMPOSTER;
extern Block HOPPER;
extern Block BLAST_FURNACE;
extern Block DISPENSER;
extern Block GRINDSTONE;
extern Block POLISHED_BLACKSTONE_SLAB;
extern Block REDSTONE_LAMP;
extern Block AMETHYST_CLUSTER;
extern Block CHISELED_QUARTZ_BLOCK;
extern Block IRON_BARS;
extern Block IRON_DOOR;
extern Block IRON_BLOCK;
extern Block LILY_OF_THE_VALLEY;
extern Block ALLIUM;
extern Block JUNGLE_PLANKS;
extern Block JUNGLE_SLAB;
extern Block JUNGLE_STAIRS;
extern Block LADDER;
extern Block LIGHT_BLUE_CONCRETE;
extern Block LIGHT_BLUE_TERRACOTTA;
extern Block LIGHT_GRAY_CONCRETE;
extern Block MOSS_BLOCK;
extern Block MOSSY_COBBLESTONE;
extern Block MUD_BRICKS;
extern Block NETHER_BRICK;
extern Block NETHER_BRICK_FENCE;
extern Block NETHER_BRICK_WALL;
extern Block NETHER_WART_BLOCK;
extern Block NETHERITE_BLOCK;
extern Block OAK_FENCE;
extern Block OAK_LEAVES;
extern Block OAK_LOG;
extern Block OAK_PLANKS;
extern Block OAK_SLAB;
extern Block ORANGE_TERRACOTTA;
extern Block ORANGE_TULIP;
extern Block OXEYE_DAISY;
extern Block PINK_TULIP;
extern Block PODZOL;
extern Block SNOWY_PODZOL;
extern Block POLISHED_ANDESITE;
extern Block POLISHED_BASALT;
extern Block QUARTZ_BLOCK;
extern Block POLISHED_BLACKSTONE;
extern Block BLACKSTONE_SLAB;
extern Block POLISHED_DEEPSLATE;
extern Block POLISHED_DIORITE;
extern Block POLISHED_GRANITE;
extern Block PRISMARINE;
extern Block PURPUR_BLOCK;
extern Block PURPUR_PILLAR;
extern Block QUARTZ_BRICKS;
extern Block RAIL;
extern Block POWERED_RAIL;
extern Block RED_FLOWER;
extern Block RED_NETHER_BRICK;
extern Block RED_TERRACOTTA;
extern Block RED_TULIP;
extern Block RED_WOOL;
extern Block SAND;
extern Block SANDSTONE;
extern Block SCAFFOLDING;
extern Block SMOOTH_QUARTZ;
extern Block SMOOTH_RED_SANDSTONE;
extern Block SMOOTH_SANDSTONE;
extern Block SMOOTH_STONE;
extern Block SPONGE;
extern Block SUNFLOWER_LOWER;
extern Block SUNFLOWER_UPPER;
extern Block SPRUCE_LOG;
extern Block SPRUCE_PLANKS;
extern Block SPRUCE_SLAB;
extern Block SPRUCE_FENCE;
extern Block STONE_BLOCK_SLAB;
extern Block STONE_BRICK_SLAB;
extern Block STONE_BRICKS;
extern Block STONE;
extern Block TERRACOTTA;
extern Block LILAC_LOWER;
extern Block LILAC_UPPER;
extern Block WARPED_PLANKS;
extern Block WARPED_STAIRS;
extern Block WARPED_TRAPDOOR;
extern Block WARPED_SLAB;
extern Block STRIPPED_WARPED_STEM;
extern Block STRIPPED_WARPED_HYPHAE;
extern Block WATER;
extern Block SEAGRASS;
extern Block KELP_PLANT;
extern Block MAGMA_BLOCK;
extern Block OBSIDIAN;
extern Block KELP;
extern Block TALL_SEAGRASS_BOTTOM;
extern Block TALL_SEAGRASS_TOP;
extern Block SEA_PICKLE;
extern Block SOUL_SAND;
extern Block EARTH_BENCH;
extern Block EARTH_TRASH_CAN;
extern Block EARTH_STREET_LAMP;
extern Block EARTH_WELL;
extern Block EARTH_BARBECUE;
extern Block EARTH_GRATING;
extern Block EARTH_FENCE_CHAINLINK;
extern Block EARTH_FENCE_BARBED;
extern Block EARTH_FENCE_PICKET;
extern Block EARTH_FENCE_WROUGHT;
extern Block SUGAR_CANE;
extern Block WHITE_CONCRETE;
extern Block WHITE_FLOWER;
extern Block WHITE_TULIP;
extern Block WHITE_STAINED_GLASS;
extern Block WHITE_TERRACOTTA;
extern Block WHITE_WOOL;
extern Block YELLOW_CONCRETE;
extern Block YELLOW_FLOWER;
extern Block ROSE_BUSH_LOWER;
extern Block ROSE_BUSH_UPPER;
extern Block PEONY_LOWER;
extern Block PEONY_UPPER;
extern Block YELLOW_WOOL;
extern Block LIME_CONCRETE;
extern Block CYAN_WOOL;
extern Block GRAY_WOOL;
extern Block BLUE_CONCRETE;
extern Block PURPLE_CONCRETE;
extern Block RED_CONCRETE;
extern Block MAGENTA_CONCRETE;
extern Block BROWN_WOOL;
extern Block OXIDIZED_COPPER;
extern Block YELLOW_TERRACOTTA;
extern Block SNOW_BLOCK;
extern Block SNOW_LAYER;
extern Block SIGN;
extern Block STEEL_SIGN;
extern Block TEXT_SIGN_SMALL;
extern Block TEXT_SIGN_MEDIUM;
extern Block TEXT_SIGN_LARGE;
extern bool STREETS_AVAILABLE;
extern Block ROAD_ASPHALT;
extern Block ROAD_SIDEWALK;
extern Block ROAD_MARK_DASHED_WHITE;
extern Block ROAD_MARK_DASHED_WHITE_R90;
extern Block ROAD_MARK_SOLID_WHITE_STRIPE;
extern Block ROAD_MARK_SOLID_WHITE_STRIPE_R90;
extern bool STREETS_MARKINGS_AVAILABLE;
extern Block STREETS_POLE;
extern Block STREETS_BOLLARD;
extern Block STREETS_GUARDRAIL;
extern std::array<Block, 3> STREETS_TRAFFIC_LIGHTS;
extern bool STREETS_RRXING_AVAILABLE;
extern Block STREETS_RRXING_BOTTOM;
extern Block STREETS_RRXING_MIDDLE;
extern Block STREETS_RRXING_TOP;
extern Block STREETS_EU_SIGN_STOP;
extern Block STREETS_EU_SIGN_YIELD;
extern Block STREETS_EU_SIGN_NO_ENTRY;
extern Block STREETS_EU_SIGN_CROSSING;
extern Block STREETS_EU_SIGN_CROSSBUCK;
extern std::array<Block, 6> STREETS_EU_SPEED_SIGNS;
extern Block STREETS_US_SIGN_STOP;
extern Block STREETS_US_SIGN_YIELD;
extern Block STREETS_US_SIGN_NO_ENTRY;
extern Block STREETS_US_SIGN_CROSSING;
extern Block STREETS_US_SIGN_CROSSBUCK;
extern bool STREET_SIGNS_AVAILABLE;
extern Block STREET_SIGN_BASIC;
extern Block STREET_SIGN_STOP;
extern Block STREET_SIGN_YIELD;
extern Block STREET_SIGN_SPEED_LIMIT;
extern Block STREET_SIGN_DO_NOT_ENTER;
extern Block STREET_SIGN_PEDESTRIAN_CROSSING;
extern Block STREET_SIGN_RR_CROSSBUCK;
extern Block STREET_SIGN_US_ROUTE;
extern Block STREET_SIGN_US_INTERSTATE;
extern Block DECAL_FRAME;
extern Block ANDESITE_WALL;
extern Block STONE_BRICK_WALL;
extern Block CARROTS;
extern Block DARK_OAK_DOOR_LOWER;
extern Block DARK_OAK_DOOR_UPPER;
extern Block DARK_OAK_LOG;
extern Block DARK_OAK_LEAVES;
extern Block JUNGLE_LOG;
extern Block JUNGLE_LEAVES;
extern Block ACACIA_LOG;
extern Block ACACIA_LEAVES;
extern Block POTATOES;
extern Block WHEAT;
extern Block BEDROCK;
extern Block RAIL_NORTH_SOUTH;
extern Block RAIL_EAST_WEST;
extern Block RAIL_ASCENDING_EAST;
extern Block RAIL_ASCENDING_WEST;
extern Block RAIL_ASCENDING_NORTH;
extern Block RAIL_ASCENDING_SOUTH;
extern Block RAIL_NORTH_EAST;
extern Block RAIL_NORTH_WEST;
extern Block RAIL_SOUTH_EAST;
extern Block RAIL_SOUTH_WEST;
extern Block ADV_RAIL_NORTH_SOUTH;
extern Block ADV_RAIL_EAST_WEST;
extern Block ADV_RAIL_DIAGONAL_NE_SW;
extern Block ADV_RAIL_DIAGONAL_NW_SE;
extern Block ADV_RAIL_STRAIGHT_0;
extern Block ADV_RAIL_STRAIGHT_30;
extern Block ADV_RAIL_STRAIGHT_45;
extern Block ADV_RAIL_STRAIGHT_60;
extern Block ADV_RAIL_CURVE_0;
extern Block ADV_RAIL_CURVE_30;
extern Block ADV_RAIL_CURVE_45;
extern Block ADV_RAIL_CURVE_60;
extern std::array<Block, 4> ADV_RAIL_SWITCH_LEFT_STRAIGHT;
extern std::array<Block, 4> ADV_RAIL_SWITCH_RIGHT_STRAIGHT;
extern std::array<Block, 4> ADV_RAIL_Y_TURNOUT;
extern std::array<Block, 4> ADV_RAIL_THREE_WAY_STRAIGHT;
extern std::array<Block, 4> ADV_RAIL_PERP_CROSSING;
extern std::array<Block, 6> ADV_RAIL_90_PLUS_CROSSING;
extern std::array<Block, 7> ADV_RAIL_DIAGONAL_CROSSING;
extern Block ADV_RAIL_SLOPE_UP;
extern Block ADV_RAIL_SLOPE_DOWN;
extern std::array<Block, 3> ADV_RAIL_GENTLE_SLOPE;
extern std::array<Block, 2> ADV_RAIL_DIAGONAL_SLOPE;
extern bool ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE;
extern bool ADVTRAINS_JUNCTIONS_AVAILABLE;
extern bool ADVTRAINS_CROSSINGS_AVAILABLE;
extern bool ADVTRAINS_SLOPES_AVAILABLE;
extern bool ADVTRAINS_GENTLE_SLOPES_AVAILABLE;
extern bool ADVTRAINS_AVAILABLE;
extern Block ADV_PLATFORM_HIGH;
extern Block CACTUS;
extern Block COARSE_DIRT;
extern Block IRON_ORE;
extern Block COAL_ORE;
extern Block GOLD_ORE;
extern Block COPPER_ORE;
extern Block LAPIS_ORE;
extern Block REDSTONE_ORE;
extern Block DIAMOND_ORE;
extern Block DEEPSLATE_COAL_ORE;
extern Block DEEPSLATE_IRON_ORE;
extern Block DEEPSLATE_COPPER_ORE;
extern Block DEEPSLATE_GOLD_ORE;
extern Block DEEPSLATE_REDSTONE_ORE;
extern Block DEEPSLATE_LAPIS_ORE;
extern Block DEEPSLATE_DIAMOND_ORE;
extern Block CLAY;
extern Block DIRT_PATH;
extern Block ICE;
extern Block PACKED_ICE;
extern Block LAVA;
extern Block POWDER_SNOW;
extern Block AMETHYST_BLOCK;
extern Block BUDDING_AMETHYST;
extern Block CALCITE;
extern Block BASALT;
extern Block SMOOTH_BASALT;
extern Block CAVE_VINES;
extern Block CAVE_VINES_PLANT;
extern Block CAVE_VINES_UNLIT;
extern Block CAVE_VINES_PLANT_LIT;
extern Block SPORE_BLOSSOM;
extern Block AZALEA;
extern Block FLOWERING_AZALEA;
extern Block AZALEA_LEAVES;
extern Block FLOWERING_AZALEA_LEAVES;
extern Block WHITE_BED;
extern Block LECTERN;
extern Block CAKE;
extern Block MELON;
extern Block LOOM;
extern Block SMITHING_TABLE;
extern Block RED_MUSHROOM_BLOCK;
extern Block BROWN_MUSHROOM_BLOCK;
extern Block MUSHROOM_STEM;
extern Block SHROOMLIGHT;
extern Block TUBE_CORAL_BLOCK;
extern Block BRAIN_CORAL_BLOCK;
extern Block BUBBLE_CORAL_BLOCK;
extern Block FIRE_CORAL_BLOCK;
extern Block HORN_CORAL_BLOCK;
extern Block DEAD_TUBE_CORAL_BLOCK;
extern Block DEAD_BRAIN_CORAL_BLOCK;
extern Block DEAD_BUBBLE_CORAL_BLOCK;
extern Block DEAD_FIRE_CORAL_BLOCK;
extern Block DEAD_HORN_CORAL_BLOCK;
extern Block TUBE_CORAL;
extern Block BRAIN_CORAL;
extern Block BUBBLE_CORAL;
extern Block FIRE_CORAL;
extern Block HORN_CORAL;
extern Block TUBE_CORAL_FAN;
extern Block BRAIN_CORAL_FAN;
extern Block BUBBLE_CORAL_FAN;
extern Block FIRE_CORAL_FAN;
extern Block HORN_CORAL_FAN;
extern Block SMALL_AMETHYST_BUD;
extern Block MEDIUM_AMETHYST_BUD;
extern Block LARGE_AMETHYST_BUD;
extern Block DRIPSTONE_BLOCK;
extern Block POINTED_DRIPSTONE;
extern Block GLOW_LICHEN;
extern Block SCULK;
extern Block SCULK_VEIN;
extern Block SCULK_CATALYST;
extern Block SCULK_SENSOR;
extern Block SCULK_SHRIEKER;
extern Block BIG_DRIPLEAF;
extern Block BIG_DRIPLEAF_STEM;
extern Block SMALL_DRIPLEAF_LOWER;
extern Block SMALL_DRIPLEAF_UPPER;
extern Block MUD;
extern Block DEAD_BUSH;
extern Block MYCELIUM;
extern Block RED_MUSHROOM;
extern Block BROWN_MUSHROOM;
extern Block MOSS_CARPET;
extern Block SWEET_BERRY_BUSH;
extern Block PUMPKIN;
extern Block LILY_PAD;
extern Block TALL_GRASS_BOTTOM;
extern Block TALL_GRASS_TOP;
extern Block CRAFTING_TABLE;
extern Block FURNACE;
extern Block WHITE_CARPET;
extern Block GREEN_CARPET;
extern Block LIGHT_BLUE_CARPET;
extern Block LIGHT_GRAY_CARPET;
extern Block BOOKSHELF;
extern Block OAK_PRESSURE_PLATE;
extern Block OAK_STAIRS;
extern Block CHEST;
extern Block RED_CARPET;
extern Block ANVIL;
extern Block NOTE_BLOCK;
extern Block OAK_DOOR;
extern Block BREWING_STAND;
extern Block RED_BED_NORTH_HEAD;
extern Block RED_BED_NORTH_FOOT;
extern Block RED_BED_EAST_HEAD;
extern Block RED_BED_EAST_FOOT;
extern Block RED_BED_SOUTH_HEAD;
extern Block RED_BED_SOUTH_FOOT;
extern Block RED_BED_WEST_HEAD;
extern Block RED_BED_WEST_FOOT;
extern Block GRAY_STAINED_GLASS;
extern Block LIGHT_GRAY_STAINED_GLASS;
extern Block BROWN_STAINED_GLASS;
extern Block TINTED_GLASS;
extern Block OAK_TRAPDOOR;
extern Block BROWN_CONCRETE;
extern Block BLACK_TERRACOTTA;
extern Block BROWN_TERRACOTTA;
extern Block STONE_BRICK_STAIRS;
extern Block MUD_BRICK_STAIRS;
extern Block POLISHED_BLACKSTONE_BRICK_STAIRS;
extern Block BRICK_STAIRS;
extern Block POLISHED_GRANITE_STAIRS;
extern Block END_STONE_BRICK_STAIRS;
extern Block POLISHED_DIORITE_STAIRS;
extern Block SMOOTH_SANDSTONE_STAIRS;
extern Block QUARTZ_STAIRS;
extern Block POLISHED_ANDESITE_STAIRS;
extern Block NETHER_BRICK_STAIRS;
extern Block COBWEB;
extern Block CHISELLED_BOOKSHELF_NORTH; // Chiseled Bookshelf
extern Block CHISELLED_BOOKSHELF_EAST;	// Chiseled Bookshelf East
extern Block CHISELLED_BOOKSHELF_SOUTH; // Chiseled Bookshelf South
extern Block CHISELLED_BOOKSHELF_WEST;	// Chiseled Bookshelf West
extern Block DAMAGED_ANVIL;				// Damaged Anvil

extern Block CHAIN;
extern Block END_ROD;
extern Block LIGHTNING_ROD;
extern Block GOLD_BLOCK;
extern Block SEA_LANTERN;
extern Block ORANGE_CONCRETE;
extern Block ORANGE_WOOL;
extern Block BLUE_WOOL;
extern Block GREEN_CONCRETE;
extern Block BRICK_WALL;
extern Block REDSTONE_BLOCK;
extern Block CHAIN_X;
extern Block CHAIN_Z;
extern Block SPRUCE_DOOR_LOWER;
extern Block SPRUCE_DOOR_UPPER;
extern Block SMOOTH_STONE_SLAB;
extern Block GLASS_PANE;
extern Block LIGHT_GRAY_TERRACOTTA;
extern Block OAK_SLAB_TOP;
extern Block OAK_DOOR_UPPER;
extern Block SPRUCE_LEAVES;
extern Block CYAN_STAINED_GLASS;
extern Block BLUE_STAINED_GLASS;
extern Block LIGHT_BLUE_STAINED_GLASS;
extern Block DAYLIGHT_DETECTOR;
extern Block RED_STAINED_GLASS;
extern Block YELLOW_STAINED_GLASS;
extern Block PURPLE_STAINED_GLASS;
extern Block ORANGE_STAINED_GLASS;
extern Block MAGENTA_STAINED_GLASS;
extern Block FLOWER_POT;
extern Block OAK_TRAPDOOR_OPEN_NORTH;
extern Block OAK_TRAPDOOR_OPEN_SOUTH;
extern Block OAK_TRAPDOOR_OPEN_EAST;
extern Block OAK_TRAPDOOR_OPEN_WEST;
extern Block QUARTZ_SLAB_TOP;
extern Block DARK_OAK_TRAPDOOR;
extern Block SPRUCE_TRAPDOOR;
extern Block BIRCH_TRAPDOOR;
extern Block MUD_BRICK_SLAB;
extern Block BRICK_SLAB;
extern Block POTTED_RED_TULIP;
extern Block POTTED_DANDELION;
extern Block POTTED_BLUE_ORCHID;
extern Block BARREL;
extern Block FERN;
extern Block CHIPPED_ANVIL;
extern Block LARGE_FERN_LOWER;
extern Block LARGE_FERN_UPPER;
extern Block LEVER;
extern Block COBBLESTONE_STAIRS;
extern Block WAXED_CUT_COPPER_STAIRS;
extern Block MOSSY_STONE_BRICK_STAIRS;
extern Block MOSSY_COBBLESTONE_STAIRS;
extern Block DEEPSLATE_BRICK_STAIRS;
extern Block POLISHED_DEEPSLATE_STAIRS;
extern Block RED_NETHER_BRICKS;
extern Block SPRUCE_STAIRS;
extern Block DARK_OAK_STAIRS;
extern Block RED_NETHER_BRICK_STAIRS;
extern Block WAXED_OXIDIZED_CUT_COPPER_STAIRS;
extern Block WAXED_OXIDIZED_COPPER;
extern Block ANDESITE_STAIRS;
extern Block WAXED_EXPOSED_CUT_COPPER_STAIRS;
extern Block WHITE_WALL_BANNER;
extern Block BLUE_WALL_BANNER;
extern Block BLACK_WALL_BANNER;
extern Block RED_WALL_BANNER;
extern Block GREEN_WALL_BANNER;
extern Block MOSSY_STONE_BRICKS;
extern Block DEEPSLATE;
extern Block TUFF;
extern Block COBBLED_DEEPSLATE;
extern Block COBBLED_DEEPSLATE_SLAB;
extern Block COBBLED_DEEPSLATE_STAIRS;
extern Block WATER_CAULDRON;
extern Block WAXED_COPPER_BLOCK;
extern Block WAXED_EXPOSED_COPPER;
extern Block WAXED_EXPOSED_CHISELED_COPPER;
extern Block WAXED_EXPOSED_CUT_COPPER;
extern Block WAXED_EXPOSED_CUT_COPPER_SLAB;
extern Block LODESTONE;
extern Block MANGROVE_LOG;
extern Block MANGROVE_LEAVES;
extern Block CHERRY_LOG;
extern Block CHERRY_LEAVES;
extern Block CHERRY_PLANKS;
extern Block CHERRY_SLAB;
extern Block CHERRY_STAIRS;
extern Block GRAY_CONCRETE_POWDER;
extern Block LIGHT_GRAY_CONCRETE_POWDER;
extern Block BROWN_CONCRETE_POWDER;
extern Block CYAN_TERRACOTTA;
extern Block BLACK_WOOL;
extern Block LIGHT_GRAY_WALL_BANNER;
extern Block BLACK_STAINED_GLASS;
extern Block BIRCH_FENCE_GATE;
extern Block DARK_OAK_BUTTON;
extern Block DARK_OAK_FENCE_GATE;
extern Block DARK_OAK_PRESSURE_PLATE;
extern Block BLUE_STAINED_GLASS_PANE;
extern Block GRAY_STAINED_GLASS_PANE;
extern Block GRAY_WALL_BANNER;
extern Block IRON_TRAPDOOR;
extern Block JUNGLE_FENCE;
extern Block JUNGLE_TRAPDOOR;
extern Block MOSSY_COBBLESTONE_SLAB;
extern Block MOSSY_STONE_BRICK_SLAB;
extern Block MOSSY_STONE_BRICK_WALL;
extern Block OAK_BUTTON;
extern Block OAK_FENCE_GATE;
extern Block PALE_OAK_TRAPDOOR;
extern Block POLISHED_ANDESITE_SLAB;
extern Block POLISHED_BLACKSTONE_BUTTON;
extern Block POLISHED_BLACKSTONE_PRESSURE_PLATE;
extern Block POLISHED_DEEPSLATE_SLAB;
extern Block POLISHED_DEEPSLATE_WALL;
extern Block POLISHED_DIORITE_SLAB;
extern Block PURPUR_SLAB;
extern Block PURPUR_STAIRS;
extern Block QUARTZ_PILLAR;
extern Block REDSTONE_TORCH;
extern Block REDSTONE_WALL_TORCH;
extern Block RED_NETHER_BRICK_SLAB;
extern Block SANDSTONE_WALL;
extern Block SMOOTH_QUARTZ_SLAB;
extern Block SMOOTH_QUARTZ_STAIRS;
extern Block SMOOTH_RED_SANDSTONE_SLAB;
extern Block SPRUCE_BUTTON;
extern Block SPRUCE_FENCE_GATE;
extern Block SPRUCE_WALL_SIGN;
extern Block STONE_BUTTON;
extern Block STONE_PRESSURE_PLATE;
extern Block STONE_STAIRS;
extern Block TRIPWIRE_HOOK;
extern Block ACACIA_TRAPDOOR;
extern Block &CHISELLED_BOOKSHELF;
extern Block GRANITE_STAIRS;
extern Block &SMOOTH_STONE_BLOCK; // = SMOOTH_STONE;
}

using namespace block_definitions;

}
