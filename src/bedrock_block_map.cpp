#include "bedrock_block_map.h"
#include "../../arnis_block.h"
#include <initializer_list>
#include <optional>
namespace arnis
{
static BedrockBlock simple(const std::string &n)
{
	return {"minecraft:" + n, {}};
}
static BedrockBlock state(
		const std::string &n, std::map<std::string, BedrockStateValue> s)
{
	return {"minecraft:" + n, std::move(s)};
}
BedrockBlock to_bedrock_block(const Block &block)
{
	using namespace block_definitions;
	if (block == AIR)
		return simple("air");
	if (block == WAXED_COPPER_BLOCK)
		return simple("waxed_copper");
	if (block == WAXED_EXPOSED_COPPER)
		return simple("waxed_exposed_copper");
	if (block == WAXED_EXPOSED_CHISELED_COPPER)
		return simple("waxed_exposed_chiseled_copper");
	if (block == WAXED_EXPOSED_CUT_COPPER)
		return simple("waxed_exposed_cut_copper");
	if (block == WAXED_OXIDIZED_COPPER)
		return simple("waxed_oxidized_copper");
	if (block == GRASS_BLOCK)
		return simple("grass_block");
	if (block == GRASS)
		return state("tallgrass", {{"tall_grass_type", std::string("tall")}});
	if (block == TALL_GRASS_BOTTOM)
		return state("double_plant", {{"double_plant_type", std::string("grass")},
											 {"upper_block_bit", false}});
	if (block == TALL_GRASS_TOP)
		return state("double_plant",
				{{"double_plant_type", std::string("grass")}, {"upper_block_bit", true}});
	if (block == STONE)
		return simple("stone");
	if (block == DIRT)
		return simple("dirt");
	if (block == WATER)
		return state("water", {{"liquid_depth", 0}});
	if (block == SUGAR_CANE)
		return state("reeds", {{"age", 0}});
	if (block == DEAD_BUSH)
		return simple("deadbush");
	if (block == KELP || block == KELP_PLANT)
		return state("kelp", {{"kelp_age", 25}});
	if (block == SEAGRASS)
		return state("seagrass", {{"sea_grass_type", std::string("default")}});
	if (block == TALL_SEAGRASS_BOTTOM)
		return state("seagrass", {{"sea_grass_type", std::string("double_bot")}});
	if (block == TALL_SEAGRASS_TOP)
		return state("seagrass", {{"sea_grass_type", std::string("double_top")}});
	if (block == OAK_LOG)
		return state("oak_log", {{"pillar_axis", std::string("y")}});
	if (block == BIRCH_LOG)
		return state("birch_log", {{"pillar_axis", std::string("y")}});
	if (block == SPRUCE_LOG)
		return state("spruce_log", {{"pillar_axis", std::string("y")}});
	if (block == JUNGLE_LOG)
		return state("jungle_log", {{"pillar_axis", std::string("y")}});
	if (block == ACACIA_LOG)
		return state("acacia_log", {{"pillar_axis", std::string("y")}});
	if (block == DARK_OAK_LOG)
		return state("dark_oak_log", {{"pillar_axis", std::string("y")}});
	if (block == OAK_LEAVES)
		return state("leaves", {{"old_leaf_type", std::string("oak")},
									   {"persistent_bit", true}, {"update_bit", false}});
	if (block == BIRCH_LEAVES)
		return state("leaves", {{"old_leaf_type", std::string("birch")},
									   {"persistent_bit", true}, {"update_bit", false}});
	if (block == JUNGLE_LEAVES)
		return state("leaves", {{"old_leaf_type", std::string("jungle")},
									   {"persistent_bit", true}, {"update_bit", false}});
	if (block == SPRUCE_LEAVES)
		return state("leaves", {{"old_leaf_type", std::string("spruce")},
									   {"persistent_bit", true}, {"update_bit", false}});
	if (block == ACACIA_LEAVES)
		return state("leaves2", {{"new_leaf_type", std::string("acacia")},
										{"persistent_bit", true}, {"update_bit", false}});
	if (block == DARK_OAK_LEAVES)
		return state("leaves2", {{"new_leaf_type", std::string("dark_oak")},
										{"persistent_bit", true}, {"update_bit", false}});
	if (block == RAIL || block == RAIL_NORTH_SOUTH)
		return state("rail", {{"rail_direction", 0}});
	if (block == RAIL_EAST_WEST)
		return state("rail", {{"rail_direction", 1}});
	if (block == RAIL_ASCENDING_EAST || block == RAIL_ASCENDING_WEST ||
			block == RAIL_ASCENDING_NORTH || block == RAIL_ASCENDING_SOUTH ||
			block == RAIL_SOUTH_EAST || block == RAIL_SOUTH_WEST ||
			block == RAIL_NORTH_WEST || block == RAIL_NORTH_EAST) {
		int direction = block == RAIL_ASCENDING_EAST	? 2
						: block == RAIL_ASCENDING_WEST	? 3
						: block == RAIL_ASCENDING_NORTH ? 4
						: block == RAIL_ASCENDING_SOUTH ? 5
						: block == RAIL_SOUTH_EAST		? 6
						: block == RAIL_SOUTH_WEST		? 7
						: block == RAIL_NORTH_WEST		? 8
														: 9;
		return state("rail", {{"rail_direction", direction}});
	}
	if (block == FARMLAND)
		return state("farmland", {{"moisturized_amount", 7}});
	if (block == OAK_DOOR)
		return state(
				"wooden_door", {{"door_hinge_bit", false}, {"direction", 0},
									   {"open_bit", false}, {"upper_block_bit", false}});
	if (block == OAK_DOOR_UPPER)
		return state(
				"wooden_door", {{"door_hinge_bit", false}, {"direction", 0},
									   {"open_bit", false}, {"upper_block_bit", true}});
	if (block == DARK_OAK_DOOR_LOWER || block == DARK_OAK_DOOR_UPPER ||
			block == SPRUCE_DOOR_LOWER || block == SPRUCE_DOOR_UPPER) {
		const bool upper = block == DARK_OAK_DOOR_UPPER || block == SPRUCE_DOOR_UPPER;
		const char *name = block == DARK_OAK_DOOR_LOWER || block == DARK_OAK_DOOR_UPPER
								   ? "dark_oak_door"
								   : "spruce_door";
		return state(name, {{"door_hinge_bit", false}, {"direction", 0},
								   {"open_bit", false}, {"upper_block_bit", upper}});
	}
	if (block == FURNACE)
		return state("furnace", {{"facing_direction", 2}, {"burning", false}});
	if (block == CHEST)
		return state("chest", {{"facing_direction", 2}});
	if (block == BREWING_STAND)
		return simple("brewing_stand");
	if (block == BOOKSHELF)
		return simple("bookshelf");
	if (block == COBWEB)
		return simple("web");
	if (block == FLOWER_POT)
		return state("flower_pot", {{"update_bit", false}});
	if (block == POTTED_RED_TULIP || block == POTTED_DANDELION ||
			block == POTTED_BLUE_ORCHID)
		return state("flower_pot", {{"update_bit", false}});
	if (block == END_ROD)
		return state("end_rod", {{"facing_direction", 1}});
	if (block == LIGHTNING_ROD)
		return state("lightning_rod", {{"facing_direction", 1}, {"powered_bit", false}});
	if (block == SEA_LANTERN)
		return simple("sea_lantern");
	if (block == DAYLIGHT_DETECTOR)
		return state("daylight_detector", {{"inverted_bit", false}});
	if (block == CHAIN || block == CHAIN_X || block == CHAIN_Z)
		return state("chain", {{"pillar_axis", std::string("y")}});
	if (block == WHITE_CARPET)
		return state("carpet", {{"color", std::string("white")}});
	if (block == RED_CARPET)
		return state("carpet", {{"color", std::string("red")}});
	if (block == GLASS_PANE)
		return simple("glass_pane");
	if (block == GRAY_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("gray")}});
	if (block == LIGHT_GRAY_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("silver")}});
	if (block == BROWN_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("brown")}});
	if (block == CYAN_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("cyan")}});
	if (block == BLUE_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("blue")}});
	if (block == LIGHT_BLUE_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("light_blue")}});
	if (block == WHITE_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("white")}});
	if (block == RED_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("red")}});
	if (block == YELLOW_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("yellow")}});
	if (block == PURPLE_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("purple")}});
	if (block == ORANGE_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("orange")}});
	if (block == MAGENTA_STAINED_GLASS)
		return state("stained_glass_pane", {{"color", std::string("magenta")}});
	if (block == CARROTS)
		return state("carrots", {{"growth", 7}});
	if (block == POTATOES)
		return state("potatoes", {{"growth", 7}});
	if (block == WHEAT)
		return state("wheat", {{"growth", 7}});
	const auto concrete = [&](Block value,
								  const char *color) -> std::optional<BedrockBlock> {
		return block == value ? std::optional<BedrockBlock>(state(
										"concrete", {{"color", std::string(color)}}))
							  : std::nullopt;
	};
	for (const auto &[value,
				 color] : std::initializer_list<std::pair<Block, const char *>>{
				 {WHITE_CONCRETE, "white"}, {BLACK_CONCRETE, "black"},
				 {GRAY_CONCRETE, "gray"}, {LIGHT_GRAY_CONCRETE, "silver"},
				 {LIGHT_BLUE_CONCRETE, "light_blue"}, {CYAN_CONCRETE, "cyan"},
				 {BLUE_CONCRETE, "blue"}, {RED_CONCRETE, "red"}, {LIME_CONCRETE, "lime"},
				 {YELLOW_CONCRETE, "yellow"}, {ORANGE_CONCRETE, "orange"},
				 {GREEN_CONCRETE, "green"}, {BROWN_CONCRETE, "brown"},
				 {PURPLE_CONCRETE, "purple"}, {MAGENTA_CONCRETE, "magenta"}})
		if (auto mapped = concrete(value, color))
			return *mapped;
	if (block == GRAY_CONCRETE_POWDER)
		return state("concrete_powder", {{"color", std::string("gray")}});
	const auto wool = [&](Block value, const char *color) -> std::optional<BedrockBlock> {
		return block == value ? std::optional<BedrockBlock>(
										state("wool", {{"color", std::string(color)}}))
							  : std::nullopt;
	};
	for (const auto &[value, color] :
			std::initializer_list<std::pair<Block, const char *>>{{WHITE_WOOL, "white"},
					{BLACK_WOOL, "black"}, {GREEN_WOOL, "green"}, {BROWN_WOOL, "brown"},
					{CYAN_WOOL, "cyan"}, {BLUE_WOOL, "blue"}, {RED_WOOL, "red"},
					{YELLOW_WOOL, "yellow"}, {ORANGE_WOOL, "orange"}})
		if (auto mapped = wool(value, color))
			return *mapped;
	const auto terracotta = [&](Block value,
									const char *color) -> std::optional<BedrockBlock> {
		return block == value ? std::optional<BedrockBlock>(state("stained_hardened_clay",
										{{"color", std::string(color)}}))
							  : std::nullopt;
	};
	for (const auto &[value, color] :
			std::initializer_list<std::pair<Block, const char *>>{
					{WHITE_TERRACOTTA, "white"}, {ORANGE_TERRACOTTA, "orange"},
					{YELLOW_TERRACOTTA, "yellow"}, {LIGHT_BLUE_TERRACOTTA, "light_blue"},
					{BLUE_TERRACOTTA, "blue"}, {CYAN_TERRACOTTA, "cyan"},
					{GRAY_TERRACOTTA, "gray"}, {RED_TERRACOTTA, "red"},
					{BLACK_TERRACOTTA, "black"}, {BROWN_TERRACOTTA, "brown"},
					{LIGHT_GRAY_TERRACOTTA, "silver"}})
		if (auto mapped = terracotta(value, color))
			return *mapped;
	const auto stained_glass = [&](Block value,
									   const char *color) -> std::optional<BedrockBlock> {
		return block == value ? std::optional<BedrockBlock>(state(
										"stained_glass", {{"color", std::string(color)}}))
							  : std::nullopt;
	};
	for (const auto &[value, color] :
			std::initializer_list<std::pair<Block, const char *>>{
					{WHITE_STAINED_GLASS, "white"}, {GRAY_STAINED_GLASS, "gray"},
					{LIGHT_GRAY_STAINED_GLASS, "silver"}, {BROWN_STAINED_GLASS, "brown"},
					{CYAN_STAINED_GLASS, "cyan"}, {BLUE_STAINED_GLASS, "blue"},
					{LIGHT_BLUE_STAINED_GLASS, "light_blue"}, {RED_STAINED_GLASS, "red"},
					{YELLOW_STAINED_GLASS, "yellow"}, {PURPLE_STAINED_GLASS, "purple"},
					{ORANGE_STAINED_GLASS, "orange"}, {MAGENTA_STAINED_GLASS, "magenta"}})
		if (auto mapped = stained_glass(value, color))
			return *mapped;
	if (block == RED_FLOWER)
		return state("red_flower", {{"flower_type", std::string("poppy")}});
	if (block == BLUE_FLOWER)
		return state("red_flower", {{"flower_type", std::string("orchid")}});
	if (block == YELLOW_FLOWER)
		return simple("yellow_flower");
	if (block == WHITE_FLOWER)
		return state("red_flower", {{"flower_type", std::string("oxeye")}});
	if (block == SEA_PICKLE)
		return state("sea_pickle", {{"cluster_count", 1}, {"dead_bit", false}});
	if (block == TALL_GRASS_BOTTOM || block == TALL_GRASS_TOP)
		return state(
				"double_plant", {{"double_plant_type", std::string("grass")},
										{"upper_block_bit", block == TALL_GRASS_TOP}});
	if (block == MOSS_BLOCK)
		return simple("moss_block");
	if (block == HAY_BALE)
		return state("hay_block", {{"pillar_axis", std::string("y")}});
	if (block == MUD_BRICKS)
		return simple("mud_bricks");
	if (block == SAND)
		return state("sand", {{"sand_type", std::string("normal")}});
	if (block == GRAVEL)
		return simple("gravel");
	if (block == DEAD_BUSH)
		return simple("deadbush");
	if (block == CLAY)
		return simple("clay");
	if (block == IRON_ORE)
		return simple("iron_ore");
	if (block == COAL_ORE)
		return simple("coal_ore");
	if (block == GOLD_ORE)
		return simple("gold_ore");
	if (block == COPPER_ORE)
		return simple("copper_ore");
	if (block == LAPIS_ORE)
		return simple("lapis_ore");
	if (block == REDSTONE_ORE)
		return simple("redstone_ore");
	if (block == DIAMOND_ORE)
		return simple("diamond_ore");
	if (block == DIRT_PATH)
		return simple("grass_path");
	if (block == MUD)
		return simple("mud");
	if (block == GOLD_BLOCK)
		return simple("gold_block");
	if (block == REDSTONE_BLOCK)
		return simple("redstone_block");
	if (block == NETHERITE_BLOCK)
		return simple("netherite_block");
	if (block == CHISELLED_BOOKSHELF_NORTH || block == CHISELLED_BOOKSHELF_EAST ||
			block == CHISELLED_BOOKSHELF_SOUTH || block == CHISELLED_BOOKSHELF_WEST)
		return state("chiseled_bookshelf", {{"facing_direction", 2}});
	if (block == COARSE_DIRT)
		return state("dirt", {{"dirt_type", std::string("coarse")}});
	if (block == PODZOL)
		return state("dirt", {{"dirt_type", std::string("podzol")}});
	// Blocks whose Bedrock identifier is unchanged. Keep this table explicit so
	// Freeminer-only ids never turn into accidental vanilla names.
	for (const auto &[value,
				 name] : std::initializer_list<std::pair<Block, const char *>>{
				 /* stone variants are emitted with their stone_type state below */
				 {COBBLESTONE, "cobblestone"}, {STONE_BRICKS, "stonebrick"},
				 {BRICK, "brick_block"}, {SANDSTONE, "sandstone"},
				 {SMOOTH_SANDSTONE, "sandstone"}, {GLASS, "glass"},
				 {GLOWSTONE, "glowstone"}, {IRON_BLOCK, "iron_block"},
				 {MOSSY_COBBLESTONE, "mossy_cobblestone"},
				 {CHISELED_STONE_BRICKS, "stonebrick"},
				 {CRACKED_STONE_BRICKS, "stonebrick"}, {MOSSY_STONE_BRICKS, "stonebrick"},
				 {END_STONE_BRICKS, "end_bricks"}, {DEEPSLATE_BRICKS, "deepslate_bricks"},
				 {POLISHED_BLACKSTONE, "blackstone"},
				 {POLISHED_BLACKSTONE_BRICKS, "blackstone"},
				 {POLISHED_DEEPSLATE, "deepslate"}, {NETHERITE_BLOCK, "netherite_block"}})
		if (block == value)
			return simple(name);
	if (block == GRANITE)
		return state("stone", {{"stone_type", std::string("granite")}});
	if (block == POLISHED_GRANITE)
		return state("stone", {{"stone_type", std::string("granite_smooth")}});
	if (block == DIORITE)
		return state("stone", {{"stone_type", std::string("diorite")}});
	if (block == POLISHED_DIORITE)
		return state("stone", {{"stone_type", std::string("diorite_smooth")}});
	if (block == ANDESITE)
		return state("stone", {{"stone_type", std::string("andesite")}});
	if (block == POLISHED_ANDESITE)
		return state("stone", {{"stone_type", std::string("andesite_smooth")}});
	for (const auto &[value, wood] :
			std::initializer_list<std::pair<Block, const char *>>{{OAK_PLANKS, "oak"},
					{SPRUCE_PLANKS, "spruce"}, {JUNGLE_PLANKS, "jungle"},
					{ACACIA_PLANKS, "acacia"}, {DARK_OAK_PLANKS, "dark_oak"}})
		if (block == value)
			return state("planks", {{"wood_type", std::string(wood)}});
	if (block == ICE)
		return simple("ice");
	if (block == PACKED_ICE)
		return simple("packed_ice");
	if (block == SNOW_BLOCK)
		return simple("snow");
	if (block == SPONGE)
		return state("sponge", {{"sponge_type", std::string("dry")}});
	if (block == IRON_BARS)
		return simple("iron_bars");
	if (block == LADDER)
		return state("ladder", {{"facing_direction", 2}});
	if (block == SCAFFOLDING)
		return state("scaffolding", {{"structure_void", false}, {"stability", 0},
											{"stability_check", false}, {"tip", false}});
	if (block == CRAFTING_TABLE)
		return simple("crafting_table");
	if (block == NOTE_BLOCK)
		return simple("noteblock");
	if (block == BARREL)
		return state("barrel", {{"facing_direction", 2}, {"open_bit", false}});
	if (block == ANVIL || block == CHIPPED_ANVIL || block == DAMAGED_ANVIL)
		return state("anvil", {{"damage", block == ANVIL		   ? 0
										  : block == CHIPPED_ANVIL ? 1
																   : 2},
									  {"facing_direction", 2}});
	if (block == FERN)
		return state("double_plant",
				{{"double_plant_type", std::string("fern")}, {"upper_block_bit", false}});
	if (block == LARGE_FERN_LOWER || block == LARGE_FERN_UPPER)
		return state(
				"double_plant", {{"double_plant_type", std::string("fern")},
										{"upper_block_bit", block == LARGE_FERN_UPPER}});
	if (block == TINTED_GLASS)
		return simple("glass");
	if (block == LEVER)
		return state("lever", {{"facing_direction", 2}, {"open_bit", false}});
	if (block == CAULDRON)
		return simple("cauldron");
	if (block == BEDROCK)
		return simple("bedrock");
	if (block == BLACKSTONE)
		return simple("blackstone");
	if (block == END_STONE)
		return simple("end_stone");
	if (block == PRISMARINE)
		return state("prismarine", {{"prismarine_block_type", std::string("default")}});
	if (block == QUARTZ_BLOCK)
		return state("quartz_block", {{"chisel_type", std::string("default")}});
	if (block == NETHER_BRICK)
		return simple("nether_brick");
	if (block == RED_NETHER_BRICK || block == RED_NETHER_BRICKS)
		return simple("red_nether_brick");
	if (block == CRIMSON_PLANKS)
		return simple("crimson_planks");
	if (block == WARPED_PLANKS)
		return simple("warped_planks");
	if (block == CUT_SANDSTONE)
		return simple("cut_sandstone");
	if (block == SMOOTH_RED_SANDSTONE)
		return simple("smooth_red_sandstone");
	if (block == SMOOTH_SANDSTONE)
		return simple("smooth_sandstone");
	if (block == SMOOTH_STONE)
		return simple("smooth_stone");
	if (block == POLISHED_BASALT)
		return state("basalt", {{"pillar_axis", std::string("y")}});
	if (block == PURPUR_BLOCK)
		return state("purpur_block", {{"chisel_type", std::string("default")},
											 {"pillar_axis", std::string("y")}});
	if (block == PURPUR_PILLAR)
		return state("purpur_block", {{"chisel_type", std::string("lines")},
											 {"pillar_axis", std::string("y")}});
	if (block == QUARTZ_BRICKS)
		return simple("quartz_bricks");
	if (block == SMOOTH_QUARTZ)
		return simple("quartz_block");
	if (block == OAK_FENCE)
		return state("fence", {{"wood_type", std::string("oak")}});
	if (block == OAK_PRESSURE_PLATE)
		return state("wooden_pressure_plate", {{"redstone_signal", 0}});
	if (block == RED_BED_NORTH_HEAD || block == RED_BED_NORTH_FOOT ||
			block == RED_BED_EAST_HEAD || block == RED_BED_EAST_FOOT ||
			block == RED_BED_SOUTH_HEAD || block == RED_BED_SOUTH_FOOT ||
			block == RED_BED_WEST_HEAD || block == RED_BED_WEST_FOOT) {
		const int direction =
				block == RED_BED_SOUTH_HEAD || block == RED_BED_SOUTH_FOOT	 ? 0
				: block == RED_BED_WEST_HEAD || block == RED_BED_WEST_FOOT	 ? 1
				: block == RED_BED_NORTH_HEAD || block == RED_BED_NORTH_FOOT ? 2
																			 : 3;
		const bool head = block == RED_BED_NORTH_HEAD || block == RED_BED_EAST_HEAD ||
						  block == RED_BED_SOUTH_HEAD || block == RED_BED_WEST_HEAD;
		return state("bed", {{"direction", direction}, {"head_piece_bit", head},
									{"occupied_bit", false}});
	}
	if (block == GREEN_STAINED_HARDENED_CLAY)
		return state("stained_hardened_clay", {{"color", std::string("green")}});
	if (block == OXIDIZED_COPPER)
		return simple("oxidized_copper");
	if (block == TUFF)
		return simple("tuff");
	if (block == COBBLED_DEEPSLATE)
		return simple("cobbled_deepslate");
	if (block == CHERRY_LOG)
		return state("cherry_log", {{"pillar_axis", std::string("y")}});
	if (block == CHERRY_LEAVES)
		return state("cherry_leaves", {{"persistent_bit", true}, {"update_bit", false}});
	if (block == SOUL_SAND)
		return simple("soul_sand");
	if (block == SNOW_LAYER)
		return state("snow_layer", {{"height", 0}, {"covered_bit", false}});
	if (block == OAK_TRAPDOOR || block == DARK_OAK_TRAPDOOR || block == SPRUCE_TRAPDOOR ||
			block == BIRCH_TRAPDOOR)
		return simple(block == OAK_TRAPDOOR		   ? "trapdoor"
					  : block == DARK_OAK_TRAPDOOR ? "dark_oak_trapdoor"
					  : block == SPRUCE_TRAPDOOR   ? "spruce_trapdoor"
												   : "birch_trapdoor");
	if (block == OAK_TRAPDOOR_OPEN_NORTH || block == OAK_TRAPDOOR_OPEN_SOUTH ||
			block == OAK_TRAPDOOR_OPEN_EAST || block == OAK_TRAPDOOR_OPEN_WEST)
		return state("trapdoor", {{"direction", block == OAK_TRAPDOOR_OPEN_EAST	   ? 0
												: block == OAK_TRAPDOOR_OPEN_WEST  ? 1
												: block == OAK_TRAPDOOR_OPEN_SOUTH ? 2
																				   : 3},
										 {"open_bit", true}, {"upside_down_bit", false}});
	if (block == OAK_SLAB || block == OAK_SLAB_TOP)
		return state("wooden_slab", {{"wood_type", std::string("oak")},
											{"top_slot_bit", block == OAK_SLAB_TOP}});
	if (block == SMOOTH_STONE_SLAB)
		return state(
				"stone_block_slab", {{"stone_slab_type", std::string("smooth_stone")},
											{"top_slot_bit", false}});
	if (block == STONE_BLOCK_SLAB)
		return state("stone_block_slab4",
				{{"stone_slab_type_4", std::string("stone")}, {"top_slot_bit", false}});
	if (block == STONE_BRICK_SLAB)
		return state("stone_block_slab", {{"stone_slab_type", std::string("stone_brick")},
												 {"top_slot_bit", false}});
	if (block == BRICK_SLAB)
		return state("stone_block_slab",
				{{"stone_slab_type", std::string("brick")}, {"top_slot_bit", false}});
	if (block == QUARTZ_SLAB_TOP)
		return state("stone_block_slab",
				{{"stone_slab_type", std::string("quartz")}, {"top_slot_bit", true}});
	if (block == MUD_BRICK_SLAB)
		return state("stone_block_slab",
				{{"stone_slab_type", std::string("mud_brick")}, {"top_slot_bit", false}});
	if (block == OAK_STAIRS)
		return simple("oak_stairs");
	if (block == SPRUCE_STAIRS)
		return simple("spruce_stairs");
	if (block == DARK_OAK_STAIRS)
		return simple("dark_oak_stairs");
	if (block == COBBLESTONE_STAIRS)
		return simple("cobblestone_stairs");
	if (block == BRICK_STAIRS)
		return simple("brick_stairs");
	if (block == STONE_BRICK_STAIRS)
		return simple("stone_brick_stairs");
	if (block == NETHER_BRICK_STAIRS)
		return simple("nether_brick_stairs");
	if (block == QUARTZ_STAIRS)
		return simple("quartz_stairs");
	if (block == ANDESITE_STAIRS)
		return simple("andesite_stairs");
	if (block == RED_NETHER_BRICK_STAIRS)
		return simple("red_nether_brick_stairs");
	if (block == MOSSY_COBBLESTONE_STAIRS)
		return simple("mossy_cobblestone_stairs");
	if (block == MOSSY_STONE_BRICK_STAIRS)
		return simple("mossy_stone_brick_stairs");
	if (block == DEEPSLATE_BRICK_STAIRS)
		return simple("deepslate_brick_stairs");
	if (block == POLISHED_DEEPSLATE_STAIRS)
		return simple("polished_deepslate_stairs");
	if (block == MUD_BRICK_STAIRS)
		return simple("mud_brick_stairs");
	if (block == POLISHED_BLACKSTONE_BRICK_STAIRS)
		return simple("polished_blackstone_brick_stairs");
	if (block == POLISHED_GRANITE_STAIRS)
		return simple("polished_granite_stairs");
	if (block == END_STONE_BRICK_STAIRS)
		return simple("end_brick_stairs");
	if (block == POLISHED_DIORITE_STAIRS)
		return simple("polished_diorite_stairs");
	if (block == SMOOTH_SANDSTONE_STAIRS)
		return simple("smooth_sandstone_stairs");
	if (block == POLISHED_ANDESITE_STAIRS)
		return simple("polished_andesite_stairs");
	if (block == WAXED_CUT_COPPER_STAIRS)
		return simple("waxed_cut_copper_stairs");
	if (block == WAXED_EXPOSED_CUT_COPPER_STAIRS)
		return simple("waxed_exposed_cut_copper_stairs");
	if (block == WAXED_OXIDIZED_CUT_COPPER_STAIRS)
		return simple("waxed_oxidized_cut_copper_stairs");
	if (block == COBBLESTONE_WALL)
		return state(
				"cobblestone_wall", {{"wall_block_type", std::string("cobblestone")}});
	if (block == ANDESITE_WALL)
		return state("cobblestone_wall", {{"wall_block_type", std::string("andesite")}});
	if (block == STONE_BRICK_WALL)
		return state(
				"cobblestone_wall", {{"wall_block_type", std::string("stone_brick")}});
	if (block == BRICK_WALL)
		return state("cobblestone_wall", {{"wall_block_type", std::string("brick")}});
	if (block == MAGMA_BLOCK)
		return simple("magma");
	if (block == WATER_CAULDRON)
		return simple("cauldron");
	if (block == SIGN)
		return state("standing_sign", {{"ground_sign_direction", 0}});
	if (block == STEEL_SIGN || block == TEXT_SIGN_SMALL || block == TEXT_SIGN_MEDIUM ||
			block == TEXT_SIGN_LARGE)
		return state("standing_sign", {{"ground_sign_direction", 0}});
	if (block == WHITE_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	if (block == BLUE_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	if (block == BLACK_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	if (block == RED_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	if (block == GREEN_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	if (block == LIGHT_GRAY_WALL_BANNER)
		return state("wall_banner", {{"facing_direction", 2}});
	return simple("unknown");
}
}
