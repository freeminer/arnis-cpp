#include <array>
#include <string>
#include <utility>
#include <vector>
#include <optional>

#include "../args.h"
#include "../floodfill_cache.h"
#include "block_definitions.h"
#include "tree.h"
#include "../osm_parser.h"
#include "world_editor.h"
#include "../deterministic_rng.h"
#include "../bresenham.h"
#include "../structures/structures.h"
#include "bridges.h"

#include "../../../arnis_adapter.h"
namespace arnis
{

namespace landuse
{

void generate_landuse(WorldEditor &editor, ProcessedWay const &element, Args const &args,
		FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface)
{
	const std::string binding = std::string();
	const std::string landuse_tag = [&]() -> std::string {
		auto it = element.tags.find(std::string("landuse"));
		return (it != element.tags.end()) ? it->second : binding;
	}();

	Block block_type = GRASS_BLOCK;
	if (landuse_tag == "greenfield" || landuse_tag == "meadow" ||
			landuse_tag == "grass" || landuse_tag == "orchard" ||
			landuse_tag == "forest") {
		block_type = GRASS_BLOCK;
	} else if (landuse_tag == "farmland") {
		block_type = FARMLAND;
	} else if (landuse_tag == "cemetery") {
		block_type = PODZOL;
	} else if (landuse_tag == "construction") {
		block_type = COARSE_DIRT;
	} else if (landuse_tag == "traffic_island") {
		block_type = STONE_BLOCK_SLAB;
	} else if (landuse_tag == "residential" || landuse_tag == "commercial") {
		return;
	} else if (landuse_tag == "education" || landuse_tag == "religious") {
		block_type = POLISHED_ANDESITE;
	} else if (landuse_tag == "industrial") {
		block_type = STONE;
	} else if (landuse_tag == "military") {
		block_type = GRAY_CONCRETE;
	} else if (landuse_tag == "railway") {
		block_type = GRAVEL;
	} else if (landuse_tag == "vineyard" || landuse_tag == "brownfield" ||
			   landuse_tag == "farmyard") {
		block_type = COARSE_DIRT;
	} else if (landuse_tag == "landfill") {
		auto it = element.tags.find(std::string("man_made"));
		std::string manmade_tag = (it != element.tags.end()) ? it->second : binding;
		if (manmade_tag == "spoil_heap" || manmade_tag == "heap") {
			block_type = GRAVEL;
		} else {
			block_type = COARSE_DIRT;
		}
	} else if (landuse_tag == "quarry") {
		block_type = STONE;
	} else {
		block_type = GRASS_BLOCK;
	}

	std::vector<std::pair<int, int>> polygon_coords;
	polygon_coords.reserve(element.nodes.size());
	for (auto const &n : element.nodes) {
		polygon_coords.emplace_back(n.x, n.z);
	}

	// Use deterministic RNG seeded by element ID for consistent results across region boundaries
	auto rng = element_rng(element.id);

	// Get the area of the landuse element using cache
	std::vector<std::pair<int, int>> floor_area =
			flood_fill_cache.get_or_compute(element, args.timeout_ref());

	// Trees ok to generate based on leaf_type
	std::vector<TreeType> trees_ok_to_generate;
	auto it_leaf_type = element.tags.find("leaf_type");
	if (it_leaf_type != element.tags.end()) {
		const std::string &leaf_type = it_leaf_type->second;
		if (leaf_type == "broadleaved") {
			trees_ok_to_generate.push_back(TreeType::Oak);
			trees_ok_to_generate.push_back(TreeType::Birch);
			trees_ok_to_generate.push_back(TreeType::TallOak);
			trees_ok_to_generate.push_back(TreeType::Bush);
			trees_ok_to_generate.push_back(TreeType::AzaleaBush);
		} else if (leaf_type == "needleleaved") {
			trees_ok_to_generate.push_back(TreeType::Spruce);
			trees_ok_to_generate.push_back(TreeType::Pine);
		} else {
			trees_ok_to_generate.push_back(TreeType::Oak);
			trees_ok_to_generate.push_back(TreeType::Spruce);
			trees_ok_to_generate.push_back(TreeType::Birch);
			trees_ok_to_generate.push_back(TreeType::TallOak);
			trees_ok_to_generate.push_back(TreeType::Pine);
			trees_ok_to_generate.push_back(TreeType::Bush);
			trees_ok_to_generate.push_back(TreeType::AzaleaBush);
			trees_ok_to_generate.push_back(TreeType::Willow);
		}
	} else {
		trees_ok_to_generate.push_back(TreeType::Oak);
		trees_ok_to_generate.push_back(TreeType::Spruce);
		trees_ok_to_generate.push_back(TreeType::Birch);
		trees_ok_to_generate.push_back(TreeType::TallOak);
		trees_ok_to_generate.push_back(TreeType::Pine);
		trees_ok_to_generate.push_back(TreeType::Bush);
		trees_ok_to_generate.push_back(TreeType::AzaleaBush);
	}

	for (auto const &coord : floor_area) {
		int x = coord.first;
		int z = coord.second;

		// Apply per-block randomness for certain landuse types
		Block actual_block = block_type;
		if (landuse_tag == "industrial") {
			// Industrial: primarily stone, with some stone bricks and smooth stone
			int random_value = static_cast<int>(rng.uniform(100));
			if (random_value < 70) {
				actual_block = STONE;
			} else if (random_value < 90) {
				actual_block = STONE_BRICKS;
			} else {
				actual_block = SMOOTH_STONE;
			}
		} else if (landuse_tag == "military") {
			int random_value = static_cast<int>(rng.uniform(100));
			if (random_value < 89) {
				actual_block = GRAY_CONCRETE;
			} else if (random_value < 99) {
				actual_block = STONE_BRICKS;
			} else {
				actual_block = COBBLESTONE;
			}
		} else if (landuse_tag == "quarry") {
			int random_value = static_cast<int>(rng.uniform(100));
			if (random_value < 40) {
				actual_block = STONE;
			} else if (random_value < 60) {
				actual_block = GRAVEL;
			} else if (random_value < 80) {
				actual_block = COBBLESTONE;
			} else {
				actual_block = ANDESITE;
			}
		}

		const std::optional<std::vector<Block>> protected_blocks(std::vector<Block>{
				BLACK_CONCRETE,
				GRAY_CONCRETE_POWDER,
				CYAN_TERRACOTTA,
				GRAY_CONCRETE,
				LIGHT_GRAY_CONCRETE,
				WHITE_CONCRETE,
				DIRT_PATH,
				SMOOTH_STONE,
				WATER,
		});
		const bool is_protected = editor.check_for_block(x, 0, z, protected_blocks);

		if (landuse_tag == "traffic_island") {
			editor.set_block(actual_block, x, 1, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
		} else if (landuse_tag == "construction" || landuse_tag == "railway") {
			editor.set_block(actual_block, x, 0, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
		} else if (!is_protected) {
			editor.set_block(actual_block, x, 0, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
		}

		if (landuse_tag == "cemetery") {
			if ((x % 3 == 0) && (z % 3 == 0)) {
				int random_choice = static_cast<int>(rng.uniform(100));
				if (random_choice < 15) {
					if (editor.check_for_block(x, 0, z,
								std::optional<std::vector<Block>>{
										std::vector<Block>{PODZOL}})) {
						if (rng.uniform(2) == 0) {
							editor.set_block(COBBLESTONE, x - 1, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x - 1, 2, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x + 1, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
						} else {
							editor.set_block(COBBLESTONE, x, 1, z - 1,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x, 2, z - 1,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
							editor.set_block(STONE_BRICK_SLAB, x, 1, z + 1,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
						}
					}
				} else if (random_choice < 30) {
					if (editor.check_for_block(x, 0, z,
								std::optional<std::vector<Block>>{
										std::vector<Block>{PODZOL}})) {
						editor.set_block(RED_FLOWER, x, 1, z,
								std::optional<std::vector<Block>>(),
								std::optional<std::vector<Block>>());
					}
				} else if (random_choice < 33) {
					Tree::create(editor, Coord{x, 1, z}, &building_footprints,
							&bridge_surface);
				} else if (!is_protected && random_choice < 35) {
					editor.set_block(OAK_LEAVES, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (!is_protected && random_choice < 37) {
					editor.set_block(FERN, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (!is_protected && random_choice < 41) {
					editor.set_block(LARGE_FERN_LOWER, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(LARGE_FERN_UPPER, x, 2, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		} else if (landuse_tag == "forest") {
			if (editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{
								std::vector<Block>{GRASS_BLOCK}})) {
				int random_choice = static_cast<int>(rng.uniform(30));
				if (random_choice == 20) {
					TreeType tree_type = trees_ok_to_generate[rng.uniform(
							static_cast<std::uint32_t>(trees_ok_to_generate.size()))];
					Tree::create_of_type(editor, Coord{x, 1, z}, tree_type,
							&building_footprints, &bridge_surface);
				} else if (random_choice == 2) {
					int pick = 1 + static_cast<int>(rng.uniform(6));
					Block flower_block = OAK_LEAVES;
					if (pick == 2)
						flower_block = RED_FLOWER;
					else if (pick == 3)
						flower_block = BLUE_FLOWER;
					else if (pick == 4)
						flower_block = YELLOW_FLOWER;
					else if (pick == 5)
						flower_block = FERN;
					else if (pick == 6)
						flower_block = WHITE_FLOWER;
					editor.set_block(flower_block, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice <= 12) {
					if (rng.uniform(100) < 12) {
						editor.set_block(FERN, x, 1, z,
								std::optional<std::vector<Block>>(),
								std::optional<std::vector<Block>>());
					} else {
						editor.set_block(GRASS, x, 1, z,
								std::optional<std::vector<Block>>(),
								std::optional<std::vector<Block>>());
					}
				}
			}
		} else if (landuse_tag == "grass") {
			if (editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{
								std::vector<Block>{GRASS_BLOCK}})) {
				int r = static_cast<int>(rng.uniform(200));
				if (r == 0) {
					editor.set_block(OAK_LEAVES, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 8) {
					editor.set_block(FERN, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 170) {
					editor.set_block(GRASS, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		} else if (landuse_tag == "greenfield") {
			if (editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{
								std::vector<Block>{GRASS_BLOCK}})) {
				int r = static_cast<int>(rng.uniform(200));
				if (r == 0) {
					editor.set_block(OAK_LEAVES, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 2) {
					editor.set_block(FERN, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 17) {
					editor.set_block(GRASS, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		} else if (landuse_tag == "meadow") {
			if (editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{
								std::vector<Block>{GRASS_BLOCK}})) {
				int random_choice = static_cast<int>(rng.uniform(1001));
				if (random_choice < 5) {
					Tree::create(editor, Coord{x, 1, z}, &building_footprints,
							&bridge_surface);
				} else if (random_choice < 6) {
					editor.set_block(RED_FLOWER, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice < 9) {
					editor.set_block(OAK_LEAVES, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice < 40) {
					editor.set_block(FERN, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice < 65) {
					editor.set_block(LARGE_FERN_LOWER, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(LARGE_FERN_UPPER, x, 2, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice < 825) {
					editor.set_block(GRASS, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		} else if (landuse_tag == "orchard") {
			if (x % 18 == 0 && z % 10 == 0) {
				Tree::create(
						editor, Coord{x, 1, z}, &building_footprints, &bridge_surface);
			} else if (editor.check_for_block(x, 0, z,
							   std::optional<std::vector<Block>>{
									   std::vector<Block>{GRASS_BLOCK}})) {
				int r = static_cast<int>(rng.uniform(100));
				if (r == 0) {
					editor.set_block(OAK_LEAVES, x, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 2) {
					editor.set_block(FERN, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (r <= 20) {
					editor.set_block(GRASS, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		} else if ((landuse_tag == "vineyard" || landuse_tag == "brownfield" ||
						   landuse_tag == "landfill") &&
				   editor.check_for_block(x, 0, z,
						   std::optional<std::vector<Block>>{
								   std::vector<Block>{COARSE_DIRT}})) {
			int r = static_cast<int>(rng.uniform(150));
			if (r <= 3) {
				editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
			} else if (r == 4) {
				editor.set_block(DEAD_BUSH, x, 1, z, std::nullopt, std::nullopt);
			} else if (r <= 15) {
				editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
			}
		} else if (landuse_tag == "farmland") {
			if (!editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{std::vector<Block>{WATER}})) {
				if (x % 9 == 0 && z % 9 == 0) {
					editor.set_block(WATER, x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{FARMLAND}},
							std::optional<std::vector<Block>>());
				} else {
					int r = static_cast<int>(rng.uniform(76));
					if (r == 0) {
						int special_choice = 1 + static_cast<int>(rng.uniform(10));
						if (special_choice <= 4) {
							editor.set_block(HAY_BALE, x, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>{
											std::vector<Block>{SPONGE}});
						} else {
							editor.set_block(OAK_LEAVES, x, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>{
											std::vector<Block>{SPONGE}});
						}
					} else {
						if (editor.check_for_block(x, 0, z,
									std::optional<std::vector<Block>>{
											std::vector<Block>{FARMLAND}})) {
							int crop_choice = static_cast<int>(rng.uniform(3));
							Block crop = WHEAT;
							if (crop_choice == 1)
								crop = CARROTS;
							else if (crop_choice == 2)
								crop = POTATOES;
							editor.set_block(crop, x, 1, z,
									std::optional<std::vector<Block>>(),
									std::optional<std::vector<Block>>());
						}
					}
				}
			}
		} else if (landuse_tag == "construction") {
			int random_choice = static_cast<int>(rng.uniform(1501));
			if (random_choice < 15) {
				editor.set_block(SCAFFOLDING, x, 1, z,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
				if (random_choice < 2) {
					editor.set_block(SCAFFOLDING, x, 2, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 3, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else if (random_choice < 4) {
					editor.set_block(SCAFFOLDING, x, 2, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 3, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 4, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 1, z + 1,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else {
					editor.set_block(SCAFFOLDING, x, 2, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 3, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 4, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x, 5, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x - 1, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(SCAFFOLDING, x + 1, 1, z - 1,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			} else if (random_choice < 55) {
				std::array<Block, 13> construction_items = {OAK_LOG, COBBLESTONE, GRAVEL,
						GLOWSTONE, STONE, COBBLESTONE_WALL, BLACK_CONCRETE, SAND,
						OAK_PLANKS, DIRT, BRICK, CRAFTING_TABLE, FURNACE};
				editor.set_block(
						construction_items[rng.uniform(
								static_cast<std::uint32_t>(construction_items.size()))],
						x, 1, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			} else if (random_choice < 65) {
				if (random_choice < 60) {
					editor.set_block(DIRT, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x, 2, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x + 1, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x, 1, z + 1,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else {
					editor.set_block(DIRT, x, 1, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x, 2, z, std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x - 1, 1, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block(DIRT, x, 1, z - 1,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			} else if (random_choice < 100) {
				editor.set_block(GRAVEL, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 115) {
				editor.set_block(SAND, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 125) {
				editor.set_block(DIORITE, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 145) {
				editor.set_block(BRICK, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 155) {
				editor.set_block(GRANITE, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 180) {
				editor.set_block(ANDESITE, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			} else if (random_choice < 565) {
				editor.set_block(COBBLESTONE, x, 0, z,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>{std::vector<Block>{SPONGE}});
			}
		} else if (landuse_tag == "quarry") {
			editor.set_block(STONE, x, -1, z,
					std::optional<std::vector<Block>>{std::vector<Block>{STONE}},
					std::optional<std::vector<Block>>());
			editor.set_block(STONE, x, -2, z,
					std::optional<std::vector<Block>>{std::vector<Block>{STONE}},
					std::optional<std::vector<Block>>());
			auto it = element.tags.find(std::string("resource"));
			if (it != element.tags.end()) {
				Block ore_block = STONE;
				std::string const &resource = it->second;
				if (resource == "iron_ore")
					ore_block = IRON_ORE;
				else if (resource == "coal")
					ore_block = COAL_ORE;
				else if (resource == "copper")
					ore_block = COPPER_ORE;
				else if (resource == "gold")
					ore_block = GOLD_ORE;
				else if (resource == "clay" || resource == "kaolinite")
					ore_block = CLAY;
				int abs_y = editor.get_absolute_y(x, 0, z);
				const int resource_range = std::max(1, 100 + abs_y);
				int random_choice = static_cast<int>(rng.uniform(resource_range));
				if (random_choice < 5) {
					editor.set_block(ore_block, x, 0, z,
							std::optional<std::vector<Block>>{std::vector<Block>{STONE}},
							std::optional<std::vector<Block>>());
				}
			}
		}
		if (landuse_tag == "cemetery")
			structures::tombstone::maybe_place(editor, x, z, road_mask);
	}

	// Generate a stone brick wall fence around cemeteries
	if (landuse_tag == "cemetery") {
		// Generate cemetery fence
		for (std::size_t i = 1; i < element.nodes.size(); ++i) {
			const auto &prev = element.nodes[i - 1];
			const auto &cur = element.nodes[i];

			std::vector<std::tuple<int, int, int>> points =
					bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			for (const auto &[bx, _, bz] : points) {
				editor.set_block(STONE_BRICK_WALL, bx, 1, bz,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
				editor.set_block(STONE_BRICK_SLAB, bx, 2, bz,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
		}
	}

	if (landuse_tag == "construction") {
		structures::crane::maybe_place_crane(editor, floor_area);
		structures::excavator::scatter_excavators(editor, floor_area);
	}

	if (landuse_tag == "farmland") {
		structures::tractor::maybe_place_tractor(editor, floor_area);
	}
}

void generate_landuse_from_relation(WorldEditor &editor, ProcessedRelation const &rel,
		Args const &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface)
{
	if (rel.tags.find(std::string("landuse")) != rel.tags.end()) {
		// Process each outer member way individually using cached flood fill.
		// We intentionally do not combine all outer nodes into one mega-way,
		// because that creates a nonsensical polygon spanning the whole relation
		// extent, misses the flood fill cache, and can cause multi-GB allocations.
		for (auto const &member : rel.members) {
			if (member.role == ProcessedMemberRole::Outer) {
				// Use relation tags so the member inherits the relation's landuse=* type
				ProcessedWay way_with_rel_tags{member.way.id, member.way.nodes, rel.tags};
				generate_landuse(editor, way_with_rel_tags, args, flood_fill_cache,
						building_footprints, road_mask, bridge_surface);
			}
		}
	}
}

/// Generates ground blocks for place=* areas (squares, neighbourhoods, etc.)
void generate_place(WorldEditor &editor, ProcessedWay const &element, Args const &args,
		FloodFillCache const &flood_fill_cache)
{
	auto it_place = element.tags.find("place");
	if (it_place == element.tags.end()) {
		return;
	}

	const std::string &place_tag = it_place->second;

	// Determine block type based on place tag
	Block block_type;
	if (place_tag == "square") {
		block_type = STONE_BRICKS;
	} else if (place_tag == "neighbourhood" || place_tag == "city_block" ||
			   place_tag == "quarter" || place_tag == "suburb") {
		return;
	} else {
		return;
	}

	// Get the area using flood fill cache
	std::vector<std::pair<int, int>> floor_area =
			flood_fill_cache.get_or_compute(element, args.timeout_ref());

	// Place ground blocks
	for (auto const &coord : floor_area) {
		int x = coord.first;
		int z = coord.second;
		editor.set_block(block_type, x, 0, z, std::optional<std::vector<Block>>(),
				std::optional<std::vector<Block>>());
	}
}

}
}
