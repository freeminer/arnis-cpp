#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <random>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <cstdint>

#include "../../../arnis_adapter.h"
#include "tree.h"
#include "bridges.h"
#include "../floodfill.h"
#include "../ground_generation.h"
#include "../deterministic_rng.h"
namespace arnis
{

namespace natural
{


uint64_t coord_hash(int x, int z)
{
	uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32;
	h ^= static_cast<uint32_t>(z);
	h += 0x9e3779b97f4a7c15ULL;
	h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
	h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
	return h ^ (h >> 31);
}

Block vary_rock_block(Block base, int x, int z)
{
	const uint64_t h = coord_hash(x, z) % 10;
	if (base == STONE) {
		if (h <= 4)
			return STONE;
		if (h <= 6)
			return ANDESITE;
		if (h == 7)
			return COBBLESTONE;
		return GRAVEL;
	}
	if (base == COBBLESTONE) {
		if (h <= 4)
			return COBBLESTONE;
		if (h <= 6)
			return ANDESITE;
		if (h == 7)
			return STONE;
		return GRAVEL;
	}
	return base;
}

namespace
{
bool wetland_wet_zone(int x, int z)
{
	return ground_generation::value_noise_01(x + 11, z + 7, 28) > 0.55;
}

bool wetland_puddle_noise(int x, int z)
{
	return ground_generation::value_noise_01(x + 31, z + 17, 6) > 0.78;
}

bool try_place_wetland_puddle(WorldEditor &editor, int x, int z)
{
	const std::optional<std::vector<Block>> ground{std::vector<Block>{MUD, GRASS_BLOCK}};
	if (!editor.check_for_block(x, 0, z, ground))
		return false;
	editor.set_block(WATER, x, 0, z, ground, std::nullopt);
	return true;
}

void place_grass_or_tall(WorldEditor &editor, ChaCha8Rng &rng, int x, int z)
{
	const int r = static_cast<int>(rng.uniform(100));
	if (r < 10) {
		editor.set_block(TALL_GRASS_BOTTOM, x, 1, z, std::nullopt, std::nullopt);
		editor.set_block(TALL_GRASS_TOP, x, 2, z, std::nullopt, std::nullopt);
	} else if (r < 25) {
		editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
	}
}

std::uint64_t cell_key(int x, int z)
{
	return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
}
} // namespace

// Generate natural area for single element
//static
void generate_natural(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	const auto &tags = element.tags();
	auto it_nat = tags.find("natural");
	if (it_nat == tags.end())
		return;
	const std::string natural_type = it_nat->second;

	if (natural_type == "tree") {
		if (element.is_node()) {
			int x = element.as_node().x;
			int z = element.as_node().z;

			// Use deterministic RNG seeded by element ID for consistent results across region boundaries
			// Rust uses element_rng(), not the implementation-defined mt19937
			// sequence.  Keep selections stable across C++ standard libraries.
			auto rng = element_rng(static_cast<std::uint64_t>(element.id()));

			// Determine tree type based on tags
			std::vector<TreeType> trees_ok_to_generate;

			// Check species tag
			auto it_species = tags.find("species");
			if (it_species != tags.end()) {
				const std::string &species = it_species->second;
				if (species.find("Betula") != std::string::npos) {
					trees_ok_to_generate.push_back(TreeType::Birch);
				}
				if (species.find("Quercus") != std::string::npos) {
					trees_ok_to_generate.push_back(TreeType::Oak);
				}
				if (species.find("Picea") != std::string::npos) {
					trees_ok_to_generate.push_back(TreeType::Spruce);
				}
			} else {
				// Check genus:wikidata tag
				auto it_genus_wikidata = tags.find("genus:wikidata");
				if (it_genus_wikidata != tags.end()) {
					const std::string &genus_wikidata = it_genus_wikidata->second;
					if (genus_wikidata == "Q12004") {
						trees_ok_to_generate.push_back(TreeType::Birch);
					} else if (genus_wikidata == "Q26782") {
						trees_ok_to_generate.push_back(TreeType::Oak);
					} else if (genus_wikidata == "Q25243") {
						trees_ok_to_generate.push_back(TreeType::Spruce);
					} else {
						trees_ok_to_generate.push_back(TreeType::Oak);
						trees_ok_to_generate.push_back(TreeType::Spruce);
						trees_ok_to_generate.push_back(TreeType::Birch);
						trees_ok_to_generate.push_back(TreeType::TallOak);
						trees_ok_to_generate.push_back(TreeType::Pine);
					}
				} else {
					// Check genus tag
					auto it_genus = tags.find("genus");
					if (it_genus != tags.end()) {
						const std::string &genus = it_genus->second;
						if (genus == "Betula") {
							trees_ok_to_generate.push_back(TreeType::Birch);
						} else if (genus == "Quercus") {
							trees_ok_to_generate.push_back(TreeType::Oak);
						} else if (genus == "Picea") {
							trees_ok_to_generate.push_back(TreeType::Spruce);
						} else {
							trees_ok_to_generate.push_back(TreeType::Oak);
						}
					} else {
						// Check leaf_type tag
						auto it_leaf_type = tags.find("leaf_type");
						if (it_leaf_type != tags.end()) {
							const std::string &leaf_type = it_leaf_type->second;
							if (leaf_type == "broadleaved") {
								trees_ok_to_generate.push_back(TreeType::Oak);
								trees_ok_to_generate.push_back(TreeType::Birch);
								trees_ok_to_generate.push_back(TreeType::TallOak);
							} else if (leaf_type == "needleleaved") {
								trees_ok_to_generate.push_back(TreeType::Spruce);
								trees_ok_to_generate.push_back(TreeType::Pine);
							} else {
								trees_ok_to_generate.push_back(TreeType::Oak);
								trees_ok_to_generate.push_back(TreeType::Spruce);
								trees_ok_to_generate.push_back(TreeType::Birch);
								trees_ok_to_generate.push_back(TreeType::TallOak);
								trees_ok_to_generate.push_back(TreeType::Pine);
							}
						} else {
							trees_ok_to_generate.push_back(TreeType::Oak);
							trees_ok_to_generate.push_back(TreeType::Spruce);
							trees_ok_to_generate.push_back(TreeType::Birch);
							trees_ok_to_generate.push_back(TreeType::TallOak);
						}
					}
				}
			}

			// Ensure we have at least one tree type
			if (trees_ok_to_generate.empty()) {
				trees_ok_to_generate.push_back(TreeType::Oak);
				trees_ok_to_generate.push_back(TreeType::Spruce);
				trees_ok_to_generate.push_back(TreeType::Birch);
			}

			// Select a random tree type
			TreeType tree_type = trees_ok_to_generate[rng.uniform(
					static_cast<std::uint32_t>(trees_ok_to_generate.size()))];

			// Create the tree
			Tree::create_of_type(editor, Coord{x, 1, z}, tree_type, &building_footprints,
					&bridge_surface, true);
		}
		return;
	}

	// Use deterministic RNG seeded by element ID for consistent results across region boundaries
	auto rng = element_rng(static_cast<std::uint64_t>(element.id()));

	// Determine block type based on natural tag
	Block block_type = GRASS_BLOCK;
	if (natural_type == "scrub" || natural_type == "grassland" ||
			natural_type == "wood" || natural_type == "heath" ||
			natural_type == "tree_row") {
		block_type = GRASS_BLOCK;
	} else if (natural_type == "sand" || natural_type == "dune") {
		block_type = SAND;
	} else if (natural_type == "beach" || natural_type == "shoal") {
		auto it_surface = tags.find("natural");
		std::string surface =
				(it_surface == tags.end()) ? std::string() : it_surface->second;
		if (surface == "gravel")
			block_type = GRAVEL;
		else
			block_type = SAND;
	} else if (natural_type == "water" || natural_type == "reef" ||
			   natural_type == "bay") {
		block_type = WATER;
	} else if (natural_type == "bare_rock") {
		block_type = STONE;
	} else if (natural_type == "blockfield") {
		block_type = COBBLESTONE;
	} else if (natural_type == "glacier") {
		block_type = PACKED_ICE;
	} else if (natural_type == "mud" || natural_type == "wetland") {
		block_type = MUD;
	} else if (natural_type == "mountain_range") {
		block_type = COBBLESTONE;
	} else if (natural_type == "saddle" || natural_type == "ridge") {
		block_type = STONE;
	} else if (natural_type == "shrubbery" || natural_type == "tundra" ||
			   natural_type == "hill") {
		block_type = GRASS_BLOCK;
	} else if (natural_type == "cliff") {
		block_type = STONE;
	} else {
		block_type = GRASS_BLOCK;
	}
	const bool rock_variation = natural_type == "blockfield" || natural_type == "cliff" ||
								natural_type == "saddle" || natural_type == "ridge" ||
								natural_type == "mountain_range";

	const std::optional<std::vector<Block>> protected_surface_blocks(std::vector<Block>{
			BLACK_CONCRETE,
			GRAY_CONCRETE_POWDER,
			CYAN_TERRACOTTA,
			GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE,
			WHITE_CONCRETE,
			DIRT_PATH,
			SMOOTH_STONE,
	});

	if (element.type != ProcessedElement::Type::Way) {
		return;
	}
	const ProcessedWay &way = element.as_way();
	// Resolve before the edge pass.  A closed ring that the capped fill
	// refuses must not leave an outline around ground it cannot fill.
	auto filled_area = flood_fill_cache.get_or_compute(way, args.timeout);
	if (filled_area.empty() && is_oversized_ring(way))
		return;

	std::optional<std::pair<int, int>> previous_node;
	std::tuple<int, int, int> corner_addup = std::make_tuple(0, 0, 0);
	std::vector<std::pair<int, int>> current_natural;

	for (const auto &node : way.nodes) {
		int x = node.x;
		int z = node.z;
		if (previous_node.has_value()) {
			auto prev = previous_node.value();
			std::vector<std::tuple<int, int, int>> bres =
					bresenham_line(prev.first, 0, prev.second, x, 0, z);
			for (const auto &t : bres) {
				int bx = std::get<0>(t);
				int bz = std::get<2>(t);
				if (!editor.check_for_block(bx, 0, bz, protected_surface_blocks)) {
					Block block = rock_variation ? vary_rock_block(block_type, bx, bz)
												 : block_type;
					editor.set_block(block, bx, 0, bz, std::nullopt, std::nullopt);
				}
			}
			current_natural.emplace_back(x, z);
			std::get<0>(corner_addup) += x;
			std::get<1>(corner_addup) += z;
			std::get<2>(corner_addup) += 1;
		}
		previous_node = std::make_pair(x, z);
	}

	if (std::get<2>(corner_addup) != 0) {
		std::vector<std::pair<int, int>> polygon_coords;
		polygon_coords.reserve(way.nodes.size());
		for (const auto &n : way.nodes) {
			polygon_coords.emplace_back(n.x, n.z);
		}

		// Determine tree types for wood/tree_row areas
		std::vector<TreeType> trees_ok_to_generate;
		auto it_leaf_type = tags.find("leaf_type");
		if (it_leaf_type != tags.end()) {
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
			}
		} else {
			trees_ok_to_generate.push_back(TreeType::Oak);
			trees_ok_to_generate.push_back(TreeType::Spruce);
			trees_ok_to_generate.push_back(TreeType::Birch);
			trees_ok_to_generate.push_back(TreeType::TallOak);
			trees_ok_to_generate.push_back(TreeType::Bush);
			trees_ok_to_generate.push_back(TreeType::AzaleaBush);
		}

		std::optional<std::vector<Block>> protected_fill_blocks(std::vector<Block>{
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

		std::vector<std::pair<int, int>> wetland_puddles;
		for (const auto &p : filled_area) {
			int x = p.first;
			int z = p.second;
			if (!editor.check_for_block(x, 0, z, protected_fill_blocks)) {
				Block block =
						rock_variation ? vary_rock_block(block_type, x, z) : block_type;
				editor.set_block(block, x, 0, z, std::nullopt, std::nullopt);
			}

			// Generate custom layer instead of dirt, must be stone on the lowest level
			if (natural_type == "beach" || natural_type == "sand" ||
					natural_type == "dune" || natural_type == "shoal") {
				editor.set_block(SAND, x, 0, z, std::nullopt, std::nullopt);
			} else if (natural_type == "glacier") {
				editor.set_block(PACKED_ICE, x, 0, z, std::nullopt, std::nullopt);
				editor.set_block(STONE, x, -1, z, std::nullopt, std::nullopt);
			} else if (natural_type == "bare_rock") {
				const uint64_t h = coord_hash(x, z) % 12;
				Block rock = STONE;
				if (h <= 4)
					rock = STONE;
				else if (h <= 6)
					rock = ANDESITE;
				else if (h <= 8)
					rock = COBBLESTONE;
				else if (h == 9)
					rock = GRAVEL;
				else if (h == 10)
					rock = TUFF;
				else
					rock = COARSE_DIRT;
				editor.set_block(rock, x, 0, z, std::nullopt, std::nullopt);
			}

			// Generate surface elements
			if (editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{std::vector<Block>{WATER}})) {
				continue;
			}

			if (natural_type == "grassland") {
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				if (rng.random_bool(.6)) {
					editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
				}
			} else if (natural_type == "heath") {
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				int random_choice = static_cast<int>(rng.uniform(500));
				if (random_choice < 33) {
					if (random_choice <= 2) {
						editor.set_block(
								COBBLESTONE, x, 0, z, std::nullopt, std::nullopt);
					} else if (random_choice < 6) {
						editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
					} else {
						editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
					}
				}
			} else if (natural_type == "scrub") {
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				int random_choice = static_cast<int>(rng.uniform(500));
				if (random_choice == 0) {
					Tree::create(
							editor, {x, 1, z}, &building_footprints, &bridge_surface);
				} else if (random_choice == 1) {
					int f = 1 + static_cast<int>(rng.uniform(4));
					Block flower_block = RED_FLOWER;
					if (f == 1)
						flower_block = RED_FLOWER;
					else if (f == 2)
						flower_block = BLUE_FLOWER;
					else if (f == 3)
						flower_block = YELLOW_FLOWER;
					else
						flower_block = WHITE_FLOWER;
					editor.set_block(flower_block, x, 1, z, std::nullopt, std::nullopt);
				} else if (random_choice < 40) {
					editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
					if (random_choice < 15) {
						editor.set_block(OAK_LEAVES, x, 2, z, std::nullopt, std::nullopt);
					}
				} else if (random_choice < 300) {
					if (random_choice < 250) {
						editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
					} else {
						editor.set_block(
								TALL_GRASS_BOTTOM, x, 1, z, std::nullopt, std::nullopt);
						editor.set_block(
								TALL_GRASS_TOP, x, 2, z, std::nullopt, std::nullopt);
					}
				}
			} else if (natural_type == "tree_row" || natural_type == "wood") {
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				const double density = ground_generation::value_noise_01(x, z, 32);
				const int tree_threshold = std::max(5, int(60.0 - density * 45.0));
				int random_choice = static_cast<int>(rng.uniform(30));
				if (rng.uniform(static_cast<std::uint32_t>(tree_threshold)) == 0) {
					// Select a random tree type from the approved list
					if (!trees_ok_to_generate.empty()) {
						TreeType tree_type = trees_ok_to_generate[rng.uniform(
								static_cast<std::uint32_t>(trees_ok_to_generate.size()))];
						Tree::create_of_type(editor, {x, 1, z}, tree_type,
								&building_footprints, &bridge_surface);
					} else {
						Tree::create(
								editor, {x, 1, z}, &building_footprints, &bridge_surface);
					}
				} else if (random_choice == 1) {
					int f = 1 + static_cast<int>(rng.uniform(4));
					Block flower_block = RED_FLOWER;
					if (f == 1)
						flower_block = RED_FLOWER;
					else if (f == 2)
						flower_block = BLUE_FLOWER;
					else if (f == 3)
						flower_block = YELLOW_FLOWER;
					else
						flower_block = WHITE_FLOWER;
					editor.set_block(flower_block, x, 1, z, std::nullopt, std::nullopt);
				} else if (random_choice <= 12) {
					editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
				}
			} else if (natural_type == "sand") {
				if (editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{SAND}}) &&
						rng.uniform(100) == 1) {
					editor.set_block(DEAD_BUSH, x, 1, z, std::nullopt, std::nullopt);
				}
			} else if (natural_type == "shoal") {
				if (rng.random_bool(.05)) {
					editor.set_block(WATER, x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{SAND, GRAVEL}},
							std::nullopt);
				}
			} else if (natural_type == "wetland") {
				const auto wet_it = tags.find("wetland");
				const std::string wetland_type =
						wet_it == tags.end() ? "" : wet_it->second;
				if (wetland_type == "wet_meadow" || wetland_type == "fen") {
					if (rng.random_bool(.3))
						editor.set_block(GRASS_BLOCK, x, 0, z, std::vector<Block>{MUD},
								std::nullopt);
					editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
					continue;
				}
				if (wetland_type == "tidalflat") {
					if (rng.random_bool(.3))
						editor.set_block(
								WATER, x, 0, z, std::vector<Block>{MUD}, std::nullopt);
					continue;
				}
				const bool wet = wetland_wet_zone(x, z);
				if (wet && wetland_puddle_noise(x, z)) {
					if (try_place_wetland_puddle(editor, x, z))
						wetland_puddles.emplace_back(x, z);
					continue;
				}
				if (wet) {
					if (ground_generation::value_noise_01(x + 53, z + 71, 8) > .55)
						editor.set_block(COARSE_DIRT, x, 0, z, std::vector<Block>{MUD},
								std::nullopt);
				} else if (rng.random_bool(.4)) {
					editor.set_block(
							GRASS_BLOCK, x, 0, z, std::vector<Block>{MUD}, std::nullopt);
				}
				if (!editor.check_for_block(x, 0, z,
							std::vector<Block>{
									MUD, MOSS_BLOCK, COARSE_DIRT, GRASS_BLOCK, DIRT}))
					continue;
				if (wetland_type == "reedbed") {
					if (rng.uniform(100) < 45) {
						editor.set_block(
								TALL_GRASS_BOTTOM, x, 1, z, std::nullopt, std::nullopt);
						editor.set_block(
								TALL_GRASS_TOP, x, 2, z, std::nullopt, std::nullopt);
					}
				} else if (wetland_type == "swamp" || wetland_type == "mangrove") {
					const int r = static_cast<int>(rng.uniform(40));
					if (r == 0) {
						const TreeType type =
								wetland_type == "mangrove"
										? TreeType::Mangrove
										: (rng.random_bool(.6) ? TreeType::Willow
															   : TreeType::Mangrove);
						Tree::create_of_type(editor, {x, 1, z}, type,
								&building_footprints, &bridge_surface);
					} else if (r < 15)
						place_grass_or_tall(editor, rng, x, z);
				} else if (wetland_type == "bog") {
					if (rng.random_bool(.2))
						editor.set_block(MOSS_BLOCK, x, 0, z, std::vector<Block>{MUD},
								std::nullopt);
					if (rng.random_bool(.08))
						place_grass_or_tall(editor, rng, x, z);
				} else {
					place_grass_or_tall(editor, rng, x, z);
				}
			} else if (natural_type == "mountain_range") {
				// Create block clusters instead of random placement
				int cluster_chance = static_cast<int>(rng.uniform(1000));

				if (cluster_chance < 50) {
					// 5% chance to start a new cluster
					int cb = static_cast<int>(rng.uniform(7));
					Block cluster_block = DIRT;
					if (cb == 0)
						cluster_block = DIRT;
					else if (cb == 1)
						cluster_block = STONE;
					else if (cb == 2)
						cluster_block = GRAVEL;
					else if (cb == 3)
						cluster_block = GRANITE;
					else if (cb == 4)
						cluster_block = DIORITE;
					else if (cb == 5)
						cluster_block = ANDESITE;
					else
						cluster_block = GRASS_BLOCK;

					// Generate cluster size (5-10 blocks radius)
					int cluster_size = 5 + static_cast<int>(rng.uniform(6));

					// Create cluster around current position
					for (int dx = -cluster_size; dx <= cluster_size; ++dx) {
						for (int dz = -cluster_size; dz <= cluster_size; ++dz) {
							int cluster_x = x + dx;
							int cluster_z = z + dz;

							// Use distance to create more natural cluster shape
							float distance = std::sqrt(float(dx * dx + dz * dz));
							if (distance <= static_cast<float>(cluster_size)) {
								// Probability decreases with distance from center
								float place_prob =
										1.0f -
										(distance / static_cast<float>(cluster_size));
								if (static_cast<float>(rng()) / 4294967296.0f <
										place_prob) {
									editor.set_block(cluster_block, cluster_x, 0,
											cluster_z, std::nullopt, std::nullopt);

									// Add vegetation on grass blocks
									if (cluster_block == GRASS_BLOCK) {
										int vegetation_chance =
												static_cast<int>(rng.uniform(100));
										if (vegetation_chance == 0) {
											// 1% chance for rare trees
											Tree::create(editor,
													{cluster_x, 1, cluster_z},
													&building_footprints,
													&bridge_surface);
										} else if (vegetation_chance < 15) {
											// 15% chance for grass
											editor.set_block(GRASS, cluster_x, 1,
													cluster_z, std::nullopt,
													std::nullopt);
										} else if (vegetation_chance < 25) {
											// 10% chance for oak leaves
											editor.set_block(OAK_LEAVES, cluster_x, 1,
													cluster_z, std::nullopt,
													std::nullopt);
										}
									}
								}
							}
						}
					}
				}
			} else if (natural_type == "saddle") {
				// Saddle areas - lowest point between peaks, mix of stone and grass
				int terrain_chance = static_cast<int>(rng.uniform(100));
				if (terrain_chance < 30) {
					// 30% chance for exposed stone
					editor.set_block(STONE, x, 0, z, std::nullopt, std::nullopt);
				} else if (terrain_chance < 50) {
					// 20% chance for gravel/rocky terrain
					editor.set_block(GRAVEL, x, 0, z, std::nullopt, std::nullopt);
				} else {
					// 50% chance for grass
					editor.set_block(GRASS_BLOCK, x, 0, z, std::nullopt, std::nullopt);
					if (rng.random_bool(.4)) {
						// 40% chance for grass on top
						editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
					}
				}
			} else if (natural_type == "ridge") {
				// Ridge areas - elevated crest, mostly rocky with some vegetation
				int ridge_chance = static_cast<int>(rng.uniform(100));
				if (ridge_chance < 60) {
					// 60% chance for stone/rocky terrain
					int rock = static_cast<int>(rng.uniform(4));
					Block rock_type = STONE;
					if (rock == 0)
						rock_type = STONE;
					else if (rock == 1)
						rock_type = COBBLESTONE;
					else if (rock == 2)
						rock_type = GRANITE;
					else
						rock_type = ANDESITE;
					editor.set_block(rock_type, x, 0, z, std::nullopt, std::nullopt);
				} else {
					// 40% chance for grass with sparse vegetation
					editor.set_block(GRASS_BLOCK, x, 0, z, std::nullopt, std::nullopt);
					int vegetation_chance = static_cast<int>(rng.uniform(100));
					if (vegetation_chance < 20) {
						// 20% chance for grass
						editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
					} else if (vegetation_chance < 25) {
						// 5% chance for small shrubs
						editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
					}
				}
			} else if (natural_type == "shrubbery") {
				// Manicured shrubs and decorative vegetation
				editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
				editor.set_block(OAK_LEAVES, x, 2, z, std::nullopt, std::nullopt);
			} else if (natural_type == "tundra") {
				// Treeless habitat with low vegetation, mosses, lichens
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				int tundra_chance = static_cast<int>(rng.uniform(100));
				if (tundra_chance < 40) {
					// 40% chance for grass (sedges, grasses)
					editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
				} else if (tundra_chance < 60) {
					// 20% chance for moss
					editor.set_block(MOSS_BLOCK, x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}},
							std::nullopt);
				} else if (tundra_chance < 70) {
					// 10% chance for dead bush (lichens)
					editor.set_block(DEAD_BUSH, x, 1, z, std::nullopt, std::nullopt);
				}
				// 30% chance for bare ground (no surface block)
			} else if (natural_type == "cliff") {
				// Cliff areas - predominantly stone with minimal vegetation
				int cliff_chance = static_cast<int>(rng.uniform(100));
				if (cliff_chance < 90) {
					// 90% chance for stone variants
					int stone_type_choice = static_cast<int>(rng.uniform(4));
					Block stone_type = STONE;
					if (stone_type_choice == 0)
						stone_type = STONE;
					else if (stone_type_choice == 1)
						stone_type = COBBLESTONE;
					else if (stone_type_choice == 2)
						stone_type = ANDESITE;
					else
						stone_type = DIORITE;
					editor.set_block(stone_type, x, 0, z, std::nullopt, std::nullopt);
				} else {
					// 10% chance for gravel/loose rock
					editor.set_block(GRAVEL, x, 0, z, std::nullopt, std::nullopt);
				}
			} else if (natural_type == "hill") {
				// Hill areas - elevated terrain with sparse trees and mostly grass
				if (!editor.check_for_block(x, 0, z,
							std::optional<std::vector<Block>>{
									std::vector<Block>{GRASS_BLOCK}})) {
					continue;
				}
				int hill_chance = static_cast<int>(rng.uniform(1000));
				if (hill_chance == 0) {
					// 0.1% chance for rare trees
					Tree::create(
							editor, {x, 1, z}, &building_footprints, &bridge_surface);
				} else if (hill_chance < 50) {
					// 5% chance for flowers
					int f = 1 + static_cast<int>(rng.uniform(4));
					Block flower_block = RED_FLOWER;
					if (f == 1)
						flower_block = RED_FLOWER;
					else if (f == 2)
						flower_block = BLUE_FLOWER;
					else if (f == 3)
						flower_block = YELLOW_FLOWER;
					else
						flower_block = WHITE_FLOWER;
					editor.set_block(flower_block, x, 1, z, std::nullopt, std::nullopt);
				} else if (hill_chance < 600) {
					// 55% chance for grass
					editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
				} else if (hill_chance < 650) {
					// 5% chance for tall grass
					editor.set_block(
							TALL_GRASS_BOTTOM, x, 1, z, std::nullopt, std::nullopt);
					editor.set_block(TALL_GRASS_TOP, x, 2, z, std::nullopt, std::nullopt);
				}
				// 35% chance for bare grass block
			}
		}
		if (!wetland_puddles.empty()) {
			std::unordered_set<std::uint64_t> area;
			area.reserve(filled_area.size() * 2);
			for (const auto &[ax, az] : filled_area)
				area.insert(cell_key(ax, az));
			for (const auto &[px, pz] : wetland_puddles)
				for (int dx = -2; dx <= 2; ++dx)
					for (int dz = -2; dz <= 2; ++dz) {
						const int d = std::max(std::abs(dx), std::abs(dz));
						const int nx = px + dx, nz = pz + dz;
						if (d == 0 || !area.contains(cell_key(nx, nz)))
							continue;
						if (d == 1)
							editor.set_block(MOSS_BLOCK, nx, 0, nz,
									std::vector<Block>{
											MUD, GRASS_BLOCK, DIRT, COARSE_DIRT},
									std::nullopt);
						else
							editor.set_block(COARSE_DIRT, nx, 0, nz,
									std::vector<Block>{MUD, GRASS_BLOCK, DIRT},
									std::nullopt);
					}
			for (const auto &[px, pz] : wetland_puddles)
				for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
							 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
					const int nx = px + dx, nz = pz + dz;
					if (!area.contains(cell_key(nx, nz)) ||
							coord_hash(nx + 89, nz + 97) % 100 >= 20 ||
							!editor.check_for_block(nx, 0, nz,
									std::vector<Block>{GRASS_BLOCK, MUD, DIRT,
											COARSE_DIRT, MOSS_BLOCK}))
						continue;
					const int height = 1 + int(coord_hash(nx + 131, nz + 137) % 3);
					for (int y = 1; y <= height; ++y) {
						// Match Rust's block_at guard: reeds never replace a
						// structure or float through an occupied column.
						if (editor.block_at(nx, y, nz))
							break;
						editor.set_block(
								SUGAR_CANE, nx, y, nz, std::nullopt, std::nullopt);
					}
				}
		}
	}
}

// Generate natural from relation
//static
void generate_natural_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	if (rel.tags.find("natural") == rel.tags.end())
		return;

	for (const auto &member : rel.members) {
		if (member.role == ProcessedMemberRole::Outer) {
			ProcessedWay inherited = member.way;
			inherited.tags = rel.tags;
			ProcessedElement elem = ProcessedElement::FromWay(inherited);
			generate_natural(editor, elem, args, flood_fill_cache, building_footprints,
					bridge_surface);
		}
	}
}

} // namespace osm
}
