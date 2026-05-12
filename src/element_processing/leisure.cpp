#include <string>
#include <tuple>
#include <utility>
#include <optional>
#include <random>
#include <algorithm>

// Project headers
#include "../args.h"
#include "block_definitions.h"
#include "bresenham.h"
#include "tree.h"
#include "floodfill.h"
#include "../osm_parser.h"
#include "world_editor.h"
#include "surfaces.h"
#include "../../../arnis_adapter.h"
namespace arnis
{

namespace leisure
{


void generate_leisure(WorldEditor& editor, const ProcessedWay& element, const Args& args, 
                     FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints) {
    auto leisure_it = element.tags.find(std::string("leisure"));
    if (leisure_it != element.tags.end()) {
        const std::string& leisure_type = leisure_it->second;
        std::optional<std::pair<int, int>> previous_node = std::nullopt;
        std::tuple<int, int, int> corner_addup = std::make_tuple(0, 0, 0);
        std::vector<std::pair<int, int>> current_leisure;

        // Use deterministic RNG seeded by element ID for consistent results across region boundaries
        std::mt19937 rng(element.id);

        Block block_type = GRASS_BLOCK;
        if (leisure_type == "park" || leisure_type == "nature_reserve" || leisure_type == "garden" ||
            leisure_type == "disc_golf_course" || leisure_type == "golf_course") {
            block_type = GRASS_BLOCK;
        } else if (leisure_type == "schoolyard") {
            block_type = BLACK_CONCRETE;
        } else if (leisure_type == "playground" || leisure_type == "recreation_ground" ||
                   leisure_type == "pitch" || leisure_type == "beach_resort" || leisure_type == "dog_park") {
            block_type = GREEN_STAINED_HARDENED_CLAY;
        } else if (leisure_type == "swimming_pool" || leisure_type == "swimming_area") {
            block_type = WATER;
        } else if (leisure_type == "bathing_place") {
            block_type = SMOOTH_SANDSTONE;
        } else if (leisure_type == "outdoor_seating") {
            block_type = SMOOTH_STONE;
        } else if (leisure_type == "water_park" || leisure_type == "slipway") {
            block_type = LIGHT_GRAY_CONCRETE;
        } else if (leisure_type == "ice_rink") {
            block_type = PACKED_ICE;
        } else {
            block_type = GRASS_BLOCK;
        }

        if (auto surf_it = element.tags.find(std::string("surface")); surf_it != element.tags.end()) {
            if (const auto* surface_blocks = surfaces::get_blocks_for_surface(surf_it->second);
                    surface_blocks && !surface_blocks->empty()) {
                block_type = surface_blocks->front();
            }
        }

        for (const ProcessedNode& node : element.nodes) {
            if (previous_node.has_value()) {
                std::pair<int,int> prev = previous_node.value();
                std::vector<std::tuple<int,int,int>> bresenham_points =
                    bresenham_line(prev.first, 0, prev.second, node.x, 0, node.z);
                for (const std::tuple<int,int,int>& t : bresenham_points) {
                    int bx = std::get<0>(t);
                    int bz = std::get<2>(t);
                    editor.set_block(block_type, bx, 0, bz,
                                     std::optional<std::vector<Block>>({
                                         GRASS_BLOCK, STONE_BRICKS, SMOOTH_STONE,
                                         LIGHT_GRAY_CONCRETE, COBBLESTONE, GRAY_CONCRETE
                                     }),
                                     std::nullopt);
                }

                current_leisure.push_back(std::make_pair(node.x, node.z));
                std::get<0>(corner_addup) += node.x;
                std::get<1>(corner_addup) += node.z;
                std::get<2>(corner_addup) += 1;
            }
            previous_node = std::make_pair(node.x, node.z);
        }

        if (corner_addup != std::make_tuple(0, 0, 0)) {
            std::vector<std::pair<int,int>> polygon_coords;
            polygon_coords.reserve(element.nodes.size());
            for (const ProcessedNode& n : element.nodes) {
                polygon_coords.push_back(std::make_pair(n.x, n.z));
            }

            std::vector<std::pair<int,int>> filled_area =
                    flood_fill_cache.get_or_compute(element, args.timeout);

            for (const std::pair<int,int>& p : filled_area) {
                int x = p.first;
                int z = p.second;
                editor.set_block(block_type, x, 0, z, std::optional<std::vector<Block>>({GRASS_BLOCK}), std::nullopt);

                // Add decorative elements for parks and gardens
                if ((leisure_type == "park" || leisure_type == "garden" || leisure_type == "nature_reserve") &&
                    editor.check_for_block(x, 0, z, std::optional<std::vector<Block>>({GRASS_BLOCK}))) {

                    std::uniform_int_distribution<int> dist(0, 999);
                    int random_choice = dist(rng);

                    if (random_choice >= 0 && random_choice < 30) {
                        // Plants
                        Block plant_choice = WHITE_FLOWER;
                        if (random_choice < 5) {
                            plant_choice = RED_FLOWER;
                        } else if (random_choice < 10) {
                            plant_choice = YELLOW_FLOWER;
                        } else if (random_choice < 16) {
                            plant_choice = BLUE_FLOWER;
                        } else if (random_choice < 22) {
                            plant_choice = WHITE_FLOWER;
                        } else {
                            plant_choice = GRASS; // Use GRASS instead of FERN since FERN is not defined
                        }
                        editor.set_block(plant_choice, x, 1, z, std::nullopt, std::nullopt);
                    } else if (random_choice >= 30 && random_choice < 90) {
                        // Grass
                        editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
                    } else if (random_choice >= 90 && random_choice < 105) {
                        // Oak leaves
                        editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
                    } else if (random_choice >= 105 && random_choice < 120) {
                        // Tree
                        Tree::create(editor, {x, 1, z});
                    }
                }

                // Add playground or recreation ground features
                if (leisure_type == "playground" || leisure_type == "recreation_ground") {
                    std::uniform_int_distribution<int> dist2(0, 4999);
                    int rc = dist2(rng);

                    if (rc >= 0 && rc < 10) {
                        // Swing set
                        for (int y = 1; y <= 3; ++y) {
                            editor.set_block(OAK_FENCE, x - 1, y, z, std::nullopt, std::nullopt);
                            editor.set_block(OAK_FENCE, x + 1, y, z, std::nullopt, std::nullopt);
                        }
                        editor.set_block(OAK_PLANKS, x - 1, 4, z, std::nullopt, std::nullopt);
                        editor.set_block(OAK_SLAB, x, 4, z, std::nullopt, std::nullopt);
                        editor.set_block(OAK_PLANKS, x + 1, 4, z, std::nullopt, std::nullopt);
                        editor.set_block(STONE_BLOCK_SLAB, x, 2, z, std::nullopt, std::nullopt);
                    } else if (rc >= 10 && rc < 20) {
                        // Slide
                        editor.set_block(OAK_SLAB, x, 1, z, std::nullopt, std::nullopt);
                        editor.set_block(OAK_SLAB, x + 1, 2, z, std::nullopt, std::nullopt);
                        editor.set_block(OAK_SLAB, x + 2, 3, z, std::nullopt, std::nullopt);

                        editor.set_block(OAK_PLANKS, x + 2, 2, z, std::nullopt, std::nullopt);
                        editor.set_block(OAK_PLANKS, x + 2, 1, z, std::nullopt, std::nullopt);

                        editor.set_block(LADDER, x + 2, 2, z - 1, std::nullopt, std::nullopt);
                        editor.set_block(LADDER, x + 2, 1, z - 1, std::nullopt, std::nullopt);
                    } else if (rc >= 20 && rc < 30) {
                        // Sandpit
                        editor.fill_blocks(SAND, x - 3, 0, z - 3, x + 3, 0, z + 3,
                                           std::optional<std::vector<Block>>({GREEN_STAINED_HARDENED_CLAY}),
                                           std::nullopt);
                    }
                }
            }
        }
    }
}

void generate_leisure_from_relation(WorldEditor& editor, const ProcessedRelation& rel, const Args& args, 
                                   FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints) {
    auto leisure_it = rel.tags.find(std::string("leisure"));
    if (leisure_it != rel.tags.end() && leisure_it->second == "park") {
        // Process each outer member way individually using cached flood fill.
        // We intentionally do not combine all outer nodes into one mega-way,
        // because that creates a nonsensical polygon spanning the whole relation
        // extent, misses the flood fill cache, and can cause multi-GB allocations.
        for (const ProcessedMember& member : rel.members) {
            if (member.role == ProcessedMemberRole::Outer) {
                // Use relation tags so the member inherits the relation's leisure=* type
                ProcessedWay way_with_rel_tags;
                way_with_rel_tags.id = member.way.id;
                way_with_rel_tags.nodes = member.way.nodes;
                way_with_rel_tags.tags = rel.tags;
                generate_leisure(editor, way_with_rel_tags, args, flood_fill_cache, building_footprints);
            }
        }
    }
}

        
}

}
