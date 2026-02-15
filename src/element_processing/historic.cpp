#include "historic.h"
#include "../deterministic_rng.h"
#include "../floodfill.h"
#include <algorithm>
#include <cmath>

namespace arnis {
namespace historic {

void generate_historic(WorldEditor &editor, const ProcessedNode &node) {
    // Skip if 'layer' or 'level' is negative in the tags
    auto it_layer = node.tags.find("layer");
    if (it_layer != node.tags.end()) {
        try {
            if (std::stoi(it_layer->second) < 0) {
                return;
            }
        } catch (...) {
            // ignore parse errors
        }
    }

    auto it_level = node.tags.find("level");
    if (it_level != node.tags.end()) {
        try {
            if (std::stoi(it_level->second) < 0) {
                return;
            }
        } catch (...) {
            // ignore parse errors
        }
    }

    auto it_historic = node.tags.find("historic");
    if (it_historic != node.tags.end()) {
        const std::string& historic_type = it_historic->second;
        if (historic_type == "memorial") {
            generate_memorial(editor, node);
        } else if (historic_type == "monument") {
            generate_monument(editor, node);
        } else if (historic_type == "wayside_cross") {
            generate_wayside_cross(editor, node);
        }
    }
}

void generate_memorial(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Use deterministic RNG for consistent results
    std::mt19937 rng = element_rng(node.id);
    std::uniform_int_distribution<int> dist(0, 1);

    // Get memorial subtype
    auto it_memorial = node.tags.find("memorial");
    std::string memorial_type = (it_memorial != node.tags.end()) ? it_memorial->second : "yes";

    if (memorial_type == "plaque") {
        // Simple plaque on a small stand
        editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_SLAB, x, 2, z, std::nullopt, std::nullopt);
    } else if (memorial_type == "statue" || memorial_type == "sculpture" || memorial_type == "bust") {
        // Statue on a pedestal
        editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);
        editor.set_block(CHISELED_STONE_BRICKS, x, 2, z, std::nullopt, std::nullopt);

        // Use polished andesite for bronze/metal statue appearance
        Block statue_block = (dist(rng) == 0) ? POLISHED_ANDESITE : POLISHED_DIORITE;
        editor.set_block(statue_block, x, 3, z, std::nullopt, std::nullopt);
        editor.set_block(statue_block, x, 4, z, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_WALL, x, 5, z, std::nullopt, std::nullopt);
    } else if (memorial_type == "stone" || memorial_type == "stolperstein") {
        // Simple memorial stone embedded in ground
        Block stone_block = (memorial_type == "stolperstein") ? GOLD_BLOCK : STONE;
        editor.set_block(stone_block, x, 0, z, std::nullopt, std::nullopt);
    } else if (memorial_type == "cross" || memorial_type == "war_memorial") {
        // Memorial cross
        generate_cross(editor, x, z, 5);
    } else if (memorial_type == "obelisk") {
        // Tall pointed pillar with fixed height
        // Base layer at Y=1
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dz = -1; dz <= 1; ++dz) {
                editor.set_block(STONE_BRICKS, x + dx, 1, z + dz, std::nullopt, std::nullopt);
            }
        }

        // Second base layer at Y=2
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dz = -1; dz <= 1; ++dz) {
                editor.set_block(STONE_BRICKS, x + dx, 2, z + dz, std::nullopt, std::nullopt);
            }
        }
        // Stone brick slabs on the 4 corners at Y=3 (on top of corner blocks)
        editor.set_block(STONE_BRICK_SLAB, x - 1, 3, z - 1, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_SLAB, x + 1, 3, z - 1, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_SLAB, x - 1, 3, z + 1, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_SLAB, x + 1, 3, z + 1, std::nullopt, std::nullopt);

        // Main shaft, fixed height of 4 blocks (Y=3 to Y=6)
        for (int y = 3; y <= 6; ++y) {
            editor.set_block(SMOOTH_QUARTZ, x, y, z, std::nullopt, std::nullopt);
        }

        editor.set_block(STONE_BRICK_SLAB, x, 7, z, std::nullopt, std::nullopt);
    } else if (memorial_type == "stele") {
        // Upright stone slab
        // Base
        editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);

        // Upright slab (using wall blocks for thin appearance)
        for (int y = 2; y <= 4; ++y) {
            editor.set_block(STONE_BRICK_WALL, x, y, z, std::nullopt, std::nullopt);
        }
        editor.set_block(STONE_BRICK_SLAB, x, 5, z, std::nullopt, std::nullopt);
    } else {
        // Default: simple stone pillar monument
        editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICKS, x, 2, z, std::nullopt, std::nullopt);
        editor.set_block(CHISELED_STONE_BRICKS, x, 3, z, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_SLAB, x, 4, z, std::nullopt, std::nullopt);
    }
}

void generate_monument(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Monuments are typically larger structures
    int height = 10;
    auto it_height = node.tags.find("height");
    if (it_height != node.tags.end()) {
        try {
            height = std::stoi(it_height->second);
        } catch (...) {
            // ignore parse errors
        }
    }
    height = std::max(5, std::min(20, height));

    // Large base platform
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dz = -2; dz <= 2; ++dz) {
            editor.set_block(STONE_BRICKS, x + dx, 1, z + dz, std::nullopt, std::nullopt);
        }
    }
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dz = -1; dz <= 1; ++dz) {
            editor.set_block(STONE_BRICKS, x + dx, 2, z + dz, std::nullopt, std::nullopt);
        }
    }

    // Main structure
    for (int y = 3; y < height; ++y) {
        editor.set_block(POLISHED_ANDESITE, x, y, z, std::nullopt, std::nullopt);
    }

    // Decorative top
    editor.set_block(CHISELED_STONE_BRICKS, x, height, z, std::nullopt, std::nullopt);
}

void generate_wayside_cross(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Simple roadside cross
    generate_cross(editor, x, z, 4);
}

void generate_cross(WorldEditor &editor, int x, int z, int height) {
    // Base
    editor.set_block(STONE_BRICKS, x, 1, z, std::nullopt, std::nullopt);

    // Vertical beam
    for (int y = 2; y <= height; ++y) {
        editor.set_block(STONE_BRICK_WALL, x, y, z, std::nullopt, std::nullopt);
    }

    // Horizontal beam (cross arm) at approximately 2/3 height, but at least 2 and at most height-1
    int arm_y = std::max(2, std::min(height - 1, (height * 2 + 2) / 3));

    // Only place horizontal arms if height allows for them (height >= 3)
    if (height >= 3) {
        editor.set_block(STONE_BRICK_WALL, x - 1, arm_y, z, std::nullopt, std::nullopt);
        editor.set_block(STONE_BRICK_WALL, x + 1, arm_y, z, std::nullopt, std::nullopt);
    }
}

void generate_pyramid(WorldEditor &editor, const ProcessedWay &element, const Args &args) {
    if (element.nodes.size() < 3) {
        return;
    }

    // Convert nodes to polygon coordinates
    std::vector<std::pair<int,int>> polygon_coords;
    polygon_coords.reserve(element.nodes.size());
    for (const auto& n : element.nodes) {
        polygon_coords.emplace_back(n.x, n.z);
    }

    // Get the footprint via flood fill
    std::vector<std::pair<int, int>> footprint = flood_fill_area(
        polygon_coords, args.timeout_ref());

    if (footprint.empty()) {
        return;
    }

    // Determine base Y from terrain or ground level
    // Use the MINIMUM ground level so the pyramid sits on the lowest point
    // and doesn't float in areas with elevation differences
    int base_y = args.ground_level;
    if (args.terrain) {
        int min_ground_level = args.ground_level;
        for (const auto& p : footprint) {
            int ground_level = editor.ground->level(XZPoint(p.first, p.second));
            if (ground_level < min_ground_level) {
                min_ground_level = ground_level;
            }
        }
        base_y = min_ground_level;
    }

    // Bounding box of the footprint
    int min_x = footprint[0].first;
    int max_x = footprint[0].first;
    int min_z = footprint[0].second;
    int max_z = footprint[0].second;

    for (const auto& p : footprint) {
        if (p.first < min_x) min_x = p.first;
        if (p.first > max_x) max_x = p.first;
        if (p.second < min_z) min_z = p.second;
        if (p.second > max_z) max_z = p.second;
    }

    double center_x = (min_x + max_x) / 2.0;
    double center_z = (min_z + max_z) / 2.0;

    // The pyramid height is half the shorter side of the bounding box (classic proportions)
    double width = max_x - min_x + 1;
    double length = max_z - min_z + 1;
    double half_base = std::min(width, length) / 2.0;
    // Height = half the shorter side (classic pyramid proportions).
    // Footprint is already in scaled Minecraft coordinates, so no extra scale factor needed.
    int pyramid_height = std::max(3, static_cast<int>(half_base));

    // Build the pyramid layer by layer.
    // For each layer, only place blocks whose Chebyshev distance from the
    // footprint centre is within the shrinking radius AND that were in the
    // original footprint.
    std::optional<int> last_placed_layer;
    for (int layer = 0; layer < pyramid_height; ++layer) {
        // The allowed radius shrinks linearly from half_base at layer 0 to 0
        double radius = half_base * (1.0 - static_cast<double>(layer) / static_cast<double>(pyramid_height));
        if (radius < 0.0) {
            break;
        }

        int y = base_y + 1 + layer;
        bool placed = false;

        for (const auto& p : footprint) {
            int px = p.first;
            int pz = p.second;
            double dx = std::abs(px - center_x);
            double dz = std::abs(pz - center_z);

            // Use Chebyshev distance (max of dx, dz) for a square-footprint pyramid
            if (dx <= radius && dz <= radius) {
                // Allow overwriting common terrain blocks so the pyramid is
                // solid even when it intersects higher ground.
                std::vector<Block> whitelist = {
                    GRASS_BLOCK, DIRT, STONE, SAND, GRAVEL, COARSE_DIRT, PODZOL, DIRT_PATH, SANDSTONE
                };
                editor.set_block_absolute(SANDSTONE, px, y, pz, std::optional<std::vector<Block>>(whitelist), std::nullopt);
                placed = true;
            }
        }

        if (placed) {
            last_placed_layer = y;
        } else {
            break; // Nothing placed, we've reached the apex
        }
    }

    // Cap with smooth sandstone one block above the last placed layer
    if (last_placed_layer.has_value()) {
        std::vector<Block> whitelist = {
            GRASS_BLOCK, DIRT, STONE, SAND, GRAVEL, COARSE_DIRT, PODZOL, DIRT_PATH, SANDSTONE
        };
        editor.set_block_absolute(SMOOTH_SANDSTONE, 
                                 static_cast<int>(std::round(center_x)), 
                                 last_placed_layer.value() + 1, 
                                 static_cast<int>(std::round(center_z)),
                                 std::optional<std::vector<Block>>(whitelist), 
                                 std::nullopt);
    }
}

}
}