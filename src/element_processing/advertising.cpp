#include "advertising.h"
#include "../deterministic_rng.h"
#include <algorithm>

namespace arnis {
namespace advertising {

void generate_advertising(WorldEditor &editor, const ProcessedNode &node) {
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

    auto it_advertising = node.tags.find("advertising");
    if (it_advertising != node.tags.end()) {
        const std::string& advertising_type = it_advertising->second;
        if (advertising_type == "column") {
            generate_advertising_column(editor, node);
        } else if (advertising_type == "flag") {
            generate_advertising_flag(editor, node);
        } else if (advertising_type == "poster_box") {
            generate_poster_box(editor, node);
        }
    }
}

void generate_advertising_column(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Two green concrete blocks stacked
    editor.set_block(GREEN_CONCRETE, x, 1, z, std::nullopt, std::nullopt);
    editor.set_block(GREEN_CONCRETE, x, 2, z, std::nullopt, std::nullopt);

    // Stone brick slab on top
    editor.set_block(STONE_BRICK_SLAB, x, 3, z, std::nullopt, std::nullopt);
}

void generate_advertising_flag(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Use deterministic RNG for flag color
    std::mt19937 rng = element_rng(node.id);
    std::uniform_int_distribution<int> dist(0, 5);

    // Get height from tags or default
    int height = 6;
    auto it_height = node.tags.find("height");
    if (it_height != node.tags.end()) {
        try {
            height = std::stoi(it_height->second);
        } catch (...) {
            // ignore parse errors
        }
    }
    height = std::clamp(height, 4, 12);

    // Flagpole
    for (int y = 1; y <= height; ++y) {
        editor.set_block(IRON_BARS, x, y, z, std::nullopt, std::nullopt);
    }

    // Flag/banner at top (using colored wool)
    // Random bright advertising colors
    std::vector<Block> flag_colors = {
        RED_WOOL, YELLOW_WOOL, BLUE_WOOL, GREEN_WOOL, ORANGE_WOOL, WHITE_WOOL
    };
    Block flag_block = flag_colors[dist(rng)];

    // Flag extends to one side (2-3 blocks)
    int flag_length = 3;
    for (int dx = 1; dx <= flag_length; ++dx) {
        editor.set_block(flag_block, x + dx, height, z, std::nullopt, std::nullopt);
        editor.set_block(flag_block, x + dx, height - 1, z, std::nullopt, std::nullopt);
    }

    // Finial at top
    editor.set_block(IRON_BLOCK, x, height + 1, z, std::nullopt, std::nullopt);
}

void generate_poster_box(WorldEditor &editor, const ProcessedNode &node) {
    int x = node.x;
    int z = node.z;

    // Y=1: Two iron bars next to each other
    editor.set_block(IRON_BARS, x, 1, z, std::nullopt, std::nullopt);
    editor.set_block(IRON_BARS, x + 1, 1, z, std::nullopt, std::nullopt);

    // Y=2 and Y=3: Two sea lanterns
    editor.set_block(SEA_LANTERN, x, 2, z, std::nullopt, std::nullopt);
    editor.set_block(SEA_LANTERN, x + 1, 2, z, std::nullopt, std::nullopt);
    editor.set_block(SEA_LANTERN, x, 3, z, std::nullopt, std::nullopt);
    editor.set_block(SEA_LANTERN, x + 1, 3, z, std::nullopt, std::nullopt);

    // Y=4: Two polished stone brick slabs
    editor.set_block(STONE_BRICK_SLAB, x, 4, z, std::nullopt, std::nullopt);
    editor.set_block(STONE_BRICK_SLAB, x + 1, 4, z, std::nullopt, std::nullopt);
}

}
}
