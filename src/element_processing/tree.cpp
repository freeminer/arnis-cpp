#include "tree.h"
#include "../deterministic_rng.h"

namespace arnis {

// Additional leaves fill patterns for new tree types
static const std::array<std::pair<Coord, Coord>, 5> DARK_OAK_LEAVES_FILL = {{
    { { -1, 3, 0 }, { -1, 6, 0 } },
    { {  1, 3, 0 }, {  1, 6, 0 } },
    { {  0, 3, -1 }, {  0, 6, -1 } },
    { {  0, 3, 1 }, {  0, 6, 1 } },
    { {  0, 6, 0 }, {  0, 7, 0 } },
}};

static const std::array<std::pair<Coord, Coord>, 5> JUNGLE_LEAVES_FILL = {{
    { { -1, 7, 0 }, { -1, 11, 0 } },
    { {  1, 7, 0 }, {  1, 11, 0 } },
    { {  0, 7, -1 }, {  0, 11, -1 } },
    { {  0, 7, 1 }, {  0, 11, 1 } },
    { {  0, 11, 0 }, {  0, 12, 0 } },
}};

static const std::array<std::pair<Coord, Coord>, 5> ACACIA_LEAVES_FILL = {{
    { { -1, 5, 0 }, { -1, 8, 0 } },
    { {  1, 5, 0 }, {  1, 8, 0 } },
    { {  0, 5, -1 }, {  0, 8, -1 } },
    { {  0, 5, 1 }, {  0, 8, 1 } },
    { {  0, 8, 0 }, {  0, 9, 0 } },
}};

Tree Tree::get_tree(TreeType kind) {
    switch (kind) {
        case TreeType::Oak: {
            Tree t;
            t.log_block = OAK_LOG;
            t.log_height = 8;
            t.leaves_block = OAK_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(OAK_LEAVES_FILL);
            t.round_ranges[0].reserve(6);
            for (int v = 8; v >= 3; --v) t.round_ranges[0].push_back(v);
            t.round_ranges[1].reserve(4);
            for (int v = 7; v >= 4; --v) t.round_ranges[1].push_back(v);
            t.round_ranges[2].reserve(2);
            for (int v = 6; v >= 5; --v) t.round_ranges[2].push_back(v);
            return t;
        }

        case TreeType::Spruce: {
            Tree t;
            t.log_block = SPRUCE_LOG;
            t.log_height = 9;
            t.leaves_block = SPRUCE_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(SPRUCE_LEAVES_FILL);
            t.round_ranges[0] = std::vector<int>{9, 7, 6, 4, 3};
            t.round_ranges[1] = std::vector<int>{6, 3};
            t.round_ranges[2] = std::vector<int>{};
            return t;
        }

        case TreeType::Birch: {
            Tree t;
            t.log_block = BIRCH_LOG;
            t.log_height = 6;
            t.leaves_block = BIRCH_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(BIRCH_LEAVES_FILL);
            t.round_ranges[0].reserve(5);
            for (int v = 6; v >= 2; --v) t.round_ranges[0].push_back(v);
            t.round_ranges[1].reserve(3);
            for (int v = 2; v <= 4; ++v) t.round_ranges[1].push_back(v);
            t.round_ranges[2] = std::vector<int>{};
            return t;
        }
        
        case TreeType::DarkOak: {
            Tree t;
            t.log_block = DARK_OAK_LOG;
            t.log_height = 5;
            t.leaves_block = DARK_OAK_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(DARK_OAK_LEAVES_FILL);
            // All 3 round patterns used for maximum width
            t.round_ranges[0].reserve(4);
            for (int v = 6; v >= 3; --v) t.round_ranges[0].push_back(v);
            t.round_ranges[1].reserve(3);
            for (int v = 5; v >= 3; --v) t.round_ranges[1].push_back(v);
            t.round_ranges[2].reserve(2);
            for (int v = 5; v >= 4; --v) t.round_ranges[2].push_back(v);
            return t;
        }
        
        case TreeType::Jungle: {
            Tree t;
            t.log_block = JUNGLE_LOG;
            t.log_height = 10;
            t.leaves_block = JUNGLE_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(JUNGLE_LEAVES_FILL);
            // Canopy only near the top of the tree
            t.round_ranges[0].reserve(5);
            for (int v = 11; v >= 7; --v) t.round_ranges[0].push_back(v);
            t.round_ranges[1].reserve(3);
            for (int v = 10; v >= 8; --v) t.round_ranges[1].push_back(v);
            t.round_ranges[2] = std::vector<int>{};
            return t;
        }
        
        case TreeType::Acacia: {
            Tree t;
            t.log_block = ACACIA_LOG;
            t.log_height = 6;
            t.leaves_block = ACACIA_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(ACACIA_LEAVES_FILL);
            // Inner rounds reach higher → gentle dome, outer stays low → wide brim
            t.round_ranges[0].reserve(4);
            for (int v = 8; v >= 5; --v) t.round_ranges[0].push_back(v);
            t.round_ranges[1].reserve(3);
            for (int v = 7; v >= 5; --v) t.round_ranges[1].push_back(v);
            t.round_ranges[2].reserve(2);
            for (int v = 7; v >= 6; --v) t.round_ranges[2].push_back(v);
            return t;
        }
    }
    // fallback (should not happen)
    return Tree{};
}

void Tree::create(WorldEditor& editor, const Coord& pos, const BuildingFootprintBitmap* building_footprints) {
    // Skip if this coordinate is inside a building
    if (building_footprints != nullptr) {
        if (building_footprints->
            
            
            contains(pos.x, pos.z)) {
            return;
        }
    }
    
    // Use deterministic RNG based on coordinates for consistent tree types across region boundaries
    // The element_id of 0 is used as a salt for tree-specific randomness
    std::mt19937 rng = coord_rng(pos.x, pos.z, 0);
    std::uniform_int_distribution<int> dist(1, 10);
    int pick = dist(rng);

    TreeType chosen = TreeType::Oak;
    if (pick >= 1 && pick <= 3) chosen = TreeType::Oak;
    else if (pick >= 4 && pick <= 5) chosen = TreeType::Spruce;
    else if (pick >= 6 && pick <= 7) chosen = TreeType::Birch;
    else if (pick == 8) chosen = TreeType::DarkOak;
    else if (pick == 9) chosen = TreeType::Jungle;
    else if (pick == 10) chosen = TreeType::Acacia;

    create_of_type(editor, pos, chosen, building_footprints);
}

std::vector<Block> Tree::get_building_wall_blocks() {
    return std::vector<Block>{
        BLACKSTONE,
        BLACK_TERRACOTTA,
        BRICK,
        BROWN_CONCRETE,
        BROWN_TERRACOTTA,
        DEEPSLATE_BRICKS,
        END_STONE_BRICKS,
        GRAY_CONCRETE,
        GRAY_TERRACOTTA,
        LIGHT_BLUE_TERRACOTTA,
        LIGHT_GRAY_CONCRETE,
        MUD_BRICKS,
        NETHER_BRICK,
        NETHERITE_BLOCK,
        POLISHED_ANDESITE,
        POLISHED_BLACKSTONE,
        POLISHED_BLACKSTONE_BRICKS,
        POLISHED_DEEPSLATE,
        POLISHED_GRANITE,
        QUARTZ_BLOCK,
        QUARTZ_BRICKS,
        SANDSTONE,
        SMOOTH_SANDSTONE,
        SMOOTH_STONE,
        STONE_BRICKS,
        WHITE_CONCRETE,
        WHITE_TERRACOTTA,
        ORANGE_TERRACOTTA,
        GREEN_STAINED_HARDENED_CLAY,
        BLUE_TERRACOTTA,
        YELLOW_TERRACOTTA,
        BLACK_CONCRETE,
        WHITE_CONCRETE,
        GRAY_CONCRETE,
        LIGHT_GRAY_CONCRETE,
        BROWN_CONCRETE,
        RED_CONCRETE,
        ORANGE_TERRACOTTA,
        YELLOW_CONCRETE,
        LIME_CONCRETE,
        GREEN_STAINED_HARDENED_CLAY,
        CYAN_CONCRETE,
        LIGHT_BLUE_CONCRETE,
        BLUE_CONCRETE,
        PURPLE_CONCRETE,
        MAGENTA_CONCRETE,
        RED_TERRACOTTA,
    };
}

std::vector<Block> Tree::get_building_floor_blocks() {
    return std::vector<Block>{
        GRAY_CONCRETE,
        LIGHT_GRAY_CONCRETE,
        WHITE_CONCRETE,
        SMOOTH_STONE,
        POLISHED_ANDESITE,
        STONE_BRICKS,
    };
}

std::vector<Block> Tree::get_structural_blocks() {
    return std::vector<Block>{
        // Fences
        OAK_FENCE,
        // Walls
        COBBLESTONE_WALL,
        ANDESITE_WALL,
        STONE_BRICK_WALL,
        // Stairs
        OAK_STAIRS,
        // Slabs
        OAK_SLAB,
        STONE_BLOCK_SLAB,
        STONE_BRICK_SLAB,
        // Rails
        RAIL,
        RAIL_NORTH_SOUTH,
        RAIL_EAST_WEST,
        RAIL_ASCENDING_EAST,
        RAIL_ASCENDING_WEST,
        RAIL_ASCENDING_NORTH,
        RAIL_ASCENDING_SOUTH,
        RAIL_NORTH_EAST,
        RAIL_NORTH_WEST,
        RAIL_SOUTH_EAST,
        RAIL_SOUTH_WEST,
        // Doors and trapdoors
        OAK_DOOR,
        DARK_OAK_DOOR_LOWER,
        DARK_OAK_DOOR_UPPER,
        OAK_TRAPDOOR,
        // Ladders
        LADDER,
    };
}

std::vector<Block> Tree::get_functional_blocks() {
    return std::vector<Block>{
        // Furniture and functional blocks
        CHEST,
        CRAFTING_TABLE,
        FURNACE,
        ANVIL,
        BREWING_STAND,
        NOTE_BLOCK,
        BOOKSHELF,
        CAULDRON,
        // Beds
        RED_BED_NORTH_HEAD,
        RED_BED_NORTH_FOOT,
        RED_BED_EAST_HEAD,
        RED_BED_EAST_FOOT,
        RED_BED_SOUTH_HEAD,
        RED_BED_SOUTH_FOOT,
        RED_BED_WEST_HEAD,
        RED_BED_WEST_FOOT,
        // Pressure plates and signs
        OAK_PRESSURE_PLATE,
        SIGN,
        // Glass blocks (windows)
        GLASS,
        WHITE_STAINED_GLASS,
        GRAY_STAINED_GLASS,
        LIGHT_GRAY_STAINED_GLASS,
        BROWN_STAINED_GLASS,
        CYAN_STAINED_GLASS,
        BLUE_STAINED_GLASS,
        LIGHT_BLUE_STAINED_GLASS,
        TINTED_GLASS,
        // Carpets
        WHITE_CARPET,
        RED_CARPET,
        // Other structural/building blocks
        IRON_BARS,
        IRON_BLOCK,
        SCAFFOLDING,
        BEDROCK,
    };
}

void Tree::create_of_type(WorldEditor& editor, const Coord& pos, TreeType tree_type, const BuildingFootprintBitmap* building_footprints) {
    // Skip if this coordinate is inside a building
    if (building_footprints != nullptr) {
        if (building_footprints->contains(pos.x, pos.z)) {
            return;
        }
    }

    std::vector<Block> blacklist;
    auto bw = get_building_wall_blocks();
    blacklist.insert(blacklist.end(), bw.begin(), bw.end());
    auto bf = get_building_floor_blocks();
    blacklist.insert(blacklist.end(), bf.begin(), bf.end());
    auto sb = get_structural_blocks();
    blacklist.insert(blacklist.end(), sb.begin(), sb.end());
    auto fb = get_functional_blocks();
    blacklist.insert(blacklist.end(), fb.begin(), fb.end());
    blacklist.push_back(WATER);

    Tree tree = get_tree(tree_type);

    // Build the logs
    editor.fill_blocks(
        tree.log_block,
        pos.x,
        pos.y,
        pos.z,
        pos.x,
        pos.y + tree.log_height,
        pos.z,
        std::nullopt,
        std::optional<std::reference_wrapper<const std::vector<Block>>>(std::cref(blacklist))
    );

    // Fill in the leaves
    for (const auto& pr : tree.leaves_fill) {
        const Coord& a = pr.first;
        const Coord& b = pr.second;
        editor.fill_blocks(
            tree.leaves_block,
            pos.x + a.x,
            pos.y + a.y,
            pos.z + a.z,
            pos.x + b.x,
            pos.y + b.y,
            pos.z + b.z,
            std::nullopt,
            std::nullopt
        );
    }

    // Do the three rounds
    for (std::size_t idx = 0; idx < tree.round_ranges.size(); ++idx) {
        const std::vector<int>& range = tree.round_ranges[idx];
        std::span<const Coord> pattern = ROUND_PATTERNS[idx];
        for (int offset : range) {
            Coord center { pos.x, pos.y + offset, pos.z };
            round(editor, tree.leaves_block, center, pattern);
        }
    }
}

}