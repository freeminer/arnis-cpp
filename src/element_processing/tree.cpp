#include "tree.h"
#include "../deterministic_rng.h"
#include <algorithm>

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

static const std::array<std::pair<Coord, Coord>, 5> CHERRY_LEAVES_FILL = {{
    { { -1, 4, 0 }, { -1, 9, 0 } },
    { {  1, 4, 0 }, {  1, 9, 0 } },
    { {  0, 4, -1 }, {  0, 9, -1 } },
    { {  0, 4, 1 }, {  0, 9, 1 } },
    { {  0, 9, 0 }, {  0, 10, 0 } },
}};

static const std::array<std::pair<Coord, Coord>, 5> TALL_OAK_LEAVES_FILL = {{
    { { -1, 8, 0 }, { -1, 12, 0 } },
    { {  1, 8, 0 }, {  1, 12, 0 } },
    { {  0, 8, -1 }, {  0, 12, -1 } },
    { {  0, 8, 1 }, {  0, 12, 1 } },
    { {  0, 12, 0 }, {  0, 13, 0 } },
}};

static const std::array<std::pair<Coord, Coord>, 6> PINE_LEAVES_FILL = {{
    { { -1, 5, 0 }, { -1, 12, 0 } },
    { {  0, 5, -1 }, {  0, 12, -1 } },
    { {  1, 5, 0 }, {  1, 12, 0 } },
    { {  0, 5, -1 }, {  0, 12, -1 } },
    { {  0, 5, 1 }, {  0, 12, 1 } },
    { {  0, 13, 0 }, {  0, 13, 0 } },
}};

constexpr int MAX_CANOPY_RADIUS = 3;

static void fill_blocks_absolute(WorldEditor& editor, const Block& block,
        int x1, int y1, int z1, int x2, int y2, int z2)
{
    auto [min_x, max_x] = std::minmax(x1, x2);
    auto [min_y, max_y] = std::minmax(y1, y2);
    auto [min_z, max_z] = std::minmax(z1, z2);
    for (int x = min_x; x <= max_x; ++x)
        for (int y = min_y; y <= max_y; ++y)
            for (int z = min_z; z <= max_z; ++z)
                editor.set_block_absolute(block, x, y, z, std::nullopt, std::nullopt);
}

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

        case TreeType::Cherry: {
            Tree t;
            t.log_block = CHERRY_LOG;
            t.log_height = 7;
            t.leaves_block = CHERRY_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(CHERRY_LEAVES_FILL);
            for (int v = 9; v >= 4; --v) t.round_ranges[0].push_back(v);
            for (int v = 8; v >= 5; --v) t.round_ranges[1].push_back(v);
            for (int v = 7; v >= 6; --v) t.round_ranges[2].push_back(v);
            return t;
        }

        case TreeType::TallOak: {
            Tree t;
            t.log_block = OAK_LOG;
            t.log_height = 11;
            t.leaves_block = OAK_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(TALL_OAK_LEAVES_FILL);
            for (int v = 12; v >= 8; --v) t.round_ranges[0].push_back(v);
            for (int v = 11; v >= 9; --v) t.round_ranges[1].push_back(v);
            t.round_ranges[2] = std::vector<int>{10};
            return t;
        }

        case TreeType::Pine: {
            Tree t;
            t.log_block = SPRUCE_LOG;
            t.log_height = 12;
            t.leaves_block = SPRUCE_LEAVES;
            t.leaves_fill = std::span<const std::pair<Coord, Coord>>(PINE_LEAVES_FILL);
            t.round_ranges[0] = std::vector<int>{11, 9, 7, 5};
            t.round_ranges[1] = std::vector<int>{8, 5};
            t.round_ranges[2] = std::vector<int>{};
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
    std::uniform_int_distribution<int> dist(1, 13);
    int pick = dist(rng);

    TreeType chosen = TreeType::Oak;
    if (pick >= 1 && pick <= 3) chosen = TreeType::Oak;
    else if (pick >= 4 && pick <= 5) chosen = TreeType::Spruce;
    else if (pick >= 6 && pick <= 7) chosen = TreeType::Birch;
    else if (pick == 8) chosen = TreeType::DarkOak;
    else if (pick == 9) chosen = TreeType::Jungle;
    else if (pick == 10) chosen = TreeType::Acacia;
    else if (pick == 11) chosen = TreeType::Cherry;
    else if (pick == 12) chosen = TreeType::TallOak;
    else if (pick == 13) chosen = TreeType::Pine;

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
        GRAY_CONCRETE_POWDER,
        CYAN_TERRACOTTA,
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

bool Tree::canopy_might_intersect_building(
        int x, int z, const BuildingFootprintBitmap* building_footprints)
{
    if (!building_footprints)
        return false;
    for (int check_x = x - MAX_CANOPY_RADIUS; check_x <= x + MAX_CANOPY_RADIUS; ++check_x)
        for (int check_z = z - MAX_CANOPY_RADIUS; check_z <= z + MAX_CANOPY_RADIUS; ++check_z)
            if (building_footprints->contains(check_x, check_z))
                return true;
    return false;
}

void Tree::create_of_type(WorldEditor& editor, const Coord& pos, TreeType tree_type, const BuildingFootprintBitmap* building_footprints) {
    // Skip if this coordinate is inside a building
    if (building_footprints != nullptr) {
        if (building_footprints->contains(pos.x, pos.z)) {
            return;
        }
    }

    const std::optional<std::vector<Block>> protected_surface_blocks(
            std::vector<Block>{
                BLACK_CONCRETE,
                GRAY_CONCRETE_POWDER,
                CYAN_TERRACOTTA,
                GRAY_CONCRETE,
                LIGHT_GRAY_CONCRETE,
                DIRT_PATH,
                SMOOTH_STONE,
                WATER,
            });
    if (editor.check_for_block(pos.x, 0, pos.z, protected_surface_blocks))
        return;

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
    const bool check_canopy_collision =
            canopy_might_intersect_building(pos.x, pos.z, building_footprints);
    const int base_y = editor.get_absolute_y(pos.x, pos.y, pos.z);

    // Build the logs
    fill_blocks_absolute(editor, tree.log_block, pos.x, base_y, pos.z,
            pos.x, base_y + tree.log_height, pos.z);

    // Fill in the leaves
    for (const auto& pr : tree.leaves_fill) {
        const Coord& a = pr.first;
        const Coord& b = pr.second;
        if (check_canopy_collision) {
            for (int leaf_x = pos.x + a.x; leaf_x <= pos.x + b.x; ++leaf_x) {
                for (int leaf_y = base_y + a.y; leaf_y <= base_y + b.y; ++leaf_y) {
                    for (int leaf_z = pos.z + a.z; leaf_z <= pos.z + b.z; ++leaf_z) {
                        if (!building_footprints || !building_footprints->contains(leaf_x, leaf_z))
                            editor.set_block_absolute(tree.leaves_block, leaf_x, leaf_y, leaf_z,
                                    std::nullopt, std::nullopt);
                    }
                }
            }
        } else {
            fill_blocks_absolute(editor, tree.leaves_block,
                    pos.x + a.x, base_y + a.y, pos.z + a.z,
                    pos.x + b.x, base_y + b.y, pos.z + b.z);
        }
    }

    // Do the three rounds
    for (std::size_t idx = 0; idx < tree.round_ranges.size(); ++idx) {
        const std::vector<int>& range = tree.round_ranges[idx];
        std::span<const Coord> pattern = ROUND_PATTERNS[idx];
        for (int offset : range) {
            if (check_canopy_collision) {
                for (const Coord& d : pattern) {
                    int leaf_x = pos.x + d.x;
                    int leaf_z = pos.z + d.z;
                    if (!building_footprints || !building_footprints->contains(leaf_x, leaf_z)) {
                        editor.set_block_absolute(tree.leaves_block, leaf_x,
                                base_y + offset + d.y, leaf_z, std::nullopt, std::nullopt);
                    }
                }
            } else {
                Coord center { pos.x, base_y + offset, pos.z };
                round_absolute(editor, tree.leaves_block, center, pattern);
            }
        }
    }
}

}
