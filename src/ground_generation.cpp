#include "ground_generation.h"

#include "block_definitions.h"
#include "element_processing/tree.h"
#include "land_cover.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace arnis::ground_generation
{

namespace
{

Block rocky_surface_for(int x, int z)
{
    const auto h = land_cover::coord_hash(x, z) % 12;
    if (h <= 4)
        return STONE;
    if (h <= 6)
        return ANDESITE;
    if (h <= 8)
        return COBBLESTONE;
    if (h == 9)
        return GRAVEL;
    if (h == 10)
        return TUFF;
    return COARSE_DIRT;
}

int local_slope(WorldEditor &editor, int x, int z)
{
    const int center = editor.get_ground_level(x, z);
    int max_delta = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dz == 0)
                continue;
            max_delta = std::max(max_delta,
                    std::abs(editor.get_ground_level(x + dx, z + dz) - center));
        }
    }
    return max_delta;
}

bool has_nearby_water(WorldEditor &editor, int x, int ground_y, int z)
{
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dz = -2; dz <= 2; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (editor.check_for_block_absolute(
                            x + dx, ground_y + dy, z + dz,
                            std::optional<std::vector<Block>>(std::vector<Block>{WATER}))) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool is_protected_surface(WorldEditor &editor, int x, int y, int z)
{
    return editor.check_for_block_absolute(x, y, z,
            std::optional<std::vector<Block>>(std::vector<Block>{
                    BLACK_CONCRETE,
                    GRAY_CONCRETE_POWDER,
                    CYAN_TERRACOTTA,
                    GRAY_CONCRETE,
                    LIGHT_GRAY_CONCRETE,
                    WHITE_CONCRETE,
                    DIRT_PATH,
                    SMOOTH_STONE,
                    WATER,
            }));
}

bool is_replaceable_surface(WorldEditor &editor, int x, int y, int z)
{
    // Rust parity gap: full src/ground_generation.rs surface palette is not ported.
    // This keeps the C++ post-OSM ground pass from overwriting roads/buildings/water.
    return !editor.block_exists_absolute(x, y, z) ||
            editor.check_for_block_absolute(x, y, z,
                    std::optional<std::vector<Block>>(std::vector<Block>{
                            STONE,
                            DIRT,
                            GRASS_BLOCK,
                            GRASS,
                            SAND,
                            SANDSTONE,
                            GRAVEL,
                            CLAY,
                            COARSE_DIRT,
                            PODZOL,
                            MUD,
                            ANDESITE,
                            COBBLESTONE,
                            TUFF,
                            DEEPSLATE,
                            COBBLED_DEEPSLATE,
                    }));
}

Block natural_surface_for(WorldEditor &editor, int x, int ground_y, int z)
{
    const int slope = local_slope(editor, x, z);
    if (slope > 8)
        return rocky_surface_for(x, z);
    if (slope > 4)
        return STONE;
    if (has_nearby_water(editor, x, ground_y, z))
        return SAND;

    const auto h = land_cover::coord_hash(x, z) % 100;
    if (h < 2)
        return COARSE_DIRT;
    if (h < 5)
        return PODZOL;
    return GRASS_BLOCK;
}

void maybe_place_vegetation(WorldEditor &editor, int x, int ground_y, int z,
        const BuildingFootprintBitmap &building_footprints)
{
    if (building_footprints.contains(x, z) ||
            editor.check_for_block_absolute(x, ground_y + 1, z))
        return;
    if (!editor.check_for_block_absolute(x, ground_y, z,
                std::optional<std::vector<Block>>(std::vector<Block>{GRASS_BLOCK, PODZOL})))
        return;

    const auto h = land_cover::coord_hash(x, z) % 1000;
    if (h == 0) {
        Tree::create(editor, Coord{x, 1, z}, &building_footprints);
    } else if (h < 12) {
        editor.set_block_absolute(OAK_LEAVES, x, ground_y + 1, z, std::nullopt, std::nullopt);
    } else if (h < 26) {
        const Block flower = h % 4 == 0 ? RED_FLOWER :
                h % 4 == 1 ? BLUE_FLOWER :
                h % 4 == 2 ? YELLOW_FLOWER : WHITE_FLOWER;
        editor.set_block_absolute(flower, x, ground_y + 1, z, std::nullopt, std::nullopt);
    } else if (h < 180) {
        editor.set_block_absolute(GRASS, x, ground_y + 1, z, std::nullopt, std::nullopt);
    }
}

}

void generate_ground_layer(
        WorldEditor &editor,
        const Args &args,
        const XZBBox &xzbbox,
        const BuildingFootprintBitmap &building_footprints)
{
    // Rust parity: src/ground_generation.rs::generate_ground_layer ordering.
    // C++ uses a conservative surface pass plus LC_WATER pre-paint for water_depth.
    for (int x = xzbbox.min_x(); x <= xzbbox.max_x(); ++x) {
        for (int z = xzbbox.min_z(); z <= xzbbox.max_z(); ++z) {
            const int ground_y = editor.get_ground_level(x, z);

            if (editor.is_lc_water(x, z)) {
                const int water_y = editor.get_water_level(x, z);
                if (ground_y <= water_y && !is_protected_surface(editor, x, water_y, z)) {
                    editor.set_block_absolute(WATER, x, water_y, z, std::nullopt, std::nullopt);
                    if (water_y - 1 > -64)
                        editor.set_block_absolute(SAND, x, water_y - 1, z,
                                std::nullopt, std::nullopt);
                    if (water_y - 2 > -64)
                        editor.set_block_absolute(SANDSTONE, x, water_y - 2, z,
                                std::nullopt, std::nullopt);
                }
                continue;
            }

            if (!is_protected_surface(editor, x, ground_y, z) &&
                    is_replaceable_surface(editor, x, ground_y, z)) {
                const Block surface = natural_surface_for(editor, x, ground_y, z);
                editor.set_block_absolute(surface, x, ground_y, z, std::nullopt, std::nullopt);

                if (surface == SAND) {
                    editor.set_block_absolute(SANDSTONE, x, ground_y - 1, z,
                            std::nullopt, std::nullopt);
                } else if (surface == STONE || surface == ANDESITE ||
                        surface == COBBLESTONE || surface == TUFF) {
                    editor.set_block_absolute(STONE, x, ground_y - 1, z,
                            std::nullopt, std::nullopt);
                } else {
                    editor.set_block_absolute(DIRT, x, ground_y - 1, z,
                            std::nullopt, std::nullopt);
                }
            }

            maybe_place_vegetation(editor, x, ground_y, z, building_footprints);

            if (args.fillground) {
                const int min_fill_y = std::max(-64, ground_y - 32);
                for (int y = min_fill_y; y < ground_y; ++y) {
                    if (!editor.check_for_block_absolute(x, y, z))
                        editor.set_block_absolute(y < ground_y - 8 ? STONE : DIRT,
                                x, y, z, std::nullopt, std::nullopt);
                }
                editor.set_block_absolute(BEDROCK, x, min_fill_y - 1, z,
                        std::nullopt, std::nullopt);
            }
        }
    }
}

}
