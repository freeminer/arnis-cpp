#pragma once

#include <cstdint>
#include <vector>

namespace arnis::land_cover
{

inline constexpr uint8_t LC_TREE_COVER = 10;
inline constexpr uint8_t LC_SHRUBLAND = 20;
inline constexpr uint8_t LC_GRASSLAND = 30;
inline constexpr uint8_t LC_CROPLAND = 40;
inline constexpr uint8_t LC_BUILT_UP = 50;
inline constexpr uint8_t LC_BARE = 60;
inline constexpr uint8_t LC_SNOW_ICE = 70;
inline constexpr uint8_t LC_WATER = 80;
inline constexpr uint8_t LC_WETLAND = 90;
inline constexpr uint8_t LC_MANGROVES = 95;
inline constexpr uint8_t LC_MOSS = 100;

struct LandCoverData {
    std::vector<std::vector<uint8_t>> grid;
    std::vector<std::vector<uint8_t>> water_distance;
    std::vector<std::vector<float>> water_blend_grid;
    std::size_t width{0};
    std::size_t height{0};

    void refresh_water_blend_grid();
};

uint64_t coord_hash(int32_t x, int32_t z);

std::vector<std::vector<uint8_t>> compute_water_distance(
        const std::vector<std::vector<uint8_t>> &grid,
        std::size_t width,
        std::size_t height);

std::vector<std::vector<float>> compute_water_blend_smooth(
        const std::vector<std::vector<uint8_t>> &grid,
        std::size_t width,
        std::size_t height);

}
