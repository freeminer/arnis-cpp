#pragma once

#include <cstddef>
#include <vector>

namespace arnis::elevation
{

inline constexpr int MAX_Y = 319;
inline constexpr std::size_t MAX_ELEVATION_GRID_DIM = 4096;

struct ElevationData {
    std::vector<std::vector<double>> heights;
    std::size_t width{0};
    std::size_t height{0};
};

std::vector<std::vector<double>> gaussian_blur_grid(
        const std::vector<std::vector<double>> &grid, double sigma);

void fill_nan_values(std::vector<std::vector<double>> &heights);
void filter_elevation_outliers(std::vector<std::vector<double>> &heights);

}
