#include "elevation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace arnis::elevation
{

static std::vector<double> gaussian_kernel(double sigma)
{
    const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0)));
    std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
    double sum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        double v = std::exp(-(static_cast<double>(i * i)) / (2.0 * sigma * sigma));
        kernel[static_cast<std::size_t>(i + radius)] = v;
        sum += v;
    }
    for (double &v : kernel)
        v /= sum;
    return kernel;
}

std::vector<std::vector<double>> gaussian_blur_grid(
        const std::vector<std::vector<double>> &grid, double sigma)
{
    if (grid.empty() || grid.front().empty())
        return {};
    const std::size_t height = grid.size();
    const std::size_t width = grid.front().size();
    const auto kernel = gaussian_kernel(sigma);
    const int radius = static_cast<int>(kernel.size() / 2);

    std::vector<std::vector<double>> tmp(height, std::vector<double>(width, 0.0));
    for (std::size_t z = 0; z < height; ++z) {
        for (std::size_t x = 0; x < width; ++x) {
            double sum = 0.0;
            double weight = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                int sx = static_cast<int>(x) + k;
                if (sx < 0 || sx >= static_cast<int>(width))
                    continue;
                double v = grid[z][static_cast<std::size_t>(sx)];
                if (!std::isfinite(v))
                    continue;
                double w = kernel[static_cast<std::size_t>(k + radius)];
                sum += v * w;
                weight += w;
            }
            tmp[z][x] = weight > 0.0 ? sum / weight : grid[z][x];
        }
    }

    std::vector<std::vector<double>> out(height, std::vector<double>(width, 0.0));
    for (std::size_t z = 0; z < height; ++z) {
        for (std::size_t x = 0; x < width; ++x) {
            double sum = 0.0;
            double weight = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                int sz = static_cast<int>(z) + k;
                if (sz < 0 || sz >= static_cast<int>(height))
                    continue;
                double v = tmp[static_cast<std::size_t>(sz)][x];
                if (!std::isfinite(v))
                    continue;
                double w = kernel[static_cast<std::size_t>(k + radius)];
                sum += v * w;
                weight += w;
            }
            out[z][x] = weight > 0.0 ? sum / weight : tmp[z][x];
        }
    }
    return out;
}

void fill_nan_values(std::vector<std::vector<double>> &heights)
{
    if (heights.empty() || heights.front().empty())
        return;

    double fallback = 0.0;
    bool have_fallback = false;
    for (const auto &row : heights) {
        for (double v : row) {
            if (std::isfinite(v)) {
                fallback = v;
                have_fallback = true;
                break;
            }
        }
        if (have_fallback)
            break;
    }

    for (std::size_t z = 0; z < heights.size(); ++z) {
        for (std::size_t x = 0; x < heights[z].size(); ++x) {
            if (std::isfinite(heights[z][x]))
                continue;
            double sum = 0.0;
            int count = 0;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = static_cast<int>(x) + dx;
                    int nz = static_cast<int>(z) + dz;
                    if (nx < 0 || nz < 0 || nz >= static_cast<int>(heights.size()) ||
                            nx >= static_cast<int>(heights[static_cast<std::size_t>(nz)].size()))
                        continue;
                    double v = heights[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
                    if (std::isfinite(v)) {
                        sum += v;
                        ++count;
                    }
                }
            }
            heights[z][x] = count > 0 ? sum / count : fallback;
        }
    }
}

void filter_elevation_outliers(std::vector<std::vector<double>> &heights)
{
    if (heights.empty() || heights.front().empty())
        return;

    auto original = heights;
    for (std::size_t z = 0; z < heights.size(); ++z) {
        for (std::size_t x = 0; x < heights[z].size(); ++x) {
            std::vector<double> neighbours;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = static_cast<int>(x) + dx;
                    int nz = static_cast<int>(z) + dz;
                    if (nx < 0 || nz < 0 || nz >= static_cast<int>(original.size()) ||
                            nx >= static_cast<int>(original[static_cast<std::size_t>(nz)].size()) ||
                            (dx == 0 && dz == 0))
                        continue;
                    double v = original[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
                    if (std::isfinite(v))
                        neighbours.push_back(v);
                }
            }
            if (neighbours.size() < 3)
                continue;
            std::sort(neighbours.begin(), neighbours.end());
            double median = neighbours[neighbours.size() / 2];
            if (std::abs(original[z][x] - median) > 80.0)
                heights[z][x] = median;
        }
    }
}

}
