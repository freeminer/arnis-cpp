#include "land_cover.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace arnis::land_cover
{

uint64_t coord_hash(int32_t x, int32_t z)
{
    uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32;
    h ^= static_cast<uint32_t>(z);
    h += 0x9e3779b97f4a7c15ULL;
    h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
    h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
    return h ^ (h >> 31);
}

static std::vector<double> gaussian_kernel(double sigma)
{
    const int radius = std::max(1, static_cast<int>(std::ceil(sigma * 3.0)));
    std::vector<double> kernel(static_cast<std::size_t>(radius * 2 + 1));
    double sum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        const double v = std::exp(-(static_cast<double>(i * i)) / (2.0 * sigma * sigma));
        kernel[static_cast<std::size_t>(i + radius)] = v;
        sum += v;
    }
    for (double &v : kernel)
        v /= sum;
    return kernel;
}

std::vector<std::vector<float>> compute_water_blend_smooth(
        const std::vector<std::vector<uint8_t>> &grid,
        std::size_t width,
        std::size_t height)
{
    if (width == 0 || height == 0)
        return {};

    const double sigma = 3.0;
    const auto kernel = gaussian_kernel(sigma);
    const int radius = static_cast<int>(kernel.size() / 2);

    std::vector<std::vector<double>> tmp(height, std::vector<double>(width, 0.0));
    for (std::size_t z = 0; z < height; ++z) {
        for (std::size_t x = 0; x < width; ++x) {
            double sum = 0.0;
            double weight = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                const int sx = static_cast<int>(x) + k;
                if (sx < 0 || sx >= static_cast<int>(width))
                    continue;
                const double w = kernel[static_cast<std::size_t>(k + radius)];
                sum += (grid[z][static_cast<std::size_t>(sx)] == LC_WATER ? 1.0 : 0.0) * w;
                weight += w;
            }
            tmp[z][x] = weight > 0.0 ? sum / weight : 0.0;
        }
    }

    std::vector<std::vector<float>> out(height, std::vector<float>(width, 0.0f));
    for (std::size_t z = 0; z < height; ++z) {
        for (std::size_t x = 0; x < width; ++x) {
            double sum = 0.0;
            double weight = 0.0;
            for (int k = -radius; k <= radius; ++k) {
                const int sz = static_cast<int>(z) + k;
                if (sz < 0 || sz >= static_cast<int>(height))
                    continue;
                const double w = kernel[static_cast<std::size_t>(k + radius)];
                sum += tmp[static_cast<std::size_t>(sz)][x] * w;
                weight += w;
            }
            out[z][x] = static_cast<float>(weight > 0.0 ? sum / weight : 0.0);
        }
    }
    return out;
}

std::vector<std::vector<uint8_t>> compute_water_distance(
        const std::vector<std::vector<uint8_t>> &grid,
        std::size_t width,
        std::size_t height)
{
    std::vector<std::vector<uint8_t>> dist(height, std::vector<uint8_t>(width, 0));
    if (width == 0 || height == 0)
        return dist;

    std::deque<std::pair<int, int>> q;
    const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (std::size_t z = 0; z < height; ++z) {
        for (std::size_t x = 0; x < width; ++x) {
            if (grid[z][x] != LC_WATER)
                continue;
            bool shore = false;
            for (const auto &d : dirs) {
                const int nx = static_cast<int>(x) + d[0];
                const int nz = static_cast<int>(z) + d[1];
                if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
                        nz >= static_cast<int>(height) ||
                        grid[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)] != LC_WATER) {
                    shore = true;
                    break;
                }
            }
            if (shore) {
                dist[z][x] = 1;
                q.emplace_back(static_cast<int>(x), static_cast<int>(z));
            }
        }
    }

    while (!q.empty()) {
        auto [x, z] = q.front();
        q.pop_front();
        const uint8_t next = dist[static_cast<std::size_t>(z)][static_cast<std::size_t>(x)] ==
                        std::numeric_limits<uint8_t>::max()
                ? std::numeric_limits<uint8_t>::max()
                : static_cast<uint8_t>(dist[static_cast<std::size_t>(z)]
                                               [static_cast<std::size_t>(x)] +
                        1);
        for (const auto &d : dirs) {
            const int nx = x + d[0];
            const int nz = z + d[1];
            if (nx < 0 || nz < 0 || nx >= static_cast<int>(width) ||
                    nz >= static_cast<int>(height))
                continue;
            auto &cell = dist[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)];
            if (grid[static_cast<std::size_t>(nz)][static_cast<std::size_t>(nx)] != LC_WATER ||
                    cell != 0)
                continue;
            cell = next;
            q.emplace_back(nx, nz);
        }
    }
    return dist;
}

void LandCoverData::refresh_water_blend_grid()
{
    water_blend_grid = compute_water_blend_smooth(grid, width, height);
}

}
