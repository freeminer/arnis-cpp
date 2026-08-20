#include "shoreline.h"

#include "land_cover.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace arnis::land_cover
{
namespace
{
std::uint8_t nearest_land(const std::vector<std::vector<std::uint8_t>> &grid,
		std::size_t width, std::size_t height, int x, int z, int radius)
{
	for (int r = 1; r <= radius; ++r)
		for (int dz = -r; dz <= r; ++dz)
			for (int dx = -r; dx <= r; ++dx) {
				if (std::abs(dx) != r && std::abs(dz) != r)
					continue;
				const int nx = x + dx, nz = z + dz;
				if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height)) {
					const auto value = grid[nz][nx];
					if (value && value != LC_WATER)
						return value;
				}
			}
	return LC_GRASSLAND;
}
}

bool reconstruct_water_shoreline(std::vector<std::vector<std::uint8_t>> &grid,
		std::size_t width, std::size_t height, double cells_per_meter)
{
	width = std::min(width, grid.empty() ? std::size_t{} : grid.front().size());
	height = std::min(height, grid.size());
	const double cells_per_pixel = 10.0 * cells_per_meter;
	if (width < 3 || height < 3 || !std::isfinite(cells_per_pixel) ||
			cells_per_pixel < 2.0)
		return false;
	const auto source = grid;
	std::vector<std::pair<int, int>> to_water, to_land;
	std::size_t before = 0;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x)
			before += source[z][x] == LC_WATER;
	if (before == 0)
		return false;

	// Remove one-cell orthogonal staircase corners while retaining endpoints and
	// broad bays. Two passes correspond to the sub-pixel midpoint/corner fit in
	// Rust, but operate on the already sampled mask used by the C++ COG reader.
	auto mask = source;
	for (int pass = 0; pass < 2; ++pass) {
		const auto snapshot = mask;
		for (std::size_t z = 1; z + 1 < height; ++z)
			for (std::size_t x = 1; x + 1 < width; ++x) {
				int water4 = 0, water8 = 0;
				for (int dz = -1; dz <= 1; ++dz)
					for (int dx = -1; dx <= 1; ++dx) {
						if (!dx && !dz)
							continue;
						const bool water = snapshot[z + dz][x + dx] == LC_WATER;
						water8 += water;
						water4 += water && (dx == 0 || dz == 0);
					}
				const bool water = snapshot[z][x] == LC_WATER;
				if (water && water4 <= 1 && water8 <= 3)
					mask[z][x] = 0;
				else if (!water && water4 >= 3 && water8 >= 5)
					mask[z][x] = LC_WATER;
			}
	}

	std::size_t after = 0;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x)
			after += mask[z][x] == LC_WATER;
	const double allowed = 0.10 * before + 4.0 * cells_per_pixel * cells_per_pixel;
	if (std::abs(double(after) - double(before)) > allowed)
		return false;

	const int radius = std::clamp(int(std::ceil(1.5 * cells_per_pixel)), 2, 64);
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const bool was_water = source[z][x] == LC_WATER;
			const bool is_water = mask[z][x] == LC_WATER;
			if (is_water && !was_water)
				to_water.emplace_back(x, z);
			else if (!is_water && was_water)
				to_land.emplace_back(x, z);
		}
	for (const auto [x, z] : to_water)
		grid[z][x] = LC_WATER;
	for (const auto [x, z] : to_land)
		grid[z][x] = nearest_land(source, width, height, x, z, radius);
	return !to_water.empty() || !to_land.empty();
}
}
