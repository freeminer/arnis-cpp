#pragma once

#include <cstddef>
#include <tuple>
#include <vector>

namespace arnis::geographic
{
class LLBBox;
}

namespace arnis::elevation
{

inline constexpr int MAX_Y = 319;
inline constexpr std::size_t MAX_ELEVATION_GRID_DIM = 16384;
inline constexpr std::size_t MAX_ELEVATION_GRID_CELLS =
		MAX_ELEVATION_GRID_DIM * MAX_ELEVATION_GRID_DIM;

struct ElevationData
{
	std::vector<std::vector<double>> heights;
	std::size_t width{0};
	std::size_t height{0};
	std::size_t world_width{0};
	std::size_t world_height{0};
	double min_height_m{0.0};
	double blocks_per_meter{0.0};
	int ground_level{0};
};

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t> compute_grid_dims(
		const geographic::LLBBox &bbox, double scale);

std::vector<std::vector<double>> gaussian_blur_grid(
		const std::vector<std::vector<double>> &grid, double sigma);

void fill_nan_values(std::vector<std::vector<double>> &heights);
void filter_elevation_outliers(std::vector<std::vector<double>> &heights);

}
