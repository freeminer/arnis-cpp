#include "elevation.h"

#include "../coordinate_system/transformation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace arnis::elevation
{

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t> compute_grid_dims(
		const geographic::LLBBox &bbox, double scale)
{
	const auto [base_z, base_x] = coordinate_system::geo_distance(bbox.min(), bbox.max());
	const auto world_width =
			static_cast<std::size_t>(std::max(0.0, std::floor(base_x) * scale)) + 1;
	const auto world_height =
			static_cast<std::size_t>(std::max(0.0, std::floor(base_z) * scale)) + 1;
	std::size_t grid_width = std::max<std::size_t>(2, world_width);
	std::size_t grid_height = std::max<std::size_t>(2, world_height);
	const double cells = static_cast<double>(grid_width) * grid_height;
	const double budget_shrink = std::sqrt(cells / MAX_ELEVATION_GRID_CELLS);
	const double axis_shrink = static_cast<double>(std::max(grid_width, grid_height)) /
							   MAX_ELEVATION_GRID_DIM;
	const double shrink = std::max(budget_shrink, axis_shrink);
	if (shrink > 1.0) {
		grid_width = std::clamp(static_cast<std::size_t>(std::floor(grid_width / shrink)),
				std::size_t{2}, std::max<std::size_t>(2, world_width));
		grid_height =
				std::clamp(static_cast<std::size_t>(std::floor(grid_height / shrink)),
						std::size_t{2}, std::max<std::size_t>(2, world_height));
	}
	return {world_width, world_height, grid_width, grid_height};
}

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

	// Match Rust: every pass reads an immutable snapshot, avoiding scan-order
	// bias when a large no-data area is filled from its perimeter.
	bool changed = true;
	while (changed) {
		const auto snapshot = heights;
		changed = false;
		for (std::size_t z = 0; z < heights.size(); ++z) {
			for (std::size_t x = 0; x < heights[z].size(); ++x) {
				if (!std::isnan(heights[z][x]))
					continue;
				double sum = 0.0;
				int count = 0;
				for (int dz = -1; dz <= 1; ++dz)
					for (int dx = -1; dx <= 1; ++dx) {
						const int nx = static_cast<int>(x) + dx;
						const int nz = static_cast<int>(z) + dz;
						if (nx < 0 || nz < 0 || nz >= static_cast<int>(snapshot.size()) ||
							nx >= static_cast<int>(snapshot[static_cast<std::size_t>(nz)].size()))
							continue;
						const double value = snapshot[static_cast<std::size_t>(nz)]
									[static_cast<std::size_t>(nx)];
						if (!std::isnan(value)) { sum += value; ++count; }
					}
				if (count > 0) {
					heights[z][x] = sum / count;
					changed = true;
				}
			}
		}
	}
}

void filter_elevation_outliers(std::vector<std::vector<double>> &heights)
{
	if (heights.empty() || heights.front().empty())
		return;

	std::vector<double> values;
	for (const auto &row : heights)
		for (double value : row)
			if (std::isfinite(value))
				values.push_back(value);
	if (values.size() < 4)
		return;

	auto nth = [&values](std::size_t index) {
		std::nth_element(values.begin(), values.begin() + index, values.end());
		return values[index];
	};
	const double q1 = nth(values.size() / 4);
	const double q3 = nth((values.size() * 3) / 4);
	const double iqr = q3 - q1;
	const double lower = q1 - 3.0 * iqr;
	const double upper = q3 + 3.0 * iqr;

	std::size_t below = 0, above = 0;
	for (double value : values) {
		below += value < lower;
		above += value > upper;
	}
	const std::size_t threshold = static_cast<std::size_t>(values.size() * 0.05);
	const bool filter_lower = below > 0 && below <= threshold;
	const bool filter_upper = above > 0 && above <= threshold;
	if (!filter_lower && !filter_upper)
		return;

	std::size_t filtered = 0;
	for (auto &row : heights)
		for (double &value : row)
			if (std::isfinite(value) &&
					((filter_lower && value < lower) || (filter_upper && value > upper))) {
				value = std::numeric_limits<double>::quiet_NaN();
				++filtered;
			}
	if (filtered > 0)
		fill_nan_values(heights);
}

}
