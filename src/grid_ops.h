#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace arnis::grid_ops
{

double mercator_source_row(
		double lat_top, double lat_bottom, std::size_t rows, std::size_t gz);

template <typename T, typename Lerp>
void remap_rows_in_place(std::vector<std::vector<T>> &grid,
		const std::function<double(std::size_t)> &source, Lerp lerp)
{
	const std::size_t rows = grid.size();
	if (rows < 2)
		return;
	bool above = true, below = true;
	for (std::size_t z = 0; z < rows; ++z) {
		const double s = source(z);
		if (s > double(z) + 1e-9)
			above = false;
		if (s < double(z) - 1e-9)
			below = false;
	}
	auto build = [&](std::size_t z) {
		const double s = std::clamp(source(z), 0.0, double(rows - 1));
		const auto a = std::size_t(std::floor(s));
		const auto b = std::min(a + 1, rows - 1);
		const double t = s - double(a);
		if (t <= 1e-9 || a == b)
			return grid[a];
		std::vector<T> out;
		out.reserve(grid[a].size());
		for (std::size_t x = 0; x < grid[a].size() && x < grid[b].size(); ++x)
			out.push_back(lerp(grid[a][x], grid[b][x], t));
		return out;
	};
	if (above)
		for (std::size_t z = rows; z-- > 0;)
			grid[z] = build(z);
	else if (below)
		for (std::size_t z = 0; z < rows; ++z)
			grid[z] = build(z);
	else {
		std::vector<std::vector<T>> fresh;
		fresh.reserve(rows);
		for (std::size_t z = 0; z < rows; ++z)
			fresh.push_back(build(z));
		grid.swap(fresh);
	}
}

template <typename T>
void crop_rows(std::vector<std::vector<T>> &grid, std::size_t x0, std::size_t z0,
		std::size_t width, std::size_t height)
{
	if (z0)
		grid.erase(grid.begin(), grid.begin() + std::min(z0, grid.size()));
	if (grid.size() > height)
		grid.resize(height);
	for (auto &row : grid) {
		if (x0)
			row.erase(row.begin(), row.begin() + std::min(x0, row.size()));
		if (row.size() > width)
			row.resize(width);
		row.shrink_to_fit();
	}
	grid.shrink_to_fit();
}

template <typename T>
void crop_flat(std::vector<T> &grid, std::size_t source_width, std::size_t x0,
		std::size_t z0, std::size_t width, std::size_t height)
{
	if (!source_width)
		return;
	const std::size_t rows = grid.size() / source_width;
	if (z0 >= rows) {
		grid.clear();
		return;
	}
	width = std::min(width, source_width > x0 ? source_width - x0 : 0);
	height = std::min(height, rows - z0);
	std::vector<T> out;
	out.reserve(width * height);
	for (std::size_t z = 0; z < height; ++z)
		out.insert(out.end(), grid.begin() + (z0 + z) * source_width + x0,
				grid.begin() + (z0 + z) * source_width + x0 + width);
	grid.swap(out);
}

}
