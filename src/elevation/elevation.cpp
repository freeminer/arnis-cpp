#include "elevation.h"

#include "../coordinate_system/transformation.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <string_view>
#include <thread>
#include <vector>

namespace arnis::elevation
{
namespace
{
std::size_t elevation_worker_count()
{
	if (const char *value = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view text(value);
		std::size_t requested = 0;
		const auto [end, error] =
				std::from_chars(text.data(), text.data() + text.size(), requested);
		if (error == std::errc{} && end == text.data() + text.size() && requested > 0)
			return requested;
	}
	const auto hardware = std::max(1u, std::thread::hardware_concurrency());
	return std::max<std::size_t>(1, static_cast<std::size_t>(hardware * 0.9));
}

template <typename Function>
void parallel_rows(std::size_t begin, std::size_t end, Function &&function)
{
	if (begin >= end)
		return;
	const auto workers_count =
			std::min<std::size_t>(end - begin, elevation_worker_count());
	if (workers_count == 1) {
		for (auto row = begin; row < end; ++row)
			function(row);
		return;
	}
	std::atomic_size_t next{begin};
	auto worker = [&] {
		for (;;) {
			const auto row = next.fetch_add(1, std::memory_order_relaxed);
			if (row >= end)
				return;
			function(row);
		}
	};
	std::vector<std::thread> threads;
	threads.reserve(workers_count - 1);
	for (std::size_t i = 1; i < workers_count; ++i) {
		try {
			threads.emplace_back(worker);
		} catch (...) {
			break;
		}
	}
	worker();
	for (auto &thread : threads)
		thread.join();
}
} // namespace

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t> compute_grid_dims(
		const geographic::LLBBox &bbox, double scale)
{
	const auto [base_z, base_x] = coordinate_system::geo_distance(bbox.min(), bbox.max());
	const auto world_width =
			static_cast<std::size_t>(std::max(0.0, std::floor(base_x) * scale)) + 1;
	const auto world_height =
			static_cast<std::size_t>(std::max(0.0, std::floor(base_z) * scale)) + 1;
	return compute_grid_dims_for_world(world_width, world_height);
}

std::tuple<std::size_t, std::size_t, std::size_t, std::size_t>
compute_grid_dims_for_world(std::size_t world_width, std::size_t world_height)
{
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

static std::vector<double> blur_line(
		const std::vector<double> &values, const std::vector<double> &kernel, int radius)
{
	const auto len = values.size();
	std::vector<std::size_t> nonfinite(len + 1, 0);
	for (std::size_t i = 0; i < len; ++i)
		nonfinite[i + 1] = nonfinite[i] + !std::isfinite(values[i]);
	const double full_weight = std::accumulate(kernel.begin(), kernel.end(), 0.0);
	std::vector<double> out(len);
	for (std::size_t i = 0; i < len; ++i) {
		const auto left = i >= static_cast<std::size_t>(radius)
								  ? i - static_cast<std::size_t>(radius)
								  : len;
		const auto right = i + static_cast<std::size_t>(radius);
		if (left < len && right < len && nonfinite[right + 1] == nonfinite[left]) {
			double sum = 0.0;
			for (std::size_t k = 0; k < kernel.size(); ++k)
				sum += values[left + k] * kernel[k];
			out[i] = sum / full_weight;
			continue;
		}
		double sum = 0.0, weight = 0.0;
		for (int k = -radius; k <= radius; ++k) {
			const auto index = static_cast<std::int64_t>(i) + k;
			if (index < 0 || index >= static_cast<std::int64_t>(len))
				continue;
			const double value = values[static_cast<std::size_t>(index)];
			if (!std::isfinite(value))
				continue;
			const double w = kernel[static_cast<std::size_t>(k + radius)];
			sum += value * w;
			weight += w;
		}
		out[i] = weight > 0.0 ? sum / weight : std::numeric_limits<double>::quiet_NaN();
	}
	return out;
}

std::vector<std::vector<double>> gaussian_blur_grid(
		const std::vector<std::vector<double>> &grid, double sigma)
{
	if (grid.empty())
		return {};
	const std::size_t height = grid.size();
	const std::size_t width = grid.front().size();
	if (width == 0)
		return std::vector<std::vector<double>>(height);
	const auto kernel = gaussian_kernel(sigma);
	const int radius = static_cast<int>(kernel.size() / 2);

	std::vector<std::vector<double>> tmp(height, std::vector<double>(width, 0.0));
	parallel_rows(0, height,
			[&](std::size_t z) { tmp[z] = blur_line(grid[z], kernel, radius); });

	constexpr std::size_t COLUMN_GROUP = 8;
	const std::size_t group_count = (width + COLUMN_GROUP - 1) / COLUMN_GROUP;
	std::vector<std::vector<double>> output(height, std::vector<double>(width));
	parallel_rows(0, group_count, [&](std::size_t group) {
		const auto x0 = group * COLUMN_GROUP;
		const auto group_width = std::min(COLUMN_GROUP, width - x0);
		std::vector<std::vector<double>> columns(
				group_width, std::vector<double>(height));
		for (std::size_t z = 0; z < height; ++z)
			for (std::size_t c = 0; c < group_width; ++c)
				columns[c][z] = tmp[z][x0 + c];
		for (std::size_t c = 0; c < group_width; ++c) {
			const auto blurred = blur_line(columns[c], kernel, radius);
			for (std::size_t z = 0; z < height; ++z)
				output[z][x0 + c] = blurred[z];
		}
	});
	return output;
}

std::vector<std::vector<double>> gaussian_blur_grid_masked(
		const std::vector<std::vector<double>> &grid,
		const std::vector<std::vector<std::uint8_t>> &masked, double sigma)
{
	const std::size_t height = std::min(grid.size(), masked.size());
	if (height == 0)
		return {};
	const std::size_t width = std::min(grid.front().size(), masked.front().size());
	if (width == 0)
		return std::vector<std::vector<double>>(height);
	const auto kernel = gaussian_kernel(sigma);
	const int radius = static_cast<int>(kernel.size() / 2);
	std::vector<std::vector<double>> after_h(height, std::vector<double>(width));
	parallel_rows(0, height, [&](std::size_t y) {
		std::vector<double> row(grid[y].begin(), grid[y].begin() + width);
		for (std::size_t x = 0; x < width; ++x)
			if (masked[y][x])
				row[x] = std::numeric_limits<double>::quiet_NaN();
		after_h[y] = blur_line(row, kernel, radius);
	});
	constexpr std::size_t COLUMN_GROUP = 8;
	const std::size_t group_count = (width + COLUMN_GROUP - 1) / COLUMN_GROUP;
	std::vector<std::vector<double>> output(height, std::vector<double>(width));
	parallel_rows(0, group_count, [&](std::size_t group) {
		const auto x0 = group * COLUMN_GROUP;
		const auto group_width = std::min(COLUMN_GROUP, width - x0);
		std::vector<std::vector<double>> columns(
				group_width, std::vector<double>(height));
		for (std::size_t y = 0; y < height; ++y)
			for (std::size_t c = 0; c < group_width; ++c)
				columns[c][y] = after_h[y][x0 + c];
		for (std::size_t c = 0; c < group_width; ++c) {
			const auto blurred = blur_line(columns[c], kernel, radius);
			for (std::size_t y = 0; y < height; ++y)
				output[y][x0 + c] = blurred[y];
		}
	});
	return output;
}

void fill_nan_values(std::vector<std::vector<double>> &heights)
{
	if (heights.empty() || heights.front().empty())
		return;
	const std::size_t height = heights.size();
	const std::size_t width = heights.front().size();
	const bool has_nan = std::any_of(heights.begin(), heights.end(), [](const auto &row) {
		return std::any_of(
				row.begin(), row.end(), [](double value) { return std::isnan(value); });
	});
	if (!has_nan)
		return;

	// Match Rust: every pass reads an immutable snapshot, avoiding scan-order
	// bias when a large no-data area is filled from its perimeter.
	std::vector<std::vector<double>> snapshot = heights;
	for (;;) {
		parallel_rows(0, height, [&](std::size_t z) {
			std::copy(heights[z].begin(), heights[z].end(), snapshot[z].begin());
		});
		std::atomic_bool changed{false};
		parallel_rows(0, height, [&](std::size_t z) {
			bool row_changed = false;
			const auto row_width = std::min(width, heights[z].size());
			for (std::size_t x = 0; x < row_width; ++x) {
				if (!std::isnan(heights[z][x]))
					continue;
				double sum = 0.0;
				int count = 0;
				for (int dz = -1; dz <= 1; ++dz)
					for (int dx = -1; dx <= 1; ++dx) {
						const int nx = static_cast<int>(x) + dx;
						const int nz = static_cast<int>(z) + dz;
						if (nx < 0 || nz < 0 || nz >= static_cast<int>(snapshot.size()) ||
								nx >= static_cast<int>(width) ||
								nx >= static_cast<int>(
											  snapshot[static_cast<std::size_t>(nz)]
													  .size()))
							continue;
						const double value = snapshot[static_cast<std::size_t>(nz)]
													 [static_cast<std::size_t>(nx)];
						if (!std::isnan(value)) {
							sum += value;
							++count;
						}
					}
				if (count > 0) {
					heights[z][x] = sum / count;
					row_changed = true;
				}
			}
			if (row_changed)
				changed.store(true, std::memory_order_relaxed);
		});
		if (!changed.load(std::memory_order_relaxed))
			break;
	}
}

void filter_elevation_outliers(std::vector<std::vector<double>> &heights)
{
	if (heights.empty() || heights.front().empty())
		return;
	// Rust deliberately uses a fixed physical validity gate.  A statistical
	// IQR filter incorrectly removes genuine isolated peaks and islands.
	constexpr double MIN_REASONABLE_M = -500.0;
	constexpr double MAX_REASONABLE_M = 9000.0;
	const std::size_t height = heights.size();
	const std::size_t width = heights.front().size();
	std::atomic_size_t filtered{0};
	parallel_rows(0, height, [&](std::size_t y) {
		std::size_t row_filtered = 0;
		for (std::size_t x = 0; x < std::min(width, heights[y].size()); ++x) {
			double &value = heights[y][x];
			if (!std::isnan(value) &&
					(value < MIN_REASONABLE_M || value > MAX_REASONABLE_M)) {
				value = std::numeric_limits<double>::quiet_NaN();
				++row_filtered;
			}
		}
		if (row_filtered)
			filtered.fetch_add(row_filtered, std::memory_order_relaxed);
	});
	const auto outliers_filtered = filtered.load(std::memory_order_relaxed);
	if (outliers_filtered > 0) {
		std::clog << "Filtered " << outliers_filtered
				  << " impossible elevations (outside " << MIN_REASONABLE_M << "m.."
				  << MAX_REASONABLE_M << "m)\n";
		fill_nan_values(heights);
	}
}

}
