#include "postprocess.h"
#include "elevation.h"
#include "../land_cover/land_cover.h"
#include "../world_editor/floor_state.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_set>
namespace arnis::elevation
{
void repair_terrain_anomalies(std::vector<std::vector<double>> &h, double meters_per_cell)
{
	if (h.size() < 5 || h[0].size() < 5)
		return;
	const std::size_t H = h.size(), W = h[0].size();
	const double abs_threshold = std::max(6.0, 0.25 * meters_per_cell);
	const int passes = meters_per_cell > 4.0 ? 2 : 10;
	std::vector<std::vector<double>> snapshot = h;
	std::vector<double> neighbors;
	std::vector<double> abs_devs;
	neighbors.reserve(24);
	abs_devs.reserve(24);
	std::size_t total_repaired = 0;
	int passes_ran = 0;
	for (int pass = 0; pass < passes; ++pass) {
		if (pass > 0)
			snapshot = h;
		std::size_t changed = 0;
		for (std::size_t y = 2; y < H - 2; ++y)
			for (std::size_t x = 2; x < W - 2; ++x) {
				const double c = snapshot[y][x];
				if (!std::isfinite(c))
					continue;
				neighbors.clear();
				double lo = std::numeric_limits<double>::infinity();
				double hi = -std::numeric_limits<double>::infinity();
				for (int dy = -2; dy <= 2; ++dy) {
					for (int dx = -2; dx <= 2; ++dx) {
						if (dx == 0 && dy == 0)
							continue;
						const double value = snapshot[static_cast<std::size_t>(
								static_cast<int>(y) +
								dy)][static_cast<std::size_t>(static_cast<int>(x) + dx)];
						if (!std::isfinite(value))
							continue;
						neighbors.push_back(value);
						lo = std::min(lo, value);
						hi = std::max(hi, value);
					}
				}
				if (neighbors.size() < 8)
					continue;
				// The median lies inside the neighbor range, so a center close to both
				// extrema cannot exceed the absolute threshold. Skip both nth_element
				// operations for the common smooth-terrain case.
				if (c - lo <= abs_threshold && hi - c <= abs_threshold)
					continue;
				const auto mid = neighbors.begin() + neighbors.size() / 2;
				std::nth_element(neighbors.begin(), mid, neighbors.end());
				const double median = *mid;
				abs_devs.clear();
				for (const double value : neighbors)
					abs_devs.push_back(std::abs(value - median));
				const auto mad_mid = abs_devs.begin() + abs_devs.size() / 2;
				std::nth_element(abs_devs.begin(), mad_mid, abs_devs.end());
				const double mad = *mad_mid;
				if (std::abs(c - median) > abs_threshold &&
						std::abs(c - median) > 3.0 * std::max(1.0, mad)) {
					h[y][x] = median;
					++changed;
				}
			}
		if (changed == 0)
			break;
		total_repaired += changed;
		passes_ran = pass + 1;
	}
	if (total_repaired > 0)
		std::clog << "Repaired " << total_repaired << " terrain anomalies in "
				  << passes_ran << " pass" << (passes_ran == 1 ? "" : "es") << '\n';
}

namespace
{
using HeightGrid = std::vector<std::vector<double>>;
using CoverGrid = std::vector<std::vector<std::uint8_t>>;
using MaskGrid = std::vector<std::vector<std::uint8_t>>;
using Cell = std::pair<std::size_t, std::size_t>;

struct FlowingWaterCell
{
	std::uint32_t x, y;
	float local_surface;
};

constexpr std::array<std::pair<int, int>, 4> CARDINAL{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr double MAX_STEEP_WATER_AREA_M2 = 250000.0;
constexpr double MIN_STEEP_WATER_SLOPE = 0.35;
constexpr double STEEP_WATER_LAND_BELOW_M = 2.0;
constexpr double MIN_PERCHED_FRACTION = 0.10;

double median(std::vector<double> values)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	const auto mid = values.begin() + values.size() / 2;
	std::nth_element(values.begin(), mid, values.end());
	return *mid;
}

double interquartile_range(std::vector<double> values)
{
	if (values.size() < 4)
		return 0.0;
	const std::size_t q1i = values.size() / 4, q3i = values.size() * 3 / 4;
	std::nth_element(values.begin(), values.begin() + q1i, values.end());
	const double q1 = values[q1i];
	std::nth_element(values.begin(), values.begin() + q3i, values.end());
	return std::max(0.0, values[q3i] - q1);
}

bool inside(int x, int y, std::size_t w, std::size_t h)
{
	return x >= 0 && y >= 0 && static_cast<std::size_t>(x) < w &&
		   static_cast<std::size_t>(y) < h;
}

std::size_t coarse_smoothing_step(
		std::size_t bbox_width, std::size_t bbox_height, double sigma_cells)
{
	constexpr double COARSE_PER_SIGMA = 8.0;
	constexpr double MAX_COARSE_CELLS = 4.0 * 1024.0 * 1024.0;
	const double from_sigma = std::max(1.0, std::floor(sigma_cells / COARSE_PER_SIGMA));
	const double from_budget =
			std::max(1.0, std::ceil(std::sqrt(static_cast<double>(bbox_width) *
											  bbox_height / MAX_COARSE_CELLS)));
	return static_cast<std::size_t>(std::max(from_sigma, from_budget));
}

std::vector<double> smooth_sparse_water_field(
		const std::vector<FlowingWaterCell> &cells, double sigma_cells)
{
	if (cells.empty())
		return {};
	std::uint32_t min_x = std::numeric_limits<std::uint32_t>::max();
	std::uint32_t min_y = std::numeric_limits<std::uint32_t>::max();
	std::uint32_t max_x = 0, max_y = 0;
	for (const auto &cell : cells) {
		min_x = std::min(min_x, cell.x);
		min_y = std::min(min_y, cell.y);
		max_x = std::max(max_x, cell.x);
		max_y = std::max(max_y, cell.y);
	}
	const std::size_t bbox_width = static_cast<std::size_t>(max_x - min_x) + 1;
	const std::size_t bbox_height = static_cast<std::size_t>(max_y - min_y) + 1;
	const std::size_t step = coarse_smoothing_step(bbox_width, bbox_height, sigma_cells);
	const std::size_t coarse_width = (bbox_width - 1) / step + 1;
	const std::size_t coarse_height = (bbox_height - 1) / step + 1;
	std::vector<std::vector<double>> sums(
			coarse_height, std::vector<double>(coarse_width, 0.0));
	std::vector<std::vector<std::uint32_t>> counts(
			coarse_height, std::vector<std::uint32_t>(coarse_width, 0));
	for (const auto &cell : cells) {
		const std::size_t x = (cell.x - min_x) / step;
		const std::size_t y = (cell.y - min_y) / step;
		sums[y][x] += cell.local_surface;
		++counts[y][x];
	}
	HeightGrid coarse(coarse_height, std::vector<double>(coarse_width));
	for (std::size_t y = 0; y < coarse_height; ++y)
		for (std::size_t x = 0; x < coarse_width; ++x)
			coarse[y][x] = counts[y][x] ? sums[y][x] / counts[y][x]
										: std::numeric_limits<double>::quiet_NaN();
	std::vector<std::vector<double>>().swap(sums);
	std::vector<std::vector<std::uint32_t>>().swap(counts);
	const auto blurred = gaussian_blur_grid(coarse, sigma_cells / step);
	std::vector<double> out;
	out.reserve(cells.size());
	for (const auto &cell : cells) {
		const double fx = static_cast<double>(cell.x - min_x) / step - 0.5;
		const double fy = static_cast<double>(cell.y - min_y) / step - 0.5;
		const double ix = std::floor(fx), iy = std::floor(fy);
		const double tx = fx - ix, ty = fy - iy;
		double weighted = 0.0, weight_sum = 0.0;
		for (int dy = 0; dy <= 1; ++dy)
			for (int dx = 0; dx <= 1; ++dx) {
				const auto x = static_cast<std::int64_t>(ix) + dx;
				const auto y = static_cast<std::int64_t>(iy) + dy;
				if (x < 0 || y < 0 || static_cast<std::size_t>(x) >= coarse_width ||
						static_cast<std::size_t>(y) >= coarse_height)
					continue;
				const double weight = (dx ? tx : 1.0 - tx) * (dy ? ty : 1.0 - ty);
				const double value =
						blurred[static_cast<std::size_t>(y)][static_cast<std::size_t>(x)];
				if (std::isfinite(value)) {
					weighted += value * weight;
					weight_sum += weight;
				}
			}
		out.push_back(weight_sum > 0.0 ? weighted / weight_sum : cell.local_surface);
	}
	return out;
}

bool has_non_water_neighbor(const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = cover.size(), w = cover.front().size();
	for (const auto &[dx, dy] : CARDINAL) {
		const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
		if (!inside(nx, ny, w, h) ||
				cover[static_cast<std::size_t>(ny)][static_cast<std::size_t>(nx)] !=
						land_cover::LC_WATER)
			return true;
	}
	return false;
}

std::optional<std::uint8_t> nearest_non_water_class(
		const CoverGrid &cover, std::size_t x, std::size_t y, int radius)
{
	const int h = static_cast<int>(cover.size());
	const int w = h ? static_cast<int>(cover.front().size()) : 0;
	for (int r = 1; r <= radius; ++r)
		for (int dy = -r; dy <= r; ++dy)
			for (int dx = -r; dx <= r; ++dx) {
				if (std::abs(dx) != r && std::abs(dy) != r)
					continue;
				const int nx = static_cast<int>(x) + dx;
				const int ny = static_cast<int>(y) + dy;
				if (!inside(nx, ny, w, h))
					continue;
				const auto value = cover[ny][nx];
				if (value != 0 && value != land_cover::LC_WATER)
					return value;
			}
	return std::nullopt;
}

double histogram_mode(const std::vector<double> &values, double bin_size)
{
	const auto &[lo, hi] = std::minmax_element(values.begin(), values.end());
	if (*hi - *lo < bin_size)
		return *lo;
	const auto count = static_cast<std::size_t>(std::ceil((*hi - *lo) / bin_size)) + 1;
	std::vector<std::size_t> bins(count);
	for (double value : values) {
		const auto index =
				std::min(count - 1, static_cast<std::size_t>((value - *lo) / bin_size));
		++bins[index];
	}
	const auto peak = static_cast<std::size_t>(
			std::max_element(bins.begin(), bins.end()) - bins.begin());
	return *lo + (static_cast<double>(peak) + 0.5) * bin_size;
}

double clamp_by_adjacent_land(double proposed, const std::vector<Cell> &component,
		const HeightGrid &heights, const CoverGrid &cover)
{
	const auto h = heights.size(), w = heights.front().size();
	std::unordered_set<std::uint64_t> seen;
	std::vector<double> adjacent;
	for (const auto &[x, y] : component)
		for (const auto &[dx, dy] : CARDINAL) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (!inside(nx, ny, w, h) || cover[ny][nx] == land_cover::LC_WATER)
				continue;
			const auto key = static_cast<std::uint64_t>(ny) * w + nx;
			if (!seen.insert(key).second || !std::isfinite(heights[ny][nx]))
				continue;
			adjacent.push_back(heights[ny][nx]);
		}
	if (adjacent.empty())
		return proposed;
	const auto q = adjacent.begin() + adjacent.size() / 4;
	std::nth_element(adjacent.begin(), q, adjacent.end());
	return std::min(proposed, *q);
}

std::optional<double> lowest_adjacent_land(
		const HeightGrid &heights, const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = heights.size(), w = heights.front().size();
	double lowest = std::numeric_limits<double>::infinity();
	for (const auto &[dx, dy] : CARDINAL) {
		const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
		if (!inside(nx, ny, w, h) || cover[ny][nx] == land_cover::LC_WATER)
			continue;
		if (std::isfinite(heights[ny][nx]))
			lowest = std::min(lowest, heights[ny][nx]);
	}
	return std::isfinite(lowest) ? std::optional<double>{lowest} : std::nullopt;
}

std::size_t drop_water_on_steep_terrain(
		const HeightGrid &heights, CoverGrid &cover, double meters_per_cell)
{
	constexpr int SEARCH_RADIUS = 8;
	if (heights.empty() || meters_per_cell <= 0.0 || !std::isfinite(meters_per_cell))
		return 0;
	const std::size_t h = heights.size(), w = heights.front().size();
	if (!w || cover.size() != h || cover.front().size() != w)
		return 0;
	// Rust's float-to-usize cast saturates before the 1..=16 clamp. Bound
	// before rounding so tiny but finite cell sizes cannot overflow llround.
	const int step = std::clamp(
			static_cast<int>(std::llround(std::clamp(10.0 / meters_per_cell, 0.0, 16.0))),
			1, 16);
	const double max_cell_count =
			MAX_STEEP_WATER_AREA_M2 / (meters_per_cell * meters_per_cell);
	const std::size_t max_cells =
			max_cell_count >= double(std::numeric_limits<std::size_t>::max())
					? std::numeric_limits<std::size_t>::max()
					: static_cast<std::size_t>(max_cell_count);
	const std::size_t cell_count = w * h;
	std::vector<std::uint64_t> visited((cell_count + 63) / 64);
	std::vector<std::uint64_t> in_component((cell_count + 63) / 64);
	const auto get_bit = [](const std::vector<std::uint64_t> &mask, std::size_t i) {
		return (mask[i >> 6] >> (i & 63)) & 1u;
	};
	const auto set_bit = [](std::vector<std::uint64_t> &mask, std::size_t i) {
		mask[i >> 6] |= std::uint64_t{1} << (i & 63);
	};
	const auto clear_bit = [](std::vector<std::uint64_t> &mask, std::size_t i) {
		mask[i >> 6] &= ~(std::uint64_t{1} << (i & 63));
	};
	const auto cell_slope = [&](std::size_t x, std::size_t y) -> std::optional<double> {
		const double here = heights[y][x];
		if (!std::isfinite(here))
			return std::nullopt;
		const auto at = [&](int dx, int dy) -> std::optional<std::pair<double, double>> {
			for (int d = step; d >= 1; --d) {
				const int nx = static_cast<int>(x) + dx * d;
				const int ny = static_cast<int>(y) + dy * d;
				if (!inside(nx, ny, w, h) || cover[ny][nx] != land_cover::LC_WATER)
					continue;
				const double value = heights[ny][nx];
				if (std::isfinite(value))
					return std::pair{value, d * meters_per_cell};
			}
			return std::nullopt;
		};
		const auto axis = [here](const auto &low,
								  const auto &high) -> std::optional<double> {
			if (low && high)
				return (high->first - low->first) / (low->second + high->second);
			if (low)
				return (here - low->first) / low->second;
			if (high)
				return (high->first - here) / high->second;
			return std::nullopt;
		};
		const auto gx = axis(at(-1, 0), at(1, 0));
		const auto gz = axis(at(0, -1), at(0, 1));
		if (!gx && !gz)
			return std::nullopt;
		return std::hypot(gx.value_or(0.0), gz.value_or(0.0));
	};
	std::vector<std::pair<std::uint32_t, std::uint32_t>> component;
	std::deque<Cell> queue;
	std::vector<std::pair<std::uint32_t, std::uint32_t>> dropped;
	std::size_t dropped_components = 0;
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			const std::size_t start = sy * w + sx;
			if (get_bit(visited, start) || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			component.clear();
			queue.clear();
			std::size_t size = 0;
			queue.emplace_back(sx, sy);
			set_bit(visited, start);
			while (!queue.empty()) {
				const auto [x, y] = queue.front();
				queue.pop_front();
				++size;
				if (max_cells == std::numeric_limits<std::size_t>::max() ||
						size <= max_cells + 1)
					component.emplace_back(
							static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y));
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx;
					const int ny = static_cast<int>(y) + dy;
					if (!inside(nx, ny, w, h) || cover[ny][nx] != land_cover::LC_WATER)
						continue;
					const std::size_t index = static_cast<std::size_t>(ny) * w + nx;
					if (!get_bit(visited, index)) {
						set_bit(visited, index);
						queue.emplace_back(static_cast<std::size_t>(nx),
								static_cast<std::size_t>(ny));
					}
				}
			}
			if (size > max_cells)
				continue;
			std::vector<double> slopes;
			slopes.reserve(component.size());
			for (const auto &[x, y] : component)
				if (const auto slope = cell_slope(x, y))
					slopes.push_back(*slope);
			if (slopes.empty() || median(std::move(slopes)) <= MIN_STEEP_WATER_SLOPE)
				continue;
			for (const auto &[x, y] : component)
				set_bit(in_component, y * w + x);
			std::size_t edge_cells = 0, perched = 0;
			for (const auto &[x, y] : component) {
				const double here = heights[y][x];
				if (!std::isfinite(here))
					continue;
				double lowest = std::numeric_limits<double>::infinity();
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx * step;
					const int ny = static_cast<int>(y) + dy * step;
					if (!inside(nx, ny, w, h))
						continue;
					const std::size_t index = static_cast<std::size_t>(ny) * w + nx;
					if (get_bit(in_component, index))
						continue;
					if (std::isfinite(heights[ny][nx]))
						lowest = std::min(lowest, heights[ny][nx]);
				}
				if (!std::isfinite(lowest))
					continue;
				++edge_cells;
				perched += lowest < here - STEEP_WATER_LAND_BELOW_M;
			}
			for (const auto &[x, y] : component)
				clear_bit(in_component, y * w + x);
			if (!edge_cells ||
					static_cast<double>(perched) < MIN_PERCHED_FRACTION * edge_cells)
				continue;
			++dropped_components;
			dropped.insert(dropped.end(), component.begin(), component.end());
		}
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	replacements.reserve(dropped.size());
	for (const auto &[x, y] : dropped)
		replacements.emplace_back(x, y,
				nearest_non_water_class(cover, x, y, SEARCH_RADIUS)
						.value_or(land_cover::LC_BARE));
	for (const auto &[x, y, value] : replacements)
		cover[y][x] = value;
	if (dropped_components)
		std::clog << "Land cover repair: dropped " << dropped_components
				  << " water blob(s) (" << replacements.size()
				  << " cells) sitting on steep terrain (ESA shadow misclassification)\n";
	return replacements.size();
}

std::optional<double> local_water_median_cover(const HeightGrid &heights,
		const CoverGrid &cover, std::size_t x, std::size_t y, int radius)
{
	const auto h = heights.size(), w = heights.front().size();
	std::vector<double> samples;
	samples.reserve(static_cast<std::size_t>((radius * 2 + 1) * (radius * 2 + 1)));
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx) {
			const int nx = static_cast<int>(x) + dx;
			const int ny = static_cast<int>(y) + dy;
			if (inside(nx, ny, w, h) && cover[ny][nx] == land_cover::LC_WATER &&
					std::isfinite(heights[ny][nx]))
				samples.push_back(heights[ny][nx]);
		}
	if (samples.size() < 8)
		return std::nullopt;
	return median(std::move(samples));
}

MaskGrid level_water_surfaces(
		HeightGrid &heights, const CoverGrid &cover, double meters_per_cell)
{
	constexpr double UP_TOLERANCE = 2.0;
	constexpr int LOCAL_SURFACE_RADIUS = 12;
	constexpr double FLOWING_IQR_THRESHOLD = 5.0;
	constexpr double FLOW_SMOOTH_SIGMA_M = 40.0;
	constexpr double FLOW_SMOOTH_MAX_M = 1.5;
	if (heights.empty() || heights.front().empty())
		return {};
	const auto h = heights.size(), w = heights.front().size();
	HeightGrid snapshot = heights;
	MaskGrid visited(h, std::vector<std::uint8_t>(w));
	MaskGrid surface(h, std::vector<std::uint8_t>(w));
	std::vector<std::vector<FlowingWaterCell>> flowing_components;
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			if (visited[sy][sx] || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			std::vector<Cell> component;
			std::deque<Cell> queue{{sx, sy}};
			std::vector<double> values;
			visited[sy][sx] = 1;
			while (!queue.empty()) {
				const auto [x, y] = queue.front();
				queue.pop_front();
				component.emplace_back(x, y);
				if (std::isfinite(snapshot[y][x]))
					values.push_back(snapshot[y][x]);
				for (const auto &[dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx;
					const int ny = static_cast<int>(y) + dy;
					if (inside(nx, ny, w, h) && !visited[ny][nx] &&
							cover[ny][nx] == land_cover::LC_WATER) {
						visited[ny][nx] = 1;
						queue.emplace_back(static_cast<std::size_t>(nx),
								static_cast<std::size_t>(ny));
					}
				}
			}
			if (values.empty())
				continue;
			const double iqr = interquartile_range(values);
			const double fallback = median(values);
			if (iqr > FLOWING_IQR_THRESHOLD) {
				std::vector<FlowingWaterCell> cells;
				cells.reserve(component.size());
				for (const auto &[x, y] : component) {
					const double original = snapshot[y][x];
					if (!std::isfinite(original))
						continue;
					const double local = local_water_median_cover(
							snapshot, cover, x, y, LOCAL_SURFACE_RADIUS)
												 .value_or(fallback);
					cells.push_back({static_cast<std::uint32_t>(x),
							static_cast<std::uint32_t>(y), static_cast<float>(local)});
				}
				flowing_components.push_back(std::move(cells));
				continue;
			}
			const double proposed =
					values.size() >= 16 ? histogram_mode(values, 1.0) : fallback;
			const double level =
					clamp_by_adjacent_land(proposed, component, snapshot, cover);
			for (const auto &[x, y] : component) {
				const double original = snapshot[y][x];
				if (!std::isfinite(original))
					continue;
				if (original <= level + UP_TOLERANCE ||
						!has_non_water_neighbor(cover, x, y)) {
					heights[y][x] = level;
					surface[y][x] = 1;
				}
			}
		}
	HeightGrid().swap(snapshot);
	MaskGrid().swap(visited);
	const double sigma = meters_per_cell > 0.0 && std::isfinite(meters_per_cell)
								 ? std::min(64.0, FLOW_SMOOTH_SIGMA_M / meters_per_cell)
								 : 0.0;
	for (const auto &component : flowing_components) {
		if (component.empty())
			continue;
		const auto smoothed = sigma >= 1.5 ? smooth_sparse_water_field(component, sigma)
										   : std::vector<double>{};
		for (std::size_t i = 0; i < component.size(); ++i) {
			const auto &cell = component[i];
			const auto x = static_cast<std::size_t>(cell.x);
			const auto y = static_cast<std::size_t>(cell.y);
			const double original = heights[y][x];
			const double local = cell.local_surface;
			const double blurred = smoothed.empty() ? local : smoothed[i];
			double level = local + std::clamp(blurred - local, -FLOW_SMOOTH_MAX_M,
										   FLOW_SMOOTH_MAX_M);
			if (const auto land = lowest_adjacent_land(heights, cover, x, y))
				level = std::min(level, std::max(original, *land));
			if (original <= level + UP_TOLERANCE ||
					!has_non_water_neighbor(cover, x, y)) {
				heights[y][x] = level;
				surface[y][x] = 1;
			}
		}
	}
	return surface;
}

std::size_t reclassify_non_surface_water_cells(CoverGrid &cover, const MaskGrid &surface)
{
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	for (std::size_t y = 0; y < cover.size(); ++y)
		for (std::size_t x = 0; x < cover[y].size(); ++x)
			if (cover[y][x] == land_cover::LC_WATER && !surface[y][x])
				replacements.emplace_back(x, y,
						nearest_non_water_class(cover, x, y, 8)
								.value_or(land_cover::LC_BARE));
	for (const auto &[x, y, value] : replacements)
		cover[y][x] = value;
	return replacements.size();
}

void pull_coastal_land_toward_water(
		HeightGrid &heights, const MaskGrid &surface, std::uint32_t max_distance)
{
	if (!max_distance)
		return;
	const auto h = heights.size(), w = heights.front().size();
	std::vector<std::vector<std::uint32_t>> distance(
			h, std::vector<std::uint32_t>(w, std::numeric_limits<std::uint32_t>::max()));
	HeightGrid water_level(
			h, std::vector<double>(w, std::numeric_limits<double>::quiet_NaN()));
	std::deque<Cell> queue;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x)
			if (surface[y][x]) {
				distance[y][x] = 0;
				water_level[y][x] = heights[y][x];
				queue.emplace_back(x, y);
			}
	while (!queue.empty()) {
		auto [x, y] = queue.front();
		queue.pop_front();
		if (distance[y][x] >= max_distance)
			continue;
		for (const auto &[dx, dy] : CARDINAL) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (inside(nx, ny, w, h) && distance[y][x] + 1 < distance[ny][nx]) {
				distance[ny][nx] = distance[y][x] + 1;
				water_level[ny][nx] = water_level[y][x];
				queue.emplace_back(nx, ny);
			}
		}
	}
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			const auto d = distance[y][x];
			const double water = water_level[y][x], original = heights[y][x];
			if (!d || d > max_distance || !std::isfinite(water) ||
					!std::isfinite(original) || original - water > 15.0)
				continue;
			const double weight = static_cast<double>(max_distance - d) / max_distance;
			heights[y][x] = original * (1.0 - weight) + water * weight;
		}
}

void smooth_built_up_gaussian(HeightGrid &heights, const CoverGrid &cover,
		const MaskGrid &surface, double sigma, const std::function<void(double)> &report)
{
	if (sigma < 1.5)
		return;
	const auto h = heights.size(), w = heights.front().size();
	HeightGrid mask(h, std::vector<double>(w));
	bool any = false;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x)
			if (cover[y][x] == land_cover::LC_BUILT_UP)
				mask[y][x] = any = true;
	if (!any)
		return;
	if (report)
		report(0.0);
	auto feathered = gaussian_blur_grid(mask, sigma);
	if (report)
		report(0.5);
	// Synthesize masked NaNs while reading each row, as Rust does; avoiding a
	// full-sized copy of the elevation grid lowers the peak during city smoothing.
	auto blurred = gaussian_blur_grid_masked(heights, surface, sigma);
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			if (surface[y][x])
				continue;
			const double weight = static_cast<double>(
					static_cast<float>(std::clamp(feathered[y][x], 0.0, 1.0)));
			if (weight > 1e-4 && std::isfinite(heights[y][x]) &&
					std::isfinite(blurred[y][x]))
				heights[y][x] = heights[y][x] * (1.0 - weight) + blurred[y][x] * weight;
		}
	if (report)
		report(1.0);
}
}

void apply_land_cover_repair(HeightGrid &heights, land_cover::LandCoverData &data,
		double built_up_sigma_cells, std::uint32_t coastal_pull_distance_cells,
		double meters_per_cell, const std::function<void(double)> &report)
{
	if (heights.empty() || heights.front().empty() ||
			data.width != heights.front().size() || data.height != heights.size() ||
			data.grid.size() != data.height)
		return;
	const auto dropped = drop_water_on_steep_terrain(heights, data.grid, meters_per_cell);
	auto surface = level_water_surfaces(heights, data.grid, meters_per_cell);
	const auto reclassified = reclassify_non_surface_water_cells(data.grid, surface);
	if (dropped + reclassified) {
		data.water_distance =
				land_cover::compute_water_distance(data.grid, data.width, data.height);
		data.water_blend_grid.clear();
	}
	smooth_built_up_gaussian(heights, data.grid, surface, built_up_sigma_cells, report);
	pull_coastal_land_toward_water(heights, surface, coastal_pull_distance_cells);
}

namespace
{
constexpr double LOWEST_LAND_M = -430.0;
constexpr double HIGHEST_LAND_M = 8849.0;
constexpr double SOFT_TOP_BLOCKS = 800.0;
constexpr double HEADROOM_MAX_BLOCKS = 96.0;
constexpr int TERRAIN_HEIGHT_BUFFER = 15;

int terrain_ceiling(bool disable_height_limit, int extended_max_y)
{
	return (disable_height_limit ? extended_max_y : 319) - TERRAIN_HEIGHT_BUFFER;
}

struct FitRange
{
	double height_range;
	double scaled_range;
};

std::pair<ElevationAffine, FitRange> derive_affine(
		const std::vector<std::vector<double>> &grid, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y)
{
	double lo = std::numeric_limits<double>::max();
	double hi = std::numeric_limits<double>::lowest();
	for (const auto &row : grid)
		for (double value : row)
			if (std::isfinite(value)) {
				lo = std::min(lo, value);
				hi = std::max(hi, value);
			}
	double height_range;
	if (!std::isfinite(lo) || !std::isfinite(hi) || lo >= hi) {
		const double real_min = std::isfinite(lo) && lo <= hi ? lo : 0.0;
		lo = real_min;
		height_range = 0.0;
	} else {
		height_range = hi - lo;
	}
	const double ideal_scaled_range = height_range * scale;
	const int ceiling = terrain_ceiling(disable_height_limit, extended_max_y);
	if (disable_height_limit && min_ground_level < ground_level &&
			std::isfinite(ideal_scaled_range)) {
		const double rounded = std::ceil(ideal_scaled_range);
		const int needed = rounded >= std::numeric_limits<int>::max()
								   ? std::numeric_limits<int>::max()
						   : rounded <= std::numeric_limits<int>::min()
								   ? std::numeric_limits<int>::min()
								   : static_cast<int>(rounded);
		const long long candidate = static_cast<long long>(ceiling) - needed;
		ground_level = static_cast<int>(
				std::clamp(candidate, static_cast<long long>(min_ground_level),
						static_cast<long long>(ground_level)));
	}
	const double available_y_range = static_cast<double>(ceiling - ground_level);
	const double scaled_range =
			ideal_scaled_range <= available_y_range
					? ideal_scaled_range
					: height_range * (available_y_range / height_range);
	const double blocks_per_meter =
			height_range > 0.0 ? scaled_range / height_range : 0.0;
	return {{lo, blocks_per_meter, ground_level, std::nullopt},
			{height_range, scaled_range}};
}
} // namespace

double ElevationAffine::y_for_metres(double height_m) const
{
	if (soft_top && height_m > soft_top->knee_m) {
		const double knee_y =
				ground_level + (soft_top->knee_m - min_height_m) * blocks_per_meter;
		const double rise = (height_m - soft_top->knee_m) * blocks_per_meter;
		return knee_y +
			   soft_top->width_blocks * std::asinh(rise / soft_top->width_blocks);
	}
	return ground_level + (height_m - min_height_m) * blocks_per_meter;
}

ElevationAffine ElevationAffine::whole_earth(double scale, int floor, int extended_max_y)
{
	ElevationAffine affine{LOWEST_LAND_M, scale, floor, std::nullopt};
	const double ceiling = terrain_ceiling(true, extended_max_y);
	const double knee_y = ceiling - SOFT_TOP_BLOCKS;
	if (affine.y_for_metres(HIGHEST_LAND_M) <= ceiling || knee_y <= floor)
		return affine;
	const double knee_m = LOWEST_LAND_M + (knee_y - floor) / scale;
	const double rise = (HIGHEST_LAND_M - knee_m) * scale;
	double lo = 1e-3, hi = 1e9;
	for (int i = 0; i < 200; ++i) {
		const double mid = (lo + hi) * 0.5;
		if (mid * std::asinh(rise / mid) > SOFT_TOP_BLOCKS)
			hi = mid;
		else
			lo = mid;
	}
	affine.soft_top = SoftTop{knee_m, lo};
	return affine;
}

std::pair<std::vector<std::vector<double>>, ElevationAffine> scale_to_minecraft_with(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y,
		const AffinePolicy &policy)
{
	const int ceiling = terrain_ceiling(disable_height_limit, extended_max_y);
	std::optional<FitRange> fit;
	ElevationAffine affine;
	switch (policy.kind) {
	case AffinePolicy::Kind::Fit: {
		auto derived = derive_affine(in, scale, ground_level, min_ground_level,
				disable_height_limit, extended_max_y);
		affine = derived.first;
		fit = derived.second;
		break;
	}
	case AffinePolicy::Kind::FitWithHeadroom: {
		auto derived = derive_affine(in, scale, ground_level, min_ground_level,
				disable_height_limit, extended_max_y);
		affine = derived.first;
		fit = derived.second;
		if (affine.blocks_per_meter <= 0.0)
			affine.blocks_per_meter = scale;
		const double free = (ceiling - affine.ground_level) - fit->scaled_range;
		const double margin =
				std::max(0.0, std::floor(std::min(free * 0.5, HEADROOM_MAX_BLOCKS)));
		if (margin > 0.0 && affine.blocks_per_meter > 0.0)
			affine.min_height_m -= margin / affine.blocks_per_meter;
		fit.reset();
		break;
	}
	case AffinePolicy::Kind::Fixed:
		affine = policy.fixed;
		break;
	}
	const double base = affine.ground_level;
	const double upper = ceiling;
	std::vector<std::vector<double>> out = in;
	for (std::size_t y = 0; y < in.size(); ++y)
		for (std::size_t x = 0; x < in[y].size(); ++x) {
			const double value = in[y][x];
			double mapped;
			if (fit) {
				const double relative =
						fit->height_range > 0.0
								? (value - affine.min_height_m) / fit->height_range
								: 0.0;
				mapped = base + relative * fit->scaled_range;
			} else {
				mapped = affine.y_for_metres(value);
			}
			out[y][x] = std::clamp(mapped, base, upper);
		}
	if (policy.kind == AffinePolicy::Kind::Fixed) {
		std::size_t total = 0, clamped = 0;
		for (const auto &row : in)
			for (double value : row)
				if (std::isfinite(value)) {
					++total;
					const double mapped = affine.y_for_metres(value);
					clamped += (mapped < base && value > LOWEST_LAND_M) || mapped > upper;
				}
		if (total && clamped * 200 > total)
			std::clog
					<< "Warning: " << (100.0 * clamped / total)
					<< "% of this area lies outside the world's height band and is flattened there.\n";
	}
	double top = base;
	if (fit)
		top = affine.ground_level + fit->scaled_range;
	else
		for (const auto &row : out)
			for (double value : row)
				if (std::isfinite(value))
					top = std::max(top, value);
	world_editor::set_terrain_top_y(static_cast<int>(std::lround(std::min(top, upper))));
	return {std::move(out), affine};
}

std::tuple<std::vector<std::vector<double>>, double, double, int> scale_to_minecraft(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y)
{
	auto [heights, affine] = scale_to_minecraft_with(in, scale, ground_level,
			min_ground_level, disable_height_limit, extended_max_y, AffinePolicy::fit());
	return {std::move(heights), affine.min_height_m, affine.blocks_per_meter,
			affine.ground_level};
}
}
