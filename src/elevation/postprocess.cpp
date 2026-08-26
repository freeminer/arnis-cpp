#include "postprocess.h"
#include "elevation.h"
#include "../land_cover/land_cover.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
namespace arnis::elevation
{
void repair_terrain_anomalies(std::vector<std::vector<double>> &h)
{
	if (h.size() < 5 || h[0].size() < 5)
		return;
	const int H = h.size(), W = h[0].size();
	for (int pass = 0; pass < 10; ++pass) {
		auto s = h;
		std::size_t changed = 0;
		for (int y = 2; y < H - 2; ++y)
			for (int x = 2; x < W - 2; ++x) {
				double c = s[y][x];
				if (!std::isfinite(c))
					continue;
				std::vector<double> n;
				for (int dy = -2; dy <= 2; ++dy)
					for (int dx = -2; dx <= 2; ++dx)
						if (dx || dy)
							if (std::isfinite(s[y + dy][x + dx]))
								n.push_back(s[y + dy][x + dx]);
				if (n.size() < 8)
					continue;
				std::nth_element(n.begin(), n.begin() + n.size() / 2, n.end());
				double m = n[n.size() / 2];
				std::vector<double> d;
				for (double v : n)
					d.push_back(std::abs(v - m));
				std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
				double mad = d[d.size() / 2];
				if (std::abs(c - m) > 6.0 && std::abs(c - m) > 3.0 * std::max(1.0, mad)) {
					h[y][x] = m;
					++changed;
				}
			}
		if (!changed)
			break;
	}
}

namespace
{
using HeightGrid = std::vector<std::vector<double>>;
using CoverGrid = std::vector<std::vector<std::uint8_t>>;
using MaskGrid = std::vector<std::vector<std::uint8_t>>;
using Cell = std::pair<std::size_t, std::size_t>;

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

std::vector<double> smooth_sparse_water_field(const std::vector<Cell> &cells,
		const std::vector<double> &values, double sigma)
{
	std::vector<double> out(values.size());
	if (cells.size() != values.size() || values.empty() || sigma < 1.5)
		return values;
	const int radius = std::max(1, static_cast<int>(std::ceil(3.0 * sigma)));
	const int bin_size = std::max(1, static_cast<int>(std::ceil(sigma)));
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins;
	const auto bin_key = [](int x, int y) {
		return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
				static_cast<std::uint32_t>(y);
	};
	for (std::size_t i = 0; i < cells.size(); ++i)
		bins[bin_key(static_cast<int>(cells[i].first) / bin_size,
				static_cast<int>(cells[i].second) / bin_size)].push_back(i);
	for (std::size_t i = 0; i < cells.size(); ++i) {
		const int x = static_cast<int>(cells[i].first), y = static_cast<int>(cells[i].second);
		const int bx = x / bin_size, by = y / bin_size;
		double weighted = 0.0, weight_sum = 0.0;
		for (int yy = by - 3; yy <= by + 3; ++yy)
			for (int xx = bx - 3; xx <= bx + 3; ++xx) {
				auto found = bins.find(bin_key(xx, yy));
				if (found == bins.end())
					continue;
				for (const auto j : found->second) {
					const double dx = static_cast<double>(cells[j].first) - x;
					const double dy = static_cast<double>(cells[j].second) - y;
					const double distance = std::hypot(dx, dy);
					if (distance > radius || !std::isfinite(values[j]))
						continue;
					const double weight = std::exp(-0.5 * distance * distance /
							(sigma * sigma));
					weighted += values[j] * weight;
					weight_sum += weight;
				}
			}
		out[i] = weight_sum > 0.0 ? weighted / weight_sum : values[i];
	}
	return out;
}

bool has_non_water_neighbor(const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = cover.size(), w = cover.front().size();
	for (const auto [dx, dy] : CARDINAL) {
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
	const auto [lo, hi] = std::minmax_element(values.begin(), values.end());
	if (*hi - *lo < bin_size)
		return *lo;
	const auto count = static_cast<std::size_t>(std::ceil((*hi - *lo) / bin_size)) + 1;
	std::vector<std::size_t> bins(count);
	for (double value : values) {
		const auto index = std::min(count - 1,
				static_cast<std::size_t>((value - *lo) / bin_size));
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
	for (const auto [x, y] : component)
		for (const auto [dx, dy] : CARDINAL) {
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

std::optional<double> local_water_median(const HeightGrid &heights,
		const CoverGrid &cover, std::size_t x, std::size_t y, int radius)
{
	std::vector<double> samples;
	samples.reserve(static_cast<std::size_t>((radius * 2 + 1) * (radius * 2 + 1)));
	const auto h = heights.size(), w = heights.front().size();
	for (int dy = -radius; dy <= radius; ++dy)
		for (int dx = -radius; dx <= radius; ++dx) {
			const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
			if (inside(nx, ny, w, h) && cover[ny][nx] == land_cover::LC_WATER &&
					std::isfinite(heights[ny][nx]))
				samples.push_back(heights[ny][nx]);
		}
	if (samples.size() < 8)
		return std::nullopt;
	return median(std::move(samples));
}

std::optional<double> lowest_adjacent_land(const HeightGrid &heights,
		const CoverGrid &cover, std::size_t x, std::size_t y)
{
	const auto h = heights.size(), w = heights.front().size();
	double lowest = std::numeric_limits<double>::infinity();
	for (const auto [dx, dy] : CARDINAL) {
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
	if (heights.empty() || heights.front().empty() || meters_per_cell <= 0.0 ||
			!std::isfinite(meters_per_cell))
		return 0;
	const auto h = heights.size(), w = heights.front().size();
	const int step = std::clamp(static_cast<int>(std::llround(10.0 / meters_per_cell)), 1, 16);
	const auto max_cells = static_cast<std::size_t>(
			MAX_STEEP_WATER_AREA_M2 / (meters_per_cell * meters_per_cell));
	MaskGrid visited(h, std::vector<std::uint8_t>(w));
	std::vector<Cell> dropped;
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			if (visited[sy][sx] || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			std::deque<Cell> queue{{sx, sy}};
			std::vector<Cell> component;
			visited[sy][sx] = 1;
			while (!queue.empty()) {
				auto [x, y] = queue.front();
				queue.pop_front();
				component.emplace_back(x, y);
				for (const auto [dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
					if (inside(nx, ny, w, h) && !visited[ny][nx] &&
							cover[ny][nx] == land_cover::LC_WATER) {
						visited[ny][nx] = 1;
						queue.emplace_back(nx, ny);
					}
				}
			}
			if (component.size() > max_cells)
				continue;
			std::vector<double> slopes;
			for (const auto [x, y] : component) {
				const double here = heights[y][x];
				if (!std::isfinite(here))
					continue;
				double gx = 0.0, gy = 0.0;
				int axes = 0;
				for (const auto [dx, dy] : std::array<std::pair<int, int>, 2>{{{1, 0}, {0, 1}}}) {
					const int lo_x = static_cast<int>(x) - dx * step;
					const int lo_y = static_cast<int>(y) - dy * step;
					const int hi_x = static_cast<int>(x) + dx * step;
					const int hi_y = static_cast<int>(y) + dy * step;
					if (!inside(lo_x, lo_y, w, h) || !inside(hi_x, hi_y, w, h) ||
							cover[lo_y][lo_x] != land_cover::LC_WATER ||
							cover[hi_y][hi_x] != land_cover::LC_WATER)
						continue;
					const double gradient = (heights[hi_y][hi_x] - heights[lo_y][lo_x]) /
							(2.0 * step * meters_per_cell);
					if (dx)
						gx = gradient;
					else
						gy = gradient;
					++axes;
				}
				if (axes)
					slopes.push_back(std::hypot(gx, gy));
			}
			if (slopes.empty() || median(std::move(slopes)) <= MIN_STEEP_WATER_SLOPE)
				continue;
			std::unordered_set<std::uint64_t> own;
			for (const auto [x, y] : component)
				own.insert(static_cast<std::uint64_t>(y) * w + x);
			std::size_t edge = 0, perched = 0;
			for (const auto [x, y] : component) {
				const double here = heights[y][x];
				double lowest = std::numeric_limits<double>::infinity();
				for (const auto [dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx * step;
					const int ny = static_cast<int>(y) + dy * step;
					if (!inside(nx, ny, w, h) || own.contains(static_cast<std::uint64_t>(ny) * w + nx))
						continue;
					if (std::isfinite(heights[ny][nx]))
						lowest = std::min(lowest, heights[ny][nx]);
				}
				if (!std::isfinite(here) || !std::isfinite(lowest))
					continue;
				++edge;
				perched += lowest < here - STEEP_WATER_LAND_BELOW_M;
			}
			if (edge && static_cast<double>(perched) >= MIN_PERCHED_FRACTION * edge)
				dropped.insert(dropped.end(), component.begin(), component.end());
		}
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	for (const auto [x, y] : dropped)
		replacements.emplace_back(x, y,
				nearest_non_water_class(cover, x, y, 8).value_or(land_cover::LC_BARE));
	for (const auto [x, y, value] : replacements)
		cover[y][x] = value;
	return replacements.size();
}

MaskGrid level_water_surfaces(
		HeightGrid &heights, const CoverGrid &cover, double meters_per_cell)
{
	constexpr double UP_TOLERANCE = 2.0;
	const auto h = heights.size(), w = heights.front().size();
	const auto snapshot = heights;
	MaskGrid visited(h, std::vector<std::uint8_t>(w));
	MaskGrid surface_mask(h, std::vector<std::uint8_t>(w));
	for (std::size_t sy = 0; sy < h; ++sy)
		for (std::size_t sx = 0; sx < w; ++sx) {
			if (visited[sy][sx] || cover[sy][sx] != land_cover::LC_WATER)
				continue;
			std::deque<Cell> queue{{sx, sy}};
			std::vector<Cell> component;
			std::vector<double> values;
			visited[sy][sx] = 1;
			while (!queue.empty()) {
				auto [x, y] = queue.front();
				queue.pop_front();
				component.emplace_back(x, y);
				if (std::isfinite(snapshot[y][x]))
					values.push_back(snapshot[y][x]);
				for (const auto [dx, dy] : CARDINAL) {
					const int nx = static_cast<int>(x) + dx, ny = static_cast<int>(y) + dy;
					if (inside(nx, ny, w, h) && !visited[ny][nx] &&
							cover[ny][nx] == land_cover::LC_WATER) {
						visited[ny][nx] = 1;
						queue.emplace_back(nx, ny);
					}
				}
			}
			if (values.empty())
				continue;
			const double fallback = median(values);
			const bool flowing = interquartile_range(values) > 5.0;
			const double still_surface = clamp_by_adjacent_land(
					values.size() >= 16 ? histogram_mode(values, 1.0) : fallback,
					component, snapshot, cover);
			std::vector<double> local_levels;
			std::vector<Cell> flowing_cells;
			for (const auto [x, y] : component) {
				const double original = snapshot[y][x];
				if (!std::isfinite(original))
					continue;
				flowing_cells.push_back({x, y});
				local_levels.push_back(flowing
						? local_water_median(snapshot, cover, x, y, 12).value_or(fallback)
						: still_surface);
			}
			if (flowing) {
				const double sigma_cells = meters_per_cell > 0.0
						? std::min(64.0, 40.0 / meters_per_cell) : 0.0;
				local_levels = smooth_sparse_water_field(flowing_cells, local_levels,
						sigma_cells);
			}
			for (std::size_t i = 0; i < flowing_cells.size(); ++i) {
				const auto [x, y] = flowing_cells[i];
				const double original = snapshot[y][x];
				double level = local_levels[i];
				if (flowing)
					level = original + std::clamp(level - original, -1.5, 1.5);
				if (flowing)
					if (auto land = lowest_adjacent_land(snapshot, cover, x, y))
						level = std::min(level, std::max(original, *land));
				if (original <= level + UP_TOLERANCE || !has_non_water_neighbor(cover, x, y)) {
					heights[y][x] = level;
					surface_mask[y][x] = 1;
				}
			}
		}
	(void)meters_per_cell;
	return surface_mask;
}

std::size_t reclassify_non_surface_water_cells(CoverGrid &cover, const MaskGrid &surface)
{
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> replacements;
	for (std::size_t y = 0; y < cover.size(); ++y)
		for (std::size_t x = 0; x < cover[y].size(); ++x)
			if (cover[y][x] == land_cover::LC_WATER && !surface[y][x])
				replacements.emplace_back(x, y,
						nearest_non_water_class(cover, x, y, 8).value_or(land_cover::LC_BARE));
	for (const auto [x, y, value] : replacements)
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
	HeightGrid water_level(h,
			std::vector<double>(w, std::numeric_limits<double>::quiet_NaN()));
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
		for (const auto [dx, dy] : CARDINAL) {
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
	auto source = heights;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x)
			if (surface[y][x])
				source[y][x] = std::numeric_limits<double>::quiet_NaN();
	auto blurred = gaussian_blur_grid(source, sigma);
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			if (surface[y][x])
				continue;
			const double weight = std::clamp(feathered[y][x], 0.0, 1.0);
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
	if (heights.empty() || heights.front().empty() || data.width != heights.front().size() ||
			data.height != heights.size() || data.grid.size() != data.height)
		return;
	const auto dropped = drop_water_on_steep_terrain(heights, data.grid, meters_per_cell);
	auto surface = level_water_surfaces(heights, data.grid, meters_per_cell);
	const auto reclassified = reclassify_non_surface_water_cells(data.grid, surface);
	if (dropped + reclassified) {
		data.water_distance = land_cover::compute_water_distance(
				data.grid, data.width, data.height);
		data.water_blend_grid.clear();
	}
	smooth_built_up_gaussian(
			heights, data.grid, surface, built_up_sigma_cells, report);
	pull_coastal_land_toward_water(
			heights, surface, coastal_pull_distance_cells);
}

std::tuple<std::vector<std::vector<double>>, double, double, int> scale_to_minecraft(
		const std::vector<std::vector<double>> &in, double scale, int ground_level,
		int min_ground_level, bool disable_height_limit, int extended_max_y)
{
	double lo = 1e300, hi = -1e300;
	for (auto &r : in)
		for (double v : r)
			if (std::isfinite(v)) {
				lo = std::min(lo, v);
				hi = std::max(hi, v);
			}
	if (!(lo <= hi) || !std::isfinite(lo)) {
		lo = 0;
		hi = 0;
	}
	const double range = hi - lo;
	const int effective_max_y = disable_height_limit ? extended_max_y : 319;
	const int ceiling = effective_max_y - 15;
	const double ideal_scaled_range = range * scale;
	if (disable_height_limit && min_ground_level < ground_level &&
			std::isfinite(ideal_scaled_range)) {
		const int needed = ideal_scaled_range >= double(std::numeric_limits<int>::max())
								   ? std::numeric_limits<int>::max()
								   : int(std::ceil(std::max(0.0, ideal_scaled_range)));
		const long long candidate = static_cast<long long>(ceiling) - needed;
		ground_level = std::clamp(
				int(std::clamp(candidate,
						static_cast<long long>(std::numeric_limits<int>::min()),
						static_cast<long long>(std::numeric_limits<int>::max()))),
				min_ground_level, ground_level);
	}
	const double available_y_range = std::max(0.0, double(ceiling - ground_level));
	const double scaled_range = ideal_scaled_range <= available_y_range
										? ideal_scaled_range
										: (range > 0.0 ? available_y_range : 0.0);
	const double factor = range > 0.0 ? scaled_range / range : 0.0;
	std::vector<std::vector<double>> out = in;
	for (auto &r : out)
		for (double &v : r)
			v = std::isfinite(v) ? std::clamp(ground_level + (v - lo) * factor,
										   double(ground_level), double(ceiling))
								 : ground_level;
	return {std::move(out), lo, factor, ground_level};
}
}
