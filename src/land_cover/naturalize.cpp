#include "naturalize.h"

#include "../ground_generation.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace arnis::land_cover
{
namespace
{
constexpr double WATER_SPAN_M = 50.0;
constexpr float WATER_HEIGHT_TOLERANCE_BLOCKS = 6.0f;
constexpr double LATTICE_M = 10.0;
constexpr double WARP_M = 40.0;
constexpr double WARP_WAVELENGTH_M = 60.0;
constexpr std::uint32_t SALT_WARP_X = 0x4E41'5458;
constexpr std::uint32_t SALT_WARP_Z = 0x4E41'545A;

bool is_wild(std::uint8_t c, biome::Climate climate)
{
	switch (c) {
	case LC_TREE_COVER:
	case LC_SHRUBLAND:
	case LC_GRASSLAND:
	case LC_MOSS:
		return true;
	case LC_BARE:
		return climate == biome::Climate::HotDesert ||
			   climate == biome::Climate::ColdDesert ||
			   climate == biome::Climate::Tundra || climate == biome::Climate::IceCap;
	default:
		return false;
	}
}

void dilate_square(std::vector<std::uint8_t> &cells, std::size_t w, std::size_t h,
		std::size_t radius, bool edge_is_set)
{
	if (w == 0 || h == 0 || cells.size() < w * h)
		return;
	const auto start =
			edge_is_set ? std::size_t(0) : std::numeric_limits<std::size_t>::max() / 2;
	auto reach = [radius](std::uint8_t &cell, std::size_t &since) {
		since = (cell & 1) ? 0 : since + 1;
		if (since <= radius)
			cell |= 2;
	};
	for (std::size_t z = 0; z < h; ++z) {
		auto row = cells.begin() + z * w;
		auto since = start;
		for (std::size_t x = 0; x < w; ++x)
			reach(row[x], since);
		since = start;
		for (std::size_t x = w; x-- > 0;)
			reach(row[x], since);
		for (std::size_t x = 0; x < w; ++x)
			row[x] >>= 1;
	}
	std::vector<std::size_t> since(w, start);
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x)
			reach(cells[z * w + x], since[x]);
	std::fill(since.begin(), since.end(), start);
	for (std::size_t z = h; z-- > 0;)
		for (std::size_t x = 0; x < w; ++x)
			reach(cells[z * w + x], since[x]);
	for (std::size_t i = 0; i < w * h; ++i)
		cells[i] >>= 1;
}

float min_level(float a, float b)
{
	if (std::isfinite(a) && std::isfinite(b))
		return std::min(a, b);
	return std::isfinite(a) ? a : b;
}

bool reachable(float cell, float level)
{
	return !std::isfinite(cell) || !std::isfinite(level) ||
		   cell <= level + WATER_HEIGHT_TOLERANCE_BLOCKS;
}

bool reclaim_water(LandCoverData &data, std::vector<std::vector<float>> *heights)
{
	const auto w = data.width, h = data.height;
	if (w == 0 || h == 0 || data.grid.size() < h ||
			std::none_of(data.grid.begin(), data.grid.begin() + h, [](const auto &row) {
				return std::find(row.begin(), row.end(), LC_WATER) != row.end();
			}))
		return false;
	const auto has_heights = heights && heights->size() >= h &&
							 std::all_of(heights->begin(), heights->begin() + h,
									 [&](const auto &row) { return row.size() >= w; });
	const auto radius = std::max<std::size_t>(
			1, static_cast<std::size_t>(std::max(
					   0.0, std::round(WATER_SPAN_M * 0.5 * data.cells_per_meter))));
	std::vector<std::uint8_t> mask(w * h);
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x)
			mask[z * w + x] = data.grid[z][x] == LC_WATER;
	dilate_square(mask, w, h, radius, false);
	for (auto &c : mask)
		c ^= 1;
	dilate_square(mask, w, h, radius, true);
	auto closed = [&](std::size_t x, std::size_t z) {
		return mask[z * w + x] == 0 && data.grid[z][x] == LC_BUILT_UP;
	};
	auto level_at = [&](std::size_t x, std::size_t z) {
		return has_heights ? (*heights)[z][x] : std::numeric_limits<float>::quiet_NaN();
	};
	constexpr int dx[] = {-1, 1, 0, 0};
	constexpr int dz[] = {0, 0, -1, 1};
	auto each_neighbour = [&](std::size_t x, std::size_t z, auto &&fn) {
		for (int i = 0; i < 4; ++i) {
			const auto nx = static_cast<std::int64_t>(x) + dx[i];
			const auto nz = static_cast<std::int64_t>(z) + dz[i];
			if (nx >= 0 && nz >= 0 && static_cast<std::size_t>(nx) < w &&
					static_cast<std::size_t>(nz) < h)
				fn(static_cast<std::size_t>(nx), static_cast<std::size_t>(nz));
		}
	};
	const auto absent = std::numeric_limits<std::size_t>::max();
	std::vector<float> reclaimed(w * h, std::numeric_limits<float>::quiet_NaN());
	std::vector<std::uint8_t> seen(w * h, 0);
	std::deque<std::size_t> queue;
	for (std::size_t z = 0; z < h; ++z) {
		for (std::size_t x = 0; x < w; ++x) {
			if (!closed(x, z))
				continue;
			float level = std::numeric_limits<float>::quiet_NaN();
			each_neighbour(x, z, [&](std::size_t nx, std::size_t nz) {
				if (data.grid[nz][nx] == LC_WATER)
					level = min_level(level, level_at(nx, nz));
			});
			if (std::isnan(level)) {
				// No adjacent water seed. NaN is also a valid water level, so check adjacency separately.
				bool touches_water = false;
				each_neighbour(x, z, [&](std::size_t nx, std::size_t nz) {
					touches_water |= data.grid[nz][nx] == LC_WATER;
				});
				if (!touches_water)
					continue;
			}
			if (!reachable(level_at(x, z), level))
				continue;
			const auto i = z * w + x;
			seen[i] = 1;
			reclaimed[i] = level;
			queue.push_back(i);
		}
	}
	while (!queue.empty()) {
		const auto i = queue.front();
		queue.pop_front();
		const auto x = i % w, z = i / w;
		each_neighbour(x, z, [&](std::size_t nx, std::size_t nz) {
			const auto ni = nz * w + nx;
			if (!seen[ni] && closed(nx, nz) &&
					reachable(level_at(nx, nz), reclaimed[i])) {
				seen[ni] = 1;
				reclaimed[ni] = reclaimed[i];
				queue.push_back(ni);
			}
		});
	}
	bool changed = false;
	for (std::size_t i = 0; i < w * h; ++i) {
		if (!seen[i])
			continue;
		const auto x = i % w, z = i / w;
		data.grid[z][x] = LC_WATER;
		if (has_heights) {
			auto &cell = (*heights)[z][x];
			if (std::isfinite(reclaimed[i]) && std::isfinite(cell) && cell > reclaimed[i])
				cell = reclaimed[i];
		}
		changed = true;
	}
	return changed;
}

void chessboard_labels(std::vector<std::uint8_t> &label, std::vector<std::uint16_t> &dist,
		std::size_t w, std::size_t h)
{
	auto relax = [&](std::size_t i, std::size_t j) {
		const auto d = dist[j] == std::numeric_limits<std::uint16_t>::max()
							   ? dist[j]
							   : static_cast<std::uint16_t>(dist[j] + 1);
		if (d < dist[i]) {
			dist[i] = d;
			label[i] = label[j];
		}
	};
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto i = z * w + x;
			if (x)
				relax(i, i - 1);
			if (z) {
				relax(i, i - w);
				if (x)
					relax(i, i - w - 1);
				if (x + 1 < w)
					relax(i, i - w + 1);
			}
		}
	for (std::size_t z = h; z-- > 0;)
		for (std::size_t x = w; x-- > 0;) {
			const auto i = z * w + x;
			if (x + 1 < w)
				relax(i, i + 1);
			if (z + 1 < h) {
				relax(i, i + w);
				if (x + 1 < w)
					relax(i, i + w + 1);
				if (x)
					relax(i, i + w - 1);
			}
		}
}

void fill_with_nearest_wild(LandCoverData &data, biome::Climate climate)
{
	const auto w = data.width, h = data.height;
	if (!w || !h)
		return;
	const double cpm = std::max(1e-6, data.cells_per_meter);
	const auto step = std::max<std::size_t>(
			1, static_cast<std::size_t>(std::max(0.0, std::round(LATTICE_M * cpm))));
	const auto lw = w / step + (w % step != 0), lh = h / step + (h % step != 0);
	std::vector<std::uint8_t> label(lw * lh, 0);
	std::vector<std::uint16_t> dist(lw * lh, std::numeric_limits<std::uint16_t>::max());
	for (std::size_t lz = 0; lz < lh; ++lz) {
		const auto z = std::min(h - 1, lz * step + step / 2);
		for (std::size_t lx = 0; lx < lw; ++lx) {
			const auto x = std::min(w - 1, lx * step + step / 2);
			const auto c = data.grid[z][x];
			if (is_wild(c, climate)) {
				label[lz * lw + lx] = c;
				dist[lz * lw + lx] = 0;
			}
		}
	}
	chessboard_labels(label, dist, lw, lh);
	const auto fallback = is_wild(LC_BARE, climate) ? LC_BARE : LC_GRASSLAND;
	const double warp = WARP_M * cpm;
	const auto wavelength = std::max(
			1, static_cast<int>(std::max(0.0, std::round(WARP_WAVELENGTH_M * cpm))));
	for (std::size_t z = 0; z < h; ++z) {
		for (std::size_t x = 0; x < w; ++x) {
			auto &c = data.grid[z][x];
			if (c != LC_BUILT_UP)
				continue;
			const int xi = static_cast<int>(x), zi = static_cast<int>(z);
			const double wx = (ground_generation::value_noise_salted(
									   xi, zi, wavelength, SALT_WARP_X) -
									  .5) *
							  2.0 * warp;
			const double wz = (ground_generation::value_noise_salted(
									   xi, zi, wavelength, SALT_WARP_Z) -
									  .5) *
							  2.0 * warp;
			const auto lx = static_cast<std::size_t>(std::clamp(
					std::floor((x + wx) / step), 0.0, static_cast<double>(lw - 1)));
			const auto lz = static_cast<std::size_t>(std::clamp(
					std::floor((z + wz) / step), 0.0, static_cast<double>(lh - 1)));
			const auto nearest = label[lz * lw + lx];
			c = nearest ? nearest : fallback;
		}
	}
}
}

void naturalize(LandCoverData &data, std::vector<std::vector<float>> *heights,
		biome::Climate climate)
{
	const auto w = data.width, h = data.height;
	if (!w || !h || data.grid.size() < h)
		return;
	bool built_up = false;
	for (std::size_t z = 0; z < h; ++z) {
		if (data.grid[z].size() < w)
			return;
		for (auto &c : data.grid[z]) {
			if (c == LC_CROPLAND)
				c = LC_GRASSLAND;
			built_up |= c == LC_BUILT_UP;
		}
	}
	if (!built_up)
		return;
	if (reclaim_water(data, heights)) {
		data.water_distance = compute_water_distance(data.grid, w, h);
		data.invalidate_water_blend_grid();
	}
	fill_with_nearest_wild(data, climate);
}
}
