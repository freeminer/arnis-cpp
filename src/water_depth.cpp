#include "water_depth.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

#include "land_cover/land_cover.h"

namespace arnis::water_depth
{

namespace
{

constexpr int MIN_Y = -64;
constexpr std::uint16_t SHOAL_DT_UNITS = 9;
constexpr std::uint8_t DT_MAX = std::numeric_limits<std::uint8_t>::max();
constexpr int MAX_WATER_DEPTH = 6;
constexpr std::size_t MAX_WATER_FIELD_CELLS = 1'000'000'000ULL;

std::uint8_t nibble_get(const std::vector<std::uint8_t> &buf, std::size_t i)
{
	const auto byte = buf[i >> 1];
	return (i & 1U) == 0 ? (byte & 0x0F) : (byte >> 4);
}

void nibble_set(std::vector<std::uint8_t> &buf, std::size_t i, std::uint8_t v)
{
	auto &byte = buf[i >> 1];
	if ((i & 1U) == 0) {
		byte = static_cast<std::uint8_t>((byte & 0xF0) | (v & 0x0F));
	} else {
		byte = static_cast<std::uint8_t>((byte & 0x0F) | ((v & 0x0F) << 4));
	}
}

bool bit_get(const std::vector<std::uint64_t> &bits, std::size_t i)
{
	return ((bits[i >> 6] >> (i & 63U)) & 1ULL) != 0;
}

void bit_set(std::vector<std::uint64_t> &bits, std::size_t i)
{
	bits[i >> 6] |= 1ULL << (i & 63U);
}

double value_noise_01(int x, int z, int scale)
{
	const int sx = scale > 0 ? x / scale : x;
	const int sz = scale > 0 ? z / scale : z;
	return static_cast<double>(land_cover::coord_hash(sx, sz) & 0x00FF'FFFFULL) /
		   static_cast<double>(0x0100'0000ULL);
}

void chamfer_3_4_dt(std::vector<std::uint8_t> &d, std::size_t w, std::size_t h)
{
	auto step = [](std::uint8_t v, std::uint8_t add) {
		const unsigned sum = static_cast<unsigned>(v) + static_cast<unsigned>(add);
		return static_cast<std::uint8_t>(std::min<unsigned>(sum, DT_MAX));
	};

	for (std::size_t j = 0; j < h; ++j) {
		for (std::size_t i = 0; i < w; ++i) {
			const std::size_t idx = j * w + i;
			if (d[idx] == 0)
				continue;
			std::uint8_t best = d[idx];
			if (i > 0)
				best = std::min(best, step(d[idx - 1], 3));
			if (j > 0) {
				best = std::min(best, step(d[idx - w], 3));
				if (i > 0)
					best = std::min(best, step(d[idx - w - 1], 4));
				if (i + 1 < w)
					best = std::min(best, step(d[idx - w + 1], 4));
			}
			d[idx] = best;
		}
	}

	for (std::size_t j = h; j-- > 0;) {
		for (std::size_t i = w; i-- > 0;) {
			const std::size_t idx = j * w + i;
			if (d[idx] == 0)
				continue;
			std::uint8_t best = d[idx];
			if (i + 1 < w)
				best = std::min(best, step(d[idx + 1], 3));
			if (j + 1 < h) {
				best = std::min(best, step(d[idx + w], 3));
				if (i > 0)
					best = std::min(best, step(d[idx + w - 1], 4));
				if (i + 1 < w)
					best = std::min(best, step(d[idx + w + 1], 4));
			}
			d[idx] = best;
		}
	}
}

int polygon_local_max(std::uint16_t component_max_units)
{
	if (component_max_units < 21)
		return 2;
	if (component_max_units < 45)
		return 3;
	if (component_max_units < 75)
		return 4;
	return 6;
}

int depth_from_dt(double dt_eff, std::uint16_t component_max_units)
{
	if (dt_eff < static_cast<double>(SHOAL_DT_UNITS))
		return 0;
	const int local_max = polygon_local_max(component_max_units);
	const double dist_blocks = (dt_eff - static_cast<double>(SHOAL_DT_UNITS)) / 3.0;
	const double span = component_max_units < 21 ? 6. : component_max_units < 45 ? 12. :
			component_max_units < 75 ? 20. : 35.;
	return std::clamp(static_cast<int>(std::floor(local_max * std::sqrt(std::clamp(dist_blocks / span, 0., 1.)))), 0, local_max);
}

int ocean_depth_for_cell(
		int x, int z, std::uint16_t dt_units, std::uint16_t component_max_units)
{
	if (dt_units == 0)
		return 0;
	const double wobble = (value_noise_01(x, z, 12) - 0.5) * 4.0;
	return depth_from_dt(static_cast<double>(dt_units) + wobble, component_max_units);
}

bool bridge_adjacent(const RoadMaskBitmap &road_mask, int x, int z)
{
	if (road_mask.is_empty())
		return false;
	for (int dz = -2; dz <= 2; ++dz) {
		for (int dx = -2; dx <= 2; ++dx) {
			if ((dx != 0 || dz != 0) && road_mask.contains(x + dx, z + dz))
				return true;
		}
	}
	return false;
}

int dune_amp(int body_max, int depth)
{
	const int target = body_max <= 3 ? 2 : body_max <= 5 ? 3 : 4;
	return std::min(target, depth - 1);
}

int place_underwater_dunes(WorldEditor &editor, int x, int z, int water_y, int bed_y,
		int depth, Block bed_block)
{
	// This legacy helper has no BigWaterField width; preserve its former
	// depth-driven amplitude while the region-aware path supplies body_max.
	const int amp = dune_amp(depth, depth);
	if (amp <= 0)
		return 0;
	const double warp_x = value_noise_01(x + 901, z + 33, 50);
	const double warp_z = value_noise_01(x + 17, z + 811, 50);
	const int wx = x + static_cast<int>((warp_x - 0.5) * 30.0);
	const int wz = z + static_cast<int>((warp_z - 0.5) * 30.0);
	const double h_f = 0.40 * value_noise_01(wx + 113, wz + 257, 44) +
					   0.30 * value_noise_01(wx + 31, wz + 71, 18) +
					   0.30 * value_noise_01(wx + 7, wz + 11, 10);
	if (h_f < 0.28)
		return 0;
	const double t = (h_f - 0.28) / 0.72;
	const int bump = std::clamp(
			static_cast<int>(std::floor(std::pow(t, 0.45) * (amp + 0.99))), 1, amp);
	for (int dy = 1; dy <= bump; ++dy) {
		const int y = bed_y + dy;
		if (y >= water_y)
			return dy - 1;
		editor.set_block_absolute(bed_block, x, y, z, std::nullopt, std::nullopt);
	}
	return bump;
}

void place_underwater_vegetation(
		WorldEditor &editor, int x, int z, int water_y, int bed_top, int depth)
{
	// Rust parity: src/water_depth.rs::place_underwater_vegetation.
	// Block substitutions come from src/mapgen/earth/blocks.cpp Earth-game mappings.
	const double field_noise = value_noise_01(x + 53, z + 89, 30);
	const double cluster_noise = value_noise_01(x + 401, z + 17, 10);
	const double combined = field_noise * cluster_noise;

	const int kelp_pick = static_cast<int>(land_cover::coord_hash(x + 91, z + 41) % 100);
	if (depth >= 4 && field_noise > 0.78 && cluster_noise > 0.80 && kelp_pick < 25) {
		const int plant_top_full = water_y - 1;
		const int plant_bottom = bed_top + 1;
		const int avail = plant_top_full - plant_bottom;
		if (avail >= 3) {
			const int hgt_pick =
					static_cast<int>(land_cover::coord_hash(x + 211, z + 503) % 100);
			const int share = 30 + hgt_pick * 70 / 100;
			const int used = std::clamp(
					static_cast<int>(static_cast<long long>(avail) * share / 100), 3,
					avail);
			const int plant_top = plant_bottom + used;
			for (int y = plant_bottom; y < plant_top; ++y)
				editor.set_block_absolute(
						KELP_PLANT, x, y, z, std::nullopt, std::nullopt);
			editor.set_block_absolute(KELP, x, plant_top, z, std::nullopt, std::nullopt);
		}
		return;
	}

	if (combined <= 0.42)
		return;
	const auto dropout = land_cover::coord_hash(x + 17, z + 31) % 10;
	if (dropout >= 4)
		return;
	const int plant_y = bed_top + 1;
	if (plant_y >= water_y)
		return;

	const int pick = static_cast<int>(land_cover::coord_hash(x + 211, z + 73) % 100);
	if (pick < 50) {
		editor.set_block_absolute(SEAGRASS, x, plant_y, z, std::nullopt, std::nullopt);
	} else if (pick < 85 && plant_y + 1 < water_y) {
		editor.set_block_absolute(
				TALL_SEAGRASS_BOTTOM, x, plant_y, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(
				TALL_SEAGRASS_TOP, x, plant_y + 1, z, std::nullopt, std::nullopt);
	} else {
		editor.set_block_absolute(SEA_PICKLE, x, plant_y, z, std::nullopt, std::nullopt);
	}
}

}

int BigWaterField::depth_at(int x, int z) const
{
	const auto lx = static_cast<long long>(x) - static_cast<long long>(min_x_);
	const auto lz = static_cast<long long>(z) - static_cast<long long>(min_z_);
	if (lx < 0 || lz < 0 || static_cast<std::size_t>(lx) >= width_ ||
			static_cast<std::size_t>(lz) >= height_)
		return 0;
	const auto idx = static_cast<std::size_t>(lz) * width_ + static_cast<std::size_t>(lx);
	return static_cast<int>(nibble_get(depth_, idx));
}
int BigWaterField::body_max_7x7(int x, int z) const
{
	int maximum = 0;
	for (int dz = -3; dz <= 3; ++dz)
		for (int dx = -3; dx <= 3; ++dx) {
			maximum = std::max(maximum, depth_at(x + dx, z + dz));
			if (maximum >= MAX_WATER_DEPTH) return maximum;
		}
	return maximum;
}

int estimate_max_carve_depth(const std::vector<std::vector<std::uint8_t>> &grid,
		std::size_t world_width, std::size_t world_height)
{
	const auto height = grid.size();
	const auto width = height ? grid.front().size() : 0;
	if (!width || !height) return 0;
	std::vector<std::uint8_t> dt(width * height);
	bool water = false;
	for (std::size_t z = 0; z < height; ++z) {
		if (grid[z].size() != width) return 0;
		for (std::size_t x = 0; x < width; ++x)
			if (grid[z][x] == land_cover::LC_WATER) { dt[z * width + x] = DT_MAX; water = true; }
	}
	if (!water) return 0;
	chamfer_3_4_dt(dt, width, height);
	const auto grid_max = *std::max_element(dt.begin(), dt.end());
	const double x_ratio = double(std::max<std::size_t>(1, world_width - 1)) / double(std::max<std::size_t>(1, width - 1));
	const double z_ratio = double(std::max<std::size_t>(1, world_height - 1)) / double(std::max<std::size_t>(1, height - 1));
	const double scaled = double(grid_max) * std::max({1., x_ratio, z_ratio});
	return depth_from_dt(scaled + 2., static_cast<std::uint16_t>(std::min(scaled, double(std::numeric_limits<std::uint16_t>::max()))));
}

BigWaterField compute_big_water_field(WorldEditor &editor, const XZBBox &xzbbox)
{
	// Rust parity: src/water_depth.rs::compute_big_water_field.
	// Uses Ground::lc_water_block_bounds when the C++ adapter has a land-cover grid.
	const int min_x = xzbbox.min_x();
	const int max_x = xzbbox.max_x();
	const int min_z = xzbbox.min_z();
	const int max_z = xzbbox.max_z();

	bool any_water = false;
	int wmin_x = max_x;
	int wmax_x = min_x;
	int wmin_z = max_z;
	int wmax_z = min_z;
	if (editor.get_ground() && editor.get_ground()->has_land_cover()) {
		if (const auto bounds = editor.get_ground()->lc_water_block_bounds()) {
			const auto [lx, lz, hx, hz] = *bounds;
			wmin_x = std::clamp(min_x + lx, min_x, max_x);
			wmin_z = std::clamp(min_z + lz, min_z, max_z);
			wmax_x = std::clamp(min_x + hx, min_x, max_x);
			wmax_z = std::clamp(min_z + hz, min_z, max_z);
			any_water = wmin_x <= wmax_x && wmin_z <= wmax_z;
		}
	} else {
		for (int z = min_z; z <= max_z; ++z) {
			for (int x = min_x; x <= max_x; ++x) {
				if (!editor.is_lc_water(x, z))
					continue;
				any_water = true;
				wmin_x = std::min(wmin_x, x);
				wmax_x = std::max(wmax_x, x);
				wmin_z = std::min(wmin_z, z);
				wmax_z = std::max(wmax_z, z);
			}
		}
	}
	if (!any_water)
		return {};

	const int smin_x = std::max(wmin_x - 1, min_x);
	const int smax_x = std::min(wmax_x + 1, max_x);
	const int smin_z = std::max(wmin_z - 1, min_z);
	const int smax_z = std::min(wmax_z + 1, max_z);
	const auto sw = static_cast<std::size_t>(smax_x - smin_x + 1);
	const auto sh = static_cast<std::size_t>(smax_z - smin_z + 1);
	const auto total = sw * sh;
	if (sw == 0 || sh == 0 || total > MAX_WATER_FIELD_CELLS || total / sw != sh) {
		std::cerr
				<< "Warning: water area too large for depth carving; rendering flat water\n";
		return {};
	}

	std::vector<std::uint8_t> dt(total, 0);
	for (int z = smin_z; z <= smax_z; ++z) {
		const auto row = static_cast<std::size_t>(z - smin_z) * sw;
		for (int x = smin_x; x <= smax_x; ++x) {
			if (editor.is_lc_water(x, z))
				dt[row + static_cast<std::size_t>(x - smin_x)] = DT_MAX;
		}
	}
	chamfer_3_4_dt(dt, sw, sh);

	BigWaterField out;
	out.depth_.assign((total + 1) / 2, 0);
	out.width_ = sw;
	out.height_ = sh;
	out.min_x_ = smin_x;
	out.min_z_ = smin_z;

	std::vector<std::uint64_t> visited((total + 63) / 64, 0);
	std::vector<std::uint32_t> comp;
	for (std::size_t start = 0; start < total; ++start) {
		if (dt[start] == 0 || bit_get(visited, start))
			continue;
		comp.clear();
		comp.push_back(static_cast<std::uint32_t>(start));
		bit_set(visited, start);
		std::uint8_t comp_max = 0;
		for (std::size_t head = 0; head < comp.size(); ++head) {
			const auto idx = static_cast<std::size_t>(comp[head]);
			comp_max = std::max(comp_max, dt[idx]);
			const auto i = idx % sw;
			const auto j = idx / sw;
			auto visit = [&](std::size_t n) {
				if (dt[n] != 0 && !bit_get(visited, n)) {
					bit_set(visited, n);
					comp.push_back(static_cast<std::uint32_t>(n));
				}
			};
			if (i > 0)
				visit(idx - 1);
			if (i + 1 < sw)
				visit(idx + 1);
			if (j > 0)
				visit(idx - sw);
			if (j + 1 < sh)
				visit(idx + sw);
		}
		const auto cm = static_cast<std::uint16_t>(comp_max);
		for (auto c : comp) {
			const auto idx = static_cast<std::size_t>(c);
			const int x = smin_x + static_cast<int>(idx % sw);
			const int z = smin_z + static_cast<int>(idx / sw);
			const int depth =
					ocean_depth_for_cell(x, z, static_cast<std::uint16_t>(dt[idx]), cm);
			nibble_set(out.depth_, idx, static_cast<std::uint8_t>(depth));
		}
	}

	return out;
}

void carve_water_column(WorldEditor &editor, int x, int z, int water_y, int depth,
		const RoadMaskBitmap &road_mask)
{
	depth = std::clamp(depth, 0, MAX_WATER_DEPTH);
	depth = std::min(depth, std::max(0, water_y - MIN_Y - 2));

	for (int dy = 0; dy <= depth; ++dy)
		editor.set_block_absolute(WATER, x, water_y - dy, z, std::nullopt, std::nullopt);

	const int bed_y = water_y - depth - 1;
	const bool near_bridge = depth >= 2 && bridge_adjacent(road_mask, x, z);

	Block top_block = SAND;
	Block under_block = SANDSTONE;
	if (depth >= 2 && near_bridge) {
		top_block = (value_noise_01(x, z, 6) < 0.4 && depth <= 3) ? SAND : GRAVEL;
		under_block = STONE;
	} else if (depth >= 2) {
		const double jn = value_noise_01(x + 7, z + 13, 22);
		const int jitter = jn < .34 ? -1 : jn > .66 ? 1 : 0;
		const int d = std::max(1, depth + jitter);
		const double warp_x = value_noise_01(x + 901, z + 33, 52);
		const double warp_z = value_noise_01(x + 17, z + 811, 52);
		const int wx = x + static_cast<int>((warp_x - 0.5) * 28.0);
		const int wz = z + static_cast<int>((warp_z - 0.5) * 28.0);
		auto vn = [&](int dx, int dz, int scale) {
			return value_noise_01(wx + dx, wz + dz, scale);
		};
		if (d <= 1)
			top_block = SAND;
		else if (d == 2)
			top_block = vn(53, 97, 56) > 0.50 ? SAND : GRAVEL;
		else if (d >= 5 && vn(401, 503, 8) > 0.96)
			top_block = MAGMA_BLOCK;
		else if (d >= 5 && vn(727, 911, 8) > 0.96)
			top_block = SOUL_SAND;
		else if (vn(73, 109, 64) > 0.74)
			top_block = CLAY;
		else if (vn(53, 97, 56) > 0.81)
			top_block = SAND;
		else if (vn(211, 41, 44) > 0.88)
			top_block = DIRT;
		else if (vn(311, 17, 50) > 0.90)
			top_block = COARSE_DIRT;
		else
			top_block = GRAVEL;
		under_block = STONE;
	}

	if (bed_y > MIN_Y)
		editor.set_block_absolute(top_block, x, bed_y, z, std::nullopt, std::nullopt);
	if (bed_y - 1 > MIN_Y)
		editor.set_block_absolute(
				under_block, x, bed_y - 1, z, std::nullopt, std::nullopt);

	const int fill_to = std::max(bed_y - 2, MIN_Y + 1);
	const int fill_from = std::max(bed_y - 12, MIN_Y + 1);
	if (fill_from <= fill_to)
		editor.fill_column_absolute(STONE, x, z, fill_from, fill_to, true);

	const int bump = depth >= 2 && !near_bridge
							 ? place_underwater_dunes(
									   editor, x, z, water_y, bed_y, dune_amp(depth, depth), top_block)
							 : 0;
	if (depth >= 3 && !near_bridge)
		place_underwater_vegetation(editor, x, z, water_y, bed_y + bump, depth);
}

void carve_lc_water_pass(
		WorldEditor &editor, const BigWaterField &bwf, const RoadMaskBitmap &road_mask)
{
	const RoadMaskBitmap empty;
	carve_lc_water_pass(editor, bwf, road_mask, empty);
}

void carve_lc_water_pass(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint)
{
	if (bwf.empty())
		return;
	for (int z = bwf.min_z(); z <= bwf.max_z(); ++z) {
		for (int x = bwf.min_x(); x <= bwf.max_x(); ++x) {
			if (road_mask.contains(x, z) || tunnel_footprint.contains(x, z) || !editor.is_lc_water(x, z))
				continue;
			const int water_y = editor.get_water_level(x, z);
			if (editor.get_ground_level(x, z) > water_y)
				continue;
			carve_water_column(editor, x, z, water_y, bwf.depth_at(x, z), road_mask);
		}
	}
}

void carve_lc_water_region(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint,
		int min_x, int max_x, int min_z, int max_z)
{
	if (bwf.empty()) return;
	const int x0 = std::max(min_x, bwf.min_x()), x1 = std::min(max_x, bwf.max_x());
	const int z0 = std::max(min_z, bwf.min_z()), z1 = std::min(max_z, bwf.max_z());
	if (x0 > x1 || z0 > z1) return;
	for (int z = z0; z <= z1; ++z)
		for (int x = x0; x <= x1; ++x) {
			if (road_mask.contains(x, z) || tunnel_footprint.contains(x, z) || !editor.is_lc_water(x, z)) continue;
			const int water_y = editor.get_water_level(x, z);
			if (editor.get_ground_level(x, z) <= water_y)
				carve_water_column(editor, x, z, water_y, bwf.depth_at(x, z), road_mask);
		}
}

}
