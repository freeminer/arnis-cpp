#include "structures.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "../block_definitions.h"
#include "../land_cover.h"

namespace arnis::structures
{
namespace
{

using arnis::land_cover::coord_hash;

std::pair<int, int> nearest_cell_to_centroid(
		const std::vector<std::pair<int, int>> &cells)
{
	if (cells.empty())
		return {0, 0};

	long long sx = 0;
	long long sz = 0;
	for (const auto &[x, z] : cells) {
		sx += x;
		sz += z;
	}
	const int cx = static_cast<int>(sx / static_cast<long long>(cells.size()));
	const int cz = static_cast<int>(sz / static_cast<long long>(cells.size()));

	return *std::min_element(cells.begin(), cells.end(),
			[cx, cz](const auto &a, const auto &b) {
				const long long adx = static_cast<long long>(a.first - cx);
				const long long adz = static_cast<long long>(a.second - cz);
				const long long bdx = static_cast<long long>(b.first - cx);
				const long long bdz = static_cast<long long>(b.second - cz);
				return adx * adx + adz * adz < bdx * bdx + bdz * bdz;
			});
}

std::pair<int, int> rotated(int dx, int dz, uint8_t rot)
{
	switch (rot & 3) {
	case 1:
		return {-dz, dx};
	case 2:
		return {-dx, -dz};
	case 3:
		return {dz, -dx};
	default:
		return {dx, dz};
	}
}

void block(WorldEditor &editor, Block b, int x, int y, int z)
{
	editor.set_block_absolute(b, x, y, z);
}

void rel(WorldEditor &editor, Block b, int x, int z, int base_y, int dx, int dy,
		int dz, uint8_t rot)
{
	const auto [rx, rz] = rotated(dx, dz, rot);
	block(editor, b, x + rx, base_y + dy, z + rz);
}

void pad(WorldEditor &editor, Block b, int x, int z, int base_y, int radius)
{
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dz = -radius; dz <= radius; ++dz)
			block(editor, b, x + dx, base_y - 1, z + dz);
	}
}

void simple_fountain(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	const bool large = area_cells >= 300;
	const int base_y = editor.get_absolute_y(x, 1, z);
	const int radius = large ? 4 : 2;
	pad(editor, STONE_BRICKS, x, z, base_y, radius);
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dz = -radius; dz <= radius; ++dz) {
			const bool edge = std::abs(dx) == radius || std::abs(dz) == radius;
			if (edge)
				block(editor, POLISHED_ANDESITE, x + dx, base_y, z + dz);
			else
				block(editor, WATER, x + dx, base_y, z + dz);
		}
	}
	for (int y = 1; y <= (large ? 4 : 2); ++y)
		block(editor, WATER, x, base_y + y, z);
	block(editor, SEA_LANTERN, x, base_y, z);
}

void simple_playground(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	pad(editor, SAND, x, z, base_y, 5);

	for (int dz : {-2, 2}) {
		for (int y = 0; y <= 3; ++y) {
			rel(editor, OAK_FENCE, x, z, base_y, -2, y, dz, rot);
			rel(editor, OAK_FENCE, x, z, base_y, 2, y, dz, rot);
		}
	}
	for (int dx = -2; dx <= 2; ++dx) {
		rel(editor, OAK_PLANKS, x, z, base_y, dx, 4, -2, rot);
		rel(editor, OAK_PLANKS, x, z, base_y, dx, 4, 2, rot);
	}
	rel(editor, CHAIN, x, z, base_y, -1, 3, 0, rot);
	rel(editor, CHAIN, x, z, base_y, 1, 3, 0, rot);
	rel(editor, OAK_SLAB, x, z, base_y, 0, 1, 0, rot);

	for (int step = 0; step < 4; ++step)
		rel(editor, STONE_BLOCK_SLAB, x, z, base_y, 4 + step, step, -1, rot);
	for (int y = 0; y <= 3; ++y)
		rel(editor, LADDER, x, z, base_y, 4, y, -2, rot);
}

void simple_lighthouse(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	pad(editor, STONE_BRICKS, x, z, base_y, 2);
	for (int y = 0; y < 12; ++y) {
		const int radius = y < 8 ? 2 : 1;
		for (int dx = -radius; dx <= radius; ++dx) {
			for (int dz = -radius; dz <= radius; ++dz) {
				if (std::abs(dx) == radius || std::abs(dz) == radius)
					rel(editor, (y / 2) % 2 == 0 ? WHITE_CONCRETE : RED_CONCRETE,
							x, z, base_y, dx, y, dz, rot);
			}
		}
	}
	for (int dx = -2; dx <= 2; ++dx) {
		for (int dz = -2; dz <= 2; ++dz)
			rel(editor, GLASS, x, z, base_y, dx, 12, dz, rot);
	}
	rel(editor, SEA_LANTERN, x, z, base_y, 0, 13, 0, rot);
	rel(editor, LIGHTNING_ROD, x, z, base_y, 0, 14, 0, rot);
}

void simple_crane(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int y = 0; y < 18; ++y)
		rel(editor, SCAFFOLDING, x, z, base_y, 0, y, 0, rot);
	for (int dx = -10; dx <= 14; ++dx)
		rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 17, 0, rot);
	for (int y = 10; y <= 16; ++y)
		rel(editor, CHAIN, x, z, base_y, 10, y, 0, rot);
	rel(editor, IRON_BLOCK, x, z, base_y, 10, 9, 0, rot);
}

void simple_excavator(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int dx = -2; dx <= 2; ++dx) {
		rel(editor, BLACK_CONCRETE, x, z, base_y, dx, 0, -1, rot);
		rel(editor, BLACK_CONCRETE, x, z, base_y, dx, 0, 1, rot);
	}
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz)
			rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 1, dz, rot);
	}
	for (int dx = 2; dx <= 5; ++dx)
		rel(editor, YELLOW_CONCRETE, x, z, base_y, dx, 2, 0, rot);
	rel(editor, IRON_BLOCK, x, z, base_y, 6, 1, 0, rot);
}

void simple_tractor(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_absolute_y(x, 1, z);
	for (int dz = -1; dz <= 1; ++dz) {
		rel(editor, RED_CONCRETE, x, z, base_y, 0, 1, dz, rot);
		rel(editor, RED_CONCRETE, x, z, base_y, 1, 1, dz, rot);
	}
	rel(editor, GLASS, x, z, base_y, -1, 2, 0, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, -1, 0, -1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, -1, 0, 1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, 2, 0, -1, rot);
	rel(editor, BLACK_CONCRETE, x, z, base_y, 2, 0, 1, rot);
}

void simple_boat(WorldEditor &editor, int x, int z, uint8_t rot)
{
	const int base_y = editor.get_water_level(x, z);
	for (int dx = -4; dx <= 4; ++dx) {
		const int half = 2 - (std::abs(dx) / 3);
		for (int dz = -half; dz <= half; ++dz)
			rel(editor, OAK_PLANKS, x, z, base_y, dx, 0, dz, rot);
	}
	for (int dx = -2; dx <= 2; ++dx)
		rel(editor, WHITE_WOOL, x, z, base_y, dx, 3, 0, rot);
	for (int y = 1; y <= 4; ++y)
		rel(editor, OAK_FENCE, x, z, base_y, 0, y, 0, rot);
}

}

namespace fountain
{
void place(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	simple_fountain(editor, x, z, area_cells);
}
}

namespace playground
{
void scatter_playgrounds(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	const std::size_t n = cells.size();
	if (n < 120)
		return;
	const std::size_t target = std::clamp<std::size_t>(n / 500, 1, 4);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = coord_hash(static_cast<int>(t) + 1, static_cast<int>(n));
		const auto [ax, az] = cells[h % n];
		if (editor.is_lc_water(ax, az))
			continue;
		const bool too_close = std::any_of(placed.begin(), placed.end(),
				[ax, az](const auto &p) {
					return std::abs(p.first - ax) < 16 && std::abs(p.second - az) < 16;
				});
		if (too_close)
			continue;
		simple_playground(editor, ax, az, static_cast<uint8_t>((h >> 7) & 3));
		placed.emplace_back(ax, az);
	}
}
}

namespace lighthouse
{
void place(WorldEditor &editor, int x, int z)
{
	const auto h = coord_hash(x, z);
	simple_lighthouse(editor, x, z, static_cast<uint8_t>(h & 3));
}
}

namespace crane
{
void maybe_place_crane(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (cells.size() < 1500)
		return;
	const auto [ax, az] = nearest_cell_to_centroid(cells);
	if (editor.is_lc_water(ax, az))
		return;
	const auto h = coord_hash(ax, az);
	if (h % 100 >= 60)
		return;
	simple_crane(editor, ax, az, static_cast<uint8_t>((h >> 8) & 3));
}
}

namespace excavator
{
void scatter_excavators(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	const std::size_t n = cells.size();
	if (n < 1500)
		return;
	const std::size_t target = std::clamp<std::size_t>(n / 2000, 1, 6);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = coord_hash(static_cast<int>(t) + 1, static_cast<int>(n));
		const auto [ax, az] = cells[h % n];
		if (editor.is_lc_water(ax, az))
			continue;
		const bool too_close = std::any_of(placed.begin(), placed.end(),
				[ax, az](const auto &p) {
					return std::abs(p.first - ax) < 24 && std::abs(p.second - az) < 24;
				});
		if (too_close)
			continue;
		simple_excavator(editor, ax, az, static_cast<uint8_t>((h >> 5) & 3));
		placed.emplace_back(ax, az);
	}
}
}

namespace tractor
{
void maybe_place_tractor(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	const std::size_t n = cells.size();
	if (n < 600)
		return;
	const auto h = coord_hash(cells.front().first, cells.front().second ^ static_cast<int>(n));
	if (h % 100 >= 30)
		return;
	const auto [ax, az] = cells[h % n];
	if (editor.is_lc_water(ax, az))
		return;
	simple_tractor(editor, ax, az, static_cast<uint8_t>((h >> 8) & 3));
}
}

namespace boat
{
void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	constexpr int spacing = 400;
	constexpr int max_boats = 200;
	int count = 0;
	for (int gz = min_z - ((min_z % spacing) + spacing) % spacing; gz <= max_z;
			gz += spacing) {
		for (int gx = min_x - ((min_x % spacing) + spacing) % spacing; gx <= max_x;
				gx += spacing) {
			if (count >= max_boats)
				return;
			const auto h = coord_hash(gx, gz);
			if (h % 100 >= 45)
				continue;
			const int ax = gx + static_cast<int>(h % 7);
			const int az = gz + static_cast<int>((h >> 3) % 7);
			if (editor.is_lc_water(ax, az) && editor.water_distance(ax, az) == 0) {
				simple_boat(editor, ax, az, static_cast<uint8_t>((h >> 5) & 3));
				++count;
			}
		}
	}
}
}

}
