#include "parking_garage.h"

#include "connected_blocks.h"
#include "../land_cover/land_cover.h"
#include "../structures/structures.h"
#include "block_definitions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_set>

namespace arnis::parking_garage
{
namespace
{
constexpr int LEVEL = 4, BAY_WIDTH = 6, BAY_DEPTH = 9, AISLE = 6;
constexpr int MODULE = 2 * BAY_DEPTH + AISLE, COLUMN_SPACING = 3 * BAY_WIDTH;
constexpr int FACADE_COLUMN_SPACING = 6, RAMP_WIDTH = 4, RAMP_RUN = 2 * LEVEL;
constexpr int SIDES[4][2] = {{0, -1}, {0, 1}, {1, 0}, {-1, 0}};
std::uint64_t key(int x, int z)
{
	return (std::uint64_t(static_cast<std::uint32_t>(x)) << 32) |
		   static_cast<std::uint32_t>(z);
}

struct Frame
{
	int min_x, min_z;
	bool along_x;
	int len_a, len_c;
	std::pair<int, int> ac(int x, int z) const
	{
		return along_x ? std::pair{x - min_x, z - min_z}
					   : std::pair{z - min_z, x - min_x};
	}
	std::pair<int, int> xz(int a, int c) const
	{
		return along_x ? std::pair{min_x + a, min_z + c}
					   : std::pair{min_x + c, min_z + a};
	}
};
struct Ramp
{
	int a0, c0;
};
struct Core
{
	int x0, z0, dx, dz;
};
struct Plan
{
	std::unordered_set<std::uint64_t> area, edge;
	Frame frame;
	std::optional<Ramp> ramp;
	std::optional<Core> core;
	bool has(const std::unordered_set<std::uint64_t> &s, int x, int z) const
	{
		return s.contains(key(x, z));
	}
	bool interior(int x, int z) const { return has(area, x, z) && !has(edge, x, z); }
	std::optional<int> ramp_step(int x, int z) const
	{
		if (!ramp)
			return std::nullopt;
		auto [a, c] = frame.ac(x, z);
		const int i = a - ramp->a0;
		if (i >= 0 && i <= RAMP_RUN + 1 && c >= ramp->c0 && c < ramp->c0 + RAMP_WIDTH)
			return i;
		return std::nullopt;
	}
	bool in_core(int x, int z) const
	{
		return core && x >= core->x0 && x < core->x0 + 3 && z >= core->z0 &&
			   z < core->z0 + 3;
	}
	bool open_deck(int x, int z) const
	{
		return interior(x, z) && !ramp_step(x, z) && !in_core(x, z);
	}
	bool column(int x, int z) const
	{
		for (auto &d : SIDES)
			if (!interior(x + d[0], z + d[1]))
				return false;
		auto [a, c] = frame.ac(x, z);
		const int m = ((c - 1) % MODULE + MODULE) % MODULE;
		return (m == 0 || m == BAY_DEPTH || m == BAY_DEPTH + AISLE - 1) &&
			   ((a - 1) % COLUMN_SPACING + COLUMN_SPACING) % COLUMN_SPACING == 0;
	}
	bool light(int x, int z) const
	{
		auto [a, c] = frame.ac(x, z);
		const int m = ((c - 1) % MODULE + MODULE) % MODULE;
		return m == BAY_DEPTH + AISLE / 2 &&
			   ((a - 1) % COLUMN_SPACING + COLUMN_SPACING) % COLUMN_SPACING ==
					   COLUMN_SPACING / 2;
	}
	bool stripe(int x, int z) const
	{
		auto [a, c] = frame.ac(x, z);
		const int m = ((c - 1) % MODULE + MODULE) % MODULE;
		return !(m >= BAY_DEPTH && m < BAY_DEPTH + AISLE) &&
			   ((a - 1) % BAY_WIDTH + BAY_WIDTH) % BAY_WIDTH == 0;
	}
	bool facade_column(int x, int z) const
	{
		auto outside = [&](int dx, int dz) { return !has(area, x + dx, z + dz); };
		const bool xs = outside(0, -1) || outside(0, 1);
		const bool zs = outside(-1, 0) || outside(1, 0);
		return (xs && (zs || (x - frame.min_x) % FACADE_COLUMN_SPACING == 0)) ||
			   (zs && (z - frame.min_z) % FACADE_COLUMN_SPACING == 0);
	}
};

std::optional<Ramp> find_ramp(const Plan &p)
{
	auto fits = [&](Ramp r) {
		for (int i = 0; i <= RAMP_RUN + 1; ++i)
			for (int c = 0; c < RAMP_WIDTH; ++c) {
				auto [x, z] = p.frame.xz(r.a0 + i, r.c0 + c);
				if (!p.has(p.area, x, z) || p.has(p.edge, x, z))
					return false;
			}
		return true;
	};
	for (int c : {1, p.frame.len_c - RAMP_WIDTH})
		for (int a = 1; a < p.frame.len_a - RAMP_RUN - 1; ++a) {
			Ramp r{a, c};
			if (fits(r))
				return r;
		}
	return std::nullopt;
}

std::optional<Core> find_core(const Plan &p)
{
	const auto &f = p.frame;
	const int max_x = f.min_x + (f.along_x ? f.len_a : f.len_c);
	const int max_z = f.min_z + (f.along_x ? f.len_c : f.len_a);
	const int mid_x = (f.min_x + max_x) / 2, mid_z = (f.min_z + max_z) / 2;
	const int corners[4][4] = {{f.min_x, f.min_z, 1, 1}, {max_x - 2, f.min_z, -1, 1},
			{f.min_x, max_z - 2, 1, -1}, {max_x - 2, max_z - 2, -1, -1}};
	for (int t = 0; t < 8; ++t)
		for (auto &c : corners) {
			const int x0 = c[0] + c[2] * t, z0 = c[1] + c[3] * t;
			bool ok = true;
			for (int dx = 0; dx < 3; ++dx)
				for (int dz = 0; dz < 3; ++dz)
					ok &= p.has(p.area, x0 + dx, z0 + dz) &&
						  !p.ramp_step(x0 + dx, z0 + dz);
			if (!ok)
				continue;
			const int dx = mid_x - (x0 + 1), dz = mid_z - (z0 + 1);
			const auto door = std::abs(dx) >= std::abs(dz)
									  ? std::pair{(dx > 0) - (dx < 0), 0}
									  : std::pair{0, (dz > 0) - (dz < 0)};
			if (!door.first && !door.second)
				continue;
			Plan test = p;
			test.core = Core{x0, z0, door.first, door.second};
			if (test.open_deck(x0 + 1 + 2 * door.first, z0 + 1 + 2 * door.second))
				return test.core;
		}
	return std::nullopt;
}

enum class Facade
{
	Concrete,
	Mesh,
	Green,
	Brick
};

void build_core(WorldEditor &e, const Core &c, int base, int top)
{
	const int cx = c.x0 + 1, cz = c.z0 + 1;
	const char *facing = c.dx > 0	? "east"
						 : c.dx < 0 ? "west"
						 : c.dz > 0 ? "south"
									: "north";
	for (int dx = 0; dx < 3; ++dx)
		for (int dz = 0; dz < 3; ++dz) {
			const int x = c.x0 + dx, z = c.z0 + dz;
			e.set_block_absolute(GRAY_CONCRETE, x, base, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
			e.set_block_absolute(LIGHT_GRAY_CONCRETE, x, top + 4, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
			if (x == cx && z == cz) {
				for (int y = base + 1; y <= top + 3; ++y)
					e.set_block_with_properties_absolute(
							{LADDER, {{"facing", facing}, {"waterlogged", "false"}}}, x,
							y, z, std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{}));
			} else {
				for (int y = base + 1; y <= top + 3; ++y) {
					const int off = (y - base) % LEVEL;
					const bool doorway =
							x == cx + c.dx && z == cz + c.dz && (off == 1 || off == 2);
					if (!doorway)
						e.set_block_absolute(LIGHT_GRAY_CONCRETE, x, y, z, std::nullopt,
								std::optional<std::vector<Block>>(std::vector<Block>{}));
				}
			}
		}
}
} // namespace

void generate(WorldEditor &e, const std::vector<std::pair<int, int>> &floor_area,
		int building_height, int base_y, std::uint64_t seed)
{
	if (floor_area.empty())
		return;
	Plan p{};
	int min_x = INT32_MAX, min_z = INT32_MAX, max_x = INT32_MIN, max_z = INT32_MIN;
	for (auto [x, z] : floor_area) {
		p.area.insert(key(x, z));
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	for (auto [x, z] : floor_area)
		for (auto &d : SIDES)
			if (!p.has(p.area, x + d[0], z + d[1])) {
				p.edge.insert(key(x, z));
				break;
			}
	const bool along_x = max_x - min_x >= max_z - min_z;
	p.frame = {min_x, min_z, along_x, along_x ? max_x - min_x : max_z - min_z,
			along_x ? max_z - min_z : max_x - min_x};
	p.ramp = find_ramp(p);
	p.core = find_core(p);
	const int decks = std::max(1, (building_height - 2) / LEVEL);
	const int top = base_y + decks * LEVEL;
	const auto facade = seed % 20 < 8	 ? Facade::Concrete
						: seed % 20 < 13 ? Facade::Mesh
						: seed % 20 < 17 ? Facade::Green
										 : Facade::Brick;
	const Block band = facade == Facade::Brick ? BRICK : LIGHT_GRAY_CONCRETE;
	for (auto [x, z] : floor_area) {
		const int ground = e.get_ground_level(x, z);
		const Block fill = p.has(p.edge, x, z) ? LIGHT_GRAY_CONCRETE : STONE;
		for (int y = ground; y < base_y; ++y)
			e.set_block_absolute(fill, x, y, z);
	}
	if (p.core)
		build_core(e, *p.core, base_y, top);
	for (int k = 0; k <= decks; ++k) {
		const int y = base_y + k * LEVEL;
		for (auto [x, z] : floor_area) {
			if (p.in_core(x, z))
				continue;
			const auto ri = p.ramp_step(x, z);
			if (k && ri && *ri > 1 && *ri < RAMP_RUN)
				continue;
			const Block b = p.has(p.edge, x, z)					  ? LIGHT_GRAY_CONCRETE
							: p.open_deck(x, z) && p.stripe(x, z) ? WHITE_CONCRETE
																  : GRAY_CONCRETE;
			e.set_block_absolute(b, x, y, z, std::nullopt,
					std::optional<std::vector<Block>>(std::vector<Block>{}));
		}
	}
	if (p.ramp)
		for (int k = 0; k < decks; ++k)
			for (int i = 1; i <= RAMP_RUN; ++i)
				for (int r = 0; r < RAMP_WIDTH; ++r) {
					auto [x, z] = p.frame.xz(p.ramp->a0 + i, p.ramp->c0 + r);
					if (i % 2 == 0)
						e.set_block_absolute(
								SMOOTH_STONE, x, base_y + k * LEVEL + i / 2, z);
					else
						e.set_block_absolute(SMOOTH_STONE_SLAB, x,
								base_y + k * LEVEL + (i + 1) / 2, z);
				}
	for (int k = 0; k < decks; ++k) {
		const int y = base_y + k * LEVEL;
		const BlockWithProperties lantern{
				LANTERN, {{"hanging", "true"}, {"waterlogged", "false"}}};
		for (auto [x, z] : floor_area) {
			if (p.in_core(x, z))
				continue;
			if (p.has(p.edge, x, z)) {
				if (p.facade_column(x, z))
					for (int dy = 1; dy < LEVEL; ++dy)
						e.set_block_absolute(
								facade == Facade::Brick ? BRICK : LIGHT_GRAY_CONCRETE, x,
								y + dy, z);
				else if (k) {
					e.set_block_absolute(band, x, y + 1, z);
					if (facade == Facade::Mesh)
						for (int dy = 2; dy < LEVEL; ++dy)
							connected_blocks::place_connected(e, IRON_BARS, x, y + dy, z);
					if (facade == Facade::Green) {
						const auto h = land_cover::coord_hash(x ^ int(seed), z ^ y);
						const Block leaf = h % 3 == 0 ? AZALEA_LEAVES : OAK_LEAVES;
						if (h % 100 < 60) {
							e.set_block_absolute(leaf, x, y + 2, z);
							if ((h >> 8) % 100 < 40)
								e.set_block_absolute(leaf, x, y + 3, z);
						}
					}
				}
			} else if (p.open_deck(x, z) && p.column(x, z))
				for (int dy = 1; dy < LEVEL; ++dy)
					e.set_block_absolute(LIGHT_GRAY_CONCRETE, x, y + dy, z);
			else if (p.open_deck(x, z) && p.light(x, z))
				e.set_block_with_properties_absolute(
						lantern, x, y + LEVEL - 1, z, std::nullopt, std::nullopt);
		}
	}
	for (auto [x, z] : floor_area) {
		if (p.in_core(x, z))
			continue;
		if (p.has(p.edge, x, z))
			e.set_block_absolute(band, x, top + 1, z);
		else if (p.open_deck(x, z) && p.light(x, z)) {
			e.set_block_absolute(ANDESITE_WALL, x, top + 1, z);
			for (int dy = 2; dy <= 3; ++dy)
				e.set_block_absolute(IRON_BARS, x, top + dy, z);
			e.set_block_absolute(SEA_LANTERN, x, top + 4, z);
			e.set_block_absolute(SMOOTH_STONE_SLAB, x, top + 5, z);
		}
	}
	// Place vehicles in the repeated Rust-layout bays on every deck.
	const int rot_base = p.frame.along_x ? 0 : 1;
	std::vector<std::pair<int, int>> bays;
	for (int m = 0; m <= p.frame.len_c / MODULE; ++m) {
		for (const int c0 : {1 + m * MODULE, 1 + m * MODULE + BAY_DEPTH + AISLE}) {
			for (int a0 = 1; a0 + BAY_WIDTH <= p.frame.len_a; a0 += BAY_WIDTH) {
				bool clear = true;
				for (int da = 1; da < BAY_WIDTH && clear; ++da) {
					for (int dc = 0; dc < BAY_DEPTH && clear; ++dc) {
						auto [x, z] = p.frame.xz(a0 + da, c0 + dc);
						clear = p.open_deck(x, z);
					}
				}
				if (clear)
					bays.push_back(p.frame.xz(a0 + BAY_WIDTH / 2, c0 + BAY_DEPTH / 2));
			}
		}
	}
	for (int k = 0; k <= decks; ++k) {
		const int headroom = k == decks ? std::numeric_limits<int>::max() : LEVEL - 1;
		const int deck_top = base_y + k * LEVEL + 1;
		for (const auto &[x, z] : bays)
			structures::car::maybe_place_car_on_deck(
					e, x, z, deck_top, static_cast<std::uint8_t>(rot_base), headroom);
	}
}
} // namespace arnis::parking_garage
