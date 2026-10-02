#include "caves_water.h"
#include "block_definitions.h"
#include "caves_rng.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <unordered_set>
#include <vector>

namespace arnis::caves
{
namespace
{
const std::vector<Block> HOSTS{STONE, DEEPSLATE, DIRT, COARSE_DIRT, GRAVEL, TUFF};

int chunk_floor(int v)
{
	return v >= 0 ? v / 16 : -(((-v) + 15) / 16);
}

bool host(const world_editor::WorldEditor &e, int x, int y, int z)
{
	return e.check_for_block_absolute(x, y, z, HOSTS);
}

void place(world_editor::WorldEditor &e, int x, int y, int z, bool water,
		std::unordered_set<std::int64_t> *water_cells = nullptr)
{
	e.set_block_absolute(water ? WATER : AIR, x, y, z,
			std::optional<std::vector<Block>>(water ? std::vector<Block>{AIR} : HOSTS),
			std::nullopt);
	if (water && water_cells)
		water_cells->insert(pack_cave_pos(x, y, z));
}

void generate_deep_lava_sea(world_editor::WorldEditor &editor, const CaveRect &region,
		int floor_y, const std::unordered_set<std::int64_t> &water_cells)
{
	const int lava_level = floor_y + 10;
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_y = std::max(floor_y + 2, write_min_y);
	const int max_y = std::min(lava_level, write_max_y);
	if (min_y > max_y)
		return;
	std::unordered_set<std::int64_t> cave_air;
	std::size_t columns = static_cast<std::size_t>(region.max_x - region.min_x + 1) *
						  static_cast<std::size_t>(region.max_z - region.min_z + 1);
	cave_air.reserve(std::min<std::size_t>(columns * 4, 1'000'000));
	for (int x = region.min_x; x <= region.max_x; ++x)
		for (int z = region.min_z; z <= region.max_z; ++z) {
			const int top =
					std::min({editor.get_ground_level(x, z) - 8, max_y, write_max_y});
			for (int y = min_y; y <= top; ++y)
				if (!editor.block_exists_absolute(x, y, z))
					cave_air.insert(pack_cave_pos(x, y, z));
		}
	if (cave_air.empty())
		return;
	constexpr std::array<std::array<int, 3>, 6> neighbours = {
			{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
	auto is_open = [&](int x, int y, int z) {
		if (!region.contains(x, z))
			return false;
		const auto p = pack_cave_pos(x, y, z);
		return !editor.block_exists_absolute(x, y, z) && !cave_air.contains(p);
	};
	std::vector<std::int64_t> candidates;
	candidates.reserve(cave_air.size());
	for (const auto p : cave_air) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (y >= lava_level)
			continue;
		const bool exposed = std::any_of(
				neighbours.begin(), neighbours.end(), [&](const auto &offset) {
					return is_open(x + offset[0], y + offset[1], z + offset[2]);
				});
		if (!exposed)
			candidates.push_back(p);
	}
	std::sort(candidates.begin(), candidates.end(), [](std::int64_t a, std::int64_t b) {
		return std::get<1>(unpack_cave_pos(a)) < std::get<1>(unpack_cave_pos(b));
	});
	std::unordered_set<std::int64_t> supported;
	supported.reserve(candidates.size());
	for (const auto p : candidates) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (editor.block_exists_absolute(x, y - 1, z) ||
				supported.contains(pack_cave_pos(x, y - 1, z)))
			supported.insert(p);
	}
	const int stone_transition = floor_y + 64;
	const std::vector<Block> rim_rock{
			STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE, DIORITE, ANDESITE};
	for (const auto p : supported) {
		const auto [x, y, z] = unpack_cave_pos(p);
		const bool touches_water = std::any_of(
				neighbours.begin(), neighbours.end(), [&](const auto &offset) {
					return water_cells.contains(
							pack_cave_pos(x + offset[0], y + offset[1], z + offset[2]));
				});
		if (touches_water) {
			const Block seam = y < stone_transition ? DEEPSLATE : STONE;
			editor.set_block_absolute(
					seam, x, y, z, std::vector<Block>{AIR}, std::nullopt);
			continue;
		}
		editor.set_block_absolute(LAVA, x, y, z, std::vector<Block>{AIR}, std::nullopt);
	}
	for (const auto p : supported) {
		const auto [x, y, z] = unpack_cave_pos(p);
		for (const auto &offset : neighbours) {
			const int nx = x + offset[0], ny = y + offset[1], nz = z + offset[2];
			const auto neighbour = pack_cave_pos(nx, ny, nz);
			if (!region.contains(nx, nz) || supported.contains(neighbour) ||
					cave_air.contains(neighbour))
				continue;
			const auto hash =
					static_cast<std::uint64_t>(neighbour) * 0x9E3779B97F4A7C15ULL;
			const auto roll = (hash >> 40) % 100;
			if (roll < 45)
				editor.set_block_absolute(OBSIDIAN, nx, ny, nz, rim_rock, std::nullopt);
			else if (roll < 62)
				editor.set_block_absolute(
						MAGMA_BLOCK, nx, ny, nz, rim_rock, std::nullopt);
		}
	}
}
}

void generate_water_features(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y)
{
	std::unordered_set<std::int64_t> water_cells;
	// Rust's FEATURE_REACH is 64 blocks: an origin up to four chunks away can
	// still touch the requested region.  A one-chunk margin silently dropped
	// pools/rivers at tile edges and made adjacent generation windows disagree.
	constexpr int feature_reach_chunks = 4;
	const int cx0 = chunk_floor(region.min_x) - feature_reach_chunks;
	const int cx1 = chunk_floor(region.max_x) + feature_reach_chunks;
	const int cz0 = chunk_floor(region.min_z) - feature_reach_chunks;
	const int cz1 = chunk_floor(region.max_z) + feature_reach_chunks;
	for (int cx = cx0; cx <= cx1; ++cx)
		for (int cz = cz0; cz <= cz1; ++cz) {
			XoroRandom pool_rng = XoroRandom::from_seed(
					seed ^ (std::int64_t(cx) * 341873128712LL) ^
					(std::int64_t(cz) * 132897987541LL) ^ 0xA044A044LL);
			if (pool_rng.next_int(30) == 0) {
				const int x0 = cx * 16 + pool_rng.next_int(16);
				const int z0 = cz * 16 + pool_rng.next_int(16);
				const int top = editor.get_ground_level(x0, z0);
				const int y0 = std::max(floor_y + 18, std::min(top - 18, floor_y + 70));
				const int rx = 5 + pool_rng.next_int(7), rz = 5 + pool_rng.next_int(7);
				const int rv = 3 + pool_rng.next_int(2);
				for (int x = x0 - rx - 1; x <= x0 + rx + 1; ++x)
					for (int z = z0 - rz - 1; z <= z0 + rz + 1; ++z)
						for (int y = y0 - rv; y <= y0 + rv; ++y) {
							if (!region.contains(x, z))
								continue;
							const double dx = double(x - x0) / rx,
										 dz = double(z - z0) / rz,
										 dy = double(y - y0) / rv;
							if (dx * dx + dz * dz + dy * dy > 1.0 ||
									!host(editor, x, y, z))
								continue;
							// Like Rust pools, the lower half is flooded and the roof remains air.
							place(editor, x, y, z, y <= y0, &water_cells);
						}
			}

			XoroRandom river_rng = XoroRandom::from_seed(
					seed ^ (std::int64_t(cx) * 341873128712LL) ^
					(std::int64_t(cz) * 132897987541LL) ^ 0xA045A045LL);
			if (river_rng.next_int(18) != 0)
				continue;
			int x = cx * 16 + river_rng.next_int(16),
				z = cz * 16 + river_rng.next_int(16);
			int y = std::max(floor_y + 20,
					std::min(editor.get_ground_level(x, z) - 20, floor_y + 74));
			const double angle = river_rng.next_float() * 2.0 * M_PI;
			for (int step = 0; step < 26 + river_rng.next_int(27); ++step) {
				if (region.contains(x, z))
					for (int dy = -1; dy <= 1; ++dy)
						if (host(editor, x, y + dy, z))
							place(editor, x, y + dy, z, dy <= 0, &water_cells);
				x += int(std::llround(
						std::cos(angle) + 0.8 * std::sin(step * .37 + double(cx))));
				z += int(std::llround(
						std::sin(angle) + 0.8 * std::cos(step * .31 + double(cz))));
				y -= step % 3 == 0 ? 1 : 0;
				if (y <= floor_y + 8)
					break;
			}
		}
	generate_deep_lava_sea(editor, region, floor_y, water_cells);
}

bool WaterPlan::solid(const CaveRect &world, int floor, int x, int y, int z,
		const std::function<bool(int, int, int)> &is_cave) const
{
	const auto p = pack_cave_pos(x, y, z);
	if (water.contains(p))
		return true;
	return world.contains(x, z) && y >= floor && !is_cave(x, y, z) && !carved.contains(p);
}
void WaterPlan::apply(world_editor::WorldEditor &editor, const CaveRect &region,
		const std::vector<Block> &cave_host) const
{
	for (const auto &p : carved) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (region.contains(x, z))
			editor.set_block_absolute(AIR, x, y, z,
					std::optional<std::vector<Block>>(cave_host), std::nullopt);
	}
	for (const auto &p : water) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (region.contains(x, z)) {
			editor.set_block_absolute(WATER, x, y, z,
					std::optional<std::vector<Block>>({AIR}), std::nullopt);
			// Freeminer's backend updates fluid propagation from the placed source;
			// it has no explicit schedule_fluid_tick API.
		}
	}
}
}
