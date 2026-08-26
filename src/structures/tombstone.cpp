#include "tombstone.h"
#include "schem_decoder.h"
#include "../land_cover/land_cover.h"
#include "../floodfill_cache.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
namespace arnis::structures
{
namespace tombstone
{
namespace
{
constexpr int SMALL_GRID = 4, LARGE_GRID = 32, LARGE_HALF = 6;
std::filesystem::path asset(const std::string &name)
{
	return std::filesystem::path(__FILE__).parent_path().parent_path() /
				("assets/structures/" + name + ".schem");
}
bool near_large_crypt(int x, int z)
{
	const int gx = static_cast<int>(std::lround(static_cast<double>(x) / LARGE_GRID)) * LARGE_GRID;
	const int gz = static_cast<int>(std::lround(static_cast<double>(z) / LARGE_GRID)) * LARGE_GRID;
	return land_cover::coord_hash(gx, gz) % 100 < 5 &&
			std::abs(x - gx) <= LARGE_HALF && std::abs(z - gz) <= LARGE_HALF;
}
}

void maybe_place(WorldEditor &e, int x, int z, const RoadMaskBitmap &road_mask)
{
	if (x % SMALL_GRID != 0 || z % SMALL_GRID != 0 || e.is_lc_water(x, z) ||
			road_mask.contains(x, z))
		return;
	const auto h = land_cover::coord_hash(x, z);
	if (x % LARGE_GRID == 0 && z % LARGE_GRID == 0 && h % 100 < 5) {
		place_schem_file_rotated(e, asset("tombstone" + std::to_string(10 + (h >> 8) % 2)),
				x, e.get_absolute_y(x, 0, z), z, (h >> 16) & 3);
		return;
	}
	if (!near_large_crypt(x, z)) {
		const auto small_hash = land_cover::coord_hash(x + 0x5f5f, z + 0x3c3c);
		if (small_hash % 100 < 28)
			place_schem_file_rotated(e, asset("tombstone" + std::to_string(1 + (small_hash >> 8) % 9)),
					x, e.get_absolute_y(x, 0, z), z, (small_hash >> 16) & 3);
	}
}
}

void scatter_tombstones(WorldEditor &e, int minx, int minz, int maxx, int maxz)
{
	auto grid_start = [](int value) {
		return value - ((value % 4) + 4) % 4;
	};
	for (int z = grid_start(minz); z <= maxz; z += 4)
		for (int x = grid_start(minx); x <= maxx; x += 4) {
			auto h = land_cover::coord_hash(x + 0x5f5f, z + 0x3c3c);
			if (h % 100 >= 28 || e.is_lc_water(x, z))
				continue;
			auto p = std::filesystem::path(__FILE__)
								 .parent_path()
								 .parent_path() /
					 ((h % 9 ? "tombstone1" : "tombstone10") + std::string(".schem"));
			place_schem_file_rotated(
					e, p, x, e.get_absolute_y(x, 0, z), z, (h >> 16) & 3);
		}
}
}
