#include "tile.h"
#include <algorithm>
namespace arnis::tiles
{
std::vector<TileBounds> create_tiles(int minx, int minz, int maxx, int maxz, int s)
{
	std::vector<TileBounds> o;
	if (s <= 0 || maxx < minx || maxz < minz)
		return o;
	const long long ax = static_cast<long long>(region_floor(minx)) * DEFAULT_TILE_SIZE;
	const long long az = static_cast<long long>(region_floor(minz)) * DEFAULT_TILE_SIZE;
	const long long bx =
			(static_cast<long long>(region_floor(maxx)) + 1) * DEFAULT_TILE_SIZE;
	const long long bz =
			(static_cast<long long>(region_floor(maxz)) + 1) * DEFAULT_TILE_SIZE;
	for (long long z = az; z < bz; z += s)
		for (long long x = ax; x < bx; x += s) {
			const long long ex = std::min(x + s, bx), ez = std::min(z + s, bz);
			if (ex > minx && x <= maxx && ez > minz && z <= maxz)
				o.push_back({static_cast<int>(x), static_cast<int>(z),
						static_cast<int>(ex), static_cast<int>(ez)});
		}
	return o;
}
}
