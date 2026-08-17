#include "tombstone.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void scatter_tombstones(WorldEditor &e, int minx, int minz, int maxx, int maxz)
{
	for (int z = minz; z <= maxz; z += 4)
		for (int x = minx; x <= maxx; x += 4) {
			auto h = land_cover::coord_hash(x + 0x5f5f, z + 0x3c3c);
			if (h % 100 >= 28 || e.is_lc_water(x, z))
				continue;
			auto p = std::filesystem::path(__FILE__)
							 .parent_path()
							 .parent_path()
							 .parent_path() /
					 ((h % 9 ? "tombstone1" : "tombstone10") + std::string(".schem"));
			place_schem_file_rotated(
					e, p, x, e.get_absolute_y(x, 0, z), z, (h >> 16) & 3);
		}
}
}
