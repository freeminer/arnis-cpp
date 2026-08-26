#include "helicopter.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void maybe_place_helicopter(WorldEditor &e, int x, int z)
{
	auto h = land_cover::coord_hash(x, z);
	if (h % 100 >= 60 || e.is_lc_water(x, z))
		return;
	auto p = std::filesystem::path(__FILE__).parent_path().parent_path() /
			 "assets/structures/helicopter.schem";
	place_schem_file_rotated(e, p, x, e.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}
