#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures::lighthouse
{
void place(WorldEditor &editor, int x, int z)
{
	if (!editor.place_schematics())
		return;
	const auto h = land_cover::coord_hash(x, z);
	place_named_schem(editor, "lighthouse", x, editor.get_absolute_y(x, 1, z), z, h & 3);
}
}
