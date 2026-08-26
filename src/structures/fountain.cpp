#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures::fountain
{
void place(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	if (!editor.place_schematics())
		return;
	const auto h = land_cover::coord_hash(x, z);
	const auto variant = area_cells >= 300 ? 4 : int((h >> 4) % 3) + 1;
	const int y = editor.get_absolute_y(x, 1, z);
	const unsigned rotation = (h >> 8) & 3;
	// Match Rust's resilient asset pool: a missing/invalid large variant must
	// not suppress a usable small fountain.
	if (place_named_schem(editor, "fountain" + std::to_string(variant), x, y, z,
			rotation))
		return;
	for (int fallback = 1; fallback <= 3; ++fallback)
		if (place_named_schem(editor, "fountain" + std::to_string(fallback), x, y, z,
				rotation))
			return;
}
}
