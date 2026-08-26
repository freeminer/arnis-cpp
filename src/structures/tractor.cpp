#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures::tractor
{
void maybe_place_tractor(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 600)
		return;
	const auto h = land_cover::coord_hash(cells.front().first, cells.front().second ^ static_cast<int>(cells.size()));
	if (h % 100 >= 30)
		return;
	const auto [x, z] = cells[h % cells.size()];
	if (!editor.is_lc_water(x, z))
		place_named_schem(editor, "tractor", x, editor.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}
