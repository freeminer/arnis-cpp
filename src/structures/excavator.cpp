#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include <algorithm>

namespace arnis::structures::excavator
{
void scatter_excavators(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 1500)
		return;
	const auto target = std::clamp<std::size_t>(cells.size() / 2000, 1, 6);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = land_cover::coord_hash(static_cast<int>(t) + 1, static_cast<int>(cells.size()));
		const auto [x, z] = cells[h % cells.size()];
		if (editor.is_lc_water(x, z) || std::any_of(placed.begin(), placed.end(), [x, z](const auto &p) {
			return std::abs(p.first-x) < 24 && std::abs(p.second-z) < 24;
		})) continue;
		place_named_schem(editor, "excavator", x, editor.get_absolute_y(x, 1, z), z, (h >> 5) & 3);
		placed.emplace_back(x, z);
	}
}
}
