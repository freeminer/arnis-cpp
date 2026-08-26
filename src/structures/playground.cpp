#include "structures.h"
#include "../land_cover/land_cover.h"
#include "../block_definitions.h"
#include "schem_decoder.h"
#include <algorithm>

namespace arnis::structures::playground
{
void scatter_playgrounds(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 120)
		return;
	const auto target = std::clamp<std::size_t>(cells.size() / 500, 1, 4);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = land_cover::coord_hash(static_cast<int>(t) + 1, static_cast<int>(cells.size()));
		const auto [x, z] = cells[h % cells.size()];
		if (editor.is_lc_water(x, z) || std::any_of(placed.begin(), placed.end(), [x, z](const auto &p) {
			return std::abs(p.first-x) < 16 && std::abs(p.second-z) < 16;
		})) continue;
		const int y = editor.get_absolute_y(x, 1, z);
		const unsigned rotation = (h >> 7) & 3;
		const int selected = static_cast<int>((h >> 5) % 3) + 1;
		bool placed_schematic = place_named_schem(editor,
				"playground" + std::to_string(selected), x, y, z, rotation, &SAND);
		// Rust skips a failed variant and keeps trying the remaining parsed
		// playgrounds, so one bad asset does not remove the whole feature.
		for (int fallback = 1; !placed_schematic && fallback <= 3; ++fallback)
			if (fallback != selected)
				placed_schematic = place_named_schem(editor,
						"playground" + std::to_string(fallback), x, y, z, rotation, &SAND);
		if (!placed_schematic)
			continue;
		placed.emplace_back(x, z);
	}
}
}
