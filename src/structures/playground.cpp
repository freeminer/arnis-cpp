#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "../block_definitions.h"
#include "schem_decoder.h"
#include <algorithm>
#include <memory>
#include <string>

namespace arnis::structures::playground
{
void scatter_playgrounds(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 120)
		return;
	const auto target = std::clamp<std::size_t>(cells.size() / 500, 1, 4);
	std::vector<std::pair<int, int>> placed;
	for (std::uint32_t t = 0; placed.size() < target && t < target * 8; ++t) {
		const auto h = land_cover::coord_hash(
				static_cast<int>(t) + 1, static_cast<int>(cells.size()));
		const auto [x, z] = cells[h % cells.size()];
		if (editor.is_lc_water(x, z) ||
				std::any_of(placed.begin(), placed.end(), [x = x, z = z](const auto &p) {
					const auto dx = static_cast<long long>(p.first) - x;
					const auto dz = static_cast<long long>(p.second) - z;
					return dx > -16 && dx < 16 && dz > -16 && dz < 16;
				}))
			continue;
		const int y = editor.get_absolute_y(x, 1, z);
		const unsigned rotation = (h >> 7) & 3;
		const int selected = static_cast<int>((h >> 5) % 3) + 1;
		const auto root = editor.get_schematic_asset_root().empty()
								  ? assets::path("structures")
								  : editor.get_schematic_asset_root();
		auto schematic = try_load_structure_cached(
				root / ("playground" + std::to_string(selected) + ".schem"));
		if (!schematic) {
			// Rust skips the candidate when its selected embedded asset fails to parse.
			continue;
		}
		// Rust's placement API has no success result, so a valid candidate consumes
		// a spacing slot even if every voxel is clipped by the active tile bounds.
		place_structure(editor, *schematic, x, y, z, rotation, &SAND);
		placed.emplace_back(x, z);
	}
}
}
