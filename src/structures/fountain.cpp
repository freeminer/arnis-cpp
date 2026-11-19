#include "structures.h"
#include "../assets_root.h"
#include "../block_definitions.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include "../tile.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace arnis::structures::fountain
{
namespace
{
constexpr std::size_t LARGE_FOUNTAIN_CELLS = 300;
constexpr int MAX_CUT = 3;

std::shared_ptr<const StructureSchematic> selected_schematic(const WorldEditor &editor,
		int x, int z, std::size_t area_cells, unsigned &rotation)
{
	const auto h = land_cover::coord_hash(x, z);
	const auto variant = area_cells >= LARGE_FOUNTAIN_CELLS ? 4 : int((h >> 4) % 3) + 1;
	rotation = (h >> 8) & 3;
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	auto schematic = try_load_structure_cached(
			root / ("fountain" + std::to_string(variant) + ".schem"));
	if (!schematic)
		for (int fallback = 1; fallback <= 3 && !schematic; ++fallback)
			schematic = try_load_structure_cached(
					root / ("fountain" + std::to_string(fallback) + ".schem"));
	if (!schematic || schematic->max_extent > TILE_EDITOR_HALO)
		return {};
	return schematic;
}
}

std::vector<std::pair<int, int>> point_columns(const WorldEditor &editor, int x, int z)
{
	if (!editor.place_prop(Prop::Fountain))
		return {};
	unsigned rotation = 0;
	const auto schematic = selected_schematic(editor, x, z, 0, rotation);
	return schematic ? structure_base(*schematic, x, z, rotation,
							   std::numeric_limits<int>::max())
					 : std::vector<std::pair<int, int>>{};
}

void place(WorldEditor &editor, int x, int z, std::size_t area_cells)
{
	if (!editor.place_prop(Prop::Fountain))
		return;
	unsigned rotation = 0;
	const auto schematic = selected_schematic(editor, x, z, area_cells, rotation);
	if (!schematic)
		return;

	// Level the basin to the lowest terrain column in its footprint, but cap
	// excavation on steep terrain and support the downhill edge with stone.
	const auto columns =
			structure_base(*schematic, x, z, rotation, std::numeric_limits<int>::max());
	if (columns.empty())
		return;
	int low = std::numeric_limits<int>::max();
	int high = std::numeric_limits<int>::min();
	for (const auto &[cx, cz] : columns) {
		const int ground = editor.get_ground_level(cx, cz);
		low = std::min(low, ground);
		high = std::max(high, ground);
	}
	const int base_y = std::max(low, high - MAX_CUT);
	const std::vector<Block> replace_none;
	for (const auto &[cx, cz] : columns) {
		if (editor.surface_override(cx, cz))
			continue;
		const int ground = editor.get_ground_level(cx, cz);
		for (int y = base_y + 1; y <= ground; ++y)
			editor.set_block_absolute(
					block_definitions::AIR, cx, y, cz, nullptr, &replace_none);
		for (int y = ground + 1; y <= base_y; ++y)
			editor.set_block_absolute(
					block_definitions::STONE_BRICKS, cx, y, cz, nullptr, &replace_none);
		editor.register_road_surface_y(cx, cz, base_y);
	}
	place_structure(editor, *schematic, x, base_y + 1, z, rotation);
}
}
