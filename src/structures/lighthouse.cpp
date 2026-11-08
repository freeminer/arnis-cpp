#include "structures.h"
#include "../assets_root.h"
#include "../block_definitions.h"
#include "../element_processing/waterfront.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures::lighthouse
{
namespace
{
constexpr int GROUND_STOREY = 10;
}

void place(WorldEditor &editor, int x, int z)
{
	if (!editor.place_schematics())
		return;
	const auto h = land_cover::coord_hash(x, z);
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	auto schematic = try_load_structure_cached(root / "lighthouse.schem");
	if (!schematic)
		return;
	const auto rotation = static_cast<unsigned>(h & 3);
	const int base_y = editor.get_absolute_y(x, 1, z);
	if (!place_structure(editor, *schematic, x, base_y, z, rotation))
		return;
	for (const auto &[cx, cz] : structure_base(*schematic, x, z, rotation, GROUND_STOREY))
		waterfront::drive_pile(
				editor, cx, cz, base_y - 1, block_definitions::STONE_BRICKS);
}
}
