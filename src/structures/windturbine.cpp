#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

namespace arnis::structures::windturbine
{
void place(WorldEditor &editor, int x, int z)
{
	if (!editor.place_schematics())
		return;
	const auto h = land_cover::coord_hash(x, z);
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	place_schem_file_anchored(editor, root / "windturbine.schem", x,
			editor.get_absolute_y(x, 1, z), z, h & 3, SchemAnchor::BaseCentroid);
}
}
