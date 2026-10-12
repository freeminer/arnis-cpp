#include "helicopter.h"
#include "../assets_root.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void maybe_place_helicopter(WorldEditor &e, int x, int z)
{
	if (!e.place_schematics())
		return;
	auto h = land_cover::coord_hash(x, z);
	if (h % 100 >= 60 || e.is_lc_water(x, z))
		return;
	const auto root = e.get_schematic_asset_root().empty() ? assets::path("structures")
														   : e.get_schematic_asset_root();
	std::shared_ptr<const StructureSchematic> schematic;
	try {
		schematic = load_centered_structure_cached(root / "helicopter.schem");
	} catch (...) {
		return;
	}
	place_structure(e, *schematic, x, e.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}
