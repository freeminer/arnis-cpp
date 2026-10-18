#include "starship.h"
#include "../assets_root.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void place_starship(WorldEditor &e, const ProcessedWay &w)
{
	if (!e.place_schematics() || w.nodes.empty())
		return;
	long long sx = 0, sz = 0;
	for (auto &n : w.nodes) {
		sx += n.x;
		sz += n.z;
	}
	int x = sx / w.nodes.size(), z = sz / w.nodes.size();
	const auto root = e.get_schematic_asset_root().empty() ? assets::path("structures")
														   : e.get_schematic_asset_root();
	const auto p = root / "starship.schem";
	// Rust loads a StructureSchematic and anchors it at its tallest column;
	// do not place the raw Sponge document using its optional Offset fields.
	auto schematic = try_load_structure_cached(p);
	if (schematic)
		place_structure(e, *schematic, x, e.get_absolute_y(x, 1, z), z, 0);
}
}
