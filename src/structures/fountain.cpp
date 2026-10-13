#include "structures.h"
#include "../assets_root.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"

#include <memory>
#include <string>

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
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	const auto load_variant = [&](int index) {
		return try_load_structure_cached(
				root / ("fountain" + std::to_string(index) + ".schem"));
	};

	// Rust chooses a fallback only when decoding the requested asset failed.
	// Keep that decision separate from placement: a valid schematic that clips
	// entirely at a tile edge must not cause a different fountain to be stamped.
	auto schematic = load_variant(variant);
	if (!schematic)
		for (int fallback = 1; fallback <= 3 && !schematic; ++fallback)
			schematic = load_variant(fallback);
	if (schematic)
		place_structure(editor, *schematic, x, y, z, rotation);
}
}
