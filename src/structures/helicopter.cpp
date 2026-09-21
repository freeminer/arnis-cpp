#include "helicopter.h"
#include "schem_decoder.h"
#include <fstream>
#include <iterator>
namespace arnis::structures
{
void maybe_place_helicopter(WorldEditor &e, int x, int z)
{
	if (!e.place_schematics())
		return;
	auto h = land_cover::coord_hash(x, z);
	if (h % 100 >= 60 || e.is_lc_water(x, z))
		return;
	auto p = std::filesystem::path(__FILE__).parent_path().parent_path() /
			 "assets/structures/helicopter.schem";
	std::ifstream in(p, std::ios::binary);
	if (!in)
		return;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	try {
		// Rust centers the helicopter schematic before placing it at the pad
		// coordinate; the generic file helper preserves the raw origin.
		auto schematic = decode_sponge_schem(bytes).centered();
		place_structure(e, schematic, x, e.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
	} catch (...) {
		return;
	}
}
}
