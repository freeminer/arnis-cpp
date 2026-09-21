#include "helicopter.h"
#include "schem_decoder.h"
#include <fstream>
#include <iterator>
#include <mutex>
namespace arnis::structures
{
void maybe_place_helicopter(WorldEditor &e, int x, int z)
{
	if (!e.place_schematics())
		return;
	auto h = land_cover::coord_hash(x, z);
	if (h % 100 >= 60 || e.is_lc_water(x, z))
		return;
	static std::once_flag once;
	static std::optional<StructureSchematic> schematic;
	std::call_once(once, [] {
		const auto p = std::filesystem::path(__FILE__).parent_path().parent_path() /
					   "assets/structures/helicopter.schem";
		std::ifstream in(p, std::ios::binary);
		if (!in)
			return;
		try {
			std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
			// Rust caches the centered schematic in OnceLock.
			schematic = load_structure(bytes).centered();
		} catch (...) {
			schematic.reset();
		}
	});
	if (!schematic)
		return;
	place_structure(e, *schematic, x, e.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}
