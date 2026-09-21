#include "boat.h"
#include "schem_decoder.h"
#include <fstream>
#include <iterator>
#include <mutex>
namespace arnis::structures
{
namespace
{
const StructureSchematic *boat_schematic()
{
	static std::once_flag once;
	static std::optional<StructureSchematic> schematic;
	std::call_once(once, [] {
		const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
						  "assets/structures/boat.schem";
		std::ifstream input(path, std::ios::binary);
		if (!input)
			return;
		try {
			std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
			schematic = load_structure(bytes).centered();
		} catch (...) {
			schematic.reset();
		}
	});
	return schematic ? &*schematic : nullptr;
}
}

void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	// Match Rust: structure scattering is disabled when schematic placement is
	// disabled, even if terrain/land-cover data is available.
	if (!editor.ground || !editor.place_schematics())
		return;
	const auto *schematic = boat_schematic();
	if (!schematic)
		return;
	int count = 0;
	// Floor division, equivalent to Rust's rem_euclid, keeps the lattice
	// identical across tile boundaries west/south of coordinate zero.
	auto lattice_start = [](int value) {
		const int remainder = ((value % 400) + 400) % 400;
		return value - remainder;
	};
	for (int z = lattice_start(min_z); z <= max_z && count < 200; z += 400)
		for (int x = lattice_start(min_x); x <= max_x && count < 200; x += 400) {
			uint64_t h = land_cover::coord_hash(x, z);
			if (h % 100 >= 45)
				continue;
			int ax = x + h % 7, az = z + (h >> 3) % 7;
			if (!editor.is_lc_water(ax, az) || editor.water_distance(ax, az) != 0)
				continue;
			int y = editor.get_water_level(ax, az) - 1;
			place_structure(editor, *schematic, ax, y, az, (h >> 5) & 3);
			++count;
		}
}
}
