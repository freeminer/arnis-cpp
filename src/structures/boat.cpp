#include "boat.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	if (!editor.ground)
		return;
	std::filesystem::path p =
		std::filesystem::path(__FILE__).parent_path().parent_path() /
		"assets/structures/boat.schem";
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
			place_schem_file_rotated(editor, p, ax, y, az, (h >> 5) & 3);
			++count;
		}
}
}
