#include "starship.h"
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
	auto p = std::filesystem::path(__FILE__).parent_path().parent_path() /
			 "assets/structures/starship.schem";
	// Rust anchors the embedded model at the launch-mount centreline with an
	// explicit upright rotation; use the rotated path so property-bearing
	// schematic blocks follow the same placement contract.
	place_schem_file_rotated(e, p, x, e.get_absolute_y(x, 1, z), z, 0);
}
}
