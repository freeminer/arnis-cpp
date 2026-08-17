#include "starship.h"
#include "schem_decoder.h"
namespace arnis::structures
{
void place_starship(WorldEditor &e, const ProcessedWay &w)
{
	if (w.nodes.empty())
		return;
	long long sx = 0, sz = 0;
	for (auto &n : w.nodes) {
		sx += n.x;
		sz += n.z;
	}
	int x = sx / w.nodes.size(), z = sz / w.nodes.size();
	auto p = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
			 "assets/structures/starship.schem";
	place_schem_file(e, p, x, e.get_absolute_y(x, 1, z), z);
}
}
