#include "structures.h"
#include "../land_cover/land_cover.h"
#include "schem_decoder.h"
#include <algorithm>

namespace arnis::structures::crane
{
void maybe_place_crane(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells)
{
	if (!editor.place_schematics() || cells.size() < 1500)
		return;
	long long sx = 0, sz = 0;
	for (const auto &[x, z] : cells) {
		sx += x;
		sz += z;
	}
	const int cx = sx / static_cast<long long>(cells.size());
	const int cz = sz / static_cast<long long>(cells.size());
	const auto it = std::min_element(
			cells.begin(), cells.end(), [cx, cz](const auto &a, const auto &b) {
				const auto distance = [cx, cz](const auto &p) {
					const long long dx = static_cast<long long>(p.first) - cx;
					const long long dz = static_cast<long long>(p.second) - cz;
					return dx * dx + dz * dz;
				};
				return distance(a) < distance(b);
			});
	const auto [x, z] = *it;
	const auto h = land_cover::coord_hash(x, z);
	if (editor.is_lc_water(x, z) || h % 100 >= 60)
		return;
	place_named_schem(
			editor, "crane", x, editor.get_absolute_y(x, 1, z), z, (h >> 8) & 3);
}
}
