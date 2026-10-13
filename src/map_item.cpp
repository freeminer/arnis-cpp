#include "map_item.h"
#include "world_utils.h"
#include <algorithm>
#include <array>
#include <limits>
namespace arnis::map_item
{
std::tuple<int, std::int8_t, bool> map_geometry(int d)
{
	for (int s = 0; s <= 4; ++s) {
		const int blocks = 1 << s;
		if (d <= MAP_SIZE * blocks)
			return {blocks, static_cast<std::int8_t>(s), true};
	}
	// An oversized preview still uses scale 4, but fit the full world into its
	// 128 pixels instead of clipping it to Minecraft's largest standard map.
	const int blocks = d / MAP_SIZE + (d % MAP_SIZE != 0);
	return {blocks, 4, false};
}
}

namespace arnis::map_item
{
int next_map_id(const std::filesystem::path &maps_dir)
{
	std::optional<int> highest;
	for (const char *name : {"idcounts.dat", "last_id.dat"}) {
		if (const auto value = world_utils::map_id_counter(maps_dir / name))
			highest = highest ? std::max(*highest, *value) : *value;
	}
	return highest ? (*highest == std::numeric_limits<int>::max() ? *highest
																  : *highest + 1)
				   : 0;
}
} // namespace arnis::map_item
