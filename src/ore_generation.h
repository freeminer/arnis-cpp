#pragma once
#include <array>
#include "../../arnis_block.h"
#include "coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::ore_generation
{
inline constexpr int MAX_ORE_DEPTH = 384;
struct OreRule
{
	Block block;
	int depth_min, depth_max;
	unsigned vein_min, vein_max, avg_veins_per_chunk;
};
const std::array<OreRule, 6> &rules();
void generate_ores(
		world_editor::WorldEditor &, int, int, int, int, bool show_progress = true);
inline void generate_ores(world_editor::WorldEditor &editor,
		const cartesian::XZBBox &bounds, bool show_progress = true)
{
	generate_ores(editor, bounds.min_x(), bounds.max_x(), bounds.min_z(), bounds.max_z(),
			show_progress);
}
inline void generate_ores_region(world_editor::WorldEditor &editor, int min_x, int max_x,
		int min_z, int max_z, bool show_progress = true)
{
	generate_ores(editor, min_x, max_x, min_z, max_z, show_progress);
}
}
