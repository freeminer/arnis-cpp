#pragma once
#include <cstdint>
#include <utility>
#include <vector>
#include "../../floodfill_cache.h"
#include "interior/index.h"
namespace arnis
{

void generate_building_interior(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &floor_area, int min_x, int min_z,
		int max_x, int max_z, int start_y_offset, int building_height, Block wall_block,
		Block floor_block, const std::vector<int> &floor_levels, const Args &args,
		const ProcessedWay &element, int abs_terrain_offset, bool is_abandoned_building,
		const CoordinateBitmap &building_passages, bool has_sloped_roof,
		const interior_uses::InteriorPlan *plan = nullptr,
		const std::vector<interior_uses::Claim> &claims = {}, double scale = 1.0,
		const std::vector<interior_uses::Entry> &entries = {},
		std::uint64_t interior_seed = 0);
}
