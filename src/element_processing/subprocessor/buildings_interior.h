#pragma once
#include <cstdint>
#include <utility>
#include <vector>

#include "../../arnis_adapter.h"
#include "../../floodfill_cache.h"
#include "interior/index.h"

namespace arnis
{

// Mirrors Rust's interior::InteriorRequest: the furnishing pass receives one
// coherent, immutable description of the building and its ownership context.
struct InteriorRequest
{
	const std::vector<std::pair<int, int>> &footprint;
	const std::vector<int> &floor_levels;
	int start_y_offset;
	int building_height;
	int abs_terrain_offset;
	Block wall_block;
	const interior_uses::InteriorPlan *plan;
	bool abandoned;
	const CoordinateBitmap &passages;
	const std::vector<interior_uses::Entry> &entrances;
	std::pair<std::pair<int, int>, std::pair<int, int>> bounds;
	const std::vector<interior_uses::Claim> &claims;
	double scale;
	Block floor_block;
	std::uint64_t seed;
};

void generate_building_interior(WorldEditor &editor, const InteriorRequest &request);
}
