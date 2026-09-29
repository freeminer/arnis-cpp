#pragma once

#include "caves_shape.h"
#include "../../arnis_world_editor.h"
#include <unordered_set>
#include <functional>
#include <vector>

namespace arnis::caves
{

// Rust caves::water::plan/apply counterpart. Features are keyed by origin
// chunk so neighbouring generated regions agree on pools and rivers.
void generate_water_features(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y);

struct WaterPlan
{
	std::unordered_set<std::int64_t> carved;
	std::unordered_set<std::int64_t> water;
	// A planned cell is solid only when it is water, untouched terrain, or outside
	// the dry cave predicate. The predicate is supplied by the caller so the plan
	// remains independent of a particular cave-density implementation.
	bool solid(const CaveRect &world, int floor, int x, int y, int z,
			const std::function<bool(int, int, int)> &is_cave) const;
	void apply(world_editor::WorldEditor &editor, const CaveRect &region,
			const std::vector<Block> &cave_host) const;
};

}
