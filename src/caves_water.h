#pragma once

#include "caves_shape.h"
#include "caves_shape_query.h"
#include "../../arnis_world_editor.h"
#include <vector>
#include <unordered_set>

namespace arnis
{
struct Args;
}
namespace arnis::caves
{
struct WaterPlan;

// Rust caves::water::plan/apply counterpart. Plan against full world bounds,
// write only the current region, and optionally return the plan for later
// feature passes such as geodes.
void generate_water_features(world_editor::WorldEditor &editor,
		const CaveRect &world_bounds, const CaveRect &region, std::int64_t seed,
		int floor_y, const Args &args, const CaveShapeQuery &cave_shape,
		std::unordered_set<std::int64_t> *basin_fluid = nullptr,
		std::unordered_set<std::int64_t> *water_cells_out = nullptr,
		std::unordered_set<std::int64_t> *cave_air_out = nullptr,
		WaterPlan *plan_out = nullptr);

// Rust caves::seal_floating_fluid_region counterpart. Run after all cave,
// surface-water, and tunnel carving passes so fluid columns cannot hang over air.
void seal_floating_fluid_region(
		world_editor::WorldEditor &editor, const CaveRect &region, int floor_y);

struct WaterPlan
{
	std::unordered_set<std::int64_t> carved;
	std::unordered_set<std::int64_t> water;
	// Rust WaterPlan::solid counterpart; the shared query supplies the cave and
	// terrain envelope while this plan accounts for future water and carved cells.
	bool solid(const CaveShapeQuery &shape, int x, int y, int z) const;
	void apply(world_editor::WorldEditor &editor, const CaveRect &region,
			const std::vector<Block> &cave_host) const;
};

}
