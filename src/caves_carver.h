#pragma once
#include "caves_shape.h"
#include "caves_shape_query.h"
#include <unordered_set>

namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis
{
struct Args;
}

namespace arnis::caves
{
struct WaterPlan;
// Deterministic cave pass used by the C++ world generator.  It keeps the
// region-local contract of Rust's carve_region entry point and is safe to run
// after terrain fill but before ores and decorations.
void carve_region(world_editor::WorldEditor &editor, const CaveRect &region,
		const CaveRect &world_bounds, std::int64_t seed, int floor_y,
		CaveEllipsoids *ellipsoids = nullptr,
		std::unordered_set<std::int64_t> *cave_air = nullptr);
// Rust caves::decoration counterpart for post-carve cave flora/mineral accents.
void decorate_region(world_editor::WorldEditor &editor, const CaveRect &region,
		const CaveRect &world_bounds, std::int64_t seed, int floor_y, const Args &args,
		const CaveShapeQuery &cave_shape, const WaterPlan &water_plan,
		const std::unordered_set<std::int64_t> *basin_fluid = nullptr,
		const std::unordered_set<std::int64_t> *water_cells = nullptr,
		const std::unordered_set<std::int64_t> *cave_air = nullptr);
// Rust caves::schems counterpart: load the configured cave-pack manifest and
// deterministically stamp formations into carved cave openings.
void stamp_schematics_region(world_editor::WorldEditor &editor, const CaveRect &region,
		const std::unordered_set<std::int64_t> &cave_air, std::int64_t seed, int floor_y,
		const Args &args, std::unordered_set<std::int64_t> *basin_fluid = nullptr);
}
