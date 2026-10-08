#pragma once

#include "caves_density.h"
#include "caves_shape.h"
#include "../../arnis_world_editor.h"

#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

namespace arnis::caves
{
// Position-only dry-cave query shared by water and decoration planning. It
// evaluates the same global cave field regardless of which tile is writing.
class CaveShapeQuery
{
public:
	CaveShapeQuery(const world_editor::WorldEditor &editor, const CaveRect &world,
			std::int64_t seed, int floor_y, const CaveEllipsoids &ellipsoids);
	bool is_cave(int x, int y, int z) const;
	bool solid(int x, int y, int z) const;

private:
	double density_at(int x, int y, int z) const;
	bool noise_carves(int x, int y, int z) const;

	const world_editor::WorldEditor &editor_;
	CaveRect world_;
	int floor_y_;
	CaveGen gen_;
	const CaveEllipsoids &ellipsoids_;
	std::map<std::pair<int, int>, std::vector<std::size_t>> carvers_by_chunk_;
	mutable std::unordered_map<std::int64_t, double> density_cache_;
};
} // namespace arnis::caves
