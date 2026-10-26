#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::water_steps
{
struct Column
{
	std::int16_t top = std::numeric_limits<std::int16_t>::min();
	std::uint8_t depth = 0;
	std::uint8_t flags = 0;
	bool free() const { return (flags & 2) != 0; }
	bool wet_at(int y) const
	{
		return depth > 0 && y <= top && static_cast<int>(top) - y < depth;
	}
};

struct Borders
{
	using Region = std::pair<int, int>;
	using Candidate = std::tuple<int, int, int>;
	using Border = std::vector<std::pair<std::uint32_t, Column>>;
	std::map<Region, std::vector<Candidate>> candidates;
	std::map<Region, Border> saved;
	std::set<Region> shaped;
};

// Preserve wet-column candidates from a completed tile before its local editor
// is merged or released. Coordinates outside the tile's owned area are ignored.
void collect_region_candidates(const world_editor::WorldEditor &editor, Borders &borders,
		int min_x, int max_x, int min_z, int max_z);

// Shape a completed 512x512 region while keeping only its 20-block water border
// for neighbouring regions that have not been shaped yet.
void shape_region(world_editor::WorldEditor &editor, Borders &borders,
		Borders::Region region, bool caves,
		const std::function<bool(Borders::Region)> &exists);

// Replace exposed, plain source-water blocks immediately before a one-node
// drop with the flowing levels produced by vanilla water propagation.
void shape_world(world_editor::WorldEditor &editor, bool caves);
}
