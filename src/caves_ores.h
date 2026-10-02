#pragma once

#include "caves_shape.h"

#include <cstdint>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::caves
{
// Vanilla-style ore blobs for cave worlds, placed after water features so
// exposed-wall discard and host-rock selection see the finished cave network.
void place_ores_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y);
}
