#pragma once
#include "caves_shape.h"

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
// Deterministic cave pass used by the C++ world generator.  It keeps the
// region-local contract of Rust's carve_region entry point and is safe to run
// after terrain fill but before ores and decorations.
void carve_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y);
// Rust caves::decoration counterpart for post-carve cave flora/mineral accents.
void decorate_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args);
// Rust caves::schems counterpart: load the configured cave-pack manifest and
// deterministically stamp formations into carved cave openings.
void stamp_schematics_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args);
}
