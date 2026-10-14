#pragma once

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::highways
{
// Returns false when the host has no native traffic-light head, allowing the
// caller to retain Rust's wool housing and patterned-banner rendering.
bool fm_place_native_traffic_signals(
		world_editor::WorldEditor &editor, int x, int head_base, int z);
}
