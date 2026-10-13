#pragma once

#include "../../../arnis_adapter.h"

namespace arnis::highways
{

/// Physical radius used by the OSM `aeroway=helipad` node renderer and tile
/// assignment; kept shared so tile ownership covers the exact rendered disc.
inline constexpr double HELIPAD_NODE_RADIUS_M = 8.0;

// Default total lane count for a highway when OSM does not provide `lanes`.
// Kept public within the generator module to match Rust's pre-scan API.
int highway_default_lanes(const std::string &highway_type);

// Number of lanes used for road marking layout, including tagged overrides.
int lane_marking_count(const std::string &highway_type, const crate::tags_t &tags);

} // namespace arnis::highways
