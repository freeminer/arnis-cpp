#pragma once

#include "../../../arnis_adapter.h"

#include <unordered_map>

namespace arnis::highways
{

/// Physical radius used by the OSM `aeroway=helipad` node renderer and tile
/// assignment; kept shared so tile ownership covers the exact rendered disc.
inline constexpr double HELIPAD_NODE_RADIUS_M = 8.0;

// Default total lane count for a highway when OSM does not provide `lanes`.
// Kept public within the generator module to match Rust's pre-scan API.
int highway_default_lanes(const std::string &highway_type);

// True when a node's OSM tags identify a traffic signal accepted by the
// highway-node dispatcher.
bool is_signal(const tags_t &tags);

// Effective half-width of a paved way, including lane and explicit width tags.
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);

// Number of lanes used for road marking layout, including tagged overrides.
int lane_marking_count(const std::string &highway_type, const crate::tags_t &tags);

} // namespace arnis::highways
