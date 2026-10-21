#pragma once

#include "../../../arnis_adapter.h"
#include "../block_definitions.h"

#include <unordered_map>
#include <vector>
#include <optional>
#include <string>

namespace arnis
{
struct ProcessedWay;
namespace world_editor
{
struct WorldEditor;
}
}

namespace arnis::highways
{

/// Physical radius used by the OSM `aeroway=helipad` node renderer and tile
/// assignment; kept shared so tile ownership covers the exact rendered disc.
inline constexpr double HELIPAD_NODE_RADIUS_M = 8.0;

// Default total lane count for a highway when OSM does not provide `lanes`.
// Kept public within the generator module to match Rust's pre-scan API.
int highway_default_lanes(const std::string &highway_type);
std::optional<double> parse_width_tag_m(
		const std::unordered_map<std::string, std::string> &tags);

/// Shared highway classification used by surface masks, barriers and signage.
bool is_carriageway(const std::string &highway_type);
std::optional<std::string> surface_highway_type(const ProcessedWay &way);
bool tunnel_renders_at_surface(
		const ProcessedWay &way, const world_editor::WorldEditor *editor, double scale);

// True when a node's OSM tags identify a traffic signal accepted by the
// highway-node dispatcher.
bool is_signal(const tags_t &tags);

// Effective half-width of a paved way, including lane and explicit width tags.
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);

// Number of lanes used for road marking layout, including tagged overrides.
int lane_marking_count(const std::string &highway_type, const crate::tags_t &tags);

// Rust exposes the selected way palette as a static block slice. The returned
// palette is backed by process-lifetime storage, so callers can retain this
// view without copying a vector for every OSM way.
const std::vector<crate::block_definitions::Block> *surface_palette_view(
		const std::string &highway_type, const crate::tags_t &tags);

} // namespace arnis::highways
