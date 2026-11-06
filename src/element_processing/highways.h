#pragma once

#include "../../../arnis_adapter.h"
#include "../block_definitions.h"
#include "fm_highway_connectivity.h"

#include <unordered_map>
#include <vector>
#include <optional>
#include <string>
#include <functional>
#include <span>
#include <memory>

namespace arnis
{
struct ProcessedWay;
class FloodFillCache;
class CoordinateBitmap;
struct Args;
namespace world_editor
{
struct WorldEditor;
}
namespace bridges
{
struct BridgeMemberInfo;
struct BridgeRampInfo;
class BridgeStructureMap;
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

/// Canonical terrain-following surface profile shared by road and bridge ramps.
std::vector<int> road_profile(const world_editor::WorldEditor &editor,
		const ProcessedWay &way, int block_range, const HighwayConnectivity &connectivity,
		const std::function<std::optional<int>(int, int)> &pin_at);
/// Stable ground level used for roads meeting at one connected junction.
int junction_level(const world_editor::WorldEditor &editor, int x, int z);

class TunnelPortalMap;
// Exact road surface for moving traffic: a deck, ramp, or the laid at-grade
// profile, rather than the top of whatever happens to occupy a column.
class RoadSurfaces
{
public:
	RoadSurfaces(const world_editor::WorldEditor &editor,
			const bridges::BridgeStructureMap &bridges,
			const HighwayConnectivity &connectivity,
			const TunnelPortalMap &tunnel_portals, double scale);
	~RoadSurfaces();
	int y_at(const ProcessedWay &way, int x, int z) const;

private:
	struct WaySurface;
	const world_editor::WorldEditor &editor_;
	const bridges::BridgeStructureMap &bridges_;
	const HighwayConnectivity &connectivity_;
	const TunnelPortalMap &tunnel_portals_;
	double scale_;
	mutable std::unordered_map<std::uint64_t, std::shared_ptr<const WaySurface>> ways_;
	std::shared_ptr<const WaySurface> surface_of(const ProcessedWay &way) const;
};

// Closed aeroway polygons handled by the aeroway renderer and flood-fill cache.
enum class AerowayAreaKind
{
	Runway,
	Apron,
};
std::optional<AerowayAreaKind> aeroway_area_kind(const ProcessedWay &way);
bool is_aeroway_area(const ProcessedWay &way);
// Remove buildings that occupy aircraft pavement before building-dependent
// indices and masks are prepared. The filtered output is left untouched when
// no buildings are removed, avoiding a copy of the OSM element vector.
std::size_t drop_buildings_on_aircraft_pavement(
		const std::vector<ProcessedElement> &elements,
		std::vector<ProcessedElement> &filtered_elements, double scale);
// Paved airport areas which take part in sealed-surface precedence.
std::optional<crate::block_definitions::Block> aeroway_area_block(
		const ProcessedWay &way);

/// Remove street lamps whose plinths were swallowed by the later water pass.
void drop_lamps_in_water(world_editor::WorldEditor &editor);

void generate_aeroway(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const Args &args, const CoordinateBitmap &building_footprints,
		const FloodFillCache &fill_cache);

// Effective half-width of a paved way, including lane and explicit width tags.
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);

// Number of lanes used for road marking layout, including tagged overrides.
int lane_marking_count(const std::string &highway_type, const crate::tags_t &tags);

// Rust exposes the selected way palette as a static block slice. The returned
// palette is backed by process-lifetime storage, so callers can retain this
// view without copying a vector for every OSM way.
std::span<const crate::block_definitions::Block> surface_palette_view(
		const std::string &highway_type, const crate::tags_t &tags);

} // namespace arnis::highways
