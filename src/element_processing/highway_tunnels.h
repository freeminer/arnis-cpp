#pragma once

#include <array>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <vector>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis::bridges
{
class BridgeStructureMap;
}

namespace arnis::highways
{
struct PortalFace
{
	std::pair<int, int> at;
	std::pair<int, int> out;
};

struct HighwayTunnelCell
{
	int x, z, road_y, half_width, terrain_y;
	bool covered, light;
	std::vector<Block> palette;
	std::vector<PortalFace> faces;
};

struct TunnelPointHash
{
	size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		return std::hash<long long>{}((static_cast<long long>(p.first) << 32) ^
									  static_cast<unsigned int>(p.second));
	}
};
using TunnelInternalEndpoints = std::unordered_set<std::pair<int, int>, TunnelPointHash>;

struct ApproachClaim
{
	int anchor = 0;
	int drop = 0;
	int run = 1;
};

struct TunnelApproach
{
	std::array<std::optional<ApproachClaim>, 2> claims;
	void push(ApproachClaim claim);
	int offset(int distance_from_start) const;
	bool full() const { return claims[0].has_value() && claims[1].has_value(); }
};

class TunnelPortalMap
{
public:
	bool empty() const { return ways_.empty(); }
	std::optional<TunnelApproach> approach(std::uint64_t way_id) const;
	int drop_at(const std::pair<int, int> &point) const;

private:
	friend TunnelPortalMap collect_tunnel_portals(const std::vector<ProcessedElement> &,
			const WorldEditor &, const bridges::BridgeStructureMap &,
			const TunnelInternalEndpoints &, double);
	std::unordered_map<std::uint64_t, TunnelApproach> ways_;
	std::unordered_map<std::pair<int, int>, int, TunnelPointHash> nodes_;
};

bool renders_as_highway_tunnel(const ProcessedWay &way);
bool tunnel_bore_fits(const WorldEditor &editor, const ProcessedWay &way, double scale);
TunnelInternalEndpoints collect_tunnel_internal_endpoints(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox);
TunnelPortalMap collect_tunnel_portals(const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor, const bridges::BridgeStructureMap &bridges,
		const TunnelInternalEndpoints &internal, double scale);
CoordinateBitmap collect_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor, const TunnelInternalEndpoints &internal,
		const XZBBox &xzbbox, double scale);
bool generate_highway_tunnel_shell(WorldEditor &editor, const ProcessedWay &way,
		const Args &args, const TunnelInternalEndpoints &internal,
		const TunnelPortalMap &portals, std::vector<HighwayTunnelCell> &cells);
void carve_highway_tunnel_interior(
		WorldEditor &editor, const std::vector<HighwayTunnelCell> &cells);
}
