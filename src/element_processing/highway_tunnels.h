#pragma once

#include <unordered_set>
#include <vector>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis::highways
{
struct HighwayTunnelCell
{
	int x, z, road_y, half_width, terrain_y;
	bool covered, light;
	std::vector<Block> palette;
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

bool renders_as_highway_tunnel(const ProcessedWay &way);
TunnelInternalEndpoints collect_tunnel_internal_endpoints(
		const std::vector<ProcessedElement> &elements);
CoordinateBitmap collect_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, double scale);
void generate_highway_tunnel_shell(WorldEditor &editor, const ProcessedWay &way,
		const Args &args, const TunnelInternalEndpoints &internal,
		std::vector<HighwayTunnelCell> &cells);
void carve_highway_tunnel_interior(
		WorldEditor &editor, const std::vector<HighwayTunnelCell> &cells);
}
