#include "highway_tunnels.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "../bresenham.h"
#include "surfaces.h"

namespace arnis::highways
{
namespace
{
constexpr int CEIL_OFFSET = 5, COVER_DROP = 7, RAMP_RUN = 3, LAYER_DROP = 7;

Block shell_block(int x, int y, int z)
{
	const uint32_t h = static_cast<uint32_t>(x) * 73856093u +
					   static_cast<uint32_t>(y) * 19349663u +
					   static_cast<uint32_t>(z) * 83492791u;
	return h % 100 < 15	  ? CRACKED_STONE_BRICKS
		   : h % 100 < 18 ? MOSSY_STONE_BRICKS
						  : STONE_BRICKS;
}
int half_width(const std::string &type, double scale)
{
	double metres = (type == "motorway" || type == "trunk")					   ? 4.0
					: (type == "primary" || type == "secondary")			   ? 3.0
					: (type == "footway" || type == "path" || type == "steps") ? 1.0
																			   : 2.0;
	return std::max(1, static_cast<int>(std::round(metres * scale)));
}
}

bool renders_as_highway_tunnel(const ProcessedWay &way)
{
	if (!way.tags.contains("highway") || way.nodes.size() < 2 ||
			way.tags.get("tunnel") != "yes")
		return false;
	if (way.tags.get("indoor") == "yes" || way.tags.get("area") == "yes")
		return false;
	if (auto it = way.tags.find("level"); it != way.tags.end())
		try {
			if (std::stoi(it->second) < 0)
				return false;
		} catch (...) {
		}
	const auto type = way.tags.get("highway");
	return type != "street_lamp" && type != "crossing" && type != "bus_stop" &&
		   type != "proposed" && type != "construction" && type != "razed";
}

TunnelInternalEndpoints collect_tunnel_internal_endpoints(
		const std::vector<ProcessedElement> &elements)
{
	std::unordered_map<std::pair<int, int>, unsigned, TunnelPointHash> counts;
	for (const auto &e : elements)
		if (e.is_way() && renders_as_highway_tunnel(e.as_way())) {
			const auto &w = e.as_way();
			++counts[{w.nodes.front().x, w.nodes.front().z}];
			if (w.nodes.front().x != w.nodes.back().x ||
					w.nodes.front().z != w.nodes.back().z)
				++counts[{w.nodes.back().x, w.nodes.back().z}];
		}
	TunnelInternalEndpoints result;
	for (const auto &[p, count] : counts)
		if (count > 1)
			result.insert(p);
	return result;
}

CoordinateBitmap collect_tunnel_footprint(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox, double scale)
{
	CoordinateBitmap result(xzbbox);
	for (const auto &e : elements) {
		if (!e.is_way() || !renders_as_highway_tunnel(e.as_way()))
			continue;
		const auto &way = e.as_way();
		const auto type = way.tags.get("highway");
		const int wall = half_width(type.empty() ? "road" : type, scale) + 1;
		for (size_t i = 1; i < way.nodes.size(); ++i)
			for (auto [x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
						 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z))
				for (int dx = -wall; dx <= wall; ++dx)
					for (int dz = -wall; dz <= wall; ++dz)
						result.set(x + dx, z + dz);
	}
	return result;
}

void generate_highway_tunnel_shell(WorldEditor &editor, const ProcessedWay &way,
		const Args &args, const TunnelInternalEndpoints &internal,
		std::vector<HighwayTunnelCell> &cells)
{
	if (!renders_as_highway_tunnel(way))
		return;
	std::vector<std::pair<int, int>> pts;
	for (size_t n = 1; n < way.nodes.size(); ++n)
		for (auto [x, y, z] : bresenham_line(way.nodes[n - 1].x, 0, way.nodes[n - 1].z,
					 way.nodes[n].x, 0, way.nodes[n].z))
			if (pts.empty() || pts.back() != std::pair{x, z})
				pts.emplace_back(x, z);
	if (pts.size() < 2)
		return;
	std::vector<int> terrain;
	for (auto [x, z] : pts)
		terrain.push_back(editor.get_ground_level(x, z));
	const size_t last = pts.size() - 1;
	const bool start_internal = internal.contains(pts.front()),
			   end_internal = internal.contains(pts.back());
	int layer = 0;
	if (auto it = way.tags.find("layer"); it != way.tags.end())
		try {
			layer = std::stoi(it->second);
		} catch (...) {
		}
	const int extra = std::max(0, -(layer + 1)) * LAYER_DROP;
	std::vector<int> road(pts.size());
	for (size_t i = 0; i < pts.size(); ++i) {
		const double t = static_cast<double>(i) / last;
		const int grade =
				std::lround(terrain.front() + (terrain.back() - terrain.front()) * t);
		const int desired = std::min(grade, terrain[i] - COVER_DROP) - extra;
		const int rs = start_internal ? INT_MIN
									  : terrain.front() - static_cast<int>(i) / RAMP_RUN;
		const int re = end_internal
							   ? INT_MIN
							   : terrain.back() - static_cast<int>(last - i) / RAMP_RUN;
		road[i] = std::min(terrain[i], std::max(desired, std::max(rs, re)));
	}
	for (size_t i = 1; i < road.size(); ++i)
		road[i] = std::min(road[i], road[i - 1] + 1);
	for (size_t i = last; i-- > 0;)
		road[i] = std::min(road[i], road[i + 1] + 1);
	std::string type = way.tags.get("highway");
	if (type.empty())
		type = "road";
	const int hw = half_width(type, args.scale), wall = hw + 1;
	std::vector<Block> defaults =
			(type == "path") ? std::vector<Block>{DIRT_PATH}
			: (type == "footway" || type == "pedestrian" || type == "service" ||
					  type == "steps")
					? std::vector<Block>{GRAY_CONCRETE}
					: std::vector<Block>{GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA};
	const auto palette = surfaces::get_blocks_for_surface_way(way, defaults);
	for (size_t i = 0; i < pts.size(); ++i) {
		auto [x, z] = pts[i];
		const int ry = road[i], ty = terrain[i];
		const bool covered = ty >= ry + COVER_DROP;
		const int top = covered ? ry + CEIL_OFFSET : ty;
		for (int dx = -wall; dx <= wall; ++dx)
			for (int dz = -wall; dz <= wall; ++dz)
				for (int y = ry - 1; y <= top; ++y) {
					const bool edge = std::abs(dx) == wall || std::abs(dz) == wall ||
									  (covered && y == ry + CEIL_OFFSET);
					editor.set_block_absolute(
							edge ? shell_block(x + dx, y, z + dz) : STONE_BRICKS, x + dx,
							y, z + dz);
				}
		cells.push_back(HighwayTunnelCell{.x = x,
				.z = z,
				.road_y = ry,
				.half_width = hw,
				.terrain_y = ty,
				.covered = covered,
				.light = covered && i % 8 == 0,
				.palette = palette});
	}
}

void carve_highway_tunnel_interior(
		WorldEditor &editor, const std::vector<HighwayTunnelCell> &cells)
{
	const std::vector<Block> carve{STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS,
			STONE, WATER, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA, GRAY_CONCRETE,
			BLACK_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE};
	const std::vector<Block> road_wl{
			AIR, STONE, STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS, WATER};
	for (const auto &c : cells) {
		const int top = c.covered ? c.road_y + CEIL_OFFSET - 1 : c.terrain_y;
		for (int dx = -c.half_width; dx <= c.half_width; ++dx)
			for (int dz = -c.half_width; dz <= c.half_width; ++dz)
				for (int y = c.road_y + 1; y <= top; ++y)
					editor.set_block_absolute(
							AIR, c.x + dx, y, c.z + dz, carve, std::nullopt);
		if (c.light)
			editor.set_block_absolute(SEA_LANTERN, c.x, c.road_y + CEIL_OFFSET - 1, c.z);
	}
	for (const auto &c : cells)
		for (int dx = -c.half_width; dx <= c.half_width; ++dx)
			for (int dz = -c.half_width; dz <= c.half_width; ++dz)
				editor.set_block_absolute(
						surfaces::semirandom_surface(c.x + dx, c.z + dz, c.palette),
						c.x + dx, c.road_y, c.z + dz, road_wl, std::nullopt);
}
}
