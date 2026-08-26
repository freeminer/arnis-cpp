#include "highway_tunnels.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_map>

#include "../bresenham.h"
#include "../world_editor/floor_state.h"
#include "bridges.h"
#include "surfaces.h"

namespace arnis::highways
{

int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);

namespace
{
constexpr int CEIL_OFFSET = 5, COVER_DROP = 7, RAMP_RUN = 3, LAYER_DROP = 7;
int min_road_y()
{
	return world_editor::terrain_floor_y() + 2;
}

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

int highway_half_width(const ProcessedWay &way, double scale)
{
	return highway_block_range(way.tags.get("highway"), way.tags, scale);
}

int layer_extra(const ProcessedWay &way)
{
	int layer = 0;
	if (auto it = way.tags.find("layer"); it != way.tags.end())
		try {
			layer = std::stoi(it->second);
		} catch (...) {
		}
	return std::clamp(-(static_cast<int64_t>(layer) + 1), int64_t{0}, int64_t{8}) *
		   LAYER_DROP;
}

bool pedestrian_way(const ProcessedWay &way)
{
	const auto type = way.tags.get("highway");
	if (type == "footway" || type == "pedestrian" || type == "steps")
		return true;
	if (auto footway = way.tags.find("footway"); footway != way.tags.end())
		return footway->second != "no";
	return false;
}

std::vector<std::pair<int, int>> way_centerline(const ProcessedWay &way)
{
	std::vector<std::pair<int, int>> points;
	for (size_t i = 1; i < way.nodes.size(); ++i)
		for (auto [x, y, z] : bresenham_line(way.nodes[i - 1].x, 0, way.nodes[i - 1].z,
					 way.nodes[i].x, 0, way.nodes[i].z))
			if (points.empty() || points.back() != std::pair{x, z})
				points.emplace_back(x, z);
	return points;
}

size_t way_bresenham_len(const ProcessedWay &way)
{
	size_t length = 1;
	for (size_t i = 1; i < way.nodes.size(); ++i) {
		const auto dx = static_cast<size_t>(
				std::abs(static_cast<int64_t>(way.nodes[i].x) - way.nodes[i - 1].x));
		const auto dz = static_cast<size_t>(
				std::abs(static_cast<int64_t>(way.nodes[i].z) - way.nodes[i - 1].z));
		length += std::max(dx, dz);
	}
	return length;
}

int footprint_min_terrain(
		const WorldEditor &editor, int x, int z, int radius, int fallback)
{
	int minimum = fallback;
	for (int dx = -radius; dx <= radius; ++dx)
		for (int dz = -radius; dz <= radius; ++dz)
			minimum = std::min(minimum, editor.get_ground_level(x + dx, z + dz));
	return minimum;
}

std::vector<PortalFace> portal_faces(const std::vector<std::pair<int, int>> &points,
		const TunnelInternalEndpoints &internal)
{
	std::vector<PortalFace> faces;
	if (points.size() < 2)
		return faces;
	auto face = [&faces](const auto &at, const auto &inner) {
		faces.push_back({at, {(at.first - inner.first > 0) - (at.first - inner.first < 0),
									 (at.second - inner.second > 0) -
											 (at.second - inner.second < 0)}});
	};
	if (!internal.contains(points.front()))
		face(points.front(), points[1]);
	if (!internal.contains(points.back()))
		face(points.back(), points[points.size() - 2]);
	return faces;
}

bool beyond_portal(const std::vector<PortalFace> &faces, int x, int z)
{
	for (const auto &face : faces)
		if ((x - face.at.first) * face.out.first +
						(z - face.at.second) * face.out.second >
				0)
			return true;
	return false;
}

bool tunnel_bore_fits_impl(const WorldEditor &editor, const ProcessedWay &way, double scale)
{
	const auto points = way_centerline(way);
	if (points.size() < 2)
		return false;
	const int wall = highway_half_width(way, scale) + 1;
	return std::any_of(points.begin(), points.end(), [&](const auto &point) {
		const int terrain = editor.get_ground_level(point.first, point.second);
		return footprint_min_terrain(editor, point.first, point.second, wall, terrain) -
					   min_road_y() >=
			   CEIL_OFFSET + 1;
	});
}

}

bool tunnel_bore_fits(const WorldEditor &editor, const ProcessedWay &way, double scale)
{
	return tunnel_bore_fits_impl(editor, way, scale);
}

void TunnelApproach::push(ApproachClaim claim)
{
	for (auto &slot : claims)
		if (!slot) {
			slot = claim;
			return;
		}
}

int TunnelApproach::offset(int distance_from_start) const
{
	int drop = 0;
	for (const auto &claim : claims)
		if (claim)
			drop = std::max(
					drop, claim->drop - std::abs(distance_from_start - claim->anchor) /
												std::max(1, claim->run));
	return -std::max(0, drop);
}

std::optional<TunnelApproach> TunnelPortalMap::approach(std::uint64_t way_id) const
{
	if (auto it = ways_.find(way_id); it != ways_.end())
		return it->second;
	return std::nullopt;
}

int TunnelPortalMap::drop_at(const std::pair<int, int> &point) const
{
	if (auto it = nodes_.find(point); it != nodes_.end())
		return it->second;
	return 0;
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
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox)
{
	// A branch may join another tunnel in its middle, not just at either end.
	// Rust also keeps clipped endpoints underground: the bore continues beyond
	// the generated bbox and must not ramp back to the surface at its edge.
	std::unordered_map<std::pair<int, int>, std::uint64_t, TunnelPointHash> owner;
	TunnelInternalEndpoints shared;
	for (const auto &e : elements)
		if (e.is_way() && renders_as_highway_tunnel(e.as_way())) {
			const auto &w = e.as_way();
			for (const auto &node : w.nodes) {
				const auto point = std::pair{node.x, node.z};
				auto [it, inserted] = owner.emplace(point, w.id);
				if (!inserted && it->second != w.id)
					shared.insert(point);
			}
		}
	TunnelInternalEndpoints result;
	for (const auto &e : elements) {
		if (!e.is_way() || !renders_as_highway_tunnel(e.as_way()))
			continue;
		const auto &w = e.as_way();
		for (const auto *node : {&w.nodes.front(), &w.nodes.back()}) {
			const auto point = std::pair{node->x, node->z};
			const bool at_edge =
					node->x <= xzbbox.min_x() + 1 || node->x >= xzbbox.max_x() - 1 ||
					node->z <= xzbbox.min_z() + 1 || node->z >= xzbbox.max_z() - 1;
			if (at_edge || shared.contains(point))
				result.insert(point);
		}
	}
	return result;
}

TunnelPortalMap collect_tunnel_portals(const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor, const bridges::BridgeStructureMap &bridges,
		const TunnelInternalEndpoints &internal, double scale)
{
	std::vector<const ProcessedWay *> tunnels;
	std::vector<const ProcessedWay *> surface;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (!way.tags.contains("highway") || way.nodes.size() < 2)
			continue;
		if (renders_as_highway_tunnel(way)) {
			if (tunnel_bore_fits_impl(editor, way, scale))
				tunnels.push_back(&way);
		} else {
			surface.push_back(&way);
		}
	}
	TunnelPortalMap portals;
	if (tunnels.empty())
		return portals;

	TunnelInternalEndpoints portal_nodes;
	for (const auto *way : tunnels)
		for (const auto *node : {&way->nodes.front(), &way->nodes.back()}) {
			const std::pair point{node->x, node->z};
			if (!internal.contains(point))
				portal_nodes.insert(point);
		}

	std::unordered_map<std::pair<int, int>, std::vector<size_t>, TunnelPointHash> ends;
	for (size_t i = 0; i < surface.size(); ++i)
		for (const auto *node : {&surface[i]->nodes.front(), &surface[i]->nodes.back()})
			ends[{node->x, node->z}].push_back(i);

	for (const auto *tunnel : tunnels) {
		const int wanted = COVER_DROP + layer_extra(*tunnel);
		const bool tunnel_pedestrian = pedestrian_way(*tunnel);
		for (const auto *node : {&tunnel->nodes.front(), &tunnel->nodes.back()}) {
			const std::pair portal{node->x, node->z};
			if (internal.contains(portal) || portals.nodes_.contains(portal))
				continue;
			const int headroom =
					editor.get_ground_level(portal.first, portal.second) - min_road_y();
			const int wanted_here = std::min(wanted, headroom);
			if (wanted_here <= 0)
				continue;

			const int budget = wanted_here * RAMP_RUN;
			std::unordered_set<std::uint64_t> taken;
			using ChainCell = std::tuple<size_t, bool, int>;
			std::vector<std::vector<ChainCell>> branches;
			auto eligible_at = [&](const std::pair<int, int> &at) {
				std::vector<size_t> eligible;
				auto found = ends.find(at);
				if (found == ends.end())
					return eligible;
				for (const size_t index : found->second) {
					const auto &candidate = *surface[index];
					if (taken.contains(candidate.id) ||
							bridges::is_bridge_way(candidate) ||
							bridges.lookup_member(candidate.id) ||
							bridges.lookup_ramp(candidate.id) ||
							pedestrian_way(candidate) != tunnel_pedestrian)
						continue;
					if (auto claimed = portals.ways_.find(candidate.id);
							claimed != portals.ways_.end() && claimed->second.full())
						continue;
					eligible.push_back(index);
				}
				return eligible;
			};

			for (const size_t first : eligible_at(portal)) {
				if (taken.contains(surface[first]->id))
					continue;
				std::vector<ChainCell> chain;
				size_t index = first;
				auto at = portal;
				int distance = 0;
				while (true) {
					const auto &candidate = *surface[index];
					const bool at_start = std::pair{candidate.nodes.front().x,
												  candidate.nodes.front().z} == at;
					chain.emplace_back(index, at_start, distance);
					taken.insert(candidate.id);
					distance += static_cast<int>(way_bresenham_len(candidate) - 1);
					const auto &far =
							at_start ? candidate.nodes.back() : candidate.nodes.front();
					at = {far.x, far.z};
					if (distance >= budget || chain.size() >= 8 ||
							portal_nodes.contains(at))
						break;
					const auto next = eligible_at(at);
					if (next.size() != 1)
						break;
					index = next.front();
				}
				branches.push_back(std::move(chain));
			}

			if (branches.empty())
				continue;
			int available = std::numeric_limits<int>::max();
			for (const auto &branch : branches) {
				if (branch.empty()) {
					available = 0;
					break;
				}
				const auto [index, at_start, distance] = branch.back();
				(void)at_start;
				available = std::min(available,
						distance +
								static_cast<int>(way_bresenham_len(*surface[index]) - 1));
			}
			const int drop = std::min(wanted_here, available);
			if (drop <= 0)
				continue;
			const int run = std::clamp(available / drop, 1, RAMP_RUN);
			for (const auto &branch : branches)
				for (const auto &[index, at_start, distance] : branch) {
					const auto &candidate = *surface[index];
					const int last = static_cast<int>(way_bresenham_len(candidate) - 1);
					const int anchor = at_start ? -distance : last + distance;
					portals.ways_[candidate.id].push({anchor, drop, run});
				}
			portals.nodes_[portal] = drop;
		}
	}
	return portals;
}

CoordinateBitmap collect_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor, const TunnelInternalEndpoints &internal,
		const XZBBox &xzbbox, double scale)
{
	CoordinateBitmap result(xzbbox);
	for (const auto &e : elements) {
		if (!e.is_way() || !renders_as_highway_tunnel(e.as_way()))
			continue;
		const auto &way = e.as_way();
		const int wall = highway_half_width(way, scale) + 1;
		const auto points = way_centerline(way);
		if (!tunnel_bore_fits_impl(editor, way, scale))
			continue;
		const auto faces = portal_faces(points, internal);
		for (const auto &[x, z] : points)
			for (int dx = -wall; dx <= wall; ++dx)
				for (int dz = -wall; dz <= wall; ++dz)
					if (!beyond_portal(faces, x + dx, z + dz))
						result.set(x + dx, z + dz);
	}
	return result;
}

bool generate_highway_tunnel_shell(WorldEditor &editor, const ProcessedWay &way,
		const Args &args, const TunnelInternalEndpoints &internal,
		const TunnelPortalMap &portals, std::vector<HighwayTunnelCell> &cells)
{
	if (!renders_as_highway_tunnel(way))
		return false;
	const auto pts = way_centerline(way);
	if (pts.size() < 2)
		return false;
	std::vector<int> terrain;
	for (auto [x, z] : pts)
		terrain.push_back(editor.get_ground_level(x, z));
	const size_t last = pts.size() - 1;
	const bool start_internal = internal.contains(pts.front()),
			   end_internal = internal.contains(pts.back());
	const int extra = layer_extra(way);
	std::string type = way.tags.get("highway");
	if (type.empty())
		type = "road";
	const int hw = highway_half_width(way, args.scale), wall = hw + 1;
	std::vector<int> cover;
	cover.reserve(pts.size());
	for (size_t i = 0; i < pts.size(); ++i)
		cover.push_back(footprint_min_terrain(
				editor, pts[i].first, pts[i].second, wall, terrain[i]));
	if (std::none_of(cover.begin(), cover.end(),
				[](int y) { return y - min_road_y() >= CEIL_OFFSET + 1; }))
		return false;
	std::vector<int> road(pts.size());
	const int start_drop = start_internal ? 0 : portals.drop_at(pts.front());
	const int end_drop = end_internal ? 0 : portals.drop_at(pts.back());
	for (size_t i = 0; i < pts.size(); ++i) {
		const double t = static_cast<double>(i) / last;
		const int grade =
				std::lround(terrain.front() + (terrain.back() - terrain.front()) * t);
		const int desired = std::min(grade, cover[i] - COVER_DROP) - extra;
		const int rs = start_internal ? INT_MIN
									  : terrain.front() - start_drop -
												static_cast<int>(i) / RAMP_RUN;
		const int re = end_internal ? INT_MIN
									: terrain.back() - end_drop -
											  static_cast<int>(last - i) / RAMP_RUN;
		road[i] = std::min(terrain[i], std::max(desired, std::max(rs, re)));
	}
	for (size_t i = 1; i < road.size(); ++i)
		road[i] = std::min(road[i], road[i - 1] + 1);
	for (size_t i = last; i-- > 0;)
		road[i] = std::min(road[i], road[i + 1] + 1);
	// Rust's grade solver keeps one continuous valley rather than allowing
	// several DEM-induced humps inside the bore.
	const auto valley = std::min_element(road.begin(), road.end());
	for (auto it = road.begin() + 1; it <= valley; ++it)
		*it = std::min(*it, *(it - 1));
	for (auto it = valley; it + 1 != road.end(); ++it)
		*it = std::min(*it, *(it + 1));
	for (auto &y : road)
		y = std::max(y, min_road_y());

	// Close short uncovered runs within a bore.  End runs are portals, but a
	// brief interior DEM dip should deepen the road instead of cutting a hole
	// through the roof.
	std::vector<bool> covered(road.size());
	for (size_t i = 0; i < road.size(); ++i)
		covered[i] = cover[i] - road[i] >= CEIL_OFFSET + 1;
	const size_t close_run = static_cast<size_t>(2 * hw + 3);
	bool deepened = false;
	for (size_t begin = 0; begin < covered.size();) {
		if (covered[begin]) {
			++begin;
			continue;
		}
		size_t end = begin;
		while (end < covered.size() && !covered[end])
			++end;
		if (begin != 0 && end != covered.size() && end - begin < close_run) {
			for (size_t j = begin; j < end; ++j) {
				const int target = std::max(min_road_y(), cover[j] - CEIL_OFFSET - 1);
				if (target < road[j]) {
					road[j] = target;
					deepened = true;
				}
			}
		}
		begin = end;
	}
	if (deepened) {
		for (size_t i = 1; i < road.size(); ++i)
			road[i] = std::min(road[i], road[i - 1] + 1);
		for (size_t i = last; i-- > 0;)
			road[i] = std::min(road[i], road[i + 1] + 1);
		for (auto &y : road)
			y = std::max(y, min_road_y());
		for (size_t i = 0; i < road.size(); ++i)
			covered[i] = cover[i] - road[i] >= CEIL_OFFSET + 1;
	}
	const auto faces = portal_faces(pts, internal);
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
		const bool cell_covered = covered[i];
		const int top = cell_covered ? ry + CEIL_OFFSET : ty;
		for (int dx = -wall; dx <= wall; ++dx) {
			for (int dz = -wall; dz <= wall; ++dz) {
				if (beyond_portal(faces, x + dx, z + dz))
					continue;
				const int column_top =
						std::min(top, editor.get_ground_level(x + dx, z + dz) - 1);
				for (int y = ry - 1; y <= std::max(ry - 1, column_top); ++y) {
					const bool edge = std::abs(dx) == wall || std::abs(dz) == wall ||
									  (cell_covered && y == ry + CEIL_OFFSET);
					editor.set_block_absolute(
							edge ? shell_block(x + dx, y, z + dz) : STONE_BRICKS, x + dx,
							y, z + dz);
				}
			}
		}
		cells.push_back(HighwayTunnelCell{.x = x,
				.z = z,
				.road_y = ry,
				.half_width = hw,
				.terrain_y = ty,
				.covered = cell_covered,
				.light = cell_covered && i % 8 == 0,
				.palette = palette,
				.faces = faces});
	}
	return true;
}

void carve_highway_tunnel_interior(
		WorldEditor &editor, const std::vector<HighwayTunnelCell> &cells)
{
	const std::vector<Block> carve{STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS,
			STONE, WATER, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA, GRAY_CONCRETE,
			BLACK_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, GRASS_BLOCK, DIRT,
			COARSE_DIRT, PODZOL, MUD, CLAY, SAND, SANDSTONE, GRAVEL, ANDESITE,
			COBBLESTONE, TUFF, DEEPSLATE, SNOW_BLOCK, SNOW_LAYER, MOSS_BLOCK, FARMLAND};
	const std::vector<Block> road_wl{AIR, STONE, STONE_BRICKS, CRACKED_STONE_BRICKS,
			MOSSY_STONE_BRICKS, WATER, SEA_LANTERN};
	for (const auto &c : cells) {
		const int top = c.covered ? c.road_y + CEIL_OFFSET - 1 : c.terrain_y;
		for (int dx = -c.half_width; dx <= c.half_width; ++dx)
			for (int dz = -c.half_width; dz <= c.half_width; ++dz)
				if (!beyond_portal(c.faces, c.x + dx, c.z + dz))
					for (int y = c.road_y + 1; y <= top; ++y)
						editor.set_block_absolute(
								AIR, c.x + dx, y, c.z + dz, carve, std::nullopt);
		if (c.light)
			editor.set_block_absolute(SEA_LANTERN, c.x, c.road_y + CEIL_OFFSET - 1, c.z);
	}
	for (const auto &c : cells)
		for (int dx = -c.half_width; dx <= c.half_width; ++dx)
			for (int dz = -c.half_width; dz <= c.half_width; ++dz)
				if (!beyond_portal(c.faces, c.x + dx, c.z + dz))
					editor.set_block_absolute(
							surfaces::semirandom_surface(c.x + dx, c.z + dz, c.palette),
							c.x + dx, c.road_y, c.z + dz, road_wl, std::nullopt);
}
}
