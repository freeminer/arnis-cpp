#include "road_markings.h"
#include "../clipping.h"
#include "../bresenham.h"
#include "../strict_parse.h"
#include "surfaces.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace arnis::highways
{
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
std::vector<Block> surface_palette(const std::string &highway_type, const tags_t &tags);
}

namespace arnis::road_markings
{
namespace
{
struct Road
{
	const ProcessedWay *way;
	int rank;
	bool link;
	bool ring;
	int oneway;
	bool marked;
	int half_width;
	std::vector<Block> surface;
	std::vector<std::uint32_t> node_t;
};

struct ArmCell
{
	int x;
	int z;
	std::size_t road;
	std::uint32_t t;
	int dir;
};

struct RoadArm
{
	std::size_t road;
	std::size_t node;
	int dir;
};

enum class Control
{
	Signal,
	Stop,
	GiveWay
};

struct JunctionArm
{
	std::size_t road;
	std::size_t node;
	int dir;
	std::vector<ArmCell> cells;
	std::pair<float, float> unit;
	std::optional<Control> control;
};

struct LocalGrid
{
	int x0;
	int z0;
	int size;
	std::vector<std::uint16_t> bits;

	LocalGrid(int cx, int cz, int radius) :
			x0(cx - radius), z0(cz - radius), size(2 * radius + 1),
			bits(static_cast<std::size_t>(size * size), 0)
	{
	}

	void stamp(int x, int z, int radius, std::uint16_t bit)
	{
		const int lx0 = std::max(0, x - radius - x0);
		const int lx1 = std::min(size - 1, x + radius - x0);
		const int lz0 = std::max(0, z - radius - z0);
		const int lz1 = std::min(size - 1, z + radius - z0);
		for (int lz = lz0; lz <= lz1; ++lz)
			for (int lx = lx0; lx <= lx1; ++lx)
				bits[static_cast<std::size_t>(lz * size + lx)] |= bit;
	}

	std::uint16_t get(int x, int z) const
	{
		const int lx = x - x0;
		const int lz = z - z0;
		if (lx < 0 || lz < 0 || lx >= size || lz >= size)
			return 0;
		return bits[static_cast<std::size_t>(lz * size + lx)];
	}
};

bool is_road_class(const std::string &highway);

std::pair<int, bool> road_class(const std::string &highway)
{
	if (highway == "motorway" || highway == "trunk")
		return {5, false};
	if (highway == "motorway_link" || highway == "trunk_link")
		return {4, true};
	if (highway == "primary")
		return {4, false};
	if (highway == "primary_link")
		return {3, true};
	if (highway == "secondary")
		return {3, false};
	if (highway == "secondary_link")
		return {2, true};
	if (highway == "tertiary")
		return {2, false};
	if (highway == "tertiary_link")
		return {1, true};
	if (highway == "service" || highway == "track")
		return {0, false};
	if (is_road_class(highway))
		return {1, false};
	return {-1, false};
}

bool is_roundabout(const tags_t &tags)
{
	return tags.get("junction") == "roundabout" || tags.get("junction") == "circular";
}

int oneway_sign(const std::string &highway, const tags_t &tags)
{
	const auto value = tags.get("oneway");
	if (value == "yes" || value == "true" || value == "1")
		return 1;
	if (value == "-1" || value == "reverse")
		return -1;
	if (value == "no" || value == "false" || value == "0" || value == "reversible" ||
			value == "alternating")
		return 0;
	return is_roundabout(tags) || highway == "motorway" || highway == "motorway_link" ? 1
																					  : 0;
}

bool is_signal(const ProcessedNode &node)
{
	return node.tags.get("highway") == "traffic_signals";
}

bool renders_on_surface(const tags_t &tags)
{
	if (tags.get("area") == "yes" || tags.get("indoor") == "yes")
		return false;
	const auto level = strict_parse::i32(tags.get("level"));
	return !level || *level >= 0;
}

bool is_road_class(const std::string &highway)
{
	static const std::unordered_set<std::string> classes = {"motorway", "trunk",
			"motorway_link", "trunk_link", "primary", "primary_link", "secondary",
			"secondary_link", "tertiary", "tertiary_link", "unclassified", "residential",
			"living_street", "road", "busway", "service", "track"};
	return classes.contains(highway);
}

bool has_lane_paint(const std::string &highway, const tags_t &tags)
{
	const auto paint = tags.get("lane_markings");
	if (paint == "no")
		return false;
	const auto parsed_lanes = strict_parse::i32(tags.get("lanes"));
	int lanes = 1;
	if (parsed_lanes) {
		lanes = std::clamp(*parsed_lanes, 1, 16);
	} else {
		const auto junction = tags.get("junction");
		if (junction == "roundabout" || junction == "circular")
			lanes = 1;
		else if (highway == "motorway" || highway == "primary" || highway == "trunk" ||
				 highway == "secondary" || highway == "tertiary")
			lanes = 2;
		else
			lanes = 1;
	}
	const bool minor = highway == "residential" || highway == "living_street" ||
					   highway == "service" || highway == "track";
	return !(minor && lanes < 3 && paint != "yes") && lanes >= 2;
}

bool signalled_crossing(const tags_t &tags)
{
	return tags.get("crossing") == "traffic_signals" ||
		   tags.get("crossing:signals") == "yes";
}

std::optional<CrossingPaint> crossing_paint(
		const tags_t &tags, std::optional<CrossingPaint> untagged, bool signal_lines)
{
	const auto crossing = tags.get("crossing");
	if (crossing == "no" || crossing == "unmarked" || crossing == "informal" ||
			crossing == "impossible")
		return std::nullopt;
	const auto markings = tags.get("crossing:markings");
	if (markings == "no" || markings == "surface")
		return std::nullopt;
	if (markings == "lines" || markings == "lines:paired")
		return CrossingPaint{CrossingPaint::Kind::Lines, false};
	if (markings == "dashes" || markings == "dashes:paired" || markings == "dots")
		return CrossingPaint{CrossingPaint::Kind::Lines, true};
	if (!markings.empty())
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	if (tags.get("crossing_ref") == "zebra" || crossing == "zebra")
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	if (signalled_crossing(tags))
		return signal_lines ? CrossingPaint{CrossingPaint::Kind::Lines, true}
							: CrossingPaint{CrossingPaint::Kind::Zebra, false};
	if (!crossing.empty())
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	return untagged;
}

std::vector<std::uint32_t> node_path_index(const ProcessedWay &way)
{
	std::vector<std::uint32_t> out;
	out.reserve(way.nodes.size());
	std::uint32_t t = 0;
	for (std::size_t i = 0; i < way.nodes.size(); ++i) {
		if (i) {
			const auto dx = std::llabs(
					static_cast<long long>(way.nodes[i].x) - way.nodes[i - 1].x);
			const auto dz = std::llabs(
					static_cast<long long>(way.nodes[i].z) - way.nodes[i - 1].z);
			t += static_cast<std::uint32_t>(std::max(dx, dz));
		}
		out.push_back(t);
	}
	return out;
}

std::vector<RoadArm> road_arms_at(const std::vector<Road> &roads,
		const std::vector<std::pair<std::size_t, std::size_t>> &refs)
{
	std::vector<RoadArm> arms;
	for (const auto &[ri, ni] : refs) {
		const auto t = roads[ri].node_t[ni];
		const auto last = roads[ri].node_t.empty() ? 0 : roads[ri].node_t.back();
		if (t < last)
			arms.push_back({ri, ni, 1});
		if (t > 0)
			arms.push_back({ri, ni, -1});
	}
	return arms;
}

std::vector<std::pair<int, int>> way_cells(const ProcessedWay &way)
{
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto points = bresenham::bresenham_line(way.nodes[i - 1].x, 0,
				way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z);
		const std::size_t begin = cells.empty() ? 0 : 1;
		for (std::size_t k = begin; k < points.size(); ++k)
			cells.emplace_back(std::get<0>(points[k]), std::get<2>(points[k]));
	}
	return cells;
}

std::vector<ArmCell> arm_cells(const std::vector<Road> &roads,
		const std::unordered_map<std::uint64_t,
				std::vector<std::pair<std::size_t, std::size_t>>> &joins,
		std::size_t ri, std::size_t ni, int dir, std::size_t limit)
{
	std::vector<ArmCell> cells;
	if (ni >= roads[ri].way->nodes.size() || limit == 0)
		return cells;
	cells.push_back({roads[ri].way->nodes[ni].x, roads[ri].way->nodes[ni].z, ri,
			roads[ri].node_t[ni], dir});
	std::size_t k = ni;
	std::size_t hops = 0;
	auto carry_on = [&]() {
		const auto at_node = joins.find(roads[ri].way->nodes[k].id);
		if (at_node == joins.end() || hops >= 8)
			return false;
		const auto arms = road_arms_at(roads, at_node->second);
		if (arms.size() != 2)
			return false;
		const auto next = std::find_if(arms.begin(), arms.end(),
				[&](const RoadArm &arm) { return arm.road != ri || arm.node != k; });
		if (next == arms.end())
			return false;
		ri = next->road;
		k = next->node;
		dir = next->dir;
		++hops;
		return true;
	};
	while (cells.size() < limit) {
		if (dir > 0) {
			if (k + 1 >= roads[ri].way->nodes.size()) {
				if (carry_on())
					continue;
				break;
			}
			const auto &a = roads[ri].way->nodes[k];
			const auto &b = roads[ri].way->nodes[k + 1];
			const auto points = bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
			for (std::size_t p = 1; p < points.size() && cells.size() < limit; ++p)
				cells.push_back({std::get<0>(points[p]), std::get<2>(points[p]), ri,
						roads[ri].node_t[k] + static_cast<std::uint32_t>(p), dir});
			++k;
		} else {
			if (k == 0) {
				if (carry_on())
					continue;
				break;
			}
			const auto &a = roads[ri].way->nodes[k - 1];
			const auto &b = roads[ri].way->nodes[k];
			const auto points = bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
			for (std::size_t p = points.size(); p-- > 1 && cells.size() < limit;)
				cells.push_back({std::get<0>(points[p - 1]), std::get<2>(points[p - 1]),
						ri, roads[ri].node_t[k] - static_cast<std::uint32_t>(p - 1),
						dir});
			--k;
		}
	}
	return cells;
}

std::optional<std::pair<float, float>> normalized(float x, float z)
{
	const float length = std::sqrt(x * x + z * z);
	if (length <= 1e-3f)
		return std::nullopt;
	return std::pair<float, float>{x / length, z / length};
}

std::optional<std::pair<float, float>> arm_unit(const std::vector<ArmCell> &cells)
{
	if (cells.size() < 2)
		return std::nullopt;
	const auto &a = cells.front();
	const auto &b = cells[std::min<std::size_t>(6, cells.size() - 1)];
	return normalized(static_cast<float>(b.x - a.x), static_cast<float>(b.z - a.z));
}

std::pair<float, float> local_direction(const std::vector<ArmCell> &cells, std::size_t at,
		std::pair<float, float> fallback)
{
	const auto &a = cells[at > 1 ? at - 2 : 0];
	const auto &b = cells[std::min(at + 2, cells.size() - 1)];
	return normalized(static_cast<float>(b.x - a.x), static_cast<float>(b.z - a.z))
			.value_or(fallback);
}

std::optional<std::size_t> node_index_at(
		const std::vector<Road> &roads, const ArmCell &cell)
{
	const auto &ts = roads[cell.road].node_t;
	const auto it = std::lower_bound(ts.begin(), ts.end(), cell.t);
	if (it == ts.end() || *it != cell.t)
		return std::nullopt;
	return static_cast<std::size_t>(it - ts.begin());
}

const ProcessedNode *node_at(const std::vector<Road> &roads, const ArmCell &cell)
{
	const auto index = node_index_at(roads, cell);
	return index ? &roads[cell.road].way->nodes[*index] : nullptr;
}

int chebyshev_distance_to(const std::vector<std::pair<int, int>> &cells, int x, int z)
{
	int distance = std::numeric_limits<int>::max();
	for (const auto &[cx, cz] : cells)
		distance = std::min(distance, std::max(std::abs(x - cx), std::abs(z - cz)));
	return distance;
}
} // namespace

bool WayMarks::in_gap(std::uint32_t t) const
{
	for (const auto &[a, b] : gaps)
		if (a <= t && t <= b)
			return true;
	return false;
}

std::int64_t WayMarks::dash_phase(std::uint32_t t) const
{
	return phase_step == 0 ? static_cast<std::int64_t>(t)
						   : phase + static_cast<std::int64_t>(phase_step) * t;
}

bool CarriagewayClip::contains(int x, int z) const
{
	return std::any_of(centreline.begin(), centreline.end(), [&](const auto &cell) {
		const auto dx = static_cast<std::int64_t>(x) - cell.first;
		const auto dz = static_cast<std::int64_t>(z) - cell.second;
		return std::abs(dx) <= half_width && std::abs(dz) <= half_width;
	});
}

RoadMarkingIndex RoadMarkingIndex::build(const std::vector<ProcessedElement> &elements,
		double scale, decals::SignRegion region)
{
	RoadMarkingIndex result;
	result.drives_on_left = decals::drives_on_left(region);
	result.yellow_centre = region == decals::SignRegion::NorthAmerica ||
						   region == decals::SignRegion::Canada;
	std::uint32_t lines = 0, zebras = 0;
	for (const auto &element : elements) {
		const auto &tags = element.tags();
		const bool crossing =
				tags.get("highway") == "crossing" || tags.get("footway") == "crossing";
		const bool signalled = signalled_crossing(tags);
		if (!crossing || !signalled)
			continue;
		const auto markings = tags.get("crossing:markings");
		if (markings == "lines" || markings == "lines:paired" || markings == "dashes" ||
				markings == "dashes:paired" || markings == "dots")
			++lines;
		else if (markings.starts_with("zebra") || markings.starts_with("ladder"))
			++zebras;
	}
	result.signal_crossing_lines =
			lines + zebras > 0 ? lines > zebras
							   : (region == decals::SignRegion::Germanic ||
										 region == decals::SignRegion::UkIreland);

	// Dash phase is carried over two-way continuations and same-level links,
	// rather than restarting at each OSM way boundary.
	std::vector<Road> roads;
	std::vector<const ProcessedWay *> crossing_ways;
	std::unordered_map<std::uint64_t, std::vector<std::pair<std::size_t, std::size_t>>>
			node_uses;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway = way.tags.get("highway");
		if (way.nodes.size() < 2 || !renders_on_surface(way.tags))
			continue;
		if (!is_road_class(highway)) {
			if (crossing_paint(way.tags, CrossingPaint{CrossingPaint::Kind::Zebra, false},
						result.signal_crossing_lines) &&
					highway == "footway" && way.tags.get("footway") == "crossing")
				crossing_ways.push_back(&way);
			continue;
		}
		const auto ri = roads.size();
		const auto range = arnis::highways::highway_block_range(highway, way.tags, scale);
		auto surface = arnis::highways::surface_palette(highway, way.tags);
		if (const auto cycleway = surfaces::cycleway_palette(way))
			surface = *cycleway;
		const auto [rank, link] = road_class(highway);
		roads.push_back({&way, rank, link, is_roundabout(way.tags),
				oneway_sign(highway, way.tags), has_lane_paint(highway, way.tags), range,
				std::move(surface), node_path_index(way)});
		for (std::size_t ni = 0; ni < way.nodes.size(); ++ni) {
			if (!clipping::is_invented_node_id(way.nodes[ni].id))
				node_uses[way.nodes[ni].id].emplace_back(ri, ni);
		}
	}
	auto add_gap = [&](const std::vector<ArmCell> &cells, std::size_t end) {
		end = std::min(end, cells.size());
		std::size_t run_start = 0;
		for (std::size_t s = 1; s <= end; ++s) {
			if (s < end && cells[s].road == cells[run_start].road)
				continue;
			const auto &first = cells[run_start];
			const auto &road = roads[first.road];
			if (road.marked && s > run_start) {
				const auto &last = cells[s - 1];
				std::int64_t start = first.t;
				if (run_start > 0)
					start = std::max<std::int64_t>(
							0, static_cast<std::int64_t>(first.t) - first.dir);
				result.ways_[road.way->id].gaps.emplace_back(
						static_cast<std::uint32_t>(std::min<std::int64_t>(start, last.t)),
						static_cast<std::uint32_t>(
								std::max<std::int64_t>(start, last.t)));
			}
			run_start = s;
		}
	};
	std::unordered_map<std::uint64_t, std::uint32_t> crossing_reach;
	std::vector<std::tuple<std::size_t, std::size_t, std::uint64_t>> signal_crossings;
	std::unordered_set<std::uint64_t> crossing_way_nodes;

	// Find the carriageway footprints intersected by each painted crossing way.
	// The clip bounds both its surface replacement and the break in lane paint.
	for (const auto *crossing : crossing_ways) {
		const auto paint = crossing_paint(crossing->tags,
				CrossingPaint{CrossingPaint::Kind::Zebra, false},
				result.signal_crossing_lines);
		if (!paint)
			continue;
		const int crossing_half =
				arnis::highways::highway_block_range("footway", crossing->tags, scale) +
				(paint->kind == CrossingPaint::Kind::Lines ? 1 : 0);
		const auto crossing_cells = way_cells(*crossing);
		if (crossing_cells.empty())
			continue;
		for (const auto &crossing_node : crossing->nodes) {
			crossing_way_nodes.insert(crossing_node.id);
			const auto refs = node_uses.find(crossing_node.id);
			if (refs == node_uses.end())
				continue;
			for (const auto &[ri, ni] : refs->second) {
				const auto &road = roads[ri];
				const std::size_t limit =
						crossing_cells.size() +
						static_cast<std::size_t>(road.half_width + crossing_half + 2);
				const std::array<std::vector<ArmCell>, 2> sides{
						arm_cells(roads, node_uses, ri, ni, 1, limit),
						arm_cells(roads, node_uses, ri, ni, -1, limit)};
				CarriagewayClip clip;
				clip.half_width = road.half_width;
				clip.surface = road.surface;
				for (const auto &side : sides) {
					for (const auto &cell : side) {
						if (chebyshev_distance_to(crossing_cells, cell.x, cell.z) <=
								road.half_width + crossing_half)
							clip.centreline.emplace_back(cell.x, cell.z);
					}
				}
				if (clip.centreline.empty())
					continue;
				result.crossings_[crossing->id].push_back(clip);

				std::unordered_set<std::uint64_t> painted;
				for (const auto &[cx, cz] : crossing_cells) {
					for (int dx = -crossing_half; dx <= crossing_half; ++dx) {
						for (int dz = -crossing_half; dz <= crossing_half; ++dz) {
							const int x = cx + dx, z = cz + dz;
							if (!clip.contains(x, z))
								continue;
							const auto key = (static_cast<std::uint64_t>(
													  static_cast<std::uint32_t>(x))
													 << 32) |
											 static_cast<std::uint32_t>(z);
							painted.insert(key);
						}
					}
				}
				auto near_paint = [&](int x, int z) {
					for (int dx = -1; dx <= 1; ++dx) {
						for (int dz = -1; dz <= 1; ++dz) {
							const auto key = (static_cast<std::uint64_t>(
													  static_cast<std::uint32_t>(x + dx))
													 << 32) |
											 static_cast<std::uint32_t>(z + dz);
							if (painted.contains(key))
								return true;
						}
					}
					return false;
				};
				for (const auto &side : sides) {
					std::optional<std::size_t> last;
					for (std::size_t k = 0; k < side.size(); ++k) {
						const auto [ux, uz] = local_direction(side, k, {1.0f, 0.0f});
						const float px = -uz, pz = ux;
						const int width = static_cast<int>(std::lround(
								road.half_width * (std::abs(px) + std::abs(pz))));
						bool touches = false;
						for (int p = -width; p <= width; ++p) {
							const int x =
									static_cast<int>(std::lround(side[k].x + px * p));
							const int z =
									static_cast<int>(std::lround(side[k].z + pz * p));
							if (near_paint(x, z)) {
								touches = true;
								break;
							}
						}
						if (touches)
							last = k;
					}
					if (last) {
						add_gap(side, *last + 1);
						auto &reach = crossing_reach[crossing_node.id];
						reach = std::max(reach, static_cast<std::uint32_t>(*last));
					}
				}
				if (signalled_crossing(road.way->nodes[ni].tags) ||
						signalled_crossing(crossing->tags)) {
					signal_crossings.emplace_back(ri, ni, crossing_node.id);
				}
			}
		}
	}

	// Node-mapped pedestrian crossings are rendered as transverse paint on every
	// carriageway way sharing the node, including both halves of a split way.
	// Match Rust's source of truth: crossing tags are read from each processed
	// road's embedded nodes, which remain available even when the parser does not
	// emit standalone ProcessedNode elements for them.
	std::vector<std::pair<std::uint64_t, const ProcessedNode *>> road_nodes;
	std::unordered_set<std::uint64_t> seen_road_nodes;
	for (const auto &road : roads)
		for (const auto &node : road.way->nodes)
			if (seen_road_nodes.insert(node.id).second)
				road_nodes.emplace_back(node.id, &node);
	for (const auto &[node_id, node_ptr] : road_nodes) {
		const auto &node = *node_ptr;
		const auto highway = node.tags.get("highway");
		if (highway != "crossing" &&
				!(highway == "traffic_signals" && !node.tags.get("crossing").empty()))
			continue;
		const auto paint =
				crossing_paint(node.tags, std::nullopt, result.signal_crossing_lines);
		if (!paint || crossing_reach.contains(node.id))
			continue;
		const auto refs = node_uses.find(node.id);
		if (refs == node_uses.end())
			continue;
		if (road_arms_at(roads, refs->second).size() > 2)
			continue;
		const std::size_t near_crossing =
				static_cast<std::size_t>(std::max(2.0, 8.0 * scale));
		if (is_signal(node)) {
			bool beside_crossing = false;
			for (const auto &[ri, ni] : refs->second) {
				for (const int dir : {1, -1}) {
					const auto cells =
							arm_cells(roads, node_uses, ri, ni, dir, near_crossing + 1);
					for (std::size_t s = 1; s < cells.size(); ++s) {
						const auto *candidate = node_at(roads, cells[s]);
						if (candidate &&
								(crossing_way_nodes.contains(candidate->id) ||
										candidate->tags.get("highway") == "crossing")) {
							beside_crossing = true;
							break;
						}
					}
					if (beside_crossing)
						break;
				}
				if (beside_crossing)
					break;
			}
			if (beside_crossing)
				continue;
		}
		const std::uint32_t reach = paint->kind == CrossingPaint::Kind::Zebra ? 2 : 3;
		crossing_reach[node_id] = std::max(crossing_reach[node_id], reach);
		for (const auto &[ri, ni] : refs->second) {
			for (const int dir : {1, -1}) {
				const auto cells = arm_cells(roads, node_uses, ri, ni, dir, reach + 1);
				add_gap(cells, cells.size());
			}
			const auto ahead = arm_cells(roads, node_uses, ri, ni, 1, 3);
			const auto behind = arm_cells(roads, node_uses, ri, ni, -1, 3);
			if (paint->kind == CrossingPaint::Kind::Zebra) {
				for (const int row : {-1, 0, 1}) {
					const auto &line = row >= 0 ? ahead : behind;
					const auto index = static_cast<std::size_t>(std::abs(row));
					if (index < line.size()) {
						const auto &cell = line[index];
						result.ways_[roads[cell.road].way->id].marks.push_back(
								{cell.t, TransverseMark::Kind::Zebra, false, 0});
					}
				}
			} else {
				for (const int row : {-2, 2}) {
					const auto &line = row >= 0 ? ahead : behind;
					const auto index = static_cast<std::size_t>(std::abs(row));
					if (index < line.size()) {
						const auto &cell = line[index];
						result.ways_[roads[cell.road].way->id].marks.push_back({cell.t,
								TransverseMark::Kind::CrossingLines, paint->broken, 0});
					}
				}
			}
			if (signalled_crossing(node.tags))
				signal_crossings.emplace_back(ri, ni, node_id);
		}
	}

	// Resolve controls and priority at shared street nodes. This mirrors the Rust
	// arm-based decision: continuous/higher-class roads keep their lane paint;
	// incoming lower-priority arms receive stop or give-way bars.
	constexpr std::size_t max_arms = 16;
	const auto sign_reach = static_cast<std::size_t>(std::max(4.0, 30.0 * scale));
	const auto min_run = static_cast<std::size_t>(std::max(3.0, 8.0 * scale));
	std::vector<std::tuple<std::size_t, std::uint32_t, std::size_t, std::uint32_t>>
			junction_links;
	std::unordered_set<std::uint64_t> junction_ids;
	std::unordered_set<std::uint64_t> signalised_ids;
	for (const auto &[node_id, refs] : node_uses) {
		const auto dirs = road_arms_at(roads, refs);
		const auto streets = std::count_if(dirs.begin(), dirs.end(),
				[&](const auto &arm) { return roads[arm.road].rank > 0; });
		if (streets > 2)
			junction_ids.insert(node_id);
	}
	for (const auto &[node_id, refs] : node_uses) {
		const auto directions = road_arms_at(roads, refs);
		if (directions.size() < 3 || directions.size() > max_arms)
			continue;
		const auto major_arms = std::count_if(directions.begin(), directions.end(),
				[&](const auto &arm) { return roads[arm.road].rank > 0; });
		const bool is_junction = junction_ids.contains(node_id) && major_arms > 2;

		int max_half = 1;
		for (const auto &arm : directions)
			max_half = std::max(max_half, roads[arm.road].half_width);
		const std::size_t near = sign_reach + 2 * static_cast<std::size_t>(max_half);
		const std::size_t cap = static_cast<std::size_t>(2 * (2 * max_half + 1) + 8);
		const std::size_t reach =
				std::max(cap + 3 * static_cast<std::size_t>(max_half) + 4,
						sign_reach + 2 * static_cast<std::size_t>(max_half) + 1);
		std::vector<JunctionArm> arms;
		arms.reserve(directions.size());
		for (const auto &direction : directions) {
			auto cells = arm_cells(roads, node_uses, direction.road, direction.node,
					direction.dir, reach);
			if (cells.size() < 2)
				continue;
			const auto unit = arm_unit(cells);
			if (!unit)
				continue;
			JunctionArm arm{direction.road, direction.node, direction.dir,
					std::move(cells), *unit, std::nullopt};
			for (std::size_t s = 1; s < arm.cells.size() && s <= near; ++s) {
				const auto *node = node_at(roads, arm.cells[s]);
				if (!node)
					continue;
				const auto feature = node->tags.get("highway");
				if (feature == "traffic_signals")
					arm.control = Control::Signal;
				else if (feature == "stop")
					arm.control = Control::Stop;
				else if (feature == "give_way")
					arm.control = Control::GiveWay;
				if (arm.control)
					break;
			}
			arms.push_back(std::move(arm));
		}
		if (arms.size() < 3)
			continue;

		std::vector<int> continuation(arms.size(), -1);
		for (std::size_t i = 0; i < arms.size(); ++i) {
			if (roads[arms[i].road].ring) {
				float best = std::numeric_limits<float>::infinity();
				for (std::size_t j = 0; j < arms.size(); ++j) {
					if (i == j || !roads[arms[j].road].ring)
						continue;
					const float dot = arms[i].unit.first * arms[j].unit.first +
									  arms[i].unit.second * arms[j].unit.second;
					if (dot < best) {
						best = dot;
						continuation[i] = static_cast<int>(j);
					}
				}
				continue;
			}
			float best = std::numeric_limits<float>::infinity();
			for (std::size_t j = 0; j < arms.size(); ++j) {
				if (i == j)
					continue;
				const float dot = arms[i].unit.first * arms[j].unit.first +
								  arms[i].unit.second * arms[j].unit.second;
				if (dot > -0.5f)
					continue;
				const auto name = roads[arms[i].road].way->tags.get("name");
				const bool same_name = !name.empty() &&
									   name == roads[arms[j].road].way->tags.get("name");
				const float score = dot - (same_name ? 0.5f : 0.0f);
				if (score < best) {
					best = score;
					continuation[i] = static_cast<int>(j);
				}
			}
		}
		for (std::size_t i = 0; i < continuation.size(); ++i) {
			const int j = continuation[i];
			if (j < 0 || static_cast<std::size_t>(j) <= i ||
					continuation[static_cast<std::size_t>(j)] != static_cast<int>(i) ||
					arms[i].road == arms[static_cast<std::size_t>(j)].road)
				continue;
			junction_links.emplace_back(arms[i].road,
					roads[arms[i].road].node_t[arms[i].node],
					arms[static_cast<std::size_t>(j)].road,
					roads[arms[static_cast<std::size_t>(j)].road]
							.node_t[arms[static_cast<std::size_t>(j)].node]);
		}

		std::vector<std::size_t> signal_arms;
		bool signed_controls = false;
		for (std::size_t i = 0; i < arms.size(); ++i) {
			if (arms[i].control == Control::Signal)
				signal_arms.push_back(i);
			if (arms[i].control == Control::Stop || arms[i].control == Control::GiveWay)
				signed_controls = true;
		}
		const bool crossing_signals =
				signal_arms.size() == 1 ||
				(signal_arms.size() == 2 &&
						continuation[signal_arms[0]] ==
								static_cast<int>(signal_arms[1]) &&
						continuation[signal_arms[1]] == static_cast<int>(signal_arms[0]));
		const ProcessedNode *junction_node = nullptr;
		for (const auto &[ri, ni] : refs) {
			if (roads[ri].way->nodes[ni].id == node_id) {
				junction_node = &roads[ri].way->nodes[ni];
				break;
			}
		}
		const bool signalised =
				is_junction && junction_node &&
				(is_signal(*junction_node) ||
						(!signal_arms.empty() && !crossing_signals && !signed_controls));
		if (signalised)
			signalised_ids.insert(node_id);
		auto through = [&](std::size_t i) {
			const int j = continuation[i];
			return j >= 0 &&
				   continuation[static_cast<std::size_t>(j)] == static_cast<int>(i);
		};
		auto outranks = [&](std::size_t j, std::size_t i) {
			const auto &a = roads[arms[i].road];
			const auto &b = roads[arms[j].road];
			return b.rank > a.rank || (b.rank == a.rank && through(j) && !through(i));
		};
		auto signed_at = [&](std::size_t i) {
			return arms[i].control == Control::Stop ||
				   arms[i].control == Control::GiveWay;
		};
		std::vector<std::uint16_t> suppressors(arms.size(), 0);
		std::vector<std::optional<TransverseMark::Kind>> needs(arms.size());
		for (std::size_t i = 0; i < arms.size(); ++i) {
			const auto &a = roads[arms[i].road];
			for (std::size_t j = 0; j < arms.size(); ++j) {
				if (roads[arms[j].road].way->id == a.way->id ||
						continuation[i] == static_cast<int>(j))
					continue;
				const auto &b = roads[arms[j].road];
				bool cuts;
				if (a.ring)
					cuts = b.ring;
				else if (b.ring)
					cuts = true;
				else if ((b.link && !a.link) || (b.rank == 0 && a.rank > 0))
					cuts = false;
				else if (signed_at(i))
					cuts = true;
				else if (signed_at(j) && through(i))
					cuts = false;
				else
					cuts = signalised || b.rank > a.rank ||
						   (b.rank == a.rank && (through(j) || !through(i)));
				if (cuts)
					suppressors[i] |= static_cast<std::uint16_t>(1u << j);
			}
			const bool incoming = a.oneway == 0 || a.oneway == -arms[i].dir;
			if (!incoming || (a.rank == 0 && !signalised))
				continue;
			if (arms[i].control == Control::Signal || arms[i].control == Control::Stop)
				needs[i] = TransverseMark::Kind::Stop;
			else if (arms[i].control == Control::GiveWay)
				needs[i] = TransverseMark::Kind::GiveWay;
			else if (signalised)
				needs[i] = TransverseMark::Kind::Stop;
			else {
				bool yields_to_ring = false, yields_to_priority = false;
				for (std::size_t j = 0; j < arms.size(); ++j) {
					if ((suppressors[i] & (1u << j)) == 0)
						continue;
					yields_to_ring |= roads[arms[j].road].ring;
					yields_to_priority |= outranks(j, i);
				}
				if (!a.ring && yields_to_ring)
					needs[i] = TransverseMark::Kind::GiveWay;
				else if (a.marked && yields_to_priority)
					needs[i] = result.yellow_centre ? TransverseMark::Kind::Stop
													: TransverseMark::Kind::GiveWay;
			}
		}
		const std::uint16_t grid_mask =
				std::accumulate(suppressors.begin(), suppressors.end(), std::uint16_t{0},
						[](std::uint16_t mask, std::uint16_t value) {
							return static_cast<std::uint16_t>(mask | value);
						});
		const auto &center = arms.front().cells.front();
		LocalGrid grid(center.x, center.z, static_cast<int>(reach) + max_half + 1);
		for (std::size_t j = 0; j < arms.size(); ++j) {
			if ((grid_mask & (1u << j)) == 0)
				continue;
			for (const auto &cell : arms[j].cells)
				grid.stamp(cell.x, cell.z, roads[cell.road].half_width + 1,
						static_cast<std::uint16_t>(1u << j));
		}
		for (std::size_t i = 0; i < arms.size(); ++i) {
			if (!suppressors[i] || (!roads[arms[i].road].marked && !needs[i]))
				continue;
			const auto &arm = arms[i];
			std::optional<std::size_t> cut;
			const auto max_s = std::min(arm.cells.size(), cap + 1);
			for (std::size_t s = 0; s < max_s; ++s) {
				const auto [ux, uz] = local_direction(arm.cells, s, arm.unit);
				const float px = -uz, pz = ux;
				const auto &cell = arm.cells[s];
				const int width = static_cast<int>(std::lround(
						roads[cell.road].half_width * (std::abs(px) + std::abs(pz))));
				bool blocked = false;
				for (int p = -width; p <= width; ++p) {
					const int x = static_cast<int>(std::lround(cell.x + px * p));
					const int z = static_cast<int>(std::lround(cell.z + pz * p));
					if ((grid.get(x, z) & suppressors[i]) != 0) {
						blocked = true;
						break;
					}
				}
				if (!blocked) {
					cut = s;
					break;
				}
			}
			if (cut && *cut == 0)
				continue;
			std::size_t clear_end = cut.value_or(std::min(arm.cells.size(), cap + 1));
			std::optional<std::size_t> mark_at = cut;
			if (needs[i] && mark_at && *needs[i] == TransverseMark::Kind::Stop)
				mark_at = std::min(*mark_at + 4 * static_cast<std::size_t>(std::max(
														  roads[arm.road].half_width, 2)),
						arm.cells.size() - 1);
			if (needs[i] && mark_at) {
				const auto cut_at = *cut;
				const auto setback =
						*needs[i] == TransverseMark::Kind::Stop
								? cut_at + 4 * static_cast<std::size_t>(std::max(
													   roads[arm.road].half_width, 2))
								: cut_at;
				std::optional<std::size_t> behind;
				for (std::size_t d = 1; d < arm.cells.size() && d <= setback + 16; ++d) {
					const auto *crossing_node = node_at(roads, arm.cells[d]);
					if (!crossing_node)
						continue;
					const auto crossing = crossing_reach.find(crossing_node->id);
					if (crossing == crossing_reach.end())
						continue;
					const auto reach = static_cast<std::size_t>(crossing->second);
					if (!behind && d + reach >= cut_at && d <= setback + reach)
						behind = d + reach + 1;
					else if (behind && d <= *behind + reach + 1)
						behind = std::max(*behind, d + reach + 1);
					else if (behind)
						break;
				}
				if (behind && *behind < arm.cells.size())
					mark_at = *behind;
			}
			if (mark_at)
				clear_end = *mark_at;
			if (mark_at) {
				const auto search_end =
						std::min(arm.cells.size(), 2 * *mark_at + min_run + 1);
				for (std::size_t s = *mark_at; s < search_end; ++s) {
					const auto *next_node = node_at(roads, arm.cells[s]);
					if (next_node && junction_ids.contains(next_node->id)) {
						mark_at.reset();
						clear_end = s + 1;
						break;
					}
				}
			}
			if (roads[arm.road].marked && clear_end > 0)
				add_gap(arm.cells, clear_end);
			if (needs[i] && mark_at) {
				const auto &at = arm.cells[*mark_at];
				auto &mark_way = result.ways_[roads[at.road].way->id];
				const int side = roads[at.road].oneway != 0
										 ? 0
										 : (result.drives_on_left ? at.dir : -at.dir);
				mark_way.marks.push_back(
						{at.t, *needs[i], false, static_cast<std::int8_t>(side)});
			}
		}
	}
	for (const auto &[ri, ni, node_id] : signal_crossings) {
		const auto reach = crossing_reach[node_id];
		const auto half = static_cast<std::size_t>(roads[ri].half_width);
		const auto clear = static_cast<std::size_t>(reach) + 2 * half + 14;
		const auto room = static_cast<std::size_t>(reach) + 2 * half + 3;
		for (const int dir : {1, -1}) {
			const auto cells = arm_cells(roads, node_uses, ri, ni, dir, clear + 16);
			bool held = false;
			for (std::size_t s = 1; s < cells.size() && s <= clear; ++s) {
				const auto *node = node_at(roads, cells[s]);
				if (node && (signalised_ids.contains(node->id) ||
									(s <= room && junction_ids.contains(node->id)))) {
					held = true;
					break;
				}
			}
			if (held)
				continue;
			std::size_t at = static_cast<std::size_t>(reach) + 1;
			for (std::size_t k = 1; k < cells.size(); ++k) {
				const auto *node = node_at(roads, cells[k]);
				if (!node)
					continue;
				const auto nearby = crossing_reach.find(node->id);
				if (nearby == crossing_reach.end())
					continue;
				const auto nearby_reach = static_cast<std::size_t>(nearby->second);
				if (k > nearby_reach && k - nearby_reach > at)
					break;
				at = std::max(at, k + nearby_reach + 1);
			}
			if (at >= cells.size())
				continue;
			const auto &cell = cells[at];
			const auto &road = roads[cell.road];
			if (road.oneway != 0 && road.oneway != -cell.dir)
				continue;
			const int side =
					road.oneway != 0 ? 0 : (result.drives_on_left ? cell.dir : -cell.dir);
			result.ways_[road.way->id].marks.push_back({cell.t,
					TransverseMark::Kind::Stop, false, static_cast<std::int8_t>(side)});
		}
	}

	std::vector<std::vector<std::tuple<std::uint32_t, std::size_t, std::uint32_t>>>
			adjacent(roads.size());
	for (const auto &[a, ta, b, tb] : junction_links) {
		if (roads[a].marked && roads[b].marked) {
			adjacent[a].emplace_back(ta, b, tb);
			adjacent[b].emplace_back(tb, a, ta);
		}
	}
	for (const auto &[node_id, refs] : node_uses) {
		(void)node_id;
		const auto arms = road_arms_at(roads, refs);
		if (arms.size() != 2 || arms[0].road == arms[1].road)
			continue;
		const auto a = arms[0].road, b = arms[1].road;
		const auto ta = roads[a].node_t[arms[0].node];
		const auto tb = roads[b].node_t[arms[1].node];
		if (roads[a].marked && roads[b].marked) {
			adjacent[a].emplace_back(ta, b, tb);
			adjacent[b].emplace_back(tb, a, ta);
		}
	}
	std::vector<std::optional<std::pair<std::int64_t, std::int8_t>>> phases(roads.size());
	for (std::size_t start = 0; start < roads.size(); ++start) {
		if (adjacent[start].empty() || phases[start])
			continue;
		phases[start] = std::pair<std::int64_t, std::int8_t>{0, 1};
		std::deque<std::size_t> queue{start};
		while (!queue.empty()) {
			const auto a = queue.front();
			queue.pop_front();
			const auto [oa, sa] = *phases[a];
			for (const auto &[ta, b, tb] : adjacent[a]) {
				if (phases[b])
					continue;
				const auto step = ta == 0 ? -sa : sa;
				const auto sb = tb == 0 ? step : -step;
				const auto at_joint = oa + static_cast<std::int64_t>(sa) * ta;
				phases[b] = std::pair<std::int64_t, std::int8_t>{
						at_joint - static_cast<std::int64_t>(sb) * tb,
						static_cast<std::int8_t>(sb)};
				queue.push_back(b);
			}
		}
	}
	for (std::size_t i = 0; i < roads.size(); ++i) {
		if (!phases[i])
			continue;
		auto &marks = result.ways_[roads[i].way->id];
		marks.phase = phases[i]->first;
		marks.phase_step = phases[i]->second;
	}
	for (auto &[id, marks] : result.ways_) {
		(void)id;
		std::sort(marks.gaps.begin(), marks.gaps.end());
		std::sort(
				marks.marks.begin(), marks.marks.end(), [](const auto &a, const auto &b) {
					return std::tie(a.t, a.kind, a.broken, a.side) <
						   std::tie(b.t, b.kind, b.broken, b.side);
				});
		marks.marks.erase(std::unique(marks.marks.begin(), marks.marks.end(),
								  [](const auto &a, const auto &b) {
									  return a.t == b.t && a.kind == b.kind &&
											 a.broken == b.broken && a.side == b.side;
								  }),
				marks.marks.end());
	}
	return result;
}

const WayMarks *RoadMarkingIndex::way(std::uint64_t id) const
{
	const auto it = ways_.find(id);
	return it == ways_.end() ? nullptr : &it->second;
}

const std::vector<CarriagewayClip> *RoadMarkingIndex::crossing_clips(
		std::uint64_t id) const
{
	const auto it = crossings_.find(id);
	return it == crossings_.end() ? nullptr : &it->second;
}

const CarriagewayClip *RoadMarkingIndex::carriageway_at(
		std::uint64_t id, int x, int z) const
{
	const auto *clips = crossing_clips(id);
	if (!clips)
		return nullptr;
	const auto it = std::find_if(clips->begin(), clips->end(),
			[&](const auto &clip) { return clip.contains(x, z); });
	return it == clips->end() ? nullptr : &*it;
}

std::optional<CrossingPaint> RoadMarkingIndex::crossing_way_paint(
		const tags_t &tags) const
{
	if (tags.get("highway") != "footway" || tags.get("footway") != "crossing")
		return std::nullopt;
	return crossing_paint(tags, CrossingPaint{CrossingPaint::Kind::Zebra, false},
			signal_crossing_lines);
}
} // namespace arnis::road_markings
