#include "road_markings.h"
#include "highways.h"
#include "../clipping.h"
#include "../bresenham.h"
#include "../strict_parse.h"
#include "surfaces.h"
#include "raceways.h"
#include "stations.h"

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
	std::span<const Block> surface;
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

bool is_roundabout_impl(const tags_t &tags)
{
	return tags.get("junction") == "roundabout" || tags.get("junction") == "circular";
}

int oneway_sign_impl(const std::string &highway, const tags_t &tags)
{
	const auto value = tags.get("oneway");
	if (value == "yes" || value == "true" || value == "1")
		return ::arnis::road_markings::is_roundabout(tags) || highway == "motorway" ||
			   highway == "motorway_link";
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

bool signalled_crossing(const tags_t &tags)
{
	return tags.get("crossing") == "traffic_signals" ||
		   tags.get("crossing:signals") == "yes";
}

bool is_crossing_way(const tags_t &tags)
{
	return tags.get("highway") == "footway" && tags.get("footway") == "crossing";
}

bool describes_crossing(const ProcessedNode &node)
{
	const auto highway = node.tags.get("highway");
	return highway == "crossing" ||
		   (highway == "traffic_signals" && node.tags.contains("crossing"));
}

bool raceway_is_pit_lane(const tags_t &tags)
{
	const auto value = tags.get("raceway");
	return value == "pitlane" || value == "pit_lane" || value == "pit" || value == "pits";
}

bool raceway_is_start_line(const std::string &value)
{
	return value == "start-finish" || value == "start" || value == "finish" ||
		   value == "startline" || value == "start/finish" || value == "finish_line";
}

// Outer nullopt means no crossing tag; an engaged empty optional means
// explicitly unmarked; an engaged paint is a marked crossing.
std::optional<std::optional<CrossingPaint>> tagged_crossing_paint(
		const tags_t &tags, bool signal_lines)
{
	const auto unmarked = [] {
		return std::optional<std::optional<CrossingPaint>>(std::in_place, std::nullopt);
	};
	const auto painted = [](CrossingPaint paint) {
		return std::optional<std::optional<CrossingPaint>>(
				std::in_place, std::move(paint));
	};
	const auto crossing = tags.get("crossing");
	if (crossing == "no" || crossing == "unmarked" || crossing == "informal" ||
			crossing == "impossible")
		return unmarked();
	const auto markings = tags.get("crossing:markings");
	if (markings == "no" || markings == "surface")
		return unmarked();
	if (markings == "lines" || markings == "lines:paired")
		return painted({CrossingPaint::Kind::Lines, false});
	if (markings == "dashes" || markings == "dots" || markings == "dashes:paired")
		return painted({CrossingPaint::Kind::Lines, true});
	if (tags.contains("crossing:markings"))
		return painted({CrossingPaint::Kind::Zebra, false});
	if (tags.get("crossing_ref") == "zebra" || crossing == "zebra")
		return painted({CrossingPaint::Kind::Zebra, false});
	if (signalled_crossing(tags))
		return painted(signal_lines ? CrossingPaint{CrossingPaint::Kind::Lines, true}
									: CrossingPaint{CrossingPaint::Kind::Zebra, false});
	if (tags.contains("crossing"))
		return painted({CrossingPaint::Kind::Zebra, false});
	return std::nullopt;
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
	// Rust matches `Some(_)` here, so even a present-but-empty tag selects
	// zebra paint; distinguish that from an absent key.
	if (tags.contains("crossing:markings"))
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	if (tags.get("crossing_ref") == "zebra" || crossing == "zebra")
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	if (signalled_crossing(tags))
		return signal_lines ? CrossingPaint{CrossingPaint::Kind::Lines, true}
							: CrossingPaint{CrossingPaint::Kind::Zebra, false};
	// Rust distinguishes an absent tag from a present-but-empty tag. Empty
	// `crossing=*` still means a mapped crossing and receives the default paint.
	if (tags.contains("crossing"))
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
			for (std::size_t distance = 1;
					distance < points.size() && cells.size() < limit; ++distance) {
				const auto point_index = points.size() - 1 - distance;
				cells.push_back({std::get<0>(points[point_index]),
						std::get<2>(points[point_index]), ri,
						roads[ri].node_t[k] - static_cast<std::uint32_t>(distance), dir});
			}
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

bool is_roundabout(const tags_t &tags)
{
	return is_roundabout_impl(tags);
}

int oneway_sign(const std::string &highway, const tags_t &tags)
{
	return oneway_sign_impl(highway, tags);
}

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
	// Raceway edges are resolved from all source ways before tiles are dispatched,
	// so borders remain continuous where neighbouring ways meet or split at a chunk.
	std::map<int, stations::SparseCells> track_surface, paved_track;
	std::vector<const ProcessedWay *> track_ways;
	std::vector<std::tuple<const ProcessedWay *, int, int>> raceway_ways;
	std::unordered_map<std::uint64_t, std::vector<int>> end_decks;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto level = strict_parse::i32(way.tags.get("level"));
		if (way.tags.get("highway") != "raceway" || way.nodes.size() < 2 ||
				way.tags.get("area") == "yes" || way.tags.get("indoor") == "yes" ||
				(way.tags.contains("tunnel") && way.tags.get("tunnel") != "no") ||
				(level && *level < 0))
			continue;
		const int half = arnis::highways::highway_block_range("raceway", way.tags, scale);
		const int deck = raceways::deck(way.tags);
		raceway_ways.emplace_back(&way, half, deck);
		end_decks[way.nodes.front().id].push_back(deck);
		end_decks[way.nodes.back().id].push_back(deck);
	}
	for (const auto &[way, half, deck] : raceway_ways) {
		const bool painted = half >= 2 && raceways::is_paved(way->tags);
		const auto cells = way_cells(*way);
		for (const auto &[x, z] : cells) {
			track_surface[deck].set_square(x, z, half);
			if (painted)
				paved_track[deck].set_square(x, z, half);
		}
		const std::size_t reach = std::min<std::size_t>(half + 2, cells.size());
		const auto copy_joint = [&](std::uint64_t id, std::size_t begin,
										std::size_t end) {
			for (const auto other_deck : end_decks[id]) {
				if (other_deck == deck)
					continue;
				for (std::size_t i = begin; i < end; ++i)
					track_surface[other_deck].set_square(
							cells[i].first, cells[i].second, half);
			}
		};
		copy_joint(way->nodes.front().id, 0, reach);
		copy_joint(way->nodes.back().id, cells.size() - reach, cells.size());
		if (painted)
			track_ways.push_back(way);
	}
	struct RacewayLineCell
	{
		int x, z;
		float s, tx, tz;
		int half, deck;
		std::int8_t kerb;
		bool both;
	};
	struct RacewayLine
	{
		std::vector<RacewayLineCell> cells;
		float length{0.0f};
		bool closed{false};
		std::size_t order{0};
	};
	std::vector<RacewayLine> raceway_lines;
	for (const auto &[deck, painted_cells] : paved_track) {
		if (painted_cells.empty())
			continue;
		// Join split centerlines at unambiguous two-way OSM nodes. This keeps
		// curvature estimates and kerb striping continuous through way boundaries,
		// including where connected ways change bridge/layer deck.
		const auto &ways = track_ways;
		std::unordered_map<std::uint64_t, std::vector<std::size_t>> ends;
		for (std::size_t i = 0; i < ways.size(); ++i) {
			if (ways[i]->nodes.front().id == ways[i]->nodes.back().id)
				continue;
			ends[ways[i]->nodes.front().id].push_back(i);
			ends[ways[i]->nodes.back().id].push_back(i);
		}
		std::vector<bool> visited(ways.size(), false);
		for (std::size_t start = 0; start < ways.size(); ++start) {
			if (visited[start])
				continue;
			std::deque<std::pair<std::size_t, bool>> run{{start, false}};
			visited[start] = true;
			const auto first_id = [&](std::size_t i) {
				return ways[i]->nodes.front().id;
			};
			const auto last_id = [&](std::size_t i) { return ways[i]->nodes.back().id; };
			const auto next_way = [&](std::uint64_t node) -> std::optional<std::size_t> {
				const auto it = ends.find(node);
				if (it == ends.end() || it->second.size() != 2)
					return std::nullopt;
				for (const auto index : it->second)
					if (!visited[index])
						return index;
				return std::nullopt;
			};
			if (first_id(start) != last_id(start)) {
				auto tail = last_id(start);
				while (const auto next = next_way(tail)) {
					const auto index = *next;
					visited[index] = true;
					const bool reversed = last_id(index) == tail;
					tail = reversed ? first_id(index) : last_id(index);
					run.emplace_back(index, reversed);
				}
				auto head = first_id(start);
				while (const auto next = next_way(head)) {
					const auto index = *next;
					visited[index] = true;
					const bool reversed = first_id(index) == head;
					head = reversed ? last_id(index) : first_id(index);
					run.emplace_front(index, reversed);
				}
			}
			if (raceways::deck(ways[run.front().first]->tags) != deck)
				continue;

			std::vector<std::pair<int, int>> path;
			std::vector<int> half_by_cell;
			std::vector<bool> kerbs_by_cell;
			std::vector<int> deck_by_cell;
			std::vector<float> distance_by_cell;
			std::vector<float> node_distances{0.0f};
			std::vector<const ProcessedNode *> chain_nodes;
			std::vector<int> half_by_node;
			std::vector<bool> kerbs_by_node;
			std::vector<int> deck_by_node;
			for (const auto &[index, reversed] : run) {
				const auto *way = ways[index];
				const int half =
						arnis::highways::highway_block_range("raceway", way->tags, scale);
				const bool kerbs = !raceway_is_pit_lane(way->tags);
				const std::size_t skip = chain_nodes.empty() ? 0 : 1;
				for (std::size_t i = skip; i < way->nodes.size(); ++i) {
					const std::size_t node_index =
							reversed ? way->nodes.size() - 1 - i : i;
					chain_nodes.push_back(&way->nodes[node_index]);
					half_by_node.push_back(half);
					kerbs_by_node.push_back(kerbs);
					deck_by_node.push_back(raceways::deck(way->tags));
				}
			}
			// Preserve Rust's chain geometry and its cumulative Euclidean `s`;
			// each connected way contributes all nodes except the shared join.
			float cumulative_distance = 0.0f;
			for (std::size_t k = 1; k < chain_nodes.size(); ++k) {
				const auto &a = *chain_nodes[k - 1];
				const auto &b = *chain_nodes[k];
				const auto points = bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
				const float dx = static_cast<float>(b.x) - static_cast<float>(a.x);
				const float dz = static_cast<float>(b.z) - static_cast<float>(a.z);
				const float segment_length = std::hypot(dx, dz);
				const float steps =
						static_cast<float>(std::max<std::size_t>(1, points.size() - 1));
				const std::size_t begin = k == 1 ? 0 : 1;
				for (std::size_t j = begin; j < points.size(); ++j) {
					path.emplace_back(std::get<0>(points[j]), std::get<2>(points[j]));
					half_by_cell.push_back(half_by_node[k]);
					kerbs_by_cell.push_back(kerbs_by_node[k]);
					deck_by_cell.push_back(deck_by_node[k]);
					distance_by_cell.push_back(
							cumulative_distance +
							segment_length * static_cast<float>(j) / steps);
				}
				cumulative_distance += segment_length;
				node_distances.push_back(cumulative_distance);
			}
			if (path.empty())
				continue;
			const float curve_window = std::max(16.0f * static_cast<float>(scale), 4.0f);
			const bool closed = chain_nodes.size() > 3 &&
								chain_nodes.front()->id == chain_nodes.back()->id;
			struct Bend
			{
				double tx{1.0}, tz{0.0};
				std::int8_t side{0};
				bool tight{false};
			};
			std::vector<Bend> bends(path.size());
			const auto line_at = [&](float distance) {
				if (closed && cumulative_distance > 0.0f) {
					distance = std::fmod(distance, cumulative_distance);
					if (distance < 0.0f)
						distance += cumulative_distance;
				} else {
					distance = std::clamp(distance, 0.0f, cumulative_distance);
				}
				const auto upper = std::upper_bound(
						node_distances.begin(), node_distances.end(), distance);
				const std::size_t i = std::clamp<std::size_t>(
						upper - node_distances.begin(), 1, chain_nodes.size() - 1);
				const float segment = node_distances[i] - node_distances[i - 1];
				const float fraction =
						segment > 0.0f ? (distance - node_distances[i - 1]) / segment
									   : 0.0f;
				const auto &a = *chain_nodes[i - 1];
				const auto &b = *chain_nodes[i];
				return std::pair{static_cast<float>(a.x) +
										 (static_cast<float>(b.x) - a.x) * fraction,
						static_cast<float>(a.z) +
								(static_cast<float>(b.z) - a.z) * fraction};
			};
			for (std::size_t pi = 0; pi < path.size(); ++pi) {
				const bool kerbs = kerbs_by_cell[pi];
				const auto behind = line_at(distance_by_cell[pi] - 2.0f);
				const auto ahead = line_at(distance_by_cell[pi] + 2.0f);
				const float tangent_x = ahead.first - behind.first;
				const float tangent_z = ahead.second - behind.second;
				const float tangent_length = std::hypot(tangent_x, tangent_z);
				if (tangent_length > 0.0f) {
					bends[pi].tx = tangent_x / tangent_length;
					bends[pi].tz = tangent_z / tangent_length;
				}
				if (!kerbs)
					continue;
				if (!closed && (distance_by_cell[pi] < curve_window * 0.75f ||
									   cumulative_distance - distance_by_cell[pi] <
											   curve_window * 0.75f))
					continue;
				const auto p = line_at(distance_by_cell[pi]);
				const auto a = line_at(distance_by_cell[pi] - curve_window);
				const auto b = line_at(distance_by_cell[pi] + curve_window);
				const float d1x = p.first - a.first, d1z = p.second - a.second;
				const float d2x = b.first - p.first, d2z = b.second - p.second;
				const float l1 = std::hypot(d1x, d1z), l2 = std::hypot(d2x, d2z);
				if (l1 >= 1.0f && l2 >= 1.0f) {
					const float turn =
							std::atan2(d1x * d2z - d1z * d2x, d1x * d2x + d1z * d2z);
					const float curvature = std::abs(turn) / ((l1 + l2) * 0.5f);
					if (curvature > 1.0f / (90.0f * static_cast<float>(scale))) {
						bends[pi].side = turn > 0.0f ? 1 : -1;
						bends[pi].tight =
								curvature > 1.0f / (35.0f * static_cast<float>(scale));
					}
				}
			}
			// Ignore tiny curvature spikes, matching Rust's minimum useful kerb run.
			const auto drop_short_runs = [](std::vector<std::int8_t> &flags,
												 std::size_t min_run, bool loop) {
				const auto n = flags.size();
				if (n == 0)
					return;
				std::size_t start = 0;
				if (loop) {
					const auto zero = std::find(flags.begin(), flags.end(), 0);
					if (zero == flags.end())
						return;
					start = static_cast<std::size_t>(zero - flags.begin());
				}
				for (std::size_t i = 0; i < n;) {
					const auto value = flags[(start + i) % n];
					std::size_t length = 1;
					while (i + length < n && flags[(start + i + length) % n] == value)
						++length;
					if (value != 0 && length < min_run)
						for (std::size_t j = 0; j < length; ++j)
							flags[(start + i + j) % n] = 0;
					i += length;
				}
			};
			std::vector<std::int8_t> side(path.size()), tight(path.size());
			for (std::size_t i = 0; i < bends.size(); ++i) {
				side[i] = bends[i].side;
				tight[i] = bends[i].tight ? 1 : 0;
			}
			const auto min_kerb_run = static_cast<std::size_t>(
					std::max(6.0f * static_cast<float>(scale), 3.0f));
			drop_short_runs(side, min_kerb_run, closed);
			drop_short_runs(tight, min_kerb_run, closed);
			RacewayLine line;
			line.length = cumulative_distance;
			line.closed = closed;
			line.order = start;
			line.cells.reserve(path.size());
			for (std::size_t pi = 0; pi < path.size(); ++pi) {
				line.cells.push_back({path[pi].first, path[pi].second,
						distance_by_cell[pi], static_cast<float>(bends[pi].tx),
						static_cast<float>(bends[pi].tz), half_by_cell[pi],
						deck_by_cell[pi], side[pi], side[pi] != 0 && tight[pi] != 0});
			}
			raceway_lines.push_back(std::move(line));
		}
	}
	// Start/finish markers are snapped to the nearest continuous raceway chain.
	// Re-check each painted cell against that same chain so parallel pit lanes
	// and nearby circuits do not inherit the marker.
	std::map<std::pair<int, int>, std::vector<std::pair<std::size_t, std::size_t>>>
			raceway_grid;
	int max_raceway_half = 1;
	for (std::size_t ci = 0; ci < raceway_lines.size(); ++ci) {
		for (std::size_t i = 0; i < raceway_lines[ci].cells.size(); ++i) {
			const auto &cell = raceway_lines[ci].cells[i];
			max_raceway_half = std::max(max_raceway_half, cell.half);
			raceway_grid[{cell.x >> 3, cell.z >> 3}].emplace_back(ci, i);
		}
	}
	const int raceway_reach = max_raceway_half * 3 / 2 + 2;
	const auto nearest_raceway_cell = [&](int x, int z, std::optional<int> deck)
			-> std::optional<std::pair<std::size_t, std::size_t>> {
		std::optional<
				std::tuple<bool, std::int64_t, std::size_t, std::size_t, std::size_t>>
				best;
		for (int gx = (x - raceway_reach) >> 3; gx <= (x + raceway_reach) >> 3; ++gx) {
			for (int gz = (z - raceway_reach) >> 3; gz <= (z + raceway_reach) >> 3;
					++gz) {
				const auto bucket = raceway_grid.find({gx, gz});
				if (bucket == raceway_grid.end())
					continue;
				for (const auto &[ci, i] : bucket->second) {
					const auto &line = raceway_lines[ci];
					const auto &cell = line.cells[i];
					if (deck && *deck != cell.deck)
						continue;
					const auto dx = static_cast<std::int64_t>(cell.x) - x;
					const auto dz = static_cast<std::int64_t>(cell.z) - z;
					const bool outside = std::max(std::abs(dx), std::abs(dz)) > cell.half;
					const auto d2 = dx * dx + dz * dz;
					const auto candidate = std::tuple{outside, d2, line.order, i, ci};
					if (!best || candidate < *best)
						best = candidate;
				}
			}
		}
		if (!best)
			return std::nullopt;
		return std::pair{std::get<4>(*best), std::get<3>(*best)};
	};
	for (const auto &[deck, paved] : paved_track) {
		const auto surface_it = track_surface.find(deck);
		if (surface_it == track_surface.end())
			continue;
		const auto &surface = surface_it->second;
		const auto rim = [&](int x, int z) {
			return surface.contains(x, z) &&
				   (!surface.contains(x + 1, z) || !surface.contains(x - 1, z) ||
						   !surface.contains(x, z + 1) || !surface.contains(x, z - 1));
		};
		for (const auto &line : raceway_lines) {
			for (const auto &cell : line.cells) {
				if (cell.deck != deck)
					continue;
				for (int dx = -cell.half; dx <= cell.half; ++dx) {
					for (int dz = -cell.half; dz <= cell.half; ++dz) {
						const int x = cell.x + dx, z = cell.z + dz;
						if (!paved.contains(x, z))
							continue;
						const int depth = rim(x, z) ? 0
										  : (rim(x + 1, z) || rim(x - 1, z) ||
													rim(x, z + 1) || rim(x, z - 1))
												  ? 1
												  : -1;
						if (depth < 0)
							continue;
						const auto nearest = nearest_raceway_cell(x, z, deck);
						if (!nearest)
							continue;
						const auto &[chain_index, cell_index] = *nearest;
						const auto &nearest_cell =
								raceway_lines[chain_index].cells[cell_index];
						const float rel_x = static_cast<float>(x - nearest_cell.x);
						const float rel_z = static_cast<float>(z - nearest_cell.z);
						const float side_value =
								nearest_cell.tx * rel_z - nearest_cell.tz * rel_x;
						const int side = side_value > 0.5f	  ? 1
										 : side_value < -0.5f ? -1
															  : 0;
						const bool kerb =
								nearest_cell.kerb != 0 && side != 0 &&
								(side == nearest_cell.kerb || nearest_cell.both) &&
								(depth == 0 || nearest_cell.half >= 5);
						if (kerb) {
							const auto stripe = static_cast<std::int64_t>(
									std::floor(nearest_cell.s / 2.0f));
							result.raceway_paint_[{deck, x, z}] =
									stripe % 2 == 0
											? crate::block_definitions::RED_CONCRETE
											: crate::block_definitions::WHITE_CONCRETE;
						} else if (depth == 0) {
							result.raceway_paint_[{deck, x, z}] =
									crate::block_definitions::WHITE_CONCRETE;
						}
					}
				}
			}
		}
	}
	for (const auto &element : elements) {
		if (!element.is_node())
			continue;
		const auto &node = element.as_node();
		const auto marker = node.tags.get("raceway");
		if (!raceway_is_start_line(marker))
			continue;
		const auto nearest = nearest_raceway_cell(node.x, node.z, std::nullopt);
		if (!nearest)
			continue;
		const auto &[chain_index, cell_index] = *nearest;
		const auto &line = raceway_lines[chain_index];
		const auto &cell = line.cells[cell_index];
		if (std::max(std::abs(cell.x - node.x), std::abs(cell.z - node.z)) >
				cell.half + 2)
			continue;
		const auto surface = paved_track.find(cell.deck);
		if (surface == paved_track.end())
			continue;
		const int radius = cell.half * 3 / 2 + 1;
		for (int x = cell.x - radius; x <= cell.x + radius; ++x) {
			for (int z = cell.z - radius; z <= cell.z + radius; ++z) {
				if (!surface->second.contains(x, z))
					continue;
				const float dx = static_cast<float>(x - cell.x);
				const float dz = static_cast<float>(z - cell.z);
				const float along = dx * cell.tx + dz * cell.tz;
				if (along < -1.0 || along >= 1.0)
					continue;
				const auto same_chain = nearest_raceway_cell(x, z, cell.deck);
				if (!same_chain || same_chain->first != chain_index)
					continue;
				const auto &other =
						raceway_lines[same_chain->first].cells[same_chain->second];
				float gap = std::abs(other.s - cell.s);
				if (line.closed)
					gap = std::min(gap, line.length - gap);
				if (gap > 3.0f)
					continue;
				const auto col = static_cast<std::int64_t>(
						std::floor(cell.tx * dz - cell.tz * dx));
				const auto row = static_cast<std::int64_t>(std::floor(along));
				result.raceway_paint_[{cell.deck, x, z}] =
						((row + col) % 2 == 0) ? crate::block_definitions::WHITE_CONCRETE
											   : crate::block_definitions::BLACK_CONCRETE;
			}
		}
	}
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
			if (is_crossing_way(way.tags))
				crossing_ways.push_back(&way);
			continue;
		}
		const auto range = arnis::highways::highway_block_range(highway, way.tags, scale);
		const auto surface = arnis::highways::surface_palette_view(highway, way.tags);
		const auto [rank, link] = road_class(highway);
		roads.push_back({&way, rank, link, is_roundabout(way.tags),
				oneway_sign_impl(highway, way.tags),
				arnis::highways::lane_marking_count(highway, way.tags) >= 2, range,
				std::move(surface), node_path_index(way)});
	}
	// A crossing way can share its crossing markings with a highway=node. Only
	// road nodes are authoritative; a tag on an unrelated footpath node does not
	// change whether the crossing itself is painted.
	std::unordered_set<std::uint64_t> crossing_nodes_for_roads;
	for (const auto *crossing : crossing_ways)
		for (const auto &node : crossing->nodes)
			crossing_nodes_for_roads.insert(node.id);
	for (const auto &road : roads)
		for (const auto &node : road.way->nodes)
			if (crossing_nodes_for_roads.contains(node.id))
				result.crossing_road_nodes_.insert(node.id);
	crossing_ways.erase(std::remove_if(crossing_ways.begin(), crossing_ways.end(),
								[&](const ProcessedWay *way) {
									return !result.crossing_way_paint(*way).has_value();
								}),
			crossing_ways.end());
	// Rust indexes only node IDs referenced at least twice across road and
	// crossing ways. For closed ways, count the closing node once, then retain
	// both path positions below when the ID is shared.
	std::unordered_map<std::uint64_t, std::uint32_t> node_use_counts;
	auto count_way_nodes = [&](const ProcessedWay &way) {
		const bool closed =
				way.nodes.size() > 2 && way.nodes.front().id == way.nodes.back().id;
		const std::size_t begin = closed ? 1 : 0;
		for (std::size_t ni = begin; ni < way.nodes.size(); ++ni) {
			const auto id = way.nodes[ni].id;
			if (!clipping::is_invented_node_id(id))
				++node_use_counts[id];
		}
	};
	for (const auto &road : roads)
		count_way_nodes(*road.way);
	for (const auto *crossing : crossing_ways)
		count_way_nodes(*crossing);
	for (std::size_t ri = 0; ri < roads.size(); ++ri) {
		const auto &way = *roads[ri].way;
		for (std::size_t ni = 0; ni < way.nodes.size(); ++ni) {
			const auto id = way.nodes[ni].id;
			const auto uses = node_use_counts.find(id);
			if (!clipping::is_invented_node_id(id) && uses != node_use_counts.end() &&
					uses->second >= 2)
				node_uses[id].emplace_back(ri, ni);
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
	std::unordered_set<std::uint64_t> signalled_crossing_way_nodes;
	std::unordered_set<std::uint64_t> crossing_way_nodes;
	std::unordered_set<std::uint64_t> crossing_arms;

	// Find the carriageway footprints intersected by each painted crossing way.
	// The clip bounds both its surface replacement and the break in lane paint.
	for (const auto *crossing : crossing_ways) {
		const auto paint = result.crossing_way_paint(*crossing);
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
			// Rust records every crossing-way node shared with a road before
			// computing its clip. A node crossing at the same location must not
			// be painted again even when clipping produces no carriageway cells.
			crossing_arms.insert(crossing_node.id);
			for (const auto &[ri, ni] : refs->second) {
				const auto &road = roads[ri];
				const std::size_t limit =
						crossing_cells.size() +
						static_cast<std::size_t>(road.half_width + crossing_half + 2);
				const std::array<std::vector<ArmCell>, 2> sides{
						arm_cells(roads, node_uses, ri, ni, 1, limit),
						arm_cells(roads, node_uses, ri, ni, -1, limit)};
				std::vector<std::pair<int, int>> centreline;
				for (const auto &side : sides)
					for (const auto &cell : side)
						if (chebyshev_distance_to(crossing_cells, cell.x, cell.z) <=
								road.half_width + crossing_half)
							centreline.emplace_back(cell.x, cell.z);
				CarriagewayClip clip;
				clip.half_width = road.half_width;
				clip.surface = road.surface;
				clip.centreline = std::move(centreline);
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
				if (signalled_crossing(crossing->tags))
					signalled_crossing_way_nodes.insert(crossing_node.id);
			}
		}
	}
	// Rust emits one signal-crossing record per shared node, anchored on the
	// first road arm; signal_crossing_stops then walks both directions from
	// that arm, carrying across plain way joints. Recording every road ref here
	// placed duplicate stop lines on multi-arm intersections.
	for (const auto &[node_id, reach] : crossing_reach) {
		(void)reach;
		const auto refs = node_uses.find(node_id);
		if (refs == node_uses.end() || refs->second.empty())
			continue;
		const auto [ri, ni] = refs->second.front();
		if (signalled_crossing(roads[ri].way->nodes[ni].tags) ||
				signalled_crossing_way_nodes.contains(node_id))
			signal_crossings.emplace_back(ri, ni, node_id);
	}

	// Node-mapped pedestrian crossings are rendered once from the first road
	// arm; arm_cells follows linked way splits in both directions.  This matches
	// Rust's crossing_reach guard, which prevents duplicate marks at a shared
	// node while still painting across both halves of a split road.
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
				!(highway == "traffic_signals" && node.tags.contains("crossing")))
			continue;
		const auto paint =
				crossing_paint(node.tags, std::nullopt, result.signal_crossing_lines);
		if (!paint || crossing_arms.contains(node.id) || crossing_reach.contains(node.id))
			continue;
		const auto refs = node_uses.find(node.id);
		std::vector<std::pair<std::size_t, std::size_t>> standalone_refs;
		const std::vector<std::pair<std::size_t, std::size_t>> *node_refs = nullptr;
		if (refs != node_uses.end()) {
			node_refs = &refs->second;
		} else {
			// Rust still paints node-mapped crossings on a road node that is not
			// shared with another way; it just has no shared-node index entry.
			bool found = false;
			for (std::size_t ri = 0; ri < roads.size() && !found; ++ri)
				for (std::size_t ni = 0; ni < roads[ri].way->nodes.size(); ++ni)
					if (roads[ri].way->nodes[ni].id == node_id) {
						standalone_refs.emplace_back(ri, ni);
						found = true;
						break;
					}
			node_refs = &standalone_refs;
		}
		if (node_refs->empty() || road_arms_at(roads, *node_refs).size() > 2)
			continue;
		const auto [ri, ni] = node_refs->front();
		const std::size_t near_crossing =
				static_cast<std::size_t>(std::max(2.0, 8.0 * scale));
		if (is_signal(node)) {
			bool beside_crossing = false;
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
				continue;
		}
		const std::uint32_t reach = paint->kind == CrossingPaint::Kind::Zebra ? 2 : 3;
		crossing_reach[node_id] = std::max(crossing_reach[node_id], reach);
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
		const std::size_t near_ = sign_reach + 2 * static_cast<std::size_t>(max_half);
		// Most shared nodes are residential junctions with neither lane paint nor
		// controls. Rust avoids building full arm geometry for those nodes; keep
		// the prescan bounded on dense OSM extracts while retaining the junction
		// index built above for nearby signal/crossing decisions.
		const auto has_control = [](const ProcessedNode *node) {
			if (!node)
				return false;
			const auto highway = node->tags.get("highway");
			return highway == "traffic_signals" || highway == "stop" ||
				   highway == "give_way";
		};
		const bool anything_to_resolve = std::any_of(
				directions.begin(), directions.end(), [&](const RoadArm &direction) {
					const auto cells = arm_cells(roads, node_uses, direction.road,
							direction.node, direction.dir, near_ + 1);
					return std::any_of(
							cells.begin(), cells.end(), [&](const ArmCell &cell) {
								const auto &road = roads[cell.road];
								return road.marked || road.ring ||
									   has_control(node_at(roads, cell));
							});
				});
		if (!anything_to_resolve)
			continue;
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
			for (std::size_t s = 1; s < arm.cells.size() && s <= near_; ++s) {
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
			const auto &[oa, sa] = *phases[a];
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
		std::stable_sort(marks.marks.begin(), marks.marks.end(),
				[](const auto &a, const auto &b) { return a.t < b.t; });
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

std::optional<Block> RoadMarkingIndex::raceway_at(int deck, int x, int z) const
{
	const auto it = raceway_paint_.find({deck, x, z});
	return it == raceway_paint_.end() ? std::nullopt : std::optional<Block>(it->second);
}

std::optional<CrossingPaint> RoadMarkingIndex::crossing_way_paint(
		const ProcessedWay &way) const
{
	if (!is_crossing_way(way.tags))
		return std::nullopt;
	if (const auto tagged = tagged_crossing_paint(way.tags, signal_crossing_lines))
		return *tagged;
	bool unmarked = false;
	bool silent = false;
	for (const auto &node : way.nodes) {
		if (!crossing_road_nodes_.contains(node.id))
			continue;
		const auto tagged = describes_crossing(node) ? tagged_crossing_paint(node.tags,
															   signal_crossing_lines)
													 : std::nullopt;
		if (tagged) {
			if (*tagged)
				return **tagged;
			unmarked = true;
		} else {
			silent = true;
		}
	}
	if (silent || !unmarked)
		return CrossingPaint{CrossingPaint::Kind::Zebra, false};
	return std::nullopt;
}
} // namespace arnis::road_markings
