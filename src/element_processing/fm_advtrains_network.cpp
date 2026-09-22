#include "advtrains.h"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <tuple>

#include "log.h"

namespace arnis::railways::advtrains
{
using namespace block_definitions;
thread_local std::map<std::uint64_t, RailPlan> plans;
thread_local std::set<std::uint64_t> generated;
thread_local std::vector<v3pos_t> tunnel_floors;

const RailPlan *get_plan(const ProcessedWay &way)
{
	const auto it = plans.find(way.id);
	return it == plans.end() ? nullptr : &it->second;
}

void mark_generated(const ProcessedWay &way)
{
	generated.insert(way.id);
}

void finish_network(WorldEditor &editor)
{
	// The Arnis editor normally keeps the first feature written at a cell.
	// Reserve railway formation and clearance after scenery/terrain/tunnels so
	// trees, poles, and adjacent tunnel shells cannot interrupt a planned track.
	const std::optional<std::vector<Block>> replace_all{std::vector<Block>{}};
	std::set<std::tuple<pos_t, pos_t, pos_t>> track_cells;
	for (auto id : generated)
		if (const auto it = plans.find(id); it != plans.end())
			for (std::size_t i = 0; i < it->second.line.size(); ++i) {
				const auto [x, z] = it->second.line[i];
				track_cells.emplace(x, it->second.heights[i], z);
				track_cells.emplace(x, it->second.heights[i] + 1, z);
			}
	for (auto id : generated) {
		const auto it = plans.find(id);
		if (it == plans.end())
			continue;
		const auto &plan = it->second;
		for (std::size_t i = 0; i < plan.line.size(); ++i) {
			const auto [x, z] = plan.line[i];
			const pos_t y = plan.heights[i];
			editor.set_block_absolute(GRAVEL, x, y, z, std::nullopt, replace_all);
			editor.set_block_absolute(
					plan.rails[i], x, y + 1, z, std::nullopt, replace_all);
			for (pos_t dy = 2; dy <= 4; ++dy)
				if (!track_cells.contains({x, y + dy, z}))
					editor.set_block_absolute(
							AIR, x, y + dy, z, std::nullopt, replace_all);
		}
	}
}

// Search in position AND heading: dtrack_cr changes heading by one sector.
// Selecting the nearest bearing independently at every cell is not sufficient.
std::vector<XZ> route(const XZ &from, const XZ &to, int departure, int arrival,
		const std::function<bool(const XZ &)> &blocked)
{
	using State = std::tuple<pos_t, pos_t, int>;
	struct Visit
	{
		double cost;
		State previous;
	};
	const auto distance = [&](pos_t x, pos_t z) {
		return std::hypot(double(x - to.first), double(z - to.second));
	};
	for (pos_t margin : {16, 48}) {
		std::map<State, Visit> visits;
		using Entry = std::tuple<double, double, State>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
		const State start{from.first, from.second, departure};
		visits.emplace(start, Visit{0, start});
		// A mildly weighted heuristic keeps long, almost straight OSM ways from
		// flooding the open-set with detours. The cost remains the same for the
		// selected route; only search ordering is changed.
		constexpr double heuristic_weight = 1.15;
		queue.emplace(heuristic_weight * distance(from.first, from.second), 0, start);
		while (!queue.empty() && visits.size() < 300000) {
			const auto [estimate, cost, state] = queue.top();
			queue.pop();
			if (cost != visits.at(state).cost)
				continue;
			const auto [x, z, heading] = state;
			if (XZ{x, z} == to && heading == arrival) {
				std::vector<XZ> result;
				std::set<XZ> seen;
				State cursor = state;
				for (;;) {
					const auto [cx, cz, cd] = cursor;
					if (!seen.emplace(cx, cz).second)
						return {};
					result.emplace_back(cx, cz);
					if (cursor == start)
						break;
					cursor = visits.at(cursor).previous;
				}
				std::reverse(result.begin(), result.end());
				return result;
			}
			for (int turn : {0, -1, 1}) {
				if (state == start && turn != 0)
					continue;
				const int direction = (heading + turn + 16) % 16;
				const auto [dx, dz] = DIR_VECTORS[direction];
				const XZ next{x + dx, z + dz};
				if (next == from || (next == to && direction != arrival))
					continue;
				if (next.first < std::min(from.first, to.first) - margin ||
						next.first > std::max(from.first, to.first) + margin ||
						next.second < std::min(from.second, to.second) - margin ||
						next.second > std::max(from.second, to.second) + margin ||
						(next != to && blocked && blocked(next)))
					continue;
				const double next_cost = cost + std::hypot(double(dx), double(dz)) +
										 (turn != 0 ? 0.25 : 0.0);
				const State next_state{next.first, next.second, direction};
				auto [it, inserted] = visits.emplace(next_state, Visit{next_cost, state});
				if (!inserted && next_cost >= it->second.cost)
					continue;
				it->second = {next_cost, state};
				queue.emplace(
						next_cost + heuristic_weight * distance(next.first, next.second),
						next_cost, next_state);
			}
		}
	}
	return {};
}

namespace
{
bool track_way(const ProcessedWay &way)
{
	static const std::set<std::string> types{"rail", "light_rail", "subway", "tram",
			"narrow_gauge", "monorail", "funicular", "miniature", "preserved", "disused"};
	return way.nodes.size() >= 2 && types.contains(way.tags.get("railway")) &&
		   way.tags.get("area") != "yes";
}

bool underground(const ProcessedWay &way)
{
	return way.tags.get("railway") == "subway" || way.tags.get("subway") == "yes" ||
		   way.tags.get("tunnel") == "yes";
}

bool bridge(const ProcessedWay &way)
{
	const auto tag = way.tags.get("bridge");
	return !underground(way) && way.tags.get("indoor") != "yes" && !tag.empty() &&
		   tag != "no";
}

std::string level(const ProcessedWay &way)
{
	return (underground(way)	 ? "tunnel:"
				   : bridge(way) ? "bridge:"
								 : "surface:") +
		   (way.tags.get("layer").empty() ? "0" : way.tags.get("layer"));
}

int level_offset(const ProcessedWay &way)
{
	const auto text = way.tags.get("layer");
	int layer = underground(way) ? -1 : bridge(way) ? 1 : 0;
	int parsed = 0;
	const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
	if (result.ec == std::errc{} && result.ptr == text.data() + text.size())
		layer = std::clamp(parsed, -100, 100);
	// Layer is an ordering tag, not metres; allow a full train clearance per
	// level when no elevation is supplied by the bridge/tunnel geometry.
	return underground(way) ? 8 * std::min(layer, -1)
		   : bridge(way)	? 8 * std::max(layer, 1)
							: 0;
}

// Explicit OSM vertices connect bridge/tunnel approaches across layer tags.
// Mere XZ overlap must not connect a flyover to the railway below it.
using AnchorKey = std::tuple<std::uint64_t, pos_t, pos_t, std::string>;
AnchorKey anchor_key(const ProcessedNode &node, const ProcessedWay &way)
{
	return {node.id, node.x, node.z, node.id ? "" : level(way)};
}

struct Anchor
{
	XZ position;
	std::map<std::size_t, int> ports;
};
struct Segment
{
	std::size_t a, b;
	std::string level;
	std::vector<XZ> line;
	std::vector<int> heights;
	std::vector<Block> rails;
};

// Fit the incident bearings to an actual registered flat track. Routing then
// meets these ports exactly instead of drawing a straight over a broken switch.
bool assign_ports(Anchor &anchor, const std::vector<Anchor> &anchors,
		const std::array<std::vector<std::vector<int>>, 5> &templates)
{
	const auto count = anchor.ports.size();
	if (count == 0 || count > 4)
		return false;
	std::vector<int> desired;
	for (const auto &[other, unused] : anchor.ports) {
		const auto &p = anchors[other].position;
		desired.push_back(closest_direction(
				p.first - anchor.position.first, p.second - anchor.position.second));
	}
	int best_cost = std::numeric_limits<int>::max();
	std::vector<int> best;
	for (auto ports : templates[count]) {
		do {
			int cost = 0;
			for (std::size_t i = 0; i < count; ++i) {
				const int delta = std::abs(ports[i] - desired[i]);
				const int angle = std::min(delta, 16 - delta);
				cost += angle * angle;
			}
			if (cost < best_cost) {
				best_cost = cost;
				best = ports;
			}
		} while (std::next_permutation(ports.begin(), ports.end()));
	}
	if (best.empty())
		return false;
	std::size_t i = 0;
	for (auto &[other, direction] : anchor.ports)
		direction = best[i++];
	return true;
}
} // namespace

void prepare_network(const std::vector<ProcessedElement> &elements, WorldEditor &editor)
{
	plans.clear();
	generated.clear();
	tunnel_floors.clear();
	if (!available())
		return;
	std::map<std::uint64_t, const ProcessedWay *> ways;
	std::map<AnchorKey, std::set<std::uint64_t>> users;
	for (const auto &element : elements)
		if (element.is_way() && track_way(element.as_way())) {
			const auto &way = element.as_way();
			ways.emplace(way.id, &way);
			for (const auto &node : way.nodes)
				users[anchor_key(node, way)].insert(way.id);
		}
	std::map<AnchorKey, std::size_t> anchor_ids;
	if (ways.empty())
		return;
	std::vector<Anchor> anchors;
	std::map<std::uint64_t, std::vector<std::size_t>> way_anchors;
	for (const auto &[id, way] : ways) {
		auto &ids = way_anchors[id];
		for (std::size_t i = 0; i < way->nodes.size(); ++i) {
			const auto &node = way->nodes[i];
			const auto key = anchor_key(node, *way);
			const XZ p{node.x, node.z};
			if (!ids.empty()) {
				const auto &previous = anchors[ids.back()].position;
				if (p == previous)
					continue;
				// Dense shape points do not leave enough room for curve assets.
				// Preserve topology anchors and the end; round only ordinary points.
				if (i + 1 < way->nodes.size() && users[key].size() == 1 &&
						std::hypot(double(p.first - previous.first),
								double(p.second - previous.second)) < 8)
					continue;
			}
			auto [it, inserted] = anchor_ids.emplace(key, anchors.size());
			if (inserted)
				anchors.push_back({p, {}});
			ids.push_back(it->second);
		}
		for (std::size_t i = 1; i < ids.size(); ++i) {
			anchors[ids[i - 1]].ports.emplace(ids[i], 0);
			anchors[ids[i]].ports.emplace(ids[i - 1], 0);
		}
	}
	std::set<std::size_t> invalid;
	std::array<std::vector<std::vector<int>>, 5> templates;
	for (unsigned mask = 1; mask < (1u << 16); ++mask) {
		const auto count = std::popcount(mask);
		if (count > 4)
			continue;
		DirectionSet directions{};
		std::vector<int> ports;
		for (int d = 0; d < 16; ++d)
			if (mask & (1u << d)) {
				directions[d] = true;
				ports.push_back(d);
			}
		if (rail_for_directions(directions))
			templates[count].push_back(std::move(ports));
	}
	for (std::size_t i = 0; i < anchors.size(); ++i)
		if (!assign_ports(anchors[i], anchors, templates))
			invalid.insert(i);
	using SegmentKey = std::tuple<std::size_t, std::size_t, std::string>;
	std::map<SegmentKey, Segment> segments;
	std::map<std::string, std::set<XZ>> occupied;
	for (const auto &[id, ids] : way_anchors)
		for (auto a : ids)
			occupied[level(*ways.at(id))].insert(anchors[a].position);
	for (const auto &[id, ids] : way_anchors) {
		const auto &way = *ways.at(id);
		for (std::size_t i = 1; i < ids.size(); ++i) {
			const auto [a, b] = std::minmax(ids[i - 1], ids[i]);
			const SegmentKey key{a, b, level(way)};
			auto [it, inserted] =
					segments.try_emplace(key, Segment{a, b, level(way), {}, {}, {}});
			auto &segment = it->second;
			if (inserted && !invalid.contains(a) && !invalid.contains(b)) {
				segment.line = route(anchors[a].position, anchors[b].position,
						anchors[a].ports.at(b), (anchors[b].ports.at(a) + 8) % 16,
						[&](const XZ &p) { return occupied[segment.level].contains(p); });
				for (const auto &p : segment.line)
					occupied[segment.level].insert(p);
			}
			if (segment.line.empty()) {
				errorstream << "Advtrains: cannot route way " << id
							<< " between OSM anchors using available tracks" << std::endl;
				continue;
			}
			std::vector<int> seed;
			for (const auto &[x, z] : segment.line)
				seed.push_back(editor.get_ground_level(x, z) +
							   (underground(way) ? level_offset(way) : 0));
			if (bridge(way)) {
				const auto [lo, hi] = std::minmax_element(seed.begin(), seed.end());
				const int deck =
						*hi + (*hi - *lo < 4 ? level_offset(way) : level_offset(way) - 8);
				std::fill(seed.begin(), seed.end(), deck);
			}
			if (segment.heights.empty())
				segment.heights = std::move(seed);
			else
				for (std::size_t j = 0; j < seed.size(); ++j)
					segment.heights[j] = std::max(segment.heights[j], seed[j]);
		}
	}
	std::vector<int> anchor_heights(anchors.size(), std::numeric_limits<int>::min());
	bool changed;
	do {
		changed = false;
		for (auto &[key, s] : segments) {
			if (s.line.empty())
				continue;
			for (const auto [a, i] :
					{std::pair{s.a, std::size_t{0}}, std::pair{s.b, s.line.size() - 1}}) {
				const int height = std::max(anchor_heights[a], s.heights[i]);
				changed |= anchor_heights[a] != height || s.heights[i] != height;
				anchor_heights[a] = s.heights[i] = height;
			}
			const auto before = s.heights;
			std::vector<bool> flat(s.line.size(), false);
			flat.front() = flat.back() = true;
			fit_profile(s.line, s.heights, flat);
			changed |= before != s.heights;
		}
	} while (changed);
	for (auto &[key, s] : segments) {
		for (std::size_t i = 0; i < s.line.size(); ++i) {
			std::optional<Block> rail;
			if (i == 0 || i + 1 == s.line.size()) {
				DirectionSet directions{};
				for (const auto &[other, direction] : anchors[i == 0 ? s.a : s.b].ports)
					directions[direction] = true;
				rail = rail_for_directions(directions);
			} else {
				rail = slope_rail(s.line, s.heights, i);
				if (!rail)
					rail = connected_rail(s.line, i);
			}
			if (!rail) {
				s.rails.clear();
				break;
			}
			s.rails.push_back(*rail);
		}
	}
	for (const auto &[id, ids] : way_anchors) {
		auto &plan = plans[id];
		for (std::size_t i = 1; i < ids.size(); ++i) {
			const auto [a, b] = std::minmax(ids[i - 1], ids[i]);
			const auto &s = segments.at({a, b, level(*ways.at(id))});
			if (s.rails.size() != s.line.size() || s.line.empty()) {
				plan = {};
				break;
			}
			for (std::size_t j = plan.line.empty() ? 0 : 1; j < s.line.size(); ++j) {
				const auto k = ids[i - 1] == a ? j : s.line.size() - j - 1;
				plan.line.push_back(s.line[k]);
				plan.heights.push_back(s.heights[k]);
				plan.rails.push_back(s.rails[k]);
			}
		}
	}
}

std::vector<XZ> build_centerline(const ProcessedWay &way)
{
	if (const auto *plan = get_plan(way))
		return plan->line;
	std::vector<XZ> line;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const XZ from{way.nodes[i - 1].x, way.nodes[i - 1].z};
		const XZ to{way.nodes[i].x, way.nodes[i].z};
		const int direction =
				closest_direction(to.first - from.first, to.second - from.second);
		auto part = route(from, to, direction, direction);
		if (part.empty())
			return {};
		line.insert(line.end(), part.begin() + (line.empty() ? 0 : 1), part.end());
	}
	return line;
}
} // namespace arnis::railways::advtrains
