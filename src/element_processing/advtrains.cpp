#include "advtrains.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "../block_definitions.h"

namespace arnis::railways::advtrains
{
using namespace block_definitions;
using XZ = std::pair<int, int>;

struct XZHash
{
	std::size_t operator()(const XZ &p) const noexcept
	{
		return std::hash<long long>{}((static_cast<long long>(p.first) << 32) ^
				static_cast<unsigned int>(p.second));
	}
};

constexpr std::array<XZ, 16> DIR_VECTORS{{
		{0, 1}, {1, 2}, {1, 1}, {2, 1}, {1, 0}, {2, -1}, {1, -1}, {1, -2},
		{0, -1}, {-1, -2}, {-1, -1}, {-2, -1}, {-1, 0}, {-2, 1}, {-1, 1},
		{-1, 2}}};

using DirectionSet = std::array<bool, 16>;
thread_local std::unordered_map<XZ, DirectionSet, XZHash> network_connections;
thread_local std::unordered_map<XZ, int, XZHash> network_heights;
thread_local std::unordered_set<XZ, XZHash> network_anchors;

bool available()
{
	return ADVTRAINS_AVAILABLE;
}

int circular_delta(int from, int to)
{
	int delta = (to - from + 16) % 16;
	if (delta > 8)
		delta -= 16;
	return delta;
}

int closest_direction(int dx, int dz)
{
	if (!dx && !dz)
		return 0;
	const double length = std::hypot(static_cast<double>(dx), static_cast<double>(dz));
	int best = 0;
	double best_dot = -std::numeric_limits<double>::infinity();
	for (int direction = 0; direction < 16; ++direction) {
		const auto [vx, vz] = DIR_VECTORS[direction];
		const double dot = (dx * vx + dz * vz) /
				(length * std::hypot(static_cast<double>(vx), static_cast<double>(vz)));
		if (dot > best_dot) {
			best_dot = dot;
			best = direction;
		}
	}
	return best;
}

std::optional<int> direction_between(const XZ &from, const XZ &to)
{
	const XZ delta{to.first - from.first, to.second - from.second};
	for (int direction = 0; direction < 16; ++direction)
		if (DIR_VECTORS[direction] == delta)
			return direction;
	return std::nullopt;
}

std::vector<XZ> build_centerline(const ProcessedWay &way)
{
	std::vector<XZ> out;
	if (way.nodes.empty())
		return out;
	XZ current{way.nodes.front().x, way.nodes.front().z};
	out.push_back(current);
	std::optional<int> heading;
	for (std::size_t target_index = 1; target_index < way.nodes.size(); ++target_index) {
		const XZ target{way.nodes[target_index].x, way.nodes[target_index].z};
		const bool must_hit_target = target_index + 1 == way.nodes.size() ||
				network_anchors.contains(target);
		if (must_hit_target)
			heading.reset();
		const std::size_t max_steps = 64 +
				static_cast<std::size_t>(std::abs(target.first - current.first) +
						std::abs(target.second - current.second)) * 8;
		for (std::size_t step = 0; step < max_steps; ++step) {
			const int dx = target.first - current.first;
			const int dz = target.second - current.second;
			const int distance2 = dx * dx + dz * dz;
			// Ordinary shape points may be rounded for a smooth heading change.
			// Shared OSM vertices are topology anchors and must be reached exactly,
			// otherwise a branch can stop beside the main line instead of on it.
			if (distance2 == 0 || (!must_hit_target && distance2 <= 4))
				break;
			const int desired = closest_direction(dx, dz);
			int chosen = desired;
			if (heading) {
				const int turn = circular_delta(*heading, desired);
				chosen = (*heading + (turn > 0 ? 1 : turn < 0 ? -1 : 0) + 16) % 16;
			}
			const auto [vx, vz] = DIR_VECTORS[chosen];
			current = {current.first + vx, current.second + vz};
			if (out.back() != current)
				out.push_back(current);
			heading = chosen;
		}
	}
	return out;
}

bool is_track_way(const ProcessedWay &way)
{
	static const std::unordered_set<std::string> types{"rail", "light_rail", "subway",
			"tram", "narrow_gauge", "monorail", "funicular", "miniature", "preserved",
			"disused"};
	return way.nodes.size() >= 2 && types.contains(way.tags.get("railway")) &&
			way.tags.get("area") != "yes";
}

bool is_at_grade(const ProcessedWay &way)
{
	const auto railway = way.tags.get("railway");
	return railway != "subway" && way.tags.get("subway") != "yes" &&
			way.tags.get("tunnel") != "yes" && way.tags.get("bridge") != "yes" &&
			way.tags.get("bridge") != "viaduct";
}

void prepare_network(
		const std::vector<ProcessedElement> &elements, WorldEditor &editor)
{
	network_connections.clear();
	network_heights.clear();
	network_anchors.clear();
	if (!available())
		return;
	std::unordered_map<XZ, std::size_t, XZHash> vertex_ways;
	for (const auto &element : elements) {
		if (!element.is_way() || !is_track_way(element.as_way()) ||
				!is_at_grade(element.as_way()))
			continue;
		std::unordered_set<XZ, XZHash> seen_in_way;
		for (const auto &node : element.as_way().nodes)
			seen_in_way.emplace(node.x, node.z);
		for (const auto &position : seen_in_way)
			++vertex_ways[position];
	}
	for (const auto &[position, ways] : vertex_ways)
		if (ways > 1)
			network_anchors.insert(position);

	std::vector<std::vector<XZ>> lines;
	for (const auto &element : elements) {
		if (!element.is_way() || !is_track_way(element.as_way()) ||
				!is_at_grade(element.as_way()))
			continue;
		auto line = build_centerline(element.as_way());
		for (std::size_t i = 1; i < line.size(); ++i) {
			const auto direction = direction_between(line[i - 1], line[i]);
			if (!direction)
				continue;
			network_connections[line[i - 1]][*direction] = true;
			network_connections[line[i]][(*direction + 8) % 16] = true;
		}
		lines.push_back(std::move(line));
	}

	// Reconcile profiles after collecting the complete topology. Doing this in
	// the element loop makes the result depend on OSM way order and lets ways
	// choose different elevations for a shared turnout. Whole-line rounds only
	// visit rail cells and propagate a raised junction through its slope pair.
	for (int round = 0; round < 4; ++round) {
		auto reconciled = network_heights;
		bool changed = false;
		for (const auto &line : lines) {
			const auto heights = height_profile(editor, line);
			for (std::size_t i = 0; i < line.size() && i < heights.size(); ++i) {
				const auto old = network_heights.find(line[i]);
				changed |= old == network_heights.end() || heights[i] > old->second;
				auto [entry, inserted] = reconciled.emplace(line[i], heights[i]);
				if (!inserted)
					entry->second = std::max(entry->second, heights[i]);
			}
		}
		network_heights.swap(reconciled);
		if (!changed)
			break;
	}
}

bool connections_match(int a, int b, int c, int d)
{
	return (a == c && b == d) || (a == d && b == c);
}

std::optional<Block> two_connection_rail(int first, int second)
{
	const std::array<Block, 4> straights{{ADV_RAIL_STRAIGHT_0,
			ADV_RAIL_STRAIGHT_30, ADV_RAIL_STRAIGHT_45, ADV_RAIL_STRAIGHT_60}};
	const std::array<Block, 4> curves{{ADV_RAIL_CURVE_0, ADV_RAIL_CURVE_30,
			ADV_RAIL_CURVE_45, ADV_RAIL_CURVE_60}};
	for (int suffix = 0; suffix < 4; ++suffix)
		for (int param2 = 0; param2 < 4; ++param2) {
			const int base = (suffix + param2 * 4) % 16;
			if (connections_match(first, second, base, (base + 8) % 16)) {
				Block rail = straights[suffix];
				rail.setParam2(param2);
				return rail;
			}
			if (connections_match(first, second, base, (base + 7) % 16)) {
				Block rail = curves[suffix];
				rail.setParam2(param2);
				return rail;
			}
		}
	return std::nullopt;
}

bool set_equals(const DirectionSet &directions, std::initializer_list<int> expected)
{
	DirectionSet wanted{};
	for (int direction : expected)
		wanted[(direction + 16) % 16] = true;
	return directions == wanted;
}

std::optional<Block> junction_rail(const DirectionSet &directions)
{
	if (!ADVTRAINS_JUNCTIONS_AVAILABLE)
		return std::nullopt;
	for (int suffix = 0; suffix < 4; ++suffix)
		for (int param2 = 0; param2 < 4; ++param2) {
			const int base = (suffix + param2 * 4) % 16;
			if (set_equals(directions, {base, base + 8, base + 7})) {
				Block rail = ADV_RAIL_SWITCH_LEFT_STRAIGHT[suffix];
				rail.setParam2(param2);
				return rail;
			}
			if (set_equals(directions, {base, base + 8, base + 9})) {
				Block rail = ADV_RAIL_SWITCH_RIGHT_STRAIGHT[suffix];
				rail.setParam2(param2);
				return rail;
			}
			if (set_equals(directions, {base, base + 7, base + 9})) {
				Block rail = ADV_RAIL_Y_TURNOUT[suffix];
				rail.setParam2(param2);
				return rail;
			}
			if (set_equals(directions, {base, base + 7, base + 8, base + 9})) {
				Block rail = ADV_RAIL_THREE_WAY_STRAIGHT[suffix];
				rail.setParam2(param2);
				return rail;
			}
			if (set_equals(directions, {base, base + 8, base + 4, base + 12})) {
				Block rail = ADV_RAIL_PERP_CROSSING[suffix];
				rail.setParam2(param2);
				return rail;
			}
		}
	if (ADVTRAINS_CROSSINGS_AVAILABLE) {
		constexpr std::array<int, 6> ninety_delta{{1, 2, 3, 5, 6, 7}};
		for (int rotation = 0; rotation < 4; ++rotation)
			for (std::size_t variant = 0; variant < ninety_delta.size(); ++variant) {
				const int base = rotation * 4;
				const int branch = base + ninety_delta[variant];
				if (set_equals(directions,
							{base, base + 8, branch, branch + 8})) {
					Block rail = ADV_RAIL_90_PLUS_CROSSING[variant];
					rail.setParam2(rotation);
					return rail;
				}
			}
		constexpr std::array<std::pair<int, int>, 7> diagonal_axes{{
				{1, 6}, {1, 3}, {3, 6}, {3, 5}, {2, 5}, {5, 7}, {2, 7}}};
		for (int rotation = 0; rotation < 4; ++rotation)
			for (std::size_t variant = 0; variant < diagonal_axes.size(); ++variant) {
				const auto [first, second] = diagonal_axes[variant];
				if (set_equals(directions, {first + rotation * 4,
							first + rotation * 4 + 8, second + rotation * 4,
							second + rotation * 4 + 8})) {
					Block rail = ADV_RAIL_DIAGONAL_CROSSING[variant];
					rail.setParam2(rotation);
					return rail;
				}
			}
	}
	return std::nullopt;
}

std::optional<Block> connected_rail(
		const std::vector<XZ> &line, std::size_t index, bool use_network)
{
	if (!available() || line.size() < 2 || index >= line.size())
		return std::nullopt;
	DirectionSet directions{};
	if (use_network)
		if (const auto found = network_connections.find(line[index]);
				found != network_connections.end())
			directions = found->second;
	if (index > 0)
		if (auto forward = direction_between(line[index - 1], line[index]))
			directions[(*forward + 8) % 16] = true;
	if (index + 1 < line.size())
		if (auto outgoing = direction_between(line[index], line[index + 1]))
			directions[*outgoing] = true;
	if (auto junction = junction_rail(directions))
		return junction;
	std::vector<int> present;
	for (int direction = 0; direction < 16; ++direction)
		if (directions[direction])
			present.push_back(direction);
	if (present.size() >= 2)
		if (auto exact = two_connection_rail(present[0], present[1]))
			return exact;
	if (present.empty())
		return std::nullopt;
	return two_connection_rail((present.front() + 8) % 16, present.front());
}

std::vector<int> height_profile(
		WorldEditor &editor, const std::vector<XZ> &line)
{
	std::vector<int> profile;
	profile.reserve(line.size());
	for (const auto &[x, z] : line)
		profile.push_back(editor.get_ground_level(x, z));
	for (std::size_t i = 0; i < line.size(); ++i)
		if (const auto found = network_heights.find(line[i]); found != network_heights.end())
			profile[i] = std::max(profile[i], found->second);
	// Advtrains ramps occupy consecutive cells at the lower node Y, followed by
	// level track one node up. Prefer its gentler three-piece family and retain
	// the two-piece family as a compatibility fallback. Raise the formation until
	// each transition has enough collinear runway. Non-cardinal pieces stay level.
	const std::size_t ramp_cells = ADVTRAINS_GENTLE_SLOPES_AVAILABLE ? 3 : 2;
	bool changed;
	do {
		changed = false;
		// A turnout or crossing must remain a flat rail node. Make it a local
		// high point so any required ramp starts on an incident branch instead
		// of replacing the junction itself with vst1/vst2.
		for (std::size_t i = 0; i < profile.size(); ++i) {
			const auto found = network_connections.find(line[i]);
			if (found == network_connections.end() ||
					std::count(found->second.begin(), found->second.end(), true) <= 2)
				continue;
			int level = profile[i];
			if (i > 0)
				level = std::max(level, profile[i - 1]);
			if (i + 1 < profile.size())
				level = std::max(level, profile[i + 1]);
			changed |= profile[i] != level;
			profile[i] = level;
		}
		for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
			const auto direction = direction_between(line[i], line[i + 1]);
			if (!direction || (*direction % 4) != 0) {
				const int level = std::max(profile[i], profile[i + 1]);
				changed |= profile[i] != level || profile[i + 1] != level;
				profile[i] = profile[i + 1] = level;
				continue;
			}
			if (profile[i + 1] > profile[i]) {
				bool has_lead_in = true;
				for (std::size_t offset = 1; offset < ramp_cells; ++offset)
					has_lead_in &= i >= offset && profile[i - offset] == profile[i] &&
							direction_between(line[i - offset], line[i - offset + 1]) ==
									direction;
				if (!has_lead_in) {
					profile[i] = profile[i + 1];
					changed = true;
				}
			} else if (profile[i] > profile[i + 1]) {
				bool has_run_out = true;
				for (std::size_t offset = 1; offset < ramp_cells; ++offset)
					has_run_out &= i + 1 + offset < profile.size() &&
							profile[i + 1] == profile[i + 1 + offset] &&
							direction_between(line[i + offset], line[i + 1 + offset]) ==
									direction;
				if (!has_run_out) {
					profile[i + 1] = profile[i];
					changed = true;
				}
			}
			if (std::abs(profile[i + 1] - profile[i]) > 1) {
				const int level = std::max(profile[i], profile[i + 1]);
				profile[i] = profile[i + 1] = level;
				changed = true;
			}
		}
	} while (changed);
	return profile;
}

std::optional<Block> slope_rail(const std::vector<XZ> &line,
		const std::vector<int> &heights, std::size_t index)
{
	if (!ADVTRAINS_SLOPES_AVAILABLE || index >= line.size() || index >= heights.size())
		return std::nullopt;
	if (const auto found = network_connections.find(line[index]);
			found != network_connections.end() &&
			std::count(found->second.begin(), found->second.end(), true) > 2)
		return std::nullopt;

	const std::size_t ramp_cells = ADVTRAINS_GENTLE_SLOPES_AVAILABLE ? 3 : 2;
	for (std::size_t piece = 0; piece < ramp_cells; ++piece) {
		// Ascending in vector order: [h:vst31, h:vst32, h:vst33,
		// h+1:level], or the equivalent two-piece fallback.
		if (index >= piece) {
			const std::size_t start = index - piece;
			if (start + ramp_cells < line.size()) {
				const int low = heights[start];
				bool matches = heights[start + ramp_cells] == low + 1;
				std::optional<int> direction;
				for (std::size_t offset = 0; matches && offset < ramp_cells; ++offset) {
					matches &= heights[start + offset] == low;
					const auto edge = direction_between(
							line[start + offset], line[start + offset + 1]);
					if (!direction)
						direction = edge;
					matches &= edge && edge == direction;
				}
				if (matches && direction && (*direction % 4) == 0) {
					Block slope = ADVTRAINS_GENTLE_SLOPES_AVAILABLE
							? ADV_RAIL_GENTLE_SLOPE[piece]
							: (piece == 0 ? ADV_RAIL_SLOPE_UP : ADV_RAIL_SLOPE_DOWN);
					slope.setParam2(static_cast<std::uint8_t>(*direction / 4));
					return slope;
				}
			}
		}

		// Descending in vector order uses the pieces in reverse order, all
		// oriented back toward the preceding high level node.
		if (index > piece) {
			const std::size_t start = index - piece - 1;
			if (start + ramp_cells < line.size()) {
				const int low = heights[start + 1];
				bool matches = heights[start] == low + 1;
				std::optional<int> travel;
				for (std::size_t offset = 0; matches && offset < ramp_cells; ++offset) {
					matches &= heights[start + 1 + offset] == low;
					const auto edge = direction_between(
							line[start + offset], line[start + offset + 1]);
					if (!travel)
						travel = edge;
					matches &= edge && edge == travel;
				}
				if (matches && travel && (*travel % 4) == 0) {
					const std::size_t reversed = ramp_cells - piece - 1;
					Block slope = ADVTRAINS_GENTLE_SLOPES_AVAILABLE
							? ADV_RAIL_GENTLE_SLOPE[reversed]
							: (reversed == 0 ? ADV_RAIL_SLOPE_UP
											 : ADV_RAIL_SLOPE_DOWN);
					const int direction_to_high = (*travel + 8) % 16;
					slope.setParam2(static_cast<std::uint8_t>(direction_to_high / 4));
					return slope;
				}
			}
		}
	}
	return std::nullopt;
}

} // namespace arnis::railways::advtrains
