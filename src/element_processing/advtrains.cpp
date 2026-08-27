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
		const bool final_target = target_index + 1 == way.nodes.size();
		const std::size_t max_steps = 64 +
				static_cast<std::size_t>(std::abs(target.first - current.first) +
						std::abs(target.second - current.second)) * 8;
		for (std::size_t step = 0; step < max_steps; ++step) {
			const int dx = target.first - current.first;
			const int dz = target.second - current.second;
			const int distance2 = dx * dx + dz * dz;
			// Intermediate OSM vertices may be rounded for a smooth heading change,
			// but the final vertex must be reached exactly. Railway ways commonly
			// meet only at endpoints; stopping one cell early loses the turnout.
			if (distance2 == 0 || (!final_target && distance2 <= 4))
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

void prepare_network(const std::vector<ProcessedElement> &elements)
{
	network_connections.clear();
	if (!available())
		return;
	for (const auto &element : elements) {
		if (!element.is_way() || !is_track_way(element.as_way()) ||
				!is_at_grade(element.as_way()))
			continue;
		const auto line = build_centerline(element.as_way());
		for (std::size_t i = 1; i < line.size(); ++i) {
			const auto direction = direction_between(line[i - 1], line[i]);
			if (!direction)
				continue;
			network_connections[line[i - 1]][*direction] = true;
			network_connections[line[i]][(*direction + 8) % 16] = true;
		}
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
	auto step_limit = [&line](std::size_t left, std::size_t right) {
		auto direction = direction_between(line[left], line[right]);
		return direction && (*direction % 4) == 0 ? 1 : 0;
	};
	for (std::size_t i = 1; i < profile.size(); ++i)
		profile[i] = std::max(profile[i], profile[i - 1] - step_limit(i - 1, i));
	for (std::size_t i = profile.size(); i-- > 1;)
		profile[i - 1] = std::max(profile[i - 1],
				profile[i] - step_limit(i - 1, i));
	return profile;
}

std::optional<Block> slope_rail(const std::vector<XZ> &line,
		const std::vector<int> &heights, std::size_t index)
{
	if (!ADVTRAINS_SLOPES_AVAILABLE || index >= line.size() || index >= heights.size())
		return std::nullopt;
	std::optional<int> direction_to_high;
	bool rising_forward = false;
	if (index + 1 < line.size() && heights[index + 1] == heights[index] + 1) {
		direction_to_high = direction_between(line[index], line[index + 1]);
		rising_forward = true;
	} else if (index > 0 && heights[index - 1] == heights[index] + 1) {
		direction_to_high = direction_between(line[index], line[index - 1]);
	}
	if (!direction_to_high || (*direction_to_high % 4) != 0)
		return std::nullopt;
	Block slope = rising_forward ? ADV_RAIL_SLOPE_UP : ADV_RAIL_SLOPE_DOWN;
	slope.setParam2(static_cast<std::uint8_t>(*direction_to_high / 4));
	return slope;
}

} // namespace arnis::railways::advtrains
