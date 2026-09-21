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
const std::array<XZ, 16> DIR_VECTORS{
		{{0, 1}, {1, 2}, {1, 1}, {2, 1}, {1, 0}, {2, -1}, {1, -1}, {1, -2}, {0, -1},
				{-1, -2}, {-1, -1}, {-2, -1}, {-1, 0}, {-2, 1}, {-1, 1}, {-1, 2}}};

bool available()
{
	return ADVTRAINS_AVAILABLE;
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
		const double dot =
				(dx * vx + dz * vz) /
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

bool connections_match(int a, int b, int c, int d)
{
	return (a == c && b == d) || (a == d && b == c);
}

std::optional<Block> two_connection_rail(int first, int second)
{
	const std::array<Block, 4> straights{{ADV_RAIL_STRAIGHT_0, ADV_RAIL_STRAIGHT_30,
			ADV_RAIL_STRAIGHT_45, ADV_RAIL_STRAIGHT_60}};
	const std::array<Block, 4> curves{
			{ADV_RAIL_CURVE_0, ADV_RAIL_CURVE_30, ADV_RAIL_CURVE_45, ADV_RAIL_CURVE_60}};
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
				if (set_equals(directions, {base, base + 8, branch, branch + 8})) {
					Block rail = ADV_RAIL_90_PLUS_CROSSING[variant];
					rail.setParam2(rotation);
					return rail;
				}
			}
		constexpr std::array<std::pair<int, int>, 7> diagonal_axes{
				{{1, 6}, {1, 3}, {3, 6}, {3, 5}, {2, 5}, {5, 7}, {2, 7}}};
		for (int rotation = 0; rotation < 4; ++rotation)
			for (std::size_t variant = 0; variant < diagonal_axes.size(); ++variant) {
				const auto [first, second] = diagonal_axes[variant];
				if (set_equals(directions,
							{first + rotation * 4, first + rotation * 4 + 8,
									second + rotation * 4, second + rotation * 4 + 8})) {
					Block rail = ADV_RAIL_DIAGONAL_CROSSING[variant];
					rail.setParam2(rotation);
					return rail;
				}
			}
	}
	return std::nullopt;
}

std::optional<Block> rail_for_directions(const DirectionSet &directions)
{
	if (auto junction = junction_rail(directions))
		return junction;
	std::vector<int> present;
	for (int direction = 0; direction < 16; ++direction)
		if (directions[direction])
			present.push_back(direction);
	if (present.size() == 2)
		return two_connection_rail(present[0], present[1]);
	if (present.size() == 1)
		return two_connection_rail((present.front() + 8) % 16, present.front());
	// Unsupported geometry must be rerouted, never replaced with a straight.
	return std::nullopt;
}

std::optional<Block> connected_rail(const std::vector<XZ> &line, std::size_t index, bool)
{
	if (!available() || index >= line.size())
		return std::nullopt;
	DirectionSet directions{};
	if (index > 0)
		if (auto d = direction_between(line[index], line[index - 1]))
			directions[*d] = true;
	if (index + 1 < line.size())
		if (auto d = direction_between(line[index], line[index + 1]))
			directions[*d] = true;
	return rail_for_directions(directions);
}

std::size_t slope_length(int direction)
{
	if (direction % 4 == 0)
		return ADVTRAINS_GENTLE_SLOPES_AVAILABLE ? 3 : ADVTRAINS_SLOPES_AVAILABLE ? 2 : 0;
	return direction % 4 == 2 && ADVTRAINS_DIAGONAL_SLOPES_AVAILABLE ? 2 : 0;
}

// The maximum input height bounds convergence. Never clamp afterwards:
// doing so destroys complete ramps and recreates disconnected height jumps.
void fit_profile(const std::vector<XZ> &line, std::vector<int> &profile,
		const std::vector<bool> &flat)
{
	bool changed;
	do {
		changed = false;
		for (std::size_t i = 0; i + 1 < profile.size(); ++i) {
			if (profile[i] == profile[i + 1])
				continue;
			const bool ascending = profile[i] < profile[i + 1];
			const std::size_t low_index = ascending ? i : i + 1;
			const int low = profile[low_index];
			const int high = profile[ascending ? i + 1 : i];
			const auto dir = direction_between(line[i], line[i + 1]);
			const std::size_t count = dir ? slope_length(*dir) : 0;
			bool valid = count && high == low + 1;
			for (std::size_t offset = 0; valid && offset < count; ++offset) {
				if ((ascending && i < offset) ||
						(!ascending && i + 1 + offset >= profile.size())) {
					valid = false;
					break;
				}
				const std::size_t cell = ascending ? i - offset : i + 1 + offset;
				valid = !flat[cell] && profile[cell] == low;
				if (cell > 0)
					valid &= direction_between(line[cell - 1], line[cell]) == dir;
				if (cell + 1 < line.size())
					valid &= direction_between(line[cell], line[cell + 1]) == dir;
				// The low end must not overlap an opposite ramp or a curve.
				if (offset + 1 == count) {
					for (std::size_t clearance = 1; clearance <= count; ++clearance) {
						if (ascending && cell >= clearance)
							valid &= profile[cell - clearance] <= low;
						if (!ascending && cell + clearance < profile.size())
							valid &= profile[cell + clearance] <= low;
					}
				}
			}
			if (!valid) {
				profile[low_index] = high - (high - low > 1 ? 1 : 0);
				changed = true;
			}
		}
	} while (changed);
}

std::vector<int> height_profile(WorldEditor &editor, const std::vector<XZ> &line)
{
	std::vector<int> heights;
	for (const auto &[x, z] : line)
		heights.push_back(editor.get_ground_level(x, z));
	fit_profile(line, heights, std::vector<bool>(line.size(), false));
	return heights;
}

std::optional<Block> slope_rail(
		const std::vector<XZ> &line, const std::vector<int> &heights, std::size_t index)
{
	if (line.size() != heights.size() || index >= line.size())
		return std::nullopt;
	for (std::size_t count : {2, 3})
		for (std::size_t piece = 0; piece < count; ++piece)
			for (bool ascending : {true, false}) {
				const std::size_t back = piece + (ascending ? 0 : 1);
				if (index < back)
					continue;
				const std::size_t start = index - back;
				if (start + count >= line.size())
					continue;
				const auto travel = direction_between(line[start], line[start + 1]);
				if (!travel || slope_length(*travel) != count)
					continue;
				const int low = heights[start + (ascending ? 0 : 1)];
				if (heights[start + (ascending ? count : 0)] != low + 1)
					continue;
				bool matches = true;
				for (std::size_t offset = 0; offset < count; ++offset) {
					matches &= heights[start + offset + (ascending ? 0 : 1)] == low;
					matches &= direction_between(line[start + offset],
									   line[start + offset + 1]) == travel;
				}
				if (!matches)
					continue;
				const int direction = ascending ? *travel : (*travel + 8) % 16;
				const std::size_t part = ascending ? piece : count - piece - 1;
				Block rail = direction % 4 == 2 ? ADV_RAIL_DIAGONAL_SLOPE[part]
							 : count == 3		? ADV_RAIL_GENTLE_SLOPE[part]
							 : part == 0		? ADV_RAIL_SLOPE_UP
												: ADV_RAIL_SLOPE_DOWN;
				rail.setParam2(direction / 4);
				return rail;
			}
	return std::nullopt;
}

} // namespace arnis::railways::advtrains
