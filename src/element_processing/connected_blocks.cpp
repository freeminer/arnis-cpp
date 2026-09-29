#include "connected_blocks.h"
#include "../../arnis_world_editor.h"
#include "../block_definitions.h"
#include <algorithm>
#include <cmath>

namespace arnis::connected_blocks
{
namespace
{
enum class Family
{
	None,
	Wall,
	Fence
};
Family family(const Block &b)
{
	if (b == IRON_BARS || b == COBBLESTONE_WALL || b == ANDESITE_WALL ||
			b == STONE_BRICK_WALL || b == BRICK_WALL)
		return Family::Wall;
	if (b == OAK_FENCE || b == EARTH_FENCE_CHAINLINK || b == EARTH_FENCE_BARBED ||
			b == EARTH_FENCE_PICKET || b == EARTH_FENCE_WROUGHT)
		return Family::Fence;
	return Family::None;
}
bool flag(bool v)
{
	return v;
}
void refresh(WorldEditor &e, const Block &b, int x, int y, int z)
{
	const auto own = family(b);
	const int dx[] = {0, 0, 1, -1};
	const int dz[] = {-1, 1, 0, 0};
	bool side[4]{};
	for (int i = 0; i < 4; ++i) {
		auto n = e.get_block_absolute(x + dx[i], y, z + dz[i]);
		side[i] = n && family(*n) == own;
	}
	BlockWithProperties joined =
			own == Family::Fence || b == IRON_BARS
					? connected_fence(b, side[0], side[1], side[2], side[3])
					: connected_wall(b, side[0], side[1], side[2], side[3]);
	e.set_block_with_properties_absolute(
			joined, x, y, z, std::optional<std::vector<Block>>{{b}}, std::nullopt);
}
}
bool is_connectable(const Block &b)
{
	return family(b) != Family::None;
}
void place_connected(WorldEditor &e, const Block &b, int x, int y, int z)
{
	e.set_block_absolute(b, x, y, z);
	if (!e.get_block_absolute(x, y, z) || !is_connectable(b))
		return;
	refresh(e, b, x, y, z);
	const int dx[] = {0, 0, 1, -1};
	const int dz[] = {-1, 1, 0, 0};
	for (int i = 0; i < 4; ++i) {
		auto n = e.get_block_absolute(x + dx[i], y, z + dz[i]);
		if (n && family(*n) == family(b))
			refresh(e, *n, x + dx[i], y, z + dz[i]);
	}
}
BlockWithProperties connected_fence(const Block &b, bool n, bool s, bool e, bool w)
{
	return {b,
			{{"north", flag(n) ? "true" : "false"}, {"south", flag(s) ? "true" : "false"},
					{"east", flag(e) ? "true" : "false"},
					{"west", flag(w) ? "true" : "false"}, {"waterlogged", "false"}}};
}
BlockWithProperties connected_wall(const Block &b, bool n, bool s, bool e, bool w)
{
	const bool straight = (n && s && !e && !w) || (e && w && !n && !s);
	return {b, {{"north", n ? "low" : "none"}, {"south", s ? "low" : "none"},
					   {"east", e ? "low" : "none"}, {"west", w ? "low" : "none"},
					   {"up", straight ? "false" : "true"}, {"waterlogged", "false"}}};
}
std::vector<std::pair<int, int>> stair_steps(
		std::pair<int, int> prev, std::pair<int, int> curr)
{
	std::vector<std::pair<int, int>> out;
	auto [x, z] = prev;
	while (x != curr.first || z != curr.second) {
		if (x != curr.first) {
			x += (curr.first > x) - (curr.first < x);
			out.emplace_back(x, z);
		}
		if (z != curr.second) {
			z += (curr.second > z) - (curr.second < z);
			out.emplace_back(x, z);
		}
	}
	if (out.empty())
		out.push_back(curr);
	return out;
}
std::vector<std::pair<int, int>> four_connected_line(
		const std::vector<std::pair<int, int>> &points)
{
	std::vector<std::pair<int, int>> out;
	for (auto p : points) {
		if (out.empty())
			out.push_back(p);
		else if (out.back() != p) {
			auto v = stair_steps(out.back(), p);
			out.insert(out.end(), v.begin(), v.end());
		}
	}
	return out;
}
std::vector<std::pair<int, int>> cross_cells(
		int x, int z, std::pair<float, float> p, int from, int to)
{
	std::vector<std::pair<int, int>> points;
	for (int o = from; o <= to; ++o)
		points.emplace_back(
				int(std::lround(x + p.first * o)), int(std::lround(z + p.second * o)));
	return four_connected_line(points);
}
}
