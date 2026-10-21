#include "connected_blocks.h"
#include "../../arnis_world_editor.h"
#include "../block_definitions.h"
#include <algorithm>
#include <cmath>
#include <string_view>

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
bool bars_or_pane(const Block &b, const WorldEditor *editor)
{
	if (b == IRON_BARS)
		return true;
	if (editor)
		if (const auto name = editor->block_name(b))
			return std::string_view(*name).ends_with("_pane");
	return false;
}
bool fence_gate(const Block &b, const WorldEditor *editor)
{
	if (!editor)
		return false;
	if (const auto name = editor->block_name(b))
		return std::string_view(*name).ends_with("_fence_gate");
	return false;
}
Family family(const Block &b, const WorldEditor *editor = nullptr)
{
	if (bars_or_pane(b, editor) || b == COBBLESTONE_WALL || b == ANDESITE_WALL ||
			b == STONE_BRICK_WALL || b == BRICK_WALL)
		return Family::Wall;
	if (b == OAK_FENCE || b == EARTH_FENCE_CHAINLINK || b == EARTH_FENCE_BARBED ||
			b == EARTH_FENCE_PICKET || b == EARTH_FENCE_WROUGHT)
		return Family::Fence;
	if (editor) {
		if (const auto name = editor->block_name(b)) {
			const std::string_view view(*name);
			if (view.ends_with("_wall"))
				return Family::Wall;
			if (view.ends_with("_fence"))
				return Family::Fence;
		}
	}
	return Family::None;
}
bool joins(const Block &own, const Block &neighbour, const WorldEditor &editor)
{
	return family(neighbour, &editor) == family(own, &editor) ||
		   (!bars_or_pane(own, &editor) && fence_gate(neighbour, &editor));
}
bool flag(bool v)
{
	return v;
}
void refresh(WorldEditor &e, const Block &b, int x, int y, int z)
{
	const auto own = family(b, &e);
	const int dx[] = {0, 0, 1, -1};
	const int dz[] = {-1, 1, 0, 0};
	bool side[4]{};
	for (int i = 0; i < 4; ++i) {
		auto n = e.get_block_absolute(x + dx[i], y, z + dz[i]);
		side[i] = n && joins(b, *n, e);
	}
	BlockWithProperties joined =
			own == Family::Fence || bars_or_pane(b, &e)
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
bool is_connectable(const WorldEditor &editor, const Block &b)
{
	return family(b, &editor) != Family::None;
}
void place_connected(WorldEditor &e, const Block &b, int x, int y, int z)
{
	e.set_block_absolute(b, x, y, z);
	if (!e.get_block_absolute(x, y, z) || family(b, &e) == Family::None)
		return;
	refresh(e, b, x, y, z);
	const int dx[] = {0, 0, 1, -1};
	const int dz[] = {-1, 1, 0, 0};
	for (int i = 0; i < 4; ++i) {
		auto n = e.get_block_absolute(x + dx[i], y, z + dz[i]);
		if (n && family(*n, &e) == family(b, &e))
			refresh(e, *n, x + dx[i], y, z + dz[i]);
	}
}
void place_gate(WorldEditor &e, const Block &gate, int x, int y, int z, bool along_x)
{
	const int dx = along_x ? 1 : 0, dz = along_x ? 0 : 1;
	auto wall = [&](int sign) {
		const auto neighbour = e.get_block_absolute(x + sign * dx, y, z + sign * dz);
		if (!neighbour)
			return false;
		const auto name = e.block_name(*neighbour);
		return name && std::string_view(*name).ends_with("_wall");
	};
	e.set_block_with_properties_absolute(
			BlockWithProperties{
					gate, {{"facing", along_x ? "north" : "east"}, {"open", "false"},
								  {"in_wall", wall(1) || wall(-1) ? "true" : "false"},
								  {"powered", "false"}}},
			x, y, z, std::nullopt, std::nullopt);
	if (e.get_block_absolute(x, y, z) != gate)
		return;
	const int dxs[] = {0, 0, 1, -1};
	const int dzs[] = {-1, 1, 0, 0};
	for (int i = 0; i < 4; ++i) {
		const auto neighbour = e.get_block_absolute(x + dxs[i], y, z + dzs[i]);
		if (neighbour && family(*neighbour, &e) != Family::None)
			refresh(e, *neighbour, x + dxs[i], y, z + dzs[i]);
	}
}
BlockWithProperties connected_fence(const Block &b, bool n, bool s, bool e, bool w)
{
	return {b,
			{{"north", flag(n) ? "true" : "false"}, {"south", flag(s) ? "true" : "false"},
					{"east", flag(e) ? "true" : "false"},
					{"west", flag(w) ? "true" : "false"}, {"waterlogged", "false"}}};
}
BlockWithProperties connected_iron_bars(bool n, bool s, bool e, bool w)
{
	return connected_fence(IRON_BARS, n, s, e, w);
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
