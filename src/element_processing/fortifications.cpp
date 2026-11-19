#include "fortifications.h"

#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace arnis::fortifications
{
namespace
{
using Cell = std::pair<int, int>;

struct CellHash
{
	std::size_t operator()(const Cell &cell) const noexcept
	{
		const auto packed =
				(std::uint64_t(static_cast<std::uint32_t>(cell.first)) << 32) |
				static_cast<std::uint32_t>(cell.second);
		return std::hash<std::uint64_t>{}(packed);
	}
};

using CellSet = std::unordered_set<Cell, CellHash>;
using CellMap = std::unordered_map<Cell, int, CellHash>;

std::string_view tag(const tags_t &tags, std::string_view key)
{
	const auto it = tags.find(std::string(key));
	return it == tags.end() ? std::string_view{} : std::string_view(it->second);
}

bool defence(const tags_t &tags)
{
	const auto historic = tag(tags, "historic");
	const auto wall = tag(tags, "wall");
	return historic == "citywalls" || historic == "city_wall" ||
		   historic == "castle_wall" || wall == "citywalls" || wall == "city_wall" ||
		   wall == "castle_wall" || tag(tags, "barrier") == "city_wall";
}

CellMap passage_clearances(
		const CellSet &own, const CellSet &passages, int height, double scale)
{
	constexpr int arch_rise = 2;
	constexpr double passage_metres = 4.0;
	CellMap steps;
	std::deque<Cell> queue;
	constexpr std::array<Cell, 4> neighbours = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
	for (const auto &cell : passages) {
		if (!own.contains(cell))
			continue;
		const bool jamb =
				std::any_of(neighbours.begin(), neighbours.end(), [&](const Cell &d) {
					const Cell n{cell.first + d.first, cell.second + d.second};
					return own.contains(n) && !passages.contains(n);
				});
		if (jamb) {
			steps.emplace(cell, 0);
			queue.push_back(cell);
		}
	}
	while (!queue.empty()) {
		const Cell cell = queue.front();
		queue.pop_front();
		const int distance = steps.at(cell);
		if (distance >= arch_rise)
			continue;
		for (const auto &d : neighbours) {
			const Cell next{cell.first + d.first, cell.second + d.second};
			if (own.contains(next) && passages.contains(next) && !steps.contains(next)) {
				steps.emplace(next, distance + 1);
				queue.push_back(next);
			}
		}
	}

	const int sides = std::max(2, static_cast<int>(std::lround(passage_metres * scale)));
	CellMap clearances;
	for (const auto &cell : passages) {
		if (!own.contains(cell))
			continue;
		const auto it = steps.find(cell);
		const int clear = sides + (it == steps.end() ? arch_rise : it->second);
		const int adjusted = height - clear >= 2   ? clear
							 : height - sides >= 2 ? height - 2
												   : height;
		clearances.emplace(cell, adjusted);
	}
	return clearances;
}

CellSet every_other(const std::vector<Cell> &faces)
{
	const CellSet face_set(faces.begin(), faces.end());
	std::vector<Cell> starts = faces;
	std::sort(starts.begin(), starts.end());
	CellMap distance;
	std::deque<Cell> queue;
	for (const auto &start : starts) {
		if (distance.contains(start))
			continue;
		distance.emplace(start, 0);
		queue.push_back(start);
		while (!queue.empty()) {
			const Cell cell = queue.front();
			queue.pop_front();
			const int d = distance.at(cell);
			for (int dx = -1; dx <= 1; ++dx) {
				for (int dz = -1; dz <= 1; ++dz) {
					const Cell next{cell.first + dx, cell.second + dz};
					if (face_set.contains(next) && !distance.contains(next)) {
						distance.emplace(next, d + 1);
						queue.push_back(next);
					}
				}
			}
		}
	}
	CellSet selected;
	for (const auto &[cell, d] : distance)
		if (d % 2 == 0)
			selected.insert(cell);
	return selected;
}
} // namespace

bool line_wall(const tags_t &tags)
{
	const auto barrier = tag(tags, "barrier");
	return (barrier == "city_wall" || barrier == "wall") && defence(tags);
}

std::optional<Kind> kind(const tags_t &tags)
{
	const auto roof_shape = tag(tags, "roof:shape");
	if (!roof_shape.empty() && roof_shape != "flat")
		return std::nullopt;
	const bool gate = tag(tags, "historic") == "city_gate" ||
					  (tag(tags, "building") == "gatehouse" && defence(tags));
	if (gate)
		return Kind::Gate;
	if (tag(tags, "building") == "wall" || defence(tags))
		return Kind::Wall;
	return std::nullopt;
}

bool crenellated(const tags_t &tags, Kind kind)
{
	return (kind == Kind::Gate || defence(tags)) && !tags.contains("building:part");
}

void generate(WorldEditor &editor, const Mass &mass)
{
	if (mass.cells.empty() || mass.height <= 0)
		return;
	CellSet own(mass.cells.begin(), mass.cells.end());
	CellSet passages(mass.passages.begin(), mass.passages.end());
	CellMap footings;
	for (const auto &[cell, y] : mass.footings)
		footings.insert_or_assign(cell, y);
	const CellMap arch =
			mass.raised ? CellMap{}
						: passage_clearances(own, passages, mass.height, mass.scale);
	constexpr std::array<Cell, 4> neighbours = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
	std::vector<Cell> faces;
	faces.reserve(mass.cells.size());
	for (const auto &cell : mass.cells) {
		const bool exposed =
				std::any_of(neighbours.begin(), neighbours.end(), [&](const Cell &d) {
					const Cell n{cell.first + d.first, cell.second + d.second};
					return !own.contains(n) &&
						   !(mass.abutting && mass.abutting(n.first, n.second));
				});
		const auto open = arch.find(cell);
		if (exposed && !(open != arch.end() && open->second >= mass.height))
			faces.push_back(cell);
	}
	const CellSet merlons =
			mass.crenellated && mass.height >= 3 ? every_other(faces) : CellSet{};
	for (const auto &[x, z] : mass.cells) {
		const auto footing = footings.find({x, z});
		const int ground = footing != footings.end() ? footing->second
													 : editor.get_absolute_y(x, 0, z);
		const int base = mass.base.value_or(ground);
		const int top = base + mass.height;
		const auto opening = arch.find({x, z});
		const int from = opening != arch.end()
								 ? std::max(base, ground) + opening->second + 1
						 : mass.raised ? base + 1
									   : std::min(ground, base) + 1;
		for (int y = from; y <= top; ++y)
			editor.set_block_absolute(mass.block, x, y, z);
		if (merlons.contains({x, z}))
			editor.set_block_absolute(mass.block, x, top + 1, z);
	}
}

} // namespace arnis::fortifications
