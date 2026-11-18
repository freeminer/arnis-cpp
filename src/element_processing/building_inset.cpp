#include "building_inset.h"

#include "../bresenham.h"
#include "../floodfill.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

namespace arnis::building_inset
{
namespace
{
constexpr double STRAIGHT_SINE = 0.17;
constexpr double MAX_CORNER_SHIFT = 2.5;
constexpr double MAX_INSET_LOSS = 0.5;
constexpr std::array<std::pair<int, int>, 4> SIDES{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

int outward_side(const std::vector<ProcessedNode> &nodes)
{
	std::int64_t area2 = 0;
	for (std::size_t i = 0; i + 1 < nodes.size(); ++i)
		area2 += std::int64_t(nodes[i].x) * nodes[i + 1].z -
				 std::int64_t(nodes[i + 1].x) * nodes[i].z;
	return area2 > 0 ? -1 : 1;
}

std::pair<int, int> normal(const ProcessedNode &a, const ProcessedNode &b, int outward)
{
	const int dx = b.x - a.x, dz = b.z - a.z;
	return {-dz * outward, dx * outward};
}

CellSet own_and_outline(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill)
{
	CellSet cells(fill.begin(), fill.end());
	for (const auto &c : ring_cells(nodes))
		cells.insert(c);
	return cells;
}

bool taken(const CellSet &own, const CoordinateBitmap &footprints, int x, int z)
{
	return footprints.contains(x, z) && !own.contains({x, z});
}

std::vector<bool> free_edges_by(const std::vector<ProcessedNode> &nodes,
		const CellSet &own, const CoordinateBitmap &footprints, bool walls)
{
	std::vector<bool> free;
	if (nodes.size() < 2)
		return free;
	const int outward = outward_side(nodes);
	free.reserve(nodes.size() - 1);
	for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[i + 1];
		const auto [nx, nz] = normal(a, b, outward);
		if (nx == 0 && nz == 0) {
			free.push_back(true);
			continue;
		}
		bool is_free = true;
		for (const auto &[x, unused_y, z] : bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)unused_y;
			for (int d = 1; d <= 2 && is_free; ++d) {
				const int cx = x + nx * d, cz = z + nz * d;
				if (taken(own, footprints, cx, cz)) {
					is_free = false;
				} else if (walls && d == 2) {
					for (const auto &[sx, sz] : SIDES)
						if (taken(own, footprints, cx + sx, cz + sz)) {
							is_free = false;
							break;
						}
				}
			}
			if (!is_free)
				break;
		}
		free.push_back(is_free);
	}
	return free;
}

std::optional<std::vector<ProcessedNode>> inset_ring(
		const std::vector<ProcessedNode> &nodes, const std::vector<bool> &step)
{
	if (nodes.size() < 4 || step.size() != nodes.size() - 1 ||
			std::pair{nodes.front().x, nodes.front().z} !=
					std::pair{nodes.back().x, nodes.back().z})
		return std::nullopt;
	std::vector<std::pair<std::size_t, bool>> corners;
	for (std::size_t i = 0; i + 1 < nodes.size(); ++i)
		if (nodes[i].x != nodes[i + 1].x || nodes[i].z != nodes[i + 1].z)
			corners.emplace_back(i, step[i]);
	const std::size_t m = corners.size();
	if (m < 3)
		return std::nullopt;
	const double outward = static_cast<double>(outward_side(nodes));
	auto at = [&](std::size_t k) {
		const auto &n = nodes[corners[k % m].first];
		return std::pair{double(n.x), double(n.z)};
	};
	struct Edge
	{
		std::pair<double, double> tangent, normal;
	};
	auto edge = [&](std::size_t k) -> std::optional<Edge> {
		const auto a = at(k), b = at(k + 1);
		const double dx = b.first - a.first, dz = b.second - a.second;
		const double len = std::hypot(dx, dz);
		if (!(len > 0.0))
			return std::nullopt;
		return Edge{{dx / len, dz / len}, {dz / len * outward, -dx / len * outward}};
	};
	std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> moved;
	moved.reserve(m);
	for (std::size_t k = 0; k < m; ++k) {
		const auto v = at(k);
		const auto prev = (k + m - 1) % m;
		const auto ep = edge(prev), ei = edge(k);
		if (!ep || !ei)
			return std::nullopt;
		const double dp = corners[prev].second ? 1.0 : 0.0;
		const double di = corners[k].second ? 1.0 : 0.0;
		const auto on_p = std::pair{
				v.first + ep->normal.first * dp, v.second + ep->normal.second * dp};
		const auto on_i = std::pair{
				v.first + ei->normal.first * di, v.second + ei->normal.second * di};
		const auto [tpx, tpz] = ep->tangent;
		const auto [tix, tiz] = ei->tangent;
		const double cross = tpx * tiz - tpz * tix;
		const double dot = tpx * tix + tpz * tiz;
		if (std::abs(cross) < STRAIGHT_SINE) {
			if (dot < 0.0)
				return std::nullopt;
			moved.emplace_back(on_p, on_i);
			continue;
		}
		const double s =
				((on_i.first - on_p.first) * tiz - (on_i.second - on_p.second) * tix) /
				cross;
		const std::pair meet{on_p.first + tpx * s, on_p.second + tpz * s};
		if (std::hypot(meet.first - v.first, meet.second - v.second) > MAX_CORNER_SHIFT)
			return std::nullopt;
		moved.emplace_back(meet, meet);
	}
	for (std::size_t k = 0; k < m; ++k) {
		const auto e = edge(k);
		if (!e)
			return std::nullopt;
		const auto &a = moved[k].second;
		const auto &b = moved[(k + 1) % m].first;
		if ((b.first - a.first) * e->tangent.first +
						(b.second - a.second) * e->tangent.second <
				-0.25)
			return std::nullopt;
	}
	std::vector<ProcessedNode> out;
	out.reserve(m + 4);
	auto push = [&](const ProcessedNode &source, const std::pair<double, double> &p,
						bool keep_tags) {
		const int x = static_cast<int>(std::llround(p.first));
		const int z = static_cast<int>(std::llround(p.second));
		if (!out.empty() && out.back().x == x && out.back().z == z) {
			if (keep_tags && out.back().tags.empty())
				out.back().tags = source.tags;
			return;
		}
		out.push_back({source.id, keep_tags ? source.tags : tags_t{}, x, z});
	};
	for (std::size_t k = 0; k < m; ++k) {
		const auto &source = nodes[corners[k].first];
		push(source, moved[k].first, true);
		push(source, moved[k].second, false);
	}
	if (out.size() > 1 && out.front().x == out.back().x && out.front().z == out.back().z)
		out.pop_back();
	if (out.size() < 3)
		return std::nullopt;
	out.push_back(out.front());
	return out;
}

int rings(const CellSet &cells, int cap)
{
	if (cells.empty())
		return 0;
	int min_x = INT_MAX, min_z = INT_MAX, max_x = INT_MIN, max_z = INT_MIN;
	for (const auto &[x, z] : cells) {
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	const std::int64_t w64 = std::int64_t(max_x) - min_x + 3;
	const std::int64_t h64 = std::int64_t(max_z) - min_z + 3;
	if (w64 <= 0 || h64 <= 0 || w64 * h64 > 25'000'000)
		return 0;
	const auto w = static_cast<std::size_t>(w64), h = static_cast<std::size_t>(h64);
	std::vector<std::uint8_t> inside(w * h, 0), seen(w * h, 0);
	for (const auto &[x, z] : cells)
		inside[static_cast<std::size_t>(z - min_z + 1) * w +
				static_cast<std::size_t>(x - min_x + 1)] = 1;
	std::vector<std::size_t> ring;
	for (std::size_t i = 0; i < inside.size(); ++i) {
		if (!inside[i])
			continue;
		const auto x = i % w, z = i / w;
		if (!inside[z * w + x - 1] || !inside[z * w + x + 1] ||
				!inside[(z - 1) * w + x] || !inside[(z + 1) * w + x]) {
			ring.push_back(i);
			seen[i] = 1;
		}
	}
	int depth = 0;
	while (!ring.empty() && depth < cap) {
		++depth;
		std::vector<std::size_t> next;
		for (const auto i : ring) {
			const std::array<std::size_t, 4> around{i - 1, i + 1, i - w, i + w};
			for (const auto n : around)
				if (inside[n] && !seen[n]) {
					seen[n] = 1;
					next.push_back(n);
				}
		}
		ring.swap(next);
	}
	return depth;
}

std::pair<int, int> spans(const CellSet &cells)
{
	int min_x = INT_MAX, min_z = INT_MAX, max_x = INT_MIN, max_z = INT_MIN;
	for (const auto &[x, z] : cells) {
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	return cells.empty() ? std::pair{0, 0}
						 : std::pair{max_x - min_x + 1, max_z - min_z + 1};
}

std::optional<OutlineInset> inset_with(const std::vector<ProcessedNode> &nodes,
		const CellSet &own, const CellSet &outline, const std::vector<bool> &step)
{
	if (std::none_of(step.begin(), step.end(), [](bool v) { return v; }))
		return std::nullopt;
	auto ring = inset_ring(nodes, step);
	if (!ring)
		return std::nullopt;
	const auto ring_coords = ring_cells(*ring);
	if (std::any_of(ring_coords.begin(), ring_coords.end(),
				[&](const auto &c) { return !outline.contains(c); }))
		return std::nullopt;
	int lo_x = INT_MAX, hi_x = INT_MIN, lo_z = INT_MAX, hi_z = INT_MIN;
	for (const auto &n : *ring) {
		lo_x = std::min(lo_x, n.x);
		hi_x = std::max(hi_x, n.x);
		lo_z = std::min(lo_z, n.z);
		hi_z = std::max(hi_z, n.z);
	}
	if (std::min(hi_x - lo_x + 1, hi_z - lo_z + 1) < MIN_INSET_SPAN)
		return std::nullopt;
	std::vector<std::pair<int, int>> polygon;
	polygon.reserve(ring->size());
	for (const auto &n : *ring)
		polygon.emplace_back(n.x, n.z);
	auto inner_all = floodfill::flood_fill_area(polygon, nullptr);
	std::vector<std::pair<int, int>> inner;
	inner.reserve(inner_all.size());
	for (const auto &c : inner_all)
		if (own.contains(c))
			inner.push_back(c);
	CellSet walls(ring_coords.begin(), ring_coords.end());
	CellSet stepped(inner.begin(), inner.end());
	bool fin = false;
	for (const auto &[x, z] : walls) {
		bool has_room = false;
		for (int dx = -1; dx <= 1 && !has_room; ++dx)
			for (int dz = -1; dz <= 1; ++dz) {
				const std::pair c{x + dx, z + dz};
				if (stepped.contains(c) && !walls.contains(c)) {
					has_room = true;
					break;
				}
			}
		if (!has_room) {
			fin = true;
			break;
		}
	}
	stepped.insert(walls.begin(), walls.end());
	const auto [sx, sz] = spans(stepped);
	const int stepped_rings = rings(stepped, MIN_SHRUNK_RINGS);
	const bool cramped = stepped_rings < MIN_SHRUNK_RINGS &&
						 stepped_rings < rings(outline, stepped_rings + 1);
	if (inner.empty() || fin || cramped || std::min(sx, sz) < MIN_INSET_SPAN ||
			double(stepped.size()) < (1.0 - MAX_INSET_LOSS) * outline.size())
		return std::nullopt;
	CellSet strip;
	for (const auto &c : outline)
		if (!stepped.contains(c))
			strip.insert(c);
	return OutlineInset{std::move(*ring), std::move(inner), std::move(strip)};
}

std::vector<bool> width_kept(
		const std::vector<ProcessedNode> &nodes, const std::vector<bool> &free)
{
	std::vector<std::optional<std::pair<double, double>>> dirs;
	std::vector<std::pair<double, double>> party;
	dirs.reserve(nodes.size() - 1);
	for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
		const double dx = double(nodes[i + 1].x) - nodes[i].x;
		const double dz = double(nodes[i + 1].z) - nodes[i].z;
		const double len = std::hypot(dx, dz);
		if (len > 0.0)
			dirs.emplace_back(std::pair{dx / len, dz / len});
		else
			dirs.emplace_back(std::nullopt);
		if (i < free.size() && !free[i] && dirs.back())
			party.push_back(*dirs.back());
	}
	std::vector<bool> keep(dirs.size(), false);
	for (std::size_t i = 0; i < dirs.size(); ++i)
		if (i < free.size() && free[i] && dirs[i])
			keep[i] = std::any_of(party.begin(), party.end(), [&](const auto &p) {
				return std::abs(dirs[i]->first * p.second - dirs[i]->second * p.first) <
					   STRAIGHT_SINE;
			});
	return keep;
}

std::optional<std::pair<std::vector<bool>, OutlineInset>> decide(
		const std::vector<ProcessedNode> &nodes, const CellSet &own,
		const CellSet &outline, const CoordinateBitmap &footprints,
		const std::vector<bool> &off)
{
	auto masked = [&](std::vector<bool> flags) {
		for (std::size_t i = 0; i < flags.size() && i < off.size(); ++i)
			flags[i] = flags[i] && !off[i];
		return flags;
	};
	const auto near_free = free_edges_by(nodes, outline, footprints, false);
	const auto near = masked(near_free);
	const auto step = masked(free_edges_by(nodes, outline, footprints, true));
	auto attempt = [&](const std::vector<bool> &near_flags,
						   const std::vector<bool> &step_flags)
			-> std::optional<std::pair<std::vector<bool>, OutlineInset>> {
		auto first = inset_with(nodes, own, outline, near_flags);
		if (step_flags == near_flags)
			return first ? std::optional(std::pair{near_flags, std::move(*first)})
						 : std::nullopt;
		if (auto inset = inset_with(nodes, own, outline, step_flags))
			return std::pair{step_flags, std::move(*inset)};
		return first ? std::optional(std::pair{near_flags, std::move(*first)})
					 : std::nullopt;
	};
	if (auto result = attempt(near, step))
		return result;
	const auto keep = width_kept(nodes, near_free);
	if (std::none_of(keep.begin(), keep.end(), [](bool v) { return v; }))
		return std::nullopt;
	auto kept = [&](const std::vector<bool> &flags) {
		auto result = flags;
		for (std::size_t i = 0; i < result.size() && i < keep.size(); ++i)
			result[i] = result[i] && !keep[i];
		return result;
	};
	return attempt(kept(near), kept(step));
}
} // namespace

std::vector<std::pair<int, int>> ring_cells(const std::vector<ProcessedNode> &nodes)
{
	std::vector<std::pair<int, int>> result;
	for (std::size_t i = 0; i + 1 < nodes.size(); ++i)
		for (const auto &[x, unused_y, z] : bresenham_line(
					 nodes[i].x, 0, nodes[i].z, nodes[i + 1].x, 0, nodes[i + 1].z)) {
			(void)unused_y;
			result.emplace_back(x, z);
		}
	return result;
}

std::vector<bool> free_edges(const std::vector<ProcessedNode> &nodes, const CellSet &own,
		const CoordinateBitmap &footprints)
{
	return free_edges_by(nodes, own, footprints, true);
}

std::optional<OutlineInset> inset_outline(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const CoordinateBitmap &footprints)
{
	if (fill.empty())
		return std::nullopt;
	const CellSet own(fill.begin(), fill.end());
	const CellSet outline = own_and_outline(nodes, fill);
	if (auto result = decide(nodes, own, outline, footprints, {}))
		return std::move(result->second);
	return std::nullopt;
}

std::optional<std::vector<bool>> step_plan(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const CoordinateBitmap &footprints,
		const std::vector<bool> &off)
{
	if (fill.empty())
		return std::nullopt;
	const CellSet own(fill.begin(), fill.end());
	const CellSet outline = own_and_outline(nodes, fill);
	if (auto result = decide(nodes, own, outline, footprints, off))
		return std::move(result->first);
	return std::nullopt;
}

std::optional<OutlineInset> inset_planned(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const std::vector<bool> &step)
{
	if (fill.empty())
		return std::nullopt;
	const CellSet own(fill.begin(), fill.end());
	const CellSet outline = own_and_outline(nodes, fill);
	return inset_with(nodes, own, outline, step);
}

} // namespace arnis::building_inset
