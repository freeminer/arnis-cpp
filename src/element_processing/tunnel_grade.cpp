#include "tunnel_grade.h"

#include "../floodfill_cache.h"
#include "../world_editor/floor_state.h"
#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <unordered_map>

namespace arnis::tunnel_grade
{
namespace
{
bool at_portal(std::size_t i, std::size_t n, const std::array<bool, 2> &portal)
{
	return (portal[0] && i <= 1) || (portal[1] && i + 2 >= n);
}

std::pair<int, int> block_of(int x, int z)
{
	auto div_floor = [](int value) {
		const int q = value / 16;
		return q - (value % 16 < 0 ? 1 : 0);
	};
	return {div_floor(x), div_floor(z)};
}

std::pair<int, int> heading(const std::vector<std::pair<int, int>> &path, std::size_t i)
{
	const auto &a = path[i > 4 ? i - 4 : 0];
	const auto &b = path[std::min(i + 4, path.size() - 1)];
	return {b.first - a.first, b.second - a.second};
}

bool parallel(std::pair<int, int> a, std::pair<int, int> b)
{
	const double dot = static_cast<double>(a.first) * b.first +
					   static_cast<double>(a.second) * b.second;
	const double length =
			std::hypot(static_cast<double>(a.first), static_cast<double>(a.second)) *
			std::hypot(static_cast<double>(b.first), static_cast<double>(b.second));
	return length > 0.0 && std::abs(dot) >= 0.9 * length;
}

std::vector<double> hull_under(const std::vector<double> &cap, std::size_t a, double la,
		std::size_t b, double lb)
{
	auto cross = [](const std::pair<double, double> &o,
						 const std::pair<double, double> &p,
						 const std::pair<double, double> &q) {
		return (p.first - o.first) * (q.second - o.second) -
			   (p.second - o.second) * (q.first - o.first);
	};
	std::vector<std::pair<double, double>> points;
	points.emplace_back(static_cast<double>(a), la);
	for (std::size_t i = a + 1; i < b; ++i)
		if (std::isfinite(cap[i]))
			points.emplace_back(static_cast<double>(i), cap[i]);
	points.emplace_back(static_cast<double>(b), lb);
	std::vector<std::pair<double, double>> hull;
	for (const auto &point : points) {
		while (hull.size() >= 2 &&
				cross(hull[hull.size() - 2], hull.back(), point) <= 0.0)
			hull.pop_back();
		hull.push_back(point);
	}
	std::vector<double> line(cap.size(), 0.0);
	for (std::size_t s = 1; s < hull.size(); ++s) {
		const auto [x0, y0] = hull[s - 1];
		const auto [x1, y1] = hull[s];
		for (std::size_t i = static_cast<std::size_t>(x0);
				i <= static_cast<std::size_t>(x1); ++i)
			line[i] = y0 + (y1 - y0) * (static_cast<double>(i) - x0) / (x1 - x0);
	}
	return line;
}

std::vector<double> support_line(const std::vector<double> &cap, double grade)
{
	const std::size_t n = cap.size();
	if (n == 1)
		return cap;
	const auto hull = hull_under(cap, 0, cap.front(), n - 1, cap.back());
	const std::size_t mid = (n - 1) / 2;
	const double slope = std::clamp(hull[mid + 1] - hull[mid], -grade, grade);
	const auto at = [&](std::size_t i) {
		return hull[mid] + slope * (static_cast<double>(i) - static_cast<double>(mid));
	};
	double over = 0.0;
	for (std::size_t i = 0; i < n; ++i)
		over = std::max(over, at(i) - cap[i]);
	std::vector<double> result(n);
	for (std::size_t i = 0; i < n; ++i)
		result[i] = at(i) - over;
	return result;
}

std::vector<double> fit(const std::vector<double> &cap, const std::vector<double> &top,
		const std::vector<std::optional<double>> &pins, double grade)
{
	const std::size_t n = cap.size();
	std::vector<std::pair<std::size_t, double>> anchors;
	for (std::size_t i = 0; i < pins.size(); ++i)
		if (pins[i])
			anchors.emplace_back(i, *pins[i]);
	if (anchors.empty()) {
		auto line = support_line(cap, grade);
		for (std::size_t i = 0; i < n; ++i)
			line[i] = std::min(line[i], top[i]);
		return line;
	}
	const auto [first, first_level] = anchors.front();
	const auto [last, last_level] = anchors.back();
	if (first > 0)
		anchors.insert(anchors.begin(), {0, std::min({first_level, cap[0], top[0]})});
	if (last < n - 1)
		anchors.emplace_back(n - 1, std::min({last_level, cap[n - 1], top[n - 1]}));
	std::vector<double> line(n, 0.0);
	for (const auto &[i, y] : anchors)
		line[i] = y;
	for (std::size_t j = 1; j < anchors.size(); ++j) {
		const auto [a, la] = anchors[j - 1];
		const auto [b, lb] = anchors[j];
		line[a] = la;
		line[b] = lb;
		if (b <= a + 1)
			continue;
		const auto hull = hull_under(cap, a, la, b, lb);
		for (std::size_t i = a; i <= b; ++i)
			line[i] = hull[i];
		const double up = std::max(grade, std::abs(lb - la) / static_cast<double>(b - a));
		for (std::size_t i = a + 1; i < b; ++i)
			line[i] = std::max(line[i], line[i - 1] - up);
		for (std::size_t i = b - 1; i > a; --i)
			line[i] = std::max(line[i], line[i + 1] - up);
		std::vector<std::size_t> over;
		for (std::size_t i = a + 1; i < b; ++i)
			if (line[i] > top[i])
				over.push_back(i);
		for (const auto i : over) {
			const double room = std::max((la - top[i]) / static_cast<double>(i - a),
					(lb - top[i]) / static_cast<double>(b - i));
			const double steep = std::max(up, room);
			for (std::size_t k = a + 1; k < b; ++k)
				line[k] = std::min(
						line[k], top[i] + steep * std::abs(static_cast<double>(k) -
														   static_cast<double>(i)));
		}
	}
	return line;
}
} // namespace

int lowest_floor()
{
	return world_editor::min_y() + FLOOR_GUARD;
}

Bore Bore::make(std::size_t cells, int depth, double least, double most, int lowest)
{
	const double span =
			static_cast<double>(std::max<std::size_t>(1, cells > 0 ? cells - 1 : 0));
	return {std::clamp(4.0 * depth / span, least, most), static_cast<double>(depth),
			static_cast<double>(lowest)};
}

std::vector<double> grade_line(const std::vector<double> &ceiling,
		const std::vector<double> &top, const std::vector<std::optional<double>> &pins,
		std::array<bool, 2> portal, const Bore &bore)
{
	const std::size_t n = ceiling.size();
	if (n == 0)
		return {};
	auto pin_at = [&](std::size_t i) {
		return pins[i] && !at_portal(i, n, portal)
					   ? *pins[i]
					   : -std::numeric_limits<double>::infinity();
	};
	std::vector<double> cone(n, -std::numeric_limits<double>::infinity());
	double c = -std::numeric_limits<double>::infinity();
	for (std::size_t i = 0; i < n; ++i) {
		c = std::max(c - bore.grade, pin_at(i));
		cone[i] = c;
	}
	c = -std::numeric_limits<double>::infinity();
	for (std::size_t i = n; i-- > 0;) {
		c = std::max(c - bore.grade, pin_at(i));
		cone[i] = std::max(cone[i], c);
	}
	std::vector<double> base(n), capped_top(n);
	for (std::size_t i = 0; i < n; ++i) {
		base[i] = std::max({ceiling[i], cone[i], bore.lowest});
		capped_top[i] = std::max(top[i], bore.lowest);
	}
	const auto full = static_cast<std::size_t>(
			std::max(1.0, std::ceil(bore.depth / bore.grade - 1e-9)));
	std::array<std::size_t, 2> reach{portal[0] ? full : 0, portal[1] ? full : 0};
	if (reach[0] + reach[1] + 2 > n && portal[0] && portal[1]) {
		const auto room = n > 2 ? n - 2 : 0;
		reach = {(room + 1) / 2, room / 2};
	}
	std::vector<double> line;
	for (int attempt = 0; attempt < 8; ++attempt) {
		std::vector<double> cap(n);
		for (std::size_t i = 0; i < n; ++i)
			cap[i] = i < reach[0] || n - 1 - i < reach[1]
							 ? std::numeric_limits<double>::infinity()
							 : base[i];
		line = fit(cap, capped_top, pins, bore.grade);
		bool moved = false;
		for (std::size_t e = 0; e < 2; ++e) {
			const auto cell = [&](std::size_t d) { return e == 0 ? d : n - 1 - d; };
			const auto end = std::min(reach[e], n);
			for (std::size_t d = 1; d < end; ++d)
				if (line[cell(d)] <= base[cell(d)]) {
					reach[e] = d;
					moved = true;
					break;
				}
		}
		if (!moved)
			break;
	}
	return line;
}

std::vector<bool> portal_cuttings(
		const std::vector<bool> &bare, std::array<bool, 2> portal, std::size_t gap)
{
	std::vector<bool> open(bare.size(), false);
	for (std::size_t e = 0; e < 2; ++e) {
		if (!portal[e])
			continue;
		std::size_t covered = 0;
		for (std::size_t d = 0; d < bare.size(); ++d) {
			const auto i = e == 0 ? d : bare.size() - 1 - d;
			if (bare[i]) {
				open[i] = true;
				covered = 0;
			} else if (++covered >= gap) {
				break;
			}
		}
	}
	return open;
}

void pin_area_edges(std::vector<std::optional<int>> &pins,
		const std::vector<std::pair<int, int>> &cells, const ::XZBBox *area,
		const std::function<int(std::size_t)> &level)
{
	if (!area)
		return;
	for (std::size_t i = 1; i < cells.size(); ++i) {
		const auto inside = [&](std::size_t j) {
			return area->contains(cells[j].first, cells[j].second);
		};
		if (inside(i - 1) == inside(i))
			continue;
		const auto lower = cells[i - 1] < cells[i] ? i - 1 : i;
		const int y = level(lower);
		if (!pins[i - 1])
			pins[i - 1] = y;
		if (!pins[i])
			pins[i] = y;
	}
}

void walkable(
		std::vector<int> &levels, const std::vector<std::optional<int>> &pins, int step)
{
	const std::size_t n = levels.size();
	if (n == 0 || step < 0)
		return;
	constexpr int low = std::numeric_limits<int>::min() / 2;
	constexpr int high = std::numeric_limits<int>::max() / 2;
	std::vector<int> lo(n, low), hi(n, high);
	int l = low, h = high;
	for (std::size_t i = 0; i < n; ++i) {
		if (pins[i])
			l = h = levels[i];
		else {
			l -= step;
			h += step;
		}
		lo[i] = l;
		hi[i] = h;
	}
	l = low;
	h = high;
	for (std::size_t i = n; i-- > 0;) {
		if (pins[i])
			l = h = levels[i];
		else {
			l -= step;
			h += step;
		}
		lo[i] = std::max(lo[i], l);
		hi[i] = std::min(hi[i], h);
	}
	for (std::size_t i = 0; i < n; ++i)
		if (!pins[i])
			levels[i] = std::max(std::min(levels[i], hi[i]), lo[i]);
	for (std::size_t i = 1; i < n; ++i)
		if (!pins[i])
			levels[i] = std::min(levels[i], levels[i - 1] + step);
	for (std::size_t i = n - 1; i-- > 0;)
		if (!pins[i])
			levels[i] = std::min(levels[i], levels[i + 1] + step);
}

std::size_t Beside::CellHash::operator()(const std::pair<int, int> &cell) const noexcept
{
	const auto packed = (std::uint64_t(static_cast<std::uint32_t>(cell.first)) << 32) |
						static_cast<std::uint32_t>(cell.second);
	return std::hash<std::uint64_t>{}(packed);
}

void Beside::add(
		const std::vector<std::pair<int, int>> &path, const std::vector<double> &levels)
{
	const auto n = std::min(path.size(), levels.size());
	for (std::size_t i = 0; i < n; ++i) {
		cells_.try_emplace(path[i], levels[i], heading(path, i));
		blocks_.insert(block_of(path[i].first, path[i].second));
	}
}

bool Beside::near(int x, int z, int reach) const
{
	const auto [x0, z0] = block_of(x - reach, z - reach);
	const auto [x1, z1] = block_of(x + reach, z + reach);
	for (int bx = x0; bx <= x1; ++bx)
		for (int bz = z0; bz <= z1; ++bz)
			if (blocks_.contains({bx, bz}))
				return true;
	return false;
}

std::vector<std::optional<double>> Beside::pins(
		const std::vector<std::pair<int, int>> &path, const std::vector<double> &line,
		int reach, std::array<bool, 2> portal) const
{
	const std::size_t n = path.size();
	std::vector<std::optional<double>> out(n);
	if (cells_.empty() || line.size() < n)
		return out;
	for (std::size_t i = 0; i < n; ++i) {
		const auto [x, z] = path[i];
		if (!near(x, z, reach))
			continue;
		const auto own = heading(path, i);
		std::optional<std::pair<int, double>> best;
		for (int dx = -reach; dx <= reach; ++dx)
			for (int dz = -reach; dz <= reach; ++dz) {
				const auto it = cells_.find({x + dx, z + dz});
				if (it == cells_.end() || !parallel(own, it->second.second))
					continue;
				const int distance = dx * dx + dz * dz;
				if (!best || distance < best->first)
					best = {distance, it->second.first};
			}
		if (best && std::abs(best->second - line[i]) <= 6.0)
			out[i] = best->second;
	}
	for (std::size_t i = 0; i < n;) {
		const auto start = i;
		while (i < n && out[i])
			++i;
		if (i - start < 8)
			std::fill(out.begin() + start, out.begin() + i, std::nullopt);
		++i;
	}
	const auto portal_range = std::min<std::size_t>(16, n);
	if (portal[0])
		std::fill(out.begin(), out.begin() + portal_range, std::nullopt);
	if (portal[1])
		std::fill(out.end() - portal_range, out.end(), std::nullopt);
	return out;
}

std::vector<std::vector<std::pair<std::size_t, bool>>> chains(std::size_t count,
		const std::function<std::optional<std::pair<std::size_t, std::size_t>>(
				std::size_t, std::size_t)> &partner)
{
	std::vector<bool> seen(count, false);
	std::vector<std::vector<std::pair<std::size_t, bool>>> result;
	for (std::size_t w = 0; w < count; ++w) {
		if (seen[w])
			continue;
		auto current = w;
		std::size_t back = 0;
		for (std::size_t guard = 0; guard < count; ++guard) {
			const auto next = partner(current, back);
			if (!next || next->first == w)
				break;
			current = next->first;
			back = 1 - next->second;
		}
		std::vector<std::pair<std::size_t, bool>> run;
		auto c = current;
		auto start = back;
		while (c < count && !seen[c]) {
			seen[c] = true;
			run.emplace_back(c, start == 1);
			const auto next = partner(c, 1 - start);
			if (!next || next->first >= count || seen[next->first])
				break;
			c = next->first;
			start = next->second;
		}
		result.push_back(std::move(run));
	}
	return result;
}

std::vector<std::size_t> planning_order(
		const std::vector<std::pair<std::size_t, std::uint64_t>> &size,
		const std::vector<std::vector<std::uint64_t>> &nodes)
{
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> at;
	for (std::size_t r = 0; r < nodes.size(); ++r)
		for (const auto id : nodes[r])
			at[id].push_back(r);
	auto rank_less = [&](std::size_t a, std::size_t b) {
		return size[a].first != size[b].first ? size[a].first > size[b].first
											  : size[a].second < size[b].second;
	};
	std::vector<std::size_t> starts(size.size());
	for (std::size_t i = 0; i < starts.size(); ++i)
		starts[i] = i;
	std::sort(starts.begin(), starts.end(), rank_less);
	std::vector<bool> seen(size.size(), false);
	std::vector<std::size_t> order;
	for (const auto start : starts) {
		if (seen[start])
			continue;
		seen[start] = true;
		std::deque<std::size_t> queue{start};
		while (!queue.empty()) {
			const auto r = queue.front();
			queue.pop_front();
			order.push_back(r);
			std::vector<std::size_t> next;
			for (const auto id : nodes[r]) {
				const auto it = at.find(id);
				if (it != at.end())
					for (const auto other : it->second)
						if (other < seen.size() && !seen[other])
							next.push_back(other);
			}
			std::sort(next.begin(), next.end(), rank_less);
			next.erase(std::unique(next.begin(), next.end()), next.end());
			for (const auto other : next) {
				seen[other] = true;
				queue.push_back(other);
			}
		}
	}
	return order;
}

} // namespace arnis::tunnel_grade
