#include "rail_track.h"

#include "../bresenham.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace arnis::railways
{

std::vector<std::tuple<int, int, int>> smooth_diagonal_rails(
		const std::vector<std::tuple<int, int, int>> &points);
Block determine_rail_with_slope(const std::pair<int, int> &current,
		const std::optional<std::pair<int, int>> &prev,
		const std::optional<std::pair<int, int>> &next, int prev_ground,
		int current_ground, int next_ground);

namespace
{
constexpr std::size_t HEADING_CELLS = 6;
constexpr double MAX_JOIN_DEFLECT_DEG = 60.0;
constexpr std::size_t BEYOND_CELLS = 2;

struct Track
{
	const ProcessedWay *way{};
	TrackLevel level{};
	std::vector<std::pair<int, int>> cells;
	std::vector<std::size_t> node_cells;
};

struct Pick
{
	std::size_t track{};
	std::size_t node{};
	int direction{};
	double bend{};
};

using Point = std::pair<std::size_t, std::size_t>;
struct PointHash
{
	std::size_t operator()(const Point &p) const noexcept
	{
		return std::hash<std::size_t>{}(p.first) ^
			   (std::hash<std::size_t>{}(p.second) << 1);
	}
};

double deflection(std::pair<double, double> a, std::pair<double, double> b)
{
	return std::abs(std::atan2(a.first * b.second - a.second * b.first,
				   a.first * b.first + a.second * b.second)) *
		   180.0 / std::acos(-1.0);
}

bool is_endpoint(
		const std::vector<Track> &tracks, const std::pair<std::size_t, std::size_t> &p)
{
	return p.second == 0 || p.second + 1 == tracks[p.first].way->nodes.size();
}

bool owns_node(const std::vector<Track> &tracks,
		const std::unordered_map<std::uint64_t, std::vector<Point>> &at_node,
		const std::unordered_map<Point, std::optional<Pick>, PointHash> &picks,
		const Point &point)
{
	const auto &here = at_node.at(tracks[point.first].way->nodes[point.second].id);
	if (std::any_of(here.begin(), here.end(),
				[&](const auto &p) { return !is_endpoint(tracks, p); }))
		return false;
	const auto mutual = [&](const auto &p) {
		const auto a = picks.find(p);
		if (a == picks.end() || !a->second)
			return false;
		const auto &[other, other_node, direction, bend] = *a->second;
		(void)direction;
		(void)bend;
		const std::pair<std::size_t, std::size_t> q{other, other_node};
		if (!is_endpoint(tracks, q))
			return false;
		const auto b = picks.find(q);
		return b != picks.end() && b->second &&
			   std::pair<std::size_t, std::size_t>{b->second->track, b->second->node} ==
					   p;
	};
	if (mutual(point))
		return true;
	if (std::any_of(here.begin(), here.end(), mutual))
		return false;
	const auto lowest = *std::min_element(
			here.begin(), here.end(), [&](const auto &a, const auto &b) {
				return std::pair{tracks[a.first].way->id, a.second} <
					   std::pair{tracks[b.first].way->id, b.second};
			});
	return lowest == point;
}

} // namespace

std::vector<bool> straight_cells(const std::vector<std::pair<int, int>> &cells,
		std::optional<std::pair<int, int>> before,
		std::optional<std::pair<int, int>> after)
{
	std::vector<bool> result(cells.size(), true);
	for (std::size_t i = 0; i < cells.size(); ++i) {
		const auto prev = i > 0 ? std::optional(cells[i - 1]) : before;
		const auto next = i + 1 < cells.size() ? std::optional(cells[i + 1]) : after;
		if (prev && next) {
			const auto &[x, z] = cells[i];
			result[i] = (prev->first == x && next->first == x) ||
						(prev->second == z && next->second == z);
		}
	}
	return result;
}

std::vector<int> rail_profile(const std::vector<bool> &straight,
		const std::function<std::pair<int, int>(std::size_t)> &range,
		const std::function<double(std::size_t, int)> &cost, double step_cost)
{
	constexpr double BROKEN = 1000.0;
	const std::size_t n = straight.size();
	if (!n)
		return {};
	std::vector<int> lows;
	std::vector<std::size_t> starts;
	std::vector<std::uint32_t> back;
	std::vector<double> prev;
	for (std::size_t i = 0; i < n; ++i) {
		auto [lo, hi] = range(i);
		if (lo > hi)
			std::swap(lo, hi);
		const auto width = static_cast<std::size_t>(hi - lo + 1);
		std::vector<double> cur(width * 2, std::numeric_limits<double>::infinity());
		starts.push_back(back.size());
		const auto base = starts.back();
		back.resize(base + width * 2, std::numeric_limits<std::uint32_t>::max());
		double far_floor = std::numeric_limits<double>::infinity();
		if (i > 0) {
			const auto nearest = std::min_element(prev.begin(), prev.end());
			if (nearest != prev.end())
				far_floor = *nearest + 2.0 * BROKEN;
		}
		for (std::size_t bi = 0; bi < width; ++bi) {
			const int b = lo + static_cast<int>(bi);
			const double own = cost(i, b);
			if (i == 0) {
				cur[bi * 2] = own;
				continue;
			}
			const auto relax_states = [&](std::size_t begin, std::size_t end) {
				for (std::size_t s = begin; s < end; ++s) {
					if (!std::isfinite(prev[s]))
						continue;
					const int a = lows[i - 1] + static_cast<int>(s / 2);
					const bool climbing_back = s % 2 == 1;
					const int d = b - a;
					double extra = 0.0;
					bool low = false;
					if (d == 1 && straight[i - 1] && !climbing_back)
						extra = step_cost;
					else if (d == -1 && straight[i]) {
						extra = step_cost;
						low = true;
					} else if (d != 0) {
						extra = d == 1 || d == -1 ? BROKEN : BROKEN * std::abs(d);
						low = d == -1;
					}
					const auto slot = bi * 2 + static_cast<std::size_t>(low);
					const double total = prev[s] + extra + own;
					if (total < cur[slot]) {
						cur[slot] = total;
						back[base + slot] = static_cast<std::uint32_t>(s);
					}
				}
			};
			// Most profiles only need adjacent height states. Farther jumps cost at least
			// two BROKEN transitions, so rescan the full row only if that can still win.
			const int previous_low = lows[i - 1];
			const auto nearby_begin = static_cast<std::size_t>(std::clamp(
					2 * (b - 1 - previous_low), 0, static_cast<int>(prev.size())));
			const auto nearby_end = static_cast<std::size_t>(std::clamp(
					2 * (b + 2 - previous_low), 0, static_cast<int>(prev.size())));
			relax_states(nearby_begin, nearby_end);
			if (cur[bi * 2] >= far_floor + own) {
				cur[bi * 2] = std::numeric_limits<double>::infinity();
				cur[bi * 2 + 1] = std::numeric_limits<double>::infinity();
				back[base + bi * 2] = std::numeric_limits<std::uint32_t>::max();
				back[base + bi * 2 + 1] = std::numeric_limits<std::uint32_t>::max();
				relax_states(0, prev.size());
			}
		}
		lows.push_back(lo);
		prev = std::move(cur);
	}
	auto state = static_cast<std::size_t>(
			std::min_element(prev.begin(), prev.end()) - prev.begin());
	std::vector<int> heights(n);
	for (std::size_t i = n; i-- > 0;) {
		heights[i] = lows[i] + static_cast<int>(state / 2);
		if (i)
			state = back[starts[i] + state];
	}
	return heights;
}

std::vector<int> fit_rail_deck(const std::vector<std::pair<int, int>> &cells,
		const std::vector<int> &terrain, std::vector<int> heights,
		std::array<bool, 2> internal)
{
	const std::size_t n = std::min({cells.size(), terrain.size(), heights.size()});
	if (n < 3)
		return heights;
	std::vector<std::optional<int>> pins(n);
	for (std::size_t e = 0; e < internal.size(); ++e) {
		const std::size_t at = e == 0 ? 0 : n - 1;
		pins[at] = heights[at];
		if (!internal[e]) {
			const std::size_t next = e == 0 ? 1 : n - 2;
			if (!pins[next])
				pins[next] = heights[at];
		}
	}
	const auto line = std::vector<std::pair<int, int>>(cells.begin(), cells.begin() + n);
	const auto straight = straight_cells(line);
	auto fitted = rail_profile(
			straight,
			[&](std::size_t i) {
				return pins[i] ? std::pair{*pins[i], *pins[i]}
							   : std::pair{std::max(terrain[i], heights[i] - 3),
										 heights[i] + 3};
			},
			[&](std::size_t i, int y) { return 3.0 * std::abs(y - heights[i]); }, 0.5);
	fitted.insert(fitted.end(), heights.begin() + n, heights.end());
	return fitted;
}

std::vector<Block> rail_shapes(const std::vector<std::pair<int, int>> &cells,
		const std::vector<int> &heights,
		std::optional<std::tuple<std::pair<int, int>, int>> before,
		std::optional<std::tuple<std::pair<int, int>, int>> after)
{
	const std::size_t n = std::min(cells.size(), heights.size());
	std::vector<Block> result;
	result.reserve(n);
	for (std::size_t i = 0; i < n; ++i) {
		const auto prev =
				i ? std::optional(std::tuple{cells[i - 1], heights[i - 1]}) : before;
		const auto next =
				i + 1 < n ? std::optional(std::tuple{cells[i + 1], heights[i + 1]})
						  : after;
		result.push_back(determine_rail_with_slope(cells[i],
				prev ? std::optional(std::get<0>(*prev)) : std::nullopt,
				next ? std::optional(std::get<0>(*next)) : std::nullopt,
				prev ? std::get<1>(*prev) : heights[i], heights[i],
				next ? std::get<1>(*next) : heights[i]));
	}
	return result;
}

std::optional<TrackLevel> track_level(const ProcessedWay &way)
{
	static const std::unordered_set<std::string> types{"rail", "light_rail", "subway",
			"tram", "narrow_gauge", "monorail", "funicular", "miniature", "preserved",
			"disused"};
	const auto railway = way.tags.get("railway");
	if (way.nodes.size() < 2 || !types.contains(railway) || way.tags.get("area") == "yes")
		return std::nullopt;
	const bool indoor = way.tags.get("indoor") == "yes";
	// Match railways::is_rail_bridge: a present bridge tag (even an empty value)
	// marks a surface crossing unless this is explicitly indoor or bridge=no.
	const bool on_bridge =
			!indoor && way.tags.contains("bridge") && way.tags.get("bridge") != "no";
	const auto tunnel = way.tags.get("tunnel");
	const bool underground = tunnel == "yes" ||
							 ((railway == "subway" || way.tags.get("subway") == "yes") &&
									 tunnel != "no" && !on_bridge);
	return underground ? TrackLevel::Tunnel : TrackLevel::Surface;
}

RailJoins RailJoins::build(
		const std::vector<ProcessedElement> &elements, const LevelFn &level)
{
	std::vector<Track> tracks;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto track_level = level(way);
		if (!track_level || way.nodes.size() < 2)
			continue;
		Track track;
		track.way = &way;
		track.level = *track_level;
		track.cells.emplace_back(way.nodes.front().x, way.nodes.front().z);
		track.node_cells.push_back(0);
		for (std::size_t k = 1; k < way.nodes.size(); ++k) {
			const auto &a = way.nodes[k - 1];
			const auto &b = way.nodes[k];
			const auto line = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
			for (const auto &[x, _, z] : smooth_diagonal_rails(line))
				if (track.cells.back() != std::pair{x, z})
					track.cells.emplace_back(x, z);
			track.node_cells.push_back(track.cells.size() - 1);
		}
		if (!track.cells.empty())
			tracks.push_back(std::move(track));
	}

	std::unordered_map<std::uint64_t, std::vector<std::pair<std::size_t, std::size_t>>>
			at_node;
	for (std::size_t t = 0; t < tracks.size(); ++t)
		for (std::size_t k = 0; k < tracks[t].way->nodes.size(); ++k)
			at_node[tracks[t].way->nodes[k].id].emplace_back(t, k);

	std::unordered_map<Point, std::optional<Pick>, PointHash> picks;
	for (std::size_t t = 0; t < tracks.size(); ++t) {
		const auto last = tracks[t].way->nodes.size() - 1;
		for (const std::size_t k : {std::size_t{0}, last}) {
			const auto &others = at_node.at(tracks[t].way->nodes[k].id);
			const auto n = tracks[t].cells.size();
			const auto reach = std::min(HEADING_CELLS, n - 1);
			const auto at = k == 0 ? tracks[t].cells.front() : tracks[t].cells.back();
			const auto back =
					k == 0 ? tracks[t].cells[reach] : tracks[t].cells[n - 1 - reach];
			const std::pair<double, double> heading{
					double(at.first - back.first), double(at.second - back.second)};
			std::optional<Pick> best;
			for (const auto &[o, ko] : others) {
				if (o == t && ko == k)
					continue;
				const auto c = tracks[o].node_cells[ko];
				for (const int dir : {1, -1}) {
					const auto room = dir > 0 ? tracks[o].cells.size() - 1 - c : c;
					if (!room)
						continue;
					const auto offset = std::min(HEADING_CELLS, room);
					const auto ahead = tracks[o].cells[static_cast<std::size_t>(
							static_cast<std::ptrdiff_t>(c) +
							dir * static_cast<std::ptrdiff_t>(offset))];
					const std::pair<double, double> out{double(ahead.first - at.first),
							double(ahead.second - at.second)};
					const double bend = deflection(heading, out);
					// Match Rust's tie window exactly: a lower-id way wins whenever its
					// bend is within 1e-9 above the current best (including a better bend).
					if (!best || bend < best->bend - 1e-9 ||
							(bend < best->bend + 1e-9 &&
									tracks[o].way->id < tracks[best->track].way->id))
						best = Pick{o, ko, dir, bend};
				}
			}
			if (best && best->bend <= MAX_JOIN_DEFLECT_DEG)
				picks[{t, k}] = best;
			else
				picks[{t, k}] = std::nullopt;
		}
	}

	RailJoins result;
	for (std::size_t t = 0; t < tracks.size(); ++t) {
		const auto last = tracks[t].way->nodes.size() - 1;
		std::array<TrackEnd, 2> pair{};
		for (const auto [e, k] : {std::pair{0, std::size_t{0}}, std::pair{1, last}}) {
			const auto &others = at_node.at(tracks[t].way->nodes[k].id);
			std::vector<Point> connected;
			for (const auto &p : others)
				if (p != Point{t, k})
					connected.push_back(p);
			if (connected.empty())
				continue;
			auto &end = pair[e];
			end.shared = true;
			end.meets_surface =
					std::any_of(connected.begin(), connected.end(), [&](const auto &p) {
						return tracks[p.first].level == TrackLevel::Surface;
					});
			const auto it = picks.find({t, k});
			if (it != picks.end() && it->second) {
				const auto &[o, ko, dir, _] = *it->second;
				const auto c = tracks[o].node_cells[ko];
				for (std::size_t step = 1; step <= BEYOND_CELLS; ++step) {
					const auto index = static_cast<std::ptrdiff_t>(c) +
									   dir * static_cast<std::ptrdiff_t>(step);
					if (index < 0 ||
							static_cast<std::size_t>(index) >= tracks[o].cells.size())
						break;
					end.beyond.push_back(
							tracks[o].cells[static_cast<std::size_t>(index)]);
				}
				end.into = std::tuple{tracks[o].way->id, c, dir};
			}
			end.owns = owns_node(tracks, at_node, picks, {t, k});
		}
		result.ends_.emplace(tracks[t].way->id, std::move(pair));
	}
	return result;
}

std::array<TrackEnd, 2> RailJoins::ends(std::uint64_t way) const
{
	const auto it = ends_.find(way);
	return it == ends_.end() ? std::array<TrackEnd, 2>{} : it->second;
}

std::vector<std::array<std::size_t, 2>> RailJoins::edge_crossings(
		const std::vector<std::pair<int, int>> &cells) const
{
	std::vector<std::array<std::size_t, 2>> crossings;
	if (!area_ || cells.size() < 2)
		return crossings;
	const auto inside = [&](const std::pair<int, int> &cell) {
		return area_->contains(cartesian::XZPoint{cell.first, cell.second});
	};
	for (std::size_t i = 1; i < cells.size(); ++i) {
		const bool previous_inside = inside(cells[i - 1]);
		const bool current_inside = inside(cells[i]);
		if (previous_inside && !current_inside)
			crossings.push_back({i - 1, i});
		else if (!previous_inside && current_inside)
			crossings.push_back({i, i - 1});
	}
	return crossings;
}

} // namespace arnis::railways
