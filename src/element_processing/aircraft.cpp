#include "aircraft.h"

#include "../clipping.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <tuple>

namespace arnis::element_processing::aircraft
{
namespace
{
constexpr double MIN_RUNWAY_M = 300.0;
constexpr double MAX_BEND = 0.03;
constexpr double SAME_RUNWAY_DEG = 20.0;
constexpr double STAND_REACH_M = 150.0;
constexpr double SAME_STAND_M = 8.0;
constexpr double DEAD_END_M = 30.0;
constexpr double DEAD_END_STAND_M = 45.0;
constexpr double PI = 3.14159265358979323846;

Point sub(Point a, Point b)
{
	return {a.first - b.first, a.second - b.second};
}
Point add(Point a, Point b)
{
	return {a.first + b.first, a.second + b.second};
}
Point mul(Point a, double k)
{
	return {a.first * k, a.second * k};
}
double dot(Point a, Point b)
{
	return a.first * b.first + a.second * b.second;
}
double cross(Point a, Point b)
{
	return a.first * b.second - a.second * b.first;
}
double norm(Point a)
{
	return std::hypot(a.first, a.second);
}
Point unit(Point a)
{
	const auto n = norm(a);
	return n > 0.0 ? Point{a.first / n, a.second / n} : Point{0.0, 1.0};
}
double turn_deg(Point a, Point b)
{
	return std::abs(std::atan2(cross(a, b), dot(a, b)) * 180.0 / PI);
}
std::pair<std::size_t, std::size_t> edge(std::size_t a, std::size_t b)
{
	return {std::min(a, b), std::max(a, b)};
}
std::uint64_t node_hash(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t h = a * 0x9E3779B97F4A7C15ULL ^ b * 0xC2B2AE3D27D4EB4FULL;
	h ^= h >> 29;
	h *= 0xBF58476D1CE4E5B9ULL;
	return h ^ (h >> 32);
}
bool unpaved(const ProcessedWay &way)
{
	const auto surface = way.tags.get("surface");
	return surface == "grass" || surface == "dirt" || surface == "ground" ||
		   surface == "earth" || surface == "soil" || surface == "sand" ||
		   surface == "mud" || surface == "clay" || surface == "unpaved" ||
		   surface == "gravel" || surface == "fine_gravel";
}
double turn_cost(double degrees)
{
	if (degrees > 150.0)
		return 600.0;
	if (degrees > 100.0)
		return 90.0;
	return degrees * 0.2;
}
} // namespace

double Field::span(std::size_t a, std::size_t b) const
{
	return norm(sub(nodes[b].at, nodes[a].at));
}

Point Field::dir(std::size_t a, std::size_t b) const
{
	return unit(sub(nodes[b].at, nodes[a].at));
}

std::optional<Kind> Field::kind(std::size_t a, std::size_t b) const
{
	if (a >= next.size())
		return std::nullopt;
	const auto found = std::find_if(next[a].begin(), next[a].end(),
			[&](const auto &link) { return link.first == b; });
	return found == next[a].end() ? std::nullopt : std::optional<Kind>(found->second);
}

void Field::link(std::size_t a, std::size_t b, Kind type)
{
	if (a == b || kind(a, b))
		return;
	next[a].emplace_back(b, type);
	next[b].emplace_back(a, type);
}

bool Field::taxis(std::size_t node) const
{
	return node < next.size() &&
		   std::any_of(next[node].begin(), next[node].end(),
				   [](const auto &link) { return link.second != Kind::Stand; });
}

Field Field::read(const std::vector<ProcessedElement> &elements, double scale)
{
	Field field;
	std::map<std::pair<int, int>, std::size_t> by_spot;
	std::map<std::pair<std::size_t, std::size_t>, Kind> links;
	std::set<std::pair<std::size_t, std::size_t>> rough;
	std::map<std::pair<std::size_t, std::size_t>, std::uint64_t> way_of;
	std::vector<std::pair<std::uint64_t, std::vector<std::size_t>>> stand_ways;
	std::vector<std::pair<std::uint64_t, Point>> stand_nodes;
	for (const auto &element : elements) {
		if (element.is_node()) {
			const auto &node = element.as_node();
			if (node.tags.get("aeroway") == "parking_position")
				stand_nodes.emplace_back(
						node.id, Point{double(node.x) + 0.5, double(node.z) + 0.5});
			continue;
		}
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto aeroway = way.tags.get("aeroway");
		Kind type;
		if (aeroway == "runway")
			type = Kind::Runway;
		else if (aeroway == "taxiway" || aeroway == "taxilane")
			type = Kind::Taxi;
		else if (aeroway == "parking_position")
			type = Kind::Stand;
		else
			continue;
		if (way.tags.get("area") == "yes" || way.nodes.size() < 2 ||
				way.nodes.front().id == way.nodes.back().id)
			continue;
		std::vector<std::size_t> line;
		line.reserve(way.nodes.size());
		for (const auto &node : way.nodes) {
			auto [it, inserted] =
					by_spot.emplace(std::pair{node.x, node.z}, field.nodes.size());
			if (inserted)
				field.nodes.push_back(
						{{double(node.x) + 0.5, double(node.z) + 0.5}, node.id});
			if (line.empty() || line.back() != it->second)
				line.push_back(it->second);
		}
		if (line.size() < 2)
			continue;
		if (type == Kind::Stand) {
			stand_ways.emplace_back(way.id, std::move(line));
			continue;
		}
		for (std::size_t i = 1; i < line.size(); ++i) {
			const auto key = edge(line[i - 1], line[i]);
			auto [it, inserted] = links.emplace(key, type);
			if (!inserted && type < it->second)
				it->second = type;
			if (type == Kind::Runway) {
				auto [id, fresh] = way_of.emplace(key, way.id);
				if (!fresh)
					id->second = std::min(id->second, way.id);
				if (unpaved(way))
					rough.insert(key);
			}
		}
	}
	field.next.resize(field.nodes.size());
	for (const auto &[key, type] : links) {
		field.next[key.first].emplace_back(key.second, type);
		field.next[key.second].emplace_back(key.first, type);
	}
	field.strips = field.read_strips(way_of, rough, scale);
	field.on_strip.resize(field.nodes.size());
	for (std::size_t s = 0; s < field.strips.size(); ++s)
		for (std::size_t i = 0; i < field.strips[s].nodes.size(); ++i)
			field.on_strip[field.strips[s].nodes[i]] = std::pair{s, i};
	std::sort(stand_ways.begin(), stand_ways.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	for (auto &[id, line] : stand_ways)
		field.add_stand(id, std::move(line), scale);
	std::sort(stand_nodes.begin(), stand_nodes.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	for (const auto &[id, at] : stand_nodes) {
		const bool duplicate = std::any_of(
				field.stands.begin(), field.stands.end(), [&](const Stand &stand) {
					return !stand.path.empty() &&
						   norm(sub(field.nodes[stand.path.back()].at, at)) <
								   12.0 * scale;
				});
		if (duplicate)
			continue;
		field.nodes.push_back({at, id});
		field.next.emplace_back();
		field.on_strip.emplace_back(std::nullopt);
		const auto node = field.nodes.size() - 1;
		if (const auto foot = field.reach_taxiway(node, std::nullopt, scale)) {
			field.link(*foot, node, Kind::Stand);
			field.stands.push_back({id, {*foot, node}});
		}
	}
	field.add_dead_ends(scale);
	field.one_line_a_stand(scale);
	return field;
}

std::vector<Strip> Field::read_strips(
		const std::map<std::pair<std::size_t, std::size_t>, std::uint64_t> &way_of,
		const std::set<std::pair<std::size_t, std::size_t>> &rough, double scale) const
{
	auto runway_nodes = [&](std::size_t n) {
		std::vector<std::size_t> result;
		for (const auto &[other, type] : next[n])
			if (type == Kind::Runway)
				result.push_back(other);
		return result;
	};
	std::set<std::pair<std::size_t, std::size_t>> used;
	std::vector<Strip> result;
	std::vector<std::size_t> starts;
	for (std::size_t n = 0; n < nodes.size(); ++n)
		if (runway_nodes(n).size() == 1)
			starts.push_back(n);
	for (std::size_t n = 0; n < nodes.size(); ++n)
		if (runway_nodes(n).size() > 2)
			starts.push_back(n);
	for (std::size_t n = 0; n < nodes.size(); ++n)
		if (runway_nodes(n).size() == 2)
			starts.push_back(n);
	for (const auto start : starts) {
		for (const auto first : runway_nodes(start)) {
			if (used.contains(edge(start, first)))
				continue;
			std::vector<std::size_t> chain{start, first};
			used.insert(edge(start, first));
			for (;;) {
				const auto a = chain[chain.size() - 2], b = chain.back();
				const auto ahead = dir(a, b);
				std::optional<std::pair<double, std::size_t>> best;
				for (const auto c : runway_nodes(b)) {
					if (c == a || used.contains(edge(b, c)))
						continue;
					const auto turn = turn_deg(ahead, dir(b, c));
					if (turn >= SAME_RUNWAY_DEG)
						continue;
					if (!best || turn < best->first)
						best = std::pair{turn, c};
				}
				if (!best)
					break;
				used.insert(edge(b, best->second));
				chain.push_back(best->second);
			}
			std::vector<double> at{0.0};
			for (std::size_t i = 1; i < chain.size(); ++i)
				at.push_back(at.back() + span(chain[i - 1], chain[i]));
			const double length = at.back();
			const auto from = nodes[chain.front()].at;
			const auto axis = unit(sub(nodes[chain.back()].at, from));
			double bend = 0.0;
			for (const auto n : chain)
				bend = std::max(bend, std::abs(cross(axis, sub(nodes[n].at, from))));
			if (length < MIN_RUNWAY_M * scale || bend > MAX_BEND * length)
				continue;
			bool paved = true;
			std::uint64_t id = std::numeric_limits<std::uint64_t>::max();
			for (std::size_t i = 1; i < chain.size(); ++i) {
				const auto key = edge(chain[i - 1], chain[i]);
				paved &= !rough.contains(key);
				if (const auto it = way_of.find(key); it != way_of.end())
					id = std::min(id, it->second);
			}
			result.push_back({std::move(chain), std::move(at), paved,
					id == std::numeric_limits<std::uint64_t>::max() ? 0 : id});
		}
	}
	std::sort(result.begin(), result.end(),
			[](const Strip &a, const Strip &b) { return a.id < b.id; });
	return result;
}

std::optional<std::tuple<std::size_t, std::size_t, double>> Field::nearest_taxiway(
		Point at, std::optional<Point> towards, double near, double far) const
{
	std::optional<std::tuple<double, std::size_t, std::size_t, double>> best;
	for (std::size_t a = 0; a < nodes.size(); ++a)
		for (const auto &[b, type] : next[a]) {
			if (type != Kind::Taxi || a > b)
				continue;
			const auto p = nodes[a].at, q = nodes[b].at;
			const auto delta = sub(q, p);
			const auto length2 = dot(delta, delta);
			if (length2 <= 1e-12)
				continue;
			const auto t = std::clamp(dot(sub(at, p), delta) / length2, 0.0, 1.0);
			const auto foot = add(p, mul(delta, t));
			const auto away = sub(foot, at);
			const auto distance = norm(away);
			if (distance > far || distance < near ||
					(distance > 1e-6 && towards && turn_deg(*towards, away) > 50.0))
				continue;
			if (!best || distance < std::get<0>(*best))
				best = {distance, a, b, t};
		}
	if (!best)
		return std::nullopt;
	return std::tuple{std::get<1>(*best), std::get<2>(*best), std::get<3>(*best)};
}

std::optional<std::size_t> Field::reach_taxiway(
		std::size_t node, std::optional<Point> towards, double scale)
{
	const double near = towards ? 1.5 : 10.0 * scale;
	const auto target =
			nearest_taxiway(nodes[node].at, towards, near, STAND_REACH_M * scale);
	if (!target)
		return std::nullopt;
	return cut_taxiway(
			std::get<0>(*target), std::get<1>(*target), std::get<2>(*target), node);
}

std::size_t Field::cut_taxiway(std::size_t a, std::size_t b, double t, std::size_t source)
{
	const auto length = span(a, b);
	if (t * length < 2.0)
		return a;
	if ((1.0 - t) * length < 2.0)
		return b;
	const auto at = add(nodes[a].at, mul(sub(nodes[b].at, nodes[a].at), t));
	const auto id = node_hash(nodes[a].id, nodes[b].id) ^ nodes[source].id;
	nodes.push_back({at, id});
	next.emplace_back();
	on_strip.emplace_back(std::nullopt);
	const auto foot = nodes.size() - 1;
	auto erase = [](auto &edges, std::size_t target) {
		edges.erase(std::remove_if(edges.begin(), edges.end(),
							[&](const auto &entry) { return entry.first == target; }),
				edges.end());
	};
	erase(next[a], b);
	erase(next[b], a);
	link(a, foot, Kind::Taxi);
	link(foot, b, Kind::Taxi);
	return foot;
}

void Field::add_stand(std::uint64_t id, std::vector<std::size_t> line, double scale)
{
	if (line.size() < 2)
		return;
	if (!taxis(line.front()) && taxis(line.back()))
		std::reverse(line.begin(), line.end());
	if (!taxis(line.front())) {
		std::optional<std::tuple<std::size_t, std::size_t, double, std::size_t>> on;
		for (std::size_t end = 0; end < line.size(); end += line.size() - 1)
			if (const auto target = nearest_taxiway(
						nodes[line[end]].at, std::nullopt, 0.0, 1.5)) {
				on = {std::get<0>(*target), std::get<1>(*target), std::get<2>(*target),
						end};
				break;
			}
		if (on) {
			const auto foot = cut_taxiway(std::get<0>(*on), std::get<1>(*on),
					std::get<2>(*on), line[std::get<3>(*on)]);
			if (std::get<3>(*on) != 0)
				std::reverse(line.begin(), line.end());
			line.front() = foot;
			line.erase(std::unique(line.begin(), line.end()), line.end());
			if (line.size() < 2)
				return;
		} else {
			const auto back = dir(line[1], line[0]);
			const auto foot = reach_taxiway(line.front(), back, scale);
			if (!foot)
				return;
			link(*foot, line.front(), Kind::Stand);
			line.insert(line.begin(), *foot);
		}
	}
	for (std::size_t i = 1; i < line.size(); ++i)
		link(line[i - 1], line[i], Kind::Stand);
	while (line.size() > 2 && kind(line[0], line[1]) != Kind::Stand)
		line.erase(line.begin());
	if (kind(line[0], line[1]) == Kind::Stand && !on_strip[line[0]])
		stands.push_back({id, std::move(line)});
}

void Field::add_dead_ends(double scale)
{
	const auto count = nodes.size();
	for (std::size_t end = 0; end < count; ++end) {
		if (next[end].size() != 1 || next[end][0].second != Kind::Taxi)
			continue;
		const auto from = next[end][0].first;
		if (on_strip[end] || clipping::is_invented_node_id(nodes[end].id) ||
				span(from, end) < DEAD_END_M * scale)
			continue;
		const auto length = span(from, end);
		const auto stand_length = DEAD_END_STAND_M * scale;
		const auto junction =
				length > stand_length + 15.0 * scale
						? cut_taxiway(from, end, 1.0 - stand_length / length, end)
						: from;
		for (const auto [a, b] : {std::pair{junction, end}, std::pair{end, junction}})
			if (auto it = std::find_if(next[a].begin(), next[a].end(),
						[&](const auto &entry) { return entry.first == b; });
					it != next[a].end())
				it->second = Kind::Stand;
		stands.push_back({nodes[end].id, {junction, end}});
	}
}

void Field::one_line_a_stand(double scale)
{
	std::vector<std::size_t> order(stands.size());
	for (std::size_t i = 0; i < order.size(); ++i)
		order[i] = i;
	const auto length = [&](const Stand &stand) {
		double result = 0.0;
		for (std::size_t i = 1; i < stand.path.size(); ++i)
			result += span(stand.path[i - 1], stand.path[i]);
		return result;
	};
	std::sort(order.begin(), order.end(), [&](auto a, auto b) {
		const auto la = length(stands[a]), lb = length(stands[b]);
		return la != lb ? la > lb : stands[a].id < stands[b].id;
	});
	std::vector<std::size_t> keep;
	for (const auto i : order) {
		const auto nose = nodes[stands[i].path.back()].at;
		const bool duplicate = std::any_of(keep.begin(), keep.end(), [&](auto other) {
			return norm(sub(nodes[stands[other].path.back()].at, nose)) <
				   SAME_STAND_M * scale;
		});
		if (!duplicate)
			keep.push_back(i);
	}
	std::sort(keep.begin(), keep.end());
	std::vector<Stand> unique;
	unique.reserve(keep.size());
	for (const auto i : keep)
		unique.push_back(std::move(stands[i]));
	stands = std::move(unique);
}

std::vector<std::size_t> Field::fields() const
{
	std::vector<std::size_t> component(
			nodes.size(), std::numeric_limits<std::size_t>::max());
	std::size_t count = 0;
	for (std::size_t start = 0; start < nodes.size(); ++start) {
		if (component[start] != std::numeric_limits<std::size_t>::max())
			continue;
		std::vector<std::size_t> stack{start};
		component[start] = count;
		while (!stack.empty()) {
			const auto node = stack.back();
			stack.pop_back();
			for (const auto &[other, unused] : next[node]) {
				(void)unused;
				if (component[other] == std::numeric_limits<std::size_t>::max()) {
					component[other] = count;
					stack.push_back(other);
				}
			}
		}
		++count;
	}
	return component;
}

std::optional<std::vector<std::size_t>> Field::taxi_route(
		std::pair<std::size_t, std::size_t> from,
		const std::vector<std::pair<std::size_t, std::size_t>> &free_edges,
		double runway_weight, double scale,
		const std::function<std::optional<double>(std::size_t, std::size_t)> &goal) const
{
	using State = std::pair<std::size_t, std::size_t>;
	struct Open
	{
		double cost;
		State state;
		bool operator<(const Open &other) const
		{
			if (cost != other.cost)
				return cost > other.cost;
			return state > other.state;
		}
	};
	std::set<std::pair<std::size_t, std::size_t>> free;
	for (const auto &[a, b] : free_edges)
		free.insert({a, b});
	std::map<State, std::pair<double, State>> best;
	std::priority_queue<Open> open;
	best[from] = {0.0, from};
	open.push({0.0, from});
	std::optional<std::pair<double, State>> found;
	while (!open.empty()) {
		const auto [cost, state] = open.top();
		open.pop();
		const auto [a, b] = state;
		if (best.contains(state) && cost > best[state].first)
			continue;
		if (found && cost >= found->first)
			break;
		if (const auto extra = goal(a, b);
				extra && (!found || cost + *extra < found->first))
			found = {cost + *extra, state};
		for (const auto &[c, type] : next[b]) {
			if (type == Kind::Stand)
				continue;
			const auto turn = turn_deg(dir(a, b), dir(b, c));
			const double weight =
					type == Kind::Runway && !free.contains({b, c}) ? runway_weight : 1.0;
			const double total = cost + span(b, c) * weight + turn_cost(turn) * scale;
			const State successor{b, c};
			if (!best.contains(successor) || total < best[successor].first) {
				best[successor] = {total, state};
				open.push({total, successor});
			}
		}
	}
	if (!found)
		return std::nullopt;
	std::vector<std::size_t> path{found->second.second};
	auto state = found->second;
	while (state != from) {
		path.push_back(state.first);
		state = best.at(state).second;
	}
	path.push_back(from.first);
	std::reverse(path.begin(), path.end());
	return path;
}

} // namespace arnis::element_processing::aircraft
