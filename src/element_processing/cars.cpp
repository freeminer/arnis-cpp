#include "cars.h"

#include "highways.h"
#include "road_markings.h"
#include "traffic_lights.h"
#include "../clipping.h"
#include "../strict_parse.h"
#include "../../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <string_view>
#include <tuple>
#include <unordered_map>

namespace arnis::highways::cars
{
namespace
{
constexpr double MAX_SPAN = 24.0;
constexpr std::size_t MAX_CARS = 1500;
constexpr double STOP_BACK = 1.5;
constexpr double JUNCTION_KMH = 25.0;
constexpr double LANE_END_KMH = 32.0;
constexpr double LANE_END_REACH = 10.0;
constexpr double U_TURN_KMH = 12.0;
constexpr double ENTRY_TURN_DEG = 135.0;
constexpr double SHARP_DEG = 100.0;
constexpr double SHORT_LANE = 3.0;
constexpr double CROSSING_REACH = 40.0;
constexpr double HAIRPIN_SWING = 3.0;
constexpr double LIMIT_SHARE = 0.92;
constexpr std::array<std::pair<std::string_view, std::uint32_t>, 8> CAR_COLOURS{
		{{"minecraft:white_concrete", 24}, {"minecraft:black_concrete", 20},
				{"minecraft:gray_concrete", 17}, {"minecraft:light_gray_concrete", 14},
				{"minecraft:blue_concrete", 9}, {"minecraft:red_concrete", 8},
				{"minecraft:cyan_concrete", 4}, {"minecraft:green_concrete", 4}}};
constexpr std::array<std::pair<std::string_view, std::uint32_t>, 3> VAN_COLOURS{
		{{"minecraft:white_concrete", 70}, {"minecraft:light_gray_concrete", 20},
				{"minecraft:gray_concrete", 10}}};

struct Path
{
	std::vector<std::pair<double, double>> points;
	std::vector<double> at;

	double length() const { return at.empty() ? 0.0 : at.back(); }
	std::pair<double, double> point_at(double distance) const
	{
		if (points.empty())
			return {};
		if (points.size() == 1)
			return points.front();
		distance = std::clamp(distance, 0.0, length());
		const auto it = std::lower_bound(at.begin(), at.end(), distance);
		const auto k = std::clamp<std::size_t>(
				static_cast<std::size_t>(it - at.begin()), 1, points.size() - 1);
		const auto &[ax, az] = points[k - 1];
		const auto &[bx, bz] = points[k];
		const double span = at[k] - at[k - 1];
		const double t = span > 0.0 ? (distance - at[k - 1]) / span : 0.0;
		return {ax + (bx - ax) * t, az + (bz - az) * t};
	}
	double bend(std::size_t k) const
	{
		if (k == 0 || k + 1 >= points.size())
			return 0.0;
		const auto unit = [](double x,
								  double z) -> std::optional<std::pair<double, double>> {
			const double n = std::hypot(x, z);
			if (n <= 1e-9)
				return std::nullopt;
			return std::pair{x / n, z / n};
		};
		const auto a = unit(points[k].first - points[k - 1].first,
				points[k].second - points[k - 1].second);
		const auto b = unit(points[k + 1].first - points[k].first,
				points[k + 1].second - points[k].second);
		if (!a || !b)
			return 0.0;
		return std::acos(std::clamp(
					   a->first * b->first + a->second * b->second, -1.0, 1.0)) *
			   180.0 / std::numbers::pi;
	}
};

struct Piece
{
	const ProcessedWay *way{};
	std::vector<const ProcessedNode *> nodes;
	int half{};
	int oneway{};
};

struct Lane
{
	std::size_t piece{};
	std::vector<const ProcessedNode *> nodes;
	std::optional<std::size_t> reverse;
	bool two_way{};
};

struct Mark
{
	double distance{};
	std::optional<std::uint8_t> stop;
	std::optional<std::int32_t> cap;
};

std::int32_t step_speed(double kmh, double scale)
{
	const double rounded = std::round(kmh / 3.6 * 0.1 * scale * 1000.0);
	if (std::isnan(rounded))
		return 0;
	if (rounded >= std::numeric_limits<std::int32_t>::max())
		return std::numeric_limits<std::int32_t>::max();
	if (rounded <= std::numeric_limits<std::int32_t>::min())
		return std::numeric_limits<std::int32_t>::min();
	return static_cast<std::int32_t>(rounded);
}

std::optional<double> parse_maxspeed(const std::string &raw)
{
	std::string value = raw;
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
		return static_cast<char>(c >= 'A' && c <= 'Z' ? c + ('a' - 'A') : c);
	});
	const auto trim = [](std::string &s) {
		const auto first = s.find_first_not_of(" \t\r\n");
		if (first == std::string::npos) {
			s.clear();
			return;
		}
		s.erase(0, first);
		s.erase(s.find_last_not_of(" \t\r\n") + 1);
	};
	trim(value);
	if (value.ends_with("mph")) {
		value.resize(value.size() - 3);
		trim(value);
		if (const auto mph = strict_parse::f64(value))
			return *mph * 1.609;
		return std::nullopt;
	}
	if (const auto kmh = strict_parse::f64(value); kmh && *kmh > 0.0)
		return *kmh;
	if (value.find("zone") != std::string::npos) {
		std::string digits;
		std::copy_if(value.begin(), value.end(), std::back_inserter(digits),
				[](unsigned char c) { return c >= '0' && c <= '9'; });
		if (const auto zone_speed = strict_parse::f64(digits))
			return *zone_speed;
	}
	const auto colon = value.rfind(':');
	const auto zone = value.substr(colon == std::string::npos ? 0 : colon + 1);
	if (zone == "walk" || zone == "living_street")
		return 10.0;
	if (zone == "urban")
		return 50.0;
	if (zone == "rural")
		return 90.0;
	if (zone == "trunk")
		return 100.0;
	if (zone == "motorway")
		return 120.0;
	if (zone == "none")
		return 130.0;
	return std::nullopt;
}

double limit_kmh(const std::string &highway, const tags_t &tags)
{
	if (const auto it = tags.find("maxspeed"); it != tags.end())
		if (const auto posted = parse_maxspeed(it->second))
			return *posted;
	if (highway == "motorway")
		return 110.0;
	if (highway == "trunk")
		return 80.0;
	if (highway == "motorway_link" || highway == "trunk_link")
		return 60.0;
	if (highway == "primary" || highway == "secondary" || highway == "tertiary")
		return 50.0;
	if (highway == "primary_link" || highway == "secondary_link" ||
			highway == "tertiary_link" || highway == "unclassified" || highway == "road")
		return 40.0;
	if (highway == "residential")
		return 30.0;
	if (highway == "service")
		return 20.0;
	if (highway == "living_street")
		return 10.0;
	return 30.0;
}

double density(const std::string &highway)
{
	if (highway == "motorway" || highway == "trunk" || highway == "motorway_link" ||
			highway == "trunk_link")
		return 1.0 / 60.0;
	if (highway == "primary" || highway == "secondary" || highway == "primary_link" ||
			highway == "secondary_link")
		return 1.0 / 40.0;
	if (highway == "tertiary" || highway == "tertiary_link")
		return 1.0 / 60.0;
	if (highway == "unclassified" || highway == "road")
		return 1.0 / 150.0;
	if (highway == "residential")
		return 1.0 / 200.0;
	if (highway == "service")
		return 1.0 / 400.0;
	return 1.0 / 500.0;
}

bool drivable(const tags_t &tags)
{
	const auto get = [&](const char *key) -> std::string_view {
		const auto it = tags.find(key);
		return it == tags.end() ? std::string_view{} : std::string_view(it->second);
	};
	const auto has_non_no_value = [&](const char *key) {
		const auto it = tags.find(key);
		return it != tags.end() && it->second != "no";
	};
	const auto highway = get("highway");
	if (highway.empty() || !is_vehicular(std::string(highway)) || highway == "busway" ||
			get("area") == "yes" || get("indoor") == "yes")
		return false;
	const bool bridge = has_non_no_value("bridge");
	int layer_value = 0;
	if (const auto it = tags.find("layer"); it != tags.end() && !it->second.empty()) {
		const auto *begin = it->second.data();
		const auto *end = begin + it->second.size();
		if (*begin == '+')
			++begin;
		const auto [parsed_end, error] = std::from_chars(begin, end, layer_value);
		if (begin == end || error != std::errc{} || parsed_end != end)
			layer_value = 0;
	}
	if (has_non_no_value("tunnel") || (layer_value != 0 && !bridge))
		return false;
	for (const auto *key : {"access", "vehicle", "motor_vehicle", "motorcar"}) {
		const auto value = get(key);
		if (value == "no" || value == "private")
			return false;
	}
	const auto service = get("service");
	return service != "parking_aisle" && service != "driveway";
}

std::optional<std::pair<double, double>> unit(double x, double z)
{
	const double len = std::hypot(x, z);
	if (len <= 1e-9)
		return std::nullopt;
	return std::pair{x / len, z / len};
}

Path make_path(std::vector<std::pair<double, double>> points)
{
	Path path;
	path.points = std::move(points);
	if (path.points.empty())
		return path;
	path.at.reserve(path.points.size());
	path.at.push_back(0.0);
	for (std::size_t i = 1; i < path.points.size(); ++i) {
		const auto &[ax, az] = path.points[i - 1];
		const auto &[bx, bz] = path.points[i];
		path.at.push_back(path.at.back() + std::hypot(bx - ax, bz - az));
	}
	return path;
}

std::vector<std::pair<double, double>> lane_line(
		const std::vector<const ProcessedNode *> &nodes, double offset, bool left)
{
	std::vector<std::pair<double, double>> centre;
	centre.reserve(nodes.size());
	for (const auto *node : nodes)
		centre.emplace_back(node->x + 0.5, node->z + 0.5);
	const auto direction = [&](std::size_t a, std::size_t b) {
		return unit(
				centre[b].first - centre[a].first, centre[b].second - centre[a].second);
	};
	std::vector<std::pair<double, double>> result;
	result.reserve(centre.size());
	for (std::size_t k = 0; k < centre.size(); ++k) {
		const auto into = k > 0 ? direction(k - 1, k) : std::nullopt;
		const auto out = k + 1 < centre.size() ? direction(k, k + 1) : std::nullopt;
		std::pair<double, double> d{1.0, 0.0};
		if (into && out)
			d = unit(into->first + out->first, into->second + out->second).value_or(*out);
		else if (into)
			d = *into;
		else if (out)
			d = *out;
		const std::pair<double, double> side =
				left ? std::pair{d.second, -d.first} : std::pair{-d.second, d.first};
		result.emplace_back(centre[k].first + side.first * offset,
				centre[k].second + side.second * offset);
	}
	return result;
}

struct RoadRef
{
	const ProcessedWay *way{};
	int half{};
	int oneway{};
};

std::optional<std::int32_t> lower(
		std::optional<std::int32_t> a, std::optional<std::int32_t> b);
void refine(double a, double b, const std::function<double(double)> &y_at, unsigned depth,
		std::vector<Mark> &out);
void tidy(Segment &segment);
void pace(Segment &segment);
void link(std::vector<Segment> &segments, const std::vector<Lane> &lanes,
		const ::XZBBox &world, const std::function<std::int32_t(double)> &speed_at);
std::vector<Car> place_cars(const std::vector<Segment> &segments, double scale);

} // namespace

CarNetwork CarNetwork::build(const std::vector<ProcessedElement> &elements, double scale,
		bool drives_on_left, bool stop_at_signals, const ::XZBBox &world,
		const std::function<int(int, int)> &surface_y)
{
	std::vector<RoadRef> roads;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !drivable(way.tags))
			continue;
		const auto highway_it = way.tags.find("highway");
		if (highway_it == way.tags.end())
			continue;
		roads.push_back({&way, highway_block_range(highway_it->second, way.tags, scale),
				road_markings::oneway_sign(highway_it->second, way.tags)});
	}

	std::unordered_map<std::uint64_t, unsigned> arms;
	std::unordered_map<std::uint64_t, int> widest;
	for (const auto &road : roads) {
		const auto &nodes = road.way->nodes;
		const bool closed = nodes.size() > 2 && nodes.front().id == nodes.back().id;
		const std::size_t begin = closed ? 1 : 0;
		const std::size_t end = nodes.size();
		for (std::size_t i = begin; i < end; ++i) {
			const bool endpoint = !closed && (i == 0 || i + 1 == end);
			arms[nodes[i].id] += endpoint ? 1 : 2;
			widest[nodes[i].id] = std::max(widest[nodes[i].id], road.half);
		}
	}
	const auto is_junction = [&](std::uint64_t id) {
		const auto it = arms.find(id);
		return it != arms.end() && it->second >= 3;
	};

	std::vector<Piece> pieces;
	for (const auto &road : roads) {
		const auto &nodes = road.way->nodes;
		std::size_t start = 0;
		for (std::size_t k = 1; k < nodes.size(); ++k) {
			if (k + 1 == nodes.size() || is_junction(nodes[k].id)) {
				Piece piece{road.way, {}, road.half, road.oneway};
				piece.nodes.reserve(k - start + 1);
				for (std::size_t i = start; i <= k; ++i)
					piece.nodes.push_back(&nodes[i]);
				pieces.push_back(std::move(piece));
				start = k;
			}
		}
	}

	std::vector<Lane> lanes;
	for (std::size_t p = 0; p < pieces.size(); ++p) {
		const auto &piece = pieces[p];
		const std::size_t first = lanes.size();
		if (piece.oneway >= 0)
			lanes.push_back({p, piece.nodes, std::nullopt, piece.oneway == 0});
		if (piece.oneway <= 0) {
			auto reversed = piece.nodes;
			std::reverse(reversed.begin(), reversed.end());
			lanes.push_back({p, std::move(reversed), std::nullopt, piece.oneway == 0});
		}
		if (piece.oneway == 0) {
			lanes[first].reverse = first + 1;
			lanes[first + 1].reverse = first;
		}
	}

	const double upstream = std::max(UPSTREAM_M * scale, 6.0);
	std::vector<Segment> segments;
	segments.reserve(lanes.size());
	for (const auto &lane : lanes) {
		const auto &piece = pieces[lane.piece];
		const auto &way = *piece.way;
		const auto highway_it = way.tags.find("highway");
		if (highway_it == way.tags.end()) {
			segments.emplace_back();
			continue;
		}
		const auto &highway = highway_it->second;
		const auto speed = step_speed(limit_kmh(highway, way.tags) * LIMIT_SHARE, scale);
		const auto capped = [&](double kmh) -> std::optional<std::int32_t> {
			const auto cap = step_speed(kmh, scale);
			return cap < speed ? std::optional<std::int32_t>(cap) : std::nullopt;
		};
		const double offset =
				lane.two_way ? std::max((piece.half + 1.0) / 2.0, 1.0) : 0.0;
		const Path path = make_path(lane_line(lane.nodes, offset, drives_on_left));
		if (lane.nodes.empty()) {
			segments.emplace_back();
			continue;
		}
		const auto first_id = lane.nodes.front()->id;
		const auto *end_node = lane.nodes.back();
		const auto last_id = end_node->id;
		const bool edge =
				clipping::is_invented_node_id(last_id) ||
				end_node->x - world.min_x() < 3 || world.max_x() - end_node->x < 3 ||
				end_node->z - world.min_z() < 3 || world.max_z() - end_node->z < 3;
		const auto arm_count = arms.contains(last_id) ? arms.at(last_id) : 0;
		const bool turns_back = lane.reverse.has_value() && arm_count <= 1 && !edge;
		const auto trim = [&](std::uint64_t id) {
			if (!is_junction(id))
				return 0.0;
			const auto it = widest.find(id);
			return (it == widest.end() ? 1 : it->second) + 1.0;
		};
		double from = trim(first_id), to = path.length() - trim(last_id);
		if (to - from < 2.0) {
			const double mid = std::clamp((from + to) / 2.0, 0.0, path.length());
			from = std::max(mid - 1.0, 0.0);
			to = std::min(mid + 1.0, path.length());
		}
		std::vector<Mark> marks{
				{from, std::nullopt,
						is_junction(first_id) ? capped(JUNCTION_KMH) : std::nullopt},
				{to, std::nullopt,
						(is_junction(last_id) || turns_back) ? capped(LANE_END_KMH)
															 : std::nullopt}};
		if ((is_junction(last_id) || turns_back) && to - from > LANE_END_REACH + 2.0)
			marks.push_back({to - LANE_END_REACH, std::nullopt, std::nullopt});
		for (std::size_t k = 1; k + 1 < path.points.size(); ++k) {
			const double s = path.at[k];
			if (s <= from || s >= to)
				continue;
			const double bend = path.bend(k);
			marks.push_back({s, std::nullopt,
					bend > 25.0 ? capped(std::max(70.0 - 0.6 * bend, 12.0))
								: std::nullopt});
		}
		if (stop_at_signals) {
			std::vector<ProcessedNode> along;
			along.reserve(lane.nodes.size());
			for (const auto *node : lane.nodes)
				along.push_back(*node);
			const bool forward_way = piece.nodes.front()->id == first_id;
			for (std::size_t k = 1; k < lane.nodes.size(); ++k) {
				const auto *node = lane.nodes[k];
				if (!is_signal(node->tags))
					continue;
				const auto dir_it = node->tags.find("traffic_signals:direction");
				const auto dir = dir_it == node->tags.end()
										 ? std::string_view{}
										 : std::string_view(dir_it->second);
				const bool held = dir == "forward"	  ? forward_way
								  : dir == "backward" ? !forward_way
													  : true;
				if (!held || signal_upstream(along, k, -1, upstream))
					continue;
				const auto heading = unit(
						node->x - lane.nodes[k - 1]->x, node->z - lane.nodes[k - 1]->z)
											 .value_or(std::pair{1.0, 0.0});
				const std::uint8_t group =
						group_of(heading.first, heading.second) == 'a' ? 1 : 2;
				const double s = k + 1 == lane.nodes.size()
										 ? to
										 : path.at[k] - STOP_BACK * std::max(scale, 0.5);
				marks.push_back({std::clamp(s, from, to), group, std::nullopt});
			}
		}
		std::stable_sort(marks.begin(), marks.end(),
				[](const Mark &a, const Mark &b) { return a.distance < b.distance; });
		std::vector<Mark> merged;
		for (const auto &mark : marks) {
			if (!merged.empty() && mark.distance - merged.back().distance < 0.5) {
				if (!merged.back().stop)
					merged.back().stop = mark.stop;
				merged.back().cap = lower(merged.back().cap, mark.cap);
			} else {
				merged.push_back(mark);
			}
		}
		const auto y_at = [&](double s) {
			const auto [x, z] = path.point_at(s);
			return static_cast<double>(surface_y(static_cast<int>(std::floor(x)),
											   static_cast<int>(std::floor(z))) +
									   1);
		};
		std::vector<Mark> stations;
		for (const auto &mark : merged) {
			if (!stations.empty())
				refine(stations.back().distance, mark.distance, y_at, 0, stations);
			stations.push_back(mark);
		}
		Segment segment;
		segment.way = way.id;
		segment.into_junction = is_junction(last_id);
		segment.turns_back = turns_back;
		segment.speed = speed;
		segment.density = density(highway);
		for (const auto &station : stations) {
			const auto [x, z] = path.point_at(station.distance);
			const int ix = static_cast<int>(std::floor(x));
			const int iz = static_cast<int>(std::floor(z));
			if (!world.contains(ix, iz))
				continue;
			segment.points.push_back({std::round(x * 10.0) / 10.0, y_at(station.distance),
					std::round(z * 10.0) / 10.0, station.stop, station.cap});
			segment.arcs.push_back(station.distance);
		}
		segments.push_back(std::move(segment));
	}

	for (auto &segment : segments)
		tidy(segment);
	link(segments, lanes, world, [scale](double kmh) { return step_speed(kmh, scale); });
	for (auto &segment : segments)
		pace(segment);
	CarNetwork result;
	result.cars_ = place_cars(segments, scale);
	for (std::size_t i = 0; i < result.cars_.size(); ++i)
		result.by_way_[segments[result.cars_[i].segment].way].push_back(i);
	result.segments_ = std::move(segments);
	result.size_ = std::clamp(scale, 0.4, 3.0);
	result.scale_ = scale;
	return result;
}

const std::vector<std::size_t> &CarNetwork::cars_for_way(std::uint64_t way) const
{
	static const std::vector<std::size_t> empty;
	const auto it = by_way_.find(way);
	return it == by_way_.end() ? empty : it->second;
}

namespace
{
std::optional<std::int32_t> lower(
		std::optional<std::int32_t> a, std::optional<std::int32_t> b)
{
	if (a && b)
		return std::min(*a, *b);
	return a ? a : b;
}

void refine(double a, double b, const std::function<double(double)> &y_at, unsigned depth,
		std::vector<Mark> &out)
{
	if (depth >= 6 || b - a < 3.0)
		return;
	const auto pieces = static_cast<std::size_t>(std::ceil((b - a) / MAX_SPAN));
	if (pieces > 1) {
		const double step = (b - a) / pieces;
		for (std::size_t j = 0; j < pieces; ++j) {
			const double end = a + step * (j + 1);
			refine(end - step, end, y_at, depth + 1, out);
			if (j + 1 < pieces)
				out.push_back({end, std::nullopt, std::nullopt});
		}
		return;
	}
	const double mid = (a + b) * 0.5;
	if (std::abs(y_at(mid) - (y_at(a) + y_at(b)) * 0.5) >= 1.0) {
		refine(a, mid, y_at, depth + 1, out);
		out.push_back({mid, std::nullopt, std::nullopt});
		refine(mid, b, y_at, depth + 1, out);
	}
}

std::vector<std::tuple<double, double, double>> curve(std::pair<double, double> from,
		std::pair<double, double> c1, std::pair<double, double> c2,
		std::pair<double, double> to)
{
	constexpr std::size_t FINE = 128;
	const auto at = [&](double t) {
		const double u = 1.0 - t;
		const auto bezier = [&](double p0, double p1, double p2, double p3) {
			return u * u * u * p0 + 3.0 * u * u * t * p1 + 3.0 * u * t * t * p2 +
				   t * t * t * p3;
		};
		return std::pair{bezier(from.first, c1.first, c2.first, to.first),
				bezier(from.second, c1.second, c2.second, to.second)};
	};
	const auto gap = [](auto p, auto q) {
		return std::hypot(p.first - q.first, p.second - q.second);
	};
	std::vector<std::tuple<double, double, double>> out;
	auto last = from;
	auto was = from;
	std::optional<std::pair<double, double>> heading;
	double turned = 0.0, run = 0.0;
	for (std::size_t k = 1; k < FINE; ++k) {
		const double t = static_cast<double>(k) / FINE;
		const auto p = at(t);
		run += gap(p, was);
		if (const auto h = unit(p.first - was.first, p.second - was.second)) {
			if (heading)
				turned += std::acos(std::clamp(
								  heading->first * h->first + heading->second * h->second,
								  -1.0, 1.0)) *
						  180.0 / std::numbers::pi;
			heading = h;
		}
		was = p;
		if ((turned >= 25.0 || run >= 4.0) && gap(p, last) >= 0.5 && gap(p, to) >= 0.5) {
			out.emplace_back(p.first, p.second, t);
			last = p;
			turned = run = 0.0;
		}
	}
	return out;
}

void tidy(Segment &segment)
{
	const auto dir = [](const Waypoint &p, const Waypoint &q) {
		return unit(q.x - p.x, q.z - p.z);
	};
	const auto gap = [](const Waypoint &p, const Waypoint &q) {
		return std::hypot(q.x - p.x, q.z - p.z);
	};
	const auto turn = [&](std::size_t k) {
		if (k == 0 || k + 1 >= segment.points.size())
			return 180.0;
		const auto a = dir(segment.points[k - 1], segment.points[k]);
		const auto b = dir(segment.points[k], segment.points[k + 1]);
		if (!a || !b)
			return 180.0;
		return std::acos(std::clamp(
					   a->first * b->first + a->second * b->second, -1.0, 1.0)) *
			   180.0 / std::numbers::pi;
	};
	for (;;) {
		std::optional<std::size_t> remove;
		for (std::size_t k = 1; k + 1 < segment.points.size(); ++k)
			if (!segment.points[k].stop && turn(k) > 120.0 &&
					std::min(gap(segment.points[k - 1], segment.points[k]),
							gap(segment.points[k], segment.points[k + 1])) < 3.0) {
				remove = k;
				break;
			}
		if (!remove)
			break;
		const auto gone = segment.points[*remove];
		segment.points.erase(segment.points.begin() + *remove);
		segment.arcs.erase(segment.arcs.begin() + *remove);
		segment.points[*remove].cap = lower(segment.points[*remove].cap, gone.cap);
	}
	const auto old = std::move(segment.points);
	const auto old_arcs = std::move(segment.arcs);
	segment.points.clear();
	segment.arcs.clear();
	for (std::size_t k = 0; k < old.size(); ++k) {
		const auto p = old[k];
		const double s = k < old_arcs.size() ? old_arcs[k] : 0.0;
		std::optional<
				std::tuple<std::pair<double, double>, std::pair<double, double>, double>>
				corner;
		if (k > 0 && k + 1 < old.size() && !p.stop) {
			const auto a = dir(old[k - 1], p);
			const auto b = dir(p, old[k + 1]);
			if (a && b) {
				const double d = std::min(
						0.45 * std::min(gap(old[k - 1], p), gap(p, old[k + 1])), 3.0);
				if (turn(k) > 40.0 && d >= 0.5)
					corner = std::tuple{*a, *b, d};
			}
		}
		if (!corner) {
			segment.points.push_back(p);
			segment.arcs.push_back(s);
			continue;
		}
		const auto &[a, b, d] = *corner;
		const std::pair from{p.x - a.first * d, p.z - a.second * d};
		const std::pair to{p.x + b.first * d, p.z + b.second * d};
		std::pair<double, double> c1, c2;
		if (turn(k) <= 150.0) {
			const double r = d * 2.0 / 3.0;
			c1 = {from.first + a.first * r, from.second + a.second * r};
			c2 = {to.first - b.first * r, to.second - b.second * r};
		} else {
			const double ab = a.first * b.first + a.second * b.second;
			const auto n = unit(a.first * ab - b.first, a.second * ab - b.second)
								   .value_or(std::pair{-a.second, a.first});
			const std::pair tip{p.x + a.first * d, p.z + a.second * d};
			c1 = {tip.first + n.first * HAIRPIN_SWING,
					tip.second + n.second * HAIRPIN_SWING};
			c2 = {tip.first - n.first * HAIRPIN_SWING,
					tip.second - n.second * HAIRPIN_SWING};
		}
		std::vector<std::tuple<double, double, double>> points{
				{from.first, from.second, 0.0}};
		auto mid = curve(from, c1, c2, to);
		points.insert(points.end(), mid.begin(), mid.end());
		points.emplace_back(to.first, to.second, 1.0);
		for (const auto &[x, z, t] : points) {
			segment.points.push_back({std::round(x * 10.0) / 10.0, p.y,
					std::round(z * 10.0) / 10.0, p.stop, p.cap});
			segment.arcs.push_back(s + d * (2.0 * t - 1.0));
		}
	}
}

void pace(Segment &segment)
{
	for (std::size_t k = 2; k < segment.points.size(); ++k) {
		const auto &a = segment.points[k - 2];
		const auto &b = segment.points[k - 1];
		const auto &c = segment.points[k];
		const auto into = unit(b.x - a.x, b.z - a.z);
		const auto out = unit(c.x - b.x, c.z - b.z);
		if (!into || !out ||
				into->first * out->first + into->second * out->second >
						std::cos(30.0 * std::numbers::pi / 180.0))
			continue;
		const auto room =
				static_cast<std::int32_t>(std::hypot(c.x - b.x, c.z - b.z) * 950.0);
		const auto cap = std::max(room, 100);
		if (cap < segment.speed)
			segment.points[k].cap = lower(segment.points[k].cap, cap);
	}
}

std::uint64_t hash(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t h = a * 0x9E3779B97F4A7C15ULL ^ b * 0xC2B2AE3D27D4EB4FULL;
	h ^= h >> 29;
	h *= 0xBF58476D1CE4E5B9ULL;
	return h ^ (h >> 32);
}

template <std::size_t N>
std::string_view pick(
		const std::array<std::pair<std::string_view, std::uint32_t>, N> &choices,
		std::uint64_t roll)
{
	std::uint32_t total = 0;
	for (const auto &choice : choices)
		total += choice.second;
	std::uint32_t at = static_cast<std::uint32_t>(roll % total);
	for (const auto &choice : choices) {
		if (at < choice.second)
			return choice.first;
		at -= choice.second;
	}
	return choices.front().first;
}

bool usable(const Segment &segment)
{
	return segment.points.size() >= 2;
}

enum class Light
{
	None,
	Head,
	Tail
};

struct ModelPart
{
	std::string block;
	std::array<float, 3> size{};
	std::array<float, 3> corner{};
	Light light{Light::None};
};

std::vector<ModelPart> body_parts(VehicleKind kind, const std::string &colour)
{
	float length, width, axle, wheel, lights;
	switch (kind) {
	case VehicleKind::Sedan:
		length = 4.6f;
		width = 1.82f;
		axle = 0.95f;
		wheel = 0.64f;
		lights = 0.66f;
		break;
	case VehicleKind::Hatchback:
		length = 4.0f;
		width = 1.76f;
		axle = 0.75f;
		wheel = 0.62f;
		lights = 0.68f;
		break;
	case VehicleKind::Van:
		length = 4.9f;
		width = 1.95f;
		axle = 0.85f;
		wheel = 0.68f;
		lights = 0.75f;
		break;
	}
	const auto part = [](std::string block, std::array<float, 3> size,
							  std::array<float, 3> corner, Light light = Light::None) {
		return ModelPart{std::move(block), size, corner, light};
	};
	const auto wheels = [&](float z) {
		return part("minecraft:black_concrete", {width + 0.08f, wheel, wheel},
				{-(width + 0.08f) / 2.0f, 0.0f, z - wheel / 2.0f});
	};
	const auto lamp = [&](const char *block, float z, Light light) {
		return part(block, {width - 0.3f, 0.13f, 0.04f},
				{-(width - 0.3f) / 2.0f, lights, z}, light);
	};
	std::vector<ModelPart> out{wheels(length / 2.0f - axle),
			wheels(-length / 2.0f + axle),
			lamp("minecraft:white_concrete", length / 2.0f, Light::Head),
			lamp("minecraft:red_concrete", -length / 2.0f - 0.04f, Light::Tail)};
	if (kind == VehicleKind::Sedan) {
		out.push_back(part(
				colour, {width, 0.72f, length}, {-width / 2.0f, 0.22f, -length / 2.0f}));
		out.push_back(part("minecraft:tinted_glass", {width - 0.2f, 0.52f, 2.3f},
				{-(width - 0.2f) / 2.0f, 0.94f, -1.3f}));
		out.push_back(part(colour, {width - 0.18f, 0.07f, 2.0f},
				{-(width - 0.18f) / 2.0f, 1.43f, -1.15f}));
	} else if (kind == VehicleKind::Hatchback) {
		out.push_back(part(
				colour, {width, 0.72f, length}, {-width / 2.0f, 0.22f, -length / 2.0f}));
		out.push_back(part("minecraft:tinted_glass", {width - 0.18f, 0.56f, 2.3f},
				{-(width - 0.18f) / 2.0f, 0.94f, -1.78f}));
		out.push_back(part(colour, {width - 0.16f, 0.07f, 2.1f},
				{-(width - 0.16f) / 2.0f, 1.47f, -1.72f}));
	} else {
		out.push_back(part(
				colour, {width, 1.5f, length}, {-width / 2.0f, 0.3f, -length / 2.0f}));
		out.push_back(part("minecraft:tinted_glass", {width + 0.04f, 0.55f, 1.0f},
				{-(width + 0.04f) / 2.0f, 1.15f, length / 2.0f - 0.98f}));
	}
	return out;
}

std::optional<std::pair<std::size_t, double>> entry(
		const std::vector<Waypoint> &from, const std::vector<Waypoint> &to)
{
	if (from.size() < 2 || to.empty())
		return std::nullopt;
	const auto &end = from.back();
	std::optional<std::pair<double, double>> heading;
	for (std::size_t k = from.size() - 1; k > 0 && !heading; --k)
		heading = unit(end.x - from[k - 1].x, end.z - from[k - 1].z);
	if (!heading)
		return std::nullopt;
	const auto turn = [](auto a, auto b) {
		return std::acos(std::clamp(a.first * b.first + a.second * b.second, -1.0, 1.0)) *
			   180.0 / std::numbers::pi;
	};
	const auto fits =
			[&](std::size_t e,
					double room) -> std::optional<std::pair<std::size_t, double>> {
		const double dx = to[e].x - end.x, dz = to[e].z - end.z;
		const double length = std::hypot(dx, dz);
		const auto leg = length >= room ? unit(dx, dz) : std::nullopt;
		if (!leg)
			return std::nullopt;
		const double onto = turn(*heading, *leg);
		double on = 0.0;
		if (e + 1 < to.size())
			if (const auto next = unit(to[e + 1].x - to[e].x, to[e + 1].z - to[e].z))
				on = turn(*leg, *next);
		if (onto < 90.0 && on <= 90.0 && onto + on <= ENTRY_TURN_DEG)
			return std::pair{e, onto + on};
		return std::nullopt;
	};
	for (double room : {1.0, 0.0})
		for (std::size_t e = 0; e < to.size(); ++e)
			if (const auto result = fits(e, room))
				return result;
	return std::nullopt;
}

bool bend_into(std::vector<Segment> &segments, std::size_t i, std::size_t j,
		const ::XZBBox &world, const std::function<std::int32_t(double)> &speed_at)
{
	if (i >= segments.size() || j >= segments.size())
		return false;
	const auto &segment = segments[i];
	if (segment.points.size() < 2 || segments[j].points.empty())
		return false;
	const auto a = segment.points.back();
	std::optional<std::pair<double, double>> heading;
	for (std::size_t k = segment.points.size() - 1; k > 0 && !heading; --k)
		heading = unit(a.x - segment.points[k - 1].x, a.z - segment.points[k - 1].z);
	if (!heading)
		return false;
	const auto &to = segments[j].points;
	const auto along = [&](std::size_t k) -> std::optional<std::pair<double, double>> {
		if (to.size() < 2)
			return std::nullopt;
		const std::size_t p = k + 1 < to.size() ? k : k - 1;
		return unit(to[p + 1].x - to[p].x, to[p + 1].z - to[p].z);
	};
	const auto first_out = along(0);
	if (!first_out)
		return false;
	const double turn = std::acos(std::clamp(heading->first * first_out->first +
													 heading->second * first_out->second,
								-1.0, 1.0)) *
						180.0 / std::numbers::pi;
	Waypoint meet;
	std::pair<double, double> out;
	bool onto_meet = false;
	if (turn > 150.0) {
		const auto &b = to.front();
		const double back = std::clamp(
				(b.x - a.x) * first_out->first + (b.z - a.z) * first_out->second, 0.0,
				12.0);
		meet = {b.x - first_out->first * back, b.y, b.z - first_out->second * back,
				std::nullopt, std::nullopt};
		out = *first_out;
		onto_meet = back > 0.5;
	} else {
		std::size_t k = to.size() - 1;
		for (std::size_t candidate = 0; candidate < to.size(); ++candidate) {
			const double dx = to[candidate].x - a.x, dz = to[candidate].z - a.z;
			const auto direction = along(candidate);
			if (std::hypot(dx, dz) >= 3.0 && direction &&
					dx * direction->first + dz * direction->second > 0.0) {
				k = candidate;
				break;
			}
		}
		const auto direction = along(k);
		if (!direction)
			return false;
		meet = to[k];
		out = *direction;
	}
	const double reach =
			std::clamp(std::hypot(meet.x - a.x, meet.z - a.z) * 2.0 / 3.0, 1.0, 5.0);
	const auto raw_cap = speed_at(std::max(70.0 - 0.6 * turn, U_TURN_KMH));
	const std::optional<std::int32_t> cap =
			raw_cap < segment.speed ? std::optional<std::int32_t>(raw_cap) : std::nullopt;
	auto points = curve({a.x, a.z},
			{a.x + heading->first * reach, a.z + heading->second * reach},
			{meet.x - out.first * reach, meet.z - out.second * reach}, {meet.x, meet.z});
	if (onto_meet)
		points.emplace_back(meet.x, meet.z, 1.0);
	double arc = segment.arcs.empty() ? 0.0 : segment.arcs.back();
	std::pair<double, double> previous{a.x, a.z};
	std::vector<std::pair<Waypoint, double>> added;
	for (const auto &[x, z, t] : points) {
		if (!world.contains(static_cast<std::int32_t>(std::floor(x)),
					static_cast<std::int32_t>(std::floor(z))))
			return false;
		arc += std::hypot(x - previous.first, z - previous.second);
		previous = {x, z};
		added.push_back({{std::round(x * 10.0) / 10.0, a.y + (meet.y - a.y) * t,
								 std::round(z * 10.0) / 10.0, std::nullopt, cap},
				arc});
	}
	auto &result = segments[i];
	for (const auto &[point, distance] : added) {
		result.points.push_back(point);
		result.arcs.push_back(distance);
	}
	return true;
}

void link(std::vector<Segment> &segments, const std::vector<Lane> &lanes,
		const ::XZBBox &world, const std::function<std::int32_t(double)> &speed_at)
{
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> starting;
	for (std::size_t i = 0; i < lanes.size() && i < segments.size(); ++i)
		if (usable(segments[i]) && !lanes[i].nodes.empty())
			starting[lanes[i].nodes.front()->id].push_back(i);
	std::vector<std::vector<std::size_t>> ways_on(lanes.size());
	for (std::size_t i = 0; i < lanes.size() && i < segments.size(); ++i) {
		const auto &lane = lanes[i];
		if (lane.nodes.empty())
			continue;
		const auto found = starting.find(lane.nodes.back()->id);
		if (found != starting.end())
			for (const auto next : found->second)
				if (!lane.reverse || next != *lane.reverse)
					ways_on[i].push_back(next);
		if (ways_on[i].empty() && segments[i].turns_back && lane.reverse &&
				*lane.reverse < segments.size() && usable(segments[*lane.reverse]))
			ways_on[i].push_back(*lane.reverse);
	}
	std::vector<bool> short_lane(segments.size());
	for (std::size_t i = 0; i < segments.size(); ++i) {
		const auto &arcs = segments[i].arcs;
		short_lane[i] =
				(arcs.empty() ? 0.0 : arcs.back()) - (arcs.empty() ? 0.0 : arcs.front()) <
				SHORT_LANE;
	}
	for (std::size_t i = 0; i < lanes.size() && i < segments.size(); ++i) {
		const auto &lane = lanes[i];
		std::vector<std::size_t> next;
		bool through_short = false;
		std::vector<std::pair<std::size_t, unsigned>> todo;
		for (const auto &j : ways_on[i])
			todo.emplace_back(j, 0);
		while (!todo.empty()) {
			const auto [j, depth] = todo.back();
			todo.pop_back();
			if (j >= segments.size())
				continue;
			if (short_lane[j] && depth < 3 && !ways_on[j].empty()) {
				through_short = true;
				for (const auto &k : ways_on[j])
					if (k != i && (!lane.reverse || k != *lane.reverse))
						todo.emplace_back(k, depth + 1);
			} else if (std::find(next.begin(), next.end(), j) == next.end()) {
				next.push_back(j);
			}
		}
		std::vector<std::pair<std::size_t, std::optional<std::pair<std::size_t, double>>>>
				entries;
		for (const auto &j : next)
			entries.emplace_back(j, entry(segments[i].points, segments[j].points));
		std::vector<std::pair<std::size_t, std::size_t>> forward;
		for (const auto &[j, e] : entries)
			if (e)
				forward.emplace_back(j, e->first);
		std::vector<std::pair<std::size_t, std::size_t>> turns;
		if (entries.size() == 1 &&
				(!entries[0].second || entries[0].second->second > SHARP_DEG)) {
			const auto j = entries[0].first;
			if (bend_into(segments, i, j, world, speed_at)) {
				const auto e = entry(segments[i].points, segments[j].points);
				turns.emplace_back(j, e ? e->first : 0);
			} else {
				turns = forward;
			}
		} else if (forward.empty()) {
			for (const auto &[j, e] : entries)
				turns.emplace_back(j, 0);
		} else {
			turns = forward;
		}
		const auto end =
				segments[i].points.empty() ? nullptr : &segments[i].points.back();
		for (const auto &[j, e] : turns) {
			const auto target = j < segments.size() && e < segments[j].points.size()
										? &segments[j].points[e]
										: nullptr;
			const bool crossing =
					(segments[i].into_junction || through_short) && end && target &&
					std::hypot(target->x - end->x, target->z - end->z) <= CROSSING_REACH;
			segments[i].next.push_back({j, e, crossing});
		}
	}
}

std::vector<Car> place_cars(const std::vector<Segment> &segments, double scale)
{
	if (!(scale > 0.0) || !std::isfinite(scale))
		return {};
	const auto length = [](const Segment &s) {
		return s.arcs.empty() ? 0.0 : s.arcs.back() - s.arcs.front();
	};
	double wanted = 0.0;
	for (const auto &segment : segments)
		if (segment.points.size() >= 2)
			wanted += length(segment) / scale * segment.density;
	const double thin = wanted > MAX_CARS ? static_cast<double>(MAX_CARS) / wanted : 1.0;
	std::vector<Car> cars;
	cars.reserve(std::min<std::size_t>(MAX_CARS, static_cast<std::size_t>(wanted + 1.0)));
	double owed = 0.5;
	for (std::size_t i = 0; i < segments.size(); ++i) {
		const auto &segment = segments[i];
		if (segment.points.size() < 2 || segment.next.empty())
			continue;
		const double len = length(segment);
		owed += len / scale * segment.density * thin;
		const auto count = static_cast<std::size_t>(std::floor(owed));
		owed -= count;
		for (std::size_t j = 0; j < count; ++j) {
			if (cars.size() >= MAX_CARS)
				return cars;
			const double distance = segment.arcs.front() + len * (j + 0.5) / count;
			const auto it =
					std::upper_bound(segment.arcs.begin(), segment.arcs.end(), distance);
			const auto target = std::clamp<std::size_t>(
					static_cast<std::size_t>(it - segment.arcs.begin()), 1,
					segment.points.size() - 1);
			const auto &a = segment.points[target - 1];
			const auto &b = segment.points[target];
			const double span = segment.arcs[target] - segment.arcs[target - 1];
			const double t =
					span > 0.0 ? std::clamp((distance - segment.arcs[target - 1]) / span,
										 0.0, 1.0)
							   : 0.0;
			const auto roll = hash(i, j);
			const auto kind_roll = roll % 100;
			const VehicleKind kind = kind_roll < 60	  ? VehicleKind::Sedan
									 : kind_roll < 86 ? VehicleKind::Hatchback
													  : VehicleKind::Van;
			const std::string_view colour = kind == VehicleKind::Van
													? pick(VAN_COLOURS, roll >> 16)
													: pick(CAR_COLOURS, roll >> 16);
			cars.push_back({i, target,
					{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t},
					kind, std::string(colour),
					static_cast<std::int32_t>(880 + ((roll >> 40) % 241))});
		}
	}
	return cars;
}
} // namespace

void CarNetwork::place_way(world_editor::WorldEditor &editor, std::uint64_t way) const
{
	for (const auto car_index : cars_for_way(way)) {
		if (car_index >= cars_.size())
			continue;
		const auto &car = cars_[car_index];
		if (car.segment >= segments_.size())
			continue;
		const auto &segment = segments_[car.segment];
		if (car.target >= segment.points.size())
			continue;
		const auto &target = segment.points[car.target];
		const double x = car.position[0], y = car.position[1], z = car.position[2];
		const float yaw = static_cast<float>(
				std::atan2(-(target.x - x), target.z - z) * 180.0 / std::numbers::pi);
		const int speed = segment.speed * car.pace / 1000;
		const nlohmann::json rotation = {yaw, 0.0f};
		nlohmann::json data = {{"s", static_cast<std::int32_t>(car.segment)},
				{"i", static_cast<std::int32_t>(car.target)}, {"x", target.x},
				{"y", target.y}, {"z", target.z}, {"v", speed},
				{"c", target.cap ? std::min(*target.cap, speed) : speed}, {"p", car.pace},
				{"w", 0}, {"q", 0}, {"b", 0}};
		if (target.stop)
			data["g"] = *target.stop;
		if (target.cap)
			data["m"] = *target.cap;
		const std::int64_t seed = static_cast<std::int64_t>(car_index) * 16;
		editor.add_entity_exact("minecraft:marker", x, y, z, seed,
				{{"data", std::move(data)}, {"Tags", {"arnis.car"}},
						{"Rotation", rotation}});
		const float scale = static_cast<float>(size_);
		const auto parts = body_parts(car.kind, car.colour);
		for (std::size_t part_index = 0; part_index < parts.size(); ++part_index) {
			const auto &part = parts[part_index];
			const auto scaled = [scale](const std::array<float, 3> &v) {
				return nlohmann::json::array({v[0] * scale, v[1] * scale, v[2] * scale});
			};
			nlohmann::json tags = {"arnis.car.part"};
			if (part.light == Light::Tail)
				tags.push_back("arnis.car.tail");
			nlohmann::json extra = {{"block_state", {{"Name", part.block}}},
					{"transformation",
							{{"left_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
									{"right_rotation", {0.0f, 0.0f, 0.0f, 1.0f}},
									{"translation", scaled(part.corner)},
									{"scale", scaled(part.size)}}},
					{"Tags", std::move(tags)}, {"teleport_duration", 2},
					{"Rotation", rotation}};
			if (part.light == Light::Head)
				extra["brightness"] = {{"sky", 15}, {"block", 15}};
			else if (part.light == Light::Tail)
				extra["brightness"] = {{"sky", 7}, {"block", 7}};
			editor.add_block_display(x, y, z,
					seed + 1 + static_cast<std::int64_t>(part_index), std::move(extra));
		}
	}
}

} // namespace arnis::highways::cars
