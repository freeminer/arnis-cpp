#include "aircraft.h"

#include "../clipping.h"
#include "plane_bodies.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <span>
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
constexpr double STEPS_PER_S = 10.0;
constexpr double MAX_STEP = 7.9;
constexpr double CREEP = 0.03;
constexpr double TAXI_MS = 11.0;
constexpr double STAND_MS = 4.0;
constexpr double PIVOT_MS = 2.5;
constexpr double TURN_PULL = 1.1;
constexpr double TAXI_ACCEL = 0.9;
constexpr double TAXI_BRAKE = 1.1;
constexpr double CORNER_M = 32.0;
constexpr double PIVOT_M = 7.0;
constexpr double PIVOT_MERGE_M = 45.0;
constexpr double PUSH_MS = 1.6;
constexpr double ROLL_OUT_MS = 20.0;
constexpr double HOLD_SHORT_M = 60.0;
constexpr double TOUCHDOWN_M = 300.0;
constexpr double TOUCHDOWN_SHARE = 0.14;
constexpr double LIFT_OFF_MARGIN_M = 120.0;
constexpr double GLIDE_DEG = 3.0;
constexpr double DESCENT_DEG = 6.0;
constexpr double INTERCEPT_M = 35.0;
constexpr double FLARE_M = 60.0;
constexpr double NOSE_DOWN_M = 140.0;
constexpr double TOUCHDOWN_NOSE = 5.0;
constexpr double LEVEL_NOSE = 2.0;
constexpr double GEAR_DOWN_M = 1300.0;
constexpr double GEAR_UP_M = 12.0;
constexpr double START_UP_S = 10.0;
constexpr double LINE_UP_S = 3.0;
constexpr std::size_t MAX_HOLD_S = 160;
constexpr std::size_t HOLD_STEP_S = 4;
constexpr std::size_t START_STEP_S = 3;
constexpr double CLEAR_M = 4.5;
constexpr double CLEAR_HEIGHT_M = 14.0;
constexpr double FINAL_S = 25.0;
constexpr std::int64_t RUNWAY_PAD_S = 6;
constexpr double RUNWAY_SHARE = 0.9;
constexpr double CLIMB_OUT_M = 900.0;
constexpr double RUNWAY_TAKEN_M = 2000.0;
constexpr std::size_t ORBITS = 2;
constexpr std::int64_t EVEN_S = 60;
constexpr double CLOCK_STEP = 1.03;
constexpr double CLEAR_RUNWAY_M = 2.0;
constexpr double CLEAR_TERRAIN_M = 20.0;
constexpr double ALTERNATE_LINES_M = 28.0;
constexpr double PARKED_LINE_SHARE = 0.75;
constexpr double NOSE_GAP_M = 1.5;
constexpr double TAIL_ROOM_M = 8.0;
constexpr std::size_t MAX_FLYING = 36;
constexpr std::size_t MAX_FLYING_PER_FIELD = 14;
constexpr std::size_t MAX_PARKED = 80;
constexpr std::size_t MAX_AIR_POINTS = 110000;
constexpr double STAND_OCCUPANCY = 0.6;
constexpr double SLOT_S = 150.0;
constexpr double MIN_ROUND_S = 600.0;
constexpr double MAX_ROUND_S = 3000.0;
constexpr double MIN_DWELL_S = 90.0;
constexpr double MIN_CIRCUIT_M = 25.0;
constexpr double TALL_M = 45.0;
constexpr double PI = 3.14159265358979323846;

struct Circuit
{
	double height{};
	double radius{};
	double beyond{};
	double before{};
};
constexpr Circuit WIDE_CIRCUIT{130.0, 450.0, 500.0, 700.0};
constexpr Circuit TIGHT_CIRCUIT{60.0, 160.0, 220.0, 320.0};

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
Point left(Point a)
{
	return {-a.second, a.first};
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

namespace
{

double yaw_of(Point direction)
{
	return std::atan2(-direction.first, direction.second) * 180.0 / PI;
}

struct Pt
{
	double x{}, y{}, z{};
	double yaw{}, climb{}, nose{};
	bool gear{};
};

struct Piece
{
	std::vector<Pt> pts;
	double len{};

	Pt at(double distance) const
	{
		if (pts.empty())
			return {};
		const double nonnegative = std::max(0.0, distance);
		const auto i = std::min<std::size_t>(
				static_cast<std::size_t>(nonnegative), pts.size() - 1);
		const double fraction = nonnegative - static_cast<double>(i);
		const Pt &point = pts[i];
		const double yaw = point.yaw * PI / 180.0;
		const double climb = point.climb * PI / 180.0;
		return {point.x - std::sin(yaw) * std::cos(climb) * fraction,
				point.y + std::sin(climb) * fraction,
				point.z + std::cos(yaw) * std::cos(climb) * fraction, point.yaw,
				point.climb, point.nose, point.gear};
	}
};

struct Leg
{
	std::size_t piece{};
	double from{}, to{}, cap{}, accel{}, brake{};
	std::optional<std::size_t> holds;
	double length() const { return std::abs(to - from); }
};

struct Taxi
{
	std::vector<Leg> legs;
	std::vector<std::pair<double, double>> turns;
	double length() const
	{
		double result = 0.0;
		for (const auto &leg : legs)
			result += leg.length();
		return result;
	}
};

struct Fine
{
	std::vector<std::array<double, 3>> at;
	std::vector<double> nose;
	std::vector<bool> gear;

	void push(std::array<double, 3> point, double attitude, bool wheels)
	{
		at.push_back(point);
		nose.push_back(attitude);
		gear.push_back(wheels);
	}

	void ease_nose(double reach, double step)
	{
		const auto width =
				static_cast<std::size_t>(std::max(0.0, std::round(reach / step)));
		std::vector<double> sums(nose.size() + 1, 0.0);
		for (std::size_t i = 0; i < nose.size(); ++i)
			sums[i + 1] = sums[i] + nose[i];
		for (std::size_t i = 0; i < nose.size(); ++i) {
			const auto lo = i > width ? i - width : 0;
			const auto hi = std::min(nose.size(), i + width + 1);
			nose[i] = (sums[hi] - sums[lo]) / static_cast<double>(hi - lo);
		}
	}

	Piece piece() const
	{
		Piece result;
		double wanted = 0.0, done = 0.0;
		for (std::size_t i = 0; i + 1 < at.size(); ++i) {
			const auto &a = at[i];
			const auto &b = at[i + 1];
			const double dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
			const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
			while (wanted <= done + length && length > 0.0) {
				const double t = (wanted - done) / length;
				result.pts.push_back({a[0] + dx * t, a[1] + dy * t, a[2] + dz * t, 0.0,
						0.0, nose[i] + (nose[i + 1] - nose[i]) * t, gear[i]});
				wanted += 1.0;
			}
			done += length;
		}
		if (at.empty())
			return result;
		const auto &last = at.back();
		for (std::size_t i = 0; i < result.pts.size(); ++i) {
			const Pt p = result.pts[i];
			const Pt q = i + 1 < result.pts.size() ? result.pts[i + 1]
												   : Pt{last[0], last[1], last[2], p.yaw,
															 p.climb, p.nose, p.gear};
			const double flat = std::hypot(q.x - p.x, q.z - p.z);
			if (flat > 1e-6 || std::abs(q.y - p.y) > 1e-6) {
				result.pts[i].yaw = yaw_of({q.x - p.x, q.z - p.z});
				result.pts[i].climb = std::atan2(q.y - p.y, flat) * 180.0 / PI;
			} else if (i > 0) {
				result.pts[i].yaw = result.pts[i - 1].yaw;
				result.pts[i].climb = result.pts[i - 1].climb;
			}
		}
		result.len = done;
		return result;
	}
};

struct Line
{
	std::vector<Point> points;
	std::vector<double> cumulative;
	double length() const { return cumulative.empty() ? 0.0 : cumulative.back(); }
	Point at(double distance) const
	{
		if (points.size() < 2)
			return points.empty() ? Point{} : points.front();
		if (distance <= 0.0)
			return add(points[0], mul(unit(sub(points[1], points[0])), distance));
		if (distance >= length())
			return add(points.back(),
					mul(unit(sub(points.back(), points[points.size() - 2])),
							distance - length()));
		const auto upper =
				std::upper_bound(cumulative.begin(), cumulative.end(), distance);
		const auto i = std::clamp<std::size_t>(
				static_cast<std::size_t>(upper - cumulative.begin()), 1,
				points.size() - 1);
		const double t =
				(distance - cumulative[i - 1]) / (cumulative[i] - cumulative[i - 1]);
		return add(points[i - 1], mul(sub(points[i], points[i - 1]), t));
	}
	Point first() const
	{
		return points.size() < 2 ? Point{0, 1} : unit(sub(points[1], points[0]));
	}
	Point last() const
	{
		return points.size() < 2 ? Point{0, 1}
								 : unit(sub(points.back(), points[points.size() - 2]));
	}
};

struct Runway
{
	std::size_t strip{};
	bool back{};
	std::vector<std::size_t> nodes;
	std::vector<double> at;
	double length() const { return at.empty() ? 0.0 : at.back(); }
	static Runway of(const Field &field, std::size_t strip, bool back)
	{
		const auto &source = field.strips.at(strip);
		Runway result{strip, back, source.nodes, source.at};
		if (back) {
			std::reverse(result.nodes.begin(), result.nodes.end());
			std::reverse(result.at.begin(), result.at.end());
			for (auto &distance : result.at)
				distance = source.length() - distance;
		}
		return result;
	}
	Line line(const Field &field) const
	{
		Line result;
		result.cumulative = at;
		for (const auto node : nodes)
			result.points.push_back(field.nodes[node].at);
		return result;
	}
	std::size_t node_before(double distance) const
	{
		const auto upper = std::upper_bound(at.begin(), at.end(), distance);
		return std::min<std::size_t>(
				upper == at.begin() ? 0
									: static_cast<std::size_t>(upper - at.begin() - 1),
				nodes.size() > 1 ? nodes.size() - 2 : 0);
	}
	std::set<std::pair<std::size_t, std::size_t>> stretches() const
	{
		std::set<std::pair<std::size_t, std::size_t>> result;
		for (std::size_t i = 1; i < nodes.size(); ++i)
			result.emplace(nodes[i - 1], nodes[i]);
		return result;
	}
};

double approach_height(double d, double top, double k)
{
	const double glide = std::tan(GLIDE_DEG * PI / 180.0);
	const double flare = FLARE_M * k;
	if (d <= 0.0)
		return 0.0;
	if (d < flare)
		return glide * d * d / (2.0 * flare);
	const double on_glide = glide * (d - flare / 2.0);
	const double meet = INTERCEPT_M * k;
	if (on_glide <= meet)
		return std::min(on_glide, top);
	const double d_meet = meet / glide + flare / 2.0;
	return std::min(meet + std::tan(DESCENT_DEG * PI / 180.0) * (d - d_meet), top);
}

std::pair<std::vector<Leg>, std::vector<Leg>> cut(
		const std::vector<Leg> &legs, double distance)
{
	std::pair<std::vector<Leg>, std::vector<Leg>> result;
	double done = 0.0;
	for (const auto &leg : legs) {
		const double length = leg.length();
		if (done + length <= distance) {
			result.first.push_back(leg);
		} else if (done >= distance) {
			result.second.push_back(leg);
		} else {
			const double middle =
					leg.from + std::copysign(distance - done, leg.to - leg.from);
			auto head = leg;
			auto tail = leg;
			head.to = middle;
			tail.from = middle;
			result.first.push_back(head);
			result.second.push_back(tail);
		}
		done += length;
	}
	return result;
}

std::vector<Leg> reversed(const std::vector<Leg> &legs, double cap)
{
	std::vector<Leg> result;
	result.reserve(legs.size());
	for (auto it = legs.rbegin(); it != legs.rend(); ++it) {
		auto leg = *it;
		std::swap(leg.from, leg.to);
		leg.cap = cap;
		result.push_back(leg);
	}
	return result;
}

std::vector<std::pair<std::size_t, double>> run(const std::vector<Leg> &source)
{
	std::vector<std::pair<std::size_t, const Leg *>> legs;
	for (std::size_t i = 0; i < source.size(); ++i)
		if (source[i].length() > 1e-6)
			legs.emplace_back(i, &source[i]);
	if (legs.empty())
		return {};
	std::vector<double> exit(legs.size());
	for (std::size_t j = legs.size() - 1; j-- > 0;) {
		const auto &after = *legs[j + 1].second;
		exit[j] = std::min({after.cap,
				std::sqrt(exit[j + 1] * exit[j + 1] + 2.0 * after.brake * after.length()),
				legs[j].second->cap});
	}
	std::vector<std::pair<std::size_t, double>> out;
	std::size_t j = 0;
	double distance = 0.0, velocity = 0.0;
	for (;;) {
		const auto &leg = *legs[j].second;
		const double limit =
				std::min(leg.cap, std::sqrt(exit[j] * exit[j] +
											2.0 * leg.brake * (leg.length() - distance)));
		velocity = std::clamp(std::min(velocity + leg.accel, limit), CREEP, MAX_STEP);
		distance += velocity;
		while (distance >= legs[j].second->length()) {
			if (j + 1 == legs.size()) {
				out.emplace_back(legs[j].first, legs[j].second->to);
				return out;
			}
			distance -= legs[j].second->length();
			++j;
		}
		const auto &current = *legs[j].second;
		out.emplace_back(legs[j].first,
				current.from + std::copysign(distance, current.to - current.from));
	}
}

enum class PieceKeyKind : std::uint8_t
{
	Edge,
	Corner,
	Pivot,
	Circuit,
	Climb,
	Orbit
};
struct PieceKey
{
	PieceKeyKind kind{};
	std::array<std::size_t, 3> nodes{};
	std::int64_t parameter{};
	bool first{}, second{};
	plane_bodies::Airframe frame{plane_bodies::Airframe::Light};
	bool operator<(const PieceKey &other) const
	{
		return std::tie(kind, nodes, parameter, first, second, frame) <
			   std::tie(other.kind, other.nodes, other.parameter, other.first,
					   other.second, other.frame);
	}
};

struct Use;

class Paths
{
public:
	using Ground = std::function<std::optional<double>(int, int)>;
	Paths(const Field &field, double scale, Ground ground, double datum, double ceiling,
			const std::vector<std::pair<Point, double>> &tall) :
			field(field), k(scale), ground(std::move(ground)), datum(datum),
			ceiling(ceiling), tall(tall)
	{
	}

	double surface(double x, double z) const
	{
		double sum = 0.0;
		int count = 0;
		for (int dx = -2; dx <= 2; ++dx)
			for (int dz = -2; dz <= 2; ++dz)
				if (const auto y = ground(static_cast<int>(std::floor(x + dx * 3.0)),
							static_cast<int>(std::floor(z + dz * 3.0)))) {
					sum += *y;
					++count;
				}
		return count ? sum / count + 1.0 : datum + 1.0;
	}

	double speed(double metres_per_second) const
	{
		return std::min(metres_per_second * k / STEPS_PER_S, MAX_STEP);
	}

	std::size_t edge_piece(std::size_t a, std::size_t b)
	{
		PieceKey key{PieceKeyKind::Edge, {a, b, 0}};
		return piece(key, [&](const Paths &me) {
			const Point from = field.nodes[a].at, direction = field.dir(a, b);
			const double length = field.span(a, b), yaw = yaw_of(direction);
			Piece result;
			for (std::size_t i = 0; i <= static_cast<std::size_t>(length); ++i) {
				const auto at = add(from, mul(direction, static_cast<double>(i)));
				result.pts.push_back({at.first, me.surface(at.first, at.second),
						at.second, yaw, 0.0, 0.0, true});
			}
			result.len = length;
			return result;
		});
	}

	Piece ground_piece(const std::vector<Point> &line) const
	{
		Fine fine;
		for (const auto &p : line)
			fine.push({p.first, surface(p.first, p.second), p.second}, 0.0, true);
		auto result = fine.piece();
		for (auto &p : result.pts)
			p.climb = 0.0;
		result.len = 0.0;
		for (std::size_t i = 1; i < line.size(); ++i)
			result.len += norm(sub(line[i], line[i - 1]));
		if (result.len < result.pts.size())
			result.pts.resize(static_cast<std::size_t>(result.len) + 1);
		return result;
	}

	std::size_t corner(std::size_t a, std::size_t b, std::size_t c, double trim)
	{
		PieceKey key{PieceKeyKind::Corner, {a, b, c},
				static_cast<std::int64_t>(std::llround(trim * 100.0))};
		return piece(key, [&](const Paths &me) {
			const auto d1 = field.dir(a, b), d2 = field.dir(b, c);
			const double theta = turn_deg(d1, d2) * PI / 180.0;
			const double radius = trim / std::tan(theta / 2.0);
			const double sign = cross(d1, d2) > 0.0	  ? 1.0
								: cross(d1, d2) < 0.0 ? -1.0
													  : 0.0;
			const Point side = mul(Point{-d1.second, d1.first}, sign);
			const Point start = sub(field.nodes[b].at, mul(d1, trim));
			const Point centre = add(start, mul(side, radius));
			const auto n = static_cast<std::size_t>(
					std::max(2.0, std::ceil(radius * theta / 0.2)));
			std::vector<Point> line;
			for (std::size_t i = 0; i <= n; ++i) {
				const double t = theta * i / n;
				line.push_back(add(sub(centre, mul(side, radius * std::cos(t))),
						mul(d1, radius * std::sin(t))));
			}
			return me.ground_piece(line);
		});
	}

	std::size_t pivot(
			std::size_t a, std::size_t b, std::size_t c, double lead, double out)
	{
		PieceKey key{PieceKeyKind::Pivot, {a, b, c},
				static_cast<std::int64_t>(std::llround(lead * 100.0))};
		return piece(key, [&](const Paths &me) {
			const auto d1 = field.dir(a, b), d2 = field.dir(b, c);
			const double theta = turn_deg(d1, d2) * PI / 180.0;
			const double determinant = cross(d1, d2);
			const double sign =
					std::abs(determinant) < 1e-9 ? 1.0 : std::copysign(1.0, determinant);
			const Point side = mul(Point{-d1.second, d1.first}, sign);
			const Point at_b = field.nodes[b].at;
			const double radius = PIVOT_M * k;
			const Point start = sub(at_b, mul(d1, lead));
			const Point centre = add(start, mul(side, radius));
			const auto n = static_cast<std::size_t>(
					std::max(2.0, std::ceil(radius * theta / 0.2)));
			std::vector<Point> line;
			for (std::size_t i = 0; i <= n; ++i) {
				const double t = theta * i / n;
				line.push_back(add(sub(centre, mul(side, radius * std::cos(t))),
						mul(d1, radius * std::sin(t))));
			}
			const auto end = line.back();
			const double along = dot(sub(end, at_b), d2);
			const double aside = dot(sub(end, at_b), Point{-d2.second, d2.first});
			const double merge = std::max(out - along, 1.0);
			const auto m = static_cast<std::size_t>(std::ceil(merge / 0.2));
			for (std::size_t i = 1; i <= m; ++i) {
				const double t = static_cast<double>(i) / m;
				const double ease = t * t * (3.0 - 2.0 * t);
				line.push_back(add(add(at_b, mul(d2, along + merge * t)),
						mul(Point{-d2.second, d2.first}, aside * (1.0 - ease))));
			}
			return me.ground_piece(line);
		});
	}

	Taxi taxi(std::span<const std::size_t> nodes, double start)
	{
		Taxi result;
		if (nodes.size() < 2)
			return result;
		const auto speed = [&](double ms) {
			return std::min(ms * k / STEPS_PER_S, MAX_STEP);
		};
		std::vector<double> lengths;
		for (std::size_t i = 1; i < nodes.size(); ++i)
			lengths.push_back(field.span(nodes[i - 1], nodes[i]));
		struct Join
		{
			std::size_t piece;
			double in, out, cap;
		};
		std::vector<std::optional<Join>> joins(nodes.size());
		for (std::size_t i = 1; i + 1 < nodes.size(); ++i) {
			const auto a = nodes[i - 1], b = nodes[i], c = nodes[i + 1];
			const double turn = turn_deg(field.dir(a, b), field.dir(b, c));
			const double room_in = i == 1 ? std::min(lengths[0] - start, lengths[0] / 2.0)
										  : lengths[i - 1] / 2.0;
			const double room_out = lengths[i] / 2.0;
			if (turn < 2.0)
				continue;
			if (turn <= 150.0) {
				const double half = std::tan(turn * PI / 360.0);
				const double radius =
						std::min(CORNER_M * k, std::min(room_in, room_out) / half);
				const double trim = radius * half;
				if (trim < 0.3)
					continue;
				const double pull = TURN_PULL * k / (STEPS_PER_S * STEPS_PER_S);
				joins[i] = Join{corner(a, b, c, trim), trim, trim,
						std::min(std::sqrt(pull * radius), speed(TAXI_MS))};
			} else {
				const double lead = std::min(
						PIVOT_M * k + 4.0 * k, lengths[i - 1] - (i == 1 ? start : 0.0));
				const double out = std::min(lead + PIVOT_MERGE_M * k, lengths[i] - 1.0);
				if (lead < 1.0 || out < lead + 2.0)
					continue;
				joins[i] = Join{pivot(a, b, c, lead, out), lead, out, speed(PIVOT_MS)};
			}
		}
		const double per_step2 = k / (STEPS_PER_S * STEPS_PER_S);
		const double accel = TAXI_ACCEL * per_step2, brake = TAXI_BRAKE * per_step2;
		result.turns.resize(nodes.size());
		double done = 0.0, enter = start;
		for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
			const auto a = nodes[i], b = nodes[i + 1];
			const double leave = lengths[i] - (joins[i + 1] ? joins[i + 1]->in : 0.0);
			double cap = speed(TAXI_MS);
			if (field.kind(a, b) == Kind::Stand)
				cap = speed(STAND_MS);
			else if (field.kind(a, b) == Kind::Runway)
				cap = speed(TAXI_MS * 1.6);
			if (leave > enter) {
				result.legs.push_back({edge_piece(a, b), enter, leave, cap, accel, brake,
						std::nullopt});
				done += leave - enter;
			}
			result.turns[i + 1].first = done;
			enter = 0.0;
			if (joins[i + 1]) {
				const auto &join = *joins[i + 1];
				const double length = pieces[join.piece].len;
				result.legs.push_back(
						{join.piece, 0.0, length, join.cap, accel, brake, std::nullopt});
				done += length;
				enter = join.out;
			}
			result.turns[i + 1].second = done;
		}
		return result;
	}

	double top(double height, double touch_y) const
	{
		return std::min(height * k, ceiling - touch_y);
	}
	double top(Circuit shape, double touch_y) const { return top(shape.height, touch_y); }
	bool clear(std::array<double, 3> at, double gap) const
	{
		if (const auto y = ground(static_cast<int>(std::floor(at[0])),
					static_cast<int>(std::floor(at[2])));
				y && at[1] < *y + 1.0 + gap)
			return false;
		for (const auto &[position, height] : tall)
			if (at[1] < height + gap && norm(sub(position, {at[0], at[2]})) < 40.0 * k)
				return false;
		return true;
	}

	std::optional<Fine> circuit(const Line &line, Circuit shape, double beyond,
			double touch, double side) const
	{
		const double radius = shape.radius * k;
		const double before = shape.before * k;
		std::vector<Point> flat;
		constexpr double step = 0.25;
		auto arc = [&](Point from, Point ahead) {
			const auto to_side = mul(left(ahead), side);
			const auto centre = add(from, mul(to_side, radius));
			const auto n = static_cast<std::size_t>(std::ceil(PI * radius / step));
			for (std::size_t i = 1; i <= n; ++i) {
				const double t = PI * i / n;
				flat.push_back(add(sub(centre, mul(to_side, radius * std::cos(t))),
						mul(ahead, radius * std::sin(t))));
			}
		};
		const auto far = line.at(line.length() + beyond);
		flat.push_back(far);
		arc(far, line.last());
		const auto downwind_from = flat.back();
		const auto near = line.at(-before);
		const auto downwind_to = add(near, mul(left(line.first()), side * 2.0 * radius));
		const double crosswind = norm(sub(downwind_to, downwind_from));
		const auto n =
				static_cast<std::size_t>(std::max(1.0, std::ceil(crosswind / step)));
		for (std::size_t i = 1; i <= n; ++i)
			flat.push_back(add(
					downwind_from, mul(sub(downwind_to, downwind_from), double(i) / n)));
		arc(downwind_to, mul(line.first(), -1.0));
		const double end = touch + NOSE_DOWN_M * k;
		const auto final_n = static_cast<std::size_t>(std::ceil((before + end) / step));
		const auto final_from = flat.size();
		for (std::size_t i = 1; i <= final_n; ++i)
			flat.push_back(line.at(-before + (before + end) * double(i) / final_n));
		const auto touch_i = final_from +
							 static_cast<std::size_t>(std::llround(
									 (before + touch) / (before + end) * final_n)) -
							 1;
		std::vector<double> to_go(flat.size());
		for (std::size_t i = touch_i; i-- > 0;)
			to_go[i] = to_go[i + 1] + norm(sub(flat[i + 1], flat[i]));
		const auto touch_at = flat[touch_i];
		const double touch_y = surface(touch_at.first, touch_at.second);
		const double height = top(shape, touch_y);
		const auto threshold = line.at(0.0);
		const double threshold_y = surface(threshold.first, threshold.second);
		const auto over_runway = final_from + static_cast<std::size_t>(std::llround(
													  before / (before + end) * final_n));
		Fine fine;
		const double glide_until =
				INTERCEPT_M * k / std::tan(GLIDE_DEG * PI / 180.0) + FLARE_M * k / 2.0;
		double past = 0.0;
		for (std::size_t i = 0; i < flat.size(); ++i) {
			const auto p = flat[i];
			if (i > touch_i) {
				past += norm(sub(flat[i], flat[i - 1]));
				const double nose = TOUCHDOWN_NOSE * (1.0 - past / (NOSE_DOWN_M * k));
				fine.push({p.first, surface(p.first, p.second), p.second},
						std::max(0.0, nose), true);
				continue;
			}
			const double d = to_go[i];
			const double under =
					i >= over_runway ? surface(p.first, p.second) : threshold_y;
			const double y = std::min(
					under + approach_height(d, std::numeric_limits<double>::max(), k),
					touch_y + height);
			const double h = y - under;
			double nose;
			if (y >= touch_y + height - 1e-6)
				nose = LEVEL_NOSE;
			else if (d > glide_until)
				nose = -1.0;
			else if (d > FLARE_M * k * 2.0)
				nose = LEVEL_NOSE;
			else
				nose = LEVEL_NOSE +
					   (TOUCHDOWN_NOSE - LEVEL_NOSE) * (1.0 - d / (FLARE_M * k * 2.0));
			const double gap = std::max(std::min(CLEAR_TERRAIN_M * k, h * 0.6),
					std::min(CLEAR_RUNWAY_M * k, h));
			if (i % 8 == 0 && i < over_runway && !clear({p.first, y, p.second}, gap))
				return std::nullopt;
			fine.push({p.first, y, p.second}, nose, d <= GEAR_DOWN_M * k);
		}
		fine.ease_nose(40.0 * k, step);
		return fine;
	}

	std::optional<Fine> climb(const Line &line, plane_bodies::Airframe frame,
			double rotate, double lift, double beyond, double height,
			double touch_y) const
	{
		const auto angles = plane_bodies::angles_degrees(frame);
		const double slope = std::tan(angles[0] * PI / 180.0);
		const auto lift_at = line.at(lift);
		const double lift_y = surface(lift_at.first, lift_at.second);
		const double rise = touch_y + height - lift_y;
		const double ease_in = 110.0 * k;
		const double ease_out = std::min(220.0 * k, rise / slope);
		const double run_up = rise / slope + ease_in / 2.0 + ease_out / 2.0;
		const double end = line.length() + beyond;
		if (rise <= 0.0 || lift + run_up > end)
			return std::nullopt;
		auto climb_height = [&](double x) {
			if (x <= 0.0)
				return 0.0;
			if (x < ease_in)
				return slope * x * x / (2.0 * ease_in);
			if (x < run_up - ease_out)
				return slope * (x - ease_in / 2.0);
			if (x < run_up)
				return rise - slope * (run_up - x) * (run_up - x) / (2.0 * ease_out);
			return rise;
		};
		constexpr double step = 0.25;
		const auto n = static_cast<std::size_t>(std::ceil((end - rotate) / step));
		if (!n)
			return std::nullopt;
		Fine fine;
		for (std::size_t i = 0; i <= n; ++i) {
			const double s = rotate + (end - rotate) * double(i) / n;
			const auto p = line.at(s);
			if (s <= lift) {
				const double nose =
						angles[1] * 0.55 *
						std::clamp((s - rotate) / std::max(lift - rotate, 1.0), 0.0, 1.0);
				fine.push({p.first, surface(p.first, p.second), p.second}, nose, true);
				continue;
			}
			const double x = s - lift;
			const double h = climb_height(x);
			const double nose = x < ease_in ? angles[1] * (0.55 + 0.45 * x / ease_in)
								: x < run_up - ease_out ? angles[1]
								: x < run_up
										? LEVEL_NOSE + (angles[1] - LEVEL_NOSE) *
															   (run_up - x) / ease_out
										: LEVEL_NOSE;
			const double y = lift_y + h;
			const double gap = std::min(CLEAR_TERRAIN_M * k, h * 0.5);
			if (i % 8 == 0 && s > line.length() && !clear({p.first, y, p.second}, gap))
				return std::nullopt;
			fine.push({p.first, y, p.second}, nose, h < GEAR_UP_M * k);
		}
		const double first = fine.nose.front();
		fine.ease_nose(25.0 * k, (end - rotate) / n);
		const auto lead = static_cast<std::size_t>(25.0 * k / ((end - rotate) / n));
		for (std::size_t i = 0; i < std::min(lead, fine.nose.size()); ++i) {
			const double t = lead ? double(i) / lead : 0.0;
			fine.nose[i] = first + (fine.nose[i] - first) * t;
		}
		return fine;
	}

	std::optional<std::size_t> circuit_piece(const Use &use, bool tight);
	std::optional<std::pair<std::size_t, double>> orbit_piece(
			const Use &use, bool tight, std::size_t circuit);
	std::optional<std::size_t> climb_piece(
			const Use &use, plane_bodies::Airframe frame, double departure);

	const Field &field;
	double k;
	Ground ground;
	double datum, ceiling;
	const std::vector<std::pair<Point, double>> &tall;
	std::vector<Piece> pieces;
	std::vector<PieceKey> keys;

private:
	std::map<PieceKey, std::size_t> by_key;
	template <typename Make>
	std::size_t piece(const PieceKey &key, Make make)
	{
		if (const auto found = by_key.find(key); found != by_key.end())
			return found->second;
		pieces.push_back(make(*this));
		keys.push_back(key);
		const auto index = pieces.size() - 1;
		by_key.emplace(key, index);
		return index;
	}
};

struct Use
{
	Runway runway;
	Line line;
	double side{};
	double touch{};
	double touch_y{};
	std::array<double, 2> beyond{};
	std::vector<plane_bodies::Airframe> frames;
};

std::vector<std::pair<Point, double>> tall_buildings(
		const std::vector<ProcessedElement> &elements, double k,
		const Paths::Ground &ground)
{
	std::vector<std::pair<Point, double>> out;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.tags.get("building").empty() && way.tags.get("building:part").empty())
			continue;
		const auto parse_height = [](std::string value) -> std::optional<double> {
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.front())))
				value.erase(value.begin());
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.back())))
				value.pop_back();
			if (!value.empty() && value.back() == 'm')
				value.pop_back();
			try {
				std::size_t used = 0;
				const double parsed = std::stod(value, &used);
				if (used != value.size() || !std::isfinite(parsed))
					return std::nullopt;
				return parsed;
			} catch (...) {
				return std::nullopt;
			}
		};
		auto height = parse_height(way.tags.get("height"));
		if (!height) {
			if (const auto levels = parse_height(way.tags.get("building:levels")))
				height = *levels * 3.2;
		}
		if (!height || *height < TALL_M || way.nodes.empty())
			continue;
		Point at{};
		for (const auto &node : way.nodes) {
			at.first += node.x;
			at.second += node.z;
		}
		at = mul(at, 1.0 / way.nodes.size());
		const auto base = ground(static_cast<int>(std::floor(at.first)),
				static_cast<int>(std::floor(at.second)))
								  .value_or(0.0);
		out.emplace_back(at, base + *height * k);
	}
	return out;
}

double climb_run(plane_bodies::Airframe frame, double rise, double k)
{
	const double slope = std::tan(plane_bodies::angles_degrees(frame)[0] * PI / 180.0);
	return rise / slope + 55.0 * k + std::min(220.0 * k, rise / slope) / 2.0;
}

std::vector<Use> runway_uses(const Paths &paths, const std::vector<std::size_t> &strips,
		std::optional<Point> centre)
{
	const auto &field = paths.field;
	std::vector<Use> uses;
	for (const auto strip_index : strips) {
		const auto &strip = field.strips[strip_index];
		const double metres = strip.length() / paths.k;
		std::vector<plane_bodies::Airframe> frames;
		for (const auto frame : plane_bodies::ALL) {
			const auto [needed, paved] = plane_bodies::runway_m(frame);
			if (metres >= needed && (strip.paved || !paved))
				frames.push_back(frame);
		}
		if (frames.empty() || strip.nodes.size() < 2)
			continue;
		const double touch =
				std::min(TOUCHDOWN_M * paths.k, TOUCHDOWN_SHARE * strip.length());
		const auto ahead = field.dir(strip.nodes.front(), strip.nodes.back());
		const bool west_back =
				ahead.first > 0.0 || (ahead.first == 0.0 && ahead.second > 0.0);
		for (const bool back : {west_back, !west_back}) {
			const auto runway = Runway::of(field, strip_index, back);
			const auto line = runway.line(field);
			const double away =
					centre ? (dot(left(line.first()),
									  sub(*centre, line.at(line.length() / 2.0))) > 0.0
											 ? -1.0
											 : 1.0)
						   : 1.0;
			for (const double side : {away, -away}) {
				const auto touchdown = line.at(touch);
				const double touch_y = paths.surface(touchdown.first, touchdown.second);
				const bool wide = std::any_of(frames.begin(), frames.end(),
						[](auto f) { return !plane_bodies::slow(f); });
				const bool tight = std::any_of(frames.begin(), frames.end(),
						[](auto f) { return plane_bodies::slow(f); });
				const auto has_room = [&](Circuit shape) {
					return paths.top(shape, touch_y) >= MIN_CIRCUIT_M * paths.k;
				};
				if ((wide && !has_room(WIDE_CIRCUIT)) ||
						(tight && !has_room(TIGHT_CIRCUIT)))
					continue;
				if ((wide && !paths.circuit(line, WIDE_CIRCUIT,
									 WIDE_CIRCUIT.beyond * paths.k, touch, side)) ||
						(tight && !paths.circuit(line, TIGHT_CIRCUIT,
										  TIGHT_CIRCUIT.beyond * paths.k, touch, side)))
					continue;
				const auto slowest = *std::min_element(
						frames.begin(), frames.end(), [](auto a, auto b) {
							return plane_bodies::angles_degrees(a)[0] <
								   plane_bodies::angles_degrees(b)[0];
						});
				const auto shape =
						plane_bodies::slow(slowest) ? TIGHT_CIRCUIT : WIDE_CIRCUIT;
				const double lift = line.length() / 2.0;
				const auto lift_at = line.at(lift);
				const double rise = touch_y + paths.top(shape, touch_y) -
									paths.surface(lift_at.first, lift_at.second);
				const double beyond = std::max(
						shape.beyond * paths.k, lift + climb_run(slowest, rise, paths.k) +
														40.0 * paths.k - line.length());
				if (!paths.climb(line, slowest, lift - 10.0, lift, beyond,
							paths.top(shape, touch_y), touch_y))
					continue;
				uses.push_back({runway, line, side, touch, touch_y,
						{WIDE_CIRCUIT.beyond * paths.k, TIGHT_CIRCUIT.beyond * paths.k},
						frames});
				break;
			}
			if (!uses.empty() && uses.back().runway.strip == strip_index)
				break;
		}
	}
	return uses;
}

struct Draft
{
	std::size_t stand{};
	plane_bodies::Airframe frame{};
	std::size_t used{};
	std::vector<Leg> push, out, line_up, roll;
	double departure{};
	std::vector<Leg> arrive;
	std::pair<std::size_t, double> park{};
	double taxied{};
};

struct Step
{
	std::size_t piece{};
	double at{};
	std::optional<std::size_t> holds;
};

struct Round
{
	std::size_t stand{};
	plane_bodies::Airframe frame{};
	std::size_t runway{};
	std::vector<Step> before;
	std::vector<std::vector<Step>> afters;
	Step park;
};

struct Pace
{
	double lift{}, land{}, air{}, accel{}, brake{};
	static Pace of(plane_bodies::Airframe frame, double k)
	{
		const auto [lift_ms, land_ms] = plane_bodies::speeds_mps(frame);
		const auto [accel_ms, brake_ms] = plane_bodies::accel_mps2(frame);
		const auto step_speed = [k](double speed) {
			return std::min(speed * k / STEPS_PER_S, MAX_STEP);
		};
		const double per_step2 = k / (STEPS_PER_S * STEPS_PER_S);
		return {step_speed(lift_ms), step_speed(land_ms), step_speed(land_ms * 1.3),
				accel_ms * per_step2, brake_ms * per_step2};
	}
	std::pair<double, double> take_off(double departure) const
	{
		const double roll = lift * lift / (2.0 * accel);
		const double turn = lift * 2.5 * STEPS_PER_S;
		const double rotate = departure + roll - 0.4 * turn;
		return {rotate, rotate + turn};
	}
};

double starts_at(const std::vector<Leg> &legs, double distance)
{
	double done = 0.0;
	for (const auto &leg : legs) {
		if (done >= distance - 1e-6)
			return leg.from;
		done += leg.length();
	}
	return 0.0;
}

std::optional<Draft> make_draft(Paths &paths, std::size_t stand_index,
		plane_bodies::Airframe frame, std::size_t use_index, const Use &use, double size)
{
	const auto &field = paths.field;
	const auto &stand = field.stands[stand_index];
	if (stand.path.size() < 2)
		return std::nullopt;
	const std::size_t junction = stand.path[0], into = stand.path[1];
	const Pace pace = Pace::of(frame, paths.k);
	const double nose = plane_bodies::reach_m(frame).first;
	const auto &runway = use.runway;
	const auto runway_edges = runway.stretches();
	const std::vector<std::pair<std::size_t, std::size_t>> free_edges(
			runway_edges.begin(), runway_edges.end());
	std::map<std::size_t, std::size_t> index;
	for (std::size_t i = 0; i < runway.nodes.size(); ++i)
		index.emplace(runway.nodes[i], i);
	const auto fits = [&](double departure) {
		return pace.take_off(departure).second + LIFT_OFF_MARGIN_M * paths.k <=
			   runway.length();
	};
	struct Candidate
	{
		double cost{};
		std::vector<std::size_t> path;
		std::size_t enters{};
	};
	std::optional<Candidate> best;
	for (const auto &[back, kind] : field.next[junction]) {
		if (kind != Kind::Taxi)
			continue;
		const auto goal = [&](std::size_t a, std::size_t b) -> std::optional<double> {
			const auto it = index.find(b);
			if (it == index.end() || it->second + 1 >= runway.nodes.size() ||
					!fits(runway.at[it->second] + CORNER_M * paths.k))
				return std::nullopt;
			const double turn =
					turn_deg(field.dir(a, b), field.dir(b, runway.nodes[it->second + 1]));
			return turn_cost(turn) * paths.k + 0.25 * runway.at[it->second];
		};
		auto route = field.taxi_route({back, junction}, free_edges, 3.0, paths.k, goal);
		if (!route || route->empty())
			continue;
		double cost = 0.0;
		for (std::size_t i = 1; i < route->size(); ++i)
			cost += field.span((*route)[i - 1], (*route)[i]);
		for (std::size_t i = 2; i < route->size(); ++i)
			cost += turn_cost(turn_deg(field.dir((*route)[i - 2], (*route)[i - 1]),
							field.dir((*route)[i - 1], (*route)[i]))) *
					paths.k;
		const auto enters = index.at(route->back());
		if (!best || cost < best->cost)
			best = Candidate{cost, std::move(*route), enters};
	}
	if (!best)
		return std::nullopt;
	const auto out_cost = best->cost;
	auto path = std::move(best->path);
	const auto enters = best->enters;
	const auto back = path.front();

	std::vector<std::size_t> stand_line{back, junction};
	stand_line.insert(stand_line.end(), stand.path.begin() + 1, stand.path.end());
	const auto probe = paths.taxi(stand_line, 0.0);
	const double behind = field.span(back, junction);
	if (probe.legs.empty())
		return std::nullopt;
	const auto edge_piece = paths.edge_piece(back, junction);
	const double corner = probe.legs.front().piece == edge_piece
								  ? behind - probe.legs.front().to
								  : behind;
	const double start =
			std::min(std::max(behind - corner - (nose * size + 4.0 * paths.k), 0.5),
					std::max(behind - corner, 0.0));
	const auto into_stand = paths.taxi(stand_line, start);
	const double stop_back = nose * size + NOSE_GAP_M * paths.k;
	const double park_at = into_stand.length() - stop_back;
	if (into_stand.turns.size() < 2 || park_at < into_stand.turns[1].second + 1.0)
		return std::nullopt;
	auto [to_park, unused_tail] = cut(into_stand.legs, park_at);
	(void)unused_tail;
	if (to_park.empty())
		return std::nullopt;
	const auto last = to_park.back();
	const std::pair<std::size_t, double> park{last.piece, last.to};
	auto push = reversed(to_park, paths.speed(PUSH_MS));

	const std::size_t at_b = path.size() - 1;
	path.insert(path.end(), runway.nodes.begin() + enters + 1, runway.nodes.end());
	const auto out = paths.taxi(path, start);
	if (out.turns.size() <= at_b)
		return std::nullopt;
	const double joined = runway.at[enters] + starts_at(out.legs, out.turns[at_b].second);
	const double departure = std::ceil(joined + 6.0 * paths.k);
	if (!fits(departure))
		return std::nullopt;
	const auto rotate = pace.take_off(departure).first;
	std::size_t first_on = at_b;
	for (std::size_t i = 2; i < path.size(); ++i)
		if (field.on_strip[path[i]] && field.on_strip[path[i]]->first == runway.strip) {
			first_on = i;
			break;
		}
	if (out.turns.size() <= first_on)
		return std::nullopt;
	const double to_line = out.turns[at_b].second + (departure - joined);
	const double to_hold =
			std::max(1.0, std::min({out.turns[first_on].first - HOLD_SHORT_M * paths.k,
								  out.turns[1].second + 2.0, out.turns[first_on].first,
								  to_line - 1.0}));
	const double to_rotate = out.turns[at_b].second + (rotate - joined);
	auto [taxi_out, rest] = cut(out.legs, to_hold);
	auto [line_up, after_line] = cut(rest, to_line - to_hold);
	auto [roll, unused_roll] = cut(after_line, to_rotate - to_line);
	(void)unused_roll;
	for (auto &leg : roll) {
		leg.cap = MAX_STEP;
		leg.accel = pace.accel;
		leg.brake = pace.brake;
		leg.holds = runway.strip;
	}

	const double roll_out = std::min(paths.speed(ROLL_OUT_MS), pace.land);
	const double set_down = use.touch + NOSE_DOWN_M * paths.k;
	const double slowed =
			use.touch +
			std::max((pace.land * pace.land - roll_out * roll_out) / (2.0 * pace.brake),
					NOSE_DOWN_M * paths.k + 1.0);
	if (slowed > runway.length())
		return std::nullopt;
	const auto from = runway.node_before(set_down);
	auto leaves = static_cast<std::size_t>(
			std::lower_bound(runway.at.begin(), runway.at.end(), slowed) -
			runway.at.begin());
	leaves = std::max(from + 1, std::min(leaves, runway.nodes.size() - 1));
	const auto goal = [&](std::size_t a, std::size_t b) -> std::optional<double> {
		if (b != junction)
			return std::nullopt;
		return turn_cost(turn_deg(field.dir(a, b), field.dir(b, into))) * paths.k;
	};
	auto way_in = field.taxi_route(
			{runway.nodes[leaves - 1], runway.nodes[leaves]}, {}, 3.0, paths.k, goal);
	if (!way_in || way_in->empty())
		return std::nullopt;
	double in_cost = 0.0;
	for (std::size_t i = 1; i < way_in->size(); ++i)
		in_cost += field.span((*way_in)[i - 1], (*way_in)[i]);
	std::vector<std::size_t> in_nodes(
			runway.nodes.begin() + from, runway.nodes.begin() + leaves - 1);
	const auto at_junction = in_nodes.size() + way_in->size() - 1;
	in_nodes.insert(in_nodes.end(), way_in->begin(), way_in->end());
	in_nodes.insert(in_nodes.end(), stand.path.begin() + 1, stand.path.end());
	const auto taxi_in = paths.taxi(in_nodes, set_down - runway.at[from]);
	if (taxi_in.turns.size() <= at_junction)
		return std::nullopt;
	const double park_in = taxi_in.length() - stop_back;
	if (park_in < taxi_in.turns[at_junction].second + 1.0)
		return std::nullopt;
	auto [arrive, unused_arrival] = cut(taxi_in.legs, park_in);
	(void)unused_arrival;
	if (arrive.empty())
		return std::nullopt;
	const auto end = arrive.back();
	if (end.piece != park.first || std::abs(end.to - park.second) > 0.05)
		return std::nullopt;
	auto [braked, rest_in] = cut(arrive, slowed - set_down);
	for (auto &leg : braked) {
		leg.cap = pace.land;
		leg.brake = pace.brake;
		leg.holds = runway.strip;
	}
	for (auto &leg : rest_in) {
		const auto &key = paths.keys[leg.piece];
		if (key.kind == PieceKeyKind::Edge &&
				runway_edges.contains({key.nodes[0], key.nodes[1]}))
			leg.cap = roll_out;
		else
			break;
	}
	braked.insert(braked.end(), rest_in.begin(), rest_in.end());

	return Draft{stand_index, frame, use_index, std::move(push), std::move(taxi_out),
			std::move(line_up), std::move(roll), departure, std::move(braked), park,
			out_cost + in_cost};
}

double segment_gap(Point a0, Point a1, Point b0, Point b1);

struct Pose
{
	Point at{};
	double y{};
	Point ahead{};
	plane_bodies::Airframe frame{};

	std::array<std::tuple<Point, Point, double>, 2> parts(double size) const
	{
		const auto [nose, tail] = plane_bodies::reach_m(frame);
		const auto [unused_length, span] = plane_bodies::size_m(frame);
		(void)unused_length;
		const auto wing = add(at, mul(ahead, 0.1 * nose * size));
		const auto tip = mul(left(ahead), span / 2.0 * size);
		return {{{add(at, mul(ahead, nose * size)), sub(at, mul(ahead, tail * size)),
						 plane_bodies::cabin_m(frame) / 2.0 * size},
				{add(wing, tip), sub(wing, tip), 0.5 * size}}};
	}
	double reach(double size) const
	{
		const auto [nose, tail] = plane_bodies::reach_m(frame);
		const auto [unused_length, span] = plane_bodies::size_m(frame);
		(void)unused_length;
		return std::max({nose, tail, span / 2.0}) * size;
	}
	bool too_close(const Pose &other, double size, double k) const
	{
		if (std::abs(y - other.y) > CLEAR_HEIGHT_M * k)
			return false;
		const double gap = CLEAR_M * k;
		if (norm(sub(at, other.at)) > reach(size) + other.reach(size) + gap)
			return false;
		for (const auto &[a0, a1, ra] : parts(size))
			for (const auto &[b0, b1, rb] : other.parts(size))
				if (segment_gap(a0, a1, b0, b1) < ra + rb + gap)
					return true;
		return false;
	}
};

double point_to_segment(Point p, Point a, Point b)
{
	const auto ab = sub(b, a);
	const double length2 = dot(ab, ab);
	const double t =
			length2 > 0.0 ? std::clamp(dot(sub(p, a), ab) / length2, 0.0, 1.0) : 0.0;
	return norm(sub(p, add(a, mul(ab, t))));
}

double segment_gap(Point a0, Point a1, Point b0, Point b1)
{
	const double d1 = cross(sub(a1, a0), sub(b0, a0));
	const double d2 = cross(sub(a1, a0), sub(b1, a0));
	const double d3 = cross(sub(b1, b0), sub(a0, b0));
	const double d4 = cross(sub(b1, b0), sub(a1, b0));
	if (d1 * d2 < 0.0 && d3 * d4 < 0.0)
		return 0.0;
	return std::min({point_to_segment(a0, b0, b1), point_to_segment(a1, b0, b1),
			point_to_segment(b0, a0, a1), point_to_segment(b1, a0, a1)});
}

std::optional<std::size_t> Paths::circuit_piece(const Use &use, bool tight)
{
	PieceKey key{
			PieceKeyKind::Circuit, {use.runway.strip, 0, 0}, 0, use.runway.back, tight};
	if (const auto found = by_key.find(key); found != by_key.end())
		return found->second;
	const Circuit shape = tight ? TIGHT_CIRCUIT : WIDE_CIRCUIT;
	const auto fine =
			circuit(use.line, shape, use.beyond[std::size_t(tight)], use.touch, use.side);
	if (!fine)
		return std::nullopt;
	return piece(key, [&](const Paths &) { return fine->piece(); });
}

std::optional<std::pair<std::size_t, double>> Paths::orbit_piece(
		const Use &use, bool tight, std::size_t circuit_index)
{
	const Circuit shape = tight ? TIGHT_CIRCUIT : WIDE_CIRCUIT;
	const double radius = shape.radius * k;
	const auto far = use.line.at(use.line.length() + use.beyond[std::size_t(tight)]);
	const auto to_side = mul(left(use.line.last()), use.side);
	const auto from = add(far, mul(to_side, 2.0 * radius));
	const auto near = use.line.at(-shape.before * k);
	const auto to = add(near, mul(left(use.line.first()), use.side * 2.0 * radius));
	const double at = PI * radius + norm(sub(to, from)) / 2.0;
	PieceKey key{
			PieceKeyKind::Orbit, {use.runway.strip, 0, 0}, 0, use.runway.back, tight};
	if (const auto found = by_key.find(key); found != by_key.end())
		return std::pair{found->second, at};
	if (circuit_index >= pieces.size())
		return std::nullopt;
	const auto start = pieces[circuit_index].at(at);
	const auto ahead = unit(sub(to, from));
	const auto centre = add({start.x, start.z}, mul(to_side, radius));
	constexpr double step = 0.25;
	const auto n = static_cast<std::size_t>(std::ceil(2.0 * PI * radius / step));
	Fine fine;
	for (std::size_t i = 0; i <= n; ++i) {
		const double t = 2.0 * PI * i / n;
		const auto p = add(sub(centre, mul(to_side, radius * std::cos(t))),
				mul(ahead, radius * std::sin(t)));
		if (i % 8 == 0 && !clear({p.first, start.y, p.second}, CLEAR_TERRAIN_M * k))
			return std::nullopt;
		fine.push({p.first, start.y, p.second}, LEVEL_NOSE, false);
	}
	return std::pair{piece(key, [&](const Paths &) { return fine.piece(); }), at};
}

std::optional<std::size_t> Paths::climb_piece(
		const Use &use, plane_bodies::Airframe frame, double departure)
{
	PieceKey key{PieceKeyKind::Climb, {use.runway.strip, 0, 0},
			static_cast<std::int64_t>(std::llround(departure)), use.runway.back, false,
			frame};
	if (const auto found = by_key.find(key); found != by_key.end())
		return found->second;
	const Circuit shape = plane_bodies::slow(frame) ? TIGHT_CIRCUIT : WIDE_CIRCUIT;
	const auto [rotate, lift] = Pace::of(frame, k).take_off(departure);
	const auto fine = climb(use.line, frame, rotate, lift,
			use.beyond[std::size_t(plane_bodies::slow(frame))], top(shape, use.touch_y),
			use.touch_y);
	if (!fine)
		return std::nullopt;
	return piece(key, [&](const Paths &) { return fine->piece(); });
}

std::optional<Round> make_round(Paths &paths, const Draft &draft, const Use &use)
{
	const double k = paths.k;
	const auto strip = use.runway.strip;
	const auto pace = Pace::of(draft.frame, k);
	const auto climb = paths.climb_piece(use, draft.frame, draft.departure);
	const auto circuit = paths.circuit_piece(use, plane_bodies::slow(draft.frame));
	if (!climb || !circuit)
		return std::nullopt;
	const auto air = [&](std::size_t piece, double from, double to, double cap,
							 bool holds) {
		return Leg{piece, from, to, cap, pace.accel * 0.6, pace.brake,
				holds ? std::optional<std::size_t>(strip) : std::nullopt};
	};
	auto fly = draft.roll;
	const auto [rotate, lift] = pace.take_off(draft.departure);
	const double climbed = paths.pieces[*climb].len;
	const double past_end = std::clamp(
			std::min(use.runway.length() + 60.0 * k, lift + CLIMB_OUT_M * k) - rotate,
			0.0, climbed);
	fly.push_back(air(*climb, 0.0, past_end, pace.air, true));
	fly.push_back(air(*climb, past_end, climbed, pace.air, false));
	const double length = paths.pieces[*circuit].len;
	const double touch = length - NOSE_DOWN_M * k;
	const double slow_from = std::max(touch - 600.0 * k, 0.0);
	const double own_from =
			std::max(touch - std::max(pace.land * FINAL_S * STEPS_PER_S, 600.0 * k), 0.0);
	std::optional<std::tuple<std::size_t, double, double>> orbit;
	if (const auto loop =
					paths.orbit_piece(use, plane_bodies::slow(draft.frame), *circuit);
			loop && loop->second < own_from)
		orbit = std::tuple{loop->first, loop->second, paths.pieces[loop->first].len};
	auto laps = [&](std::size_t orbits) {
		auto legs = fly;
		if (orbit && orbits > 0) {
			const auto [piece, at, lap_length] = *orbit;
			legs.push_back(air(*circuit, 0.0, at, pace.air, false));
			for (std::size_t i = 0; i < orbits; ++i)
				legs.push_back(air(piece, 0.0, lap_length, pace.air, false));
			legs.push_back(air(*circuit, at, own_from, pace.air, false));
		} else {
			legs.push_back(air(*circuit, 0.0, own_from, pace.air, false));
		}
		legs.push_back(air(*circuit, own_from, slow_from, pace.air, true));
		legs.push_back(air(*circuit, slow_from, length, pace.land, true));
		legs.insert(legs.end(), draft.arrive.begin(), draft.arrive.end());
		return legs;
	};
	const auto steps = [](const std::vector<Leg> &legs) {
		std::vector<Step> out;
		for (const auto &[leg_index, at] : run(legs))
			out.push_back({legs[leg_index].piece, at, legs[leg_index].holds});
		return out;
	};
	const Step park{draft.park.first, draft.park.second, std::nullopt};
	const auto pause = [](Step at, double seconds) {
		return std::vector<Step>(static_cast<std::size_t>(seconds * STEPS_PER_S), at);
	};
	auto before = steps(draft.push);
	if (before.empty())
		return std::nullopt;
	const Step pushed = before.back();
	const auto startup = pause(pushed, START_UP_S);
	before.insert(before.end(), startup.begin(), startup.end());
	const auto taxi_out = steps(draft.out);
	before.insert(before.end(), taxi_out.begin(), taxi_out.end());
	auto lined_up = steps(draft.line_up);
	const Step lined = lined_up.empty() ? before.back() : lined_up.back();
	const auto lined_pause = pause(lined, LINE_UP_S);
	lined_up.insert(lined_up.end(), lined_pause.begin(), lined_pause.end());
	const std::size_t orbit_count = orbit ? ORBITS + 1 : 1;
	std::vector<std::vector<Step>> afters;
	afters.reserve(orbit_count);
	for (std::size_t count = 0; count < orbit_count; ++count) {
		auto after = lined_up;
		const auto flying_steps = steps(laps(count));
		after.insert(after.end(), flying_steps.begin(), flying_steps.end());
		afters.push_back(std::move(after));
	}
	return Round{
			draft.stand, draft.frame, strip, std::move(before), std::move(afters), park};
}

Pose pose(const Paths &paths, Step step, plane_bodies::Airframe frame)
{
	const auto p = paths.pieces.at(step.piece).at(step.at);
	const double yaw = p.yaw * PI / 180.0;
	return {{p.x, p.z}, p.y, {-std::sin(yaw), std::cos(yaw)}, frame};
}

std::optional<std::size_t> on_strip(
		const Paths &paths, Step step, const std::vector<std::size_t> &strips)
{
	if (step.holds)
		return step.holds;
	const auto kind = paths.keys.at(step.piece).kind;
	if (kind == PieceKeyKind::Circuit || kind == PieceKeyKind::Climb ||
			kind == PieceKeyKind::Orbit)
		return std::nullopt;
	const auto p = paths.pieces.at(step.piece).at(step.at);
	for (const auto strip_index : strips) {
		const auto &strip = paths.field.strips[strip_index];
		if (strip.nodes.size() < 2)
			continue;
		const auto a = paths.field.nodes[strip.nodes.front()].at;
		const auto b = paths.field.nodes[strip.nodes.back()].at;
		if (point_to_segment({p.x, p.z}, a, b) < 0.9 * HOLD_SHORT_M * paths.k)
			return strip_index;
	}
	return std::nullopt;
}

using Half = std::pair<std::vector<Pose>,
		std::vector<std::tuple<std::size_t, std::int64_t, std::int64_t>>>;
struct Sampled
{
	std::vector<Pose> before;
	std::vector<std::tuple<std::size_t, std::int64_t, std::int64_t>> busy_before;
	std::vector<Half> afters;
	Pose park;
};

Sampled sample(
		const Paths &paths, const Round &round, const std::vector<std::size_t> &strips)
{
	const auto half = [&](const std::vector<Step> &steps) {
		Half result;
		for (std::size_t i = 0; i < steps.size();
				i += static_cast<std::size_t>(STEPS_PER_S)) {
			result.first.push_back(pose(paths, steps[i], round.frame));
			const auto strip = on_strip(paths, steps[i], strips);
			if (!strip)
				continue;
			const auto seconds = static_cast<std::int64_t>(i / STEPS_PER_S);
			if (!result.second.empty() && std::get<0>(result.second.back()) == *strip &&
					std::get<2>(result.second.back()) >= seconds - 1) {
				std::get<2>(result.second.back()) = seconds;
			} else {
				result.second.emplace_back(*strip, seconds, seconds);
			}
		}
		return result;
	};
	const auto [before, busy_before] = half(round.before);
	Sampled result{before, busy_before, {}, pose(paths, round.park, round.frame)};
	for (const auto &after : round.afters)
		result.afters.push_back(half(after));
	return result;
}

struct Flight
{
	Round round;
	std::size_t start{}, wait{}, orbits{};
	const std::vector<Step> &after() const { return round.afters.at(orbits); }
};

bool overlap(std::pair<std::int64_t, std::int64_t> a,
		std::pair<std::int64_t, std::int64_t> b, std::int64_t round)
{
	const auto difference = b.first - a.first;
	const auto normalized = ((difference % round) + round) % round;
	const auto offset = normalized - difference;
	b.first += offset;
	b.second += offset;
	for (const auto offset : {-round, std::int64_t(0), round})
		if (a.first <= b.second + offset && b.first + offset <= a.second)
			return true;
	return false;
}

struct Sky
{
	std::size_t round{};
	std::vector<std::vector<Pose>> poses;
	std::vector<std::vector<std::pair<std::int64_t, std::int64_t>>> busy;

	std::int64_t room_around(std::size_t strip, std::int64_t from, std::int64_t to) const
	{
		const auto period = static_cast<std::int64_t>(round);
		const auto modulo = [period](std::int64_t value) {
			return ((value % period) + period) % period;
		};
		std::int64_t room = period;
		for (const auto &[a, b] : busy[strip])
			room = std::min(room, std::min(modulo(a - to), modulo(from - b)));
		return room;
	}
	bool stands_clear(const Pose &pose, double size, double k) const
	{
		for (const auto &at : poses)
			for (const auto &other : at)
				if (pose.too_close(other, size, k))
					return false;
		return true;
	}
	std::int64_t longest_idle() const
	{
		const auto period = static_cast<std::int64_t>(round);
		std::int64_t longest = 0;
		for (const auto &strip : busy) {
			std::vector<std::pair<std::int64_t, std::int64_t>> uses;
			for (const auto &[from, to] : strip) {
				const auto start = ((from % period) + period) % period;
				uses.emplace_back(start, start + to - from);
			}
			if (uses.empty())
				continue;
			std::sort(uses.begin(), uses.end());
			for (std::size_t i = 0; i < uses.size(); ++i) {
				const auto next = i + 1 < uses.size() ? uses[i + 1].first
													  : uses.front().first + period;
				longest = std::max(longest, next - uses[i].second);
			}
		}
		return longest;
	}
};

using Time = std::tuple<std::size_t, std::size_t, std::size_t>;

std::optional<Time> sky_place(
		Sky &sky, const Sampled &sampled, std::size_t first, double size, double k)
{
	const auto round = sky.round;
	if (!round || sampled.before.empty() || sampled.afters.empty())
		return std::nullopt;
	std::vector<std::uint32_t> blocked(round + 1);
	for (std::size_t t = 0; t < round; ++t) {
		const bool hit = std::any_of(
				sky.poses[t].begin(), sky.poses[t].end(), [&](const Pose &other) {
					return sampled.park.too_close(other, size, k);
				});
		blocked[t + 1] = blocked[t] + std::uint32_t(hit);
	}
	const auto stand_free = [&](std::size_t from, std::size_t to) {
		const auto a = from % round, b = to % round;
		if (to - from >= round)
			return blocked[round] == 0;
		return a <= b ? blocked[b] == blocked[a]
					  : blocked[round] - blocked[a] + blocked[b] == 0;
	};
	const auto meets = [&](const Pose &p, std::size_t t) {
		return std::any_of(sky.poses[t % round].begin(), sky.poses[t % round].end(),
				[&](const Pose &other) { return p.too_close(other, size, k); });
	};
	const auto runway_free = [&](std::size_t strip, std::int64_t from, std::int64_t to) {
		return std::none_of(
				sky.busy[strip].begin(), sky.busy[strip].end(), [&](const auto &busy) {
					return overlap({from - RUNWAY_PAD_S, to + RUNWAY_PAD_S}, busy,
							static_cast<std::int64_t>(round));
				});
	};
	const auto before_count = sampled.before.size();
	for (std::size_t delta = 0; delta < round; delta += START_STEP_S) {
		const auto start = (first + delta) % round;
		bool clear = std::all_of(sampled.busy_before.begin(), sampled.busy_before.end(),
				[&](const auto &busy) {
					return runway_free(std::get<0>(busy),
							static_cast<std::int64_t>(start) + std::get<1>(busy),
							static_cast<std::int64_t>(start) + std::get<2>(busy));
				});
		for (std::size_t i = 0; clear && i < sampled.before.size(); ++i)
			clear = !meets(sampled.before[i], start + i);
		if (!clear)
			continue;
		const Pose hold = sampled.before.back();
		for (std::size_t wait = 0; wait <= MAX_HOLD_S; wait += HOLD_STEP_S) {
			if (before_count + wait + sampled.afters[0].first.size() + MIN_DWELL_S >
					round)
				break;
			bool hold_clear = true;
			for (std::size_t i = 0; i < wait; ++i)
				if (meets(hold, start + before_count + i)) {
					hold_clear = false;
					break;
				}
			if (!hold_clear)
				break;
			const auto off = start + before_count + wait;
			std::optional<std::pair<std::int64_t, std::size_t>> best;
			for (std::size_t orbit = 0; orbit < sampled.afters.size(); ++orbit) {
				const auto &[after, busy_after] = sampled.afters[orbit];
				const auto total = before_count + wait + after.size();
				if (total + MIN_DWELL_S > round)
					break;
				bool usable = std::all_of(
						busy_after.begin(), busy_after.end(), [&](const auto &busy) {
							return runway_free(std::get<0>(busy),
									static_cast<std::int64_t>(off) + std::get<1>(busy),
									static_cast<std::int64_t>(off) + std::get<2>(busy));
						});
				if (!usable || !stand_free(start + total, start + round))
					continue;
				for (std::size_t i = 0; usable && i < after.size(); ++i)
					usable = !meets(after[i], off + i);
				if (!usable)
					continue;
				const auto took_off = busy_after.empty() ? std::size_t(0) : ([&] {
					const auto strip = std::get<0>(busy_after.back());
					for (std::size_t i = 0; i < busy_after.size(); ++i)
						if (std::get<0>(busy_after[i]) == strip)
							return i + 1;
					return std::size_t(0);
				}());
				std::int64_t room = EVEN_S;
				for (std::size_t i = took_off; i < busy_after.size(); ++i)
					room = std::min(room, sky.room_around(std::get<0>(busy_after[i]),
												  static_cast<std::int64_t>(off) +
														  std::get<1>(busy_after[i]),
												  static_cast<std::int64_t>(off) +
														  std::get<2>(busy_after[i])));
				if (!best || room > best->first)
					best = {room, orbit};
			}
			if (!best)
				continue;
			const auto orbit = best->second;
			const auto &[after, busy_after] = sampled.afters[orbit];
			const auto total = before_count + wait + after.size();
			for (std::size_t i = 0; i < sampled.before.size(); ++i)
				sky.poses[(start + i) % round].push_back(sampled.before[i]);
			for (std::size_t i = 0; i < wait; ++i)
				sky.poses[(start + before_count + i) % round].push_back(hold);
			for (std::size_t i = 0; i < after.size(); ++i)
				sky.poses[(off + i) % round].push_back(after[i]);
			for (std::size_t t = start + total; t < start + round; ++t)
				sky.poses[t % round].push_back(sampled.park);
			for (const auto &busy : sampled.busy_before)
				sky.busy[std::get<0>(busy)].emplace_back(
						static_cast<std::int64_t>(start) + std::get<1>(busy),
						static_cast<std::int64_t>(start) + std::get<2>(busy));
			for (const auto &busy : busy_after)
				sky.busy[std::get<0>(busy)].emplace_back(
						static_cast<std::int64_t>(off) + std::get<1>(busy),
						static_cast<std::int64_t>(off) + std::get<2>(busy));
			return Time{start, wait, orbit};
		}
	}
	return std::nullopt;
}

Point nose_of(const Field &field, std::size_t stand)
{
	return field.nodes[field.stands.at(stand).path.back()].at;
}

std::map<std::size_t, double> stands_apart(
		const Field &field, const std::vector<std::size_t> &stands, double k)
{
	std::vector<std::size_t> group(stands.size());
	std::iota(group.begin(), group.end(), 0);
	const auto root = [&](std::size_t i) {
		while (group[i] != i) {
			group[i] = group[group[i]];
			i = group[i];
		}
		return i;
	};
	std::vector<Point> noses;
	for (const auto stand : stands)
		noses.push_back(nose_of(field, stand));
	for (std::size_t a = 0; a < stands.size(); ++a)
		for (std::size_t b = a + 1; b < stands.size(); ++b)
			if (norm(sub(noses[a], noses[b])) < ALTERNATE_LINES_M * k)
				group[root(a)] = root(b);
	std::map<std::size_t, std::size_t> sizes;
	for (std::size_t i = 0; i < stands.size(); ++i)
		++sizes[root(i)];
	std::map<std::size_t, double> result;
	for (std::size_t a = 0; a < stands.size(); ++a) {
		const bool alternate = sizes[root(a)] == 3;
		double apart = std::numeric_limits<double>::max();
		if (!alternate)
			for (std::size_t b = 0; b < stands.size(); ++b)
				if (a != b)
					apart = std::min(apart, norm(sub(noses[b], noses[a])));
		result.emplace(stands[a], apart);
	}
	return result;
}

std::vector<plane_bodies::Airframe> frames_for(const Field &field, std::size_t stand,
		double apart, const std::vector<Use> &uses,
		std::optional<std::optional<plane_bodies::Airframe>> flies, double size, double k)
{
	const auto &path = field.stands[stand].path;
	double line = 0.0;
	for (std::size_t i = 1; i < path.size(); ++i)
		line += field.span(path[i - 1], path[i]);
	std::vector<plane_bodies::Airframe> result;
	for (auto it = plane_bodies::ALL.rbegin(); it != plane_bodies::ALL.rend(); ++it) {
		const auto frame = *it;
		const auto [length, span] = plane_bodies::size_m(frame);
		double needs;
		if (flies) {
			const auto passer = flies->value_or(frame);
			const double wing = plane_bodies::size_m(passer).second / 2.0 * size +
								(CLEAR_M + 1.0) * k;
			needs = length * size + NOSE_GAP_M * k + std::max(wing, TAIL_ROOM_M * k);
		} else {
			needs = std::max(PARKED_LINE_SHARE * length,
							plane_bodies::reach_m(frame).first + 2.0) *
							size +
					NOSE_GAP_M * k;
		}
		if (needs > line || span * size + (CLEAR_M + 1.5) * k > apart)
			continue;
		if (std::any_of(uses.begin(), uses.end(), [&](const Use &use) {
				return std::find(use.frames.begin(), use.frames.end(), frame) !=
					   use.frames.end();
			}))
			result.push_back(frame);
	}
	return result;
}

struct Taken
{
	std::vector<std::tuple<std::size_t, Point, plane_bodies::Airframe>> aircraft;
	bool room(const Field &field, std::size_t stand, plane_bodies::Airframe frame,
			double size, double k) const
	{
		const auto nose = nose_of(field, stand);
		for (const auto &[other, at, has] : aircraft)
			if (other != stand &&
					norm(sub(at, nose)) < (plane_bodies::size_m(frame).second +
												  plane_bodies::size_m(has).second) /
														  2.0 * size +
												  (CLEAR_M + 1.5) * k)
				return false;
		return true;
	}
	void take(const Field &field, std::size_t stand, plane_bodies::Airframe frame)
	{
		aircraft.emplace_back(stand, nose_of(field, stand), frame);
	}
};

std::vector<bool> pieces_used(std::size_t count, const std::vector<Flight> &flights,
		const std::vector<Step> &parked)
{
	std::vector<bool> used(count);
	for (const auto &flight : flights) {
		for (const auto &step : flight.round.before)
			used[step.piece] = true;
		for (const auto &step : flight.after())
			used[step.piece] = true;
		used[flight.round.park.piece] = true;
	}
	for (const auto &step : parked)
		used[step.piece] = true;
	return used;
}

std::size_t points_held(const std::vector<Piece> &pieces, const std::vector<bool> &used)
{
	std::size_t count = 1;
	for (std::size_t i = 0; i < std::min(pieces.size(), used.size()); ++i)
		if (used[i])
			count += pieces[i].pts.size();
	return count;
}

struct Room
{
	std::size_t flying{}, parked{}, points{};
	double ceiling{};
};

std::uint64_t hash(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t value = a * 0x9E3779B97F4A7C15ULL ^ b * 0xC2B2AE3D27D4EB4FULL;
	value ^= value >> 29;
	value *= 0xBF58476D1CE4E5B9ULL;
	return value ^ (value >> 32);
}

std::optional<std::tuple<trains::Route, std::vector<trains::Service>, std::size_t>>
plan_field(const Field &field, const std::vector<std::size_t> &strips,
		const std::vector<std::size_t> &stands, double k, double size,
		const Paths::Ground &ground, const std::vector<std::pair<Point, double>> &tall,
		Room room)
{
	if (strips.empty())
		return std::nullopt;
	const auto first = field.nodes[field.strips[strips.front()].nodes.front()].at;
	const double datum = ground(static_cast<int>(std::floor(first.first)),
			static_cast<int>(std::floor(first.second)))
								 .value_or(64.0);
	Paths paths(field, k, ground, datum, room.ceiling, tall);
	std::optional<Point> centre;
	if (!stands.empty()) {
		Point sum{};
		for (const auto s : stands)
			sum = add(sum, field.nodes[field.stands[s].path.back()].at);
		centre = mul(sum, 1.0 / stands.size());
	}
	auto uses = runway_uses(paths, strips, centre);
	if (uses.empty())
		return std::nullopt;

	const auto apart = stands_apart(field, stands, k);
	std::optional<plane_bodies::Airframe> largest;
	for (const auto s : stands) {
		const auto fits = frames_for(field, s, apart.at(s), uses,
				std::optional<std::optional<plane_bodies::Airframe>>(
						std::in_place, std::nullopt),
				size, k);
		if (!fits.empty() && (!largest || fits.front() > *largest))
			largest = fits.front();
	}
	std::vector<std::size_t> order = stands;
	std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		const auto fits_a = frames_for(field, a, apart.at(a), uses,
				std::optional<std::optional<plane_bodies::Airframe>>(
						std::in_place, largest),
				size, k);
		const auto fits_b = frames_for(field, b, apart.at(b), uses,
				std::optional<std::optional<plane_bodies::Airframe>>(
						std::in_place, largest),
				size, k);
		const auto frame_a = fits_a.empty() ? std::optional<plane_bodies::Airframe>{}
											: std::optional(fits_a.front());
		const auto frame_b = fits_b.empty() ? std::optional<plane_bodies::Airframe>{}
											: std::optional(fits_b.front());
		if (frame_a != frame_b)
			return frame_a > frame_b;
		return std::tuple{hash(field.stands[a].id, 0x41), a} <
			   std::tuple{hash(field.stands[b].id, 0x41), b};
	});
	const auto pick = [](std::size_t stand,
							  const std::vector<plane_bodies::Airframe> &fits) {
		const auto roll = hash(stand, 0x7) % 100;
		std::size_t start = roll <= 54 || fits.size() == 1	 ? 0
							: roll <= 84 || fits.size() == 2 ? 1
															 : 2;
		start = std::min(start, fits.size() - 1);
		std::vector<plane_bodies::Airframe> result(fits.begin() + start, fits.end());
		for (std::size_t i = start; i > 0; --i)
			result.push_back(fits[i - 1]);
		return result;
	};
	const auto target = std::min({MAX_FLYING_PER_FIELD, room.flying, uses.size() * 7});
	std::vector<Draft> drafts;
	std::vector<std::size_t> idle;
	Taken taken;
	for (const auto s : order) {
		const auto fits = frames_for(field, s, apart.at(s), uses,
				std::optional<std::optional<plane_bodies::Airframe>>(
						std::in_place, largest),
				size, k);
		std::optional<Draft> made;
		if (drafts.size() < target && !fits.empty()) {
			for (const auto frame : pick(s, fits)) {
				if (!taken.room(field, s, frame, size, k))
					continue;
				const auto cost = [&](const Draft &draft) {
					const auto already = std::count_if(drafts.begin(), drafts.end(),
							[&](const Draft &other) { return other.used == draft.used; });
					return draft.taxied + already * RUNWAY_TAKEN_M * k;
				};
				std::optional<Draft> best;
				for (std::size_t i = 0; i < uses.size(); ++i) {
					if (std::find(uses[i].frames.begin(), uses[i].frames.end(), frame) ==
							uses[i].frames.end())
						continue;
					if (auto draft = make_draft(paths, s, frame, i, uses[i], size);
							draft && (!best || cost(*draft) < cost(*best)))
						best = std::move(draft);
				}
				if (best) {
					made = std::move(best);
					break;
				}
			}
		}
		if (made) {
			taken.take(field, s, made->frame);
			drafts.push_back(std::move(*made));
		} else {
			idle.push_back(s);
		}
	}

	for (const auto &draft : drafts) {
		auto &use = uses[draft.used];
		const auto shape = plane_bodies::slow(draft.frame) ? TIGHT_CIRCUIT : WIDE_CIRCUIT;
		const auto lift = Pace::of(draft.frame, k).take_off(draft.departure).second;
		const auto at = use.line.at(lift);
		const double rise = use.touch_y + paths.top(shape, use.touch_y) -
							paths.surface(at.first, at.second);
		const double need =
				lift + climb_run(draft.frame, rise, k) + 40.0 * k - use.line.length();
		const std::size_t slot = shape.radius == TIGHT_CIRCUIT.radius ? 1 : 0;
		use.beyond[slot] = std::max(use.beyond[slot], need);
	}
	std::vector<Round> rounds;
	for (const auto &draft : drafts) {
		if (auto round = make_round(paths, draft, uses[draft.used]))
			rounds.push_back(std::move(*round));
		else
			idle.push_back(draft.stand);
	}
	const auto seconds = [](const Round &round) {
		return static_cast<double>(round.before.size() + round.afters.front().size()) /
			   STEPS_PER_S;
	};
	rounds.erase(std::remove_if(rounds.begin(), rounds.end(),
						 [&](const Round &round) {
							 const bool fits =
									 seconds(round) + MIN_DWELL_S + 30.0 <= MAX_ROUND_S;
							 if (!fits)
								 idle.push_back(round.stand);
							 return !fits;
						 }),
			rounds.end());
	std::vector<Sampled> sampled;
	for (const auto &round : rounds)
		sampled.push_back(sample(paths, round, strips));
	const double longest =
			rounds.empty() ? 0.0
						   : seconds(*std::max_element(rounds.begin(), rounds.end(),
									 [&](const auto &a, const auto &b) {
										 return seconds(a) < seconds(b);
									 }));
	const auto clock_for = [&](const std::vector<Round> &rs,
								   const std::vector<Sampled> &samples) {
		std::vector<double> in_use(field.strips.size());
		std::vector<std::size_t> flown(field.strips.size());
		for (std::size_t i = 0; i < rs.size(); ++i) {
			++flown[rs[i].runway];
			const auto add_busy = [&](const auto &busy) {
				for (const auto &[strip, from, to] : busy)
					in_use[strip] += to - from + 2 * RUNWAY_PAD_S;
			};
			add_busy(samples[i].busy_before);
			if (!samples[i].afters.empty())
				add_busy(samples[i].afters.front().second);
		}
		const double busiest =
				in_use.empty() ? 0.0 : *std::max_element(in_use.begin(), in_use.end());
		const auto most =
				flown.empty() ? 0 : *std::max_element(flown.begin(), flown.end());
		return std::max({most * SLOT_S, longest + MIN_DWELL_S + 30.0,
				busiest / RUNWAY_SHARE, MIN_ROUND_S});
	};
	while (sampled.size() > 1 && clock_for(rounds, sampled) > MAX_ROUND_S) {
		sampled.pop_back();
		idle.push_back(rounds.back().stand);
		rounds.pop_back();
	}
	std::vector<std::size_t> on_runway(field.strips.size());
	std::vector<std::size_t> rank;
	for (const auto &round : rounds)
		rank.push_back(on_runway[round.runway]++);
	const std::size_t longest_clock = static_cast<std::size_t>(MAX_ROUND_S);
	std::size_t clock = std::max<std::size_t>(
			1, static_cast<std::size_t>(
					   std::ceil(std::min(clock_for(rounds, sampled), MAX_ROUND_S))));
	std::vector<std::optional<Time>> times;
	Sky winning_sky;
	for (;;) {
		const bool last = clock >= longest_clock;
		std::optional<std::tuple<std::int64_t, std::vector<std::optional<Time>>, Sky>>
				best;
		for (const bool packed : {false, true}) {
			Sky sky;
			sky.round = clock;
			sky.poses.resize(clock);
			sky.busy.resize(field.strips.size());
			std::vector<std::optional<Time>> candidate_times;
			std::size_t from = 0;
			for (std::size_t i = 0; i < rounds.size(); ++i) {
				const auto &round = rounds[i];
				const auto &sampled_round = sampled[i];
				const std::size_t first =
						packed ? from
							   : (rank[i] * clock /
												 std::max<std::size_t>(
														 1, on_runway[round.runway]) +
										 clock - sampled_round.before.size() % clock) %
										 clock;
				const auto time = sky_place(sky, sampled_round, first, size, k);
				if (time)
					from = std::get<0>(*time);
				else if (!last)
					break;
				candidate_times.push_back(time);
			}
			const bool all = candidate_times.size() == rounds.size() &&
							 std::all_of(candidate_times.begin(), candidate_times.end(),
									 [](const auto &time) { return time.has_value(); });
			if (all || (last && !packed)) {
				const auto idle_time = all ? sky.longest_idle()
										   : std::numeric_limits<std::int64_t>::max();
				if (!best || idle_time < std::get<0>(*best))
					best.emplace(idle_time, std::move(candidate_times), std::move(sky));
			}
		}
		if (best) {
			times = std::move(std::get<1>(*best));
			winning_sky = std::move(std::get<2>(*best));
			break;
		}
		clock = std::min(
				longest_clock, static_cast<std::size_t>(std::ceil(clock * CLOCK_STEP)));
	}
	std::vector<Flight> flights;
	for (std::size_t i = 0; i < rounds.size(); ++i) {
		if (times[i]) {
			const auto [start, wait, orbit] = *times[i];
			flights.push_back({std::move(rounds[i]), start, wait, orbit});
		} else {
			idle.push_back(rounds[i].stand);
		}
	}
	(void)winning_sky;
	while (!flights.empty() &&
			points_held(paths.pieces, pieces_used(paths.pieces.size(), flights, {})) >
					room.points)
		flights.pop_back();
	std::sort(idle.begin(), idle.end(), [&](std::size_t a, std::size_t b) {
		return std::tuple{hash(field.stands[a].id, 0x41), a} <
			   std::tuple{hash(field.stands[b].id, 0x41), b};
	});
	idle.erase(std::unique(idle.begin(), idle.end()), idle.end());
	std::vector<std::tuple<std::size_t, plane_bodies::Airframe, Step>> parked;
	std::vector<Pose> standing;
	Taken parked_taken;
	for (const auto &flight : flights)
		parked_taken.take(field, flight.round.stand, flight.round.frame);
	for (const auto s : idle) {
		if (parked.size() >= room.parked)
			break;
		if (hash(field.stands[s].id, 0x13) % 1000 >= STAND_OCCUPANCY * 1000.0)
			continue;
		const auto fits = frames_for(field, s, apart.at(s), uses, std::nullopt, size, k);
		for (const auto frame : pick(s, fits)) {
			if (!parked_taken.room(field, s, frame, size, k))
				continue;
			const auto line = paths.taxi(field.stands[s].path, 0.0);
			const double nose = plane_bodies::reach_m(frame).first;
			const double park_at = line.length() - (nose * size + NOSE_GAP_M * k);
			const auto [to_park, unused] = cut(line.legs, park_at);
			(void)unused;
			if (to_park.empty())
				continue;
			const Step step{to_park.back().piece, to_park.back().to, std::nullopt};
			const Pose aircraft_pose = pose(paths, step, frame);
			if (winning_sky.stands_clear(aircraft_pose, size, k) &&
					std::none_of(
							standing.begin(), standing.end(), [&](const Pose &other) {
								return aircraft_pose.too_close(other, size, k);
							})) {
				standing.push_back(aircraft_pose);
				parked_taken.take(field, s, frame);
				parked.emplace_back(s, frame, step);
				break;
			}
		}
	}
	const auto parked_steps = [&] {
		std::vector<Step> result;
		for (const auto &[stand, frame, step] : parked) {
			(void)stand;
			(void)frame;
			result.push_back(step);
		}
		return result;
	};
	auto used = pieces_used(paths.pieces.size(), flights, parked_steps());
	while (points_held(paths.pieces, used) > room.points && !parked.empty()) {
		parked.pop_back();
		used = pieces_used(paths.pieces.size(), flights, parked_steps());
	}
	if (flights.empty() && parked.empty())
		return std::nullopt;

	std::vector<std::size_t> starts(paths.pieces.size());
	std::vector<trains::RoutePoint> route_points;
	std::vector<double> surface_y;
	trains::Air air;
	for (std::size_t i = 0; i < paths.pieces.size(); ++i) {
		if (!used[i])
			continue;
		starts[i] = route_points.size();
		for (const auto &point : paths.pieces[i].pts) {
			route_points.push_back({point.x, point.z});
			surface_y.push_back(point.y);
			air.yaw.push_back(static_cast<float>(point.yaw));
			air.climb.push_back(static_cast<float>(point.climb));
			air.nose.push_back(static_cast<float>(point.nose));
			air.gear.push_back(point.gear);
		}
	}
	if (route_points.empty())
		return std::nullopt;
	const auto last = route_points.size() - 1;
	route_points.push_back(route_points[last]);
	surface_y.push_back(surface_y[last]);
	air.yaw.push_back(air.yaw[last]);
	air.climb.push_back(0.0f);
	air.nose.push_back(air.nose[last]);
	air.gear.push_back(air.gear[last]);
	const auto place = [&](const Step &step) {
		return static_cast<std::int32_t>(
				std::llround((starts[step.piece] + step.at) * 1000.0));
	};
	trains::Route route;
	route.mode = ::arnis::trains::Mode::Air;
	route.shape = ::arnis::trains::Shape::Shuttle;
	route.ends[0] = route.ends[1] = ::arnis::trains::End::Buffer;
	route.ways.assign(route_points.size(), 0);
	route.points = std::move(route_points);
	route.surface_y = std::move(surface_y);
	route.surface_lift = 0.0;
	route.air = std::move(air);
	const auto livery_for = [&](std::size_t stand, plane_bodies::Airframe frame) {
		const auto choices = plane_bodies::liveries(frame);
		return choices[hash(field.stands[stand].id, 0x29) % choices.size()];
	};
	const auto service = [&](plane_bodies::Airframe frame,
								 const train_bodies::Livery &livery,
								 std::vector<std::int32_t> timetable) {
		::arnis::trains::Service out;
		out.route = 0;
		out.coaches.emplace_back(train_bodies::Plane{frame});
		out.offsets.push_back(0.0);
		out.timetable = std::move(timetable);
		out.trains.push_back({0, livery});
		return out;
	};
	const auto per = static_cast<std::size_t>(STEPS_PER_S);
	std::vector<::arnis::trains::Service> services;
	const auto flying = flights.size();
	for (const auto &flight : flights) {
		const auto length = clock * per;
		std::vector<std::int32_t> timetable(length, place(flight.round.park));
		const Step hold = flight.round.before.empty() ? flight.round.park
													  : flight.round.before.back();
		std::vector<Step> steps = flight.round.before;
		steps.insert(steps.end(), flight.wait * per, hold);
		const auto &after = flight.after();
		steps.insert(steps.end(), after.begin(), after.end());
		for (std::size_t i = 0; i < steps.size(); ++i)
			timetable[(flight.start * per + i) % length] = place(steps[i]);
		services.push_back(service(flight.round.frame,
				livery_for(flight.round.stand, flight.round.frame),
				std::move(timetable)));
	}
	for (const auto &[stand, frame, step] : parked)
		services.push_back(service(frame, livery_for(stand, frame), {place(step)}));
	return std::tuple(std::move(route), std::move(services), flying);
}

} // namespace

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

AirPlan AirPlan::build(const std::vector<ProcessedElement> &elements, double scale,
		std::size_t point_budget, double ceiling, Ground ground)
{
	const double k = scale;
	const double size = std::clamp(scale, 0.4, 3.0);
	const auto field = Field::read(elements, k);
	AirPlan plan;
	if (field.strips.empty())
		return plan;
	const auto tall = tall_buildings(elements, k, ground);
	const auto component = field.fields();
	std::vector<std::size_t> order = component;
	std::sort(order.begin(), order.end());
	order.erase(std::unique(order.begin(), order.end()), order.end());
	const auto longest = [&](std::size_t f) {
		double result = 0.0;
		for (const auto &strip : field.strips)
			if (component[strip.nodes.front()] == f)
				result = std::max(result, strip.length());
		return result;
	};
	order.erase(std::remove_if(order.begin(), order.end(),
						[&](std::size_t f) { return longest(f) <= 0.0; }),
			order.end());
	std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		if (longest(a) != longest(b))
			return longest(a) > longest(b);
		return a < b;
	});
	const auto room_for = std::min<std::size_t>(MAX_AIR_POINTS, point_budget);
	std::size_t used_points = 0;
	for (const auto f : order) {
		std::vector<std::size_t> strips, stands;
		for (std::size_t s = 0; s < field.strips.size(); ++s)
			if (component[field.strips[s].nodes.front()] == f)
				strips.push_back(s);
		for (std::size_t s = 0; s < field.stands.size(); ++s)
			if (component[field.stands[s].path.front()] == f)
				stands.push_back(s);
		Room room{MAX_FLYING > plan.flying_ ? MAX_FLYING - plan.flying_ : 0,
				MAX_PARKED > plan.parked_ ? MAX_PARKED - plan.parked_ : 0,
				room_for > used_points ? room_for - used_points : 0, ceiling};
		auto planned = plan_field(field, strips, stands, k, size, ground, tall, room);
		if (!planned)
			continue;
		auto &[route, services, flying] = *planned;
		used_points += route.points.size();
		plan.flying_ += flying;
		plan.parked_ += services.size() - flying;
		plan.fields_.emplace_back(std::move(route), std::move(services));
		const double pad = 30.0 * k;
		std::array<double, 4> extent{std::numeric_limits<double>::max(),
				std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
				std::numeric_limits<double>::lowest()};
		for (std::size_t n = 0; n < field.nodes.size(); ++n) {
			if (component[n] != f)
				continue;
			extent[0] = std::min(extent[0], field.nodes[n].at.first - pad);
			extent[1] = std::min(extent[1], field.nodes[n].at.second - pad);
			extent[2] = std::max(extent[2], field.nodes[n].at.first + pad);
			extent[3] = std::max(extent[3], field.nodes[n].at.second + pad);
		}
		plan.extents_.push_back(extent);
	}
	return plan;
}

} // namespace arnis::element_processing::aircraft
