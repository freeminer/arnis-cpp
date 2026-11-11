#include "ships.h"

#include "ship_bodies.h"
#include "train_bodies.h"
#include "../clipping.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <set>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace arnis::element_processing::ships
{
namespace
{
constexpr std::size_t MAX_FERRIES = 40;
constexpr std::size_t MAX_WATERWAYS = 40;
constexpr std::size_t MAX_LEISURE = 40;
constexpr double EDGE_REACH = 3.0;
constexpr double WATER_LIFT = 0.875;
constexpr double HEADROOM = 0.5;
constexpr std::size_t DRY_GAP = 3;
constexpr double CHANNEL_M = 48.0;
constexpr std::size_t CHANNEL_STRIDE = 3;
constexpr std::size_t WATER_CELL = 8;
constexpr double BOATS_PER_KM2 = 2.0;
constexpr double LOOP_RADIUS_MIN_M = 25.0;
constexpr double LOOP_RADIUS_MAX_M = 90.0;
constexpr double SHORE_M = 15.0;
constexpr double KEEP_OFF_RIVER_M = 120.0;
constexpr double KEEP_OFF_ROUTE_M = 60.0;
constexpr double PIER_REACH_M = 9.0;
constexpr double STOP_MERGE_M = 80.0;
constexpr double SHARED_M = 15.0;

std::uint64_t ship_hash(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t h = a * 0x9E3779B97F4A7C15ULL ^ b * 0xC2B2AE3D27D4EB4FULL;
	h ^= h >> 29;
	h *= 0xBF58476D1CE4E5B9ULL;
	return h ^ (h >> 32);
}

using ShipNode = std::tuple<std::uint64_t, double, double>;
using ShipNodes = std::vector<ShipNode>;

struct LeisureLoop
{
	std::uint64_t seed{};
	std::vector<trains::RoutePoint> points;
};

struct FerryWay
{
	ShipNodes nodes;
	std::uint64_t way{};
	bool cars{};
	std::uint64_t tag{};
	std::optional<std::uint32_t> duration;
};

struct Chain
{
	ShipNodes nodes;
	std::uint64_t way{};
	std::vector<std::uint64_t> ways;
	bool cars{};
	std::uint64_t tag{};
};

std::pair<int, int> block(double x, double z)
{
	return {static_cast<int>(std::floor(x)), static_cast<int>(std::floor(z))};
}

std::vector<double> channel_offsets(double size)
{
	const double step = std::max(2.0 * size, 1.0);
	const auto count = static_cast<std::size_t>(std::floor(CHANNEL_M * size / step));
	std::vector<double> out;
	out.reserve(count);
	for (std::size_t i = 1; i <= count; ++i)
		out.push_back(i * step);
	return out;
}

std::optional<bool> ferry_cars(const tags_t &tags)
{
	if (tags.get("route") != "ferry" || tags.get("disused") == "yes" ||
			tags.get("abandoned") == "yes" || tags.contains("highway") ||
			tags.contains("railway"))
		return std::nullopt;
	for (const auto *key : {"motor_vehicle", "motorcar", "hgv", "vehicle"})
		if (tags.get(key) == "yes" || tags.get(key) == "designated")
			return true;
	return false;
}

bool ferry_terminal(const tags_t &tags)
{
	if (tags.get("amenity") == "ferry_terminal")
		return true;
	if (tags.get("ferry") != "yes")
		return false;
	const auto pt = tags.get("public_transport");
	return pt == "platform" || pt == "stop_position" || pt == "station";
}

std::optional<std::uint8_t> waterway_class(const tags_t &tags)
{
	const auto waterway = tags.get("waterway");
	if (waterway != "river" && waterway != "canal" && waterway != "fairway")
		return std::nullopt;
	const auto tunnel = tags.get("tunnel");
	if ((!tunnel.empty() && tunnel != "no") || tags.get("lock") == "yes" ||
			tags.get("intermittent") == "yes")
		return std::nullopt;
	const auto is_no = [&](const char *key) {
		const auto value = tags.get(key);
		return value == "no" || value == "private";
	};
	const auto is_yes = [&](const char *key) {
		const auto value = tags.get(key);
		return value == "yes" || value == "designated" || value == "permissive";
	};
	if ((is_no("ship") && is_no("boat")) || (is_no("motorboat") && !is_yes("ship")))
		return std::nullopt;
	const auto cemt = tags.get("CEMT");
	if (!cemt.empty()) {
		std::string c;
		c.reserve(cemt.size());
		for (const unsigned char ch : cemt)
			if (!std::isspace(ch))
				c.push_back(static_cast<char>(std::toupper(ch)));
		if (c.starts_with("VII"))
			return 7;
		if (c.starts_with("VI"))
			return 6;
		if (c.starts_with('V'))
			return 5;
		if (c.starts_with("IV"))
			return 4;
		if (c.starts_with("III"))
			return 3;
		if (c.starts_with("II"))
			return 2;
		if (c.starts_with('I'))
			return 1;
		return 0;
	}
	if (waterway == "fairway")
		return 4;
	if (is_yes("ship"))
		return 3;
	if (is_yes("boat") || is_yes("motorboat"))
		return 0;
	return std::nullopt;
}

std::vector<ShipNodes> split_at_barriers(const ProcessedWay &way)
{
	std::vector<ShipNodes> out;
	ShipNodes run;
	for (const auto &node : way.nodes) {
		run.emplace_back(node.id, node.x + 0.5, node.z + 0.5);
		const auto barrier = node.tags.get("waterway");
		const bool stops_navigation = barrier == "weir" || barrier == "dam" ||
									  barrier == "waterfall" || barrier == "lock_gate" ||
									  barrier == "rapids" ||
									  node.tags.get("lock") == "yes";
		if (stops_navigation) {
			if (run.size() > 1)
				out.push_back(std::move(run));
			run.clear();
		}
	}
	if (run.size() > 1)
		out.push_back(std::move(run));
	return out;
}

std::optional<std::uint32_t> duration_minutes(const std::string &value)
{
	std::vector<std::uint32_t> parts;
	std::size_t start = 0;
	for (;;) {
		const auto colon = value.find(':', start);
		auto token = value.substr(
				start, colon == std::string::npos ? std::string::npos : colon - start);
		const auto first = token.find_first_not_of(" \t\r\n");
		const auto last = token.find_last_not_of(" \t\r\n");
		if (first == std::string::npos)
			return std::nullopt;
		token = token.substr(first, last - first + 1);
		std::uint32_t parsed{};
		const auto [ptr, ec] =
				std::from_chars(token.data(), token.data() + token.size(), parsed);
		if (ec != std::errc{} || ptr != token.data() + token.size())
			return std::nullopt;
		parts.push_back(parsed);
		if (colon == std::string::npos)
			break;
		start = colon + 1;
	}
	if (parts.size() == 1)
		return parts.front();
	if (parts.size() == 2 || parts.size() == 3) {
		if (parts[0] > (std::numeric_limits<std::uint32_t>::max() - parts[1]) / 60)
			return std::nullopt;
		return parts[0] * 60 + parts[1];
	}
	return std::nullopt;
}

std::optional<std::pair<double, double>> unit(double x, double z)
{
	const double length = std::hypot(x, z);
	if (length <= 1e-9)
		return std::nullopt;
	return std::pair{x / length, z / length};
}

double angle_degrees(std::pair<double, double> a, std::pair<double, double> b)
{
	const double dot = std::clamp(a.first * b.first + a.second * b.second, -1.0, 1.0);
	return std::acos(dot) * 180.0 / std::numbers::pi;
}

std::pair<double, double> heading(const ShipNodes &nodes, bool from_start)
{
	if (nodes.size() < 2)
		return {1.0, 0.0};
	const auto a = from_start ? nodes.front() : nodes.back();
	const auto b = from_start ? nodes[1] : nodes[nodes.size() - 2];
	return unit(std::get<1>(b) - std::get<1>(a), std::get<2>(b) - std::get<2>(a))
			.value_or(std::pair{1.0, 0.0});
}

std::vector<Chain> chains(const std::vector<FerryWay> &ways, double max_turn)
{
	std::unordered_map<std::uint64_t, std::vector<std::pair<std::size_t, bool>>> at;
	for (std::size_t i = 0; i < ways.size(); ++i) {
		if (ways[i].nodes.size() < 2)
			continue;
		at[std::get<0>(ways[i].nodes.front())].emplace_back(i, true);
		at[std::get<0>(ways[i].nodes.back())].emplace_back(i, false);
	}
	std::vector<std::size_t> order(ways.size());
	std::iota(order.begin(), order.end(), 0);
	std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
		const auto &an = ways[a].nodes;
		const auto &bn = ways[b].nodes;
		const bool aloose = at[std::get<0>(an.front())].size() == 1 ||
							at[std::get<0>(an.back())].size() == 1;
		const bool bloose = at[std::get<0>(bn.front())].size() == 1 ||
							at[std::get<0>(bn.back())].size() == 1;
		return std::tuple{!aloose, ways[a].way} < std::tuple{!bloose, ways[b].way};
	});
	std::vector<bool> used(ways.size(), false);
	std::vector<Chain> result;
	for (const auto start : order) {
		if (used[start] || ways[start].nodes.size() < 2)
			continue;
		used[start] = true;
		ShipNodes nodes = ways[start].nodes;
		std::vector<std::uint64_t> joined{ways[start].way};
		if (at[std::get<0>(nodes.back())].size() == 1 &&
				at[std::get<0>(nodes.front())].size() > 1)
			std::reverse(nodes.begin(), nodes.end());
		for (int direction = 0; direction < 2; ++direction) {
			for (;;) {
				const auto end = nodes.back();
				auto incoming = heading(nodes, false);
				incoming = {-incoming.first, -incoming.second};
				const auto found = at.find(std::get<0>(end));
				std::optional<std::tuple<std::size_t, bool, double>> next;
				if (found != at.end())
					for (const auto &[index, starts] : found->second) {
						if (used[index] || ways[index].tag != ways[start].tag)
							continue;
						const double turn = angle_degrees(
								incoming, heading(ways[index].nodes, starts));
						if (turn > max_turn)
							continue;
						if (!next || turn < std::get<2>(*next) ||
								(turn == std::get<2>(*next) &&
										ways[index].way < ways[std::get<0>(*next)].way))
							next = {index, starts, turn};
					}
				if (!next)
					break;
				const auto [index, starts, _] = *next;
				used[index] = true;
				joined.push_back(ways[index].way);
				auto more = ways[index].nodes;
				if (!starts)
					std::reverse(more.begin(), more.end());
				nodes.insert(nodes.end(), std::next(more.begin()), more.end());
			}
			std::reverse(nodes.begin(), nodes.end());
		}
		result.push_back({std::move(nodes), ways[start].way, std::move(joined),
				ways[start].cars, ways[start].tag});
	}
	return result;
}

std::vector<trains::RoutePoint> resample(const ShipNodes &nodes)
{
	std::vector<std::pair<double, double>> points;
	points.reserve(nodes.size());
	for (const auto &node : nodes)
		points.emplace_back(std::get<1>(node), std::get<2>(node));
	points.erase(std::unique(points.begin(), points.end(),
						 [](const auto &a, const auto &b) {
							 return std::hypot(a.first - b.first, a.second - b.second) <
									1e-6;
						 }),
			points.end());
	if (points.size() < 2)
		return {};
	double total = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i)
		total += std::hypot(points[i].first - points[i - 1].first,
				points[i].second - points[i - 1].second);
	const auto count = static_cast<std::size_t>(std::floor(total)) + 1;
	std::vector<trains::RoutePoint> out;
	out.reserve(count);
	std::size_t segment = 0;
	double start = 0.0;
	for (std::size_t k = 0; k < count; ++k) {
		const double distance = static_cast<double>(k);
		for (;;) {
			const auto [a, b] = std::pair{points[segment], points[segment + 1]};
			const double length = std::hypot(b.first - a.first, b.second - a.second);
			if (distance <= start + length || segment + 2 >= points.size()) {
				const double t =
						length > 0.0 ? std::clamp((distance - start) / length, 0.0, 1.0)
									 : 0.0;
				out.push_back({a.first + (b.first - a.first) * t,
						a.second + (b.second - a.second) * t});
				break;
			}
			start += length;
			++segment;
		}
	}
	return out;
}

std::vector<trains::RoutePoint> resample(const std::vector<trains::RoutePoint> &line)
{
	std::vector<trains::RoutePoint> points = line;
	points.erase(std::unique(points.begin(), points.end(),
						 [&](const auto &a, const auto &b) {
							 return std::hypot(a.x - b.x, a.z - b.z) < 1e-6;
						 }),
			points.end());
	if (points.size() < 2)
		return points;
	double total = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i)
		total += std::hypot(points[i].x - points[i - 1].x, points[i].z - points[i - 1].z);
	const auto count = static_cast<std::size_t>(std::floor(total)) + 1;
	std::vector<trains::RoutePoint> out;
	out.reserve(count);
	std::size_t segment = 0;
	double start = 0.0;
	for (std::size_t k = 0; k < count; ++k) {
		const double distance = static_cast<double>(k);
		for (;;) {
			const auto &a = points[segment];
			const auto &b = points[segment + 1];
			const double length = std::hypot(b.x - a.x, b.z - a.z);
			if (distance <= start + length || segment + 2 >= points.size()) {
				const double t =
						length > 0.0 ? std::clamp((distance - start) / length, 0.0, 1.0)
									 : 0.0;
				out.push_back({a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t});
				break;
			}
			start += length;
			++segment;
		}
	}
	return out;
}

void smooth(std::vector<trains::RoutePoint> &points)
{
	constexpr std::size_t REACH = 4;
	if (points.size() < 2 * REACH + 2)
		return;
	const auto source = points;
	for (std::size_t i = REACH; i + REACH < points.size(); ++i) {
		double x = 0.0, z = 0.0;
		for (std::size_t j = i - REACH; j <= i + REACH; ++j) {
			x += source[j].x;
			z += source[j].z;
		}
		points[i] = {x / (2 * REACH + 1), z / (2 * REACH + 1)};
	}
}

std::vector<trains::RoutePoint> resample_loop(std::vector<trains::RoutePoint> ring)
{
	ring.erase(std::unique(ring.begin(), ring.end(),
					   [](const auto &a, const auto &b) {
						   return std::hypot(a.x - b.x, a.z - b.z) < 1e-6;
					   }),
			ring.end());
	if (ring.size() < 2)
		return ring;
	ring.push_back(ring.front());
	double total = 0.0;
	for (std::size_t i = 1; i < ring.size(); ++i)
		total += std::hypot(ring[i].x - ring[i - 1].x, ring[i].z - ring[i - 1].z);
	const auto count =
			std::max<std::size_t>(1, static_cast<std::size_t>(std::llround(total)));
	const double spacing = total / count;
	std::vector<trains::RoutePoint> out;
	out.reserve(count);
	std::size_t segment = 0;
	double start = 0.0;
	for (std::size_t k = 0; k < count; ++k) {
		const double distance = static_cast<double>(k) * spacing;
		for (;;) {
			const auto &a = ring[segment];
			const auto &b = ring[segment + 1];
			const double length = std::hypot(b.x - a.x, b.z - a.z);
			if (distance <= start + length || segment + 2 >= ring.size()) {
				const double t =
						length > 0.0 ? std::clamp((distance - start) / length, 0.0, 1.0)
									 : 0.0;
				out.push_back({a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t});
				break;
			}
			start += length;
			++segment;
		}
	}
	return out;
}

void smooth_loop(std::vector<trains::RoutePoint> &points)
{
	constexpr std::size_t reach = 4;
	if (points.size() < 2 * reach + 2)
		return;
	const auto source = points;
	for (std::size_t i = 0; i < points.size(); ++i) {
		double x = 0.0, z = 0.0;
		for (std::int64_t d = -static_cast<std::int64_t>(reach);
				d <= static_cast<std::int64_t>(reach); ++d) {
			const auto n = static_cast<std::int64_t>(source.size());
			const auto j =
					static_cast<std::size_t>((static_cast<std::int64_t>(i) + d + n) % n);
			x += source[j].x;
			z += source[j].z;
		}
		points[i] = {x / (2 * reach + 1), z / (2 * reach + 1)};
	}
}

std::vector<LeisureLoop> leisure_loops(const cartesian::XZBBox &world, double scale,
		const std::function<bool(int, int)> &is_water,
		const std::vector<ShipNodes> &rivers,
		const std::vector<std::vector<trains::RoutePoint>> &routes)
{
	const int x0 = world.min_x(), z0 = world.min_z();
	const auto w = static_cast<std::size_t>(
			std::max(1, (world.max_x() - x0) / static_cast<int>(WATER_CELL) + 1));
	const auto h = static_cast<std::size_t>(
			std::max(1, (world.max_z() - z0) / static_cast<int>(WATER_CELL) + 1));
	if (w > 40000000 / h)
		return {};
	const auto centre = [&](std::size_t cx, std::size_t cz) {
		return std::pair{x0 + static_cast<int>(cx * WATER_CELL + WATER_CELL / 2),
				z0 + static_cast<int>(cz * WATER_CELL + WATER_CELL / 2)};
	};
	std::vector<bool> open(w * h);
	for (std::size_t i = 0; i < open.size(); ++i) {
		const auto [x, z] = centre(i % w, i / w);
		open[i] = x <= world.max_x() && z <= world.max_z() &&
				  world.contains(cartesian::XZPoint{x, z}) && is_water(x, z);
	}
	if (std::none_of(open.begin(), open.end(), [](bool value) { return value; }))
		return {};
	const auto keep_off = [&](const std::vector<trains::RoutePoint> &line, double reach) {
		const int radius = static_cast<int>(std::ceil(reach / WATER_CELL));
		const double step = std::max(reach / 2.0, static_cast<double>(WATER_CELL));
		for (std::size_t i = 1; i < line.size(); ++i) {
			const auto &a = line[i - 1];
			const auto &b = line[i];
			const double length = std::hypot(b.x - a.x, b.z - a.z);
			const auto steps = static_cast<std::size_t>(std::ceil(length / step));
			for (std::size_t s = 0; s <= steps; ++s) {
				const double t = steps ? static_cast<double>(s) / steps : 0.0;
				const double x = a.x + (b.x - a.x) * t;
				const double z = a.z + (b.z - a.z) * t;
				const int cx = static_cast<int>(std::floor((x - x0) / WATER_CELL));
				const int cz = static_cast<int>(std::floor((z - z0) / WATER_CELL));
				for (int dz = -radius; dz <= radius; ++dz)
					for (int dx = -radius; dx <= radius; ++dx) {
						const int px = cx + dx, pz = cz + dz;
						if (dx * dx + dz * dz > radius * radius || px < 0 || pz < 0 ||
								px >= static_cast<int>(w) || pz >= static_cast<int>(h))
							continue;
						open[static_cast<std::size_t>(pz) * w + px] = false;
					}
			}
		}
	};
	for (const auto &river : rivers) {
		std::vector<trains::RoutePoint> points;
		points.reserve(river.size());
		for (const auto &node : river)
			points.push_back({std::get<1>(node), std::get<2>(node)});
		keep_off(points, KEEP_OFF_RIVER_M * scale);
	}
	for (const auto &route : routes)
		keep_off(route, KEEP_OFF_ROUTE_M * scale);
	std::vector<std::uint16_t> distance(
			open.size(), std::numeric_limits<std::uint16_t>::max() / 2);
	for (std::size_t i = 0; i < distance.size(); ++i) {
		const auto cx = i % w, cz = i / w;
		if (!open[i] || cx == 0 || cz == 0 || cx + 1 == w || cz + 1 == h)
			distance[i] = 0;
	}
	for (int pass = 0; pass < 2; ++pass) {
		for (std::size_t step = 0; step < distance.size(); ++step) {
			const auto i = pass == 0 ? step : distance.size() - 1 - step;
			const auto cx = static_cast<std::int64_t>(i % w);
			const auto cz = static_cast<std::int64_t>(i / w);
			const std::int64_t sign = pass == 0 ? -1 : 1;
			std::uint16_t best = distance[i];
			for (const auto [dx, dz, cost] : std::array<std::array<int, 3>, 4>{
						 {{static_cast<int>(sign), 0, 3}, {0, static_cast<int>(sign), 3},
								 {static_cast<int>(sign), static_cast<int>(sign), 4},
								 {-static_cast<int>(sign), static_cast<int>(sign), 4}}}) {
				const auto px = cx + dx, pz = cz + dz;
				if (px < 0 || pz < 0 || px >= static_cast<std::int64_t>(w) ||
						pz >= static_cast<std::int64_t>(h))
					continue;
				const auto neighbor = distance[static_cast<std::size_t>(pz) * w + px];
				best = std::min<std::uint16_t>(best,
						static_cast<std::uint16_t>(
								std::min<int>(std::numeric_limits<std::uint16_t>::max(),
										neighbor + cost)));
			}
			distance[i] = best;
		}
	}
	const auto metres = [&](std::size_t i) {
		return distance[i] / 3.0 * WATER_CELL / scale;
	};
	const auto fits = [&](std::size_t i) {
		return open[i] && metres(i) >= LOOP_RADIUS_MIN_M + SHORE_M;
	};
	std::size_t count = 0;
	for (std::size_t i = 0; i < distance.size(); ++i)
		count += fits(i) ? 1U : 0U;
	const double cell_m = WATER_CELL / scale;
	const double area_km2 = count * cell_m * cell_m / 1e6;
	const auto wanted = std::min<std::size_t>(MAX_LEISURE,
			static_cast<std::size_t>(std::llround(area_km2 * BOATS_PER_KM2)));
	if (wanted == 0)
		return {};
	const auto every = std::max<std::size_t>(1, count / (wanted * 64));
	std::vector<std::size_t> candidates;
	for (std::size_t i = 0; i < distance.size(); ++i)
		if (fits(i) && ship_hash(i, 0x5A3) % every == 0)
			candidates.push_back(i);
	std::sort(candidates.begin(), candidates.end(),
			[](auto a, auto b) { return ship_hash(a, 0xB0A7) < ship_hash(b, 0xB0A7); });
	std::vector<std::pair<trains::RoutePoint, double>> placed;
	std::vector<LeisureLoop> out;
	for (const auto i : candidates) {
		if (out.size() >= wanted)
			break;
		const auto [x, z] = centre(i % w, i / w);
		const trains::RoutePoint at{static_cast<double>(x), static_cast<double>(z)};
		const double radius = std::min(metres(i) - SHORE_M, LOOP_RADIUS_MAX_M) * scale;
		const bool separated =
				std::all_of(placed.begin(), placed.end(), [&](const auto &p) {
					return std::hypot(p.first.x - at.x, p.first.z - at.z) >=
						   radius + p.second + 20.0 * scale;
				});
		if (!separated)
			continue;
		placed.emplace_back(at, radius);
		const auto roll = ship_hash(i, 0x100F);
		const auto phase = [&](unsigned k) {
			return static_cast<double>((roll >> (8 * k)) & 0xff) / 255.0 * 2.0 *
				   std::numbers::pi;
		};
		const double stretch = 0.6 + 0.4 * ((roll >> 40) & 0xff) / 255.0;
		const double turn = phase(3);
		const double sin_turn = std::sin(turn), cos_turn = std::cos(turn);
		std::vector<trains::RoutePoint> ring;
		ring.reserve(360);
		for (int k = 0; k < 360; ++k) {
			const double theta = k * std::numbers::pi / 180.0;
			const double rho = radius * (0.78 + 0.14 * std::sin(2.0 * theta + phase(1)) +
												0.08 * std::sin(3.0 * theta + phase(2)));
			const double u = rho * std::cos(theta), v = rho * stretch * std::sin(theta);
			ring.push_back({at.x + u * cos_turn - v * sin_turn,
					at.z + u * sin_turn + v * cos_turn});
		}
		if ((roll & 1U) == 0)
			std::reverse(ring.begin(), ring.end());
		auto points = resample_loop(std::move(ring));
		if (points.size() >= 40)
			out.push_back({0x5EAULL ^ (out.size() * 0x9E3779B9ULL), std::move(points)});
	}
	return out;
}

bool wet(const std::optional<water_probe::Sample> &sample)
{
	return sample && sample->clearance().value_or(0) >= 1;
}

std::vector<ship_bodies::Vessel> waterway_vessels(std::uint8_t klass, std::uint64_t seed)
{
	const auto roll = seed % 100;
	std::vector<ship_bodies::Vessel> out;
	if (klass >= 4) {
		out.push_back(roll < 40	  ? ship_bodies::Vessel::Barge
					  : roll < 70 ? ship_bodies::Vessel::ContainerBarge
					  : roll < 88 ? ship_bodies::Vessel::TankBarge
								  : ship_bodies::Vessel::RiverCruise);
	} else if (klass > 0) {
		out.push_back(
				roll < 70 ? ship_bodies::Vessel::Peniche : ship_bodies::Vessel::TourBoat);
	} else {
		out.push_back(roll < 75 ? ship_bodies::Vessel::TourBoat
								: ship_bodies::Vessel::Motorboat);
	}
	for (const auto vessel : {ship_bodies::Vessel::Peniche, ship_bodies::Vessel::TourBoat,
				 ship_bodies::Vessel::Motorboat})
		if (std::find(out.begin(), out.end(), vessel) == out.end() &&
				(klass > 0 || vessel != ship_bodies::Vessel::Peniche))
			out.push_back(vessel);
	return out;
}

std::pair<double, double> normal(
		const std::vector<trains::RoutePoint> &points, std::size_t i, bool looped = false)
{
	const auto at = [&](std::int64_t index) -> const trains::RoutePoint & {
		const auto n = static_cast<std::int64_t>(points.size());
		if (looped)
			return points[static_cast<std::size_t>((index % n + n) % n)];
		return points[static_cast<std::size_t>(
				std::clamp(index, std::int64_t{0}, n - 1))];
	};
	const auto &a = at(static_cast<std::int64_t>(i) - 3);
	const auto &b = at(static_cast<std::int64_t>(i) + 3);
	const double dx = b.x - a.x, dz = b.z - a.z;
	const double len = std::hypot(dx, dz);
	return len > 1e-9 ? std::pair{-dz / len, dx / len} : std::pair{0.0, 1.0};
}

std::optional<water_probe::Sample> sample_at(
		const water_probe::WaterProbe &probe, double x, double z)
{
	const auto [bx, bz] = block(x, z);
	return probe.get(bx, bz, 0);
}

double waterway_headway(std::uint8_t klass)
{
	if (klass >= 6)
		return 300.0;
	if (klass == 5)
		return 420.0;
	if (klass == 4)
		return 600.0;
	return klass == 0 ? 900.0 : 1200.0;
}

std::vector<double> vessel_speed_caps(const std::vector<trains::RoutePoint> &points,
		ship_bodies::Vessel vessel, double scale, bool looped = false)
{
	if (points.empty())
		return {};
	const auto lateral = ship_bodies::acceleration(vessel)[2];
	const auto reach = static_cast<std::ptrdiff_t>(
			std::clamp(std::llround(30.0 * scale), 3LL, 60LL));
	const auto heading = [&](std::size_t i) -> std::optional<std::pair<double, double>> {
		if (!looped && i + 1 >= points.size())
			return std::nullopt;
		const auto next = (i + 1) % points.size();
		return unit(points[next].x - points[i].x, points[next].z - points[i].z);
	};
	std::vector<double> turns(points.size(), 0.0);
	for (std::size_t i = looped ? 0 : 1; i < points.size(); ++i) {
		const auto previous = i == 0 ? points.size() - 1 : i - 1;
		const auto a = heading(previous), b = heading(i);
		if (a && b)
			turns[i] = std::atan2(a->first * b->second - a->second * b->first,
					a->first * b->first + a->second * b->second);
	}
	std::vector<double> caps;
	caps.reserve(points.size());
	for (std::size_t i = 0; i < points.size(); ++i) {
		double turn = 0.0;
		if (looped) {
			const auto n = static_cast<std::int64_t>(points.size());
			for (std::int64_t d = -reach; d <= reach; ++d)
				turn += turns[static_cast<std::size_t>(
						(static_cast<std::int64_t>(i) + d + n * 2) % n)];
		} else {
			const auto begin = i > static_cast<std::size_t>(reach)
									   ? i - static_cast<std::size_t>(reach)
									   : 0;
			const auto end =
					std::min(points.size(), i + static_cast<std::size_t>(reach) + 1);
			for (std::size_t j = begin; j < end; ++j)
				turn += turns[j];
		}
		double kmh = ship_bodies::cruise_kmh(vessel);
		if (std::abs(turn) > 1e-3) {
			const double radius = (2.0 * reach + 1.0) / std::abs(turn) / scale;
			kmh = std::min(kmh, std::max(3.0, std::sqrt(lateral * radius) * 3.6));
		}
		caps.push_back(std::min(7.9, kmh * scale / 36.0));
	}
	return caps;
}

std::optional<std::pair<double, double>> widest(
		const std::vector<int> &room, const std::vector<double> &offsets, bool middle)
{
	std::optional<std::pair<double, double>> best;
	std::optional<std::size_t> start;
	for (std::size_t k = 0; k <= room.size(); ++k) {
		const bool open = k < room.size() && room[k] > 0;
		if (open && !start) {
			start = k;
		} else if (!open && start) {
			const auto a = *start;
			const double inner = a == 0 && middle ? 0.0 : offsets[a];
			const std::pair span{inner, offsets[k - 1]};
			if (!best || span.second - span.first > best->second - best->first)
				best = span;
			start.reset();
		}
	}
	return best;
}

std::optional<std::vector<double>> water_levels(
		const std::vector<trains::RoutePoint> &points,
		const water_probe::WaterProbe &probe, bool looped = false)
{
	std::vector<std::optional<double>> raw;
	raw.reserve(points.size());
	for (const auto &point : points) {
		const auto sample = sample_at(probe, point.x, point.z);
		raw.push_back(sample && sample->water() ? std::optional<double>(*sample->water())
												: std::nullopt);
	}
	const auto first = std::find_if(
			raw.begin(), raw.end(), [](const auto &level) { return level.has_value(); });
	if (first == raw.end())
		return std::nullopt;
	std::vector<double> filled;
	filled.reserve(raw.size());
	double last = **first;
	for (const auto &level : raw) {
		if (level)
			last = *level;
		filled.push_back(last);
	}
	std::vector<double> eased(filled.size());
	for (std::size_t i = 0; i < filled.size(); ++i) {
		double sum = 0.0;
		std::size_t count = 0;
		if (looped) {
			const auto n = static_cast<std::int64_t>(filled.size());
			for (std::int64_t d = -5; d <= 5; ++d) {
				const auto j = static_cast<std::size_t>(
						(static_cast<std::int64_t>(i) + d + n * 2) % n);
				sum += filled[j];
				++count;
			}
		} else {
			const auto begin = i > 5 ? i - 5 : 0;
			const auto end = std::min(filled.size(), i + 6);
			for (std::size_t j = begin; j < end; ++j) {
				sum += filled[j];
				++count;
			}
		}
		eased[i] = sum / count;
	}
	return eased;
}

std::optional<std::pair<trains::Route, trains::Service>> waterway_route(
		std::uint8_t water_class, std::uint64_t way,
		const std::vector<trains::RoutePoint> &points,
		const water_probe::WaterProbe &probe, double scale, double size)
{
	if (points.size() < 2)
		return std::nullopt;
	const auto offsets = channel_offsets(size);
	std::vector<std::array<std::vector<int>, 2>> rows;
	std::vector<bool> middle;
	for (std::size_t i = 0; i < points.size(); i += CHANNEL_STRIDE) {
		const auto &p = points[i];
		const auto [nx, nz] = normal(points, i);
		std::array<std::vector<int>, 2> row;
		for (std::size_t side = 0; side < 2; ++side) {
			const double sign = side == 0 ? 1.0 : -1.0;
			row[side].reserve(offsets.size());
			for (const double d : offsets) {
				const auto sample =
						sample_at(probe, p.x + sign * nx * d, p.z + sign * nz * d);
				const auto clearance = sample ? sample->clearance() : std::nullopt;
				row[side].push_back(clearance ? (*clearance >= 1 ? *clearance : -1) : 0);
			}
		}
		middle.push_back(wet(sample_at(probe, p.x, p.z)));
		rows.push_back(std::move(row));
	}
	const auto dry = std::count(middle.begin(), middle.end(), false);
	if (rows.empty() || dry * 20 > rows.size())
		return std::nullopt;
	auto levels = water_levels(points, probe);
	if (!levels)
		return std::nullopt;
	const auto spans_for = [&](std::size_t side) {
		std::vector<std::optional<std::pair<double, double>>> spans;
		spans.reserve(rows.size());
		for (std::size_t i = 0; i < rows.size(); ++i)
			spans.push_back(widest(rows[i][side], offsets, middle[i]));
		return spans;
	};
	for (const double direction : {1.0, -1.0}) {
		const std::size_t side = direction > 0.0 ? 0 : 1;
		const auto spans = spans_for(side);
		const auto lane_for = [&](double length, double half) {
			const auto window =
					static_cast<std::size_t>(std::ceil(length / 2.0 / CHANNEL_STRIDE));
			std::vector<std::optional<std::pair<double, double>>> bands;
			bands.reserve(spans.size());
			for (std::size_t k = 0; k < spans.size(); ++k) {
				const auto begin = k > window ? k - window : 0;
				const auto end = std::min(spans.size(), k + window + 1);
				double inner = 0.0, outer = std::numeric_limits<double>::max();
				for (std::size_t j = begin; j < end; ++j) {
					if (!spans[j]) {
						inner = std::numeric_limits<double>::max();
						outer = std::numeric_limits<double>::lowest();
						break;
					}
					inner = std::max(inner, spans[j]->first);
					outer = std::min(outer, spans[j]->second);
				}
				if (inner < outer)
					bands.push_back(std::pair{inner, outer});
				else
					bands.push_back(std::nullopt);
			}
			std::vector<double> lane(
					bands.size(), std::numeric_limits<double>::quiet_NaN());
			for (std::size_t k = 0; k < bands.size(); ++k) {
				const auto span = spans[k];
				if (bands[k])
					lane[k] = std::max(bands[k]->first,
							std::min((bands[k]->first + bands[k]->second) / 2.0,
									30.0 * size));
				else if (span)
					lane[k] = (span->first + span->second) / 2.0;
			}
			double last = 0.0;
			for (auto &offset : lane) {
				if (std::isnan(offset))
					offset = last;
				last = offset;
			}
			const double step = static_cast<double>(CHANNEL_STRIDE) / 8.0;
			const auto keep = [&](std::size_t k, double offset) {
				if (bands[k] && bands[k]->first + half <= bands[k]->second - half)
					return std::clamp(
							offset, bands[k]->first + half, bands[k]->second - half);
				return offset;
			};
			for (int pass = 0; pass < 3; ++pass) {
				for (std::size_t k = 1; k < lane.size(); ++k)
					lane[k] = keep(k,
							std::clamp(lane[k], lane[k - 1] - step, lane[k - 1] + step));
				for (std::size_t k = lane.size(); k-- > 1;)
					lane[k - 1] = keep(k - 1,
							std::clamp(lane[k - 1], lane[k] - step, lane[k] + step));
			}
			std::vector<std::pair<double, double>> result;
			result.reserve(lane.size());
			for (std::size_t k = 0; k < lane.size(); ++k) {
				const double room = bands[k] ? std::min(lane[k] - bands[k]->first,
													   bands[k]->second - lane[k])
											 : -1.0;
				result.emplace_back(lane[k], room);
			}
			return result;
		};
		const auto seed = ship_hash(way, side);
		const auto candidates = waterway_vessels(water_class, seed);
		for (const auto vessel : candidates) {
			const double length = ship_bodies::size_m(vessel).first * size;
			const double half = ship_bodies::size_m(vessel).second * size / 2.0;
			const auto lane = lane_for(length, half);
			const auto ends =
					static_cast<std::size_t>(std::ceil(length / CHANNEL_STRIDE));
			std::vector<double> room;
			if (lane.size() > 2 * ends)
				for (std::size_t i = ends; i < lane.size() - ends; ++i)
					room.push_back(lane[i].second - half);
			std::sort(room.begin(), room.end());
			if (room.size() / 50 >= room.size() || room[room.size() / 50] < 1.0)
				continue;
			std::optional<bool> low;
			for (const bool lowered : ship_bodies::lowers(vessel)
											  ? std::array{false, true}
											  : std::array{false, false}) {
				const double top =
						ship_bodies::air_draft_m(vessel, lowered) * size + HEADROOM;
				bool fits = true;
				for (std::size_t r = 0; r < rows.size() && fits; ++r)
					for (std::size_t j = 0; j < offsets.size(); ++j) {
						const int clearance = rows[r][side][j];
						if (clearance != 0 &&
								std::abs(offsets[j] - lane[r].first) <= half + 0.5 &&
								static_cast<double>(clearance) < top) {
							fits = false;
							break;
						}
					}
				if (fits) {
					low = lowered;
					break;
				}
			}
			if (!low)
				continue;
			const auto lane_at = [&](std::size_t i) {
				const auto k = i / CHANNEL_STRIDE;
				const double t = static_cast<double>(i % CHANNEL_STRIDE) / CHANNEL_STRIDE;
				const double a = lane[std::min(k, lane.size() - 1)].first;
				const double b = lane[std::min(k + 1, lane.size() - 1)].first;
				return a + (b - a) * t;
			};
			std::vector<trains::RoutePoint> shifted;
			shifted.reserve(points.size());
			for (std::size_t i = 0; i < points.size(); ++i) {
				const auto [nx, nz] = normal(points, i);
				const double offset = direction * lane_at(i);
				shifted.push_back({points[i].x + nx * offset, points[i].z + nz * offset});
			}
			if (direction < 0.0)
				std::reverse(shifted.begin(), shifted.end());
			auto heights = *levels;
			if (direction < 0.0)
				std::reverse(heights.begin(), heights.end());
			auto route_points = resample(shifted);
			smooth(route_points);
			std::vector<double> route_heights;
			route_heights.reserve(route_points.size());
			for (std::size_t i = 0; i < route_points.size(); ++i) {
				const auto index = i * (heights.size() - 1) /
								   std::max<std::size_t>(route_points.size() - 1, 1);
				route_heights.push_back(heights[index]);
			}
			trains::Route route;
			route.mode = trains::Mode::Water;
			route.water_vessel = vessel;
			route.shape = trains::Shape::Through;
			route.ends[0] = route.ends[1] = trains::End::Edge;
			route.points = std::move(route_points);
			route.ways.assign(route.points.size(), way);
			route.speed_caps = vessel_speed_caps(route.points, vessel, scale);
			route.surface_y = std::move(route_heights);
			route.surface_lift = WATER_LIFT;
			auto service = trains::ship_service(route, vessel, *low, seed, scale, size,
					waterway_headway(water_class));
			if (!service)
				continue;
			return std::pair{std::move(route), std::move(*service)};
		}
	}
	return std::nullopt;
}

std::optional<std::pair<trains::Route, trains::Service>> leisure_route(std::uint64_t seed,
		const std::vector<trains::RoutePoint> &points,
		const water_probe::WaterProbe &probe, double scale, double size)
{
	if (points.size() < 40)
		return std::nullopt;
	const bool sailboat_first = ship_hash(seed, 1) % 100 < 60;
	const std::array choices{
			ship_bodies::Vessel::Sailboat, ship_bodies::Vessel::Motorboat};
	for (std::size_t candidate = 0; candidate < (sailboat_first ? 2U : 1U); ++candidate) {
		const auto vessel = choices[sailboat_first ? candidate : 1];
		const double half = ship_bodies::size_m(vessel).second * size / 2.0;
		std::vector<double> offsets{1.7 * size, 1.3 * size};
		std::sort(offsets.begin(), offsets.end());
		offsets.erase(std::unique(offsets.begin(), offsets.end()), offsets.end());
		std::vector<double> across{0.0};
		for (const double d : offsets)
			if (d <= half + 1e-6)
				across.push_back(d);
		const auto try_build = [&](bool low) {
			const double top = ship_bodies::air_draft_m(vessel, low) * size + HEADROOM;
			std::size_t checked = 0, missed = 0;
			for (std::size_t i = 0; i < points.size(); ++i) {
				const auto [nx, nz] = normal(points, i, true);
				const auto reach = i % CHANNEL_STRIDE == 0 ? across.size() : 1U;
				for (std::size_t j = 0; j < reach; ++j)
					for (const double sign : {1.0, -1.0}) {
						if (j == 0 && sign < 0.0)
							continue;
						++checked;
						const auto sample =
								sample_at(probe, points[i].x + sign * nx * across[j],
										points[i].z + sign * nz * across[j]);
						const auto clearance =
								sample ? sample->clearance() : std::nullopt;
						if (clearance && *clearance < top)
							return false;
						if (!clearance)
							++missed;
					}
			}
			return missed * 30 <= checked;
		};
		bool low = false;
		if (!try_build(false)) {
			if (!ship_bodies::lowers(vessel) || !try_build(true))
				continue;
			low = true;
		}
		auto levels = water_levels(points, probe, true);
		if (!levels)
			continue;
		trains::Route route;
		route.mode = trains::Mode::Water;
		route.water_vessel = vessel;
		route.shape = trains::Shape::Loop;
		route.ends[0] = route.ends[1] = trains::End::Junction;
		route.points = points;
		route.ways.assign(route.points.size(), 0);
		route.speed_caps = vessel_speed_caps(route.points, vessel, scale, true);
		route.surface_y = std::move(*levels);
		route.surface_lift = WATER_LIFT;
		auto service = trains::ship_service(route, vessel, low, seed, scale, size);
		if (!service)
			continue;
		return std::pair{std::move(route), std::move(*service)};
	}
	return std::nullopt;
}
} // namespace

ShipPlan ShipPlan::build(const std::vector<ProcessedElement> &elements, double scale,
		const cartesian::XZBBox &world, const std::function<bool(int, int)> &is_water)
{
	ShipPlan plan;
	plan.scale_ = scale;
	plan.size_ = std::clamp(scale, 0.4, 3.0);
	const ::XZBBox clipping_world =
			::XZBBox(world.min_x(), world.min_z(), world.max_x(), world.max_z())
					.with_outline(world.outline());
	const auto in_outline = [&](ShipNodes nodes, std::uint64_t way, std::uint64_t &cuts) {
		if (auto runs = clipping::cut_line_to_outline(way, nodes, clipping_world, cuts))
			return *runs;
		std::vector<ShipNodes> runs;
		runs.push_back(std::move(nodes));
		return runs;
	};
	std::vector<FerryWay> ferry_ways;
	std::vector<FerryWay> waterway_ways;
	std::vector<ShipNodes> rivers;
	std::unordered_map<std::uint64_t, std::uint32_t> durations;
	for (const auto &element : elements) {
		if (element.is_node()) {
			const auto &node = element.as_node();
			if (ferry_terminal(node.tags)) {
				const trains::RoutePoint at{node.x + 0.5, node.z + 0.5};
				plan.piers_.push_back({at, {at}});
			}
			continue;
		}
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			continue;
		const auto waterway = way.tags.get("waterway");
		if (waterway == "river" || waterway == "canal") {
			ShipNodes river;
			river.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				river.emplace_back(node.id, node.x + 0.5, node.z + 0.5);
			rivers.push_back(std::move(river));
		}
		const auto cars = ferry_cars(way.tags);
		const auto channel_class = waterway_class(way.tags);
		if (!cars && !channel_class && ferry_terminal(way.tags)) {
			trains::RoutePoint centre{};
			std::vector<trains::RoutePoint> at;
			at.reserve(way.nodes.size());
			for (const auto &node : way.nodes) {
				const trains::RoutePoint point{node.x + 0.5, node.z + 0.5};
				centre.x += point.x;
				centre.z += point.z;
				at.push_back(point);
			}
			centre.x /= way.nodes.size();
			centre.z /= way.nodes.size();
			plan.piers_.push_back({centre, std::move(at)});
		}
		if (!cars && !channel_class)
			continue;
		FerryWay ferry;
		ferry.way = way.id;
		ferry.cars = cars.value_or(false);
		ferry.tag = cars ? std::uint64_t(*cars) : *channel_class;
		if (cars)
			ferry.duration = duration_minutes(way.tags.get("duration"));
		if (cars && ferry.duration)
			durations.insert_or_assign(way.id, *ferry.duration);
		if (channel_class && !cars) {
			std::uint64_t cuts = 0;
			for (auto &segment : split_at_barriers(way)) {
				if (segment.size() < 2)
					continue;
				for (auto &run : in_outline(std::move(segment), way.id, cuts))
					if (run.size() >= 2)
						waterway_ways.push_back({std::move(run), way.id, false,
								*channel_class, std::nullopt});
			}
			continue;
		}
		ferry.nodes.reserve(way.nodes.size());
		for (const auto &node : way.nodes)
			ferry.nodes.emplace_back(node.id, node.x + 0.5, node.z + 0.5);
		if (cars) {
			std::uint64_t cuts = 0;
			for (auto &run : in_outline(std::move(ferry.nodes), way.id, cuts)) {
				if (run.size() < 2)
					continue;
				auto piece = ferry;
				piece.nodes = std::move(run);
				ferry_ways.push_back(std::move(piece));
			}
		}
	}
	const auto at_edge = [&](const ShipNode &node) {
		const auto [id, x, z] = node;
		const auto near = [](double value, int lo, int hi) {
			return value < static_cast<double>(lo) + EDGE_REACH ||
				   value > static_cast<double>(hi + 1) - EDGE_REACH;
		};
		return clipping::is_invented_node_id(id) ||
			   near(x, world.min_x(), world.max_x()) ||
			   near(z, world.min_z(), world.max_z());
	};
	for (auto &chain : chains(ferry_ways, 120.0)) {
		if (plan.lines_.size() >= MAX_FERRIES)
			break;
		auto points = resample(chain.nodes);
		if (points.size() < 16)
			continue;
		smooth(points);
		Line line;
		line.way = chain.way;
		line.seed = chain.way;
		line.cars = chain.cars;
		for (const auto way_id : chain.ways) {
			const auto found = durations.find(way_id);
			if (found != durations.end() &&
					(!line.minutes || found->second > *line.minutes))
				line.minutes = found->second;
		}
		line.ends[0] =
				at_edge(chain.nodes.front()) ? trains::End::Edge : trains::End::Buffer;
		line.ends[1] =
				at_edge(chain.nodes.back()) ? trains::End::Edge : trains::End::Buffer;
		line.points = std::move(points);
		plan.lines_.push_back(std::move(line));
	}
	std::size_t sailed = 0;
	for (auto &chain : chains(waterway_ways, 40.0)) {
		if (sailed >= MAX_WATERWAYS)
			break;
		if (chain.nodes.size() < 2 ||
				!(at_edge(chain.nodes.front()) && at_edge(chain.nodes.back())))
			continue;
		auto points = resample(chain.nodes);
		if (static_cast<double>(points.size()) < 200.0 * scale)
			continue;
		smooth(points);
		Line line;
		line.way = chain.way;
		line.waterway_class = static_cast<std::uint8_t>(chain.tag);
		line.ends[0] = line.ends[1] = trains::End::Edge;
		line.points = std::move(points);
		plan.lines_.push_back(std::move(line));
		++sailed;
	}
	std::vector<std::vector<trains::RoutePoint>> existing_routes;
	existing_routes.reserve(plan.lines_.size());
	for (const auto &line : plan.lines_)
		existing_routes.push_back(line.points);
	for (auto &loop : leisure_loops(world, scale, is_water, rivers, existing_routes)) {
		Line line;
		line.leisure = true;
		line.looped = true;
		line.seed = loop.seed;
		line.points = std::move(loop.points);
		line.ends[0] = line.ends[1] = trains::End::Junction;
		plan.lines_.push_back(std::move(line));
	}
	return plan;
}

std::shared_ptr<water_probe::WaterProbe> ShipPlan::probe() const
{
	std::vector<water_probe::Column> columns;
	for (const auto &line : lines_) {
		std::vector<double> offsets;
		if (line.waterway_class)
			offsets = channel_offsets(size_);
		else if (line.leisure)
			offsets = {1.7 * size_, 1.3 * size_};
		else {
			for (const auto vessel :
					{ship_bodies::Vessel::RiverFerry, ship_bodies::Vessel::WaterBus,
							ship_bodies::Vessel::CarFerry, ship_bodies::Vessel::SeaFerry})
				offsets.push_back(ship_bodies::size_m(vessel).second * 0.5 * size_);
			offsets.push_back(2.0 * size_);
			offsets.push_back(6.0 * size_);
			std::sort(offsets.begin(), offsets.end());
			offsets.erase(std::unique(offsets.begin(), offsets.end()), offsets.end());
		}
		for (std::size_t i = 0; i < line.points.size(); ++i) {
			const auto &p = line.points[i];
			const auto [x, z] = block(p.x, p.z);
			columns.emplace_back(x, z, 0);
			if (i % (line.waterway_class || line.leisure ? CHANNEL_STRIDE : 3) != 0)
				continue;
			const auto [nx, nz] = normal(line.points, i, line.looped);
			for (const double d : offsets) {
				for (const double sign : {-1.0, 1.0}) {
					const auto [sx, sz] = block(p.x + nx * d * sign, p.z + nz * d * sign);
					columns.emplace_back(sx, sz, 0);
				}
			}
		}
	}
	return std::make_shared<water_probe::WaterProbe>(columns);
}

std::vector<std::pair<trains::Route, trains::Service>> ShipPlan::finish(
		const water_probe::WaterProbe &probe) const
{
	std::vector<std::pair<trains::Route, trains::Service>> out;
	std::vector<std::pair<trains::Route, trains::Service>> ferries;
	for (const auto &line : lines_) {
		if (line.leisure) {
			if (auto ship = leisure_route(line.seed, line.points, probe, scale_, size_))
				out.push_back(std::move(*ship));
			continue;
		}
		if (line.waterway_class) {
			if (auto ship = waterway_route(*line.waterway_class, line.way, line.points,
						probe, scale_, size_))
				out.push_back(std::move(*ship));
			continue;
		}
		const std::size_t n = line.points.size();
		std::vector<bool> is_wet(n);
		for (std::size_t i = 0; i < n; ++i)
			is_wet[i] = wet(sample_at(probe, line.points[i].x, line.points[i].z));
		const std::size_t gap = static_cast<std::size_t>(std::ceil(DRY_GAP * scale_));
		std::vector<std::pair<std::size_t, std::size_t>> sections;
		std::size_t section_start = 0;
		bool invalid_section = false;
		for (std::size_t i = 0; i < n;) {
			if (is_wet[i]) {
				++i;
				continue;
			}
			const auto dry_start = i;
			while (i < n && !is_wet[i])
				++i;
			if (dry_start == 0 || i == n || i - dry_start <= gap)
				continue;
			const double reach = 25.0 * scale_;
			const bool has_stage =
					std::any_of(piers_.begin(), piers_.end(), [&](const auto &pier) {
						for (std::size_t p = dry_start; p < i; ++p)
							for (const auto &at : pier.at)
								if (std::hypot(line.points[p].x - at.x,
											line.points[p].z - at.z) <= reach)
									return true;
						return false;
					});
			if (!has_stage) {
				invalid_section = true;
				break;
			}
			sections.emplace_back(section_start, dry_start);
			section_start = i - (i % 3);
		}
		if (invalid_section)
			continue;
		sections.emplace_back(section_start, n);
		for (std::size_t section_index = 0; section_index < sections.size();
				++section_index) {
			const auto [section_begin, section_end] = sections[section_index];
			if (section_end <= section_begin + 1)
				continue;
			const std::size_t k = std::max<std::size_t>(
					1, std::min<std::size_t>(12, (section_end - section_begin) / 4));
			const auto firm = [&](std::size_t a, std::size_t b) {
				if (b <= a)
					return false;
				const auto count =
						std::count(is_wet.begin() + a, is_wet.begin() + b, true);
				return count * 10 >= (b - a) * 9;
			};
			std::optional<std::size_t> start;
			for (std::size_t i = section_begin; i + k <= section_end; ++i)
				if (is_wet[i] && firm(i, i + k)) {
					start = i;
					break;
				}
			std::optional<std::size_t> end;
			for (std::size_t i = section_end; i >= section_begin + k; --i)
				if (is_wet[i - 1] && firm(i - k, i)) {
					end = i;
					break;
				}
			if (!start || !end || *end <= *start + 20)
				continue;
			const std::size_t begin = *start;
			const std::size_t finish = *end;
			std::size_t dry = 0;
			bool gap_too_long = false;
			for (std::size_t i = begin; i < finish; ++i) {
				dry = is_wet[i] ? 0 : dry + 1;
				if (dry > gap) {
					gap_too_long = true;
					break;
				}
			}
			if (gap_too_long)
				continue;
			const std::size_t span = finish - begin;
			const double length_m = static_cast<double>(span) / scale_;
			std::vector<ship_bodies::Vessel> choices;
			const bool reaches_edge =
					(begin <= 2 && line.ends[0] == trains::End::Edge) ||
					(finish + 3 >= n && line.ends[1] == trains::End::Edge);
			const bool sea =
					line.minutes ? *line.minutes >= 45
								 : length_m > 8000.0 || reaches_edge && length_m > 3000.0;
			if (line.cars) {
				if (sea)
					choices = {ship_bodies::Vessel::SeaFerry,
							ship_bodies::Vessel::CarFerry,
							ship_bodies::Vessel::RiverFerry};
				else if (length_m > 450.0)
					choices = {ship_bodies::Vessel::CarFerry,
							ship_bodies::Vessel::RiverFerry};
				else
					choices = {ship_bodies::Vessel::RiverFerry,
							ship_bodies::Vessel::WaterBus};
			} else {
				choices = {ship_bodies::Vessel::WaterBus};
			}
			std::optional<ship_bodies::Vessel> vessel;
			bool low = false;
			const auto sample_fit = [&](ship_bodies::Vessel v, bool lowered) {
				const auto half = ship_bodies::size_m(v).second * size_ * 0.5;
				std::vector<double> across{0.0};
				for (const auto candidate : {ship_bodies::Vessel::RiverFerry,
							 ship_bodies::Vessel::WaterBus, ship_bodies::Vessel::CarFerry,
							 ship_bodies::Vessel::SeaFerry}) {
					const double offset =
							ship_bodies::size_m(candidate).second * size_ * 0.5;
					if (offset <= half + 1e-6)
						across.push_back(offset);
				}
				for (const double offset : {2.0 * size_, 6.0 * size_})
					if (offset <= half + 1e-6)
						across.push_back(offset);
				std::sort(across.begin(), across.end());
				across.erase(std::unique(across.begin(), across.end()), across.end());
				const double draft =
						ship_bodies::air_draft_m(v, lowered) * size_ + HEADROOM;
				const auto margin = static_cast<std::size_t>(std::ceil(6.0 * scale_));
				const auto fit_begin = std::min(finish, begin + margin);
				const auto fit_end = finish > margin ? finish - margin : finish;
				std::size_t checked = 0, missed = 0;
				for (std::size_t i = fit_begin; i < fit_end; ++i) {
					const auto [nx, nz] = normal(line.points, i);
					const auto sample_count = i % 3 == 0 ? across.size() : std::size_t{1};
					for (std::size_t j = 0; j < sample_count; ++j)
						for (const double sign : {1.0, -1.0}) {
							if (j == 0 && sign < 0.0)
								continue;
							++checked;
							const auto sample = sample_at(probe,
									line.points[i].x + sign * nx * across[j],
									line.points[i].z + sign * nz * across[j]);
							const auto clearance =
									sample ? sample->clearance() : std::nullopt;
							if (clearance && *clearance < draft)
								return false;
							if (!clearance)
								++missed;
						}
				}
				return missed * 30 <= checked;
			};
			for (const auto candidate : choices) {
				const double boat_length = ship_bodies::size_m(candidate).first * size_;
				const auto margin = std::ceil(6.0 * scale_);
				if (static_cast<double>(span) < 2.5 * boat_length + 2.0 * margin)
					continue;
				if (sample_fit(candidate, false)) {
					vessel = candidate;
					break;
				}
				if (ship_bodies::lowers(candidate) && sample_fit(candidate, true)) {
					vessel = candidate;
					low = true;
					break;
				}
			}
			if (!vessel)
				continue;
			trains::Route route;
			route.mode = trains::Mode::Water;
			route.water_vessel = *vessel;
			route.shape = trains::Shape::Shuttle;
			route.ends[0] = begin <= 2 && line.ends[0] == trains::End::Edge
									? trains::End::Edge
									: trains::End::Buffer;
			route.ends[1] = finish + 3 >= n && line.ends[1] == trains::End::Edge
									? trains::End::Edge
									: trains::End::Buffer;
			route.points.assign(
					line.points.begin() + begin, line.points.begin() + finish);
			route.ways.assign(route.points.size(), line.way);
			route.speed_caps = vessel_speed_caps(route.points, *vessel, scale_);
			auto levels = water_levels(route.points, probe);
			if (!levels)
				continue;
			route.surface_y = std::move(*levels);
			route.surface_lift = WATER_LIFT;
			const double pier_reach = PIER_REACH_M * scale_;
			const double centre_reach = std::max(pier_reach, PIER_REACH_M * scale_ * 4.0);
			const double vessel_clear = ship_bodies::size_m(*vessel).first * size_;
			std::vector<std::pair<double, int>> found_stops;
			for (const auto &pier : piers_) {
				const auto nearest = [&](const trains::RoutePoint &point, double limit)
						-> std::optional<std::pair<std::size_t, double>> {
					std::optional<std::pair<std::size_t, double>> best;
					for (std::size_t i = 0; i < route.points.size(); ++i) {
						const double distance = std::hypot(
								route.points[i].x - point.x, route.points[i].z - point.z);
						if (distance <= limit && (!best || distance < best->second))
							best = {i, distance};
					}
					return best;
				};
				const bool touches = std::any_of(
						pier.at.begin(), pier.at.end(), [&](const auto &point) {
							return nearest(point, pier_reach).has_value();
						});
				if (!touches)
					continue;
				if (const auto stop = nearest(pier.centre, centre_reach))
					found_stops.emplace_back(static_cast<double>(stop->first), 0);
			}
			std::stable_sort(found_stops.begin(), found_stops.end(),
					[](const auto &a, const auto &b) { return a.first < b.first; });
			const double stop_merge = STOP_MERGE_M * scale_;
			for (const auto &[position, source] : found_stops) {
				(void)source;
				if (!route.stops.empty() && position - route.stops.back() < stop_merge) {
					continue;
				}
				if (position > vessel_clear &&
						position + vessel_clear < route.points.size() - 1)
					route.stops.push_back(position);
			}
			const auto seed = section_index == 0
									  ? line.seed
									  : ship_hash(line.seed, 100 + section_index);
			auto service = trains::ship_service(route, *vessel, low, seed, scale_, size_);
			if (!service)
				continue;
			ferries.emplace_back(std::move(route), std::move(*service));
		}
	}
	std::stable_sort(ferries.begin(), ferries.end(), [](const auto &a, const auto &b) {
		return a.first.points.size() > b.first.points.size();
	});
	const double shared_cell = std::max(SHARED_M * size_, 4.0);
	const auto cell = [shared_cell](const trains::RoutePoint &point) {
		return std::pair{static_cast<int>(std::floor(point.x / shared_cell)),
				static_cast<int>(std::floor(point.z / shared_cell))};
	};
	std::set<std::pair<int, int>> sailed;
	std::vector<std::pair<trains::Route, trains::Service>> kept;
	for (auto &ferry : ferries) {
		auto &points = ferry.first.points;
		std::size_t shared = 0;
		for (const auto &point : points) {
			const auto [cx, cz] = cell(point);
			bool near_sailed = false;
			for (int dx = -1; dx <= 1 && !near_sailed; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					if (sailed.contains({cx + dx, cz + dz})) {
						near_sailed = true;
						break;
					}
			shared += near_sailed;
		}
		if (shared * 2 > points.size())
			continue;
		for (const auto &point : points)
			sailed.insert(cell(point));
		kept.push_back(std::move(ferry));
	}
	kept.insert(kept.end(), std::make_move_iterator(out.begin()),
			std::make_move_iterator(out.end()));
	out = std::move(kept);
	return out;
}
} // namespace arnis::element_processing::ships
