#include "trains.h"

#include "../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include "../datapack/mod.h"
#include "../clipping.h"
#include "../strict_parse.h"
#include "../unicode_text.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace arnis::trains
{
namespace
{
constexpr double MAX_DEFLECT_DEG = 40.0;
constexpr double HEADING_REACH = 3.0;
constexpr double EDGE_REACH = 3.0;
constexpr double PLATFORM_REACH_M = 9.0;
constexpr double STOP_REACH_M = 6.0;
constexpr double STATION_REACH_M = 90.0;
constexpr double STOP_MERGE_M = 80.0;
constexpr double STEPS_PER_S = 10.0;
// Stay inside the datapack's carriage-following and cleanup reach at large scales.
constexpr double MAX_STEP = 7.9;
constexpr double LAYOVER_S = 25.0;
constexpr std::size_t MAX_TRAINS = 160;
constexpr std::size_t MAX_CARRIAGES = 1400;
constexpr std::size_t MAX_POINTS = 250000;
constexpr std::size_t MAX_STEPS = 72000;
constexpr double COUPLING_M = 0.4;
constexpr double TERMINUS_STOP_M = 160.0;
constexpr double END_MARGIN_M = 6.0;
constexpr double JUNCTION_RUN_M = 400.0;
constexpr double CREEP = 0.03;

std::optional<std::vector<double>> bedrock_route_heights(const Route &route,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y)
{
	std::vector<std::optional<double>> raw;
	// Rust zips points with ways, so a malformed/partial route must not probe
	// way id 0 for points that have no corresponding source way.
	const auto point_count = std::min(route.points.size(), route.ways.size());
	raw.reserve(point_count);
	for (std::size_t i = 0; i < point_count; ++i) {
		const auto &[x, z] = route.points[i];
		const std::uint64_t way = route.ways[i];
		const int bx = static_cast<int>(std::floor(x));
		const int bz = static_cast<int>(std::floor(z));
		std::optional<std::pair<int, int>> best;
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				if (const auto y = rail_y(way, bx + dx, bz + dz)) {
					const int distance = dx * dx + dz * dz;
					if (!best || distance < best->first)
						best = std::pair{distance, *y};
				}
		raw.push_back(best ? std::optional<double>(best->second) : std::nullopt);
	}
	const auto known = static_cast<std::size_t>(std::count_if(
			raw.begin(), raw.end(), [](const auto &y) { return y.has_value(); }));
	if (known * 2 < raw.size())
		return std::nullopt;
	auto filled = raw;
	std::optional<double> last;
	for (auto &y : filled) {
		if (y)
			last = y;
		else if (last)
			y = last;
	}
	std::optional<double> next;
	for (std::size_t i = filled.size(); i > 0; --i) {
		if (raw[i - 1])
			next = raw[i - 1];
		else if (!filled[i - 1] && next)
			filled[i - 1] = next;
	}
	std::vector<double> heights;
	heights.reserve(filled.size());
	for (const auto &y : filled)
		heights.push_back(y.value_or(0.0));
	constexpr int reach = 3;
	const auto n = heights.size();
	if (n == 0)
		return heights;
	std::vector<double> smooth(n);
	for (std::size_t i = 0; i < n; ++i) {
		double sum = 0.0;
		int count = 0;
		for (int d = -reach; d <= reach; ++d) {
			long j = static_cast<long>(i) + d;
			if (route.shape == Shape::Loop) {
				j %= static_cast<long>(n);
				if (j < 0)
					j += static_cast<long>(n);
			} else if (j < 0 || j >= static_cast<long>(n)) {
				continue;
			}
			sum += heights[static_cast<std::size_t>(j)];
			++count;
		}
		smooth[i] = count ? sum / count : 0.0;
	}
	return smooth;
}

double bedrock_heading(const Route &route, std::size_t i)
{
	const auto n = route.points.size();
	if (n == 0)
		return 0.0;
	const RoutePoint *a;
	const RoutePoint *b;
	if (route.shape == Shape::Loop) {
		a = &route.points[i % n];
		b = &route.points[(i + 1) % n];
	} else if (i + 1 < n) {
		a = &route.points[i];
		b = &route.points[i + 1];
	} else {
		a = &route.points[i ? i - 1 : i];
		b = &route.points[i];
	}
	return std::atan2(-(b->x - a->x), b->z - a->z) * 180.0 / std::numbers::pi;
}

std::vector<std::int64_t> bedrock_run_lengths(const std::vector<std::int32_t> &values)
{
	if (values.empty())
		return {};
	std::vector<std::int64_t> out{values.front()};
	for (std::size_t k = 1; k < values.size();) {
		const auto step = static_cast<std::int64_t>(values[k]) - values[k - 1];
		std::size_t count = 1;
		while (k + count < values.size() &&
				static_cast<std::int64_t>(values[k + count]) - values[k + count - 1] ==
						step)
			++count;
		out.push_back(step);
		out.push_back(static_cast<std::int64_t>(count));
		k += count;
	}
	return out;
}

double step_speed(double kmh, double scale)
{
	return std::min(kmh / 3.6 / STEPS_PER_S * scale, MAX_STEP);
}

struct Piece
{
	std::uint64_t way{};
	Mode mode{};
	std::optional<double> maxspeed;
	std::vector<RoutePoint> nodes;
	std::vector<std::uint64_t> node_ids;
};

struct Step
{
	std::size_t piece{};
	bool forward{};
};

struct StopFeature
{
	enum class Kind : std::uint8_t
	{
		Tram,
		Metro,
		Train,
		Any
	};
	enum class Source : std::uint8_t
	{
		Station,
		OnTrack,
		Platform
	};
	Kind kind{Kind::Any};
	Source source{Source::Station};
	std::vector<RoutePoint> at;
	RoutePoint centre;
};

std::uint64_t network_hash(std::uint64_t a, std::uint64_t b)
{
	std::uint64_t h = a * 0x9E3779B97F4A7C15ULL ^ b * 0xC2B2AE3D27D4EB4FULL;
	h ^= h >> 29;
	h *= 0xBF58476D1CE4E5B9ULL;
	return h ^ (h >> 32);
}

std::string_view tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? std::string_view{} : std::string_view(it->second);
}

bool yes(const tags_t &tags, const char *key)
{
	return tag(tags, key) == "yes";
}

std::optional<Mode> mode_of(const ProcessedWay &way)
{
	const auto &tags = way.tags;
	if (tags.contains("building") || tags.contains("highway") ||
			tags.contains("landuse") || tags.contains("natural") ||
			tags.contains("amenity") || tags.contains("leisure") ||
			tags.contains("barrier") || tags.contains("waterway") ||
			tag(tags, "area") == "yes" || tag(tags, "disused") == "yes")
		return std::nullopt;
	const auto service = tag(tags, "service");
	if (service == "yard" || service == "siding" || service == "spur" ||
			service == "crossover")
		return std::nullopt;
	const auto usage = tag(tags, "usage");
	if (usage == "industrial" || usage == "military" || usage == "test")
		return std::nullopt;
	const auto railway = tag(tags, "railway");
	if (railway == "tram")
		return Mode::Tram;
	if (railway == "subway")
		return Mode::Metro;
	if (railway == "light_rail")
		return Mode::SBahn;
	if (railway == "rail" || railway == "narrow_gauge" || railway == "preserved")
		return Mode::Rail;
	return std::nullopt;
}

double angle_deg(RoutePoint a, RoutePoint b)
{
	const auto dot = std::clamp(a.x * b.x + a.z * b.z, -1.0, 1.0);
	return std::acos(dot) * 180.0 / std::numbers::pi;
}

std::optional<RoutePoint> unit(double x, double z)
{
	const double length = std::hypot(x, z);
	if (length <= 1e-9)
		return std::nullopt;
	return RoutePoint{x / length, z / length};
}

std::optional<double> parse_kmh(std::string_view raw)
{
	const auto [begin, end] = unicode_text::trim_whitespace(raw);
	std::string value(raw.substr(begin, end - begin));
	std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
		return static_cast<char>(c >= 'A' && c <= 'Z' ? c + ('a' - 'A') : c);
	});
	if (value.ends_with("mph")) {
		value.resize(value.size() - 3);
		const auto [mph_begin, mph_end] = unicode_text::trim_whitespace(value);
		value = value.substr(mph_begin, mph_end - mph_begin);
		if (auto mph = strict_parse::f64(value))
			return *mph * 1.609;
		return std::nullopt;
	}
	if (auto kmh = strict_parse::f64(value); kmh && *kmh > 0.0)
		return kmh;
	return std::nullopt;
}

double cruise_kmh(Mode mode)
{
	switch (mode) {
	case Mode::Tram:
		return 45.0;
	case Mode::Metro:
		return 65.0;
	case Mode::SBahn:
		return 80.0;
	case Mode::Rail:
		return 110.0;
	}
	return 110.0;
}

double lateral_accel(Mode mode)
{
	switch (mode) {
	case Mode::Tram:
		return 1.0;
	case Mode::Metro:
		return 0.8;
	case Mode::SBahn:
		return 0.7;
	case Mode::Rail:
		return 0.6;
	}
	return 0.6;
}

bool served_by(Mode mode, StopFeature::Kind kind)
{
	switch (kind) {
	case StopFeature::Kind::Tram:
		return mode == Mode::Tram;
	case StopFeature::Kind::Metro:
		return mode == Mode::Metro;
	case StopFeature::Kind::Train:
		return mode == Mode::SBahn || mode == Mode::Rail;
	case StopFeature::Kind::Any:
		return mode != Mode::Tram;
	}
	return false;
}

StopFeature::Kind stop_kind(const tags_t &tags)
{
	if (yes(tags, "tram") || tag(tags, "railway") == "tram_stop")
		return StopFeature::Kind::Tram;
	if (yes(tags, "subway") || tag(tags, "station") == "subway")
		return StopFeature::Kind::Metro;
	if (yes(tags, "train") || yes(tags, "light_rail"))
		return StopFeature::Kind::Train;
	return StopFeature::Kind::Any;
}

std::vector<Piece> make_pieces(const std::vector<ProcessedElement> &elements)
{
	std::vector<std::pair<const ProcessedWay *, Mode>> ways;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() >= 2)
			if (auto mode = mode_of(way))
				ways.emplace_back(&way, *mode);
	}
	std::unordered_map<std::uint64_t, std::size_t> uses;
	for (const auto &[way, mode] : ways) {
		(void)mode;
		for (const auto &node : way->nodes)
			++uses[node.id];
	}
	std::vector<Piece> out;
	for (const auto &[way, mode] : ways) {
		Piece piece;
		piece.way = way->id;
		piece.mode = mode;
		if (const auto it = way->tags.find("maxspeed"); it != way->tags.end())
			piece.maxspeed = parse_kmh(it->second);
		for (std::size_t k = 0; k < way->nodes.size(); ++k) {
			const auto &node = way->nodes[k];
			if (!piece.node_ids.empty() && piece.node_ids.back() == node.id)
				continue;
			piece.node_ids.push_back(node.id);
			piece.nodes.push_back({node.x + 0.5, node.z + 0.5});
			const bool inner = k > 0 && k + 1 < way->nodes.size();
			if (inner && uses[node.id] > 1 && piece.nodes.size() >= 2) {
				out.push_back(std::move(piece));
				piece = Piece{};
				piece.way = way->id;
				piece.mode = mode;
				if (const auto it = way->tags.find("maxspeed"); it != way->tags.end())
					piece.maxspeed = parse_kmh(it->second);
				piece.node_ids.push_back(node.id);
				piece.nodes.push_back({node.x + 0.5, node.z + 0.5});
			}
		}
		if (piece.nodes.size() >= 2)
			out.push_back(std::move(piece));
	}
	return out;
}

std::optional<RoutePoint> heading_out(const Piece &piece, bool from_end)
{
	if (piece.nodes.size() < 2)
		return std::nullopt;
	const auto at = from_end ? piece.nodes.back() : piece.nodes.front();
	std::size_t far_index = from_end ? 0 : piece.nodes.size() - 1;
	if (from_end) {
		for (std::size_t i = piece.nodes.size() - 1; i-- > 0;) {
			far_index = i;
			if (std::hypot(piece.nodes[i].x - at.x, piece.nodes[i].z - at.z) >=
					HEADING_REACH)
				break;
		}
	} else {
		for (std::size_t i = 1; i < piece.nodes.size(); ++i) {
			far_index = i;
			if (std::hypot(piece.nodes[i].x - at.x, piece.nodes[i].z - at.z) >=
					HEADING_REACH)
				break;
		}
	}
	return unit(piece.nodes[far_index].x - at.x, piece.nodes[far_index].z - at.z);
}

std::pair<std::uint64_t, RoutePoint> arrival(const std::vector<Piece> &pieces, Step step)
{
	const auto &piece = pieces[step.piece];
	const auto node = step.forward ? piece.node_ids.back() : piece.node_ids.front();
	const auto heading = heading_out(piece, step.forward).value_or(RoutePoint{1.0, 0.0});
	return {node, {-heading.x, -heading.z}};
}

std::vector<Step> extend(const std::vector<Piece> &pieces,
		const std::unordered_map<std::uint64_t, std::vector<Step>> &at,
		std::vector<bool> &used, std::uint64_t node, RoutePoint incoming, Mode mode)
{
	std::vector<Step> out;
	for (;;) {
		const auto it = at.find(node);
		if (it == at.end())
			break;
		std::optional<std::tuple<std::size_t, bool, double>> best;
		for (const auto candidate : it->second) {
			if (used[candidate.piece] || pieces[candidate.piece].mode != mode)
				continue;
			const auto heading = heading_out(pieces[candidate.piece], !candidate.forward);
			if (!heading)
				continue;
			const auto turn = angle_deg(incoming, *heading);
			if (turn > MAX_DEFLECT_DEG)
				continue;
			// Rust orders bends with f64::total_cmp, then the piece index. Keep
			// that exact ordering: an epsilon can incorrectly prefer a slightly
			// worse route merely because it has a lower index.
			if (!best || turn < std::get<2>(*best) ||
					(turn == std::get<2>(*best) && candidate.piece < std::get<0>(*best)))
				best = {candidate.piece, candidate.forward, turn};
		}
		if (!best)
			break;
		const Step step{std::get<0>(*best), std::get<1>(*best)};
		used[step.piece] = true;
		out.push_back(step);
		std::tie(node, incoming) = arrival(pieces, step);
	}
	return out;
}

End end_of(std::uint64_t node, const cartesian::XZBBox &world,
		const std::unordered_map<std::uint64_t, std::vector<Step>> &at,
		const std::unordered_map<std::uint64_t, RoutePoint> &positions)
{
	const auto p = positions.at(node);
	const bool edge =
			clipping::is_invented_node_id(node) || p.x < world.min_x() + EDGE_REACH ||
			p.x > world.max_x() + 1.0 - EDGE_REACH || p.z < world.min_z() + EDGE_REACH ||
			p.z > world.max_z() + 1.0 - EDGE_REACH;
	if (edge)
		return End::Edge;
	const auto it = at.find(node);
	return it == at.end() || it->second.size() <= 1 ? End::Buffer : End::Junction;
}

std::optional<Route> lay_out(const std::vector<Piece> &pieces,
		const std::vector<Step> &run, const cartesian::XZBBox &world,
		const std::unordered_map<std::uint64_t, std::vector<Step>> &at,
		const std::unordered_map<std::uint64_t, RoutePoint> &positions, double scale)
{
	if (run.empty())
		return std::nullopt;
	const auto mode = pieces[run.front().piece].mode;
	struct SourcePoint
	{
		RoutePoint p;
		std::uint64_t way;
		std::optional<double> speed;
	};
	std::vector<SourcePoint> line;
	for (const auto step : run) {
		const auto &piece = pieces[step.piece];
		for (std::size_t k = 0; k < piece.nodes.size(); ++k) {
			const auto index = step.forward ? k : piece.nodes.size() - 1 - k;
			const auto &node = piece.nodes[index];
			if (k == 0 && !line.empty()) {
				line.back().way = piece.way;
				line.back().speed = piece.maxspeed;
				continue;
			}
			line.push_back({node, piece.way, piece.maxspeed});
		}
	}
	if (line.size() < 2)
		return std::nullopt;
	const auto first = run.front().forward ? pieces[run.front().piece].node_ids.front()
										   : pieces[run.front().piece].node_ids.back();
	const auto [last, arrive_heading] = arrival(pieces, run.back());
	const auto initial_heading =
			heading_out(pieces[run.front().piece], !run.front().forward);
	const bool closed = first == last && line.size() > 3 && initial_heading &&
						angle_deg(arrive_heading, *initial_heading) <= MAX_DEFLECT_DEG;
	Route route;
	route.mode = mode;
	if (closed) {
		line.pop_back();
		route.shape = Shape::Loop;
		route.ends[0] = route.ends[1] = End::Junction;
	} else {
		route.ends[0] = end_of(first, world, at, positions);
		route.ends[1] = end_of(last, world, at, positions);
		route.shape = route.ends[0] == End::Edge && route.ends[1] == End::Edge
							  ? Shape::Through
							  : Shape::Shuttle;
	}
	if (line.size() < 2)
		return std::nullopt;
	const auto segs = closed ? line.size() : line.size() - 1;
	const auto seg_length = [&](std::size_t i) {
		return std::hypot(line[(i + 1) % line.size()].p.x - line[i].p.x,
				line[(i + 1) % line.size()].p.z - line[i].p.z);
	};
	double total = 0.0;
	for (std::size_t i = 0; i < segs; ++i)
		total += seg_length(i);
	const auto count = closed ? static_cast<std::size_t>(std::llround(total))
							  : static_cast<std::size_t>(std::floor(total)) + 1;
	if (count < 8)
		return std::nullopt;
	const double spacing = closed ? total / count : 1.0;
	std::size_t segment = 0;
	double start = 0.0;
	for (std::size_t k = 0; k < count; ++k) {
		const double distance = k * spacing;
		for (;;) {
			const double length = seg_length(segment);
			if (distance <= start + length || segment + 1 >= segs) {
				const double t =
						length > 0.0 ? std::clamp((distance - start) / length, 0.0, 1.0)
									 : 0.0;
				const auto &a = line[segment];
				const auto &b = line[(segment + 1) % line.size()];
				route.points.push_back(
						{a.p.x + (b.p.x - a.p.x) * t, a.p.z + (b.p.z - a.p.z) * t});
				route.ways.push_back(a.way);
				route.speed_caps.push_back(a.speed.value_or(cruise_kmh(mode)));
				break;
			}
			start += length;
			++segment;
		}
	}
	const std::size_t n = route.points.size();
	if (n > 4) {
		const auto raw = route.points;
		for (std::size_t i = 0; i < n; ++i) {
			if (!closed && (i < 2 || i + 2 >= n))
				continue;
			double x = 0.0, z = 0.0;
			for (int d = -2; d <= 2; ++d) {
				const auto j =
						closed ? static_cast<std::size_t>(
										 (static_cast<std::int64_t>(i) + d +
												 static_cast<std::int64_t>(2 * n)) %
										 n)
							   : static_cast<std::size_t>(
										 static_cast<std::int64_t>(i) + d);
				x += raw[j].x;
				z += raw[j].z;
			}
			route.points[i] = {x / 5.0, z / 5.0};
		}
	}
	const int reach = std::clamp(
			static_cast<int>(std::llround((mode == Mode::Tram ? 10.0 : 20.0) * scale)), 3,
			40);
	std::vector<double> turns(n, 0.0);
	const auto heading = [&](std::size_t i) -> std::optional<RoutePoint> {
		const auto j = i + 1 < n ? i + 1 : (closed ? 0 : n);
		if (j == n)
			return std::nullopt;
		return unit(route.points[j].x - route.points[i].x,
				route.points[j].z - route.points[i].z);
	};
	for (std::size_t i = 0; i < n; ++i) {
		const auto prev = i ? heading(i - 1) : (closed ? heading(n - 1) : std::nullopt);
		const auto next = heading(i);
		if (prev && next)
			turns[i] = std::atan2(prev->x * next->z - prev->z * next->x,
					prev->x * next->x + prev->z * next->z);
	}
	route.speed_caps.resize(n);
	for (std::size_t i = 0; i < n; ++i) {
		double kmh = std::min(cruise_kmh(mode), route.speed_caps[i]);
		double turn = 0.0;
		for (int d = -reach; d <= reach; ++d) {
			const auto j = static_cast<std::int64_t>(i) + d;
			if (closed)
				turn += turns[static_cast<std::size_t>(
						(j % static_cast<std::int64_t>(n) + n) % n)];
			else if (j >= 0 && static_cast<std::size_t>(j) < n)
				turn += turns[static_cast<std::size_t>(j)];
		}
		if (std::abs(turn) > 1e-3) {
			const double radius = (2 * reach + 1) / std::abs(turn) / scale;
			kmh = std::min(
					kmh, std::max(std::sqrt(lateral_accel(mode) * radius) * 3.6, 12.0));
		}
		route.speed_caps[i] = step_speed(kmh, scale);
	}
	return route;
}

std::vector<Route> make_routes(
		const std::vector<Piece> &pieces, const cartesian::XZBBox &world, double scale)
{
	std::unordered_map<std::uint64_t, std::vector<Step>> at;
	std::unordered_map<std::uint64_t, RoutePoint> positions;
	for (std::size_t p = 0; p < pieces.size(); ++p) {
		const auto &piece = pieces[p];
		at[piece.node_ids.front()].push_back({p, true});
		at[piece.node_ids.back()].push_back({p, false});
		for (std::size_t i = 0; i < piece.node_ids.size(); ++i)
			positions[piece.node_ids[i]] = piece.nodes[i];
	}
	std::vector<bool> used(pieces.size(), false);
	std::vector<std::vector<Step>> runs;
	std::vector<std::pair<bool, std::uint64_t>> starts;
	for (const auto &[node, steps] : at)
		if (steps.size() == 1)
			starts.emplace_back(end_of(node, world, at, positions) != End::Edge, node);
	std::sort(starts.begin(), starts.end());
	for (const auto &[_, node] : starts) {
		const auto &step = at.at(node).front();
		if (used[step.piece])
			continue;
		used[step.piece] = true;
		std::vector<Step> run{step};
		const auto [end, incoming] = arrival(pieces, step);
		auto tail = extend(pieces, at, used, end, incoming, pieces[step.piece].mode);
		run.insert(run.end(), tail.begin(), tail.end());
		runs.push_back(std::move(run));
	}
	std::vector<std::size_t> rest;
	for (std::size_t p = 0; p < pieces.size(); ++p)
		if (!used[p])
			rest.push_back(p);
	std::sort(rest.begin(), rest.end(), [&](auto a, auto b) {
		return std::tie(pieces[a].way, a) < std::tie(pieces[b].way, b);
	});
	for (const auto p : rest) {
		if (used[p])
			continue;
		used[p] = true;
		const auto mode = pieces[p].mode;
		const Step forward{p, true};
		const auto [end, incoming] = arrival(pieces, forward);
		auto ahead = extend(pieces, at, used, end, incoming, mode);
		const Step reverse{p, false};
		const auto [start, back_heading] = arrival(pieces, reverse);
		auto behind = extend(pieces, at, used, start, back_heading, mode);
		std::vector<Step> run;
		for (auto it = behind.rbegin(); it != behind.rend(); ++it)
			run.push_back({it->piece, !it->forward});
		run.push_back(forward);
		run.insert(run.end(), ahead.begin(), ahead.end());
		runs.push_back(std::move(run));
	}
	std::vector<Route> out;
	for (const auto &run : runs)
		if (auto route = lay_out(pieces, run, world, at, positions, scale))
			out.push_back(std::move(*route));
	return out;
}

std::vector<StopFeature> stop_features(const std::vector<ProcessedElement> &elements)
{
	std::vector<StopFeature> out;
	for (const auto &element : elements) {
		if (element.is_node()) {
			const auto &node = element.as_node();
			const auto railway = tag(node.tags, "railway");
			const auto pt = tag(node.tags, "public_transport");
			StopFeature::Source source;
			if (railway == "stop" || railway == "tram_stop" || pt == "stop_position")
				source = StopFeature::Source::OnTrack;
			else if (railway == "station" || railway == "halt" ||
					 (pt == "station" &&
							 (node.tags.contains("railway") || yes(node.tags, "train"))))
				source = StopFeature::Source::Station;
			else
				continue;
			const auto kind = stop_kind(node.tags);
			if (pt == "stop_position" && kind == StopFeature::Kind::Any &&
					railway.empty())
				continue;
			const RoutePoint p{node.x + 0.5, node.z + 0.5};
			out.push_back({kind, source, {p}, p});
		} else if (element.is_way()) {
			const auto &way = element.as_way();
			const bool platform =
					tag(way.tags, "railway") == "platform" ||
					(tag(way.tags, "public_transport") == "platform" &&
							(yes(way.tags, "train") || yes(way.tags, "subway") ||
									yes(way.tags, "light_rail") ||
									yes(way.tags, "tram")));
			if (!platform || way.nodes.empty())
				continue;
			StopFeature f;
			f.kind = stop_kind(way.tags);
			f.source = StopFeature::Source::Platform;
			for (const auto &node : way.nodes)
				f.at.push_back({node.x + 0.5, node.z + 0.5});
			for (const auto p : f.at) {
				f.centre.x += p.x;
				f.centre.z += p.z;
			}
			f.centre.x /= f.at.size();
			f.centre.z /= f.at.size();
			out.push_back(std::move(f));
		}
	}
	return out;
}

void assign_stops(Route &route, const std::vector<StopFeature> &features, double scale)
{
	constexpr double CELL = 32.0;
	using Cell = std::pair<int, int>;
	const auto key = [=](double x, double z) {
		return Cell{static_cast<int>(std::floor(x / CELL)),
				static_cast<int>(std::floor(z / CELL))};
	};
	std::map<Cell, std::vector<std::size_t>> grid;
	for (std::size_t i = 0; i < route.points.size(); ++i)
		grid[key(route.points[i].x, route.points[i].z)].push_back(i);
	const auto nearest =
			[&](RoutePoint p,
					double reach) -> std::optional<std::pair<std::size_t, double>> {
		const int span = static_cast<int>(std::ceil(reach / CELL));
		const auto [cx, cz] = key(p.x, p.z);
		std::optional<std::pair<std::size_t, double>> best;
		for (int dx = -span; dx <= span; ++dx)
			for (int dz = -span; dz <= span; ++dz) {
				const auto it = grid.find({cx + dx, cz + dz});
				if (it == grid.end())
					continue;
				for (const auto i : it->second) {
					const auto &q = route.points[i];
					const double d = std::hypot(q.x - p.x, q.z - p.z);
					if (d <= reach && (!best || d < best->second))
						best = {i, d};
				}
			}
		return best;
	};
	std::vector<std::pair<double, StopFeature::Source>> found;
	for (const auto &feature : features) {
		if (!served_by(route.mode, feature.kind))
			continue;
		double reach = feature.source == StopFeature::Source::Platform	? PLATFORM_REACH_M
					   : feature.source == StopFeature::Source::OnTrack ? STOP_REACH_M
																		: STATION_REACH_M;
		reach *= scale;
		if (std::none_of(feature.at.begin(), feature.at.end(),
					[&](RoutePoint p) { return nearest(p, reach).has_value(); }))
			continue;
		if (const auto at = nearest(
					feature.centre, std::max(reach, PLATFORM_REACH_M * scale * 4.0)))
			found.emplace_back(at->first, feature.source);
	}
	std::stable_sort(found.begin(), found.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	std::vector<std::pair<double, StopFeature::Source>> stops;
	const double merge = STOP_MERGE_M * scale;
	for (const auto &[position, source] : found) {
		if (!stops.empty() && position - stops.back().first < merge) {
			if (source > stops.back().second)
				stops.back() = {position, source};
		} else {
			stops.emplace_back(position, source);
		}
	}
	if (route.shape == Shape::Loop && stops.size() > 1 &&
			stops.front().first + route.points.size() - stops.back().first < merge)
		stops.pop_back();
	for (const auto &[position, source] : stops) {
		(void)source;
		route.stops.push_back(position);
	}
}

void keep_right(std::vector<Route> &routes, double scale)
{
	const double near = 2.0 * scale, far = 8.0 * scale, cell = std::max(far, 1.0);
	using Cell = std::pair<int, int>;
	using Ref = std::pair<std::size_t, std::size_t>;
	const auto key = [cell](RoutePoint p) {
		return Cell{static_cast<int>(std::floor(p.x / cell)),
				static_cast<int>(std::floor(p.z / cell))};
	};
	std::map<Cell, std::vector<Ref>> grid;
	for (std::size_t r = 0; r < routes.size(); ++r) {
		if (routes[r].shape == Shape::Shuttle)
			continue;
		for (std::size_t i = 0; i < routes[r].points.size(); ++i)
			grid[key(routes[r].points[i])].emplace_back(r, i);
	}
	std::vector<bool> flip(routes.size(), false);
	for (std::size_t a = 0; a < routes.size(); ++a) {
		const auto &route = routes[a];
		if (route.shape == Shape::Shuttle)
			continue;
		std::size_t left = 0, right = 0;
		const auto n = route.points.size();
		for (std::size_t i = 0; i + 1 < n; i += std::max<std::size_t>(n / 32, 1)) {
			const auto p = route.points[i];
			const auto direction =
					unit(route.points[i + 1].x - p.x, route.points[i + 1].z - p.z);
			if (!direction)
				continue;
			std::unordered_map<std::size_t, std::pair<double, RoutePoint>>
					nearest_by_route;
			const auto [cx, cz] = key(p);
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz) {
					const auto it = grid.find({cx + dx, cz + dz});
					if (it == grid.end())
						continue;
					for (const auto [b, j] : it->second) {
						if (b == a || routes[b].mode != route.mode)
							continue;
						const auto q = routes[b].points[j];
						const double distance = std::hypot(q.x - p.x, q.z - p.z);
						auto found = nearest_by_route.find(b);
						if (found == nearest_by_route.end() ||
								distance < found->second.first)
							nearest_by_route[b] = {distance, q};
					}
				}
			for (const auto &[b, candidate] : nearest_by_route) {
				(void)b;
				const auto [distance, q] = candidate;
				if (distance < near || distance > far)
					continue;
				const double cross =
						direction->x * (q.z - p.z) - direction->z * (q.x - p.x);
				if (cross > 0.0)
					++right;
				else
					++left;
			}
		}
		flip[a] = right > left;
	}
	for (std::size_t r = 0; r < routes.size(); ++r) {
		if (!flip[r])
			continue;
		auto &route = routes[r];
		const auto n = route.points.size();
		std::reverse(route.points.begin(), route.points.end());
		std::reverse(route.ways.begin(), route.ways.end());
		std::reverse(route.speed_caps.begin(), route.speed_caps.end());
		std::swap(route.ends[0], route.ends[1]);
		for (auto &stop : route.stops)
			stop = static_cast<double>(n - 1) - stop;
		std::reverse(route.stops.begin(), route.stops.end());
		if (route.shape == Shape::Loop) {
			std::rotate(route.points.rbegin(), route.points.rbegin() + 1,
					route.points.rend());
			std::rotate(route.ways.rbegin(), route.ways.rbegin() + 1, route.ways.rend());
			std::rotate(route.speed_caps.rbegin(), route.speed_caps.rbegin() + 1,
					route.speed_caps.rend());
			for (auto &stop : route.stops) {
				stop = std::fmod(stop + 1.0, static_cast<double>(n));
				if (stop < 0.0)
					stop += n;
			}
			std::sort(route.stops.begin(), route.stops.end());
		}
	}
}

TrainKind route_kind(const Route &route, bool has_stops, double len_m, std::uint64_t seed)
{
	if (route.mode == Mode::Tram)
		return TrainKind::Tram;
	if (route.mode == Mode::Metro)
		return TrainKind::Metro;
	if (route.mode == Mode::SBahn)
		return TrainKind::SBahn;
	const auto roll = network_hash(seed, 3) % 100;
	if (has_stops) {
		if (roll <= 24 && len_m > 1500.0)
			return TrainKind::Intercity;
		return roll <= 59 ? TrainKind::Regional : TrainKind::DoubleDeck;
	}
	if (roll <= 54)
		return TrainKind::Freight;
	return roll <= 79 ? TrainKind::Intercity : TrainKind::Regional;
}

std::pair<double, double> accel_brake(Mode mode)
{
	switch (mode) {
	case Mode::Tram:
		return {1.1, 1.2};
	case Mode::Metro:
		return {1.0, 1.0};
	case Mode::SBahn:
		return {0.9, 0.9};
	case Mode::Rail:
		return {0.6, 0.7};
	}
	return {0.6, 0.7};
}

std::pair<double, double> dwell_seconds(Mode mode)
{
	switch (mode) {
	case Mode::Tram:
		return {14, 30};
	case Mode::Metro:
		return {18, 30};
	case Mode::SBahn:
		return {22, 40};
	case Mode::Rail:
		return {35, 60};
	}
	return {35, 60};
}

double headway_seconds(Mode mode)
{
	switch (mode) {
	case Mode::Tram:
	case Mode::Metro:
		return 150;
	case Mode::SBahn:
		return 200;
	case Mode::Rail:
		return 330;
	}
	return 330;
}

struct Body
{
	double front{}, back{}, span{};
};

struct Target
{
	double at{};
	std::optional<std::size_t> dwell;
};

struct Profile
{
	std::int64_t base{};
	std::optional<double> wrap;
	std::vector<double> ahead, back;

	static Profile build(std::int64_t lo, std::int64_t hi, std::optional<double> wrap,
			const Body &body, double braking,
			const std::function<double(std::int64_t)> &cap_at)
	{
		const auto behind = static_cast<std::int64_t>(std::ceil(body.span + body.back));
		const auto ahead_by = static_cast<std::int64_t>(std::ceil(body.front));
		const auto n = static_cast<std::size_t>(std::max<std::int64_t>(hi - lo + 1, 1));
		std::vector<double> caps;
		caps.reserve(static_cast<std::size_t>(hi - lo + behind + ahead_by + 1));
		for (auto i = lo - behind; i <= hi + ahead_by; ++i)
			caps.push_back(cap_at(i));
		const auto width = static_cast<std::size_t>(behind + ahead_by + 1);
		std::vector<double> under;
		under.reserve(n);
		std::deque<std::size_t> window;
		for (std::size_t i = 0; i < caps.size(); ++i) {
			while (!window.empty() && caps[window.back()] >= caps[i])
				window.pop_back();
			window.push_back(i);
			if (!window.empty() && window.front() + width <= i)
				window.pop_front();
			if (i + 1 >= width)
				under.push_back(caps[window.front()]);
		}
		under.resize(std::min(under.size(), n));
		const auto ease = [braking](
								  double v) { return std::sqrt(v * v + 2.0 * braking); };
		auto fwd = under, bwd = under;
		const int rounds = wrap ? 2 : 1;
		for (int round = 0; round < rounds; ++round) {
			for (std::size_t i = n; i-- > 0;) {
				if (i + 1 < n)
					fwd[i] = std::min(fwd[i], ease(fwd[i + 1]));
				else if (wrap)
					fwd[i] = std::min(fwd[i], ease(fwd[0]));
			}
			for (std::size_t i = 0; i < n; ++i) {
				if (i > 0)
					bwd[i] = std::min(bwd[i], ease(bwd[i - 1]));
				else if (wrap)
					bwd[i] = std::min(bwd[i], ease(bwd[n - 1]));
			}
		}
		return {lo, wrap, std::move(fwd), std::move(bwd)};
	}

	double at(double u, double dir) const
	{
		if (wrap)
			u = std::fmod(std::fmod(u, *wrap) + *wrap, *wrap);
		const auto &list = dir > 0 ? ahead : back;
		const auto n = static_cast<std::int64_t>(list.size());
		const auto cell = [&](std::int64_t i) {
			i -= base;
			if (wrap)
				i = (i % n + n) % n;
			else
				i = std::clamp<std::int64_t>(i, 0, n - 1);
			return list[static_cast<std::size_t>(i)];
		};
		return std::min(cell(static_cast<std::int64_t>(std::floor(u))),
				cell(static_cast<std::int64_t>(std::ceil(u))));
	}
};

struct Trip
{
	double u{}, v{}, a{}, b{};
	const Profile *profile{};
	std::vector<double> steps;

	void wait(std::size_t count)
	{
		v = 0;
		steps.insert(steps.end(), count, u);
	}
	bool run(double dir, Target target)
	{
		for (;;) {
			if (steps.size() > MAX_STEPS)
				return false;
			const double dist = (target.at - u) * dir;
			if (dist <= 1e-6)
				break;
			double top = std::min(v + a, profile->at(u, dir));
			if (target.dwell)
				top = std::min(top, std::sqrt(2.0 * b * dist));
			v = std::max(top, std::min(CREEP, dist));
			if (v >= dist) {
				u = target.at;
				steps.push_back(u);
				break;
			}
			u += dir * v;
			steps.push_back(u);
		}
		if (target.dwell)
			wait(*target.dwell);
		return true;
	}
	bool finish() const { return !steps.empty() && steps.size() <= MAX_STEPS; }
};

std::optional<std::pair<std::vector<std::int32_t>, std::int64_t>> timetable(
		const Route &route, const Body &body, double scale)
{
	const auto [accel, brake] = accel_brake(route.mode);
	const double a = accel / (STEPS_PER_S * STEPS_PER_S) * scale;
	const double b = brake / (STEPS_PER_S * STEPS_PER_S) * scale;
	const double cruise = step_speed(cruise_kmh(route.mode), scale);
	const auto [ds, de] = dwell_seconds(route.mode);
	const auto dwell_stop = static_cast<std::size_t>(ds * STEPS_PER_S);
	const auto dwell_end = static_cast<std::size_t>(de * STEPS_PER_S);
	const double len = route.length(), margin = END_MARGIN_M * scale;
	const double layover = cruise * LAYOVER_S * STEPS_PER_S;
	const auto n = route.points.size();
	const auto stop_at = [&](double s) { return s + body.span / 2.0; };
	const auto cap_at = [&](std::int64_t s) {
		if (route.shape == Shape::Loop)
			return route.speed_caps[static_cast<std::size_t>(
					(s % static_cast<std::int64_t>(n) + n) % n)];
		if (s < 0 || static_cast<double>(s) > len)
			return cruise;
		return route
				.speed_caps[std::min<std::size_t>(static_cast<std::size_t>(s), n - 1)];
	};
	if (route.shape == Shape::Shuttle) {
		const auto first = route.stops.empty() ? std::optional<double>{}
											   : stop_at(route.stops.front());
		const auto last = route.stops.empty() ? std::optional<double>{}
											  : stop_at(route.stops.back());
		const double low_end = margin + body.back + body.span;
		const double high_end = len - margin - body.front;
		const double low =
				route.ends[0] == End::Edge
						? -(body.front + layover)
						: (first && *first >= low_end &&
												  *first - low_end <
														  TERMINUS_STOP_M * scale
										  ? *first
										  : low_end);
		const double high =
				route.ends[1] == End::Edge
						? len + body.span + body.back + layover
						: (last && *last <= high_end &&
												  high_end - *last <
														  TERMINUS_STOP_M * scale
										  ? *last
										  : high_end);
		if (high - low < 10)
			return std::nullopt;
		auto profile = Profile::build(static_cast<std::int64_t>(std::floor(low)) - 1,
				static_cast<std::int64_t>(std::ceil(high)) + 1, {}, body, b, cap_at);
		const auto end_dwell = [&](End end) -> std::optional<std::size_t> {
			if (end == End::Edge)
				return {};
			return end == End::Buffer ? dwell_end : dwell_stop / 2;
		};
		std::vector<double> inner;
		for (double s : route.stops) {
			const double at = stop_at(s);
			if (at > low + 1 && at < high - 1)
				inner.push_back(at);
		}
		Trip trip{low, 0, a, b, &profile, {}};
		if (auto d = end_dwell(route.ends[0]))
			trip.wait(*d);
		else
			trip.v = cruise;
		for (double at : inner)
			if (!trip.run(1, {at, dwell_stop}))
				return {};
		if (!trip.run(1, {high, end_dwell(route.ends[1])}))
			return {};
		for (auto it = inner.rbegin(); it != inner.rend(); ++it)
			if (!trip.run(-1, {*it, dwell_stop}))
				return {};
		if (!trip.run(-1, {low, end_dwell(route.ends[0])}))
			return {};
		if (auto d = end_dwell(route.ends[0]))
			trip.steps.resize(trip.steps.size() > *d ? trip.steps.size() - *d : 0);
		if (!trip.finish())
			return {};
		std::vector<std::int32_t> out;
		out.reserve(trip.steps.size());
		for (double u : trip.steps)
			out.push_back(static_cast<std::int32_t>(std::llround(u * 1000)));
		return std::pair{std::move(out), std::int64_t{0}};
	}
	const double lap = route.shape == Shape::Loop
							   ? len
							   : len + body.span + body.back + body.front + layover;
	std::vector<double> stops;
	for (double s : route.stops) {
		double at = stop_at(s);
		if (route.shape == Shape::Loop || at <= len) {
			at = std::fmod(std::fmod(at, lap) + lap, lap);
			stops.push_back(at);
		}
	}
	std::sort(stops.begin(), stops.end());
	const auto cells = static_cast<std::int64_t>(std::ceil(lap));
	auto profile = Profile::build(0, cells - 1, lap, body, b, [&](std::int64_t s) {
		return cap_at(route.shape == Shape::Loop ? s : (s % cells + cells) % cells);
	});
	const double start =
			stops.empty() ? (route.shape == Shape::Loop
											? 0.0
											: len + body.span + body.back + layover / 2.0)
						  : stops.front();
	Trip trip{start, 0, a, b, &profile, {}};
	if (stops.empty()) {
		trip.v = std::min(profile.at(start, 1), cruise);
		if (!trip.run(1, {start + lap, {}}))
			return {};
	} else {
		trip.wait(dwell_stop);
		for (std::size_t i = 1; i < stops.size(); ++i)
			if (!trip.run(1, {stops[i], dwell_stop}))
				return {};
		if (!trip.run(1, {stops.front() + lap, dwell_stop}))
			return {};
		trip.steps.resize(
				trip.steps.size() > dwell_stop ? trip.steps.size() - dwell_stop : 0);
	}
	if (!trip.finish())
		return {};
	const auto lap_mm = static_cast<std::int64_t>(std::llround(lap * 1000));
	std::vector<std::int32_t> out;
	out.reserve(trip.steps.size());
	for (double u : trip.steps)
		out.push_back(static_cast<std::int32_t>(
				static_cast<std::int64_t>(std::llround(u * 1000)) % lap_mm));
	return std::pair{std::move(out), lap_mm};
}

double closest(const std::vector<std::int32_t> &timetable, std::int64_t period,
		std::size_t count)
{
	const auto n = timetable.size(), gap = n / count;
	double closest_distance = std::numeric_limits<double>::max();
	for (std::size_t i = 0; i < n; ++i) {
		const auto d =
				(static_cast<std::int64_t>(timetable[(i + gap) % n]) - timetable[i]) %
				std::max<std::int64_t>(period, 1);
		const auto modulus = std::max<std::int64_t>(period, 1);
		closest_distance =
				std::min(closest_distance, static_cast<double>((d + modulus) % modulus));
	}
	return closest_distance;
}

std::optional<Service> service_for(
		std::size_t route_index, const Route &route, double scale, double size)
{
	const std::uint64_t seed =
			route.ways.empty() ? route_index : route.ways[0] ^ (route_index * 0x9E37ULL);
	const double len_m = route.length() / scale;
	const auto kind = route_kind(route, !route.stops.empty(), len_m, seed);
	for (bool short_train : {false, true}) {
		auto [coaches, livery_for_departure] =
				train_bodies::compose(kind, seed, short_train || len_m < 400.0);
		std::vector<double> lengths;
		lengths.reserve(coaches.size());
		for (auto coach : coaches)
			lengths.push_back(train_bodies::length_m(coach) * size);
		std::vector<double> offsets{0};
		for (std::size_t i = 1; i < lengths.size(); ++i)
			offsets.push_back(offsets.back() + (lengths[i - 1] + lengths[i]) / 2 +
							  COUPLING_M * size);
		Body body{lengths.front() / 2, lengths.back() / 2, offsets.back()};
		const double train_m = (body.span + body.front + body.back) / scale;
		if (route.shape == Shape::Shuttle &&
				(route.ends[0] == End::Junction || route.ends[1] == End::Junction) &&
				len_m < std::max(JUNCTION_RUN_M, 3 * train_m))
			continue;
		auto schedule = timetable(route, body, scale);
		if (!schedule)
			continue;
		auto [times, period] = std::move(*schedule);
		std::vector<TrainDeparture> departures;
		if (route.shape == Shape::Shuttle) {
			departures.push_back({0, livery_for_departure(0)});
		} else {
			const auto headway =
					static_cast<std::size_t>(headway_seconds(route.mode) * STEPS_PER_S);
			const double room = body.span + body.front + body.back + 30 * scale;
			const std::size_t least = route.shape == Shape::Loop ? 2 : 1;
			std::size_t count = std::max(least, (times.size() + headway / 2) / headway);
			while (count > 1 && closest(times, period, count) < room * 1000)
				--count;
			if (count == 1 && period > 0 && period < room * 2000)
				continue;
			const auto start =
					static_cast<std::size_t>(network_hash(seed, 7) % times.size());
			for (std::size_t k = 0; k < count; ++k) {
				departures.push_back({(start + k * times.size() / count) % times.size(),
						livery_for_departure(k)});
			}
		}
		return Service{route_index, std::move(coaches), std::move(offsets),
				std::move(times), period, std::move(departures)};
	}
	return {};
}

} // namespace

double Route::length() const
{
	return shape == Shape::Loop
				   ? static_cast<double>(points.size())
				   : static_cast<double>(points.size() > 0 ? points.size() - 1 : 0);
}

TrainRouteNetwork TrainRouteNetwork::build(const std::vector<ProcessedElement> &elements,
		double scale, const cartesian::XZBBox &world)
{
	TrainRouteNetwork result;
	result.size_ = std::clamp(scale, 0.4, 3.0);
	auto track_pieces = make_pieces(elements);
	result.routes_ = make_routes(track_pieces, world, scale);
	const auto stops = stop_features(elements);
	for (auto &route : result.routes_)
		assign_stops(route, stops, scale);
	keep_right(result.routes_, scale);
	std::size_t trains = 0, carriages = 0, points = 0;
	for (std::size_t r = 0; r < result.routes_.size(); ++r) {
		const auto &route = result.routes_[r];
		if (trains >= MAX_TRAINS || carriages >= MAX_CARRIAGES)
			break;
		if (points + route.points.size() > MAX_POINTS)
			continue;
		auto service = service_for(r, route, scale, result.size_);
		if (!service || service->coaches.empty())
			continue;
		const auto room = std::min(MAX_TRAINS - trains,
				(MAX_CARRIAGES - carriages) / service->coaches.size());
		if (service->trains.size() > room)
			service->trains.resize(room);
		if (service->trains.empty())
			continue;
		trains += service->trains.size();
		carriages += service->trains.size() * service->coaches.size();
		points += route.points.size();
		result.services_.push_back(std::move(*service));
	}
	return result;
}

std::size_t TrainRouteNetwork::train_count() const
{
	std::size_t total = 0;
	for (const auto &service : services_)
		total += service.trains.size();
	return total;
}

std::vector<std::pair<std::string, std::string>> TrainRouteNetwork::datapack_functions(
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y) const
{
	std::vector<std::pair<std::string, std::string>> out;
	std::ostringstream network;
	network << "data modify storage arnis:trains tmp set value []\n";
	std::ostringstream all;
	std::map<std::string, std::vector<train_bodies::Part>> bodies;
	std::size_t carriage = 0;
	for (std::size_t route_index = 0; route_index < services_.size(); ++route_index) {
		const auto &service = services_[route_index];
		if (service.route >= routes_.size())
			continue;
		const auto &route = routes_[service.route];
		const std::size_t n = route.points.size();
		if (n == 0 || route.ways.size() < n)
			continue;
		std::vector<std::optional<double>> raw(n);
		for (std::size_t i = 0; i < n; ++i) {
			const auto bx = static_cast<int>(std::floor(route.points[i].x));
			const auto bz = static_cast<int>(std::floor(route.points[i].z));
			std::optional<std::pair<int, int>> best;
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					if (const auto y = rail_y(route.ways[i], bx + dx, bz + dz)) {
						const int distance = dx * dx + dz * dz;
						if (!best || distance < best->first)
							best = std::pair{distance, *y};
					}
			if (best)
				raw[i] = best->second;
		}
		const auto known = static_cast<std::size_t>(std::count_if(
				raw.begin(), raw.end(), [](const auto &v) { return v.has_value(); }));
		if (known * 2 < n)
			continue;
		auto filled = raw;
		std::optional<double> previous;
		for (auto &height : filled) {
			if (height)
				previous = height;
			else
				height = previous;
		}
		std::optional<double> next;
		for (std::size_t i = n; i-- > 0;) {
			if (raw[i])
				next = raw[i];
			else if (!filled[i])
				filled[i] = next;
		}
		std::vector<double> heights(n);
		for (std::size_t i = 0; i < n; ++i)
			heights[i] = filled[i].value_or(0.0);
		const auto unsmoothed_heights = heights;
		const bool looped = route.shape == Shape::Loop;
		for (std::size_t i = 0; i < n; ++i) {
			double sum = 0.0;
			int count = 0;
			for (int d = -3; d <= 3; ++d) {
				const auto j = static_cast<std::int64_t>(i) + d;
				if (looped) {
					const auto wrapped = static_cast<std::size_t>(
							(j % static_cast<std::int64_t>(n) +
									static_cast<std::int64_t>(n)) %
							static_cast<std::int64_t>(n));
					sum += unsmoothed_heights[wrapped];
					++count;
				} else if (j >= 0 && j < static_cast<std::int64_t>(n)) {
					sum += unsmoothed_heights[static_cast<std::size_t>(j)];
					++count;
				}
			}
			heights[i] = sum / count;
		}

		const std::size_t r = route_index;
		network << "data modify storage arnis:trains r" << r << " set value []\n";
		for (std::size_t start = 0; start < n; start += 400) {
			const auto end = std::min(n, start + 400);
			network << "data modify storage arnis:trains tmp set value [";
			for (std::size_t i = start; i < end; ++i) {
				if (i != start)
					network << ',';
				const auto &a = route.points[route.shape == Shape::Loop
													 ? i
													 : (i + 1 < n ? i : i - (i > 0))];
				const auto &b = route.points[route.shape == Shape::Loop
													 ? (i + 1) % n
													 : (i + 1 < n ? i + 1 : i)];
				const double heading =
						std::atan2(-(b.x - a.x), b.z - a.z) * 180.0 / std::numbers::pi;
				network << "{x:" << std::fixed << std::setprecision(2)
						<< ::arnis::datapack::macro_coord(route.points[i].x)
						<< "d,y:" << std::setprecision(3) << heights[i] + 0.0625
						<< "d,z:" << std::setprecision(2)
						<< ::arnis::datapack::macro_coord(route.points[i].z)
						<< "d,h:" << std::setprecision(1) << heading << "f}";
			}
			network << "]\ndata modify storage arnis:trains r" << r
					<< " append from storage arnis:trains tmp[]\n";
		}
		network << "data modify storage arnis:trains s" << r << " set value [I;";
		for (std::size_t i = 0; i < service.timetable.size(); ++i) {
			if (i)
				network << ',';
			network << service.timetable[i];
		}
		network << "]\n";
		const auto length_mm =
				static_cast<std::int64_t>(std::llround(route.length() * 1000.0));
		const auto visible = route.shape == Shape::Loop ? length_mm - 1 : length_mm;
		const auto period =
				route.shape == Shape::Loop
						? length_mm
						: (route.shape == Shape::Through ? service.period : 0);
		const double middle =
				service.offsets.empty() ? 0.0 : service.offsets.back() / 2.0;
		for (std::size_t train_index = 0; train_index < service.trains.size();
				++train_index) {
			const auto &departure = service.trains[train_index];
			const auto name = std::to_string(r) + "_" + std::to_string(train_index);
			std::ostringstream text;
			text << "# Train " << train_index << " on route " << r
				 << ": position from its timetable.\n"
				 << "scoreboard players operation #i arnis.train = #now arnis.train\n"
				 << "scoreboard players add #i arnis.train " << departure.phase << '\n'
				 << "scoreboard players set #T arnis.train " << service.timetable.size()
				 << '\n'
				 << "scoreboard players operation #i arnis.train %= #T arnis.train\n"
				 << "data modify storage arnis:trains a set value {r:" << r << "}\n"
				 << "execute store result storage arnis:trains a.i int 1 run scoreboard players get #i arnis.train\n"
				 << "function arnis:trains/when with storage arnis:trains a\n"
				 << "scoreboard players set #L arnis.train " << visible << '\n'
				 << "scoreboard players set #P arnis.train " << period << '\n'
				 << "scoreboard players operation #sk arnis.train = #s arnis.train\n"
				 << "scoreboard players remove #sk arnis.train "
				 << static_cast<std::int64_t>(std::llround(middle * 1000.0)) << '\n'
				 << "function arnis:trains/near\n"
				 << "execute unless score #go arnis.train matches 1 run return 0\n";
			for (std::size_t i = 0; i < service.coaches.size(); ++i) {
				const auto &coach = service.coaches[i];
				const auto &livery = departure.livery;
				const auto body_key = train_bodies::key(coach, livery);
				bodies.try_emplace(body_key, train_bodies::parts(coach, livery));
				const double offset =
						i < service.offsets.size() ? service.offsets[i] : 0.0;
				text << "scoreboard players operation #sk arnis.train = #s arnis.train\n"
					 << "scoreboard players remove #sk arnis.train "
					 << static_cast<std::int64_t>(std::llround(offset * 1000.0)) << '\n'
					 << "data modify storage arnis:trains c set value {r:" << r
					 << ",c:" << carriage + i << ",b:\"" << body_key << "\"}\n"
					 << "function arnis:trains/car\n";
			}
			carriage += service.coaches.size();
			all << "function arnis:trains/t/" << name << '\n';
			out.emplace_back(
					"data/arnis/function/trains/t/" + name + ".mcfunction", text.str());
		}
	}
	network << "data remove storage arnis:trains tmp\n";
	out.emplace_back("data/arnis/function/trains/network.mcfunction", network.str());
	out.emplace_back("data/arnis/function/trains/all.mcfunction", all.str());
	for (const auto &[key, parts] : bodies) {
		for (const bool bare : {false, true}) {
			const auto body = train_bodies::summon_function(parts, size_, bare);
			const std::string prefix = bare ? "ov121/" : "";
			out.emplace_back(
					prefix + "data/arnis/function/trains/body/" + key + ".mcfunction",
					body);
		}
	}
	return out;
}

std::pair<nlohmann::json, std::vector<bedrock_addon::Body>>
TrainRouteNetwork::bedrock_data(
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y) const
{
	nlohmann::json routes = nlohmann::json::array();
	nlohmann::json service_data = nlohmann::json::array();
	std::vector<bedrock_addon::Body> bodies;
	std::unordered_map<std::string, std::size_t> body_of;
	std::size_t carriage = 0;
	const float size = static_cast<float>(size_);
	for (const auto &service : services_) {
		if (service.route >= routes_.size())
			continue;
		const auto &route = routes_[service.route];
		const auto heights = bedrock_route_heights(route, rail_y);
		if (!heights)
			continue;
		std::vector<std::int64_t> points;
		points.reserve(route.points.size() * 4);
		std::array<std::int64_t, 4> previous{};
		for (std::size_t i = 0; i < route.points.size(); ++i) {
			const auto &[x, z] = route.points[i];
			const std::array<std::int64_t, 4> at{
					static_cast<std::int64_t>(std::llround(x * 100.0)),
					static_cast<std::int64_t>(
							std::llround(((*heights)[i] + 0.0625) * 1000.0)),
					static_cast<std::int64_t>(std::llround(z * 100.0)),
					static_cast<std::int64_t>(
							std::llround(bedrock_heading(route, i) * 10.0))};
			for (std::size_t axis = 0; axis < at.size(); ++axis)
				points.push_back(at[axis] - previous[axis]);
			previous = at;
		}
		if (route.points.empty())
			continue;
		double min_x = std::numeric_limits<double>::max();
		double min_z = std::numeric_limits<double>::max();
		double max_x = std::numeric_limits<double>::lowest();
		double max_z = std::numeric_limits<double>::lowest();
		for (const auto &[x, z] : route.points) {
			min_x = std::min(min_x, x);
			min_z = std::min(min_z, z);
			max_x = std::max(max_x, x);
			max_z = std::max(max_z, z);
		}
		routes.push_back(
				{{"p", bedrock_addon::list(points)}, {"loop", route.shape == Shape::Loop},
						{"box", {std::floor(min_x), std::floor(min_z), std::ceil(max_x),
										std::ceil(max_z)}}});

		const auto length_mm =
				static_cast<std::int64_t>(std::llround(route.length() * 1000.0));
		const auto visible = route.shape == Shape::Loop ? length_mm - 1 : length_mm;
		const auto period = route.shape == Shape::Loop		? length_mm
							: route.shape == Shape::Through ? service.period
															: 0;
		const double middle =
				service.offsets.empty() ? 0.0 : service.offsets.back() / 2.0;
		nlohmann::json departures = nlohmann::json::array();
		for (const auto &departure : service.trains) {
			nlohmann::json cars = nlohmann::json::array();
			for (std::size_t i = 0; i < service.coaches.size(); ++i) {
				const auto &coach = service.coaches[i];
				const std::string name =
						"train_" + train_bodies::key(coach, departure.livery);
				auto [it, inserted] = body_of.emplace(name, bodies.size());
				if (inserted) {
					bedrock_addon::Body body;
					body.name = name;
					body.fleet = bedrock_addon::Fleet::Train;
					for (const auto &part :
							train_bodies::parts(coach, departure.livery)) {
						bedrock_addon::Shape shape;
						shape.block = part.block;
						shape.size = part.size;
						shape.corner = part.corner;
						shape.tilt = part.tilt;
						shape.lamp = part.glow ? bedrock_addon::Lamp::On
											   : bedrock_addon::Lamp::Off;
						for (std::size_t axis = 0; axis < 3; ++axis) {
							shape.size[axis] *= size;
							shape.corner[axis] *= size;
						}
						body.shapes.push_back(std::move(shape));
					}
					bodies.push_back(std::move(body));
				}
				const double offset =
						i < service.offsets.size() ? service.offsets[i] : 0.0;
				cars.push_back({static_cast<std::int64_t>(carriage + i),
						static_cast<std::int64_t>(it->second),
						static_cast<std::int64_t>(std::llround(offset * 1000.0))});
			}
			carriage += service.coaches.size();
			departures.push_back({{"phase", departure.phase}, {"cars", std::move(cars)}});
		}
		service_data.push_back({{"route", routes.size() - 1},
				{"steps", bedrock_addon::list(bedrock_run_lengths(service.timetable))},
				{"count", service.timetable.size()}, {"visible", visible},
				{"period", period},
				{"middle", static_cast<std::int64_t>(std::llround(middle * 1000.0))},
				{"trains", std::move(departures)}});
	}
	nlohmann::json names = nlohmann::json::array();
	for (const auto &body : bodies)
		names.push_back(body.name);
	return {{{"routes", std::move(routes)}, {"services", std::move(service_data)},
					{"bodies", std::move(names)}},
			std::move(bodies)};
}

} // namespace arnis::trains
