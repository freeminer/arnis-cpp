#pragma once
#include "types.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iterator>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
namespace arnis::mapillary
{
inline Frame build_frame(const BBox &bbox)
{
	const auto centre = bbox.centre();
	return Frame(centre[0], centre[1]);
}

inline std::array<std::array<double, 2>, 4> bbox_polygon_xy(
		const BBox &bbox, const Frame &frame, double margin_m)
{
	const auto lo = frame.to_enu(bbox.min_lon, bbox.min_lat);
	const auto hi = frame.to_enu(bbox.max_lon, bbox.max_lat);
	return {{{lo[0] - margin_m, lo[1] - margin_m}, {hi[0] + margin_m, lo[1] - margin_m},
			{hi[0] + margin_m, hi[1] + margin_m}, {lo[0] - margin_m, hi[1] + margin_m}}};
}

inline std::optional<std::array<double, 2>> wall_tangent(const Wall &w)
{
	double x = w.b[0] - w.a[0], y = w.b[1] - w.a[1], n = std::hypot(x, y);
	return n > 1e-9 ? std::optional<std::array<double, 2>>{{x / n, y / n}} : std::nullopt;
}
inline std::optional<std::array<double, 2>> wall_outward(const Wall &w)
{
	auto t = wall_tangent(w);
	return t ? std::optional<std::array<double, 2>>{{(*t)[1], -(*t)[0]}} : std::nullopt;
}
inline std::array<double, 3> wall_point(const Wall &w, double s, double h)
{
	auto t = wall_tangent(w).value_or(std::array<double, 2>{0, 0});
	return {w.a[0] + t[0] * s, w.a[1] + t[1] * s, h};
}
inline bool faces_camera(const Wall &w, const Camera &c)
{
	auto n = wall_outward(w);
	if (!n)
		return false;
	return (c.centre[0] - w.a[0]) * (*n)[0] + (c.centre[1] - w.a[1]) * (*n)[1] > 0;
}
inline std::optional<std::array<double, 2>> project_wall_point(
		const Camera &c, const Wall &w, double s, double h)
{
	return faces_camera(w, c) ? project(c, wall_point(w, s, h)) : std::nullopt;
}

// WGS84 conversion used by Rust when OpenSfM's topocentric cloud coordinates
// are brought into the map frame.  Keeping this ellipsoidal path avoids the
// metre-scale drift caused by using a spherical shortcut for only the cloud.
namespace geodesy
{
inline constexpr double wgs84_a = 6378137.0;
inline constexpr double wgs84_e2 = 6.69437999014e-3;
inline std::array<double, 3> lla_to_ecef(
		double longitude, double latitude, double altitude)
{
	double lon = longitude * .017453292519943295769,
		   lat = latitude * .017453292519943295769;
	double sl = std::sin(lat), cl = std::cos(lat);
	double n = wgs84_a / std::sqrt(1 - wgs84_e2 * sl * sl);
	return {(n + altitude) * cl * std::cos(lon), (n + altitude) * cl * std::sin(lon),
			(n * (1 - wgs84_e2) + altitude) * sl};
}
inline std::array<double, 3> ecef_to_lla(const std::array<double, 3> &point)
{
	double longitude = std::atan2(point[1], point[0]);
	double horizontal = std::hypot(point[0], point[1]);
	double latitude = std::atan2(point[2], horizontal * (1 - wgs84_e2)), altitude = 0;
	for (unsigned i = 0; i < 6; ++i) {
		double sl = std::sin(latitude), n = wgs84_a / std::sqrt(1 - wgs84_e2 * sl * sl);
		altitude = horizontal / std::cos(latitude) - n;
		latitude = std::atan2(point[2], horizontal * (1 - wgs84_e2 * n / (n + altitude)));
	}
	return {longitude * 57.295779513082320876, latitude * 57.295779513082320876,
			altitude};
}
inline std::array<std::array<double, 3>, 3> enu_rows(double longitude, double latitude)
{
	double lon = longitude * .017453292519943295769,
		   lat = latitude * .017453292519943295769;
	double slon = std::sin(lon), clon = std::cos(lon), slat = std::sin(lat),
		   clat = std::cos(lat);
	return {{{-slon, clon, 0}, {-slat * clon, -slat * slon, clat},
			{clat * clon, clat * slon, slat}}};
}
inline std::array<double, 3> topocentric_to_lla(
		const std::array<double, 3> &point, const std::array<double, 3> &reference_lla)
{
	auto ecef = lla_to_ecef(reference_lla[0], reference_lla[1], reference_lla[2]);
	auto rows = enu_rows(reference_lla[0], reference_lla[1]);
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			ecef[j] += point[i] * rows[i][j];
	return ecef_to_lla(ecef);
}
inline std::array<double, 3> lla_to_topocentric(double longitude, double latitude,
		double altitude, const std::array<double, 3> &reference_lla)
{
	auto origin = lla_to_ecef(reference_lla[0], reference_lla[1], reference_lla[2]);
	auto ecef = lla_to_ecef(longitude, latitude, altitude);
	auto rows = enu_rows(reference_lla[0], reference_lla[1]);
	std::array<double, 3> result{};
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			result[i] += rows[i][j] * (ecef[j] - origin[j]);
	return result;
}

inline std::array<double, 3> topocentric_to_run(const std::array<double, 3> &point,
		const std::array<double, 3> &reference_lla, const Frame &frame)
{
	const auto lla = topocentric_to_lla(point, reference_lla);
	const auto xy = frame.to_enu(lla[0], lla[1]);
	return {xy[0], xy[1], point[2]};
}
inline std::optional<double> parse_length_m(const std::string &text)
{
	std::size_t begin = 0, end = text.size();
	while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])))
		++begin;
	while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
		--end;
	std::size_t split = begin;
	while (split < end && (std::isdigit(static_cast<unsigned char>(text[split])) ||
								  text[split] == '.' || text[split] == ',' ||
								  text[split] == '+' || text[split] == '-'))
		++split;
	std::string number = text.substr(begin, split - begin);
	std::replace(number.begin(), number.end(), ',', '.');
	if (number.empty() || std::count(number.begin(), number.end(), '.') > 1)
		return {};
	char *tail = nullptr;
	double value = std::strtod(number.c_str(), &tail);
	if (!tail || *tail)
		return {};
	std::string unit = text.substr(split, end - split);
	while (!unit.empty() && std::isspace(static_cast<unsigned char>(unit.front())))
		unit.erase(unit.begin());
	for (char &c : unit)
		c = char(std::tolower(static_cast<unsigned char>(c)));
	if (unit.empty() || unit == "m" || unit == "meter" || unit == "meters" ||
			unit == "metre" || unit == "metres")
		return value;
	if (unit == "ft" || unit == "feet" || unit == "foot" || unit == "'")
		return value * .3048;
	return {};
}
} // namespace geodesy

// Ring helpers used by the footprint normalizer.  They intentionally operate
// on the same lightweight arrays as the Rust geometry module, keeping node ids
// aligned when a clockwise OSM ring is reversed.
inline double signed_area(const std::vector<std::array<double, 2>> &ring)
{
	if (ring.empty())
		return 0.0;
	double area = 0.0;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[(i + 1) % ring.size()];
		area += a[0] * b[1] - b[0] * a[1];
	}
	return area * 0.5;
}

inline void ensure_ccw(
		std::vector<std::array<double, 2>> &ring, std::vector<std::int64_t> &node_ids)
{
	if (signed_area(ring) >= 0.0)
		return;
	// Preserve the first OSM vertex, exactly as ensure_ccw in geometry.rs.
	if (ring.size() > 1)
		std::reverse(ring.begin() + 1, ring.end());
	if (node_ids.size() == ring.size() && node_ids.size() > 1)
		std::reverse(node_ids.begin() + 1, node_ids.end());
}

inline std::array<double, 2> outward_normal(
		const std::array<double, 2> &t, bool ccw = true)
{
	return ccw ? std::array<double, 2>{t[1], -t[0]} : std::array<double, 2>{-t[1], t[0]};
}

inline double ring_distance_squared(
		const std::array<double, 2> &a, const std::array<double, 2> &b)
{
	const double dx = a[0] - b[0], dy = a[1] - b[1];
	return dx * dx + dy * dy;
}

// Drops a repeated closing vertex and consecutive duplicates while retaining
// the corresponding OSM node IDs.  The 1 mm tolerance is the same tolerance
// used by geometry.rs and avoids zero-length wall edges after projection.
inline std::pair<std::vector<std::array<double, 2>>, std::vector<std::int64_t>>
dedupe_ring(const std::vector<std::array<double, 2>> &ring,
		const std::vector<std::int64_t> &node_ids)
{
	constexpr double tolerance_squared = 1e-12;
	if (ring.empty())
		return {};
	std::size_t count = ring.size();
	if (count > 1 && ring_distance_squared(ring.front(), ring.back()) < tolerance_squared)
		--count;
	std::vector<std::array<double, 2>> points;
	std::vector<std::int64_t> ids;
	points.reserve(count);
	ids.reserve(count);
	for (std::size_t i = 0; i < count; ++i) {
		if (!points.empty() &&
				ring_distance_squared(points.back(), ring[i]) < tolerance_squared)
			continue;
		points.push_back(ring[i]);
		ids.push_back(i < node_ids.size() ? node_ids[i] : -1);
	}
	if (points.size() > 1 &&
			ring_distance_squared(points.front(), points.back()) < tolerance_squared) {
		points.pop_back();
		ids.pop_back();
	}
	return {std::move(points), std::move(ids)};
}

inline double ring_orientation(const std::array<double, 2> &a,
		const std::array<double, 2> &b, const std::array<double, 2> &c)
{
	return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

inline bool ring_segments_cross(const std::array<double, 2> &a1,
		const std::array<double, 2> &a2, const std::array<double, 2> &b1,
		const std::array<double, 2> &b2)
{
	constexpr double epsilon = 1e-9;
	const double ab = ring_orientation(a1, a2, b1);
	const double ac = ring_orientation(a1, a2, b2);
	const double cd = ring_orientation(b1, b2, a1);
	const double ce = ring_orientation(b1, b2, a2);
	return ((ab > epsilon && ac < -epsilon) || (ab < -epsilon && ac > epsilon)) &&
		   ((cd > epsilon && ce < -epsilon) || (cd < -epsilon && ce > epsilon));
}

// Self-intersection test for a simple polygon. Adjacent edges are excluded;
// their shared endpoint is expected and is not a footprint crossing.
inline bool ring_is_simple(const std::vector<std::array<double, 2>> &ring)
{
	if (ring.size() < 3)
		return false;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto i_next = (i + 1) % ring.size();
		for (std::size_t j = i + 1; j < ring.size(); ++j) {
			const auto j_next = (j + 1) % ring.size();
			if (i == j || i_next == j || j_next == i)
				continue;
			if (ring_segments_cross(ring[i], ring[i_next], ring[j], ring[j_next]))
				return false;
		}
	}
	return true;
}

inline std::array<double, 2> ring_centroid(const std::vector<std::array<double, 2>> &ring)
{
	if (ring.empty())
		return {0.0, 0.0};
	double area_twice = 0.0, cx = 0.0, cy = 0.0;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i], &b = ring[(i + 1) % ring.size()];
		const double cross = a[0] * b[1] - b[0] * a[1];
		area_twice += cross;
		cx += (a[0] + b[0]) * cross;
		cy += (a[1] + b[1]) * cross;
	}
	if (std::abs(area_twice) < 1e-12)
		return ring.front();
	return {cx / (3.0 * area_twice), cy / (3.0 * area_twice)};
}

// Joins fragmented Overpass way members by shared endpoint IDs.  Each output
// vector retains the original direction where possible and is closed when a
// complete ring was assembled, matching geometry.rs::join_ways.
inline std::vector<std::vector<std::int64_t>> join_way_segments(
		const std::vector<std::vector<std::int64_t>> &segments)
{
	std::vector<std::vector<std::int64_t>> remaining;
	for (const auto &segment : segments)
		if (segment.size() >= 2)
			remaining.push_back(segment);
	std::vector<std::vector<std::int64_t>> rings;
	while (!remaining.empty()) {
		std::vector<std::int64_t> current = std::move(remaining.back());
		remaining.pop_back();
		bool changed = true;
		while (changed && current.front() != current.back()) {
			changed = false;
			for (std::size_t i = 0; i < remaining.size(); ++i) {
				auto &piece = remaining[i];
				const auto first = current.front(), last = current.back();
				if (piece.front() == last) {
					current.insert(current.end(), piece.begin() + 1, piece.end());
				} else if (piece.back() == last) {
					for (auto it = piece.rbegin() + 1; it != piece.rend(); ++it)
						current.push_back(*it);
				} else if (piece.back() == first) {
					std::vector<std::int64_t> joined(piece.begin(), piece.end() - 1);
					joined.insert(joined.end(), current.begin(), current.end());
					current = std::move(joined);
				} else if (piece.front() == first) {
					std::vector<std::int64_t> joined(piece.rbegin(), piece.rend() - 1);
					joined.insert(joined.end(), current.begin(), current.end());
					current = std::move(joined);
				} else {
					continue;
				}
				remaining.erase(remaining.begin() + i);
				changed = true;
				break;
			}
		}
		if (current.size() >= 4 && current.front() == current.back())
			rings.push_back(std::move(current));
	}
	return rings;
}

struct WallParams
{
	double merge_deg{3.0};
	double min_wall_m{2.0};
	double split_wall_m{32.0};
};

inline std::vector<std::vector<std::size_t>> merge_edges(
		const std::vector<std::array<double, 2>> &ring, double merge_deg)
{
	if (ring.empty())
		return {};
	const auto n = ring.size();
	std::vector<std::array<double, 2>> dirs(n);
	for (std::size_t i = 0; i < n; ++i) {
		const auto &a = ring[i], &b = ring[(i + 1) % n];
		const double length = std::max(1e-12, std::hypot(b[0] - a[0], b[1] - a[1]));
		dirs[i] = {(b[0] - a[0]) / length, (b[1] - a[1]) / length};
	}
	const double threshold = std::cos(merge_deg * 0.017453292519943295769);
	auto dot = [](const auto &a, const auto &b) { return a[0] * b[0] + a[1] * b[1]; };
	std::size_t start = 0;
	for (std::size_t i = 0; i < n; ++i)
		if (dot(dirs[i], dirs[(i + n - 1) % n]) < threshold) {
			start = i;
			break;
		}
	std::vector<std::vector<std::size_t>> groups;
	std::vector<std::size_t> current{start};
	for (std::size_t k = 1; k < n; ++k) {
		const auto i = (start + k) % n;
		if (dot(dirs[i], dirs[current.back()]) >= threshold &&
				dot(dirs[i], dirs[current.front()]) >= threshold)
			current.push_back(i);
		else {
			groups.push_back(std::move(current));
			current = {i};
		}
	}
	groups.push_back(std::move(current));
	return groups;
}

inline std::vector<Wall> walls_from_building(
		const Building &building, const WallParams &params = {})
{
	const auto &ring = building.ring;
	if (ring.size() < 3)
		return {};
	const auto ids = building.node_ids.size() == ring.size()
							 ? building.node_ids
							 : std::vector<std::int64_t>(ring.size(), -1);
	const bool ccw = signed_area(ring) > 0.0;
	std::vector<Wall> result;
	std::size_t wall_index = 0;
	for (const auto &group : merge_edges(ring, params.merge_deg)) {
		const auto first = group.front();
		const auto last = (group.back() + 1) % ring.size();
		const auto a = ring[first], endpoint = ring[last];
		const double length = std::hypot(endpoint[0] - a[0], endpoint[1] - a[1]);
		if (length < params.min_wall_m)
			continue;
		const std::array<double, 2> tangent{
				(endpoint[0] - a[0]) / length, (endpoint[1] - a[1]) / length};
		const auto normal = outward_normal(tangent, ccw);
		std::vector<WallEdge> edges;
		for (const auto edge : group) {
			const auto p0 = ring[edge], p1 = ring[(edge + 1) % ring.size()];
			edges.push_back({edge, ids[edge], ids[(edge + 1) % ring.size()],
					(p0[0] - a[0]) * tangent[0] + (p0[1] - a[1]) * tangent[1],
					(p1[0] - a[0]) * tangent[0] + (p1[1] - a[1]) * tangent[1]});
		}
		const std::size_t pieces = length > params.split_wall_m
										   ? static_cast<std::size_t>(std::ceil(
													 length / params.split_wall_m))
										   : 1;
		const double piece_length = length / pieces;
		for (std::size_t piece = 0; piece < pieces; ++piece) {
			const double s0 = piece * piece_length, s1 = (piece + 1) * piece_length;
			Wall wall;
			wall.key = wall_key(building.key, wall_index, piece, pieces);
			wall.building_key = building.key;
			wall.index = wall_index;
			wall.node_a = ids[first];
			wall.node_b = ids[last];
			wall.a = {a[0] + s0 * tangent[0], a[1] + s0 * tangent[1]};
			wall.b = {a[0] + s1 * tangent[0], a[1] + s1 * tangent[1]};
			wall.normal = normal;
			wall.length = s1 - s0;
			wall.merged_indices = group;
			wall.piece = piece;
			wall.pieces = pieces;
			wall.height_osm = building.height_osm;
			wall.height_source = building.height_source;
			wall.edges = edges;
			for (auto &edge : wall.edges) {
				edge.s0 -= s0;
				edge.s1 -= s0;
			}
			wall.s_offset = s0;
			result.push_back(std::move(wall));
		}
		++wall_index;
	}
	return result;
}

inline std::vector<Wall> walls_from_building(
		const Building &building, const Params &params)
{
	return walls_from_building(building,
			WallParams{params.merge_deg, params.min_wall_m, params.split_wall_m});
}

inline constexpr double WALL_MARGIN_M = 5.0;

inline std::vector<Wall> walls_from_buildings(const std::vector<Building> &buildings,
		const Params &params, const Frame &frame,
		const std::optional<BBox> &bbox = std::nullopt)
{
	std::array<double, 2> lower{}, upper{};
	const bool restrict = bbox.has_value();
	if (restrict) {
		const auto polygon = bbox_polygon_xy(*bbox, frame, WALL_MARGIN_M);
		lower = polygon[0];
		upper = polygon[2];
	}
	std::vector<Wall> result;
	for (const auto &building : buildings) {
		if (restrict) {
			if (building.ring.empty())
				continue;
			auto min_x = building.ring.front()[0], max_x = min_x;
			auto min_y = building.ring.front()[1], max_y = min_y;
			for (const auto &point : building.ring) {
				min_x = std::min(min_x, point[0]);
				max_x = std::max(max_x, point[0]);
				min_y = std::min(min_y, point[1]);
				max_y = std::max(max_y, point[1]);
			}
			const bool intersects = max_x >= lower[0] && min_x <= upper[0] &&
									max_y >= lower[1] && min_y <= upper[1];
			if (!intersects)
				continue;
		}
		auto walls = walls_from_building(building, params);
		result.insert(result.end(), std::make_move_iterator(walls.begin()),
				std::make_move_iterator(walls.end()));
	}
	return result;
}

inline std::tuple<std::optional<double>, HeightSource, double> height_from_tags(
		const std::map<std::string, std::string> &tags, double metres_per_level = 3.0)
{
	std::optional<double> height;
	HeightSource source = HeightSource::Default;
	if (const auto it = tags.find("height"); it != tags.end()) {
		if (const auto parsed = geodesy::parse_length_m(it->second);
				parsed && *parsed > 1.0 && *parsed < 400.0) {
			height = *parsed;
			source = HeightSource::Tag;
		}
	}
	if (!height) {
		if (const auto it = tags.find("building:levels"); it != tags.end()) {
			try {
				std::string levels_text = it->second;
				std::replace(levels_text.begin(), levels_text.end(), ',', '.');
				double levels = std::stod(levels_text);
				if (levels >= 1.0 && levels < 150.0) {
					double roof = 0.0;
					if (const auto r = tags.find("roof:levels"); r != tags.end()) {
						std::string roof_text = r->second;
						std::replace(roof_text.begin(), roof_text.end(), ',', '.');
						roof = std::max(0.0, std::stod(roof_text));
					}
					height = (levels + roof) * metres_per_level;
					source = HeightSource::Levels;
				}
			} catch (...) {
			}
		}
	}
	double min_height = 0.0;
	if (const auto it = tags.find("min_height"); it != tags.end()) {
		if (const auto parsed = geodesy::parse_length_m(it->second);
				parsed && *parsed >= 0.0)
			min_height = *parsed;
	} else if (const auto it = tags.find("building:min_level"); it != tags.end()) {
		try {
			std::string level_text = it->second;
			std::replace(level_text.begin(), level_text.end(), ',', '.');
			min_height = std::max(0.0, std::stod(level_text) * metres_per_level);
		} catch (...) {
		}
	}
	return std::make_tuple(height, source, min_height);
}

inline std::tuple<std::optional<double>, HeightSource, double> height_from_tags(
		const std::map<std::string, std::string> &tags, const Params &params)
{
	return height_from_tags(tags, params.metres_per_level);
}

// Parse Overpass `out body; >; out skel qt;` JSON into facade footprints.
// The implementation lives in geometry.cpp so users of the header do not
// acquire a JSON parser dependency unless they use this entry point.
std::vector<Building> parse_overpass(const std::string &json, const Frame &frame,
		const Params &params, const std::optional<BBox> &target_bbox = std::nullopt);
}
