#include "bridges.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <string>
#include <limits>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../bresenham.h"
#include "../strict_parse.h"
#include "bridge_modules.h"

namespace arnis::railways
{
bool is_at_grade_track(const ProcessedWay &way);
}

namespace arnis::bridges
{

namespace
{
constexpr int LAYER_HEIGHT_STEP = 6;
constexpr int FLAT_TERRAIN_DIP_THRESHOLD = 4;
constexpr std::size_t SHORT_BRIDGE_LENGTH_BLOCKS = 30;
constexpr int BRIDGE_NAME_FUSE_DISTANCE_BLOCKS = 200;
constexpr float DUAL_CARRIAGEWAY_MAX_DISTANCE_BLOCKS = 12.0f;
constexpr float DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG = 20.0f;
constexpr int ROAD_HEADROOM = 6;
constexpr int PATH_HEADROOM = 5;
constexpr int RAIL_HEADROOM = 8;
constexpr int RIVER_HEADROOM = 4;
constexpr int STREAM_HEADROOM = 2;
constexpr int STACKED_DECK_HEADROOM = 6;
constexpr int OBSTACLE_GRID_CELL = 64;

struct XZPairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
									  static_cast<unsigned long long>(p.second));
	}
};

int effective_layer(const ProcessedWay &way);
int highway_block_range(const ProcessedWay &way, double scale);
bool is_non_vehicular_bridge_highway(const ProcessedWay &way);

struct GradeSegment
{
	float ax = 0.0f;
	float az = 0.0f;
	float bx = 0.0f;
	float bz = 0.0f;
	float reach = 0.0f;
	int headroom = 0;
	std::size_t way = 0;
};

int div_euclid(int value, int divisor)
{
	int quotient = value / divisor;
	if (value % divisor < 0)
		--quotient;
	return quotient;
}

std::optional<std::pair<int, int>> grade_obstacle(const ProcessedWay &way, double scale)
{
	if (way.nodes.size() < 2 || way.tags.get("area") == "yes" ||
			way.tags.get("indoor") == "yes")
		return std::nullopt;
	const auto highway = way.tags.get("highway");
	if (!highway.empty()) {
		const auto level = way.tags.get("level");
		const auto parsed_level = strict_parse::i32(level);
		const bool below_ground =
				way.tags.get("tunnel") == "yes" || (parsed_level && *parsed_level < 0);
		const auto excluded = highway == "street_lamp" || highway == "crossing" ||
							  highway == "bus_stop" || highway == "proposed" ||
							  highway == "construction" || highway == "razed" ||
							  highway == "abandoned" || highway == "elevator" ||
							  highway == "platform" || highway == "corridor";
		if (is_bridge_way(way) || below_ground || effective_layer(way) > 0 || excluded)
			return std::nullopt;
		const auto headroom = is_non_vehicular_bridge_highway(way) || highway == "track"
									  ? PATH_HEADROOM
									  : ROAD_HEADROOM;
		return std::pair{highway_block_range(way, scale), headroom};
	}

	const auto railway = way.tags.get("railway");
	if (railways::is_at_grade_track(way)) {
		const int headroom = railway == "tram"		  ? ROAD_HEADROOM
							 : railway == "miniature" ? PATH_HEADROOM
													  : RAIL_HEADROOM;
		return std::pair{1, headroom};
	}
	return std::nullopt;
}

std::optional<int> waterway_clearance(const ProcessedWay &way)
{
	if (way.nodes.size() < 2 || way.tags.get("area") == "yes")
		return std::nullopt;
	const auto tunnel = way.tags.get("tunnel");
	if (!tunnel.empty() && tunnel != "no")
		return std::nullopt;
	const auto waterway = way.tags.get("waterway");
	if (waterway == "river" || waterway == "canal")
		return RIVER_HEADROOM;
	if (waterway == "stream")
		return STREAM_HEADROOM;
	return std::nullopt;
}

float point_segment_distance(float px, float pz, const GradeSegment &segment)
{
	const float dx = segment.bx - segment.ax;
	const float dz = segment.bz - segment.az;
	const float length_squared = dx * dx + dz * dz;
	const float t =
			length_squared > 0.0f
					? std::clamp(((px - segment.ax) * dx + (pz - segment.az) * dz) /
										 length_squared,
							  0.0f, 1.0f)
					: 0.0f;
	const float ox = px - (segment.ax + t * dx);
	const float oz = pz - (segment.az + t * dz);
	return std::sqrt(ox * ox + oz * oz);
}

bool crosses_segment(float hx, float hz, const GradeSegment &segment)
{
	const float sx = segment.bx - segment.ax;
	const float sz = segment.bz - segment.az;
	const float norms = std::sqrt(hx * hx + hz * hz) * std::sqrt(sx * sx + sz * sz);
	return norms <= 0.0f || std::abs(hx * sx + hz * sz) / norms < 0.866f;
}

struct GradeObstacleIndex
{
	std::vector<const ProcessedWay *> ways;
	std::vector<GradeSegment> segments;
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash> buckets;

	void build(const std::vector<ProcessedElement> &elements, double scale, int min_x,
			int min_z, int max_x, int max_z)
	{
		for (const auto &element : elements) {
			if (!element.is_way())
				continue;
			const auto &way = element.as_way();
			const auto obstacle = grade_obstacle(way, scale);
			const auto water_clearance = waterway_clearance(way);
			if (!obstacle && !water_clearance)
				continue;
			const int half_width = obstacle ? obstacle->first : 0;
			const float reach = static_cast<float>(half_width) + 0.5f;
			const int padding = half_width + 1;
			const int headroom = obstacle ? obstacle->second : *water_clearance;
			std::optional<std::size_t> way_index;
			for (std::size_t i = 1; i < way.nodes.size(); ++i) {
				const auto &a = way.nodes[i - 1];
				const auto &b = way.nodes[i];
				const int x0 = std::min(a.x, b.x) - padding;
				const int x1 = std::max(a.x, b.x) + padding;
				const int z0 = std::min(a.z, b.z) - padding;
				const int z1 = std::max(a.z, b.z) + padding;
				if (x1 < min_x || x0 > max_x || z1 < min_z || z0 > max_z)
					continue;
				if (!way_index) {
					ways.push_back(&way);
					way_index = ways.size() - 1;
				}
				const auto segment_index = segments.size();
				segments.push_back({static_cast<float>(a.x), static_cast<float>(a.z),
						static_cast<float>(b.x), static_cast<float>(b.z), reach, headroom,
						*way_index});
				for (int bx = div_euclid(x0, OBSTACLE_GRID_CELL);
						bx <= div_euclid(x1, OBSTACLE_GRID_CELL); ++bx) {
					for (int bz = div_euclid(z0, OBSTACLE_GRID_CELL);
							bz <= div_euclid(z1, OBSTACLE_GRID_CELL); ++bz)
						buckets[{bx, bz}].push_back(segment_index);
				}
			}
		}
	}

	const std::vector<std::size_t> *at(int x, int z) const
	{
		const auto it = buckets.find(
				{div_euclid(x, OBSTACLE_GRID_CELL), div_euclid(z, OBSTACLE_GRID_CELL)});
		return it == buckets.end() ? nullptr : &it->second;
	}
};

class UnionFind
{
public:
	explicit UnionFind(std::size_t n) : parent_(n), rank_(n, 0)
	{
		for (std::size_t i = 0; i < n; ++i)
			parent_[i] = i;
	}

	std::size_t find(std::size_t i)
	{
		while (parent_[i] != i) {
			parent_[i] = parent_[parent_[i]];
			i = parent_[i];
		}
		return i;
	}

	void unite(std::size_t a, std::size_t b)
	{
		const auto ra = find(a);
		const auto rb = find(b);
		if (ra == rb)
			return;
		if (rank_[ra] < rank_[rb]) {
			parent_[ra] = rb;
		} else if (rank_[ra] > rank_[rb]) {
			parent_[rb] = ra;
		} else {
			parent_[rb] = ra;
			++rank_[ra];
		}
	}

private:
	std::vector<std::size_t> parent_;
	std::vector<unsigned char> rank_;
};

int effective_layer(const ProcessedWay &way)
{
	const auto it = way.tags.find("layer");
	if (it == way.tags.end())
		return is_bridge_way(way) ? 1 : 0;
	try {
		return std::max(0, std::stoi(it->second));
	} catch (...) {
		return is_bridge_way(way) ? 1 : 0;
	}
}

int highway_block_range(const ProcessedWay &way, double scale)
{
	const auto h = way.tags.get("highway");
	int block_range = 2;
	if (h == "footway" || h == "pedestrian" || h == "path" || h == "cycleway" ||
			h == "track" || h == "secondary_link" || h == "tertiary_link" ||
			h == "escape" || h == "steps") {
		block_range = 1;
	} else if (h == "motorway" || h == "primary" || h == "trunk") {
		block_range = 5;
	} else if (h == "secondary") {
		block_range = 4;
	} else if (h == "tertiary") {
		block_range = 2;
	} else if (h == "service") {
		block_range = 2;
	} else if (const auto it = way.tags.find("lanes"); it != way.tags.end()) {
		if (it->second == "2")
			block_range = 3;
		else if (it->second != "1")
			block_range = 4;
	}
	const int default_lanes = h == "motorway" || h == "primary" || h == "trunk" ||
											  h == "secondary" || h == "tertiary"
									  ? 2
									  : 1;
	int lanes = default_lanes;
	if (const auto lanes_it = way.tags.find("lanes"); lanes_it != way.tags.end())
		lanes = strict_parse::i32(lanes_it->second).value_or(default_lanes);
	lanes = std::clamp(lanes, 1, 16);
	if (const auto width_it = way.tags.find("width"); width_it != way.tags.end()) {
		std::string_view width = width_it->second;
		while (!width.empty() && std::isspace(static_cast<unsigned char>(width.back())))
			width.remove_suffix(1);
		while (!width.empty() && width.back() == 'm')
			width.remove_suffix(1);
		while (!width.empty() && std::isspace(static_cast<unsigned char>(width.back())))
			width.remove_suffix(1);
		while (!width.empty() && std::isspace(static_cast<unsigned char>(width.front())))
			width.remove_prefix(1);
		if (const auto metres = strict_parse::f64(width);
				metres && std::isfinite(*metres) && *metres > 0.0) {
			const double rounded = std::round(*metres / 2.0);
			block_range = rounded > 8.0 ? 9 : static_cast<int>(rounded);
		}
	} else if (h != "footway" && h != "pedestrian" && h != "path" && h != "cycleway" &&
			   h != "track" && h != "escape" && h != "steps") {
		block_range = std::max(
				block_range, static_cast<int>(std::round((lanes * 3.5 - 1.0) / 2.0)));
	}
	block_range = std::clamp(block_range, 1, 8);
	if (scale < 1.0)
		block_range = std::max(1, static_cast<int>(std::floor(block_range * scale)));
	return block_range;
}

bool is_ramp_candidate(const ProcessedWay &way)
{
	if (is_bridge_way(way))
		return false;
	if (way.tags.get("indoor") == "yes")
		return false;
	if (const auto it = way.tags.find("embankment");
			it != way.tags.end() && it->second != "no")
		return true;
	if (way.tags.get("man_made") == "embankment")
		return true;
	if (const auto it = way.tags.find("layer"); it != way.tags.end()) {
		try {
			return std::stoi(it->second) >= 1;
		} catch (...) {
			return false;
		}
	}
	return false;
}

bool is_oneway(const ProcessedWay &way)
{
	const auto oneway = way.tags.get("oneway");
	return oneway == "yes" || oneway == "-1" || oneway == "true";
}

std::size_t way_length_blocks(const ProcessedWay &way)
{
	std::size_t total = 0;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const float dx = static_cast<float>(b.x - a.x);
		const float dz = static_cast<float>(b.z - a.z);
		total += static_cast<std::size_t>(std::sqrt(dx * dx + dz * dz));
	}
	return total;
}

bool is_non_vehicular_bridge_highway(const ProcessedWay &way)
{
	const auto highway = way.tags.get("highway");
	return highway == "footway" || highway == "pedestrian" || highway == "path" ||
		   highway == "cycleway" || highway == "bridleway" || highway == "steps";
}

std::vector<std::pair<float, float>> coverage_samples(const ProcessedWay &way)
{
	if (way.nodes.size() < 2) {
		std::vector<std::pair<float, float>> points;
		for (const auto &node : way.nodes)
			points.emplace_back(static_cast<float>(node.x), static_cast<float>(node.z));
		return points;
	}
	std::vector<float> cumulative(way.nodes.size(), 0.0f);
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const float dx = static_cast<float>(way.nodes[i].x - way.nodes[i - 1].x);
		const float dz = static_cast<float>(way.nodes[i].z - way.nodes[i - 1].z);
		cumulative[i] = cumulative[i - 1] + std::sqrt(dx * dx + dz * dz);
	}
	std::vector<std::pair<float, float>> points;
	points.reserve(5);
	for (const float fraction : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
		const float target = cumulative.back() * fraction;
		std::size_t segment = 0;
		while (segment + 1 < cumulative.size() - 1 && cumulative[segment + 1] < target)
			++segment;
		const float length =
				std::max(1e-6f, cumulative[segment + 1] - cumulative[segment]);
		const float t = std::clamp((target - cumulative[segment]) / length, 0.0f, 1.0f);
		const auto &a = way.nodes[segment];
		const auto &b = way.nodes[segment + 1];
		points.emplace_back(a.x + t * static_cast<float>(b.x - a.x),
				a.z + t * static_cast<float>(b.z - a.z));
	}
	return points;
}

bool is_monotonic(const std::vector<float> &values, float tolerance)
{
	if (values.size() < 2)
		return true;
	const bool rising = values.back() >= values.front();
	float extreme = values.front();
	for (const float value : values) {
		if (rising) {
			if (value < extreme - tolerance)
				return false;
			extreme = std::max(extreme, value);
		} else {
			if (value > extreme + tolerance)
				return false;
			extreme = std::min(extreme, value);
		}
	}
	return true;
}

std::vector<std::pair<float, float>> upper_hull(
		std::vector<std::pair<float, float>> points)
{
	std::sort(points.begin(), points.end(), [](const auto &a, const auto &b) {
		return a.first < b.first || (a.first == b.first && a.second > b.second);
	});
	std::vector<std::pair<float, float>> hull;
	for (const auto &point : points) {
		if (!hull.empty() && std::abs(hull.back().first - point.first) < 1e-4f)
			continue;
		while (hull.size() >= 2) {
			const auto &origin = hull[hull.size() - 2];
			const auto &last = hull.back();
			const float cross =
					(last.first - origin.first) * (point.second - origin.second) -
					(last.second - origin.second) * (point.first - origin.first);
			if (cross >= 0.0f)
				hull.pop_back();
			else
				break;
		}
		hull.push_back(point);
	}
	return hull;
}

float hull_at(const std::vector<std::pair<float, float>> &hull, float u)
{
	if (hull.empty())
		return 0.0f;
	if (hull.size() == 1 || u <= hull.front().first)
		return hull.front().second;
	if (u >= hull.back().first)
		return hull.back().second;
	const auto upper = std::upper_bound(hull.begin(), hull.end(), u,
			[](float value, const auto &point) { return value < point.first; });
	const auto &b = *upper;
	const auto &a = *(upper - 1);
	return a.second +
		   (b.second - a.second) * (u - a.first) / std::max(1e-6f, b.first - a.first);
}

float lateral_offset_to_way(float px, float pz, const ProcessedWay &way)
{
	float best = std::numeric_limits<float>::max();
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const float ax = static_cast<float>(way.nodes[i - 1].x);
		const float az = static_cast<float>(way.nodes[i - 1].z);
		const float dx = static_cast<float>(way.nodes[i].x - way.nodes[i - 1].x);
		const float dz = static_cast<float>(way.nodes[i].z - way.nodes[i - 1].z);
		const float length_squared = dx * dx + dz * dz;
		const float t =
				length_squared > 0.0f
						? std::clamp(((px - ax) * dx + (pz - az) * dz) / length_squared,
								  0.0f, 1.0f)
						: 0.0f;
		const float ox = px - (ax + t * dx);
		const float oz = pz - (az + t * dz);
		best = std::min(best, std::sqrt(ox * ox + oz * oz));
	}
	return best;
}

std::pair<int, int> centroid(const ProcessedWay &way)
{
	if (way.nodes.empty())
		return {0, 0};
	long long sx = 0;
	long long sz = 0;
	for (const auto &node : way.nodes) {
		sx += node.x;
		sz += node.z;
	}
	const auto n = static_cast<long long>(way.nodes.size());
	return {static_cast<int>(sx / n), static_cast<int>(sz / n)};
}

std::vector<std::pair<int, int>> way_cells(const ProcessedWay &way)
{
	std::vector<std::pair<int, int>> cells;
	for (std::size_t segment = 1; segment < way.nodes.size(); ++segment) {
		const auto points = bresenham_line(way.nodes[segment - 1].x, 0,
				way.nodes[segment - 1].z, way.nodes[segment].x, 0, way.nodes[segment].z);
		const std::size_t skip = segment > 1 ? 1 : 0;
		for (std::size_t i = skip; i < points.size(); ++i)
			cells.emplace_back(std::get<0>(points[i]), std::get<2>(points[i]));
	}
	return cells;
}

std::optional<float> heading_deg(const ProcessedWay &way)
{
	if (way.nodes.size() < 2)
		return std::nullopt;
	const auto &s = way.nodes.front();
	const auto &e = way.nodes.back();
	const float dx = static_cast<float>(e.x - s.x);
	const float dz = static_cast<float>(e.z - s.z);
	if (dx == 0.0f && dz == 0.0f)
		return std::nullopt;
	constexpr float pi = 3.14159265358979323846f;
	return std::atan2(dz, dx) * 180.0f / pi;
}

std::pair<int, int> midpoint(const ProcessedWay &way)
{
	const auto &n = way.nodes[way.nodes.size() / 2];
	return {n.x, n.z};
}

bool are_dual_carriageway_pair(const ProcessedWay &a, const ProcessedWay &b)
{
	const auto ha = heading_deg(a);
	const auto hb = heading_deg(b);
	if (!ha || !hb)
		return false;
	float diff = std::fmod(std::abs(*ha - *hb), 360.0f);
	if (diff > 180.0f)
		diff = 360.0f - diff;
	const bool parallel =
			diff <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG ||
			std::abs(180.0f - diff) <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG;
	if (!parallel)
		return false;
	const auto ma = midpoint(a);
	const auto mb = midpoint(b);
	const float dx = static_cast<float>(ma.first - mb.first);
	const float dz = static_cast<float>(ma.second - mb.second);
	return std::sqrt(dx * dx + dz * dz) <= DUAL_CARRIAGEWAY_MAX_DISTANCE_BLOCKS;
}

bool headings_parallel(const ProcessedWay &a, const ProcessedWay &b)
{
	const auto ha = heading_deg(a);
	const auto hb = heading_deg(b);
	if (!ha || !hb)
		return false;
	float diff = std::fmod(std::abs(*ha - *hb), 360.0f);
	if (diff > 180.0f)
		diff = 360.0f - diff;
	return diff <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG ||
		   std::abs(180.0f - diff) <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG;
}

std::optional<int> decide_internal_ramp(const std::pair<int, int> &xz, int deck_y,
		const std::unordered_map<std::pair<int, int>, std::size_t, XZPairHash>
				&endpoint_counts,
		const std::unordered_map<std::pair<int, int>, bool, XZPairHash>
				&boundary_with_external_ramp,
		const WorldEditor &editor)
{
	const auto count_it = endpoint_counts.find(xz);
	if (count_it != endpoint_counts.end() && count_it->second > 1)
		return std::nullopt;
	const auto ramp_it = boundary_with_external_ramp.find(xz);
	if (ramp_it != boundary_with_external_ramp.end() && ramp_it->second)
		return std::nullopt;
	const int ground_y = editor.get_ground_level(xz.first, xz.second);
	return deck_y > ground_y ? std::optional<int>(ground_y) : std::nullopt;
}

bridge_styles::BridgeStyle majority_style(const std::vector<std::size_t> &group_indices,
		const std::vector<const ProcessedWay *> &bridge_ways,
		const bridge_styles::BridgeOutlineIndex &outlines)
{
	std::unordered_map<bridge_styles::BridgeStyle, std::size_t> counts;
	for (const auto &idx : group_indices) {
		const auto style = bridge_styles::resolve_bridge_style_with_outline(
				*bridge_ways[idx], outlines);
		++counts[style];
	}
	const bridge_styles::BridgeStyle priority[] = {
			bridge_styles::BridgeStyle::Suspension,
			bridge_styles::BridgeStyle::CableStayed,
			bridge_styles::BridgeStyle::Arch,
			bridge_styles::BridgeStyle::Truss,
			bridge_styles::BridgeStyle::Covered,
			bridge_styles::BridgeStyle::Boardwalk,
			bridge_styles::BridgeStyle::Beam,
	};
	bridge_styles::BridgeStyle best = bridge_styles::BridgeStyle::Beam;
	std::size_t best_count = 0;
	for (const auto &style : priority) {
		const auto count = counts[style];
		if (count > best_count) {
			best = style;
			best_count = count;
		}
	}
	return best;
}
}

bool is_bridge_way(const ProcessedWay &way)
{
	if (way.tags.get("indoor") == "yes")
		return false;
	if (way.tags.get("aeroway") == "jet_bridge")
		return false;
	const auto it = way.tags.find("bridge");
	return it != way.tags.end() && it->second != "no";
}

int BridgeMemberInfo::y_at(
		std::size_t tds, std::size_t total_bresenham, std::size_t ramp_length) const
{
	if (!ys.empty())
		return ys[std::min(tds, ys.size() - 1)];
	if (total_bresenham == 0)
		return deck_y;
	const auto last_idx = total_bresenham - 1;
	const float denom = static_cast<float>(std::max<std::size_t>(1, ramp_length - 1));
	if (start_internal_ramp && tds < ramp_length) {
		const float t = std::min(1.0f, static_cast<float>(tds) / denom);
		return static_cast<int>(
				std::round(*start_internal_ramp + (deck_y - *start_internal_ramp) * t));
	}
	const auto dist_from_end = last_idx > tds ? last_idx - tds : 0;
	if (end_internal_ramp && dist_from_end < ramp_length) {
		const float t = std::min(1.0f, static_cast<float>(dist_from_end) / denom);
		return static_cast<int>(
				std::round(*end_internal_ramp + (deck_y - *end_internal_ramp) * t));
	}
	return deck_y;
}

int BridgeRampInfo::y_at(std::size_t tds, std::size_t total_bresenham) const
{
	if (total_bresenham == 0)
		return deck_y;
	const float denom = static_cast<float>(std::max<std::size_t>(1, total_bresenham - 1));
	const int start_y = bridge_side_at_start ? deck_y : ground_y;
	const int end_y = bridge_side_at_start ? ground_y : deck_y;
	const float t = std::min(1.0f, static_cast<float>(tds) / denom);
	return static_cast<int>(std::round(start_y + (end_y - start_y) * t));
}

BridgeStructureMap BridgeStructureMap::build(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor,
		double scale)
{
	const auto outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	return build(elements, editor, outlines, scale);
}

BridgeStructureMap BridgeStructureMap::build(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor,
		const bridge_styles::BridgeOutlineIndex &outlines, double scale)
{
	BridgeStructureMap result;
	std::vector<const ProcessedWay *> bridge_ways;
	std::vector<const ProcessedWay *> other_highway_ways;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !way.tags.contains("highway"))
			continue;
		if (is_bridge_way(way))
			bridge_ways.push_back(&way);
		else
			other_highway_ways.push_back(&way);
	}
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash>
			node_to_bridge_indices;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		const auto &way = *bridge_ways[i];
		const auto &start = way.nodes.front();
		const auto &end = way.nodes.back();
		node_to_bridge_indices[{start.x, start.z}].push_back(i);
		if (end.x != start.x || end.z != start.z)
			node_to_bridge_indices[{end.x, end.z}].push_back(i);
	}

	UnionFind uf(bridge_ways.size());

	for (const auto &entry : node_to_bridge_indices) {
		const auto &indices = entry.second;
		if (indices.size() < 2)
			continue;
		std::unordered_map<int, std::vector<std::size_t>> by_layer;
		for (const auto &idx : indices)
			by_layer[effective_layer(*bridge_ways[idx])].push_back(idx);
		for (const auto &layer_entry : by_layer) {
			const auto &group = layer_entry.second;
			if (group.size() < 2)
				continue;
			for (std::size_t i = 1; i < group.size(); ++i)
				uf.unite(group.front(), group[i]);
		}
	}

	std::unordered_map<std::string, std::vector<std::size_t>> by_name;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		const auto name = bridge_ways[i]->tags.get("bridge:name");
		if (!name.empty())
			by_name[name].push_back(i);
	}
	for (const auto &entry : by_name) {
		const auto &group = entry.second;
		if (group.size() < 2)
			continue;
		std::vector<std::pair<int, int>> centroids;
		centroids.reserve(group.size());
		for (const auto &idx : group)
			centroids.push_back(centroid(*bridge_ways[idx]));
		for (std::size_t i = 0; i < group.size(); ++i) {
			for (std::size_t j = i + 1; j < group.size(); ++j) {
				const auto a = group[i];
				const auto b = group[j];
				if (effective_layer(*bridge_ways[a]) != effective_layer(*bridge_ways[b]))
					continue;
				if (std::abs(centroids[i].first - centroids[j].first) <=
								BRIDGE_NAME_FUSE_DISTANCE_BLOCKS &&
						std::abs(centroids[i].second - centroids[j].second) <=
								BRIDGE_NAME_FUSE_DISTANCE_BLOCKS) {
					uf.unite(a, b);
				}
			}
		}
	}

	std::vector<std::size_t> oneway_indices;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		if (is_oneway(*bridge_ways[i]))
			oneway_indices.push_back(i);
	}
	for (std::size_t ai = 0; ai < oneway_indices.size(); ++ai) {
		for (std::size_t bi = ai + 1; bi < oneway_indices.size(); ++bi) {
			const auto a = oneway_indices[ai];
			const auto b = oneway_indices[bi];
			if (uf.find(a) == uf.find(b))
				continue;
			if (effective_layer(*bridge_ways[a]) != effective_layer(*bridge_ways[b]))
				continue;
			if (are_dual_carriageway_pair(*bridge_ways[a], *bridge_ways[b]))
				uf.unite(a, b);
		}
	}

	std::unordered_map<std::size_t, std::vector<std::size_t>> groups;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i)
		groups[uf.find(i)].push_back(i);
	std::vector<std::vector<std::size_t>> ordered_groups;
	ordered_groups.reserve(groups.size());
	for (auto &entry : groups)
		ordered_groups.push_back(std::move(entry.second));
	const auto group_layer = [&](const std::vector<std::size_t> &group) {
		int layer = 0;
		for (const auto idx : group)
			layer = std::max(layer, effective_layer(*bridge_ways[idx]));
		return layer;
	};
	std::sort(ordered_groups.begin(), ordered_groups.end(),
			[&](const auto &a, const auto &b) {
				const int layer_a = group_layer(a);
				const int layer_b = group_layer(b);
				return layer_a != layer_b ? layer_a < layer_b : a.front() < b.front();
			});

	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash>
			node_to_other_highways;
	for (std::size_t i = 0; i < other_highway_ways.size(); ++i) {
		const auto &way = *other_highway_ways[i];
		const auto &start = way.nodes.front();
		const auto &end = way.nodes.back();
		node_to_other_highways[{start.x, start.z}].push_back(i);
		if (end.x != start.x || end.z != start.z)
			node_to_other_highways[{end.x, end.z}].push_back(i);
	}
	int bounds_min_x = std::numeric_limits<int>::max();
	int bounds_min_z = std::numeric_limits<int>::max();
	int bounds_max_x = std::numeric_limits<int>::lowest();
	int bounds_max_z = std::numeric_limits<int>::lowest();
	for (const auto *way : bridge_ways) {
		for (const auto &node : way->nodes) {
			bounds_min_x = std::min(bounds_min_x, node.x);
			bounds_min_z = std::min(bounds_min_z, node.z);
			bounds_max_x = std::max(bounds_max_x, node.x);
			bounds_max_z = std::max(bounds_max_z, node.z);
		}
	}
	GradeObstacleIndex obstacles;
	if (!bridge_ways.empty())
		obstacles.build(
				elements, scale, bounds_min_x, bounds_min_z, bounds_max_x, bounds_max_z);
	std::unordered_map<std::pair<int, int>, std::pair<int, int>, XZPairHash>
			resolved_decks;
	std::unordered_map<std::pair<int, int>, int, XZPairHash> resolved_joints;

	std::unordered_set<std::uint64_t> claimed_ramp_ways;

	for (const auto &group_indices : ordered_groups) {
		const auto group_style = majority_style(group_indices, bridge_ways, outlines);
		std::unordered_map<std::pair<int, int>, std::size_t, XZPairHash> endpoint_counts;
		for (const auto &idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto &s = way.nodes.front();
			const auto &e = way.nodes.back();
			++endpoint_counts[{s.x, s.z}];
			if (e.x != s.x || e.z != s.z)
				++endpoint_counts[{e.x, e.z}];
		}

		int max_layer = 0;
		for (const auto &idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto it = way.tags.find("layer");
			if (it == way.tags.end())
				continue;
			if (const auto layer = strict_parse::i32(it->second))
				max_layer = std::max(max_layer, std::max(0, *layer));
		}

		std::unordered_map<std::size_t, std::vector<std::pair<int, int>>> paths;
		std::unordered_map<std::size_t, std::vector<int>> path_terrain;
		int terrain_min = std::numeric_limits<int>::max();
		int terrain_max = std::numeric_limits<int>::lowest();
		for (const auto &idx : group_indices) {
			auto &path = paths[idx];
			path = way_cells(*bridge_ways[idx]);
			auto &terrain = path_terrain[idx];
			terrain.reserve(path.size());
			for (const auto &[x, z] : path) {
				const int y = editor.get_ground_level(x, z);
				terrain.push_back(y);
				terrain_min = std::min(terrain_min, y);
				terrain_max = std::max(terrain_max, y);
			}
		}
		if (terrain_min == std::numeric_limits<int>::max())
			continue;
		const int dip = terrain_max - terrain_min;
		std::size_t total_length = 0;
		for (const auto &idx : group_indices)
			total_length += way_length_blocks(*bridge_ways[idx]);
		const auto &asset_root = editor.get_schematic_asset_root();
		const auto member_range = [&](std::size_t idx) {
			return highway_block_range(*bridge_ways[idx], scale);
		};
		const bool group_has_vehicular_member = std::any_of(
				group_indices.begin(), group_indices.end(), [&](std::size_t idx) {
					return !is_non_vehicular_bridge_highway(*bridge_ways[idx]);
				});
		const bool structure_has_module =
				group_style == bridge_styles::BridgeStyle::Beam &&
				group_has_vehicular_member &&
				std::any_of(
						group_indices.begin(), group_indices.end(), [&](std::size_t idx) {
							return bridge_modules::pick_module_index(
									member_range(idx), total_length, asset_root)
									.has_value();
						});

		// A wide modular deck replaces fully contained parallel members. Keep the
		// decision in the structure map so rendering, support placement, and the
		// prescanned bridge surface all agree on which way owns the deck.
		std::unordered_set<std::uint64_t> covered_ids;
		std::unordered_set<std::uint64_t> widened_ids;
		if (structure_has_module && group_indices.size() > 1) {
			auto order = group_indices;
			std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
				return member_range(a) != member_range(b)
							   ? member_range(a) > member_range(b)
					   : way_length_blocks(*bridge_ways[a]) !=
									   way_length_blocks(*bridge_ways[b])
							   ? way_length_blocks(*bridge_ways[a]) >
										 way_length_blocks(*bridge_ways[b])
							   : bridge_ways[a]->id < bridge_ways[b]->id;
			});
			for (const auto idx : order) {
				const auto &way = *bridge_ways[idx];
				for (const auto wider_idx : order) {
					const auto &wider = *bridge_ways[wider_idx];
					if (wider.id == way.id || covered_ids.contains(wider.id))
						continue;
					const int wider_range = member_range(wider_idx);
					const int range = member_range(idx);
					const auto wider_length = way_length_blocks(wider);
					const auto length = way_length_blocks(way);
					const bool is_wider =
							wider_range > range ||
							(wider_range == range && (wider_length > length ||
															 (wider_length == length &&
																	 wider.id < way.id)));
					if (!is_wider || length > wider_length + 30 ||
							!headings_parallel(way, wider))
						continue;
					const auto module_idx = bridge_modules::pick_module_index(
							wider_range, total_length, asset_root);
					const int deck_half = module_idx ? bridge_modules::module_half_width(
															   *module_idx, asset_root)
													 : 0;
					float threshold = static_cast<float>(std::max(0, deck_half - 2));
					if (wider_range == range)
						threshold =
								std::min(threshold, DUAL_CARRIAGEWAY_MAX_DISTANCE_BLOCKS);
					const auto samples = coverage_samples(way);
					const bool covered =
							!samples.empty() &&
							std::all_of(samples.begin(), samples.end(),
									[&](const auto &point) {
										return lateral_offset_to_way(point.first,
													   point.second, wider) <= threshold;
									});
					if (!covered)
						continue;
					covered_ids.insert(way.id);
					if (wider_range == range)
						widened_ids.insert(wider.id);
				}
			}
		}
		int deck_y = terrain_max;

		// Derive a shared longitudinal deck profile for the whole bridge group.
		// A single max-endpoint height makes long bridges float over ridges and
		// bury into terrain; the upper hull preserves terrain clearance while
		// keeping the profile continuous across connected members.
		std::unordered_map<std::size_t, std::vector<float>> path_axis;
		std::vector<std::pair<int, int>> profile_ends;
		std::unordered_set<std::uint64_t> cable_carriers;
		std::vector<std::pair<int, int>> cable_pylons;
		if (group_style == bridge_styles::BridgeStyle::Suspension ||
				group_style == bridge_styles::BridgeStyle::CableStayed) {
			std::optional<std::size_t> main_idx;
			for (const auto idx : group_indices) {
				if (covered_ids.contains(bridge_ways[idx]->id))
					continue;
				if (!main_idx ||
						way_length_blocks(*bridge_ways[idx]) >
								way_length_blocks(*bridge_ways[*main_idx]) ||
						(way_length_blocks(*bridge_ways[idx]) ==
										way_length_blocks(*bridge_ways[*main_idx]) &&
								bridge_ways[idx]->id < bridge_ways[*main_idx]->id))
					main_idx = idx;
			}
			if (main_idx) {
				const auto &main_way = *bridge_ways[*main_idx];
				const auto &main_path = paths[*main_idx];
				for (const auto point : bridge_styles::default_pylons(
							 group_style, main_path.size(), true, true))
					if (point < main_path.size())
						cable_pylons.push_back(main_path[point]);
				if (!cable_pylons.empty()) {
					const auto main_length = way_length_blocks(main_way);
					for (const auto idx : group_indices) {
						const auto &way = *bridge_ways[idx];
						if (idx == *main_idx ||
								(!covered_ids.contains(way.id) &&
										way_length_blocks(way) * 10 >= main_length * 6 &&
										bridge_styles::resolve_bridge_style_with_outline(
												way, outlines) == group_style &&
										headings_parallel(way, main_way)))
							cable_carriers.insert(way.id);
					}
				}
			}
		}

		for (const auto idx : group_indices) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			const auto &way = *bridge_ways[idx];
			const auto &path = paths[idx];
			if (path.empty())
				continue;
			profile_ends.emplace_back(way.nodes.front().x, way.nodes.front().z);
			profile_ends.emplace_back(way.nodes.back().x, way.nodes.back().z);
			auto &axis = path_axis[idx];
			axis.reserve(path.size());
		}
		std::pair<int, int> axis_start{0, 0};
		std::pair<int, int> axis_end{0, 0};
		std::int64_t axis_distance_squared = 0;
		for (std::size_t a = 0; a < profile_ends.size(); ++a) {
			for (std::size_t b = a + 1; b < profile_ends.size(); ++b) {
				const std::int64_t dx = profile_ends[b].first - profile_ends[a].first;
				const std::int64_t dz = profile_ends[b].second - profile_ends[a].second;
				const std::int64_t distance = dx * dx + dz * dz;
				if (distance > axis_distance_squared) {
					axis_distance_squared = distance;
					axis_start = profile_ends[a];
					axis_end = profile_ends[b];
				}
			}
		}
		const float axis_length = std::sqrt(static_cast<float>(axis_distance_squared));
		const float axis_dx =
				axis_length > 0.0f
						? static_cast<float>(axis_end.first - axis_start.first) /
								  axis_length
						: 0.0f;
		const float axis_dz =
				axis_length > 0.0f
						? static_cast<float>(axis_end.second - axis_start.second) /
								  axis_length
						: 0.0f;
		bool follows_axis = axis_length >= 1.0f;
		for (auto &[idx, path] : paths) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			auto &us = path_axis[idx];
			us.reserve(path.size());
			for (const auto &[x, z] : path)
				us.push_back((x - axis_start.first) * axis_dx +
							 (z - axis_start.second) * axis_dz);
			follows_axis = follows_axis && is_monotonic(us, 2.0f);
		}
		std::vector<std::pair<float, float>> profile_requirements;
		std::vector<float> profile_anchors;
		std::vector<std::pair<float, int>> profile_anchor_levels;
		std::vector<float> profile_free_ends;
		std::unordered_set<std::pair<int, int>, XZPairHash> group_nodes;
		for (const auto idx : group_indices) {
			for (const auto &node : bridge_ways[idx]->nodes)
				group_nodes.emplace(node.x, node.z);
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			const auto &us = path_axis[idx];
			const auto collect_endpoint = [&](const ProcessedNode &node, float u) {
				const std::pair<int, int> xz{node.x, node.z};
				const auto count = endpoint_counts.find(xz);
				const bool boundary =
						count == endpoint_counts.end() || count->second == 1;
				bool joins_other_bridge = false;
				if (const auto members = node_to_bridge_indices.find(xz);
						members != node_to_bridge_indices.end()) {
					joins_other_bridge = std::any_of(members->second.begin(),
							members->second.end(), [&](std::size_t other) {
								return std::find(group_indices.begin(),
											   group_indices.end(),
											   other) == group_indices.end();
							});
				}
				if (joins_other_bridge) {
					if (const auto joint = resolved_joints.find(xz);
							joint != resolved_joints.end()) {
						profile_anchors.push_back(u);
						profile_anchor_levels.emplace_back(u, joint->second);
					} else {
						profile_free_ends.push_back(u);
					}
					return;
				}
				bool has_external_ramp = false;
				if (boundary) {
					const auto candidates = node_to_other_highways.find(xz);
					if (candidates != node_to_other_highways.end()) {
						for (const auto candidate_idx : candidates->second) {
							const auto &candidate = *other_highway_ways[candidate_idx];
							if (!is_ramp_candidate(candidate) ||
									claimed_ramp_ways.contains(candidate.id))
								continue;
							const bool at_start = candidate.nodes.front().x == node.x &&
												  candidate.nodes.front().z == node.z;
							const auto &far = at_start ? candidate.nodes.back()
													   : candidate.nodes.front();
							if (!endpoint_counts.contains({far.x, far.z})) {
								has_external_ramp = true;
								break;
							}
						}
					}
				}
				if (boundary && !has_external_ramp) {
					profile_anchors.push_back(u);
					profile_anchor_levels.emplace_back(
							u, editor.get_ground_level(node.x, node.z));
				} else if (boundary && has_external_ramp) {
					profile_free_ends.push_back(u);
				}
			};
			collect_endpoint(bridge_ways[idx]->nodes.front(), us.front());
			collect_endpoint(bridge_ways[idx]->nodes.back(), us.back());
		}
		auto reachable = [&](float u) {
			float limit = std::numeric_limits<float>::infinity();
			for (const auto &[anchor_u, anchor_y] : profile_anchor_levels)
				limit = std::min(limit, anchor_y + std::abs(u - anchor_u));
			return limit;
		};
		int profile_level = terrain_max;
		int raised_top = terrain_max;
		std::unordered_map<std::size_t, bool> connected_obstacles;
		for (const auto idx : group_indices) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			const auto &path = paths[idx];
			const auto &us = path_axis[idx];
			const auto &terrain = path_terrain[idx];
			const int layer = effective_layer(*bridge_ways[idx]);
			for (std::size_t i = 0; i < path.size(); ++i) {
				const auto [x, z] = path[i];
				const int ground = terrain[i];
				profile_requirements.emplace_back(us[i], ground);
				int need = ground;
				const auto [px, pz] = path[i == 0 ? 0 : i - 1];
				const auto [nx, nz] = path[std::min(i + 1, path.size() - 1)];
				const float heading_x = static_cast<float>(nx - px);
				const float heading_z = static_cast<float>(nz - pz);
				if (const auto *nearby = obstacles.at(x, z)) {
					for (const auto segment_index : *nearby) {
						const auto &segment = obstacles.segments[segment_index];
						if (point_segment_distance(static_cast<float>(x),
									static_cast<float>(z), segment) > segment.reach ||
								!crosses_segment(heading_x, heading_z, segment))
							continue;
						const auto [touch_it, inserted] =
								connected_obstacles.try_emplace(segment.way, false);
						if (inserted) {
							const auto *other = obstacles.ways[segment.way];
							touch_it->second = std::any_of(other->nodes.begin(),
									other->nodes.end(), [&](const auto &node) {
										return group_nodes.contains({node.x, node.z});
									});
						}
						if (!touch_it->second)
							need = std::max(need, ground + segment.headroom);
					}
				}
				if (const auto lower = resolved_decks.find({x, z});
						lower != resolved_decks.end() && lower->second.second < layer)
					need = std::max(need, lower->second.first + STACKED_DECK_HEADROOM);
				if (need > ground) {
					profile_level = std::max(profile_level, need);
					const float capped = std::max(static_cast<float>(ground),
							std::min(static_cast<float>(need), reachable(us[i])));
					profile_requirements.emplace_back(us[i], capped);
					raised_top =
							std::max(raised_top, static_cast<int>(std::lround(capped)));
				}
			}
		}
		const bool has_boundary_endpoint =
				!profile_anchors.empty() || !profile_free_ends.empty();
		const bool flat_span = has_boundary_endpoint &&
							   dip < FLAT_TERRAIN_DIP_THRESHOLD &&
							   total_length >= SHORT_BRIDGE_LENGTH_BLOCKS;
		int profile_clearance = flat_span ? max_layer * LAYER_HEIGHT_STEP : 0;
		if (flat_span && group_style == bridge_styles::BridgeStyle::Arch)
			profile_clearance = std::max(profile_clearance, 8);
		profile_level = std::max(profile_level, terrain_max + profile_clearance);
		raised_top = std::max(raised_top, profile_level);
		for (const float u : profile_free_ends)
			profile_requirements.emplace_back(u, raised_top);
		if (profile_clearance > 0) {
			float min_u = std::numeric_limits<float>::max();
			float max_u = std::numeric_limits<float>::lowest();
			for (const auto &[idx, values] : path_axis) {
				(void)idx;
				for (const float u : values) {
					min_u = std::min(min_u, u);
					max_u = std::max(max_u, u);
				}
			}
			const float span = std::max(0.0f, max_u - min_u);
			const float ramp =
					std::min(std::clamp(span * 0.35f, 15.0f, 50.0f), span / 2.0f);
			bool placed_plateau = false;
			for (const auto &[idx, values] : path_axis) {
				(void)idx;
				for (const float u : values) {
					if (std::all_of(profile_anchors.begin(), profile_anchors.end(),
								[&](float anchor) {
									return std::abs(u - anchor) >= ramp;
								})) {
						profile_requirements.emplace_back(u, profile_level);
						placed_plateau = true;
					}
				}
			}
			if (!placed_plateau)
				profile_requirements.emplace_back((min_u + max_u) * 0.5f, profile_level);
		}
		const auto profile_hull = follows_axis
										  ? upper_hull(std::move(profile_requirements))
										  : std::vector<std::pair<float, float>>{};
		deck_y = profile_level;
		std::unordered_map<std::size_t, std::vector<int>> member_profiles;
		for (const auto &[idx, path] : paths) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			auto &ys = member_profiles[idx];
			const auto &us = path_axis[idx];
			const auto &terrain = path_terrain[idx];
			ys.reserve(path.size());
			for (std::size_t i = 0; i < path.size(); ++i) {
				const int y = follows_axis ? static_cast<int>(std::lround(
													 hull_at(profile_hull, us[i])))
										   : profile_level;
				ys.push_back(std::max(y, terrain[i]));
			}
		}
		for (const auto idx : group_indices) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			const auto profile = member_profiles.find(idx);
			if (profile == member_profiles.end() || profile->second.empty())
				continue;
			const auto &way = *bridge_ways[idx];
			for (std::size_t endpoint = 0; endpoint < 2; ++endpoint) {
				const auto &node = endpoint == 0 ? way.nodes.front() : way.nodes.back();
				const std::pair<int, int> xz{node.x, node.z};
				const auto count = endpoint_counts.find(xz);
				if (count == endpoint_counts.end() || count->second != 1)
					continue;
				const int y =
						endpoint == 0 ? profile->second.front() : profile->second.back();
				auto [joint, inserted] = resolved_joints.try_emplace(xz, y);
				if (!inserted)
					joint->second = std::max(joint->second, y);
			}
		}
		for (const auto idx : group_indices) {
			if (covered_ids.contains(bridge_ways[idx]->id))
				continue;
			const auto profile = member_profiles.find(idx);
			if (profile == member_profiles.end())
				continue;
			const auto &path = paths[idx];
			const int layer = effective_layer(*bridge_ways[idx]);
			const bool member_has_module =
					structure_has_module &&
					!is_non_vehicular_bridge_highway(*bridge_ways[idx]);
			int half_width = highway_block_range(*bridge_ways[idx], scale);
			if (member_has_module) {
				const int width = member_range(idx) +
								  (widened_ids.contains(bridge_ways[idx]->id) ? 1 : 0);
				if (const auto module_idx = bridge_modules::pick_module_index(
							width, total_length, asset_root))
					half_width =
							bridge_modules::module_half_width(*module_idx, asset_root);
			}
			for (std::size_t i = 0; i < path.size(); ++i) {
				const int y = profile->second[i];
				for (int dx = -half_width; dx <= half_width; ++dx) {
					for (int dz = -half_width; dz <= half_width; ++dz) {
						const auto key =
								std::pair{path[i].first + dx, path[i].second + dz};
						auto [it, inserted] = resolved_decks.try_emplace(key, y, layer);
						if (!inserted && y > it->second.first)
							it->second = {y, layer};
					}
				}
			}
		}

		std::unordered_map<std::pair<int, int>, bool, XZPairHash>
				boundary_with_external_ramp;
		for (const auto &entry : endpoint_counts) {
			const auto xz = entry.first;
			const auto count = entry.second;
			if (count > 1)
				continue;
			const auto other_it = node_to_other_highways.find(xz);
			if (other_it == node_to_other_highways.end()) {
				boundary_with_external_ramp[xz] = false;
				continue;
			}

			bool found_ramp = false;
			for (const auto &oi : other_it->second) {
				const auto &candidate = *other_highway_ways[oi];
				if (!is_ramp_candidate(candidate))
					continue;
				if (claimed_ramp_ways.contains(candidate.id))
					continue;
				const bool bridge_side_at_start = candidate.nodes.front().x == xz.first &&
												  candidate.nodes.front().z == xz.second;
				const auto &far_node = bridge_side_at_start ? candidate.nodes.back()
															: candidate.nodes.front();
				if (endpoint_counts.contains({far_node.x, far_node.z}))
					continue;

				BridgeRampInfo info;
				info.bridge_side_at_start = bridge_side_at_start;
				info.deck_y = deck_y;
				for (const auto idx : group_indices) {
					const auto *member = bridge_ways[idx];
					const auto profile = member_profiles.find(idx);
					if (profile == member_profiles.end() || profile->second.empty())
						continue;
					if (member->nodes.front().x == xz.first &&
							member->nodes.front().z == xz.second) {
						info.deck_y = profile->second.front();
						break;
					}
					if (member->nodes.back().x == xz.first &&
							member->nodes.back().z == xz.second) {
						info.deck_y = profile->second.back();
						break;
					}
				}
				info.ground_y = editor.get_ground_level(far_node.x, far_node.z);
				result.ramps_.emplace(candidate.id, info);
				claimed_ramp_ways.insert(candidate.id);
				found_ramp = true;
			}
			boundary_with_external_ramp[xz] = found_ramp;
		}

		for (const auto &idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto &s = way.nodes.front();
			const auto &e = way.nodes.back();
			BridgeMemberInfo info;
			info.deck_y = deck_y;
			info.style = group_style;
			info.start_internal_ramp = decide_internal_ramp({s.x, s.z}, deck_y,
					endpoint_counts, boundary_with_external_ramp, editor);
			info.end_internal_ramp = decide_internal_ramp({e.x, e.z}, deck_y,
					endpoint_counts, boundary_with_external_ramp, editor);
			info.covered_by_wider = covered_ids.contains(way.id);
			if (cable_carriers.contains(way.id))
				info.cable_pylons = cable_pylons;
			if (const auto profile = member_profiles.find(idx);
					profile != member_profiles.end())
				info.ys = profile->second;
			if (structure_has_module && !info.covered_by_wider &&
					!is_non_vehicular_bridge_highway(way)) {
				const int width =
						member_range(idx) + (widened_ids.contains(way.id) ? 1 : 0);
				info.module_idx = bridge_modules::pick_module_index(
						width, total_length, asset_root);
				if (info.module_idx)
					info.module_half_width = bridge_modules::module_half_width(
							*info.module_idx, asset_root);
			}
			result.members_.emplace(way.id, info);
		}
	}

	// Plan rail decks after road bridge profiles are resolved. This mirrors Rust's
	// carried-track detection and shared level for connected rail viaducts.
	std::vector<const ProcessedWay *> rail_bridge_ways;
	for (const auto &element : elements)
		if (element.is_way() && railways::renders_as_rail_bridge(element.as_way()))
			rail_bridge_ways.push_back(&element.as_way());
	if (!rail_bridge_ways.empty()) {
		std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash>
				rail_endpoints;
		int min_x = std::numeric_limits<int>::max();
		int min_z = std::numeric_limits<int>::max();
		int max_x = std::numeric_limits<int>::lowest();
		int max_z = std::numeric_limits<int>::lowest();
		for (std::size_t i = 0; i < rail_bridge_ways.size(); ++i) {
			const auto &way = *rail_bridge_ways[i];
			const auto &start = way.nodes.front();
			const auto &end = way.nodes.back();
			rail_endpoints[{start.x, start.z}].push_back(i);
			if (start.x != end.x || start.z != end.z)
				rail_endpoints[{end.x, end.z}].push_back(i);
			for (const auto &node : way.nodes) {
				min_x = std::min(min_x, node.x);
				min_z = std::min(min_z, node.z);
				max_x = std::max(max_x, node.x);
				max_z = std::max(max_z, node.z);
			}
		}
		GradeObstacleIndex rail_obstacles;
		rail_obstacles.build(elements, scale, min_x, min_z, max_x, max_z);
		UnionFind rail_uf(rail_bridge_ways.size());
		for (const auto &[endpoint, indices] : rail_endpoints) {
			(void)endpoint;
			std::unordered_map<int, std::vector<std::size_t>> by_layer;
			for (const auto index : indices)
				by_layer[effective_layer(*rail_bridge_ways[index])].push_back(index);
			for (const auto &[layer, group] : by_layer) {
				(void)layer;
				for (std::size_t i = 1; i < group.size(); ++i)
					rail_uf.unite(group.front(), group[i]);
			}
		}

		std::unordered_set<std::uint64_t> carried_ids;
		for (const auto *way : rail_bridge_ways) {
			const auto path = railways::build_smoothed_centerline(*way);
			if (path.empty())
				continue;
			const int layer = effective_layer(*way);
			std::vector<std::optional<int>> road_y(path.size());
			std::size_t carried = 0;
			for (std::size_t i = 0; i < path.size(); ++i) {
				int best_distance = std::numeric_limits<int>::max();
				for (int dx = -2; dx <= 2; ++dx)
					for (int dz = -2; dz <= 2; ++dz) {
						auto deck = resolved_decks.find(
								{path[i].first + dx, path[i].second + dz});
						const int distance = std::abs(dx) + std::abs(dz);
						if (deck != resolved_decks.end() &&
								deck->second.second == layer &&
								distance < best_distance) {
							best_distance = distance;
							road_y[i] = deck->second.first;
						}
					}
				carried += road_y[i].has_value();
			}
			if (carried * 10 < path.size() * 9)
				continue;
			for (std::size_t i = 1; i < road_y.size(); ++i)
				if (!road_y[i])
					road_y[i] = road_y[i - 1];
			for (std::size_t i = road_y.size(); i-- > 1;)
				if (!road_y[i - 1])
					road_y[i - 1] = road_y[i];
			RailDeckInfo info;
			info.kind = RailDeckKind::Carried;
			info.ys.reserve(path.size());
			for (std::size_t i = 0; i < path.size(); ++i)
				info.ys.push_back(std::max(road_y[i].value_or(0),
						editor.get_ground_level(path[i].first, path[i].second)));
			result.rail_decks_[way->id] = std::move(info);
			carried_ids.insert(way->id);
		}

		std::unordered_map<std::size_t, std::vector<std::size_t>> rail_groups;
		for (std::size_t i = 0; i < rail_bridge_ways.size(); ++i)
			if (!carried_ids.contains(rail_bridge_ways[i]->id))
				rail_groups[rail_uf.find(i)].push_back(i);
		for (const auto &[root, group] : rail_groups) {
			(void)root;
			int terrain_min = std::numeric_limits<int>::max();
			int terrain_max = std::numeric_limits<int>::lowest();
			int floor_y = std::numeric_limits<int>::lowest();
			int layer = 0;
			bool arch = false;
			for (const auto index : group) {
				const auto &way = *rail_bridge_ways[index];
				layer = std::max(layer, effective_layer(way));
				arch = arch || bridge_styles::resolve_bridge_style_with_outline(
									   way, outlines) == bridge_styles::BridgeStyle::Arch;
				std::unordered_set<std::pair<int, int>, XZPairHash> own_nodes;
				for (const auto &node : way.nodes)
					own_nodes.insert({node.x, node.z});
				const auto path = railways::build_smoothed_centerline(way);
				for (std::size_t p = 0; p < path.size(); ++p) {
					const auto [x, z] = path[p];
					const int ground = editor.get_ground_level(x, z);
					terrain_min = std::min(terrain_min, ground);
					terrain_max = std::max(terrain_max, ground);
					if (const auto lower = resolved_decks.find({x, z});
							lower != resolved_decks.end() &&
							lower->second.second < effective_layer(way))
						floor_y = std::max(
								floor_y, lower->second.first + STACKED_DECK_HEADROOM);
					if (const auto *nearby = rail_obstacles.at(x, z)) {
						const auto [px, pz] = path[p == 0 ? p : p - 1];
						const auto [nx, nz] = path[std::min(p + 1, path.size() - 1)];
						const float hx = static_cast<float>(nx - px);
						const float hz = static_cast<float>(nz - pz);
						for (const auto segment_index : *nearby) {
							const auto &segment = rail_obstacles.segments[segment_index];
							if (point_segment_distance(static_cast<float>(x),
										static_cast<float>(z), segment) > segment.reach ||
									!crosses_segment(hx, hz, segment))
								continue;
							const auto *obstacle_way = rail_obstacles.ways[segment.way];
							if (std::any_of(obstacle_way->nodes.begin(),
										obstacle_way->nodes.end(), [&](const auto &node) {
											return own_nodes.contains({node.x, node.z});
										}))
								continue;
							floor_y = std::max(floor_y, ground + segment.headroom);
						}
					}
				}
			}
			if (terrain_max == std::numeric_limits<int>::lowest())
				continue;
			int clearance = railways::RAIL_BRIDGE_FLAT_CLEARANCE +
							std::max(0, layer - 1) * LAYER_HEIGHT_STEP;
			if (arch)
				clearance = std::max(clearance, 8);
			const int deck_y =
					terrain_max - terrain_min < railways::RAIL_BRIDGE_DIP_THRESHOLD
							? terrain_max + clearance
							: terrain_max;
			for (const auto index : group) {
				RailDeckInfo info;
				info.kind = RailDeckKind::Level;
				info.level_y = std::max(deck_y, floor_y);
				result.rail_decks_[rail_bridge_ways[index]->id] = std::move(info);
			}
		}
	}
	return result;
}

const BridgeMemberInfo *BridgeStructureMap::lookup_member(std::uint64_t way_id) const
{
	auto it = members_.find(way_id);
	return it == members_.end() ? nullptr : &it->second;
}

const BridgeRampInfo *BridgeStructureMap::lookup_ramp(std::uint64_t way_id) const
{
	auto it = ramps_.find(way_id);
	return it == ramps_.end() ? nullptr : &it->second;
}

const RailDeckInfo *BridgeStructureMap::rail_deck(std::uint64_t way_id) const
{
	auto it = rail_decks_.find(way_id);
	return it == rail_decks_.end() ? nullptr : &it->second;
}

BridgeSurfaceMap BridgeSurfaceMap::build(const std::vector<ProcessedElement> &elements,
		const BridgeStructureMap &structures, double scale)
{
	BridgeSurfaceMap result;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !way.tags.contains("highway"))
			continue;
		const auto *info = structures.lookup_member(way.id);
		const auto *ramp = structures.lookup_ramp(way.id);
		if (!info && !ramp)
			continue;
		if (info && info->covered_by_wider)
			continue;
		const int block_range = info && info->module_idx
										? info->module_half_width
										: highway_block_range(way, scale);

		std::size_t total_bresenham = 1;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &a = way.nodes[i - 1];
			const auto &b = way.nodes[i];
			total_bresenham += static_cast<std::size_t>(
					std::max(std::abs(b.x - a.x), std::abs(b.z - a.z)));
		}
		const std::size_t raw_ramp = static_cast<std::size_t>(
				std::clamp(static_cast<float>(total_bresenham) * 0.35f, 15.0f, 50.0f));
		const std::size_t internal_ramp_length = std::clamp<std::size_t>(
				raw_ramp, 1, std::max<std::size_t>(1, total_bresenham / 2));

		std::size_t tds = 0;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &prev = way.nodes[i - 1];
			const auto &cur = way.nodes[i];
			const auto points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			const std::size_t skip_first = i == 1 ? 0 : 1;
			for (std::size_t point_idx = skip_first; point_idx < points.size();
					++point_idx) {
				const auto &p = points[point_idx];
				const int x = std::get<0>(p);
				const int z = std::get<2>(p);
				const int cell_y =
						info ? info->y_at(tds, total_bresenham, internal_ramp_length)
							 : ramp->y_at(tds, total_bresenham);
				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						const std::pair<int, int> cell{x + dx, z + dz};
						auto existing = result.deck_y_.find(cell);
						if (existing == result.deck_y_.end())
							result.deck_y_.emplace(cell, cell_y);
						else
							existing->second = std::max(existing->second, cell_y);
						auto low = result.deck_low_y_.find(cell);
						if (low == result.deck_low_y_.end())
							result.deck_low_y_.emplace(cell, cell_y);
						else
							low->second = std::min(low->second, cell_y);
					}
				}
				++tds;
			}
		}
	}
	// Rail viaducts are narrower than road decks; Rust treats a 3x3 footprint
	// around each planned level-deck centreline cell as sheltered surface.
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto *rail = structures.rail_deck(way.id);
		if (!rail || rail->kind != RailDeckKind::Level)
			continue;
		for (const auto &[x, z] : railways::build_smoothed_centerline(way)) {
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz) {
					const auto cell = std::pair{x + dx, z + dz};
					auto [it, inserted] =
							result.rail_deck_y_.try_emplace(cell, rail->level_y);
					if (!inserted)
						it->second = std::min(it->second, rail->level_y);
				}
		}
	}
	// Mark at-grade roads/tracks below a raised deck, including the whole road
	// footprint and a one-cell shoulder. Match Rust's coarse spatial prefilter
	// and grade-obstacle eligibility: tunnels, elevated ways, pedestrian
	// features, and approach ramps are not support blockers.
	std::unordered_set<std::pair<int, int>, XZPairHash> deck_buckets;
	for (const auto &entry : result.deck_y_)
		deck_buckets.emplace(div_euclid(entry.first.first, OBSTACLE_GRID_CELL),
				div_euclid(entry.first.second, OBSTACLE_GRID_CELL));
	for (const auto &entry : result.rail_deck_y_)
		deck_buckets.emplace(div_euclid(entry.first.first, OBSTACLE_GRID_CELL),
				div_euclid(entry.first.second, OBSTACLE_GRID_CELL));
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (structures.lookup_ramp(way.id))
			continue;
		const auto obstacle = grade_obstacle(way, scale);
		if (!obstacle)
			continue;
		const int reach = obstacle->first + 1;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &a = way.nodes[i - 1];
			const auto &b = way.nodes[i];
			const int x0 = std::min(a.x, b.x) - reach;
			const int x1 = std::max(a.x, b.x) + reach;
			const int z0 = std::min(a.z, b.z) - reach;
			const int z1 = std::max(a.z, b.z) + reach;
			bool near_deck = false;
			for (int bx = div_euclid(x0, OBSTACLE_GRID_CELL);
					bx <= div_euclid(x1, OBSTACLE_GRID_CELL) && !near_deck; ++bx)
				for (int bz = div_euclid(z0, OBSTACLE_GRID_CELL);
						bz <= div_euclid(z1, OBSTACLE_GRID_CELL); ++bz)
					if (deck_buckets.contains({bx, bz})) {
						near_deck = true;
						break;
					}
			if (!near_deck)
				continue;
			const auto points = bresenham_line(way.nodes[i - 1].x, 0, way.nodes[i - 1].z,
					way.nodes[i].x, 0, way.nodes[i].z);
			for (const auto &p : points) {
				for (int dx = -reach; dx <= reach; ++dx)
					for (int dz = -reach; dz <= reach; ++dz) {
						const std::pair<int, int> cell{
								std::get<0>(p) + dx, std::get<2>(p) + dz};
						if (result.deck_y_.contains(cell) ||
								result.rail_deck_y_.contains(cell))
							result.grade_crossings_.insert(cell);
					}
			}
		}
	}
	return result;
}

std::optional<int> BridgeSurfaceMap::deck_y_at(int x, int z) const
{
	auto it = deck_y_.find({x, z});
	auto rail = rail_deck_y_.find({x, z});
	if (it == deck_y_.end() && rail == rail_deck_y_.end())
		return std::nullopt;
	if (it == deck_y_.end())
		return rail->second;
	if (rail == rail_deck_y_.end())
		return it->second;
	return std::max(it->second, rail->second);
}

std::optional<int> BridgeSurfaceMap::nearby_deck_y(int x, int z, int radius) const
{
	if (auto direct = deck_y_at(x, z))
		return direct;
	std::optional<int> found;
	for (int r = 1; r <= radius; ++r) {
		for (int dx = -r; dx <= r; ++dx) {
			for (int dz = -r; dz <= r; ++dz) {
				if (std::abs(dx) != r && std::abs(dz) != r)
					continue;
				if (auto y = deck_y_at(x + dx, z + dz))
					found = found ? std::max(*found, *y) : *y;
			}
		}
	}
	return found;
}

bool BridgeSurfaceMap::deck_near(int x, int z, int y, int tolerance) const
{
	const auto high = deck_y_.find({x, z});
	const auto low = deck_low_y_.find({x, z});
	return high != deck_y_.end() && low != deck_low_y_.end() &&
		   low->second <= y + tolerance && high->second >= y - tolerance;
}

bool BridgeSurfaceMap::support_blocked(int x, int z, int deck_y) const
{
	if (grade_crossings_.contains({x, z}))
		return true;
	const auto lower = deck_low_y_.find({x, z});
	const auto rail = rail_deck_y_.find({x, z});
	return (lower != deck_low_y_.end() && lower->second <= deck_y - 6) ||
		   (rail != rail_deck_y_.end() && rail->second <= deck_y - 6);
}

std::optional<int> BridgeSurfaceMap::supported_top(
		const WorldEditor &editor, int x, int z, int radius) const
{
	int low = std::numeric_limits<int>::max();
	int high = std::numeric_limits<int>::min();
	for (int dx = -radius; dx <= radius; ++dx)
		for (int dz = -radius; dz <= radius; ++dz) {
			if (const auto it = deck_low_y_.find({x + dx, z + dz});
					it != deck_low_y_.end()) {
				low = std::min(low, it->second);
				high = std::max(high, deck_y_.at({x + dx, z + dz}));
			}
			if (const auto rail = rail_deck_y_.find({x + dx, z + dz});
					rail != rail_deck_y_.end()) {
				low = std::min(low, rail->second);
				high = std::max(high, rail->second);
			}
		}
	if (low > high)
		return std::nullopt;
	return editor.highest_block_between(x, z, low - 2, high);
}

bool BridgeSurfaceMap::deck_clears(int x, int z, int water_y) const
{
	// Road and rail deck footprints use their respective Rust structure depths.
	const auto deck = deck_low_y_.find({x, z});
	const auto rail = rail_deck_y_.find({x, z});
	const int low =
			deck == deck_low_y_.end() ? std::numeric_limits<int>::max() : deck->second;
	const int rail_low =
			rail == rail_deck_y_.end() ? std::numeric_limits<int>::max() : rail->second;
	if (low == std::numeric_limits<int>::max() &&
			rail_low == std::numeric_limits<int>::max())
		return false;
	return std::min(low == std::numeric_limits<int>::max() ? low : low - 2,
				   rail_low == std::numeric_limits<int>::max() ? rail_low
															   : rail_low - 2) > water_y;
}

bool BridgeSurfaceMap::over_grade_way(int x, int z) const
{
	return grade_crossings_.contains({x, z});
}

}
