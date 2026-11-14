#include "river_bed.h"

#include "block_definitions.h"
#include "ground_generation.h"
#include "land_cover/land_cover.h"
#include "osm_parser.h"
#include "bresenham.h"
#include "element_processing/relation_area.h"
#include "element_processing/way_segments.h"
#include "element_processing/waterways.h"
#include "water_depth.h"
#include "water_detail.h"
#include "../../arnis_ground.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>

namespace arnis::river_bed
{
namespace
{
constexpr std::uint32_t SALT_EARTH = 0x5256'0004;
constexpr std::uint32_t SALT_SAND = 0x5256'0005;
constexpr std::uint32_t SALT_ROCK = 0x5256'0006;
constexpr std::uint32_t SALT_CLAY = 0x5256'0007;
constexpr std::uint32_t SALT_SILT = 0x5256'0008;
constexpr std::uint32_t SALT_ZONE = 0x5256'0003;
constexpr std::uint32_t SALT_POOL = 0x5256'0001;
constexpr std::uint32_t SALT_RIPPLE = 0x5256'0002;
constexpr int DT_MAX = 255;
constexpr double DT_UNITS_PER_BLOCK = 3.0;
constexpr double BAND_MIN = 8.0, BAND_MAX = 32.0;
constexpr double EMBED_MARGIN = 4.0;

struct RiverLine
{
	const ProcessedWay *way{};
	int half_width{};
};

using Span = std::pair<int, int>;

struct ScanlineEdge
{
	double x1{}, z1{}, x2{}, z2{};
};

struct RiverPolygon
{
	std::vector<std::vector<XZPoint>> outers;
	std::vector<std::vector<XZPoint>> inners;
	std::vector<std::vector<ScanlineEdge>> outer_edges;
	std::vector<std::vector<ScanlineEdge>> inner_edges;
	int rows_z0{};
	std::vector<std::vector<Span>> rows;
	int min_x = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
};

bool tag_is(
		const tags_t &tags, const char *key, std::initializer_list<const char *> values)
{
	const auto it = tags.find(key);
	if (it == tags.end())
		return false;
	return std::any_of(
			values.begin(), values.end(), [&](const char *v) { return it->second == v; });
}

bool river_polygon_tags(const tags_t &tags)
{
	return tag_is(tags, "water", {"river", "canal", "stream"}) ||
		   tag_is(tags, "waterway", {"riverbank"});
}

bool nonriver_water_tags(const tags_t &tags)
{
	return !river_polygon_tags(tags) &&
		   (tags.contains("water") || tag_is(tags, "natural", {"water", "bay"}));
}

bool closed_ring(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 4)
		return false;
	const auto &a = nodes.front(), &b = nodes.back();
	return a.id == b.id || (std::abs(a.x - b.x) <= 1 && std::abs(a.z - b.z) <= 1);
}

std::vector<XZPoint> ring_points(const std::vector<ProcessedNode> &nodes)
{
	std::vector<XZPoint> points;
	points.reserve(nodes.size());
	for (const auto &n : nodes)
		points.emplace_back(n.x, n.z);
	return points;
}

void update_bounds(RiverPolygon &poly)
{
	for (const auto &ring : poly.outers)
		for (const auto &p : ring) {
			poly.min_x = std::min(poly.min_x, p.x);
			poly.max_x = std::max(poly.max_x, p.x);
			poly.min_z = std::min(poly.min_z, p.z);
			poly.max_z = std::max(poly.max_z, p.z);
		}
}

std::vector<ScanlineEdge> collect_ring_edges(const std::vector<XZPoint> &ring)
{
	std::vector<ScanlineEdge> edges;
	if (ring.size() < 2)
		return edges;
	for (std::size_t i = 0; i + 1 < ring.size(); ++i)
		if (ring[i].z != ring[i + 1].z)
			edges.push_back({static_cast<double>(ring[i].x),
					static_cast<double>(ring[i].z), static_cast<double>(ring[i + 1].x),
					static_cast<double>(ring[i + 1].z)});
	if (ring.front().z != ring.back().z)
		edges.push_back({static_cast<double>(ring.back().x),
				static_cast<double>(ring.back().z), static_cast<double>(ring.front().x),
				static_cast<double>(ring.front().z)});
	return edges;
}

std::vector<Span> ring_spans_at(
		const std::vector<ScanlineEdge> &edges, int z, int min_x, int max_x)
{
	std::vector<double> crossings;
	for (const auto &edge : edges)
		if ((edge.z1 > z) != (edge.z2 > z)) {
			const double t = (z - edge.z1) / (edge.z2 - edge.z1);
			crossings.push_back(edge.x1 + t * (edge.x2 - edge.x1));
		}
	std::sort(crossings.begin(), crossings.end());
	std::vector<Span> spans;
	for (std::size_t i = 0; i + 1 < crossings.size(); i += 2) {
		const int begin = std::max(min_x, static_cast<int>(std::ceil(crossings[i])));
		const int end = std::min(max_x, static_cast<int>(std::floor(crossings[i + 1])));
		if (begin <= end)
			spans.emplace_back(begin, end);
	}
	return spans;
}

std::vector<Span> union_spans(std::vector<Span> spans)
{
	if (spans.empty())
		return spans;
	std::sort(spans.begin(), spans.end());
	std::vector<Span> merged{spans.front()};
	for (std::size_t i = 1; i < spans.size(); ++i) {
		auto &last = merged.back();
		if (spans[i].first <= last.second + 1)
			last.second = std::max(last.second, spans[i].second);
		else
			merged.push_back(spans[i]);
	}
	return merged;
}

std::vector<Span> subtract_spans(
		const std::vector<Span> &outer, const std::vector<Span> &inner)
{
	std::vector<Span> result;
	std::size_t i = 0;
	for (const auto &[start, end] : outer) {
		std::int64_t cursor = start;
		while (i < inner.size() && inner[i].second < cursor)
			++i;
		for (std::size_t j = i; j < inner.size() && inner[j].first <= end; ++j) {
			if (inner[j].first > cursor)
				result.emplace_back(static_cast<int>(cursor),
						static_cast<int>(
								std::min<std::int64_t>(end, inner[j].first - 1LL)));
			cursor = std::max(cursor, static_cast<std::int64_t>(inner[j].second) + 1);
			if (cursor > end)
				break;
		}
		if (cursor <= end)
			result.emplace_back(static_cast<int>(cursor), end);
	}
	return result;
}

void scan_polygon(RiverPolygon &poly, int world_min_x, int world_max_x, int world_min_z,
		int world_max_z)
{
	poly.rows.clear();
	poly.rows_z0 = std::max(poly.min_z, world_min_z);
	const int rows_z1 = std::min(poly.max_z, world_max_z);
	if (poly.rows_z0 > rows_z1)
		return;
	poly.outer_edges.clear();
	poly.inner_edges.clear();
	poly.outer_edges.reserve(poly.outers.size());
	poly.inner_edges.reserve(poly.inners.size());
	for (const auto &ring : poly.outers)
		poly.outer_edges.push_back(collect_ring_edges(ring));
	for (const auto &ring : poly.inners)
		poly.inner_edges.push_back(collect_ring_edges(ring));
	poly.rows.reserve(static_cast<std::size_t>(rows_z1 - poly.rows_z0 + 1));
	const auto groups = element_processing::hole_groups(poly.outers, poly.inners);
	for (int z = poly.rows_z0; z <= rows_z1; ++z) {
		std::vector<Span> row;
		for (const auto &[outer_indices, inner_indices] : groups) {
			std::vector<Span> outer, inner;
			for (const auto i : outer_indices) {
				auto spans =
						ring_spans_at(poly.outer_edges[i], z, world_min_x, world_max_x);
				outer.insert(outer.end(), spans.begin(), spans.end());
			}
			for (const auto i : inner_indices) {
				auto spans =
						ring_spans_at(poly.inner_edges[i], z, world_min_x, world_max_x);
				inner.insert(inner.end(), spans.begin(), spans.end());
			}
			outer = union_spans(std::move(outer));
			inner = union_spans(std::move(inner));
			const auto group_spans = subtract_spans(outer, inner);
			row.insert(row.end(), group_spans.begin(), group_spans.end());
		}
		poly.rows.push_back(union_spans(std::move(row)));
	}
}

void collect_polygon(const ProcessedWay &way, std::vector<RiverPolygon> &to)
{
	if (!closed_ring(way.nodes))
		return;
	RiverPolygon p;
	p.outers.push_back(ring_points(way.nodes));
	update_bounds(p);
	to.push_back(std::move(p));
}

std::optional<RiverPolygon> collect_relation(const ProcessedRelation &rel)
{
	std::vector<std::vector<ProcessedNode>> outers, inners;
	for (const auto &member : rel.members) {
		if (!member.way)
			continue;
		if (member.role == ProcessedMemberRole::Outer)
			outers.push_back(member.way->nodes);
		else if (member.role == ProcessedMemberRole::Inner)
			inners.push_back(member.way->nodes);
	}
	merge_way_segments(outers);
	merge_way_segments(inners);
	RiverPolygon p;
	for (const auto &ring : outers)
		if (closed_ring(ring))
			p.outers.push_back(ring_points(ring));
	for (const auto &ring : inners)
		if (closed_ring(ring))
			p.inners.push_back(ring_points(ring));
	if (p.outers.empty())
		return std::nullopt;
	update_bounds(p);
	return p;
}

double smoothstep(double u)
{
	return 3.0 * u * u - 2.0 * u * u * u;
}

double river_depth_cap(double hw)
{
	return std::clamp(6.0 * std::pow(hw / 30.0, .7), 1.0, 6.0);
}

double river_profile(double d, double hw)
{
	const double t =
			std::clamp((d - std::min(hw / 4.0, 1.0)) / std::max(hw, 1.0), 0.0, 1.0);
	const double power = 2.0 + std::clamp((hw - 3.0) / 7.0, 0.0, 1.0);
	return river_depth_cap(hw) * (1.0 - std::pow(1.0 - t, power));
}

double natural_depth(double d, int x, int z)
{
	const double pool = .88 + .24 * ground_generation::ground_noise(x, z, 36, SALT_POOL);
	const double ripple =
			(ground_generation::ground_noise(x, z, 5, SALT_RIPPLE) - .5) * .5;
	return d * pool + ripple * std::min(d, 1.0);
}

void tent3(std::vector<float> &values, std::size_t w, std::size_t h)
{
	std::vector<float> tmp(values.size());
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto left = x ? x - 1 : x;
			const auto right = std::min(x + 1, w - 1);
			const auto i = z * w + x;
			tmp[i] = .25f * values[z * w + left] + .5f * values[i] +
					 .25f * values[z * w + right];
		}
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto up = z ? z - 1 : z;
			const auto down = std::min(z + 1, h - 1);
			const auto i = z * w + x;
			values[i] = .25f * tmp[up * w + x] + .5f * tmp[i] + .25f * tmp[down * w + x];
		}
}

std::vector<std::uint8_t> sliding_max_1d(
		const std::uint8_t *values, std::size_t size, std::size_t radius)
{
	std::vector<std::uint8_t> out(size, 0);
	if (size == 0)
		return out;
	std::deque<std::size_t> candidates;
	std::size_t next = 0;
	for (std::size_t i = 0; i < size; ++i) {
		const auto right = std::min(size - 1, i + radius);
		while (next <= right) {
			while (!candidates.empty() && values[candidates.back()] <= values[next])
				candidates.pop_back();
			candidates.push_back(next++);
		}
		const auto left = i > radius ? i - radius : 0;
		while (!candidates.empty() && candidates.front() < left)
			candidates.pop_front();
		out[i] = values[candidates.front()];
	}
	return out;
}

std::vector<std::uint8_t> windowed_max_2d(const std::vector<std::uint8_t> &values,
		std::size_t width, std::size_t height, std::size_t radius)
{
	std::vector<std::uint8_t> horizontal(values.size());
	for (std::size_t z = 0; z < height; ++z) {
		auto row = sliding_max_1d(values.data() + z * width, width, radius);
		std::copy(row.begin(), row.end(), horizontal.begin() + z * width);
	}
	std::vector<std::uint8_t> out(values.size());
	std::vector<std::uint8_t> column(height);
	for (std::size_t x = 0; x < width; ++x) {
		for (std::size_t z = 0; z < height; ++z)
			column[z] = horizontal[z * width + x];
		const auto filtered = sliding_max_1d(column.data(), height, radius);
		for (std::size_t z = 0; z < height; ++z)
			out[z * width + x] = filtered[z];
	}
	return out;
}

int floor_div_tile(int value)
{
	const int quotient = value / TILE_SIDE;
	return quotient - (value % TILE_SIDE < 0 ? 1 : 0);
}

int saturating_offset(int value, int offset)
{
	return static_cast<int>(
			std::clamp<std::int64_t>(static_cast<std::int64_t>(value) + offset,
					std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
}

using TileKey = std::pair<int, int>;

struct TileWork
{
	bool active = false;
	std::vector<std::size_t> lines;
	std::vector<std::size_t> rivers;
	std::vector<std::size_t> lakes;
};

using TileWorks = std::map<TileKey, TileWork>;

template <typename F>
void for_tiles(int x0, int x1, int z0, int z1, int world_min_x, int world_max_x,
		int world_min_z, int world_max_z, F &&f)
{
	x0 = std::max(x0, world_min_x);
	x1 = std::min(x1, world_max_x);
	z0 = std::max(z0, world_min_z);
	z1 = std::min(z1, world_max_z);
	if (x0 > x1 || z0 > z1)
		return;
	const int tx0 = floor_div_tile(x0), tx1 = floor_div_tile(x1);
	const int tz0 = floor_div_tile(z0), tz1 = floor_div_tile(z1);
	for (std::int64_t tz = tz0; tz <= tz1; ++tz)
		for (std::int64_t tx = tx0; tx <= tx1; ++tx)
			f(TileKey{static_cast<int>(tx), static_cast<int>(tz)});
}

void add_unique(std::vector<std::size_t> &items, std::size_t value)
{
	if (std::find(items.begin(), items.end(), value) == items.end())
		items.push_back(value);
}

void rasterize_poly(const RiverPolygon &poly, int min_x, int max_x, int min_z, int max_z,
		const auto &idx, std::vector<std::uint8_t> &mask)
{
	if (poly.rows.empty())
		return;
	const int z0 = std::max(min_z, poly.rows_z0);
	const int z1 = static_cast<int>(std::min<std::int64_t>(
			max_z, static_cast<std::int64_t>(poly.rows_z0) + poly.rows.size() - 1));
	if (min_x > max_x || z0 > z1)
		return;
	for (int z = z0; z <= z1; ++z) {
		for (const auto &[span_min, span_max] :
				poly.rows[static_cast<std::size_t>(z - poly.rows_z0)]) {
			const int a = std::max(min_x, span_min);
			const int b = std::min(max_x, span_max);
			for (int x = a; x <= b; ++x)
				mask[idx(x, z)] = 1;
		}
	}
}

void build_tile(RiverBedField &result, const TileKey &tile, const TileWork &work,
		const std::vector<RiverLine> &lines,
		const std::vector<RiverPolygon> &river_polygons,
		const std::vector<RiverPolygon> &nonriver_polygons, const Ground &ground,
		const water_depth::BigWaterField &legacy, int world_origin_x, int world_origin_z,
		int world_min_x, int world_max_x, int world_min_z, int world_max_z, int halo,
		double scale)
{
	const std::int64_t raw_core_x = static_cast<std::int64_t>(tile.first) * TILE_SIDE;
	const std::int64_t raw_core_z = static_cast<std::int64_t>(tile.second) * TILE_SIDE;
	const int core_min_x =
			static_cast<int>(std::max<std::int64_t>(world_min_x, raw_core_x));
	const int core_min_z =
			static_cast<int>(std::max<std::int64_t>(world_min_z, raw_core_z));
	const int core_max_x = static_cast<int>(
			std::min<std::int64_t>(world_max_x, raw_core_x + TILE_SIDE - 1));
	const int core_max_z = static_cast<int>(
			std::min<std::int64_t>(world_max_z, raw_core_z + TILE_SIDE - 1));
	if (core_min_x > core_max_x || core_min_z > core_max_z)
		return;
	const int min_x =
			static_cast<int>(std::max<std::int64_t>(world_min_x, raw_core_x - halo));
	const int min_z =
			static_cast<int>(std::max<std::int64_t>(world_min_z, raw_core_z - halo));
	const int max_x = static_cast<int>(
			std::min<std::int64_t>(world_max_x, raw_core_x + TILE_SIDE - 1 + halo));
	const int max_z = static_cast<int>(
			std::min<std::int64_t>(world_max_z, raw_core_z + TILE_SIDE - 1 + halo));
	const auto width = static_cast<std::size_t>(max_x - min_x + 1);
	const auto height = static_cast<std::size_t>(max_z - min_z + 1);
	if (!width || !height || width > std::numeric_limits<std::size_t>::max() / height)
		return;
	const auto count = width * height;
	std::vector<std::uint8_t> line_mask(count, 0), poly_mask(count, 0),
			nonriver_mask(count, 0), lc_water(count, 0), removed(count, 0),
			hw_line(count, 0), dist(count, 0), foreign_dist(count, DT_MAX);
	const auto idx = [&](int x, int z) {
		return static_cast<std::size_t>(z - min_z) * width + (x - min_x);
	};
	const auto inside = [&](int x, int z) {
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
	};
	for (int z = min_z; z <= max_z; ++z)
		for (int x = min_x; x <= max_x; ++x)
			lc_water[idx(x, z)] = ground.cover_class({x - world_origin_x,
										  z - world_origin_z}) == land_cover::LC_WATER;
	for (const auto line_index : work.lines) {
		const auto &line = lines[line_index];
		const int radius = line.half_width + 1;
		const auto hw = static_cast<std::uint8_t>(std::clamp(radius + 1, 1, 255));
		for (std::size_t k = 1; k < line.way->nodes.size(); ++k) {
			const auto &a = line.way->nodes[k - 1], &b = line.way->nodes[k];
			if (std::max(a.x, b.x) + radius < min_x ||
					std::min(a.x, b.x) - radius > max_x ||
					std::max(a.z, b.z) + radius < min_z ||
					std::min(a.z, b.z) - radius > max_z)
				continue;
			for (const auto &[cx, _, cz] : bresenham_line(a.x, 0, a.z, b.x, 0, b.z))
				for (int x = cx - radius; x <= cx + radius; ++x)
					for (int z = cz - radius; z <= cz + radius; ++z)
						if (inside(x, z)) {
							const auto i = idx(x, z);
							line_mask[i] = 1;
							hw_line[i] = std::max(hw_line[i], hw);
						}
		}
	}
	for (const auto i : work.rivers)
		rasterize_poly(river_polygons[i], min_x, max_x, min_z, max_z, idx, poly_mask);
	bool has_river_in_core = false;
	for (int z = core_min_z; z <= core_max_z && !has_river_in_core; ++z)
		for (int x = core_min_x; x <= core_max_x; ++x)
			if (line_mask[idx(x, z)] || poly_mask[idx(x, z)]) {
				has_river_in_core = true;
				break;
			}
	if (!has_river_in_core)
		return;
	for (const auto i : work.lakes)
		rasterize_poly(
				nonriver_polygons[i], min_x, max_x, min_z, max_z, idx, nonriver_mask);
	for (std::size_t i = 0; i < count; ++i)
		if (line_mask[i] && nonriver_mask[i]) {
			line_mask[i] = 0;
			removed[i] = 1;
		}
	const bool has_line = std::find(line_mask.begin(), line_mask.end(),
								  std::uint8_t{1}) != line_mask.end();
	if (has_line) {
		for (std::size_t i = 0; i < count; ++i)
			dist[i] = lc_water[i] ? DT_MAX : 0;
		water_depth::chamfer_3_4_dt(dist, width, height);
		for (std::size_t i = 0; i < count; ++i) {
			if (!line_mask[i] || poly_mask[i])
				continue;
			if (dist[i] / DT_UNITS_PER_BLOCK > hw_line[i] + EMBED_MARGIN) {
				line_mask[i] = 0;
				removed[i] = 1;
			}
		}
	}
	for (std::size_t i = 0; i < count; ++i)
		dist[i] = line_mask[i] || poly_mask[i] ? DT_MAX : 0;
	water_depth::chamfer_3_4_dt(dist, width, height);
	std::vector<std::uint8_t> distance_in_mask(count, 0);
	for (std::size_t i = 0; i < count; ++i)
		if (line_mask[i] || poly_mask[i])
			distance_in_mask[i] = static_cast<std::uint8_t>(
					std::clamp(std::lround(dist[i] / DT_UNITS_PER_BLOCK), 0L, 255L));
	const auto poly_half_width =
			std::find(poly_mask.begin(), poly_mask.end(), std::uint8_t{1}) !=
							poly_mask.end()
					? windowed_max_2d(distance_in_mask, width, height,
							  static_cast<std::size_t>(std::clamp(
									  static_cast<int>(std::llround(16.0 * scale)), 1,
									  48)))
					: std::vector<std::uint8_t>{};
	std::vector<float> hw_field(count, 0.f), depth_field(count, 0.f);
	for (std::size_t i = 0; i < count; ++i) {
		if (!line_mask[i] && !poly_mask[i])
			continue;
		const double d = dist[i] / DT_UNITS_PER_BLOCK;
		const double hw = std::max({1.0, double(hw_line[i]),
				poly_mask[i] ? double(std::max(poly_half_width[i], distance_in_mask[i]))
							 : 0.0});
		hw_field[i] = static_cast<float>(hw);
		depth_field[i] = static_cast<float>(river_profile(d, hw));
	}
	tent3(depth_field, width, height);
	for (std::size_t i = 0; i < count; ++i) {
		if (line_mask[i] || poly_mask[i])
			continue;
		const int x = min_x + static_cast<int>(i % width);
		const int z = min_z + static_cast<int>(i / width);
		bool adjacent = false;
		for (int dz = -1; dz <= 1 && !adjacent; ++dz)
			for (int dx = -1; dx <= 1; ++dx)
				if ((dx || dz) && inside(x + dx, z + dz) &&
						(line_mask[idx(x + dx, z + dz)] ||
								poly_mask[idx(x + dx, z + dz)])) {
					adjacent = true;
					break;
				}
		if (adjacent && (nonriver_mask[i] || removed[i] || lc_water[i]))
			foreign_dist[i] = 0;
	}
	const bool has_foreign = std::find(foreign_dist.begin(), foreign_dist.end(),
									 std::uint8_t{0}) != foreign_dist.end();
	if (has_foreign)
		water_depth::chamfer_3_4_dt(foreign_dist, width, height);
	for (int z = core_min_z; z <= core_max_z; ++z)
		for (int x = core_min_x; x <= core_max_x; ++x) {
			const auto i = idx(x, z);
			if (!line_mask[i] && !poly_mask[i])
				continue;
			const double river = natural_depth(depth_field[i], x, z);
			double value = river;
			if (has_foreign) {
				const double band = std::clamp(2.0 * hw_field[i], BAND_MIN, BAND_MAX);
				const double m = foreign_dist[i] / DT_UNITS_PER_BLOCK;
				const double weight = smoothstep(std::clamp(m / band, 0.0, 1.0));
				const double old = legacy.legacy_depth_at(x, z);
				value = old + (river - old) * weight;
			}
			result.set(x, z,
					static_cast<std::uint8_t>(std::clamp(std::lround(value), 0L, 6L)));
		}
}
}

std::optional<int> RiverBedField::depth_override(int x, int z) const
{
	const int tx = floor_div_tile(x), tz = floor_div_tile(z);
	const auto key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tx)) << 32) |
					 static_cast<std::uint32_t>(tz);
	const auto it = tiles_.find(static_cast<std::int64_t>(key));
	if (it == tiles_.end())
		return std::nullopt;
	const int lx = x - tx * TILE_SIDE, lz = z - tz * TILE_SIDE;
	const auto depth = it->second[static_cast<std::size_t>(lz) * TILE_SIDE + lx];
	return depth == NO_RIVER ? std::nullopt : std::optional<int>(depth);
}

void RiverBedField::set(int x, int z, std::uint8_t depth)
{
	const int tx = floor_div_tile(x), tz = floor_div_tile(z);
	const auto key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tx)) << 32) |
					 static_cast<std::uint32_t>(tz);
	auto [it, inserted] = tiles_.try_emplace(static_cast<std::int64_t>(key));
	if (inserted)
		it->second.assign(static_cast<std::size_t>(TILE_SIDE) * TILE_SIDE, NO_RIVER);
	const int lx = x - tx * TILE_SIDE, lz = z - tz * TILE_SIDE;
	it->second[static_cast<std::size_t>(lz) * TILE_SIDE + lx] = depth;
	if (!has_bounds_) {
		min_x_ = max_x_ = x;
		min_z_ = max_z_ = z;
		has_bounds_ = true;
	} else {
		min_x_ = std::min(min_x_, x);
		min_z_ = std::min(min_z_, z);
		max_x_ = std::max(max_x_, x);
		max_z_ = std::max(max_z_, z);
	}
}

std::pair<Block, Block> bed_blocks(int x, int z, int depth, bool sandy)
{
	using namespace block_definitions;
	const double dither =
			static_cast<double>(land_cover::coord_hash(x ^ 0x52B3, z ^ 0x1F7D) % 1000) /
					1000.0 -
			.5;
	const double zone = depth +
						(ground_generation::ground_noise(x, z, 9, SALT_ZONE) - .5) * 2.4 +
						dither * .8;
	const double s = std::clamp((zone - .5) / 4.0, 0.0, 1.0);
	const auto field = [&](int sample_scale, std::uint32_t salt) {
		return ground_generation::ground_noise(x, z, sample_scale, salt) + dither * .14;
	};
	if (zone > .8 && field(10, SALT_CLAY) > .94)
		return {CLAY, STONE};
	const double earth = .56 - .52 * std::pow(s, .7);
	if (field(9, SALT_EARTH) < (sandy ? earth * .3 : earth)) {
		const double silt = ground_generation::ground_noise(x, z, 3, SALT_SILT);
		const Block top = silt < .55 ? DIRT : silt < .88 ? COARSE_DIRT : MUD;
		return {top, DIRT};
	}
	const double sand = .25 + .6 * s * (1.0 - s) - .13 * s;
	if (field(10, SALT_SAND) < (sandy ? .85 : sand))
		return {SAND, STONE};
	if (field(6, SALT_ROCK) > 1.0 - .12 * s) {
		const auto pick = land_cover::coord_hash(x ^ 0x0C0B, z ^ 0x7E55) % 20;
		const Block top = pick <= 10 ? STONE : pick <= 15 ? ANDESITE : MOSSY_COBBLESTONE;
		return {top, STONE};
	}
	return {GRAVEL, STONE};
}

RiverBedField compute_river_bed_field(RiverBed mode,
		const std::vector<ProcessedElement> &elements, const Ground &ground,
		const water_depth::BigWaterField &legacy, int world_origin_x, int world_origin_z,
		int world_min_x, int world_max_x, int world_min_z, int world_max_z, double scale,
		std::optional<int> channel_width_cap)
{
	RiverBedField result;
	if (mode == RiverBed::Off || world_min_x > world_max_x || world_min_z > world_max_z)
		return result;
	std::vector<RiverLine> lines;
	std::vector<RiverPolygon> river_polygons, nonriver_polygons;
	const auto collect_way = [&](const ProcessedWay &way) {
		if (river_polygon_tags(way.tags)) {
			collect_polygon(way, river_polygons);
			return;
		}
		if (nonriver_water_tags(way.tags)) {
			collect_polygon(way, nonriver_polygons);
			return;
		}
		const auto it = way.tags.find("waterway");
		if (it == way.tags.end() || way.nodes.size() < 2 ||
				!tag_is(way.tags, "waterway",
						{"river", "stream", "canal", "brook", "fairway", "flowline"}) ||
				!waterways::is_channel_waterway(it->second) ||
				waterways::is_underground_waterway(way.tags))
			return;
		const int width =
				waterways::capped_waterway_width(it->second, way.tags, channel_width_cap);
		lines.push_back({&way, width / 2});
	};
	for (const auto &element : elements) {
		if (element.is_way())
			collect_way(element.as_way());
		else if (element.is_relation()) {
			const auto &rel = element.as_relation();
			auto *target = river_polygon_tags(rel.tags)	   ? &river_polygons
						   : nonriver_water_tags(rel.tags) ? &nonriver_polygons
														   : nullptr;
			if (target)
				if (auto p = collect_relation(rel))
					target->push_back(std::move(*p));
		}
	}
	if (lines.empty() && river_polygons.empty())
		return result;
	const int halo = static_cast<int>(std::ceil(DT_MAX / DT_UNITS_PER_BLOCK)) +
					 std::clamp(static_cast<int>(std::llround(16.0 * scale)), 1, 48) + 3;
	TileWorks work;
	for (std::size_t i = 0; i < lines.size(); ++i) {
		const auto &line = lines[i];
		const int stamp_radius = line.half_width + 1;
		for (std::size_t k = 1; k < line.way->nodes.size(); ++k) {
			const auto &a = line.way->nodes[k - 1], &b = line.way->nodes[k];
			for_tiles(saturating_offset(std::min(a.x, b.x), -stamp_radius),
					saturating_offset(std::max(a.x, b.x), stamp_radius),
					saturating_offset(std::min(a.z, b.z), -stamp_radius),
					saturating_offset(std::max(a.z, b.z), stamp_radius), world_min_x,
					world_max_x, world_min_z, world_max_z,
					[&](const TileKey &tile) { work[tile].active = true; });
			const int margin = saturating_offset(stamp_radius, halo);
			for_tiles(saturating_offset(std::min(a.x, b.x), -margin),
					saturating_offset(std::max(a.x, b.x), margin),
					saturating_offset(std::min(a.z, b.z), -margin),
					saturating_offset(std::max(a.z, b.z), margin), world_min_x,
					world_max_x, world_min_z, world_max_z,
					[&](const TileKey &tile) { add_unique(work[tile].lines, i); });
		}
	}
	for (auto &poly : river_polygons)
		scan_polygon(poly, world_min_x, world_max_x, world_min_z, world_max_z);
	for (std::size_t i = 0; i < river_polygons.size(); ++i) {
		const auto &poly = river_polygons[i];
		for (std::size_t row = 0; row < poly.rows.size(); ++row) {
			const int z = poly.rows_z0 + static_cast<int>(row);
			for (const auto &[x0, x1] : poly.rows[row])
				for_tiles(x0, x1, z, z, world_min_x, world_max_x, world_min_z,
						world_max_z,
						[&](const TileKey &tile) { work[tile].active = true; });
		}
	}
	for (std::size_t i = 0; i < river_polygons.size(); ++i) {
		const auto &poly = river_polygons[i];
		for_tiles(saturating_offset(poly.min_x, -halo),
				saturating_offset(poly.max_x, halo), saturating_offset(poly.min_z, -halo),
				saturating_offset(poly.max_z, halo), world_min_x, world_max_x,
				world_min_z, world_max_z,
				[&](const TileKey &tile) { add_unique(work[tile].rivers, i); });
	}
	for (std::size_t i = 0; i < nonriver_polygons.size(); ++i) {
		const auto &poly = nonriver_polygons[i];
		bool near_river = false;
		for_tiles(saturating_offset(poly.min_x, -halo),
				saturating_offset(poly.max_x, halo), saturating_offset(poly.min_z, -halo),
				saturating_offset(poly.max_z, halo), world_min_x, world_max_x,
				world_min_z, world_max_z, [&](const TileKey &tile) {
					auto it = work.find(tile);
					near_river |= it != work.end() && it->second.active;
				});
		if (!near_river)
			continue;
		auto &scanned = nonriver_polygons[i];
		scan_polygon(scanned, world_min_x, world_max_x, world_min_z, world_max_z);
		for_tiles(saturating_offset(poly.min_x, -halo),
				saturating_offset(poly.max_x, halo), saturating_offset(poly.min_z, -halo),
				saturating_offset(poly.max_z, halo), world_min_x, world_max_x,
				world_min_z, world_max_z, [&](const TileKey &tile) {
					auto it = work.find(tile);
					if (it != work.end() && it->second.active)
						add_unique(it->second.lakes, i);
				});
	}
	for (const auto &[tile, tile_work] : work)
		if (tile_work.active)
			build_tile(result, tile, tile_work, lines, river_polygons, nonriver_polygons,
					ground, legacy, world_origin_x, world_origin_z, world_min_x,
					world_max_x, world_min_z, world_max_z, halo, scale);
	return result;
}
}
