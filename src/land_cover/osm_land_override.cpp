#include "osm_land_override.h"

#include "../../arnis_adapter.h"
#include "../bresenham.h"
#include "../clipping.h"
#include "../element_processing/bridges.h"
#include "../element_processing/way_segments.h"
#include "../element_processing/waterways.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <tuple>

namespace arnis::highways
{
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
}

namespace arnis::land_cover
{
namespace
{
bool bit(const std::vector<std::uint64_t> &mask, std::size_t index)
{
	return (mask[index >> 6] >> (index & 63)) & 1U;
}

void set_bit(std::vector<std::uint64_t> &mask, std::size_t index)
{
	mask[index >> 6] |= std::uint64_t{1} << (index & 63);
}

bool enabled_tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second != "no" && it->second != "0" &&
		   it->second != "false";
}

bool is_water_area_way(const tags_t &tags)
{
	const auto natural = tags.get("natural"), landuse = tags.get("landuse"),
			   waterway = tags.get("waterway");
	return natural == "water" || landuse == "reservoir" || waterway == "dock" ||
		   waterway == "riverbank" || enabled_tag(tags, "water");
}

bool is_water_area_relation(const tags_t &tags)
{
	const auto natural = tags.get("natural"), landuse = tags.get("landuse");
	return natural == "water" || natural == "bay" || landuse == "reservoir" ||
		   enabled_tag(tags, "water");
}

bool building(const tags_t &tags)
{
	return enabled_tag(tags, "building") || tags.contains("building:part");
}

std::optional<int> over_water_half_width(const ProcessedWay &way, double scale)
{
	const auto man_made = way.tags.get("man_made");
	const bool structure = man_made == "pier" || man_made == "breakwater" ||
						   man_made == "groyne" || man_made == "dolphin" ||
						   man_made == "quay" || bridges::is_bridge_way(way) ||
						   way.tags.get("floating") == "yes";
	if (!structure)
		return std::nullopt;
	const auto highway = way.tags.get("highway");
	if (highway.empty())
		return 2;
	return std::max(2, arnis::highways::highway_block_range(highway, way.tags, scale));
}

std::optional<int> land_line_half_width(const ProcessedWay &way, double scale)
{
	if (enabled_tag(way.tags, "tunnel") || way.tags.get("area") == "yes" ||
			way.tags.contains("man_made"))
		return std::nullopt;
	const auto highway = way.tags.get("highway");
	if (!highway.empty()) {
		if (highway == "proposed" || highway == "construction" || highway == "raceway")
			return std::nullopt;
		return std::max(
				0, arnis::highways::highway_block_range(highway, way.tags, scale));
	}
	const auto railway = way.tags.get("railway");
	if ((railway == "rail" || railway == "light_rail" || railway == "tram" ||
				railway == "subway" || railway == "narrow_gauge" ||
				railway == "monorail") &&
			way.tags.get("location") != "underground")
		return 1;
	return std::nullopt;
}

bool closed(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return false;
	const auto &a = nodes.front(), &b = nodes.back();
	return a.id == b.id || (std::abs(static_cast<std::int64_t>(a.x) - b.x) <= 1 &&
								   std::abs(static_cast<std::int64_t>(a.z) - b.z) <= 1);
}

struct GridMap
{
	int min_x, min_z;
	double sx, sz;
	std::size_t width, height;
	int x(int value) const { return int(std::lround((value - min_x) * sx)); }
	int z(int value) const { return int(std::lround((value - min_z) * sz)); }
};

std::vector<std::vector<ProcessedNode>> relation_rings(
		const ProcessedRelation &relation, const XZBBox &bbox)
{
	std::vector<std::vector<ProcessedNode>> outer, inner;
	for (const auto &member : relation.members) {
		if (member.way.nodes.size() < 2)
			continue;
		if (member.role == ProcessedMemberRole::Outer)
			outer.push_back(member.way.nodes);
		else if (member.role == ProcessedMemberRole::Inner)
			inner.push_back(member.way.nodes);
	}
	merge_way_segments(outer);
	merge_way_segments(inner);
	std::vector<std::vector<ProcessedNode>> result;
	const auto append_closed_clipped = [&](const auto &rings) {
		for (const auto &ring : rings) {
			if (!closed(ring))
				continue;
			if (auto clipped = clipping::clip_water_ring_to_bbox(ring, bbox))
				result.push_back(std::move(*clipped));
		}
	};
	append_closed_clipped(outer);
	append_closed_clipped(inner);
	return result;
}

void stamp_line(std::vector<std::uint64_t> &mask, const std::vector<ProcessedNode> &nodes,
		int half_width, const GridMap &map, const XZBBox &bbox)
{
	half_width = std::clamp(half_width, 0, waterways::MAX_WATERWAY_WIDTH);
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		const auto &a = nodes[i - 1];
		const auto &b = nodes[i];
		if (std::int64_t(std::max(a.x, b.x)) + half_width < bbox.min_x() ||
				std::int64_t(std::min(a.x, b.x)) - half_width > bbox.max_x() ||
				std::int64_t(std::max(a.z, b.z)) + half_width < bbox.min_z() ||
				std::int64_t(std::min(a.z, b.z)) - half_width > bbox.max_z())
			continue;
		for (const auto &[x, y, z] :
				bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)y;
			for (int dz = -half_width; dz <= half_width; ++dz)
				for (int dx = -half_width; dx <= half_width; ++dx) {
					const auto wx = std::int64_t(x) + dx;
					const auto wz = std::int64_t(z) + dz;
					if (wx < std::numeric_limits<int>::min() ||
							wx > std::numeric_limits<int>::max() ||
							wz < std::numeric_limits<int>::min() ||
							wz > std::numeric_limits<int>::max())
						continue;
					const int gx = map.x(static_cast<int>(wx));
					const int gz = map.z(static_cast<int>(wz));
					if (gx >= 0 && gz >= 0 && gx < int(map.width) && gz < int(map.height))
						set_bit(mask, std::size_t(gz) * map.width + gx);
				}
		}
	}
}

void fill_rings(std::vector<std::uint64_t> &mask,
		const std::vector<std::vector<ProcessedNode>> &rings, const GridMap &map)
{
	for (int z = 0; z < int(map.height); ++z) {
		std::vector<double> crossings;
		for (const auto &nodes : rings) {
			if (!closed(nodes))
				continue;
			for (std::size_t i = 0, j = nodes.size() - 1; i < nodes.size(); j = i++) {
				const auto &a = nodes[j];
				const auto &b = nodes[i];
				const double ax = (a.x - map.min_x) * map.sx;
				const double az = (a.z - map.min_z) * map.sz;
				const double bx = (b.x - map.min_x) * map.sx;
				const double bz = (b.z - map.min_z) * map.sz;
				if ((az > z) != (bz > z))
					crossings.push_back(ax + (z - az) * (bx - ax) / (bz - az));
			}
		}
		std::sort(crossings.begin(), crossings.end());
		for (std::size_t i = 1; i < crossings.size(); i += 2) {
			const int begin = std::max(0, int(std::ceil(crossings[i - 1])));
			const int end = std::min(int(map.width) - 1, int(std::floor(crossings[i])));
			for (int x = begin; x <= end; ++x)
				set_bit(mask, std::size_t(z) * map.width + x);
		}
	}
}

void fill_ring(std::vector<std::uint64_t> &mask, const std::vector<ProcessedNode> &ring,
		const GridMap &map)
{
	fill_rings(mask, std::vector<std::vector<ProcessedNode>>{ring}, map);
}

std::vector<std::uint64_t> dilate(const std::vector<std::uint64_t> &seeds,
		const std::vector<std::vector<std::uint8_t>> &grid, std::size_t width,
		std::size_t height, int distance, bool through_water)
{
	const std::size_t count = width * height;
	std::vector<std::uint64_t> seen = seeds, out((count + 63) / 64);
	std::deque<std::pair<std::size_t, int>> queue;
	for (std::size_t i = 0; i < count; ++i)
		if (bit(seeds, i))
			queue.emplace_back(i, 0);
	constexpr std::array<std::pair<int, int>, 4> neighbours{
			{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	while (!queue.empty()) {
		const auto [index, depth] = queue.front();
		queue.pop_front();
		if (depth >= distance)
			continue;
		const int x = index % width, z = index / width;
		for (const auto &[dx, dz] : neighbours) {
			const int nx = x + dx, nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
				continue;
			const auto next = std::size_t(nz) * width + nx;
			if (bit(seen, next) || (grid[nz][nx] == LC_WATER) != through_water)
				continue;
			set_bit(seen, next);
			set_bit(out, next);
			queue.emplace_back(next, depth + 1);
		}
	}
	return out;
}

std::optional<std::uint8_t> nearest_land(
		const LandCoverData &data, int x, int z, int radius)
{
	for (int r = 1; r <= radius; ++r)
		for (int dz = -r; dz <= r; ++dz)
			for (int dx = -r; dx <= r; ++dx) {
				if (std::abs(dx) != r && std::abs(dz) != r)
					continue;
				const int nx = x + dx, nz = z + dz;
				if (nx >= 0 && nz >= 0 && nx < int(data.width) && nz < int(data.height)) {
					const auto value = data.grid[nz][nx];
					if (value && value != LC_WATER)
						return value;
				}
			}
	return std::nullopt;
}
}

void apply_osm_land_override(LandCoverData &data, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2)
		return;
	if (std::none_of(data.grid.begin(), data.grid.end(), [](const auto &row) {
			return std::find(row.begin(), row.end(), LC_WATER) != row.end();
		}))
		return;
	const std::size_t count = data.width * data.height;
	GridMap map{bbox.min_x(), bbox.min_z(), double(data.width - 1) / (world_width - 1),
			double(data.height - 1) / (world_height - 1), data.width, data.height};
	std::vector<std::uint64_t> water_area((count + 63) / 64),
			water_line((count + 63) / 64), land((count + 63) / 64),
			over_water((count + 63) / 64);
	for (const auto &element : elements) {
		if (element.is_relation()) {
			const auto &relation = element.as_relation();
			std::vector<std::uint64_t> *target = nullptr;
			if (is_water_area_relation(relation.tags))
				target = &water_area;
			else if (building(relation.tags))
				target = &land;
			if (target) {
				fill_rings(*target, relation_rings(relation, bbox), map);
			}
			continue;
		}
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			continue;
		if (const auto half_width = over_water_half_width(way, scale)) {
			stamp_line(over_water, way.nodes, *half_width, map, bbox);
			if (closed(way.nodes))
				if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox))
					fill_ring(over_water, *ring, map);
			continue;
		}
		if (is_water_area_way(way.tags)) {
			if (closed(way.nodes))
				if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox))
					fill_ring(water_area, *ring, map);
			continue;
		}
		const auto waterway = way.tags.get("waterway");
		if (!waterway.empty()) {
			if (waterways::is_channel_waterway(waterway) &&
					!waterways::is_underground_waterway(way.tags)) {
				const int half_width =
						std::max(0, waterways::waterway_width(waterway, way.tags) / 2);
				stamp_line(water_line, way.nodes, half_width, map, bbox);
			}
			continue;
		}
		if (const auto half_width = land_line_half_width(way, scale)) {
			stamp_line(land, way.nodes, *half_width, map, bbox);
			continue;
		}
		if (building(way.tags) && closed(way.nodes))
			if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox))
				fill_ring(land, *ring, map);
	}
	for (std::size_t i = 0; i < land.size(); ++i)
		land[i] &= ~over_water[i];
	const bool has_land =
			std::any_of(land.begin(), land.end(), [](auto word) { return word != 0; });
	const bool has_water_area = std::any_of(
			water_area.begin(), water_area.end(), [](auto word) { return word != 0; });
	if (!has_land && !has_water_area)
		return;
	std::vector<std::uint64_t> land_seed((count + 63) / 64);
	for (std::size_t i = 0; i < count; ++i)
		if (data.grid[i / data.width][i % data.width] != LC_WATER)
			set_bit(land_seed, i);
	// Match Rust's band_cells: invalid resolutions use the minimum band, while
	// very large finite resolutions saturate before integer conversion.
	const double raw_band = 15.0 * data.cells_per_meter;
	const int band =
			!std::isfinite(data.cells_per_meter) || data.cells_per_meter <= 0.0 ? 1
			: raw_band >= 64.0													? 64
							   : std::max(1, static_cast<int>(std::lround(raw_band)));
	const auto rim = dilate(land_seed, data.grid, data.width, data.height, band, true);
	const auto past_outline =
			has_water_area
					? dilate(water_area, data.grid, data.width, data.height, band, true)
					: std::vector<std::uint64_t>((count + 63) / 64);
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> changes;
	for (std::size_t i = 0; i < count; ++i) {
		if (!bit(rim, i) || bit(water_area, i) || bit(water_line, i) ||
				(!bit(land, i) && !bit(past_outline, i)))
			continue;
		const int x = i % data.width, z = i / data.width;
		if (const auto cls = nearest_land(data, x, z, band + 2))
			changes.emplace_back(x, z, *cls);
	}
	for (const auto &[x, z, cls] : changes)
		data.grid[z][x] = cls;
	if (!changes.empty()) {
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.water_blend_grid.clear();
	}
}
}
