#include "osm_land_override.h"

#include "../../arnis_adapter.h"
#include "../bresenham.h"
#include "../clipping.h"
#include "../element_processing/bridges.h"
#include "../element_processing/way_segments.h"
#include "../element_processing/waterways.h"
#include "../element_processing/swimming_pools.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
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
	if (!way.tags.contains("highway"))
		return 2;
	return std::max(2, arnis::highways::highway_block_range(highway, way.tags, scale));
}

std::optional<int> land_line_half_width(const ProcessedWay &way, double scale)
{
	if (enabled_tag(way.tags, "tunnel") || way.tags.get("area") == "yes" ||
			way.tags.contains("man_made"))
		return std::nullopt;
	const auto highway = way.tags.get("highway");
	if (way.tags.contains("highway")) {
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
	double ox = 0.0, oz = 0.0;
	double gx(int value) const { return (value - min_x) * sx - ox; }
	double gz(int value) const { return (value - min_z) * sz - oz; }
	int x(int value) const { return int(std::lround(gx(value))); }
	int z(int value) const { return int(std::lround(gz(value))); }
};

std::vector<std::vector<ProcessedNode>> relation_rings(
		const ProcessedRelation &relation, const XZBBox &bbox)
{
	std::vector<std::vector<ProcessedNode>> outer, inner;
	for (const auto &member : relation.members) {
		if (!member.way || member.way->nodes.size() < 2)
			continue;
		if (member.role == ProcessedMemberRole::Outer)
			outer.push_back(member.way->nodes);
		else if (member.role == ProcessedMemberRole::Inner)
			inner.push_back(member.way->nodes);
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
	// Match Rust's half-open scanline fill exactly. Sampling crossings at integer
	// rows with an `az > z` predicate shifts edges that land on grid rows and can
	// change which land-cover cells are considered inside an OSM water/building
	// ring. Collect only rows touched by each non-horizontal edge, then fill the
	// inclusive horizontal span for each paired crossing.
	std::vector<std::vector<double>> rows(map.height);
	for (const auto &nodes : rings) {
		if (!closed(nodes))
			continue;
		for (std::size_t i = 0; i < nodes.size(); ++i) {
			const auto &a = nodes[i];
			const auto &b = nodes[(i + 1) % nodes.size()];
			const double ay = map.gz(a.z);
			const double by = map.gz(b.z);
			if (ay == by)
				continue;
			const double ax = map.gx(a.x);
			const double bx = map.gx(b.x);
			const double y_lo = std::min(ay, by);
			const double y_hi = std::max(ay, by);
			const auto z_start = static_cast<std::int64_t>(std::ceil(y_lo));
			const auto z_end =
					std::min<std::int64_t>(static_cast<std::int64_t>(std::ceil(y_hi)),
							static_cast<std::int64_t>(map.height));
			for (auto z = std::max<std::int64_t>(0, z_start); z < z_end; ++z) {
				const double t = (static_cast<double>(z) - ay) / (by - ay);
				rows[static_cast<std::size_t>(z)].push_back(ax + t * (bx - ax));
			}
		}
	}
	for (std::size_t z = 0; z < rows.size(); ++z) {
		auto &crossings = rows[z];
		std::sort(crossings.begin(), crossings.end());
		for (std::size_t i = 0; i + 1 < crossings.size(); i += 2) {
			const auto begin = std::max<std::int64_t>(
					0, static_cast<std::int64_t>(std::ceil(crossings[i])));
			const auto end =
					std::min<std::int64_t>(static_cast<std::int64_t>(map.width) - 1,
							static_cast<std::int64_t>(std::floor(crossings[i + 1])));
			for (auto x = begin; x <= end; ++x)
				set_bit(mask, z * map.width + static_cast<std::size_t>(x));
		}
	}
}

void fill_ring(std::vector<std::uint64_t> &mask, const std::vector<ProcessedNode> &ring,
		const GridMap &map)
{
	fill_rings(mask, std::vector<std::vector<ProcessedNode>>{ring}, map);
}

void pool_changes(const LandCoverData &data,
		const std::vector<std::vector<ProcessedNode>> &rings, const GridMap &map,
		const std::vector<std::uint64_t> &water_area,
		const std::vector<std::uint64_t> &water_line,
		std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> &changes)
{
	constexpr std::size_t max_window_cells = 1U << 20;
	double min_x = std::numeric_limits<double>::max();
	double max_x = std::numeric_limits<double>::lowest();
	double min_z = std::numeric_limits<double>::max();
	double max_z = std::numeric_limits<double>::lowest();
	for (const auto &ring : rings)
		for (const auto &node : ring) {
			min_x = std::min(min_x, map.gx(node.x));
			max_x = std::max(max_x, map.gx(node.x));
			min_z = std::min(min_z, map.gz(node.z));
			max_z = std::max(max_z, map.gz(node.z));
		}
	if (min_x > max_x)
		return;
	const auto margin = std::max<std::int64_t>(
								1, static_cast<std::int64_t>(
										   std::ceil(2.0 * std::max(map.sx, map.sz)))) +
						1;
	const auto clamp_cell = [](double v, std::int64_t low, std::int64_t high) {
		return std::clamp(static_cast<std::int64_t>(v), low, high);
	};
	const auto x0 = clamp_cell(
			std::floor(min_x) - margin, 0, static_cast<std::int64_t>(map.width) - 1);
	const auto z0 = clamp_cell(
			std::floor(min_z) - margin, 0, static_cast<std::int64_t>(map.height) - 1);
	const auto x1 = clamp_cell(
			std::ceil(max_x) + margin, 0, static_cast<std::int64_t>(map.width) - 1);
	const auto z1 = clamp_cell(
			std::ceil(max_z) + margin, 0, static_cast<std::int64_t>(map.height) - 1);
	if (x0 > x1 || z0 > z1)
		return;
	const auto width = static_cast<std::size_t>(x1 - x0 + 1);
	const auto height = static_cast<std::size_t>(z1 - z0 + 1);
	if (width > max_window_cells / height)
		return;
	GridMap window = map;
	window.width = width;
	window.height = height;
	window.ox = static_cast<double>(x0);
	window.oz = static_cast<double>(z0);
	std::vector<std::uint64_t> inside((width * height + 63) / 64);
	fill_rings(inside, rings, window);
	const auto m = static_cast<std::size_t>(margin);
	std::vector<std::uint8_t> row_hit(width * height, 0), near_inside(width * height, 0);
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const auto first = x > m ? x - m : 0;
			const auto last = std::min(width - 1, x + m);
			for (std::size_t ix = first; ix <= last; ++ix)
				if (bit(inside, z * width + ix)) {
					row_hit[z * width + x] = 1;
					break;
				}
		}
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const auto first = z > m ? z - m : 0;
			const auto last = std::min(height - 1, z + m);
			for (std::size_t iz = first; iz <= last; ++iz)
				if (row_hit[iz * width + x]) {
					near_inside[z * width + x] = 1;
					break;
				}
		}
	const int radius = static_cast<int>(
			std::min<std::size_t>((std::max(width, height) / 2 + m + 2), 64));
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			if (!near_inside[z * width + x])
				continue;
			const auto gx = static_cast<std::size_t>(x0) + x;
			const auto gz = static_cast<std::size_t>(z0) + z;
			const auto index = gz * map.width + gx;
			if (data.grid[gz][gx] != LC_WATER || bit(water_area, index) ||
					bit(water_line, index))
				continue;
			const auto cls =
					nearest_land_class(data.grid, map.width, map.height, gx, gz, radius)
							.value_or(LC_BUILT_UP);
			changes.emplace_back(gx, gz, cls);
		}
}

std::vector<std::uint64_t> dilate(const std::vector<std::uint64_t> &seeds,
		const std::vector<std::vector<std::uint8_t>> &grid, std::size_t width,
		std::size_t height, int distance, bool through_water)
{
	const std::size_t count = width * height;
	std::vector<std::uint64_t> seen = seeds, out((count + 63) / 64);
	std::vector<std::uint32_t> frontier;
	frontier.reserve(std::min(count, width * 2 + height * 2));
	constexpr std::array<std::pair<int, int>, 4> neighbours{
			{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	const auto is_passable = [&](std::size_t index) {
		return (grid[index / width][index % width] == LC_WATER) == through_water;
	};
	for (std::size_t index = 0; index < count; ++index) {
		if (!bit(seeds, index))
			continue;
		const int x = index % width, z = index / width;
		bool touches_passable = false;
		for (const auto &[dx, dz] : neighbours) {
			const int nx = x + dx, nz = z + dz;
			if (nx >= 0 && nz >= 0 && nx < int(width) && nz < int(height) &&
					is_passable(std::size_t(nz) * width + nx)) {
				touches_passable = true;
				break;
			}
		}
		if (touches_passable)
			frontier.push_back(static_cast<std::uint32_t>(index));
	}
	std::vector<std::uint32_t> next_frontier;
	for (int step = 0; step < std::max(0, distance) && !frontier.empty(); ++step) {
		next_frontier.clear();
		for (const auto index32 : frontier) {
			const auto index = static_cast<std::size_t>(index32);
			const int x = index % width, z = index / width;
			for (const auto &[dx, dz] : neighbours) {
				const int nx = x + dx, nz = z + dz;
				if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
					continue;
				const auto neighbor = std::size_t(nz) * width + nx;
				if (bit(seen, neighbor) || !is_passable(neighbor))
					continue;
				set_bit(seen, neighbor);
				set_bit(out, neighbor);
				next_frontier.push_back(static_cast<std::uint32_t>(neighbor));
			}
		}
		frontier.swap(next_frontier);
	}
	return out;
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
	std::vector<std::vector<std::vector<ProcessedNode>>> pools;
	bool any_area = false;
	bool any_land = false;
	for (const auto &element : elements) {
		if (element.is_relation()) {
			const auto &relation = element.as_relation();
			if (swimming_pools::is_swimming_pool(relation.tags)) {
				auto rings = relation_rings(relation, bbox);
				if (!rings.empty())
					pools.push_back(std::move(rings));
				continue;
			}
			std::vector<std::uint64_t> *target = nullptr;
			if (is_water_area_relation(relation.tags)) {
				any_area = true;
				target = &water_area;
			} else if (building(relation.tags)) {
				any_land = true;
				target = &land;
			}
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
				if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox)) {
					fill_ring(water_area, *ring, map);
					any_area = true;
				}
			continue;
		}
		const auto waterway = way.tags.get("waterway");
		if (way.tags.contains("waterway")) {
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
			any_land = true;
			continue;
		}
		if (building(way.tags) && closed(way.nodes)) {
			if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox)) {
				fill_ring(land, *ring, map);
				any_land = true;
			}
		} else if (swimming_pools::is_swimming_pool(way.tags) && closed(way.nodes)) {
			if (auto ring = clipping::clip_water_ring_to_bbox(way.nodes, bbox))
				pools.push_back({std::move(*ring)});
		}
	}
	if (!any_area && !any_land && pools.empty())
		return;
	std::vector<std::tuple<std::size_t, std::size_t, std::uint8_t>> changes;
	if (any_area || any_land) {
		for (std::size_t i = 0; i < land.size(); ++i)
			land[i] &= ~over_water[i];
		std::vector<std::uint64_t> land_seed((count + 63) / 64);
		for (std::size_t i = 0; i < count; ++i)
			if (data.grid[i / data.width][i % data.width] != LC_WATER)
				set_bit(land_seed, i);
		const double raw_band = 15.0 * data.cells_per_meter;
		const int band =
				!std::isfinite(data.cells_per_meter) || data.cells_per_meter <= 0.0 ? 1
				: raw_band >= 64.0													? 64
								   : std::max(1, static_cast<int>(std::lround(raw_band)));
		const auto rim =
				dilate(land_seed, data.grid, data.width, data.height, band, true);
		const auto past_outline = any_area
										  ? dilate(water_area, data.grid, data.width,
													data.height, band, true)
										  : std::vector<std::uint64_t>((count + 63) / 64);
		for (std::size_t i = 0; i < count; ++i) {
			if (!bit(rim, i) || bit(water_area, i) || bit(water_line, i) ||
					(!bit(land, i) && !bit(past_outline, i)))
				continue;
			const int x = i % data.width, z = i / data.width;
			if (const auto cls = nearest_land_class(
						data.grid, data.width, data.height, x, z, band + 2))
				changes.emplace_back(x, z, *cls);
		}
	}
	for (const auto &rings : pools)
		pool_changes(data, rings, map, water_area, water_line, changes);
	for (const auto &[x, z, cls] : changes)
		data.grid[z][x] = cls;
	if (!changes.empty()) {
		std::cerr << "OSM land override: " << changes.size()
				  << " shore cells reclassified from ESA water to land\n";
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.invalidate_water_blend_grid();
	}
}
}
