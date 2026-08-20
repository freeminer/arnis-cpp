#include "osm_land_override.h"

#include "../../arnis_adapter.h"
#include "../bresenham.h"
#include "../element_processing/bridges.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>

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

bool is_water_area(const tags_t &tags)
{
	const auto natural = tags.get("natural"), landuse = tags.get("landuse"),
			   waterway = tags.get("waterway");
	return natural == "water" || natural == "bay" || landuse == "reservoir" ||
		   waterway == "dock" || waterway == "riverbank" || enabled_tag(tags, "water");
}

bool building(const tags_t &tags)
{
	return enabled_tag(tags, "building") || tags.contains("building:part");
}

bool closed(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return false;
	const auto &a = nodes.front(), &b = nodes.back();
	return a.id == b.id || (std::abs(a.x - b.x) <= 1 && std::abs(a.z - b.z) <= 1);
}

struct GridMap
{
	int min_x, min_z;
	double sx, sz;
	std::size_t width, height;
	int x(int value) const { return int(std::lround((value - min_x) * sx)); }
	int z(int value) const { return int(std::lround((value - min_z) * sz)); }
};

void stamp_line(std::vector<std::uint64_t> &mask, const std::vector<ProcessedNode> &nodes,
		int half_width, const GridMap &map)
{
	const int rx = std::max(0, int(std::ceil(half_width * map.sx)));
	const int rz = std::max(0, int(std::ceil(half_width * map.sz)));
	for (std::size_t i = 1; i < nodes.size(); ++i)
		for (const auto [x, y, z] : bresenham::bresenham_line(map.x(nodes[i - 1].x), 0,
					 map.z(nodes[i - 1].z), map.x(nodes[i].x), 0, map.z(nodes[i].z))) {
			(void)y;
			for (int dz = -rz; dz <= rz; ++dz)
				for (int dx = -rx; dx <= rx; ++dx) {
					const int gx = x + dx, gz = z + dz;
					if (gx >= 0 && gz >= 0 && gx < int(map.width) && gz < int(map.height))
						set_bit(mask, std::size_t(gz) * map.width + gx);
				}
		}
}

void fill_ring(std::vector<std::uint64_t> &mask, const std::vector<ProcessedNode> &nodes,
		const GridMap &map)
{
	if (!closed(nodes))
		return;
	std::vector<std::pair<double, double>> ring;
	for (const auto &node : nodes)
		ring.emplace_back((node.x - map.min_x) * map.sx, (node.z - map.min_z) * map.sz);
	for (int z = 0; z < int(map.height); ++z) {
		std::vector<double> crossings;
		for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
			const auto [ax, az] = ring[j];
			const auto [bx, bz] = ring[i];
			if ((az > z) != (bz > z))
				crossings.push_back(ax + (z - az) * (bx - ax) / (bz - az));
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
		for (const auto [dx, dz] : neighbours) {
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

std::uint8_t nearest_land(const LandCoverData &data, int x, int z, int radius)
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
	return LC_GRASSLAND;
}
}

void apply_osm_land_override(LandCoverData &data, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2)
		return;
	const std::size_t count = data.width * data.height;
	GridMap map{bbox.min_x(), bbox.min_z(), double(data.width - 1) / (world_width - 1),
			double(data.height - 1) / (world_height - 1), data.width, data.height};
	std::vector<std::uint64_t> water_area((count + 63) / 64), land((count + 63) / 64),
			over_water((count + 63) / 64);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			continue;
		const bool structure =
				bridges::is_bridge_way(way) || way.tags.get("man_made") == "pier" ||
				way.tags.get("man_made") == "quay" || way.tags.get("floating") == "yes";
		if (structure) {
			stamp_line(over_water, way.nodes, 2, map);
			continue;
		}
		if (is_water_area(way.tags)) {
			fill_ring(water_area, way.nodes, map);
			continue;
		}
		if (building(way.tags)) {
			fill_ring(land, way.nodes, map);
			continue;
		}
		const auto highway = way.tags.get("highway"), railway = way.tags.get("railway");
		if (!highway.empty() && way.tags.get("tunnel") != "yes")
			stamp_line(land, way.nodes,
					arnis::highways::highway_block_range(highway, way.tags, scale), map);
		else if (railway == "rail" || railway == "light_rail" || railway == "tram")
			stamp_line(land, way.nodes, 1, map);
	}
	for (std::size_t i = 0; i < land.size(); ++i)
		land[i] &= ~over_water[i];
	std::vector<std::uint64_t> land_seed((count + 63) / 64);
	for (std::size_t i = 0; i < count; ++i)
		if (data.grid[i / data.width][i % data.width] != LC_WATER)
			set_bit(land_seed, i);
	const int band = std::clamp(int(std::lround(15.0 * data.cells_per_meter)), 1, 64);
	const auto rim = dilate(land_seed, data.grid, data.width, data.height, band, true);
	const auto past_outline =
			dilate(water_area, data.grid, data.width, data.height, band, true);
	std::size_t changed = 0;
	for (std::size_t i = 0; i < count; ++i) {
		if (!bit(rim, i) || bit(water_area, i) ||
				(!bit(land, i) && !bit(past_outline, i)))
			continue;
		const int x = i % data.width, z = i / data.width;
		data.grid[z][x] = nearest_land(data, x, z, band + 2);
		++changed;
	}
	if (changed) {
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.water_blend_grid.clear();
	}
}
}
