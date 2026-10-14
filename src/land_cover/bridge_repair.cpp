#include "bridge_repair.h"

#include "../bresenham.h"
#include "../element_processing/bridges.h"
#include "../osm_parser.h"
#include "../floodfill_cache.h"
#include "../../arnis_adapter.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace arnis::highways
{
int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale);
}

namespace arnis::land_cover
{
namespace
{
constexpr int BRIDGE_STAMP_MARGIN_CELLS = 8;
constexpr float WATER_HEIGHT_TOLERANCE_BLOCKS = 6.0f;

int scaled_block_range(int raw, double scale)
{
	return scale < 1.0 ? std::max(1, static_cast<int>(std::floor(raw * scale))) : raw;
}

int bridge_block_range_world(const ProcessedWay &way, double scale)
{
	const auto highway = way.tags.get("highway");
	if (!highway.empty())
		return highways::highway_block_range(highway, way.tags, scale);
	if (way.tags.find("railway") != way.tags.end()) {
		int tracks = 1;
		const auto tracks_tag = way.tags.get("tracks");
		if (!tracks_tag.empty()) {
			int parsed_tracks = 0;
			const auto parsed = std::from_chars(tracks_tag.data(),
					tracks_tag.data() + tracks_tag.size(), parsed_tracks);
			if (parsed.ec == std::errc{} &&
					parsed.ptr == tracks_tag.data() + tracks_tag.size())
				tracks = std::max(1, parsed_tracks);
		}
		return scaled_block_range(tracks, scale);
	}
	return scaled_block_range(1, scale);
}

int world_to_grid(int relative, double scale)
{
	return static_cast<int>(std::lround(std::max(relative, 0) * scale));
}

} // namespace

void apply_bridge_land_cover_repair(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2 ||
			heights.size() < data.height ||
			std::any_of(heights.begin(), heights.begin() + data.height,
					[&](const auto &row) { return row.size() < data.width; }))
		return;
	if (std::none_of(elements.begin(), elements.end(), [](const auto &element) {
			return element.is_way() && bridges::is_bridge_way(element.as_way());
		}))
		return;

	const double scale_x = double(data.width - 1) / double(world_width - 1);
	const double scale_z = double(data.height - 1) / double(world_height - 1);
	std::vector<std::uint64_t> bridge_mask((data.width * data.height + 63) / 64);
	std::vector<std::uint32_t> bridge_cells;
	for (const auto &element : elements) {
		if (!element.is_way() || !bridges::is_bridge_way(element.as_way()) ||
				element.as_way().nodes.size() < 2)
			continue;
		const auto &way = element.as_way();
		const int block_range =
				bridge_block_range_world(way, scale) + BRIDGE_STAMP_MARGIN_CELLS;
		const int range_x =
				std::max(1, static_cast<int>(std::ceil(block_range * scale_x)));
		const int range_z =
				std::max(1, static_cast<int>(std::ceil(block_range * scale_z)));
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const int x0 = world_to_grid(way.nodes[i - 1].x - bbox.min_x(), scale_x);
			const int z0 = world_to_grid(way.nodes[i - 1].z - bbox.min_z(), scale_z);
			const int x1 = world_to_grid(way.nodes[i].x - bbox.min_x(), scale_x);
			const int z1 = world_to_grid(way.nodes[i].z - bbox.min_z(), scale_z);
			for (const auto &[x, y, z] :
					bresenham::bresenham_line(x0, 0, z0, x1, 0, z1)) {
				(void)y;
				for (int dz = -range_z; dz <= range_z; ++dz) {
					for (int dx = -range_x; dx <= range_x; ++dx) {
						const int gx = x + dx;
						const int gz = z + dz;
						if (gx < 0 || gz < 0 || gx >= static_cast<int>(data.width) ||
								gz >= static_cast<int>(data.height) ||
								data.grid[gz][gx] != LC_BUILT_UP)
							continue;
						const auto index = static_cast<std::size_t>(gz) * data.width + gx;
						const auto bit = std::uint64_t{1} << (index & 63);
						if (!(bridge_mask[index >> 6] & bit)) {
							bridge_mask[index >> 6] |= bit;
							bridge_cells.push_back(static_cast<std::uint32_t>(index));
						}
					}
				}
			}
		}
	}
	if (bridge_cells.empty())
		return;

	const auto is_bridge_cell = [&](std::size_t index) {
		return ((bridge_mask[index >> 6] >> (index & 63)) & 1U) != 0;
	};
	constexpr std::array<std::pair<int, int>, 4> neighbours{
			{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	struct Assignment
	{
		std::uint32_t cell;
		std::uint8_t land_class;
		float water_level;
	};
	std::unordered_map<std::uint32_t, std::uint8_t> assigned;
	std::unordered_map<std::uint32_t, float> water_levels;
	std::deque<Assignment> current, next;
	const auto water_reachable = [&](std::size_t index, float level) {
		const float height = heights[index / data.width][index % data.width];
		return !std::isfinite(height) || !std::isfinite(level) ||
			   height <= level + WATER_HEIGHT_TOLERANCE_BLOCKS;
	};

	// Seed water-adjacent bridge cells first so water wins when it can reach the
	// deck without climbing more than the Rust-side tolerance.
	for (const auto cell : bridge_cells) {
		const int x = cell % data.width;
		const int z = cell / data.width;
		for (const auto &[dx, dz] : neighbours) {
			const int nx = x + dx;
			const int nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= static_cast<int>(data.width) ||
					nz >= static_cast<int>(data.height))
				continue;
			const auto neighbour = static_cast<std::size_t>(nz) * data.width + nx;
			if (is_bridge_cell(neighbour) || data.grid[nz][nx] != LC_WATER)
				continue;
			const float level = heights[nz][nx];
			if (!water_reachable(cell, level))
				continue;
			assigned.emplace(cell, LC_WATER);
			water_levels.emplace(cell, level);
			current.push_back({cell, LC_WATER, level});
			break;
		}
	}
	// Then seed from the first adjacent non-water land-cover class.
	for (const auto cell : bridge_cells) {
		if (assigned.contains(cell))
			continue;
		const int x = cell % data.width;
		const int z = cell / data.width;
		for (const auto &[dx, dz] : neighbours) {
			const int nx = x + dx;
			const int nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= static_cast<int>(data.width) ||
					nz >= static_cast<int>(data.height))
				continue;
			const auto neighbour = static_cast<std::size_t>(nz) * data.width + nx;
			const auto land_class = data.grid[nz][nx];
			if (is_bridge_cell(neighbour) || land_class == 0 || land_class == LC_WATER)
				continue;
			assigned.emplace(cell, land_class);
			current.push_back(
					{cell, land_class, std::numeric_limits<float>::quiet_NaN()});
			break;
		}
	}

	for (std::size_t depth = 1; !current.empty() && depth < 64; ++depth) {
		while (!current.empty()) {
			const auto [cell, land_class, level] = current.front();
			current.pop_front();
			const int x = cell % data.width;
			const int z = cell / data.width;
			for (const auto &[dx, dz] : neighbours) {
				const int nx = x + dx;
				const int nz = z + dz;
				if (nx < 0 || nz < 0 || nx >= static_cast<int>(data.width) ||
						nz >= static_cast<int>(data.height))
					continue;
				const auto neighbour = static_cast<std::size_t>(nz) * data.width + nx;
				if (!is_bridge_cell(neighbour) || assigned.contains(neighbour) ||
						(land_class == LC_WATER && !water_reachable(neighbour, level)))
					continue;
				const auto index = static_cast<std::uint32_t>(neighbour);
				assigned.emplace(index, land_class);
				if (land_class == LC_WATER)
					water_levels.emplace(index, level);
				next.push_back({index, land_class, level});
			}
		}
		current.swap(next);
	}

	bool water_changed = false;
	std::size_t mutated = 0;
	std::size_t reclassified_to_water = 0;
	for (const auto cell : bridge_cells) {
		const auto assignment = assigned.find(cell);
		if (assignment == assigned.end())
			continue;
		const auto x = static_cast<std::size_t>(cell % data.width);
		const auto z = static_cast<std::size_t>(cell / data.width);
		const auto old_class = data.grid[z][x];
		const auto new_class = assignment->second;
		if (old_class == new_class)
			continue;
		data.grid[z][x] = new_class;
		++mutated;
		if (new_class == LC_WATER) {
			++reclassified_to_water;
			const auto level = water_levels.find(cell);
			if (level != water_levels.end() && std::isfinite(level->second) &&
					std::isfinite(heights[z][x]) && heights[z][x] > level->second)
				heights[z][x] = level->second;
		}
		water_changed |= (old_class == LC_WATER) != (new_class == LC_WATER);
	}
	std::clog << "Bridge LC repair: " << mutated << " cells reclassified ("
			  << reclassified_to_water << " to water)\n";
	if (water_changed) {
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.invalidate_water_blend_grid();
	}
}

} // namespace arnis::land_cover
