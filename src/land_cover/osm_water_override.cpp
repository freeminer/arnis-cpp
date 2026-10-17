#include "osm_water_override.h"

#include "../element_processing/waterways.h"
#include "../element_processing/way_segments.h"
#include "../bresenham.h"
#include "../clipping.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

namespace arnis::land_cover
{
WaterOverrideGuard build_water_override_guard(
		const std::vector<std::vector<uint8_t>> &grid,
		const std::vector<std::vector<float>> &heights, double cells_per_m2)
{
	WaterOverrideGuard out;
	const std::size_t height = grid.size(), width = height ? grid.front().size() : 0;
	if (width < 2 || height < 2)
		return out;
	// Rust's build_water_context rejects malformed grid/elevation dimensions before
	// constructing either guard. Keep the public C++ helper's behavior consistent,
	// even when it is called outside apply_osm_water_override's validation path.
	if (std::any_of(grid.begin(), grid.end(),
				[&](const auto &row) { return row.size() < width; }) ||
			heights.size() < height ||
			std::any_of(heights.begin(),
					heights.begin() + std::min(height, heights.size()),
					[&](const auto &row) { return row.size() < width; }))
		return out;
	// Rust returns no WaterContext when the source grid has no ESA-water seed.
	// In that case OSM water must not be constrained by a land-component mask:
	// with no reference water, every component would otherwise be classified as
	// protected land and polygon/river overrides would silently do nothing.
	const bool has_water_seed =
			std::any_of(grid.begin(), grid.end(), [&](const auto &row) {
				return row.size() >= width && std::find(row.begin(), row.begin() + width,
													  LC_WATER) != row.begin() + width;
			});
	if (!has_water_seed)
		return out;
	out.width = width;
	out.protected_mask.assign((width * height + 63) / 64, 0);
	auto bit = [&](std::size_t i) {
		return (out.protected_mask[i >> 6] >> (i & 63)) & 1U;
	};
	auto set = [&](std::size_t i) {
		out.protected_mask[i >> 6] |= std::uint64_t(1) << (i & 63);
	};
	const std::size_t threshold = std::max<std::size_t>(
			100, std::size_t(std::llround(4000.0 * std::max(0.0, cells_per_m2))));
	std::vector<std::uint32_t> seeds, stack;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const auto start = z * width + x;
			if (grid[z][x] == LC_WATER || bit(start))
				continue;
			stack = {std::uint32_t(start)};
			set(start);
			std::size_t count = 0, minx = x, maxx = x, minz = z, maxz = z;
			while (!stack.empty()) {
				auto cur = stack.back();
				stack.pop_back();
				auto cx = cur % width, cz = cur / width;
				++count;
				minx = std::min(minx, std::size_t(cx));
				maxx = std::max(maxx, std::size_t(cx));
				minz = std::min(minz, std::size_t(cz));
				maxz = std::max(maxz, std::size_t(cz));
				for (auto [dx, dz] : std::array<std::pair<int, int>, 4>{
							 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
					int nx = int(cx) + dx, nz = int(cz) + dz;
					if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
						continue;
					auto ni = std::size_t(nz) * width + nx;
					if (grid[nz][nx] != LC_WATER && !bit(ni)) {
						set(ni);
						stack.push_back(ni);
					}
				}
			}
			const auto long_axis = std::max(maxx - minx + 1, maxz - minz + 1),
					   short_axis = std::max<std::size_t>(
							   1, std::min(maxx - minx + 1, maxz - minz + 1));
			if (count >= threshold && double(long_axis) / short_axis <= 3.0)
				seeds.push_back(start);
		}
	std::fill(out.protected_mask.begin(), out.protected_mask.end(), 0);
	for (auto seed : seeds) {
		if (bit(seed))
			continue;
		stack = {seed};
		set(seed);
		while (!stack.empty()) {
			auto cur = stack.back();
			stack.pop_back();
			auto cx = cur % width, cz = cur / width;
			for (auto [dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				int nx = int(cx) + dx, nz = int(cz) + dz;
				if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height))
					continue;
				auto ni = std::size_t(nz) * width + nx;
				if (grid[nz][nx] != LC_WATER && !bit(ni)) {
					set(ni);
					stack.push_back(ni);
				}
			}
		}
	}
	if (width * height > 100U * 1024U * 1024U)
		return out;
	out.nearest_water_y.assign(
			height, std::vector<float>(width, std::numeric_limits<float>::quiet_NaN()));
	std::deque<std::pair<int, int>> q;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x)
			if (grid[z][x] == LC_WATER && std::isfinite(heights[z][x])) {
				out.nearest_water_y[z][x] = heights[z][x];
				q.push_back({int(x), int(z)});
			}
	while (!q.empty()) {
		auto [x, z] = q.front();
		q.pop_front();
		for (auto [dx, dz] :
				std::array<std::pair<int, int>, 4>{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
			int nx = x + dx, nz = z + dz;
			if (nx < 0 || nz < 0 || nx >= int(width) || nz >= int(height) ||
					std::isfinite(out.nearest_water_y[nz][nx]))
				continue;
			out.nearest_water_y[nz][nx] = out.nearest_water_y[z][x];
			q.push_back({nx, nz});
		}
	}
	out.has_elevation = true;
	return out;
}

bool WaterOverrideGuard::allows(const std::vector<std::vector<float>> &heights,
		std::size_t x, std::size_t z, float tolerance) const
{
	if (!width || x >= width || z >= heights.size() || x >= heights[z].size())
		return true;
	if ((protected_mask[(z * width + x) >> 6] >> ((z * width + x) & 63)) & 1U)
		return false;
	if (!has_elevation || z >= nearest_water_y.size() || x >= nearest_water_y[z].size())
		return true;
	const float h = heights[z][x], water = nearest_water_y[z][x];
	return !std::isfinite(h) || !std::isfinite(water) || h <= water + tolerance;
}

void apply_osm_water_override(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox)
{
	if (data.width < 2 || data.height < 2 || world_width < 2 || world_height < 2 ||
			heights.size() < data.height ||
			std::any_of(heights.begin(), heights.begin() + data.height,
					[&](const auto &row) { return row.size() < data.width; }))
		return;
	const double sx = double(data.width - 1) / double(world_width - 1);
	const double sz = double(data.height - 1) / double(world_height - 1);
	const auto guard = build_water_override_guard(data.grid, heights, sx * sz);
	const auto tag = [](const tags_t &tags, const char *name) -> const std::string * {
		auto it = tags.find(name);
		return it == tags.end() ? nullptr : &it->second;
	};
	const auto has_explicit_water = [&](const tags_t &tags) {
		const auto *water = tag(tags, "water");
		return water && *water != "no" && *water != "0" && *water != "false";
	};
	const auto is_water_polygon_way = [&](const ProcessedWay &w) {
		if (w.nodes.size() < 3)
			return false;
		const auto *natural = tag(w.tags, "natural");
		const auto *landuse = tag(w.tags, "landuse");
		const auto *waterway = tag(w.tags, "waterway");
		return (natural && *natural == "water") || has_explicit_water(w.tags) ||
			   (landuse && *landuse == "reservoir") ||
			   (waterway && (*waterway == "dock" || *waterway == "riverbank"));
	};
	const auto is_water_relation = [&](const tags_t &tags) {
		const auto *natural = tag(tags, "natural");
		return (natural && (*natural == "water" || *natural == "bay")) ||
			   has_explicit_water(tags);
	};
	const auto closed_ring = [](const std::vector<ProcessedNode> &nodes) {
		if (nodes.size() < 3)
			return false;
		const auto &first = nodes.front();
		const auto &last = nodes.back();
		return first.id == last.id ||
			   (std::abs(first.x - last.x) <= 1 && std::abs(first.z - last.z) <= 1);
	};
	const auto to_world_ring = [](const std::vector<ProcessedNode> &nodes) {
		std::vector<std::pair<int, int>> ring;
		ring.reserve(nodes.size());
		for (const auto &node : nodes)
			ring.emplace_back(node.x, node.z);
		return ring;
	};
	std::size_t changed = 0;
	const auto write_water = [&](std::size_t x, std::size_t z) {
		// Match Rust's passes_water_guard for both polygon fills and line rasterization:
		// OSM water must not overwrite large compact land components or climb above
		// the nearest ESA-water elevation by more than the configured tolerance.
		if (data.grid[z][x] == LC_WATER || !guard.allows(heights, x, z, 1.5f))
			return std::size_t{0};
		data.grid[z][x] = LC_WATER;
		return std::size_t{1};
	};
	const auto edge_crossings = [](const std::vector<std::pair<int, int>> &ring, double z,
										std::vector<double> &out) {
		if (ring.size() < 3)
			return;
		std::size_t j = ring.size() - 1;
		for (std::size_t i = 0; i < ring.size(); ++i) {
			const double zi = ring[i].second, zj = ring[j].second;
			if ((zi > z) != (zj > z)) {
				const double t = (z - zi) / (zj - zi);
				out.push_back(ring[i].first + (ring[j].first - ring[i].first) * t);
			}
			j = i;
		}
	};
	const auto inside_ranges = [](double x, const std::vector<double> &ranges) {
		for (std::size_t i = 0; i + 1 < ranges.size(); i += 2)
			if (x > ranges[i] && x < ranges[i + 1])
				return true;
		return false;
	};
	const auto fill_rings =
			[&](const std::vector<std::vector<std::pair<int, int>>> &outers,
					const std::vector<std::vector<std::pair<int, int>>> &inners) {
				if (outers.empty())
					return std::size_t{0};
				int p_min_z = INT32_MAX, p_max_z = INT32_MIN;
				for (const auto &ring : outers)
					for (const auto &[x, z] : ring) {
						(void)x;
						p_min_z = std::min(p_min_z, z);
						p_max_z = std::max(p_max_z, z);
					}
				if (p_min_z == INT32_MAX || p_max_z < bbox.min_z() ||
						p_min_z > bbox.max_z())
					return std::size_t{0};
				const double inv_x = 1.0 / sx, inv_z = 1.0 / sz;
				const int gz_start = std::max(
						0, int(std::floor(std::max(0, p_min_z - bbox.min_z()) * sz)));
				const int gz_end = std::clamp(
						int(std::ceil(std::max(0, p_max_z - bbox.min_z()) * sz)), 0,
						int(data.height) - 1);
				std::vector<double> outer_x, inner_x;
				std::size_t count = 0;
				for (int gz = gz_start; gz <= gz_end; ++gz) {
					const double world_z = gz * inv_z + bbox.min_z();
					outer_x.clear();
					for (const auto &ring : outers)
						edge_crossings(ring, world_z, outer_x);
					if (outer_x.size() < 2)
						continue;
					std::sort(outer_x.begin(), outer_x.end());
					inner_x.clear();
					for (const auto &ring : inners)
						edge_crossings(ring, world_z, inner_x);
					std::sort(inner_x.begin(), inner_x.end());
					for (std::size_t i = 0; i + 1 < outer_x.size(); i += 2) {
						const double left = std::max(outer_x[i], double(bbox.min_x()));
						const double right =
								std::min(outer_x[i + 1], double(bbox.max_x()));
						if (left > right)
							continue;
						const int gx0 =
								std::max(0, int(std::ceil((left - bbox.min_x()) * sx)));
						const int gx1 =
								std::clamp(int(std::floor((right - bbox.min_x()) * sx)),
										0, int(data.width) - 1);
						for (int gx = gx0; gx <= gx1; ++gx) {
							const double world_x = gx * inv_x + bbox.min_x();
							if (!inside_ranges(world_x, inner_x))
								count += write_water(gx, gz);
						}
					}
				}
				return count;
			};
	const auto rasterize_line = [&](const std::vector<ProcessedNode> &nodes,
										int half_width) {
		if (nodes.size() < 2 || half_width < 0)
			return std::size_t{0};
		std::size_t count = 0;
		for (std::size_t i = 1; i < nodes.size(); ++i) {
			const auto &a = nodes[i - 1];
			const auto &b = nodes[i];
			const int margin = std::max(0, half_width);
			if (std::max(a.x, b.x) + margin < bbox.min_x() ||
					std::min(a.x, b.x) - margin > bbox.max_x() ||
					std::max(a.z, b.z) + margin < bbox.min_z() ||
					std::min(a.z, b.z) - margin > bbox.max_z())
				continue;
			for (const auto &[bx, y, bz] :
					bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
				(void)y;
				for (int dx = -half_width; dx <= half_width; ++dx)
					for (int dz = -half_width; dz <= half_width; ++dz) {
						const int wx = bx + dx, wz = bz + dz;
						// Match Rust's rasterize_line: reject coordinates below the
						// bbox origin, map the remaining world coordinate to the
						// land-cover grid, then clip by grid dimensions. A point just
						// beyond the positive edge can round onto the last raster cell.
						if (wx < bbox.min_x() || wz < bbox.min_z())
							continue;
						const auto gx = static_cast<std::size_t>(
								std::llround((wx - bbox.min_x()) * sx));
						const auto gz = static_cast<std::size_t>(
								std::llround((wz - bbox.min_z()) * sz));
						if (gx < data.width && gz < data.height)
							count += write_water(gx, gz);
					}
			}
		}
		return count;
	};
	for (const auto &e : elements) {
		if (e.is_relation()) {
			const auto &rel = e.as_relation();
			if (!is_water_relation(rel.tags))
				continue;
			std::vector<std::vector<ProcessedNode>> outer_nodes, inner_nodes;
			for (const auto &m : rel.members) {
				if (m.way->nodes.size() < 2)
					continue;
				if (m.role == ProcessedMemberRole::Outer)
					outer_nodes.push_back(m.way->nodes);
				else if (m.role == ProcessedMemberRole::Inner)
					inner_nodes.push_back(m.way->nodes);
			}
			merge_way_segments(outer_nodes);
			merge_way_segments(inner_nodes);
			std::vector<std::vector<std::pair<int, int>>> outer, inner;
			for (const auto &ring : outer_nodes)
				if (closed_ring(ring))
					if (auto clipped = clipping::clip_water_ring_to_bbox(ring, bbox))
						outer.push_back(to_world_ring(*clipped));
			for (const auto &ring : inner_nodes)
				if (closed_ring(ring))
					if (auto clipped = clipping::clip_water_ring_to_bbox(ring, bbox))
						inner.push_back(to_world_ring(*clipped));
			changed += fill_rings(outer, inner);
			continue;
		}
		if (!e.is_way() || e.as_way().nodes.size() < 2)
			continue;
		const auto &w = e.as_way();
		const auto *waterway = tag(w.tags, "waterway");
		if (is_water_polygon_way(w)) {
			if (!closed_ring(w.nodes))
				continue;
			auto clipped = clipping::clip_water_ring_to_bbox(w.nodes, bbox);
			if (!clipped)
				continue;
			changed += fill_rings({to_world_ring(*clipped)}, {});
		} else if (waterway && waterways::is_channel_waterway(*waterway) &&
				   !waterways::is_underground_waterway(w.tags)) {
			const int half_width = waterways::waterway_width(*waterway, w.tags) / 2;
			changed += rasterize_line(w.nodes, half_width);
		}
	}
	if (changed) {
		std::clog << "OSM water override: reclassified " << changed
				  << " cells to LC_WATER inside OSM water polygons/lines\n";
		data.water_distance = compute_water_distance(data.grid, data.width, data.height);
		data.invalidate_water_blend_grid();
		// OSM water may extend ESA water slightly, but should not float on a raised
		// bank. Sink only newly reachable cells that are within Rust's 1.5-block
		// elevation guard; the ESA water seeds remain at their sampled heights.
		if (guard.has_elevation) {
			std::size_t sunk = 0;
			for (std::size_t z = 0; z < data.height; ++z)
				for (std::size_t x = 0; x < data.width; ++x) {
					if (data.grid[z][x] != LC_WATER || !std::isfinite(heights[z][x]) ||
							z >= guard.nearest_water_y.size() ||
							x >= guard.nearest_water_y[z].size())
						continue;
					const float water_y = guard.nearest_water_y[z][x];
					if (std::isfinite(water_y) && heights[z][x] > water_y &&
							heights[z][x] <= water_y + 1.5f) {
						heights[z][x] = water_y;
						++sunk;
					}
				}
			if (sunk)
				std::clog << "OSM water override: sank " << sunk
						  << " overridden cells onto the adjacent water surface\n";
		}
	}
}
} // namespace arnis::land_cover
