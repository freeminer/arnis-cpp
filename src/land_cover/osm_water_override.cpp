#include "osm_water_override.h"

#include "../element_processing/waterways.h"
#include "../element_processing/way_segments.h"
#include "../bresenham.h"
#include "../clipping.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <limits>
#include <numeric>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace arnis::land_cover
{
namespace
{
// Split the independent row scans just as the Rust Rayon pass does. Each worker
// owns one row vector; run ids are assigned afterward in row order so component
// merging and the resulting protected mask remain deterministic.
template <typename Function>
void parallel_rows(std::size_t row_count, Function &&function)
{
	if (row_count < 32) {
		for (std::size_t row = 0; row < row_count; ++row)
			function(row);
		return;
	}
	std::size_t worker_limit = 0;
	if (const char *configured = std::getenv("RAYON_NUM_THREADS")) {
		const std::string_view text(configured);
		const auto [ptr, error] =
				std::from_chars(text.data(), text.data() + text.size(), worker_limit);
		if (error != std::errc{} || ptr != text.data() + text.size() || worker_limit == 0)
			worker_limit = 0;
	}
	if (worker_limit == 0) {
		const auto hardware = std::max(1u, std::thread::hardware_concurrency());
		worker_limit = std::max<std::size_t>(1, hardware * 9 / 10);
	}
	const auto workers = std::min(row_count, worker_limit);
	if (workers == 1) {
		for (std::size_t row = 0; row < row_count; ++row)
			function(row);
		return;
	}
	std::atomic_size_t next{0};
	auto worker = [&] {
		for (;;) {
			const auto row = next.fetch_add(1, std::memory_order_relaxed);
			if (row >= row_count)
				return;
			function(row);
		}
	};
	std::vector<std::thread> threads;
	threads.reserve(workers - 1);
	for (std::size_t i = 1; i < workers; ++i)
		try {
			threads.emplace_back(worker);
		} catch (...) {
			break;
		}
	worker();
	for (auto &thread : threads)
		thread.join();
}
} // namespace

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
	if (height > std::numeric_limits<std::size_t>::max() / width)
		return out;
	const auto cells = width * height;
	out.width = width;
	out.height = height;
	out.protected_mask.assign(cells / 64 + (cells % 64 != 0), 0);
	const std::size_t threshold = std::max<std::size_t>(
			100, std::size_t(std::llround(4000.0 * std::max(0.0, cells_per_m2))));
	struct Run
	{
		std::size_t begin, end, id = 0;
	};
	std::vector<std::vector<Run>> rows(height);
	parallel_rows(height, [&](std::size_t z) {
		for (std::size_t x = 0; x < width;) {
			if (grid[z][x] == LC_WATER) {
				++x;
				continue;
			}
			const auto begin = x;
			while (x < width && grid[z][x] != LC_WATER)
				++x;
			rows[z].push_back({begin, x});
		}
	});
	std::vector<std::size_t> first_run(height + 1, 0);
	for (std::size_t z = 0; z < height; ++z)
		first_run[z + 1] = first_run[z] + rows[z].size();
	std::vector<std::size_t> parent(first_run.back());
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t i = 0; i < rows[z].size(); ++i)
			rows[z][i].id = first_run[z] + i;
	std::iota(parent.begin(), parent.end(), 0);
	auto root = [&](std::size_t id) {
		while (parent[id] != id) {
			parent[id] = parent[parent[id]];
			id = parent[id];
		}
		return id;
	};
	for (std::size_t z = 1; z < height; ++z) {
		const auto &above = rows[z - 1];
		const auto &here = rows[z];
		std::size_t a = 0, b = 0;
		while (a < above.size() && b < here.size()) {
			const auto &ra = above[a];
			const auto &rb = here[b];
			if (ra.begin < rb.end && rb.begin < ra.end) {
				const auto root_a = root(ra.id);
				const auto root_b = root(rb.id);
				if (root_a != root_b)
					parent[std::max(root_a, root_b)] = std::min(root_a, root_b);
			}
			if (ra.end < rb.end)
				++a;
			else
				++b;
		}
	}
	struct Bounds
	{
		std::size_t min_x = std::numeric_limits<std::size_t>::max(),
					min_z = std::numeric_limits<std::size_t>::max(), max_x = 0, max_z = 0,
					count = 0;
	};
	std::vector<Bounds> components(parent.size());
	for (std::size_t z = 0; z < height; ++z)
		for (const auto &run : rows[z]) {
			auto &component = components[root(run.id)];
			component.count += run.end - run.begin;
			component.min_x = std::min(component.min_x, run.begin);
			component.max_x = std::max(component.max_x, run.end - 1);
			component.min_z = std::min(component.min_z, z);
			component.max_z = std::max(component.max_z, z);
		}
	for (std::size_t z = 0; z < height; ++z)
		for (const auto &run : rows[z]) {
			const auto &component = components[root(run.id)];
			const auto box_width = component.max_x - component.min_x + 1;
			const auto box_height = component.max_z - component.min_z + 1;
			const auto long_axis = std::max(box_width, box_height);
			const auto short_axis =
					std::max<std::size_t>(1, std::min(box_width, box_height));
			if (component.count < threshold || double(long_axis) / short_axis > 3.0)
				continue;
			for (auto x = run.begin; x < run.end; ++x) {
				const auto index = z * width + x;
				out.protected_mask[index >> 6] |= std::uint64_t(1) << (index & 63);
			}
		}
	out.nearest_water_y.assign(cells, std::numeric_limits<float>::quiet_NaN());
	std::deque<std::size_t> q;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x)
			if (grid[z][x] == LC_WATER && std::isfinite(heights[z][x])) {
				const auto i = z * width + x;
				out.nearest_water_y[i] = heights[z][x];
				q.push_back(i);
			}
	while (!q.empty()) {
		const auto i = q.front();
		q.pop_front();
		const auto x = i % width;
		const auto water_y = out.nearest_water_y[i];
		const std::array<std::size_t, 4> neighbours = {x > 0 ? i - 1 : cells,
				x + 1 < width ? i + 1 : cells, i >= width ? i - width : cells,
				i < cells - width ? i + width : cells};
		for (const auto next : neighbours) {
			if (next == cells || std::isfinite(out.nearest_water_y[next]))
				continue;
			out.nearest_water_y[next] = water_y;
			q.push_back(next);
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
	const auto i = z * width + x;
	if (!has_elevation || i >= nearest_water_y.size())
		return true;
	const float h = heights[z][x], water = nearest_water_y[i];
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
					const auto index = z * guard.width + x;
					if (data.grid[z][x] != LC_WATER || !std::isfinite(heights[z][x]) ||
							index >= guard.nearest_water_y.size())
						continue;
					const float water_y = guard.nearest_water_y[index];
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
