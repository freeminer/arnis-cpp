#include "urban_ground.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace arnis
{
namespace
{
using Cell = std::pair<int, int>;
using Points = std::vector<Cell>;

struct CellHash
{
	std::size_t operator()(const Cell &cell) const noexcept
	{
		const auto key = (std::uint64_t(std::uint32_t(cell.first)) << 32) |
						 std::uint32_t(cell.second);
		return std::hash<std::uint64_t>{}(key);
	}
};

using Grid = std::unordered_map<Cell, Points, CellHash>;
using Cells = std::unordered_set<Cell, CellHash>;

int rust_cell_div(int value, int divisor)
{
	// Match Rust's integer division semantics, including rejecting a zero cell
	// size rather than silently merging every building into cell (0, 0).
	if (divisor == 0)
		throw std::domain_error("urban ground cell_size must not be zero");
	return value / divisor;
}

int adaptive_expansion(
		const UrbanGroundConfig &config, const Cells &dense, const Grid &grid)
{
	if (dense.size() < 2)
		return config.cell_expansion;

	std::size_t total = 0;
	int min_x = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &cell : dense) {
		total += grid.at(cell).size();
		min_x = std::min(min_x, cell.first);
		max_x = std::max(max_x, cell.first);
		min_z = std::min(min_z, cell.second);
		max_z = std::max(max_z, cell.second);
	}

	const double average = static_cast<double>(total) / dense.size();
	const double span_x = static_cast<double>(max_x) - min_x + 1.0;
	const double span_z = static_cast<double>(max_z) - min_z + 1.0;
	const double occupancy = static_cast<double>(dense.size()) / (span_x * span_z);
	const double density_factor = average < 3.0 ? 1.5 : 1.0;
	const double occupancy_factor =
			occupancy < 0.4 ? 1.5 : (occupancy < 0.6 ? 1.25 : 1.0);
	const auto adaptive = static_cast<int>(
			std::ceil(config.cell_expansion * density_factor * occupancy_factor));
	return std::max(config.cell_expansion, std::min(4, adaptive));
}

Cells expanded(const Cells &cells, int amount)
{
	Cells result = cells;
	if (amount <= 0)
		return result;
	for (const auto &[x, z] : cells)
		for (int dz = -amount; dz <= amount; ++dz)
			for (int dx = -amount; dx <= amount; ++dx)
				result.emplace(x + dx, z + dz);
	return result;
}

std::vector<UrbanCluster> clusters(const UrbanGroundConfig &config, const Grid &grid)
{
	Cells dense;
	for (const auto &[cell, buildings] : grid)
		if (buildings.size() >= config.min_buildings_per_cell)
			dense.insert(cell);
	if (dense.empty())
		return {};

	const Cells area = expanded(dense, adaptive_expansion(config, dense, grid));
	Cells visited;
	std::vector<UrbanCluster> result;
	for (const auto &start : area) {
		if (!visited.insert(start).second)
			continue;
		std::deque<Cell> queue{start};
		UrbanCluster cluster;
		while (!queue.empty()) {
			const auto current = queue.front();
			queue.pop_front();
			cluster.cells.push_back(current);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					if (dx == 0 && dz == 0)
						continue;
					const Cell next{current.first + dx, current.second + dz};
					if (area.contains(next) && visited.insert(next).second)
						queue.push_back(next);
				}
		}
		for (const auto &cell : cluster.cells) {
			const auto it = grid.find(cell);
			if (it != grid.end())
				cluster.building_centroids.insert(cluster.building_centroids.end(),
						it->second.begin(), it->second.end());
		}
		cluster.building_count = cluster.building_centroids.size();
		if (cluster.building_count >= config.min_buildings_for_cluster)
			result.push_back(std::move(cluster));
	}
	return result;
}

Grid density_grid(const Points &points, const cartesian::XZBBox &bbox, int cell_size)
{
	Grid grid;
	for (const auto &[x, z] : points)
		grid[{rust_cell_div(x - bbox.min_x(), cell_size),
					 rust_cell_div(z - bbox.min_z(), cell_size)}]
				.push_back({x, z});
	return grid;
}

} // namespace

std::size_t UrbanGroundLookup::CellHash::operator()(
		const std::pair<int, int> &cell) const noexcept
{
	const auto key =
			(std::uint64_t(std::uint32_t(cell.first)) << 32) | std::uint32_t(cell.second);
	return std::hash<std::uint64_t>{}(key);
}

UrbanGroundLookup UrbanGroundLookup::empty()
{
	return {};
}

bool UrbanGroundLookup::is_urban(int x, int z) const
{
	if (urban_cells_.empty())
		return false;
	const int cx = rust_cell_div(x - bbox_min_x_, cell_size_);
	const int cz = rust_cell_div(z - bbox_min_z_, cell_size_);
	return urban_cells_.contains({cx, cz});
}

UrbanGroundComputer::UrbanGroundComputer(cartesian::XZBBox bbox,
		UrbanGroundConfig config) : config_(std::move(config)), bbox_(std::move(bbox))
{
}

UrbanGroundComputer UrbanGroundComputer::with_defaults(cartesian::XZBBox bbox)
{
	return UrbanGroundComputer(std::move(bbox));
}

void UrbanGroundComputer::add_building_centroid(int x, int z)
{
	if (x >= bbox_.min_x() && x <= bbox_.max_x() && z >= bbox_.min_z() &&
			z <= bbox_.max_z())
		building_centroids_.emplace_back(x, z);
}

void UrbanGroundComputer::add_building_centroids(const Points &points)
{
	for (const auto &[x, z] : points)
		add_building_centroid(x, z);
}

UrbanGroundLookup UrbanGroundComputer::compute_lookup() const
{
	if (building_centroids_.size() < config_.min_buildings_for_cluster)
		return UrbanGroundLookup::empty();

	const auto grid = density_grid(building_centroids_, bbox_, config_.cell_size);
	const auto found = clusters(config_, grid);
	if (found.empty())
		return UrbanGroundLookup::empty();

	UrbanGroundLookup result;
	result.set_bounds(bbox_.min_x(), bbox_.min_z(), config_.cell_size);
	for (const auto &cluster : found)
		for (const auto &[cx, cz] : cluster.cells)
			result.add_cell(cx, cz);
	return result;
}

Points UrbanGroundComputer::compute(
		const std::optional<std::chrono::duration<double>> &timeout) const
{
	// Rust accepts this timeout for API compatibility, but deliberately does
	// not apply it to the direct cell-fill implementation.
	(void)timeout;
	Points result;
	if (building_centroids_.size() < config_.min_buildings_for_cluster)
		return result;
	const auto grid = density_grid(building_centroids_, bbox_, config_.cell_size);
	for (const auto &cluster : clusters(config_, grid))
		for (const auto &[cx, cz] : cluster.cells) {
			const int x0 =
					std::max(bbox_.min_x(), bbox_.min_x() + cx * config_.cell_size);
			const int x1 = std::min(
					bbox_.max_x(), bbox_.min_x() + (cx + 1) * config_.cell_size - 1);
			const int z0 =
					std::max(bbox_.min_z(), bbox_.min_z() + cz * config_.cell_size);
			const int z1 = std::min(
					bbox_.max_z(), bbox_.min_z() + (cz + 1) * config_.cell_size - 1);
			if (x0 > x1 || z0 > z1)
				continue;
			for (int x = x0; x <= x1; ++x)
				for (int z = z0; z <= z1; ++z)
					result.emplace_back(x, z);
		}
	return result;
}

std::optional<Cell> compute_centroid(const Points &points)
{
	if (points.empty())
		return std::nullopt;
	std::int64_t x = 0;
	std::int64_t z = 0;
	for (const auto &[px, pz] : points) {
		x += px;
		z += pz;
	}
	const auto count = static_cast<std::int64_t>(points.size());
	return Cell{static_cast<int>(x / count), static_cast<int>(z / count)};
}

Points compute_urban_ground(const Points &points, const cartesian::XZBBox &bbox,
		const std::optional<std::chrono::duration<double>> &timeout)
{
	auto computer = UrbanGroundComputer::with_defaults(bbox);
	computer.add_building_centroids(points);
	return computer.compute(timeout);
}

UrbanGroundLookup compute_urban_ground_lookup(const Points &points,
		const cartesian::XZBBox &bbox, const UrbanGroundConfig &config)
{
	UrbanGroundComputer computer(bbox, config);
	computer.add_building_centroids(points);
	return computer.compute_lookup();
}

UrbanGroundLookup compute_urban_lookup(
		const Points &points, int min_x, int min_z, const UrbanGroundConfig &config)
{
	if (points.empty()) {
		UrbanGroundLookup result;
		result.set_bounds(min_x, min_z, config.cell_size);
		return result;
	}
	int max_x = min_x;
	int max_z = min_z;
	for (const auto &[x, z] : points) {
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	return compute_urban_ground_lookup(points,
			cartesian::XZBBox::rect_from_min_max(min_x, min_z, max_x, max_z), config);
}

} // namespace arnis
