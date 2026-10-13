#pragma once

#include "coordinate_system/cartesian/xzbbox/xzbbox_enum.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arnis
{

struct UrbanGroundConfig
{
	int cell_size = 64;
	std::size_t min_buildings_per_cell = 1;
	std::size_t min_buildings_for_cluster = 5;
	double concavity = 2.0;
	bool expand_hull = true;
	int cell_expansion = 2;
};

class UrbanGroundLookup
{
	struct CellHash
	{
		std::size_t operator()(const std::pair<int, int> &cell) const noexcept;
	};

	std::unordered_set<std::pair<int, int>, CellHash> urban_cells_;
	int cell_size_ = 64;
	int bbox_min_x_ = 0;
	int bbox_min_z_ = 0;

public:
	static UrbanGroundLookup empty();
	bool is_urban(int x, int z) const;
	std::size_t cell_count() const { return urban_cells_.size(); }
	bool is_empty() const { return urban_cells_.empty(); }
	void add_cell(int cx, int cz) { urban_cells_.emplace(cx, cz); }
	void set_bounds(int x, int z, int size)
	{
		bbox_min_x_ = x;
		bbox_min_z_ = z;
		// Rust stores the configured cell size verbatim. In particular, a
		// negative size is not silently replaced by the default; it affects
		// coordinate-to-cell division just as it does while building the grid.
		cell_size_ = size;
	}
};

struct UrbanCluster
{
	std::vector<std::pair<int, int>> cells;
	std::vector<std::pair<int, int>> building_centroids;
	std::size_t building_count = 0;
};

class UrbanGroundComputer
{
	UrbanGroundConfig config_;
	std::vector<std::pair<int, int>> building_centroids_;
	cartesian::XZBBox bbox_;

public:
	UrbanGroundComputer(
			cartesian::XZBBox bbox, UrbanGroundConfig config = UrbanGroundConfig{});
	static UrbanGroundComputer with_defaults(cartesian::XZBBox bbox);
	void add_building_centroid(int x, int z);
	void add_building_centroids(const std::vector<std::pair<int, int>> &centroids);
	template <typename Range>
	void add_building_centroids(const Range &centroids)
	{
		for (const auto &[x, z] : centroids)
			add_building_centroid(x, z);
	}
	std::size_t building_count() const { return building_centroids_.size(); }
	std::vector<std::pair<int, int>> compute(
			const std::optional<std::chrono::duration<double>> &timeout =
					std::nullopt) const;
	UrbanGroundLookup compute_lookup() const;
};

std::optional<std::pair<int, int>> compute_centroid(
		const std::vector<std::pair<int, int>> &points);
std::vector<std::pair<int, int>> compute_urban_ground(
		const std::vector<std::pair<int, int>> &points, const cartesian::XZBBox &bbox,
		const std::optional<std::chrono::duration<double>> &timeout = std::nullopt);
UrbanGroundLookup compute_urban_ground_lookup(
		const std::vector<std::pair<int, int>> &points, const cartesian::XZBBox &bbox,
		const UrbanGroundConfig &config = UrbanGroundConfig{});
UrbanGroundLookup compute_urban_lookup(const std::vector<std::pair<int, int>> &points,
		int min_x, int min_z, const UrbanGroundConfig &config = UrbanGroundConfig{});

} // namespace arnis
