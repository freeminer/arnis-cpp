#pragma once
#include "coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include <cstddef>
#include <optional>
#include <unordered_set>
#include <vector>
#include <utility>
namespace arnis {
struct UrbanGroundConfig {
	int cell_size=64;
	std::size_t min_buildings_per_cell=1, min_buildings_for_cluster=5;
	double concavity=2.0;
	bool expand_hull=true;
	int cell_expansion=2;
};
class UrbanGroundLookup {
	std::unordered_set<long long> cells_;
	int cell_size_=64, min_x_=0, min_z_=0;
public:
	bool is_urban(int x,int z) const;
	std::size_t cell_count() const{return cells_.size();}
	bool empty() const{return cells_.empty();}
	void add_cell(int cx,int cz);
	void set_bounds(int x,int z,int size){min_x_=x;min_z_=z;cell_size_=size>0?size:64;}
};
struct UrbanCluster { std::vector<std::pair<int,int>> cells, building_centroids; std::size_t building_count=0; };
class UrbanGroundComputer {
	UrbanGroundConfig config_;
	std::vector<std::pair<int,int>> building_centroids_;
	cartesian::XZBBox bbox_;
public:
	UrbanGroundComputer(cartesian::XZBBox bbox, UrbanGroundConfig config=UrbanGroundConfig{});
	void add_building_centroid(int x, int z);
	void add_building_centroids(const std::vector<std::pair<int,int>> &centroids);
	std::size_t building_count() const { return building_centroids_.size(); }
	std::vector<std::pair<int,int>> compute() const;
	UrbanGroundLookup compute_lookup() const;
};
std::optional<std::pair<int,int>> compute_centroid(const std::vector<std::pair<int,int>> &);
std::vector<std::pair<int,int>> compute_urban_ground(const std::vector<std::pair<int,int>> &, const cartesian::XZBBox &);
UrbanGroundLookup compute_urban_ground_lookup(const std::vector<std::pair<int,int>> &, const cartesian::XZBBox &, const UrbanGroundConfig &config=UrbanGroundConfig{});
// Compatibility entry point retained for existing C++ callers without a bbox.
UrbanGroundLookup compute_urban_lookup(const std::vector<std::pair<int,int>> &,int min_x,int min_z,const UrbanGroundConfig & config=UrbanGroundConfig{});
}
