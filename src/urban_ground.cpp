#include "urban_ground.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <deque>
#include <unordered_map>

namespace arnis {
namespace {
using Cell=std::pair<int,int>;
using Points=std::vector<Cell>;
std::uint64_t urban_key(int x,int z) { return (std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(z); }
struct CellHash { std::size_t operator()(Cell c) const { return std::hash<std::uint64_t>{}(urban_key(c.first,c.second)); } };
using Grid=std::unordered_map<Cell,Points,CellHash>;
using Cells=std::unordered_set<Cell,CellHash>;

int adaptive_expansion(const UrbanGroundConfig &config, const Cells &dense, const Grid &grid)
{
	if (dense.size()<2) return config.cell_expansion;
	std::size_t total=0; int minx=INT_MAX,maxx=INT_MIN,minz=INT_MAX,maxz=INT_MIN;
	for (const auto &cell:dense) { total+=grid.at(cell).size(); minx=std::min(minx,cell.first); maxx=std::max(maxx,cell.first); minz=std::min(minz,cell.second); maxz=std::max(maxz,cell.second); }
	const double average=double(total)/dense.size();
	const double occupancy=double(dense.size())/double((maxx-minx+1)*(maxz-minz+1));
	const double density_factor=average<3.0?1.5:1.0;
	const double occupancy_factor=occupancy<0.4?1.5:(occupancy<0.6?1.25:1.0);
	return std::max(config.cell_expansion,std::min(4,int(std::ceil(config.cell_expansion*density_factor*occupancy_factor))));
}

Cells expanded(const Cells &cells,int amount)
{
	Cells out=cells; if(amount<=0) return out;
	for(auto [x,z]:cells) for(int dz=-amount;dz<=amount;++dz) for(int dx=-amount;dx<=amount;++dx) out.insert({x+dx,z+dz});
	return out;
}

std::vector<UrbanCluster> clusters(const UrbanGroundConfig &config,const Grid &grid)
{
	Cells dense; for(const auto &[cell,buildings]:grid) if(buildings.size()>=config.min_buildings_per_cell) dense.insert(cell);
	if(dense.empty()) return {};
	const Cells area=expanded(dense,adaptive_expansion(config,dense,grid));
	Cells visited; std::vector<UrbanCluster> out;
	for(const auto &start:area) {
		if(!visited.insert(start).second) continue;
		std::deque<Cell> queue{start}; UrbanCluster cluster;
		while(!queue.empty()) { const Cell current=queue.front();queue.pop_front();cluster.cells.push_back(current);
			for(int dz=-1;dz<=1;++dz) for(int dx=-1;dx<=1;++dx) { if(dx==0&&dz==0) continue; Cell next{current.first+dx,current.second+dz}; if(area.contains(next)&&visited.insert(next).second) queue.push_back(next); }
		}
		for(const auto &cell:cluster.cells) { auto it=grid.find(cell);if(it!=grid.end()) cluster.building_centroids.insert(cluster.building_centroids.end(),it->second.begin(),it->second.end()); }
		cluster.building_count=cluster.building_centroids.size();
		if(cluster.building_count>=config.min_buildings_for_cluster) out.push_back(std::move(cluster));
	}
	return out;
}
}

UrbanGroundLookup UrbanGroundLookup::empty() { return {}; }
bool UrbanGroundLookup::is_urban(int x,int z) const { return !cells_.empty() && cells_.contains(urban_key((x-min_x_)/cell_size_,(z-min_z_)/cell_size_)); }
void UrbanGroundLookup::add_cell(int x,int z) { cells_.insert(urban_key(x,z)); }

UrbanGroundComputer::UrbanGroundComputer(cartesian::XZBBox bbox, UrbanGroundConfig config):config_(std::move(config)),bbox_(std::move(bbox)) { if(config_.cell_size<=0) config_.cell_size=64; }
UrbanGroundComputer UrbanGroundComputer::with_defaults(cartesian::XZBBox bbox) { return UrbanGroundComputer(std::move(bbox)); }
void UrbanGroundComputer::add_building_centroid(int x,int z) { if(x>=bbox_.min_x()&&x<=bbox_.max_x()&&z>=bbox_.min_z()&&z<=bbox_.max_z()) building_centroids_.push_back({x,z}); }
void UrbanGroundComputer::add_building_centroids(const Points &points) { for(auto [x,z]:points) add_building_centroid(x,z); }

UrbanGroundLookup UrbanGroundComputer::compute_lookup() const
{
	UrbanGroundLookup result; result.set_bounds(bbox_.min_x(),bbox_.min_z(),config_.cell_size);
	if(building_centroids_.size()<config_.min_buildings_for_cluster) return result;
	Grid grid; for(auto [x,z]:building_centroids_) grid[{(x-bbox_.min_x())/config_.cell_size,(z-bbox_.min_z())/config_.cell_size}].push_back({x,z});
	for(const auto &cluster:clusters(config_,grid)) for(const auto &cell:cluster.cells) result.add_cell(cell.first,cell.second);
	return result;
}

Points UrbanGroundComputer::compute() const
{
	Points out; if(building_centroids_.size()<config_.min_buildings_for_cluster) return out;
	Grid grid; for(auto [x,z]:building_centroids_) grid[{(x-bbox_.min_x())/config_.cell_size,(z-bbox_.min_z())/config_.cell_size}].push_back({x,z});
	for(const auto &cluster:clusters(config_,grid)) for(auto [cx,cz]:cluster.cells) {
		const int x0=std::max(bbox_.min_x(),bbox_.min_x()+cx*config_.cell_size), x1=std::min(bbox_.max_x(),bbox_.min_x()+(cx+1)*config_.cell_size-1);
		const int z0=std::max(bbox_.min_z(),bbox_.min_z()+cz*config_.cell_size), z1=std::min(bbox_.max_z(),bbox_.min_z()+(cz+1)*config_.cell_size-1);
		if(x0>x1||z0>z1) continue; for(int x=x0;x<=x1;++x) for(int z=z0;z<=z1;++z) out.push_back({x,z});
	}
	return out;
}

std::optional<Cell> compute_centroid(const Points &points) { if(points.empty()) return std::nullopt; long long x=0,z=0;for(auto p:points){x+=p.first;z+=p.second;}return Cell{int(x/points.size()),int(z/points.size())}; }
Points compute_urban_ground(const Points &points,const cartesian::XZBBox &bbox) { UrbanGroundComputer computer(bbox);computer.add_building_centroids(points);return computer.compute(); }
UrbanGroundLookup compute_urban_ground_lookup(const Points &points,const cartesian::XZBBox &bbox,const UrbanGroundConfig &config) { UrbanGroundComputer computer(bbox,config);computer.add_building_centroids(points);return computer.compute_lookup(); }
UrbanGroundLookup compute_urban_lookup(const Points &points,int min_x,int min_z,const UrbanGroundConfig &config) { if(points.empty()){UrbanGroundLookup out;out.set_bounds(min_x,min_z,config.cell_size);return out;}int max_x=min_x,max_z=min_z;for(auto [x,z]:points){max_x=std::max(max_x,x);max_z=std::max(max_z,z);}return compute_urban_ground_lookup(points,cartesian::XZBBox::rect_from_min_max(min_x,min_z,max_x,max_z),config); }
}
