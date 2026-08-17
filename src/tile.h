#pragma once
#include "osm_parser.h"
#include <vector>
#include <utility>
#include <algorithm>
#include <functional>
#include <array>
#include <cmath>
#include <unordered_map>
namespace arnis::tiles { struct TileBounds{int min_x,min_z,max_x,max_z; bool contains(int x,int z)const{return x>=min_x&&x<max_x&&z>=min_z&&z<max_z;} TileBounds expanded(int h)const{return{min_x-h,min_z-h,max_x+h,max_z+h};}}; constexpr int DEFAULT_TILE_SIZE=512; constexpr int TILE_EDITOR_HALO=64; std::vector<TileBounds> create_tiles(int min_x,int min_z,int max_x,int max_z,int tile_size=DEFAULT_TILE_SIZE); }
namespace arnis::tiles { inline bool intersects(const TileBounds&t,int min_x,int min_z,int max_x,int max_z){return min_x<t.max_x&&max_x>=t.min_x&&min_z<t.max_z&&max_z>=t.min_z;} }
namespace arnis::tiles { inline std::pair<int,int> region_key(int x,int z){return {x>=0?x/512:(x-511)/512,z>=0?z/512:(z-511)/512};} }
namespace arnis::tiles { inline std::vector<std::pair<int,int>> region_keys_around(int x,int z,int radius){auto a=region_key(x-radius,z-radius),b=region_key(x+radius,z+radius);std::vector<std::pair<int,int>> out;for(int rz=a.second;rz<=b.second;++rz)for(int rx=a.first;rx<=b.first;++rx)out.emplace_back(rx,rz);return out;} }
namespace arnis::tiles { inline std::size_t tile_count_for_bounds(int min_x,int min_z,int max_x,int max_z,int tile_size=DEFAULT_TILE_SIZE){return create_tiles(min_x,min_z,max_x,max_z,tile_size).size();} }
namespace arnis::tiles {
inline bool element_bounds(const ProcessedElement &element, int &min_x, int &min_z, int &max_x, int &max_z)
{
	bool any = false;
	auto add = [&](int x, int z) { if (!any) { min_x=max_x=x; min_z=max_z=z; any=true; } else { min_x=std::min(min_x,x); max_x=std::max(max_x,x); min_z=std::min(min_z,z); max_z=std::max(max_z,z); } };
	if (element.is_node()) add(element.as_node().x, element.as_node().z);
	else if (element.is_way()) for (const auto &n : element.as_way().nodes) add(n.x,n.z);
	else for (const auto &m : element.as_relation().members) for (const auto &n : m.way.nodes) add(n.x,n.z);
	return any;
}
}
struct ElementTileBounds { int min_x,min_z,max_x,max_z; bool linear=false; };
namespace arnis::tiles { inline std::vector<std::vector<std::size_t>> assign_elements(const std::vector<ElementTileBounds>& els,const std::vector<TileBounds>& ts,int halo=TILE_EDITOR_HALO){std::vector<std::vector<std::size_t>> out(ts.size());for(std::size_t i=0;i<els.size();++i){auto e=els[i];for(std::size_t j=0;j<ts.size();++j){auto t=ts[j].expanded(halo);if(intersects(t,e.min_x,e.min_z,e.max_x,e.max_z))out[j].push_back(i);}}return out;} }
namespace arnis::tiles { inline std::vector<std::size_t> elements_for_tile(const std::vector<std::vector<std::size_t>>&a,std::size_t i){return i<a.size()?a[i]:std::vector<std::size_t>{};} }
namespace arnis::tiles { inline void deduplicate_assignments(std::vector<std::vector<std::size_t>>&a){for(auto&v:a){std::sort(v.begin(),v.end());v.erase(std::unique(v.begin(),v.end()),v.end());}} }
namespace arnis::tiles { inline std::size_t assigned_element_count(const std::vector<std::vector<std::size_t>>&a){std::size_t n=0;for(const auto&v:a)n+=v.size();return n;} }
namespace arnis::tiles { inline std::vector<std::vector<std::size_t>> assign_points(const std::vector<std::pair<int,int>>&pts,const std::vector<TileBounds>&ts){std::vector<std::vector<std::size_t>>o(ts.size());for(std::size_t i=0;i<pts.size();++i)for(std::size_t j=0;j<ts.size();++j)if(ts[j].contains(pts[i].first,pts[i].second)){o[j].push_back(i);break;}return o;} }
namespace arnis::tiles { inline std::vector<std::size_t> tiles_intersecting(const TileBounds&box,const std::vector<TileBounds>&ts,int halo=TILE_EDITOR_HALO){std::vector<std::size_t>o;auto e=box.expanded(halo);for(std::size_t i=0;i<ts.size();++i)if(intersects(e,ts[i].min_x,ts[i].min_z,ts[i].max_x,ts[i].max_z))o.push_back(i);return o;} }
namespace arnis::tiles { inline std::vector<std::pair<int,int>> region_keys_for_tile(const TileBounds&t){auto a=region_key(t.min_x,t.min_z),b=region_key(t.max_x-1,t.max_z-1);std::vector<std::pair<int,int>>o;for(int z=a.second;z<=b.second;++z)for(int x=a.first;x<=b.first;++x)o.emplace_back(x,z);return o;} }
namespace arnis::tiles { inline std::vector<TileBounds> expanded_tiles(const std::vector<TileBounds>&ts,int halo){std::vector<TileBounds>o;o.reserve(ts.size());for(const auto&t:ts)o.push_back(t.expanded(halo));return o;} }
namespace arnis::tiles {
inline std::vector<std::vector<std::size_t>> assign_elements_region_indexed(
		const std::vector<ElementTileBounds> &elements, const std::vector<TileBounds> &tiles,
		int halo = TILE_EDITOR_HALO)
{
	std::vector<std::vector<std::size_t>> out(tiles.size());
	std::unordered_map<std::pair<int,int>, std::vector<std::size_t>,
			std::function<std::size_t(const std::pair<int,int>&)>> index(0,
			[](const auto &p){ return std::hash<int>{}(p.first) ^ (std::hash<int>{}(p.second)<<1); });
	for (std::size_t i=0;i<tiles.size();++i)
		for (auto key: region_keys_for_tile(tiles[i])) index[key].push_back(i);
	for (std::size_t ei=0;ei<elements.size();++ei) {
		auto e=elements[ei]; std::array<int,4> rr={(e.min_x-halo)>>9,(e.max_x+halo)>>9,(e.min_z-halo)>>9,(e.max_z+halo)>>9};
		std::vector<std::size_t> candidates;
		for(int rz=rr[2];rz<=rr[3];++rz) for(int rx=rr[0];rx<=rr[1];++rx) {
			auto it=index.find({rx,rz}); if(it!=index.end()) candidates.insert(candidates.end(),it->second.begin(),it->second.end());
		}
		std::sort(candidates.begin(),candidates.end()); candidates.erase(std::unique(candidates.begin(),candidates.end()),candidates.end());
		for(auto ti:candidates) { auto t=tiles[ti].expanded(halo); if(intersects(t,e.min_x,e.min_z,e.max_x,e.max_z)) out[ti].push_back(ei); }
	}
	return out;
}
struct AssignmentStats { std::size_t elements=0, assignments=0, empty_tiles=0, max_per_tile=0; };
inline AssignmentStats assignment_stats(const std::vector<std::vector<std::size_t>> &a,std::size_t element_count){AssignmentStats s; s.elements=element_count; for(const auto&v:a){s.assignments+=v.size(); if(v.empty())++s.empty_tiles; s.max_per_tile=std::max(s.max_per_tile,v.size());} return s;}
inline std::vector<std::size_t> tiles_for_element(const std::vector<std::vector<std::size_t>> &a,std::size_t element){std::vector<std::size_t> out;for(std::size_t i=0;i<a.size();++i)if(std::find(a[i].begin(),a[i].end(),element)!=a[i].end())out.push_back(i);return out;}
inline std::vector<std::size_t> unique_assigned_elements(const std::vector<std::vector<std::size_t>> &a){std::vector<std::size_t> out;for(const auto&v:a)out.insert(out.end(),v.begin(),v.end());std::sort(out.begin(),out.end());out.erase(std::unique(out.begin(),out.end()),out.end());return out;}
inline std::vector<std::vector<std::size_t>> assign_relation_members(const std::vector<std::vector<ElementTileBounds>>& relations,const std::vector<TileBounds>& tiles,int halo=TILE_EDITOR_HALO){std::vector<std::vector<std::size_t>> out(tiles.size());for(std::size_t ri=0;ri<relations.size();++ri){for(std::size_t ti=0;ti<tiles.size();++ti){auto t=tiles[ti].expanded(halo);for(const auto&e:relations[ri])if(intersects(t,e.min_x,e.min_z,e.max_x,e.max_z)){out[ti].push_back(ri);break;}}}return out;}
inline void merge_assignments(std::vector<std::vector<std::size_t>>&dst,const std::vector<std::vector<std::size_t>>&src,std::size_t index_offset=0){if(dst.size()<src.size())dst.resize(src.size());for(std::size_t i=0;i<src.size();++i)for(auto id:src[i])dst[i].push_back(id+index_offset);deduplicate_assignments(dst);}
inline std::vector<std::size_t> assignment_histogram(const std::vector<std::vector<std::size_t>>&a){std::vector<std::size_t> h;for(const auto&v:a){if(v.size()>=h.size())h.resize(v.size()+1);++h[v.size()];}return h;}
inline std::vector<std::size_t> tiles_with_assignments(const std::vector<std::vector<std::size_t>>&a){std::vector<std::size_t> out;for(std::size_t i=0;i<a.size();++i)if(!a[i].empty())out.push_back(i);return out;}
inline std::vector<std::size_t> empty_tiles(const std::vector<std::vector<std::size_t>>&a){std::vector<std::size_t> out;for(std::size_t i=0;i<a.size();++i)if(a[i].empty())out.push_back(i);return out;}
}
namespace arnis::tiles { inline std::size_t tile_area(const TileBounds&t){return static_cast<std::size_t>(std::max(0,t.max_x-t.min_x))*static_cast<std::size_t>(std::max(0,t.max_z-t.min_z));} }
namespace arnis::tiles { inline TileBounds clamp_to(const TileBounds&t,int min_x,int min_z,int max_x,int max_z){return{std::max(t.min_x,min_x),std::max(t.min_z,min_z),std::min(t.max_x,max_x),std::min(t.max_z,max_z)};} }
namespace arnis::tiles { inline bool valid(const TileBounds&t){return t.min_x<t.max_x&&t.min_z<t.max_z;} inline bool operator==(const TileBounds&a,const TileBounds&b){return a.min_x==b.min_x&&a.min_z==b.min_z&&a.max_x==b.max_x&&a.max_z==b.max_z;} }
namespace arnis::tiles { inline std::pair<int,int> center(const TileBounds&t){return {(t.min_x+t.max_x-1)/2,(t.min_z+t.max_z-1)/2};} }
namespace arnis::tiles { inline std::pair<int,int> dimensions(const TileBounds&t){return {std::max(0,t.max_x-t.min_x),std::max(0,t.max_z-t.min_z)};} }
namespace arnis::tiles { inline TileBounds translated(const TileBounds&t,int dx,int dz){return{t.min_x+dx,t.min_z+dz,t.max_x+dx,t.max_z+dz};} }
namespace arnis::tiles { inline std::size_t overlap_area(const TileBounds&a,const TileBounds&b){int x=std::max(0,std::min(a.max_x,b.max_x)-std::max(a.min_x,b.min_x));int z=std::max(0,std::min(a.max_z,b.max_z)-std::max(a.min_z,b.min_z));return static_cast<std::size_t>(x)*z;} }
namespace arnis::tiles { inline std::size_t containing_tile(const std::vector<TileBounds>&ts,int x,int z){for(std::size_t i=0;i<ts.size();++i)if(ts[i].contains(x,z))return i;return ts.size();} }
namespace arnis::tiles { inline std::array<int,4> region_range(int min_x,int min_z,int max_x,int max_z,int halo){return {(min_x-halo)>>9,(max_x+halo)>>9,(min_z-halo)>>9,(max_z+halo)>>9};} }
namespace arnis::tiles { inline int linear_halo(double scale){return std::max(TILE_EDITOR_HALO,static_cast<int>(std::ceil(40.0*scale)));} }
namespace arnis::tiles { inline std::pair<int,int> region_for_point(int x,int z){return {x>>9,z>>9};} }
namespace arnis::tiles { inline bool point_in_region(int x,int z,int rx,int rz){auto p=region_for_point(x,z);return p.first==rx&&p.second==rz;} }
namespace std { template<> struct hash<arnis::tiles::TileBounds>{ size_t operator()(const arnis::tiles::TileBounds&t) const noexcept { size_t h=std::hash<int>{}(t.min_x); h^=std::hash<int>{}(t.min_z)+(h<<6)+(h>>2); h^=std::hash<int>{}(t.max_x)+(h<<6)+(h>>2); return h^=std::hash<int>{}(t.max_z)+(h<<6)+(h>>2); } }; }
namespace arnis::tiles { inline TileBounds make(int min_x,int min_z,int max_x,int max_z){return{min_x,min_z,max_x,max_z};} }
