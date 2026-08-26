#include "landmarks.h"
#include "../../arnis_adapter.h"
#include "structures/schem_decoder.h"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace arnis::landmarks {
namespace {
using Key = std::pair<std::string, std::uint64_t>;

bool contains(const std::vector<Key> &keys, const Key &key)
{
	return std::find(keys.begin(), keys.end(), key) != keys.end();
}

std::string tag(const ProcessedElement &element, const char *name)
{
	auto it = element.tags().find(name);
	return it == element.tags().end() ? std::string{} : it->second;
}

std::vector<std::pair<int, int>> points(const ProcessedElement &element)
{
	std::vector<std::pair<int, int>> result;
	if (element.is_node())
		result.push_back({element.as_node().x, element.as_node().z});
	else if (element.is_way())
		for (const auto &node : element.as_way().nodes)
			result.push_back({node.x, node.z});
	else if (element.is_relation())
		for (const auto &member : element.as_relation().members)
			for (const auto &node : member.way.nodes)
				result.push_back({node.x, node.z});
	return result;
}

bool replaceable(const ProcessedElement &element)
{
	const auto tags = element.tags();
	if (tags.find("building") != tags.end() || tags.find("building:part") != tags.end())
		return true;
	const auto leisure = tag(element, "leisure");
	return leisure == "stadium" || leisure == "pitch" || leisure == "track" ||
		leisure == "sports_centre";
}

std::vector<std::pair<int, int>> regions_around(int x, int z, int radius)
{
	std::vector<std::pair<int, int>> out;
	for (int rz = (z - radius) >> 9; rz <= (z + radius) >> 9; ++rz)
		for (int rx = (x - radius) >> 9; rx <= (x + radius) >> 9; ++rx)
			out.push_back({rx, rz});
	return out;
}

std::pair<double,double> rotate(double x,double z,unsigned quarter)
{
	switch(quarter&3) { case 1:return {-z,x}; case 2:return {-x,-z}; case 3:return {z,-x}; default:return {x,z}; }
}

std::filesystem::path asset_path(const Landmark &landmark)
{
	return std::filesystem::path(__FILE__).parent_path().parent_path() / landmark.schematic_path;
}

int median_base_y(WorldEditor &editor,const Placement &placement,double scale,unsigned rotation)
{
	std::vector<int> levels; levels.reserve(289);
	const auto &landmark=*placement.landmark;
	for(int ix=0;ix<=16;++ix) for(int iz=0;iz<=16;++iz) {
		const double mx=landmark.anchor_x+landmark.suppress_half_x*(2.0*ix/16.0-1.0);
		const double mz=landmark.anchor_z+landmark.suppress_half_z*(2.0*iz/16.0-1.0);
		auto [dx,dz]=rotate((mx-landmark.anchor_x)*scale,(mz-landmark.anchor_z)*scale,rotation);
		levels.push_back(editor.get_ground_level(int(std::lround(placement.world_x+dx)),int(std::lround(placement.world_z+dz))));
	}
	std::nth_element(levels.begin(),levels.begin()+levels.size()/2,levels.end());
	return levels[levels.size()/2]+1+landmark.ground_offset;
}
} // namespace

const std::vector<Landmark> &catalogue()
{
	static const std::vector<Landmark> entries = {
		{"Olympiastadion M\xC3\xBCnchen", "Q131610", {{"way", 419656920}},
			"assets/structures/landmarks/olympiastadion_munich.schem", 48.1731012, 11.5464833,
			224.5, 238.0, 12, -20, {"green_wool", "green_stained_hardened_clay"}, 10, 135., 145., 240},
		{"Olympiahalle", "Q48849", {{"way", 303099272}},
			"assets/structures/landmarks/olympiahalle_munich.schem", 48.1749058, 11.5500308,
			115., 130., 0, -9, {"white_concrete"}, 0, 95., 75., 160},
		{"Olympia-Schwimmhalle", "Q3882013", {{"way", 227012665}},
			"assets/structures/landmarks/olympia_schwimmhalle_munich.schem", 48.1735723, 11.5514797,
			99., 126.5, 0, -6, {"glass"}, 0, 50., 70., 160},
		{"Olympiaturm", "Q599148", {{"way", 164084344}},
			"assets/structures/landmarks/olympiaturm_munich.schem", 48.1744095, 11.5537401,
			11.5, 13.5, 0, 0, {}, 0, 22., 22., 30},
	};
	return entries;
}

PrescanResult prescan(const std::vector<ProcessedElement> &elements,
		const std::vector<WorldAnchor> &anchors, double scale,
		const std::vector<Key> &already_suppressed)
{
	PrescanResult result;
	if (scale <= 0.0)
		return result;
	for (const auto &landmark : catalogue()) {
		auto anchor = std::find_if(anchors.begin(), anchors.end(), [&](const WorldAnchor &value) {
			return std::string(value.qid ? value.qid : "") == landmark.qid;
		});
		if (anchor == anchors.end())
			continue;
		const Placement placement{&landmark, anchor->x, anchor->z};
		result.placements.push_back(placement);
		const int radius = static_cast<int>(std::ceil(landmark.reach_m * scale));
		auto regions = regions_around(anchor->x, anchor->z, radius);
		result.deferred_regions.insert(result.deferred_regions.end(), regions.begin(), regions.end());
		for (const auto &element : elements) {
			const Key key{std::string(element.kind()), element.id()};
			if (contains(already_suppressed, key) || contains(result.suppressed, key))
				continue;
			const bool is_landmark = tag(element, "wikidata") == landmark.qid ||
				contains(landmark.osm_ids, key);
			if (is_landmark) {
				result.suppressed.push_back(key);
				continue;
			}
			if (!replaceable(element))
				continue;
			const auto geometry = points(element);
			if (geometry.empty())
				continue;
			long long sum_x = 0, sum_z = 0;
			for (const auto [x, z] : geometry) { sum_x += x; sum_z += z; }
			const double dx = (sum_x / static_cast<double>(geometry.size()) - anchor->x) / scale;
			const double dz = (sum_z / static_cast<double>(geometry.size()) - anchor->z) / scale;
			if (std::abs(dx) <= landmark.suppress_half_x && std::abs(dz) <= landmark.suppress_half_z)
				result.suppressed.push_back(key);
		}
	}
	std::sort(result.suppressed.begin(), result.suppressed.end());
	result.suppressed.erase(std::unique(result.suppressed.begin(), result.suppressed.end()), result.suppressed.end());
	std::sort(result.deferred_regions.begin(), result.deferred_regions.end());
	result.deferred_regions.erase(std::unique(result.deferred_regions.begin(), result.deferred_regions.end()), result.deferred_regions.end());
	return result;
}

bool place(WorldEditor &editor,const Placement &placement,double scale,unsigned rotation)
{
	if(!placement.landmark || scale<=0.0) return false;
	std::ifstream input(asset_path(*placement.landmark),std::ios::binary);
	if(!input) return false;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),{});
	structures::SchemDocument document;
	try { document=structures::decode_sponge_schem(bytes); } catch(...) { return false; }
	const int base=median_base_y(editor,placement,scale,rotation);
	const auto [min_x,min_z]=editor.get_min_coords();
	const auto [max_x,max_z]=editor.get_max_coords();
	std::size_t placed=0;
	for(const auto &voxel:document.voxels) {
		const Block block=structures::resolve_schem_block(voxel.block);
		if(block==AIR) continue;
		auto [dx,dz]=rotate((voxel.x-placement.landmark->anchor_x)*scale,
				(voxel.z-placement.landmark->anchor_z)*scale,rotation);
		const int x=int(std::lround(placement.world_x+dx));
		const int z=int(std::lround(placement.world_z+dz));
		if(x<min_x||x>max_x||z<min_z||z>max_z) continue;
		const int y0=base+int(std::lround((voxel.y-placement.landmark->ground_y)*scale));
		const int y1=std::max(y0+1,base+int(std::lround((voxel.y+1-placement.landmark->ground_y)*scale)));
		for(int y=y0;y<y1;++y) { editor.set_block_absolute(block,x,y,z); ++placed; }
	}
	return placed>0;
}

std::size_t place_all(WorldEditor &editor,const PrescanResult &prescan,double scale,unsigned rotation)
{
	std::size_t placed=0;
	for(const auto &placement:prescan.placements) placed+=place(editor,placement,scale,rotation)?1:0;
	return placed;
}
} // namespace arnis::landmarks
