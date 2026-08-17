#include "climate.h"
#include "land_cover/land_cover.h"
#include "block_definitions.h"
#include "deterministic_rng.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <utility>
namespace arnis::climate
{
namespace
{
constexpr std::size_t KOPPEN_COLS = 3600, KOPPEN_ROWS = 1800;
const std::vector<std::uint8_t> &koppen_grid()
{
	static const std::vector<std::uint8_t> grid=[] {
		const auto path=std::filesystem::path(__FILE__).parent_path().parent_path() /
				"assets/climate/koppen_0p1.bin";
		std::ifstream in(path,std::ios::binary);
		if(!in) return std::vector<std::uint8_t>{};
		return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in),{});
	}();
	return grid;
}
}
std::optional<std::pair<Block,Block>> surface_palette(arnis::biome::Climate c,std::uint8_t cover,int x,int z)
{
	using namespace arnis::biome; using namespace arnis::land_cover; using namespace arnis::block_definitions;
	if(c==Climate::Temperate||c==Climate::TropicalSavanna||c==Climate::DryContinental) return std::nullopt;
	const bool veg=cover==LC_TREE_COVER||cover==LC_SHRUBLAND||cover==LC_GRASSLAND||cover==LC_CROPLAND||cover==LC_MOSS, bare=cover==LC_BARE||cover==LC_SNOW_ICE;
	if (!veg && !bare)
		return std::nullopt;
	const auto h = coord_hash(x, z);
	if(c==Climate::IceCap) return h%6==0?std::make_pair(PACKED_ICE,PACKED_ICE):std::make_pair(SNOW_BLOCK,SNOW_BLOCK);
	if(c==Climate::HotDesert) return h%12==0?std::make_pair(SANDSTONE,SANDSTONE):h%12==1?std::make_pair(SMOOTH_SANDSTONE,SANDSTONE):std::make_pair(SAND,SANDSTONE);
	if(c==Climate::HotSteppe)
		return bare ? (h%10<5 ? std::make_pair(SAND,SANDSTONE) : std::make_pair(COARSE_DIRT,DIRT))
			: (h%10<3 ? std::make_pair(SAND,SANDSTONE) : h%10<6 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(GRASS_BLOCK,DIRT));
	if(c==Climate::ColdDesert)
		return bare ? (h%12<5 ? std::make_pair(GRAVEL,STONE) : h%12<9 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(STONE,STONE))
			: (h%10<5 ? std::make_pair(COARSE_DIRT,DIRT) : h%10<8 ? std::make_pair(GRAVEL,STONE) : std::make_pair(GRASS_BLOCK,DIRT));
	if(c==Climate::ColdSteppe)
		return bare ? (h%10<6 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(GRAVEL,STONE))
			: (h%10<3 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(GRASS_BLOCK,DIRT));
	if(c==Climate::Boreal)
		return bare ? (h%10<5 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(GRAVEL,STONE))
			: (h%10<4 ? std::make_pair(PODZOL,DIRT) : h%10<6 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(GRASS_BLOCK,DIRT));
	if(c==Climate::Tundra)
		return bare ? (h%10<5 ? std::make_pair(GRAVEL,STONE) : h%10<8 ? std::make_pair(COARSE_DIRT,DIRT) : std::make_pair(STONE,STONE))
			: (h%10<4 ? std::make_pair(COARSE_DIRT,DIRT) : h%10<6 ? std::make_pair(MOSS_BLOCK,DIRT) : std::make_pair(GRASS_BLOCK,DIRT));
	return std::nullopt;
}
arnis::biome::Climate from_koppen_class(unsigned char c)
{
	switch (c) {
	case 3:
		return arnis::biome::Climate::TropicalSavanna;
	case 4:
		return arnis::biome::Climate::HotDesert;
	case 5:
		return arnis::biome::Climate::ColdDesert;
	case 6:
		return arnis::biome::Climate::HotSteppe;
	case 7:
		return arnis::biome::Climate::ColdSteppe;
	case 17:
	case 18:
	case 21:
	case 22:
		return arnis::biome::Climate::DryContinental;
	case 19:
	case 20:
	case 23:
	case 24:
	case 27:
	case 28:
		return arnis::biome::Climate::Boreal;
	case 29:
		return arnis::biome::Climate::Tundra;
	case 30:
		return arnis::biome::Climate::IceCap;
	default:
		return arnis::biome::Climate::Temperate;
	}
}
arnis::biome::Climate classify(double latitude,double longitude)
{
	const auto &grid=koppen_grid();
	if(grid.size()!=KOPPEN_COLS*KOPPEN_ROWS || !std::isfinite(latitude) || !std::isfinite(longitude))
		return arnis::biome::Climate::Temperate;
	const auto col=std::clamp<long>(long(std::floor((longitude+180.)/.1)),0,long(KOPPEN_COLS-1));
	const auto row=std::clamp<long>(long(std::floor((90.-latitude)/.1)),0,long(KOPPEN_ROWS-1));
	return from_koppen_class(grid[std::size_t(row)*KOPPEN_COLS+std::size_t(col)]);
}
arnis::biome::Climate classify_bbox(double min_latitude,double min_longitude,
		double max_latitude,double max_longitude)
{
	return classify((min_latitude+max_latitude)*.5,(min_longitude+max_longitude)*.5);
}
}
