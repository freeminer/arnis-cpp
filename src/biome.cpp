#include "biome.h"
#include "land_cover/land_cover.h"
#include "../../arnis_adapter.h"
#include <cmath>
#include <algorithm>
namespace arnis::biome
{
std::string biome_for_class(std::uint8_t lc, Climate c, double lat, std::uint8_t wd)
{
	const double a = std::abs(lat);
	if (lc == land_cover::LC_WATER) {
		bool cold = c == Climate::IceCap || c == Climate::Tundra || c == Climate::Boreal;
		if (wd < 8)
			return cold ? "minecraft:frozen_river" : "minecraft:river";
		if (cold)
			return wd >= 12 ? "minecraft:deep_frozen_ocean" : "minecraft:frozen_ocean";
		if (a < 23.5 || c == Climate::HotDesert || c == Climate::HotSteppe ||
				c == Climate::TropicalSavanna)
			return "minecraft:warm_ocean";
		if (a < 45)
			return wd >= 12 ? "minecraft:deep_lukewarm_ocean"
							: "minecraft:lukewarm_ocean";
		return wd >= 12 ? "minecraft:deep_cold_ocean" : "minecraft:cold_ocean";
	}
	switch (c) {
	case Climate::HotDesert:
	case Climate::ColdDesert:
		return "minecraft:desert";
	case Climate::HotSteppe:
	case Climate::TropicalSavanna:
	case Climate::DryContinental:
		return "minecraft:savanna";
	case Climate::ColdSteppe:
		return "minecraft:plains";
	case Climate::Tundra:
	case Climate::IceCap:
		return "minecraft:snowy_plains";
	case Climate::Boreal:
		return lc == land_cover::LC_WETLAND
					   ? "minecraft:swamp"
					   : (lc == land_cover::LC_TREE_COVER || lc == land_cover::LC_MOSS
										 ? "minecraft:taiga"
										 : "minecraft:snowy_plains");
	case Climate::Temperate:
		break;
	}
	if (lc == land_cover::LC_TREE_COVER)
		return a > 55 ? "minecraft:taiga"
					  : (a < 23.5 ? "minecraft:jungle" : "minecraft:forest");
	if (lc == land_cover::LC_SHRUBLAND)
		return a < 23.5 ? "minecraft:sparse_jungle" : "minecraft:savanna";
	if (lc == land_cover::LC_BARE)
		return "minecraft:desert";
	if (lc == land_cover::LC_SNOW_ICE)
		return "minecraft:snowy_plains";
	if (lc == land_cover::LC_WETLAND)
		return "minecraft:swamp";
	if (lc == land_cover::LC_MANGROVES)
		return "minecraft:mangrove_swamp";
	if (lc == land_cover::LC_MOSS)
		return "minecraft:taiga";
	return "minecraft:plains";
}

unsigned biome_bits_per_index(std::size_t palette_size)
{
	if(palette_size<=1)return 0;
	unsigned bits=0;for(std::size_t n=palette_size-1;n;n>>=1)++bits;
	return bits;
}
std::vector<std::int64_t> pack_chunk_biome_indices(const std::array<std::uint8_t,16> &indices,unsigned bits)
{
	if(bits==0||bits>6)return {};
	const std::size_t per_long=64/bits, words=(64+per_long-1)/per_long;
	std::vector<std::uint64_t> packed(words,0);const std::uint64_t mask=(std::uint64_t(1)<<bits)-1;
	for(std::size_t cell=0;cell<64;++cell) {
		const auto word=cell/per_long,shift=(cell%per_long)*bits;
		packed[word]|=(std::uint64_t(indices[cell%16])&mask)<<shift;
	}
	return {packed.begin(),packed.end()};
}
ChunkBiomeData build_chunk_biomes(int chunk_x,int chunk_z,const BiomeSampler &sample,Climate climate,double latitude)
{
	ChunkBiomeData out;std::array<std::string,16> names{};
	for(int z=0;z<4;++z)for(int x=0;x<4;++x) {
		const auto value=sample?sample(chunk_x*16+x*4+2,chunk_z*16+z*4+2):BiomeSample{};
		names[std::size_t(z*4+x)]=biome_for_class(value.land_cover,climate,latitude,value.water_distance);
	}
	for(std::size_t i=0;i<names.size();++i) {
		auto it=std::find(out.palette.begin(),out.palette.end(),names[i]);
		if(it==out.palette.end()){out.horizontal_indices[i]=std::uint8_t(out.palette.size());out.palette.push_back(names[i]);}
		else out.horizontal_indices[i]=std::uint8_t(std::distance(out.palette.begin(),it));
	}
	out.bits_per_index=biome_bits_per_index(out.palette.size());
	if(out.bits_per_index)out.packed_indices=pack_chunk_biome_indices(out.horizontal_indices,out.bits_per_index);
	return out;
}

ChunkBiomeData build_chunk_biomes_for_ground(int chunk_x, int chunk_z,
		const ::arnis::Ground *ground, double center_latitude, int world_origin_x,
		int world_origin_z)
{
	if (!ground)
		return build_chunk_biomes(chunk_x, chunk_z, {}, Climate::Temperate, center_latitude);
	const BiomeSampler sample = [ground, world_origin_x, world_origin_z](int world_x,
															 int world_z) {
		const XZPoint local{world_x - world_origin_x, world_z - world_origin_z};
		return BiomeSample{ground->cover_class(local), ground->water_distance(local)};
	};
	return build_chunk_biomes(chunk_x, chunk_z, sample, ground->climate(), center_latitude);
}
}
