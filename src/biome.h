#pragma once
#include <cstdint>
#include <array>
#include <functional>
#include <string>
#include <vector>
namespace arnis { struct Ground; }
namespace arnis::biome
{
enum class Climate
{
	Temperate,
	TropicalSavanna,
	HotDesert,
	HotSteppe,
	ColdDesert,
	ColdSteppe,
	DryContinental,
	Boreal,
	Tundra,
	IceCap
};
std::string biome_for_class(std::uint8_t land_cover, Climate climate, double latitude,
		std::uint8_t water_distance);
// The exporter-independent form of Rust's 1.18+ chunk biome palette.  The
// 16 horizontal values repeat through the four vertical biome cells.
struct BiomeSample { std::uint8_t land_cover=0,water_distance=0; };
using BiomeSampler=std::function<BiomeSample(int world_x,int world_z)>;
struct ChunkBiomeData {
	std::vector<std::string> palette;
	std::vector<std::int64_t> packed_indices;
	std::array<std::uint8_t,16> horizontal_indices{};
	unsigned bits_per_index=0;
};
unsigned biome_bits_per_index(std::size_t palette_size);
std::vector<std::int64_t> pack_chunk_biome_indices(const std::array<std::uint8_t,16> &,unsigned bits);
ChunkBiomeData build_chunk_biomes(int chunk_x,int chunk_z,const BiomeSampler &,Climate,double center_latitude);
// Direct counterpart of Rust's build_chunk_biome_nbt.  Ground stores grids in
// map-local coordinates, so origin translates an absolute chunk position into
// that grid; leave it at zero for a standalone Ground.
ChunkBiomeData build_chunk_biomes_for_ground(int chunk_x, int chunk_z,
		const ::arnis::Ground *ground, double center_latitude,
		int world_origin_x = 0, int world_origin_z = 0);
}
