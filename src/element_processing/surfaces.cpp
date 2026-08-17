#include "surfaces.h"

#include <cstdint>
#include <unordered_map>

namespace arnis::surfaces
{

const std::vector<Block> *get_blocks_for_surface(const std::string &surface_type)
{
	using namespace block_definitions;
	static const std::unordered_map<std::string, std::vector<Block>> surfaces = {
			{"clay", {TERRACOTTA}},
			{"sand", {SAND}},
			{"tartan", {RED_TERRACOTTA}},
			{"grass", {GRASS_BLOCK}},
			{"grass_paver", {GRASS_BLOCK}},
			{"artificial_turf", {GREEN_WOOL}},
			{"dirt", {DIRT}},
			{"ground", {DIRT}},
			{"earth", {DIRT}},
			{"soil", {DIRT}},
			{"unpaved", {DIRT}},
			{"mud", {MUD}},
			{"mulch", {PODZOL}},
			{"woodchips", {PODZOL}},
			{"pebblestone", {COBBLESTONE}},
			{"cobblestone", {COBBLESTONE}},
			{"unhewn_cobblestone", {COBBLESTONE}},
			{"stepping_stones", {COBBLESTONE}},
			{"stone", {STONE}},
			{"rock", {STONE}},
			{"ice", {PACKED_ICE}},
			{"paving_stones", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"sett", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"paved", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"cement", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"chipseal", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"bitmac", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"concrete:plates", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"concrete:lanes", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"bricks", {BRICK}},
			{"brick", {BRICK}},
			{"metal", {IRON_BLOCK}},
			{"wood", {OAK_PLANKS}},
			{"asphalt", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
			{"gravel", {GRAVEL}},
			{"fine_gravel", {GRAVEL}},
			{"compacted", {GRAVEL}},
			{"concrete", {GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA}},
	};

	auto it = surfaces.find(surface_type);
	if (it == surfaces.end())
		return nullptr;
	return &it->second;
}

std::vector<Block> get_blocks_for_surface_way(
		const ProcessedWay &way, const std::vector<Block> &default_blocks)
{
	auto it = way.tags.find("surface");
	if (it != way.tags.end()) {
		if (const auto *blocks = get_blocks_for_surface(it->second))
			return *blocks;
	}
	return default_blocks;
}

Block semirandom_surface(int x, int z, const std::vector<Block> &block_types)
{
	if (block_types.empty())
		return block_definitions::STONE;
	uint32_t h = static_cast<uint32_t>(x) * 0x9E3779B9u ^
				 static_cast<uint32_t>(z) * 0x517CC1B7u;
	h ^= h >> 16;
	h *= 0x45D9F3Bu;
	h ^= h >> 16;
	return block_types[h % block_types.size()];
}

}
