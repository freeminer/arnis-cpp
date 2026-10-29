#include "surfaces.h"

#include <cstdint>
#include <cctype>
#include <algorithm>
#include <array>
#include <unordered_map>
#include <vector>

namespace arnis::surfaces
{

std::span<const Block> get_blocks_for_surface(const std::string &surface_type)
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
	return it == surfaces.end() ? std::span<const Block>{}
								: std::span<const Block>(it->second);
}

std::span<const Block> get_blocks_for_surface_way(
		const ProcessedWay &way, std::span<const Block> default_blocks)
{
	auto it = way.tags.find("surface");
	if (it != way.tags.end()) {
		if (const auto blocks = get_blocks_for_surface(it->second); !blocks.empty())
			return blocks;
	}
	return default_blocks;
}

namespace
{
bool is_red_colour(std::string value)
{
	const auto not_space = [](unsigned char c) { return !std::isspace(c); };
	value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
	value.erase(
			std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
	std::transform(value.begin(), value.end(), value.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });

	// Match Rust's is_red_colour: a value beginning with '#' is parsed only as
	// hex, never as a named colour, even if malformed text contains "red".
	if (!value.empty() && value.front() == '#') {
		const std::string hex = value.substr(1);
		if ((hex.size() != 3 && hex.size() != 6) ||
				!std::all_of(hex.begin(), hex.end(),
						[](unsigned char c) { return std::isxdigit(c); }))
			return false;
		const int step = hex.size() == 3 ? 1 : 2;
		const auto channel = [&](std::size_t i) {
			const int n = std::stoi(hex.substr(i * step, step), nullptr, 16);
			return step == 1 ? n * 17 : n;
		};
		const int r = channel(0), g = channel(1), b = channel(2);
		return r >= 0x80 && 2 * r > 3 * g && 2 * r > 3 * b;
	}
	return value.find("red") != std::string::npos || value == "maroon" ||
		   value == "crimson" || value == "brick";
}
} // namespace

std::span<const Block> cycleway_palette(const tags_t &tags)
{
	if (tags.get("cycleway") == "crossing" || tags.contains("crossing"))
		return {};
	const auto surface_colour = tags.find("surface:colour");
	const auto colour = tags.find("colour");
	const auto selected_colour = surface_colour != tags.end() ? surface_colour : colour;
	if (selected_colour != tags.end() && !is_red_colour(selected_colour->second))
		return {};
	const auto surface = tags.get("surface");
	if (!surface.empty() && surface != "asphalt" && surface != "paved" &&
			surface != "concrete" && surface != "concrete:plates" &&
			surface != "concrete:lanes" && surface != "cement" && surface != "chipseal" &&
			surface != "bitmac" && surface != "paving_stones" && surface != "sett" &&
			surface != "bricks" && surface != "brick")
		return {};
	static const std::array<Block, 3> palette{block_definitions::RED_TERRACOTTA,
			block_definitions::RED_TERRACOTTA, block_definitions::RED_CONCRETE};
	return palette;
}

std::span<const Block> cycleway_palette(const ProcessedWay &way)
{
	return cycleway_palette(way.tags);
}

Block semirandom_surface(int x, int z, std::span<const Block> block_types)
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
