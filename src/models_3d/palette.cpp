#include "palette.h"
#include "../block_definitions.h"
#include <algorithm>
#include <array>
#include <tuple>

namespace arnis::models_3d
{
namespace
{
std::uint8_t usage_for(Block block)
{
	const std::array model_only = {WHITE_WOOL, SNOW_BLOCK, BLACK_WOOL, DIRT, COARSE_DIRT,
			RED_WOOL, ORANGE_WOOL, YELLOW_WOOL, GOLD_BLOCK, GREEN_WOOL, BLUE_WOOL};
	const std::array model_wall = {DIORITE, POLISHED_DIORITE, CHISELED_STONE_BRICKS,
			CRACKED_STONE_BRICKS, COBBLESTONE, COBBLED_DEEPSLATE, NETHERITE_BLOCK,
			OAK_LOG, SPRUCE_LOG, DARK_OAK_LOG, GRANITE, POLISHED_GRANITE,
			END_STONE_BRICKS, RED_CONCRETE, ORANGE_CONCRETE, YELLOW_CONCRETE,
			LIME_CONCRETE, MOSSY_COBBLESTONE, BLUE_CONCRETE, PURPLE_CONCRETE,
			MAGENTA_CONCRETE};
	if (std::find(model_only.begin(), model_only.end(), block) != model_only.end())
		return USE_MODEL;
	if (block == HAY_BALE || block == MOSS_BLOCK)
		return USE_MODEL | USE_ROOF;
	if (std::find(model_wall.begin(), model_wall.end(), block) != model_wall.end())
		return USE_MODEL | USE_WALL;
	return USE_MODEL | USE_WALL | USE_ROOF;
}
struct E
{
	RGBTuple color;
	Block block;
	std::uint8_t usage;
	E(RGBTuple color_, Block block_) :
			color(color_), block(block_), usage(usage_for(block_))
	{
	}
	E(RGBTuple color_, Block block_, std::uint8_t usage_) :
			color(color_), block(block_), usage(usage_)
	{
	}
};
// Kept in the same order as palette.rs.  Order is significant for stable
// ties, so independently generated tiles choose the same material.
const auto palette = std::to_array<E>({{{207, 213, 214}, WHITE_CONCRETE, 7},
		{{210, 178, 161}, WHITE_TERRACOTTA}, {{234, 236, 237}, WHITE_WOOL},
		{{236, 230, 223}, QUARTZ_BLOCK}, {{235, 229, 222}, QUARTZ_BRICKS},
		{{249, 254, 254}, SNOW_BLOCK}, {{220, 220, 220}, IRON_BLOCK},
		{{125, 125, 115}, LIGHT_GRAY_CONCRETE}, {{135, 107, 98}, LIGHT_GRAY_TERRACOTTA},
		{{189, 188, 189}, DIORITE}, {{193, 193, 195}, POLISHED_DIORITE},
		{{132, 135, 134}, POLISHED_ANDESITE}, {{159, 159, 159}, SMOOTH_STONE},
		{{122, 122, 122}, STONE_BRICKS}, {{120, 119, 120}, CHISELED_STONE_BRICKS},
		{{118, 118, 118}, CRACKED_STONE_BRICKS}, {{126, 126, 126}, STONE},
		{{128, 127, 128}, COBBLESTONE}, {{136, 136, 137}, ANDESITE},
		{{55, 58, 62}, GRAY_CONCRETE}, {{58, 42, 36}, GRAY_TERRACOTTA},
		{{80, 80, 83}, DEEPSLATE}, {{71, 71, 71}, DEEPSLATE_BRICKS},
		{{72, 73, 73}, POLISHED_DEEPSLATE}, {{77, 77, 81}, COBBLED_DEEPSLATE},
		{{42, 36, 41}, BLACKSTONE}, {{53, 49, 57}, POLISHED_BLACKSTONE},
		{{48, 43, 50}, POLISHED_BLACKSTONE_BRICKS}, {{8, 10, 15}, BLACK_CONCRETE},
		{{37, 23, 16}, BLACK_TERRACOTTA}, {{21, 21, 26}, BLACK_WOOL},
		{{67, 61, 64}, NETHERITE_BLOCK}, {{96, 60, 32}, BROWN_CONCRETE},
		{{77, 51, 36}, BROWN_TERRACOTTA}, {{137, 104, 79}, MUD_BRICKS},
		{{134, 96, 67}, DIRT}, {{119, 86, 59}, COARSE_DIRT}, {{162, 131, 79}, OAK_PLANKS},
		{{109, 85, 51}, OAK_LOG}, {{115, 85, 49}, SPRUCE_PLANKS},
		{{59, 38, 17}, SPRUCE_LOG}, {{67, 43, 20}, DARK_OAK_PLANKS},
		{{60, 47, 26}, DARK_OAK_LOG}, {{149, 103, 86}, GRANITE},
		{{154, 107, 89}, POLISHED_GRANITE}, {{216, 203, 156}, SANDSTONE},
		{{224, 214, 170}, SMOOTH_SANDSTONE}, {{218, 224, 162}, END_STONE_BRICKS},
		{{166, 136, 38}, HAY_BALE}, {{151, 98, 83}, BRICK},
		{{143, 61, 47}, RED_TERRACOTTA}, {{161, 39, 35}, RED_WOOL},
		{{142, 33, 33}, RED_CONCRETE}, {{70, 7, 9}, RED_NETHER_BRICK},
		{{44, 22, 26}, NETHER_BRICK}, {{152, 94, 68}, TERRACOTTA},
		{{162, 84, 38}, ORANGE_TERRACOTTA}, {{241, 118, 20}, ORANGE_WOOL},
		{{224, 97, 1}, ORANGE_CONCRETE}, {{192, 108, 80}, WAXED_COPPER_BLOCK},
		{{161, 126, 104}, WAXED_EXPOSED_COPPER}, {{241, 175, 21}, YELLOW_CONCRETE},
		{{186, 133, 35}, YELLOW_TERRACOTTA}, {{249, 198, 40}, YELLOW_WOOL},
		{{246, 208, 62}, GOLD_BLOCK}, {{73, 91, 36}, GREEN_CONCRETE},
		{{85, 110, 28}, GREEN_WOOL}, {{94, 169, 24}, LIME_CONCRETE},
		{{89, 110, 45}, MOSS_BLOCK}, {{110, 118, 95}, MOSSY_COBBLESTONE},
		{{82, 163, 133}, WAXED_OXIDIZED_COPPER}, {{45, 47, 143}, BLUE_CONCRETE},
		{{74, 60, 91}, BLUE_TERRACOTTA}, {{53, 57, 157}, BLUE_WOOL},
		{{36, 137, 199}, LIGHT_BLUE_CONCRETE}, {{113, 109, 138}, LIGHT_BLUE_TERRACOTTA},
		{{100, 32, 156}, PURPLE_CONCRETE}, {{169, 48, 159}, MAGENTA_CONCRETE},
		{{21, 119, 136}, CYAN_CONCRETE}, {{87, 91, 91}, CYAN_TERRACOTTA}});

float tag_match_distance(const RGBTuple &a, const RGBTuple &b)
{
	const auto lhs = oklab_components(a), rhs = oklab_components(b);
	const float dl = .5f * (lhs[0] - rhs[0]);
	const float da = lhs[1] - rhs[1], db = lhs[2] - rhs[2];
	return dl * dl + da * da + db * db;
}
}
Block closest_block(RGBTuple color)
{
	return std::min_element(palette.begin(), palette.end(), [&](const E &a, const E &b) {
		return oklab_distance(color, a.color) < oklab_distance(color, b.color);
	})->block;
}
std::vector<Block> closest_blocks(RGBTuple color, std::size_t k)
{
	std::vector<E> values(palette.begin(), palette.end());
	std::stable_sort(values.begin(), values.end(), [&](const E &a, const E &b) {
		return oklab_distance(color, a.color) < oklab_distance(color, b.color);
	});
	std::vector<Block> out;
	out.reserve(std::min(std::max<std::size_t>(1, k), values.size()));
	for (std::size_t i = 0; i < std::min(std::max<std::size_t>(1, k), values.size()); ++i)
		out.push_back(values[i].block);
	return out;
}

std::vector<Block> closest_blocks_for_usage(
		RGBTuple color, std::size_t k, std::uint8_t usage)
{
	std::vector<std::pair<float, Block>> values;
	for (const auto &entry : palette)
		if ((entry.usage & usage) != 0)
			values.emplace_back(tag_match_distance(color, entry.color), entry.block);
	std::stable_sort(values.begin(), values.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	if (values.empty())
		return {STONE_BRICKS};
	const float cutoff = std::max(values.front().first, 1e-6f) * 1.5f;
	const std::size_t limit =
			std::min({std::max<std::size_t>(1, k), values.size(), std::size_t(3)});
	std::vector<Block> out;
	for (std::size_t i = 0; i < limit && values[i].first <= cutoff; ++i)
		out.push_back(values[i].second);
	return out;
}

std::vector<Block> all_blocks_for_usage(std::uint8_t usage)
{
	std::vector<Block> out;
	for (const auto &entry : palette)
		if ((entry.usage & usage) != 0)
			out.push_back(entry.block);
	return out;
}

Block block_for_usage(
		RGBTuple color, std::uint8_t usage, std::uint64_t deterministic_seed)
{
	auto candidates = closest_blocks_for_usage(color, 3, usage);
	return candidates[deterministic_seed % candidates.size()];
}

Block block_for_usage(RGBTuple color, std::uint8_t usage, ChaCha8Rng &rng)
{
	auto candidates = closest_blocks_for_usage(color, 3, usage);
	return candidates[rng.uniform(static_cast<std::uint32_t>(candidates.size()))];
}
}
