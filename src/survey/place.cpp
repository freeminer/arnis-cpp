#include "place.h"

#include "../block_definitions.h"
#include "../block_palette.h"
#include "../element_processing/bush.h"
#include "../ground_generation.h"
#include "../land_cover/land_cover.h"
#include "../world_editor/common.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace arnis::survey
{
namespace
{
using Palette = std::vector<std::pair<RGBTuple, Block>>;

const Palette &paving_palette()
{
	static const Palette palette{{{159, 159, 159}, SMOOTH_STONE},
			{{132, 135, 134}, POLISHED_ANDESITE}, {{122, 121, 122}, STONE_BRICKS},
			{{55, 58, 62}, GRAY_CONCRETE}, {{154, 107, 89}, POLISHED_GRANITE},
			{{150, 97, 83}, BRICK}, {{137, 104, 79}, MUD_BRICKS}};
	return palette;
}

Block paving_block(Block nearest, int x, int z)
{
	static const std::array<std::pair<std::uint64_t, Block>, 6> pale{
			{{50, STONE_BRICKS}, {22, SMOOTH_STONE}, {12, POLISHED_ANDESITE},
					{8, CRACKED_STONE_BRICKS}, {4, STONE}, {4, COBBLESTONE}}};
	static const std::array<std::pair<std::uint64_t, Block>, 6> mid{
			{{55, STONE_BRICKS}, {15, POLISHED_ANDESITE}, {13, CRACKED_STONE_BRICKS},
					{5, ANDESITE}, {5, STONE}, {7, COBBLESTONE}}};
	static const std::array<std::pair<std::uint64_t, Block>, 4> town{{{72, STONE_BRICKS},
			{15, CRACKED_STONE_BRICKS}, {5, STONE}, {8, COBBLESTONE}}};
	const auto choose = [&](const auto &mix) {
		auto value = land_cover::coord_hash(x, z) % 100;
		for (const auto &[weight, block] : mix) {
			if (value < weight)
				return block;
			value -= weight;
		}
		return mix.front().second;
	};
	if (nearest == SMOOTH_STONE)
		return choose(pale);
	if (nearest == POLISHED_ANDESITE)
		return choose(mid);
	if (nearest == STONE_BRICKS)
		return choose(town);
	return nearest;
}

int div_euclid(int value, int divisor)
{
	const int quotient = value / divisor;
	return value % divisor < 0 ? quotient - 1 : quotient;
}

std::pair<Block, Block> open_ground(int x, int z)
{
	const double noise = ground_generation::ground_noise(x, z, 6, 0x0BA2E000);
	if (noise < .55)
		return {COARSE_DIRT, DIRT};
	if (noise < .85)
		return {DIRT, DIRT};
	return {GRAVEL, STONE};
}

int blocks_tall(float height, int min, int max)
{
	return std::clamp(static_cast<int>(std::lround(height)), min, max);
}

Block hedge_leaf(int x, int z)
{
	const auto h = land_cover::coord_hash(div_euclid(x, 12), div_euclid(z, 12)) % 100;
	if (h < 45)
		return SPRUCE_LEAVES;
	if (h < 85)
		return OAK_LEAVES;
	return DARK_OAK_LEAVES;
}

Block bed_plant(int x, int z)
{
	static const std::array<Block, 10> drifts{RED_FLOWER, YELLOW_FLOWER, WHITE_FLOWER,
			BLUE_FLOWER, PINK_TULIP, RED_TULIP, OXEYE_DAISY, CORNFLOWER, ALLIUM, FERN};
	if (land_cover::coord_hash(x, z) % 100 < 20)
		return GRASS;
	const auto drift = land_cover::coord_hash(div_euclid(x, 3) ^ 0x5B, div_euclid(z, 3));
	return drifts[drift % drifts.size()];
}
} // namespace

bool claims_cover(std::uint8_t cover)
{
	return cover == land_cover::LC_BUILT_UP || cover == land_cover::LC_GRASSLAND ||
		   cover == land_cover::LC_SHRUBLAND || cover == land_cover::LC_TREE_COVER;
}

std::optional<std::pair<Block, Block>> ground_blocks(
		Cell cell, int x, int z, std::uint8_t cover)
{
	switch (cell.kind) {
	case Kind::Paved:
	case Kind::Shed: {
		const auto squeeze = [](std::uint8_t value) {
			return static_cast<std::uint8_t>(std::lround(40.0 + .7 * value));
		};
		const auto color = RGBTuple{
				squeeze(cell.color[0]), squeeze(cell.color[1]), squeeze(cell.color[2])};
		return std::pair{
				paving_block(block_palette::closest_of(paving_palette(), color), x, z),
				STONE};
	}
	case Kind::Pool:
		return std::nullopt;
	case Kind::Bare:
		return std::make_optional(cover == land_cover::LC_BUILT_UP
										  ? std::pair{STONE_BRICKS, STONE}
										  : open_ground(x, z));
	case Kind::Building:
		return std::pair{POLISHED_ANDESITE, STONE};
	case Kind::Tree:
	case Kind::Bush:
	case Kind::Hedge:
	case Kind::Bed:
	case Kind::Fence:
		if (cover == land_cover::LC_TREE_COVER)
			return std::nullopt;
		return std::pair{GRASS_BLOCK, DIRT};
	default:
		return std::pair{GRASS_BLOCK, DIRT};
	}
}

bool lawn_tuft(int x, int z)
{
	return land_cover::coord_hash(x ^ 0x6A, z ^ 0x2D) % 100 < 6;
}

void place_objects(
		world_editor::WorldEditor &editor, Cell cell, int x, int ground_y, int z)
{
	const auto column = [&](int tall, const auto &block_at_y) {
		for (int dy = 1; dy <= tall; ++dy) {
			if (editor.block_exists_absolute(x, ground_y + dy, z))
				break;
			editor.set_block_absolute(block_at_y(ground_y + dy), x, ground_y + dy, z,
					std::nullopt, std::nullopt);
		}
	};
	switch (cell.kind) {
	case Kind::Hedge: {
		const auto leaf = hedge_leaf(x, z);
		column(blocks_tall(cell.height, 2, 4), [&](int) { return leaf; });
		break;
	}
	case Kind::Bush: {
		const auto species = bush::shrubbery_species(editor, x, z);
		column(blocks_tall(cell.height, 1, 3),
				[&](int y) { return bush::shrubbery_leaf(species, x, y, z); });
		break;
	}
	case Kind::Bed: {
		const auto plant = bed_plant(x, z);
		column(1, [&](int) { return plant; });
		break;
	}
	case Kind::Fence:
		column(blocks_tall(cell.height, 1, 2), [](int) { return SPRUCE_FENCE; });
		break;
	default:
		break;
	}
}

} // namespace arnis::survey
