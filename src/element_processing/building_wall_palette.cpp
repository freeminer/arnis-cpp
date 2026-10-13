#include "buildings.h"

#include "../biome.h"
#include "../block_definitions.h"
#include "../deterministic_rng.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace arnis::buildings
{
namespace
{
const std::array<Block, 29> RESIDENTIAL_WALL_OPTIONS = {BRICK, STONE_BRICKS,
		WHITE_TERRACOTTA, BROWN_TERRACOTTA, SANDSTONE, SMOOTH_SANDSTONE, QUARTZ_BRICKS,
		MUD_BRICKS, POLISHED_GRANITE, END_STONE_BRICKS, BROWN_CONCRETE, DEEPSLATE_BRICKS,
		GRAY_CONCRETE, GRAY_TERRACOTTA, LIGHT_BLUE_TERRACOTTA, LIGHT_GRAY_CONCRETE,
		LIGHT_GRAY_TERRACOTTA, NETHER_BRICK, POLISHED_ANDESITE, POLISHED_BLACKSTONE,
		POLISHED_BLACKSTONE_BRICKS, POLISHED_DEEPSLATE, QUARTZ_BLOCK, WHITE_CONCRETE,
		ORANGE_TERRACOTTA, RED_TERRACOTTA, RED_NETHER_BRICKS, GRANITE, TERRACOTTA};
const std::array<Block, 8> COMMERCIAL_WALL_OPTIONS = {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE,
		GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE, QUARTZ_BLOCK, QUARTZ_BRICKS,
		STONE_BRICKS};
const std::array<Block, 7> INDUSTRIAL_WALL_OPTIONS = {GRAY_CONCRETE, LIGHT_GRAY_CONCRETE,
		STONE, SMOOTH_STONE, POLISHED_ANDESITE, DEEPSLATE_BRICKS, BLACKSTONE};
const std::array<Block, 9> RELIGIOUS_WALL_OPTIONS = {STONE_BRICKS, CHISELED_STONE_BRICKS,
		QUARTZ_BLOCK, WHITE_CONCRETE, SANDSTONE, SMOOTH_SANDSTONE, POLISHED_DIORITE,
		END_STONE_BRICKS, WAXED_OXIDIZED_COPPER};
const std::array<Block, 8> INSTITUTIONAL_WALL_OPTIONS = {WHITE_CONCRETE,
		LIGHT_GRAY_CONCRETE, QUARTZ_BRICKS, STONE_BRICKS, POLISHED_ANDESITE, SMOOTH_STONE,
		SANDSTONE, END_STONE_BRICKS};
const std::array<Block, 8> FARM_WALL_OPTIONS = {OAK_PLANKS, SPRUCE_PLANKS,
		DARK_OAK_PLANKS, COBBLESTONE, STONE, MUD_BRICKS, MOSSY_COBBLESTONE,
		BROWN_TERRACOTTA};
const std::array<Block, 16> HISTORIC_WALL_OPTIONS = {STONE_BRICKS, CRACKED_STONE_BRICKS,
		CHISELED_STONE_BRICKS, COBBLESTONE, SANDSTONE, SMOOTH_SANDSTONE,
		POLISHED_BLACKSTONE_BRICKS, DEEPSLATE_BRICKS, POLISHED_ANDESITE, ANDESITE,
		SMOOTH_STONE, BRICK, RED_NETHER_BRICKS, MOSSY_STONE_BRICKS, MOSSY_COBBLESTONE,
		COBBLED_DEEPSLATE};
const std::array<Block, 6> GARAGE_WALL_OPTIONS = {BRICK, STONE_BRICKS, POLISHED_ANDESITE,
		COBBLESTONE, SMOOTH_STONE, LIGHT_GRAY_CONCRETE};
const std::array<Block, 1> SHED_WALL_OPTIONS = {OAK_LOG};
const std::array<Block, 4> GREENHOUSE_WALL_OPTIONS = {
		GLASS, CYAN_STAINED_GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS};
const std::array<Block, 3> NORDIC_WOOD_ADDITIONS = {
		SPRUCE_PLANKS, OAK_PLANKS, DARK_OAK_PLANKS};
const std::array<Block, 8> TOWER_WALL_OPTIONS = {STONE_BRICKS, COBBLESTONE,
		CRACKED_STONE_BRICKS, BRICK, POLISHED_ANDESITE, ANDESITE, DEEPSLATE_BRICKS,
		SMOOTH_STONE};
const std::array<Block, 7> MODERN_SKYSCRAPER_WALL_OPTIONS = {GRAY_CONCRETE,
		LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE,
		QUARTZ_BLOCK, SMOOTH_SANDSTONE};
const std::array<Block, 7> CONTEMPORARY_WALL_OPTIONS = {LIGHT_GRAY_CONCRETE,
		WHITE_CONCRETE, GRAY_CONCRETE, QUARTZ_BLOCK, SMOOTH_STONE, SMOOTH_SANDSTONE,
		SANDSTONE};
const std::array<Block, 3> MASONRY_WARM_OPTIONS = {
		SMOOTH_SANDSTONE, SANDSTONE, END_STONE_BRICKS};
const std::array<Block, 4> MASONRY_GREY_OPTIONS = {
		STONE_BRICKS, SMOOTH_STONE, POLISHED_ANDESITE, POLISHED_DIORITE};
const std::array<Block, 4> GLASSY_WALL_OPTIONS = {GRAY_STAINED_GLASS, CYAN_STAINED_GLASS,
		BLUE_STAINED_GLASS, LIGHT_BLUE_STAINED_GLASS};

bool is_rare_wall_block(Block block)
{
	return block == NETHER_BRICK || block == RED_NETHER_BRICKS || block == RED_TERRACOTTA;
}

template <typename Palette>
Block pick_with_rare_filter(const Palette &palette, ChaCha8Rng &rng)
{
	Block last = palette[rng.uniform(static_cast<std::uint32_t>(palette.size()))];
	for (int retry = 0; retry < 8; ++retry) {
		if (!is_rare_wall_block(last) || rng.random_bool(0.20))
			return last;
		last = palette[rng.uniform(static_cast<std::uint32_t>(palette.size()))];
	}
	return last;
}

bool contains_block(const std::vector<Block> &palette, Block block)
{
	return std::find(palette.begin(), palette.end(), block) != palette.end();
}

std::vector<Block> era_allow_list(ArchEra era)
{
	switch (era) {
	case ArchEra::TraditionalPreWar:
		return {BRICK, STONE_BRICKS, WHITE_TERRACOTTA, BROWN_TERRACOTTA, SANDSTONE,
				SMOOTH_SANDSTONE, MUD_BRICKS, GRANITE, POLISHED_GRANITE, TERRACOTTA,
				END_STONE_BRICKS, QUARTZ_BRICKS, ORANGE_TERRACOTTA, RED_TERRACOTTA,
				RED_NETHER_BRICKS, NETHER_BRICK, COBBLESTONE, OAK_PLANKS, SPRUCE_PLANKS};
	case ArchEra::PostWarPanel:
		return {GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, BROWN_CONCRETE,
				GRAY_TERRACOTTA, LIGHT_GRAY_TERRACOTTA, POLISHED_ANDESITE, SMOOTH_STONE,
				BRICK, WHITE_TERRACOTTA};
	case ArchEra::Contemporary:
		return {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, QUARTZ_BLOCK,
				QUARTZ_BRICKS, POLISHED_ANDESITE, SMOOTH_STONE, POLISHED_DEEPSLATE,
				DEEPSLATE_BRICKS, POLISHED_BLACKSTONE, LIGHT_GRAY_TERRACOTTA};
	case ArchEra::HistoricOrnate:
		return {SANDSTONE, SMOOTH_SANDSTONE, STONE_BRICKS, CHISELED_STONE_BRICKS,
				END_STONE_BRICKS, QUARTZ_BRICKS, WHITE_TERRACOTTA, BRICK,
				POLISHED_DIORITE, GRANITE, POLISHED_GRANITE};
	case ArchEra::Unknown:
		return {};
	}
	return {};
}

bool era_filters_category(BuildingCategory category)
{
	return category == BuildingCategory::House ||
		   category == BuildingCategory::Residential ||
		   category == BuildingCategory::Commercial ||
		   category == BuildingCategory::Office || category == BuildingCategory::Hotel ||
		   category == BuildingCategory::Default;
}

bool climate_applies(biome::Climate climate, BuildingCategory category)
{
	if (category == BuildingCategory::House || category == BuildingCategory::Residential)
		return true;
	return (category == BuildingCategory::Commercial ||
				   category == BuildingCategory::Office ||
				   category == BuildingCategory::Hotel) &&
		   (climate == biome::Climate::HotDesert ||
				   climate == biome::Climate::HotSteppe ||
				   climate == biome::Climate::TropicalSavanna);
}

std::uint8_t climate_wall_weight(
		biome::Climate climate, BuildingCategory category, Block block)
{
	const auto residential_scale = [category](std::uint8_t weight) {
		return category == BuildingCategory::Residential
					   ? std::max<std::uint8_t>(1, weight / 2)
					   : weight;
	};
	switch (climate) {
	case biome::Climate::Temperate:
		return 1;
	case biome::Climate::HotDesert:
	case biome::Climate::HotSteppe:
		if (block == SANDSTONE || block == SMOOTH_SANDSTONE || block == MUD_BRICKS ||
				block == WHITE_TERRACOTTA)
			return 4;
		if (block == WHITE_CONCRETE || block == END_STONE_BRICKS || block == TERRACOTTA)
			return 3;
		if (block == LIGHT_GRAY_CONCRETE || block == QUARTZ_BRICKS ||
				block == QUARTZ_BLOCK || block == ORANGE_TERRACOTTA ||
				block == BROWN_TERRACOTTA)
			return 2;
		if (block == DEEPSLATE_BRICKS || block == POLISHED_DEEPSLATE ||
				block == POLISHED_BLACKSTONE || block == POLISHED_BLACKSTONE_BRICKS ||
				block == NETHER_BRICK || block == RED_NETHER_BRICKS ||
				block == RED_TERRACOTTA || block == GRAY_TERRACOTTA ||
				block == BROWN_CONCRETE || block == SPRUCE_PLANKS ||
				block == OAK_PLANKS || block == DARK_OAK_PLANKS)
			return 0;
		return 1;
	case biome::Climate::TropicalSavanna:
		if (block == WHITE_CONCRETE)
			return 4;
		if (block == WHITE_TERRACOTTA || block == LIGHT_GRAY_CONCRETE || block == BRICK)
			return 3;
		if (block == GRAY_CONCRETE || block == TERRACOTTA || block == ORANGE_TERRACOTTA ||
				block == LIGHT_BLUE_TERRACOTTA || block == QUARTZ_BRICKS ||
				block == SMOOTH_SANDSTONE || block == MUD_BRICKS)
			return 2;
		if (block == DEEPSLATE_BRICKS || block == POLISHED_DEEPSLATE ||
				block == POLISHED_BLACKSTONE || block == POLISHED_BLACKSTONE_BRICKS ||
				block == NETHER_BRICK || block == RED_NETHER_BRICKS ||
				block == SPRUCE_PLANKS || block == OAK_PLANKS || block == DARK_OAK_PLANKS)
			return 0;
		return 1;
	case biome::Climate::Boreal:
		if (block == SPRUCE_PLANKS)
			return residential_scale(5);
		if (block == OAK_PLANKS)
			return residential_scale(3);
		if (block == DARK_OAK_PLANKS)
			return residential_scale(2);
		if (block == RED_TERRACOTTA)
			return 3;
		if (block == WHITE_CONCRETE || block == WHITE_TERRACOTTA || block == BRICK)
			return 2;
		if (block == SANDSTONE || block == SMOOTH_SANDSTONE || block == MUD_BRICKS)
			return 0;
		return 1;
	case biome::Climate::Tundra:
	case biome::Climate::IceCap:
		if (block == SPRUCE_PLANKS)
			return residential_scale(3);
		if (block == OAK_PLANKS)
			return residential_scale(2);
		if (block == LIGHT_BLUE_TERRACOTTA || block == RED_TERRACOTTA ||
				block == WHITE_CONCRETE || block == GRAY_CONCRETE ||
				block == LIGHT_GRAY_CONCRETE)
			return 2;
		if (block == SANDSTONE || block == SMOOTH_SANDSTONE || block == MUD_BRICKS ||
				block == DARK_OAK_PLANKS)
			return 0;
		return 1;
	case biome::Climate::ColdSteppe:
	case biome::Climate::ColdDesert:
	case biome::Climate::DryContinental:
		if (block == BRICK)
			return 3;
		if (block == WHITE_TERRACOTTA || block == WHITE_CONCRETE ||
				block == LIGHT_GRAY_CONCRETE || block == GRAY_CONCRETE ||
				block == STONE_BRICKS || block == MUD_BRICKS)
			return 2;
		if (block == SPRUCE_PLANKS || block == OAK_PLANKS || block == DARK_OAK_PLANKS)
			return 0;
		return 1;
	}
	return 1;
}

template <typename Palette>
Block pick_weighted_wall(
		const Palette &pool, std::optional<Block> rare_exempt, ChaCha8Rng &rng)
{
	std::uint32_t total = 0;
	for (const auto &[_, weight] : pool)
		total += weight;
	for (int retry = 0; retry < 8; ++retry) {
		std::uint32_t roll = rng.uniform(total);
		Block picked = pool.front().first;
		for (const auto &[block, weight] : pool) {
			if (roll < weight) {
				picked = block;
				break;
			}
			roll -= weight;
		}
		if (!is_rare_wall_block(picked) || (rare_exempt && picked == *rare_exempt) ||
				rng.random_bool(0.20))
			return picked;
	}
	return pool.front().first;
}

template <typename Palette>
Block pick_palette_block(const Palette &palette, ChaCha8Rng &rng)
{
	return palette[rng.uniform(static_cast<std::uint32_t>(palette.size()))];
}
} // namespace

Block rust_category_wall_block(
		BuildingCategory category, ArchEra era, biome::Climate climate, ChaCha8Rng &rng)
{
	if (climate != biome::Climate::Temperate && climate_applies(climate, category)) {
		const bool residential = category == BuildingCategory::House ||
								 category == BuildingCategory::Residential;
		const bool nordic = residential && (climate == biome::Climate::Boreal ||
												   climate == biome::Climate::Tundra ||
												   climate == biome::Climate::IceCap);
		std::vector<Block> source;
		if (residential)
			source.assign(
					RESIDENTIAL_WALL_OPTIONS.begin(), RESIDENTIAL_WALL_OPTIONS.end());
		else
			source.assign(COMMERCIAL_WALL_OPTIONS.begin(), COMMERCIAL_WALL_OPTIONS.end());
		if (nordic)
			source.insert(source.end(), NORDIC_WOOD_ADDITIONS.begin(),
					NORDIC_WOOD_ADDITIONS.end());
		const auto allowed = era_allow_list(era);
		std::vector<std::pair<Block, std::uint8_t>> pool;
		for (const Block block : source) {
			if (!allowed.empty() && !contains_block(allowed, block))
				continue;
			const auto weight = climate_wall_weight(climate, category, block);
			if (weight > 0)
				pool.emplace_back(block, weight);
		}
		if (pool.size() >= 2) {
			const bool boreal_exception = climate == biome::Climate::Boreal ||
										  climate == biome::Climate::Tundra ||
										  climate == biome::Climate::IceCap;
			return pick_weighted_wall(pool,
					boreal_exception ? std::optional<Block>(RED_TERRACOTTA)
									 : std::nullopt,
					rng);
		}
	}

	if (era_filters_category(category)) {
		const auto allowed = era_allow_list(era);
		if (!allowed.empty()) {
			std::vector<Block> category_palette;
			if (category == BuildingCategory::House ||
					category == BuildingCategory::Residential)
				category_palette.assign(
						RESIDENTIAL_WALL_OPTIONS.begin(), RESIDENTIAL_WALL_OPTIONS.end());
			else
				category_palette.assign(
						COMMERCIAL_WALL_OPTIONS.begin(), COMMERCIAL_WALL_OPTIONS.end());
			std::vector<Block> filtered;
			for (const Block block : category_palette)
				if (contains_block(allowed, block))
					filtered.push_back(block);
			return filtered.size() >= 4 ? pick_with_rare_filter(filtered, rng)
										: pick_with_rare_filter(allowed, rng);
		}
	}

	switch (category) {
	case BuildingCategory::House:
	case BuildingCategory::Residential:
		return pick_with_rare_filter(RESIDENTIAL_WALL_OPTIONS, rng);
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::Hotel:
		return pick_with_rare_filter(COMMERCIAL_WALL_OPTIONS, rng);
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		return pick_with_rare_filter(INDUSTRIAL_WALL_OPTIONS, rng);
	case BuildingCategory::Religious:
		return pick_with_rare_filter(RELIGIOUS_WALL_OPTIONS, rng);
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		return pick_with_rare_filter(INSTITUTIONAL_WALL_OPTIONS, rng);
	case BuildingCategory::Farm:
		return pick_with_rare_filter(FARM_WALL_OPTIONS, rng);
	case BuildingCategory::Historic:
		return pick_with_rare_filter(HISTORIC_WALL_OPTIONS, rng);
	case BuildingCategory::Garage:
		return pick_with_rare_filter(GARAGE_WALL_OPTIONS, rng);
	case BuildingCategory::Shed:
		// The Rust BuildingStylePreset fixes this block, so its category
		// fallback (and its one-element random draw) is not reached in resolve.
		return SHED_WALL_OPTIONS.front();
	case BuildingCategory::Greenhouse:
		return pick_palette_block(GREENHOUSE_WALL_OPTIONS, rng);
	case BuildingCategory::Tower:
		return pick_palette_block(TOWER_WALL_OPTIONS, rng);
	case BuildingCategory::TallBuilding:
		return pick_palette_block(COMMERCIAL_WALL_OPTIONS, rng);
	case BuildingCategory::ModernSkyscraper:
		return pick_palette_block(MODERN_SKYSCRAPER_WALL_OPTIONS, rng);
	case BuildingCategory::ContemporarySkyscraper:
	case BuildingCategory::GridSkyscraper:
		return pick_palette_block(CONTEMPORARY_WALL_OPTIONS, rng);
	case BuildingCategory::MasonrySkyscraper:
		return rng.random_bool(0.5) ? pick_palette_block(MASONRY_WARM_OPTIONS, rng)
									: pick_palette_block(MASONRY_GREY_OPTIONS, rng);
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
		return pick_palette_block(GLASSY_WALL_OPTIONS, rng);
	case BuildingCategory::Default:
		return get_fallback_building_block(rng);
	}
	return get_fallback_building_block(rng);
}

} // namespace arnis::buildings
