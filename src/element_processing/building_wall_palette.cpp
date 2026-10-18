#include "buildings.h"

#include "../biome.h"
#include "../block_definitions.h"
#include "../deterministic_rng.h"
#include "building_region.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace arnis::buildings
{
namespace
{
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
	// Rust palettes contain stable logical block IDs. The host resolves C++ block
	// IDs at runtime, so sample them here rather than copying defaults at startup.
	const std::array<Block, 29> RESIDENTIAL_WALL_OPTIONS = {BRICK, STONE_BRICKS,
			WHITE_TERRACOTTA, BROWN_TERRACOTTA, SANDSTONE, SMOOTH_SANDSTONE,
			QUARTZ_BRICKS, MUD_BRICKS, POLISHED_GRANITE, END_STONE_BRICKS, BROWN_CONCRETE,
			DEEPSLATE_BRICKS, GRAY_CONCRETE, GRAY_TERRACOTTA, LIGHT_BLUE_TERRACOTTA,
			LIGHT_GRAY_CONCRETE, LIGHT_GRAY_TERRACOTTA, NETHER_BRICK, POLISHED_ANDESITE,
			POLISHED_BLACKSTONE, POLISHED_BLACKSTONE_BRICKS, POLISHED_DEEPSLATE,
			QUARTZ_BLOCK, WHITE_CONCRETE, ORANGE_TERRACOTTA, RED_TERRACOTTA,
			RED_NETHER_BRICKS, GRANITE, TERRACOTTA};
	const std::array<Block, 8> COMMERCIAL_WALL_OPTIONS = {WHITE_CONCRETE,
			LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE,
			QUARTZ_BLOCK, QUARTZ_BRICKS, STONE_BRICKS};
	const std::array<Block, 7> INDUSTRIAL_WALL_OPTIONS = {GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE, STONE, SMOOTH_STONE, POLISHED_ANDESITE, DEEPSLATE_BRICKS,
			BLACKSTONE};
	const std::array<Block, 9> RELIGIOUS_WALL_OPTIONS = {STONE_BRICKS,
			CHISELED_STONE_BRICKS, QUARTZ_BLOCK, WHITE_CONCRETE, SANDSTONE,
			SMOOTH_SANDSTONE, POLISHED_DIORITE, END_STONE_BRICKS, WAXED_OXIDIZED_COPPER};
	const std::array<Block, 8> INSTITUTIONAL_WALL_OPTIONS = {WHITE_CONCRETE,
			LIGHT_GRAY_CONCRETE, QUARTZ_BRICKS, STONE_BRICKS, POLISHED_ANDESITE,
			SMOOTH_STONE, SANDSTONE, END_STONE_BRICKS};
	const std::array<Block, 8> FARM_WALL_OPTIONS = {OAK_PLANKS, SPRUCE_PLANKS,
			DARK_OAK_PLANKS, COBBLESTONE, STONE, MUD_BRICKS, MOSSY_COBBLESTONE,
			BROWN_TERRACOTTA};
	const std::array<Block, 16> HISTORIC_WALL_OPTIONS = {STONE_BRICKS,
			CRACKED_STONE_BRICKS, CHISELED_STONE_BRICKS, COBBLESTONE, SANDSTONE,
			SMOOTH_SANDSTONE, POLISHED_BLACKSTONE_BRICKS, DEEPSLATE_BRICKS,
			POLISHED_ANDESITE, ANDESITE, SMOOTH_STONE, BRICK, RED_NETHER_BRICKS,
			MOSSY_STONE_BRICKS, MOSSY_COBBLESTONE, COBBLED_DEEPSLATE};
	const std::array<Block, 6> GARAGE_WALL_OPTIONS = {BRICK, STONE_BRICKS,
			POLISHED_ANDESITE, COBBLESTONE, SMOOTH_STONE, LIGHT_GRAY_CONCRETE};
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
	const std::array<Block, 4> GLASSY_WALL_OPTIONS = {GRAY_STAINED_GLASS,
			CYAN_STAINED_GLASS, BLUE_STAINED_GLASS, LIGHT_BLUE_STAINED_GLASS};

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
			else if (category == BuildingCategory::Commercial ||
					 category == BuildingCategory::Office ||
					 category == BuildingCategory::Hotel)
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
		// Keep Rust's one-entry random-range draw so subsequent seeded choices
		// remain aligned when this selector is called without a fixed preset.
		return pick_palette_block(SHED_WALL_OPTIONS, rng);
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

// Regional weighted wall pools and district palette selection ported from
// buildings.rs. A small deterministic sub-palette keeps neighboring buildings
// visually coherent without making the entire district monochrome.
namespace
{
using WallEntry = std::pair<Block, std::uint8_t>;
// Host block IDs are assigned after static initialization. Keep references in
// the regional tables and copy their resolved values only when sampling them.
using WallSource = std::pair<const Block &, std::uint8_t>;
using WallPool = std::span<const WallSource>;
enum class WallTypology
{
	House,
	Traditional,
	Civic,
	Modern
};

static const WallSource NA_TRADITIONAL_WALLS[] = {{BRICK, 30}, {POLISHED_GRANITE, 6},
		{SMOOTH_SANDSTONE, 13}, {SANDSTONE, 3}, {BROWN_TERRACOTTA, 7}, {MUD_BRICKS, 4},
		{WHITE_TERRACOTTA, 5}, {MUSHROOM_STEM, 6}, {CALCITE, 2}, {CYAN_TERRACOTTA, 6},
		{LIGHT_GRAY_CONCRETE_POWDER, 4}, {WHITE_CONCRETE, 4}, {LIGHT_GRAY_CONCRETE, 3},
		{SMOOTH_STONE, 2}};
static const WallSource NA_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 18}, {MUSHROOM_STEM, 12},
		{LIGHT_GRAY_CONCRETE_POWDER, 12}, {CYAN_TERRACOTTA, 12}, {WHITE_TERRACOTTA, 5},
		{CALCITE, 5}, {BRICK, 9}, {POLISHED_GRANITE, 3}, {SMOOTH_STONE, 6},
		{GRAY_CONCRETE, 5}, {TUFF, 3}, {LIGHT_GRAY_CONCRETE, 5}, {SANDSTONE, 3}};
static const WallSource NA_MODERN_WALLS[] = {{LIGHT_GRAY_CONCRETE_POWDER, 14},
		{LIGHT_GRAY_CONCRETE, 12}, {WHITE_CONCRETE, 12}, {GRAY_CONCRETE, 10},
		{CYAN_TERRACOTTA, 10}, {BRICK, 12}, {SMOOTH_STONE, 8}, {POLISHED_ANDESITE, 6},
		{MUSHROOM_STEM, 4}, {SMOOTH_SANDSTONE, 4}, {BLACK_CONCRETE, 3},
		{POLISHED_DEEPSLATE, 2}};
static const WallSource NA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 16}, {BRICK, 12},
		{SMOOTH_SANDSTONE, 9}, {WHITE_TERRACOTTA, 9}, {MUSHROOM_STEM, 8}, {CALCITE, 5},
		{LIGHT_GRAY_CONCRETE, 6}, {CYAN_TERRACOTTA, 5}, {POLISHED_GRANITE, 3},
		{BROWN_TERRACOTTA, 3}, {OAK_PLANKS, 4}, {SPRUCE_PLANKS, 4},
		{YELLOW_TERRACOTTA, 2}};
static const WallSource SUNBELT_HOUSE_WALLS[] = {{WHITE_CONCRETE, 18},
		{SMOOTH_SANDSTONE, 16}, {WHITE_TERRACOTTA, 16}, {MUSHROOM_STEM, 10}, {CALCITE, 6},
		{LIGHT_GRAY_CONCRETE, 5}, {CYAN_TERRACOTTA, 4}, {BRICK, 4},
		{YELLOW_TERRACOTTA, 3}, {TERRACOTTA, 3}, {OAK_PLANKS, 3}, {SPRUCE_PLANKS, 3}};
static const WallSource SUNBELT_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 16},
		{SMOOTH_SANDSTONE, 14}, {WHITE_TERRACOTTA, 14}, {MUSHROOM_STEM, 8}, {CALCITE, 4},
		{BRICK, 10}, {POLISHED_GRANITE, 3}, {CYAN_TERRACOTTA, 4},
		{LIGHT_GRAY_CONCRETE, 5}, {YELLOW_TERRACOTTA, 3}, {TERRACOTTA, 3},
		{SANDSTONE, 3}};
static const WallSource CENTRAL_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 22}, {CALCITE, 8},
		{WHITE_TERRACOTTA, 14}, {CHERRY_PLANKS, 4}, {SMOOTH_SANDSTONE, 12},
		{MUSHROOM_STEM, 8}, {BIRCH_PLANKS, 8}, {YELLOW_TERRACOTTA, 4},
		{RED_TERRACOTTA, 4}, {LIGHT_GRAY_CONCRETE_POWDER, 4}, {SMOOTH_STONE, 2},
		{BRICK, 4}};
static const WallSource BRICK_NORTH_TRADITIONAL_WALLS[] = {{BRICK, 32},
		{POLISHED_GRANITE, 8}, {MUD_BRICKS, 5}, {BROWN_TERRACOTTA, 4},
		{RED_TERRACOTTA, 3}, {TERRACOTTA, 3}, {WHITE_CONCRETE, 12}, {WHITE_TERRACOTTA, 5},
		{SMOOTH_SANDSTONE, 4}, {MUSHROOM_STEM, 4}, {LIGHT_GRAY_CONCRETE, 3},
		{CALCITE, 2}};
static const WallSource BRITISH_TRADITIONAL_WALLS[] = {{MUD_BRICKS, 22}, {BRICK, 20},
		{POLISHED_GRANITE, 4}, {WHITE_CONCRETE, 12}, {MUSHROOM_STEM, 6},
		{SMOOTH_SANDSTONE, 5}, {CALCITE, 3}, {WHITE_TERRACOTTA, 3}, {BROWN_TERRACOTTA, 3},
		{LIGHT_GRAY_CONCRETE, 3}};
static const WallSource FRANCE_TRADITIONAL_WALLS[] = {{MUSHROOM_STEM, 18},
		{SMOOTH_SANDSTONE, 12}, {CALCITE, 8}, {WHITE_CONCRETE, 12}, {WHITE_TERRACOTTA, 7},
		{LIGHT_GRAY_CONCRETE, 4}, {LIGHT_GRAY_CONCRETE_POWDER, 3}, {SANDSTONE, 3},
		{BRICK, 4}, {YELLOW_TERRACOTTA, 2}};
static const WallSource MEDITERRANEAN_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 16},
		{WHITE_TERRACOTTA, 14}, {SMOOTH_SANDSTONE, 11}, {YELLOW_TERRACOTTA, 7},
		{BIRCH_PLANKS, 4}, {CHERRY_PLANKS, 3}, {TERRACOTTA, 5}, {ORANGE_TERRACOTTA, 3},
		{MUSHROOM_STEM, 6}, {CALCITE, 4}, {SANDSTONE, 4}, {BRICK, 4},
		{RED_TERRACOTTA, 3}};
static const WallSource NORDIC_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 12}, {BRICK, 14},
		{POLISHED_GRANITE, 3}, {YELLOW_TERRACOTTA, 6}, {BIRCH_PLANKS, 4},
		{WHITE_TERRACOTTA, 9}, {RED_TERRACOTTA, 6}, {SMOOTH_SANDSTONE, 6},
		{MUSHROOM_STEM, 5}, {LIGHT_GRAY_CONCRETE, 4}, {CALCITE, 3}, {MUD_BRICKS, 3}};
static const WallSource EU_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 16}, {WHITE_CONCRETE, 12},
		{MUSHROOM_STEM, 10}, {WHITE_TERRACOTTA, 7}, {CALCITE, 6}, {BIRCH_PLANKS, 4},
		{LIGHT_GRAY_CONCRETE_POWDER, 6}, {YELLOW_TERRACOTTA, 4}, {SANDSTONE, 4},
		{BRICK, 5}, {QUARTZ_BLOCK, 3}};
static const WallSource EU_MODERN_WALLS[] = {{WHITE_CONCRETE, 18},
		{LIGHT_GRAY_CONCRETE_POWDER, 12}, {SMOOTH_STONE, 8}, {LIGHT_GRAY_CONCRETE, 6},
		{CYAN_TERRACOTTA, 5}, {GRAY_CONCRETE, 3}, {WHITE_TERRACOTTA, 5}, {CALCITE, 5},
		{MUSHROOM_STEM, 5}, {BIRCH_PLANKS, 3}, {BRICK, 4}, {QUARTZ_BLOCK, 2},
		{YELLOW_TERRACOTTA, 2}};
static const WallSource EU_BRICK_MODERN_WALLS[] = {{BRICK, 22}, {POLISHED_GRANITE, 5},
		{MUD_BRICKS, 6}, {BROWN_TERRACOTTA, 3}, {LIGHT_GRAY_CONCRETE, 12},
		{WHITE_CONCRETE, 12}, {GRAY_CONCRETE, 6}, {SMOOTH_STONE, 5},
		{LIGHT_GRAY_CONCRETE_POWDER, 4}, {CALCITE, 3}};
static const WallSource CANADA_HOUSE_WALLS[] = {{BRICK, 16}, {WHITE_CONCRETE, 14},
		{LIGHT_GRAY_CONCRETE, 8}, {CYAN_TERRACOTTA, 6}, {SPRUCE_PLANKS, 6},
		{OAK_PLANKS, 5}, {MUSHROOM_STEM, 7}, {SMOOTH_SANDSTONE, 6}, {WHITE_TERRACOTTA, 6},
		{BROWN_TERRACOTTA, 5}, {POLISHED_GRANITE, 4}, {MUD_BRICKS, 4}, {CALCITE, 4},
		{DARK_OAK_PLANKS, 2}};
static const WallSource CANADA_TRADITIONAL_WALLS[] = {{BRICK, 26}, {POLISHED_GRANITE, 6},
		{MUD_BRICKS, 6}, {LIGHT_GRAY_CONCRETE_POWDER, 8}, {SMOOTH_STONE, 6},
		{STONE_BRICKS, 4}, {SMOOTH_SANDSTONE, 6}, {WHITE_CONCRETE, 6},
		{BROWN_TERRACOTTA, 5}, {MUSHROOM_STEM, 4}, {CYAN_TERRACOTTA, 3},
		{WHITE_TERRACOTTA, 3}};
static const WallSource EAST_HOUSE_WALLS[] = {{SPRUCE_PLANKS, 10}, {DARK_OAK_PLANKS, 6},
		{OAK_PLANKS, 4}, {BIRCH_PLANKS, 4}, {BRICK, 14}, {WHITE_CONCRETE, 12},
		{CALCITE, 6}, {LIGHT_GRAY_CONCRETE, 6}, {SMOOTH_SANDSTONE, 6},
		{YELLOW_TERRACOTTA, 4}, {MUSHROOM_STEM, 6}, {WHITE_TERRACOTTA, 4}};
static const WallSource EAST_TRADITIONAL_WALLS[] = {{YELLOW_TERRACOTTA, 10},
		{BIRCH_PLANKS, 8}, {WHITE_CONCRETE, 14}, {WHITE_TERRACOTTA, 10},
		{CHERRY_PLANKS, 4}, {SMOOTH_SANDSTONE, 8}, {BRICK, 12}, {RED_TERRACOTTA, 4},
		{CYAN_TERRACOTTA, 3}, {MUSHROOM_STEM, 6}, {LIGHT_GRAY_CONCRETE_POWDER, 4},
		{CALCITE, 4}};
static const WallSource EAST_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 14}, {MUSHROOM_STEM, 12},
		{WHITE_TERRACOTTA, 8}, {YELLOW_TERRACOTTA, 6}, {WHITE_CONCRETE, 10},
		{LIGHT_GRAY_CONCRETE_POWDER, 8}, {BRICK, 8}, {POLISHED_GRANITE, 4}, {CALCITE, 6},
		{SANDSTONE, 4}, {RED_TERRACOTTA, 3}, {SMOOTH_STONE, 4}};
static const WallSource EAST_MODERN_WALLS[] = {{LIGHT_GRAY_CONCRETE, 18},
		{WHITE_CONCRETE, 14}, {SMOOTH_STONE, 10}, {MUSHROOM_STEM, 8}, {CALCITE, 8},
		{BRICK, 10}, {LIGHT_GRAY_CONCRETE_POWDER, 8}, {GRAY_CONCRETE, 6},
		{WHITE_TERRACOTTA, 5}, {YELLOW_TERRACOTTA, 3}, {CYAN_TERRACOTTA, 3}};
static const WallSource CENTRAL_ASIA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 14},
		{SMOOTH_SANDSTONE, 12}, {MUD_BRICKS, 8}, {WHITE_TERRACOTTA, 10}, {BRICK, 10},
		{YELLOW_TERRACOTTA, 5}, {TERRACOTTA, 5}, {LIGHT_GRAY_CONCRETE, 8},
		{MUSHROOM_STEM, 6}, {SANDSTONE, 6}, {CALCITE, 4}};
static const WallSource CENTRAL_ASIA_TRADITIONAL_WALLS[] = {{SMOOTH_SANDSTONE, 14},
		{WHITE_TERRACOTTA, 12}, {TERRACOTTA, 6}, {SANDSTONE, 8}, {WHITE_CONCRETE, 12},
		{BRICK, 10}, {MUSHROOM_STEM, 8}, {YELLOW_TERRACOTTA, 5}, {LIGHT_GRAY_CONCRETE, 6},
		{CALCITE, 4}, {MUD_BRICKS, 4}};
static const WallSource MIDDLE_EAST_TRADITIONAL_WALLS[] = {{SMOOTH_SANDSTONE, 20},
		{SANDSTONE, 10}, {WHITE_CONCRETE, 14}, {MUSHROOM_STEM, 10},
		{WHITE_TERRACOTTA, 10}, {CALCITE, 8}, {YELLOW_TERRACOTTA, 5}, {TERRACOTTA, 5},
		{BRICK, 4}, {LIGHT_GRAY_CONCRETE, 6}, {MUD_BRICKS, 4}};
static const WallSource MIDDLE_EAST_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 20},
		{SANDSTONE, 12}, {CALCITE, 10}, {WHITE_CONCRETE, 12}, {MUSHROOM_STEM, 8},
		{QUARTZ_BLOCK, 4}, {YELLOW_TERRACOTTA, 4}, {WHITE_TERRACOTTA, 6}};
static const WallSource MIDDLE_EAST_MODERN_WALLS[] = {{WHITE_CONCRETE, 18},
		{SMOOTH_SANDSTONE, 16}, {CALCITE, 10}, {MUSHROOM_STEM, 10}, {QUARTZ_BLOCK, 6},
		{LIGHT_GRAY_CONCRETE, 10}, {SANDSTONE, 6}, {WHITE_TERRACOTTA, 8},
		{SMOOTH_STONE, 4}};
static const WallSource AFRICA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 16},
		{SMOOTH_SANDSTONE, 12}, {WHITE_TERRACOTTA, 10}, {YELLOW_TERRACOTTA, 8},
		{TERRACOTTA, 8}, {BRICK, 10}, {MUD_BRICKS, 6}, {LIGHT_GRAY_CONCRETE, 10},
		{MUSHROOM_STEM, 6}, {PINK_TERRACOTTA, 3}, {LIME_TERRACOTTA, 3}};
static const WallSource AFRICA_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 16},
		{LIGHT_GRAY_CONCRETE, 12}, {SMOOTH_SANDSTONE, 10}, {WHITE_TERRACOTTA, 8},
		{YELLOW_TERRACOTTA, 8}, {MUSHROOM_STEM, 8}, {TERRACOTTA, 6}, {BRICK, 6},
		{CYAN_TERRACOTTA, 4}, {SANDSTONE, 4}, {PINK_TERRACOTTA, 3}, {LIME_TERRACOTTA, 3}};
static const WallSource AFRICA_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 14},
		{WHITE_CONCRETE, 14}, {SMOOTH_STONE, 10}, {STONE_BRICKS, 8}, {MUSHROOM_STEM, 8},
		{SANDSTONE, 6}, {CALCITE, 6}, {BRICK, 6}, {LIGHT_GRAY_CONCRETE, 6}};
static const WallSource AFRICA_MODERN_WALLS[] = {{WHITE_CONCRETE, 20},
		{LIGHT_GRAY_CONCRETE, 14}, {SMOOTH_SANDSTONE, 10}, {MUSHROOM_STEM, 10},
		{SMOOTH_STONE, 8}, {CALCITE, 6}, {GRAY_CONCRETE, 6}, {WHITE_TERRACOTTA, 6},
		{YELLOW_TERRACOTTA, 4}, {BRICK, 4}, {QUARTZ_BLOCK, 3}};
static const WallSource SOUTH_ASIA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 16},
		{SMOOTH_SANDSTONE, 10}, {WHITE_TERRACOTTA, 10}, {PINK_TERRACOTTA, 6},
		{YELLOW_TERRACOTTA, 6}, {LIME_TERRACOTTA, 4}, {BRICK, 10}, {TERRACOTTA, 4},
		{LIGHT_GRAY_CONCRETE, 10}, {MUSHROOM_STEM, 6}, {CALCITE, 4}};
static const WallSource SOUTH_ASIA_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 12},
		{LIGHT_GRAY_CONCRETE, 12}, {CYAN_TERRACOTTA, 8}, {SMOOTH_SANDSTONE, 10},
		{WHITE_TERRACOTTA, 8}, {YELLOW_TERRACOTTA, 6}, {PINK_TERRACOTTA, 4},
		{LIME_TERRACOTTA, 3}, {BRICK, 8}, {MUSHROOM_STEM, 8}, {GRAY_CONCRETE, 5},
		{SANDSTONE, 4}};
static const WallSource SOUTH_ASIA_CIVIC_WALLS[] = {{SANDSTONE, 14},
		{SMOOTH_SANDSTONE, 12}, {TERRACOTTA, 8}, {WHITE_CONCRETE, 12}, {MUSHROOM_STEM, 8},
		{TUFF, 6}, {BRICK, 8}, {CALCITE, 4}, {LIGHT_GRAY_CONCRETE, 6}};
static const WallSource SOUTH_ASIA_MODERN_WALLS[] = {{WHITE_CONCRETE, 20},
		{LIGHT_GRAY_CONCRETE, 14}, {SMOOTH_SANDSTONE, 10}, {MUSHROOM_STEM, 10},
		{CYAN_TERRACOTTA, 6}, {WHITE_TERRACOTTA, 6}, {SMOOTH_STONE, 6},
		{GRAY_CONCRETE, 5}, {CALCITE, 5}, {YELLOW_TERRACOTTA, 3}, {QUARTZ_BLOCK, 3}};
static const WallSource SOUTHEAST_ASIA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 18},
		{SMOOTH_SANDSTONE, 12}, {WHITE_TERRACOTTA, 10}, {YELLOW_TERRACOTTA, 6},
		{PINK_TERRACOTTA, 5}, {LIME_TERRACOTTA, 4}, {LIGHT_GRAY_CONCRETE, 10},
		{MUSHROOM_STEM, 8}, {BIRCH_PLANKS, 6}, {OAK_PLANKS, 4}, {TERRACOTTA, 4}};
static const WallSource SOUTHEAST_ASIA_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 16},
		{SMOOTH_SANDSTONE, 10}, {WHITE_TERRACOTTA, 10}, {YELLOW_TERRACOTTA, 8},
		{PINK_TERRACOTTA, 5}, {LIME_TERRACOTTA, 4}, {LIGHT_GRAY_CONCRETE, 10},
		{MUSHROOM_STEM, 8}, {CYAN_TERRACOTTA, 5}, {GRAY_CONCRETE, 4}, {CALCITE, 4}};
static const WallSource SOUTHEAST_ASIA_MODERN_WALLS[] = {{WHITE_CONCRETE, 22},
		{LIGHT_GRAY_CONCRETE, 14}, {MUSHROOM_STEM, 10}, {SMOOTH_SANDSTONE, 8},
		{CALCITE, 8}, {SMOOTH_STONE, 8}, {WHITE_TERRACOTTA, 6}, {GRAY_CONCRETE, 5},
		{QUARTZ_BLOCK, 4}, {CYAN_TERRACOTTA, 3}, {YELLOW_TERRACOTTA, 3}};
static const WallSource EAST_ASIA_HOUSE_WALLS[] = {{WHITE_CONCRETE, 18}, {BRICK, 14},
		{STONE_BRICKS, 10}, {LIGHT_GRAY_CONCRETE, 10}, {MUSHROOM_STEM, 8},
		{WHITE_TERRACOTTA, 8}, {SMOOTH_SANDSTONE, 6}, {CALCITE, 6}, {TERRACOTTA, 4},
		{POLISHED_ANDESITE, 4}};
static const WallSource EAST_ASIA_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 16},
		{LIGHT_GRAY_CONCRETE, 14}, {STONE_BRICKS, 8}, {BRICK, 8}, {MUSHROOM_STEM, 10},
		{WHITE_TERRACOTTA, 10}, {SMOOTH_SANDSTONE, 8}, {SMOOTH_STONE, 6},
		{GRAY_CONCRETE, 4}, {CYAN_TERRACOTTA, 4}, {CALCITE, 4}};
static const WallSource EAST_ASIA_MODERN_WALLS[] = {{WHITE_CONCRETE, 24},
		{LIGHT_GRAY_CONCRETE, 14}, {MUSHROOM_STEM, 12}, {SMOOTH_STONE, 8}, {CALCITE, 8},
		{SMOOTH_SANDSTONE, 6}, {WHITE_TERRACOTTA, 6}, {QUARTZ_BLOCK, 4},
		{GRAY_CONCRETE, 4}, {BROWN_TERRACOTTA, 4}};
static const WallSource KOREA_HOUSE_WALLS[] = {{BRICK, 20}, {MUD_BRICKS, 8},
		{WHITE_CONCRETE, 16}, {MUSHROOM_STEM, 10}, {LIGHT_GRAY_CONCRETE, 10},
		{SMOOTH_SANDSTONE, 8}, {WHITE_TERRACOTTA, 8}, {BROWN_TERRACOTTA, 6},
		{CALCITE, 4}};
static const WallSource KOREA_TRADITIONAL_WALLS[] = {{BRICK, 18}, {MUD_BRICKS, 8},
		{POLISHED_GRANITE, 4}, {WHITE_CONCRETE, 16}, {LIGHT_GRAY_CONCRETE, 12},
		{MUSHROOM_STEM, 10}, {SMOOTH_STONE, 6}, {WHITE_TERRACOTTA, 6},
		{BROWN_TERRACOTTA, 6}, {GRAY_CONCRETE, 4}};
static const WallSource JAPAN_HOUSE_WALLS[] = {{WHITE_CONCRETE, 14}, {MUSHROOM_STEM, 14},
		{SMOOTH_SANDSTONE, 10}, {CALCITE, 8}, {LIGHT_GRAY_CONCRETE, 10},
		{LIGHT_GRAY_CONCRETE_POWDER, 8}, {WHITE_TERRACOTTA, 8}, {BIRCH_PLANKS, 5},
		{BROWN_TERRACOTTA, 6}, {SPRUCE_PLANKS, 4}, {DARK_OAK_PLANKS, 3},
		{CYAN_TERRACOTTA, 3}, {SMOOTH_STONE, 4}};
static const WallSource JAPAN_URBAN_WALLS[] = {{WHITE_CONCRETE, 16},
		{LIGHT_GRAY_CONCRETE, 14}, {MUSHROOM_STEM, 12}, {SMOOTH_STONE, 8},
		{WHITE_TERRACOTTA, 8}, {BROWN_TERRACOTTA, 8}, {SMOOTH_SANDSTONE, 8}, {CALCITE, 6},
		{TERRACOTTA, 4}, {BRICK, 4}, {GRAY_CONCRETE, 4}, {QUARTZ_BLOCK, 3}};
static const WallSource LATAM_HOUSE_WALLS[] = {{WHITE_CONCRETE, 14},
		{YELLOW_TERRACOTTA, 8}, {ORANGE_TERRACOTTA, 6}, {PINK_TERRACOTTA, 7},
		{WHITE_TERRACOTTA, 10}, {SMOOTH_SANDSTONE, 10}, {LIME_TERRACOTTA, 4},
		{TERRACOTTA, 6}, {BRICK, 10}, {LIGHT_GRAY_CONCRETE, 8}, {RED_TERRACOTTA, 3},
		{MUSHROOM_STEM, 4}};
static const WallSource LATAM_TRADITIONAL_WALLS[] = {{WHITE_CONCRETE, 12},
		{SMOOTH_SANDSTONE, 12}, {WHITE_TERRACOTTA, 10}, {YELLOW_TERRACOTTA, 8},
		{PINK_TERRACOTTA, 6}, {ORANGE_TERRACOTTA, 4}, {TERRACOTTA, 6}, {MUSHROOM_STEM, 8},
		{LIGHT_GRAY_CONCRETE, 8}, {BRICK, 6}, {RED_TERRACOTTA, 4}, {LIME_TERRACOTTA, 3},
		{CALCITE, 4}};
static const WallSource LATAM_CIVIC_WALLS[] = {{SMOOTH_SANDSTONE, 16}, {SANDSTONE, 6},
		{WHITE_CONCRETE, 12}, {MUSHROOM_STEM, 10}, {CALCITE, 6}, {RED_TERRACOTTA, 6},
		{YELLOW_TERRACOTTA, 6}, {WHITE_TERRACOTTA, 8}, {LIGHT_GRAY_CONCRETE, 6},
		{TUFF, 4}};
static const WallSource LATAM_MODERN_WALLS[] = {{WHITE_CONCRETE, 18},
		{LIGHT_GRAY_CONCRETE, 14}, {SMOOTH_STONE, 8}, {MUSHROOM_STEM, 10},
		{SMOOTH_SANDSTONE, 8}, {WHITE_TERRACOTTA, 6}, {BRICK, 8}, {GRAY_CONCRETE, 6},
		{CALCITE, 5}, {YELLOW_TERRACOTTA, 3}, {TERRACOTTA, 4}, {QUARTZ_BLOCK, 3}};
static const WallSource SOUTHERN_CONE_HOUSE_WALLS[] = {{WHITE_CONCRETE, 16}, {BRICK, 16},
		{MUSHROOM_STEM, 10}, {SMOOTH_SANDSTONE, 10}, {WHITE_TERRACOTTA, 10},
		{LIGHT_GRAY_CONCRETE, 8}, {YELLOW_TERRACOTTA, 5}, {TERRACOTTA, 5}, {CALCITE, 5},
		{MUD_BRICKS, 4}, {PINK_TERRACOTTA, 3}};
static const WallSource SOUTHERN_CONE_TRADITIONAL_WALLS[] = {{MUSHROOM_STEM, 16},
		{SMOOTH_SANDSTONE, 12}, {WHITE_CONCRETE, 12}, {LIGHT_GRAY_CONCRETE_POWDER, 8},
		{WHITE_TERRACOTTA, 10}, {CALCITE, 6}, {BRICK, 8}, {YELLOW_TERRACOTTA, 4},
		{LIGHT_GRAY_CONCRETE, 6}, {PINK_TERRACOTTA, 3}, {SMOOTH_STONE, 4}};
static const WallSource AUSTRALASIA_HOUSE_WALLS[] = {{BRICK, 18}, {TERRACOTTA, 8},
		{MUD_BRICKS, 6}, {SMOOTH_SANDSTONE, 8}, {WHITE_CONCRETE, 14}, {MUSHROOM_STEM, 8},
		{BIRCH_PLANKS, 6}, {CALCITE, 5}, {LIGHT_GRAY_CONCRETE, 6}, {WHITE_TERRACOTTA, 5},
		{OAK_PLANKS, 4}, {BROWN_TERRACOTTA, 3}, {CYAN_TERRACOTTA, 2}};
static const WallSource AUSTRALASIA_TRADITIONAL_WALLS[] = {{BRICK, 16}, {SANDSTONE, 8},
		{SMOOTH_SANDSTONE, 10}, {WHITE_CONCRETE, 12}, {MUSHROOM_STEM, 8},
		{WHITE_TERRACOTTA, 8}, {MUD_BRICKS, 6}, {TERRACOTTA, 5}, {CALCITE, 4},
		{YELLOW_TERRACOTTA, 3}, {LIGHT_GRAY_CONCRETE, 5}};
static const WallSource AUSTRALASIA_CIVIC_WALLS[] = {{SANDSTONE, 14},
		{SMOOTH_SANDSTONE, 14}, {BRICK, 10}, {WHITE_CONCRETE, 8}, {MUSHROOM_STEM, 8},
		{CALCITE, 4}, {TUFF, 4}, {LIGHT_GRAY_CONCRETE, 6}};
static const WallSource AUSTRALASIA_MODERN_WALLS[] = {{WHITE_CONCRETE, 16},
		{LIGHT_GRAY_CONCRETE, 14}, {BRICK, 12}, {SMOOTH_STONE, 8}, {MUSHROOM_STEM, 8},
		{CALCITE, 6}, {GRAY_CONCRETE, 6}, {SMOOTH_SANDSTONE, 6}, {MUD_BRICKS, 4},
		{TERRACOTTA, 4}, {QUARTZ_BLOCK, 3}};

WallPool regional_wall_pool(building_style::MaterialRegion region, WallTypology type)
{
	using R = building_style::MaterialRegion;
	using T = WallTypology;
	switch (region) {
	case R::Other:
		return {};
	case R::Canada:
		switch (type) {
		case T::House:
			return CANADA_HOUSE_WALLS;
		case T::Traditional:
			return CANADA_TRADITIONAL_WALLS;
		case T::Civic:
			return NA_CIVIC_WALLS;
		case T::Modern:
			return NA_MODERN_WALLS;
		}
		break;
	case R::EastEurope:
		switch (type) {
		case T::House:
			return EAST_HOUSE_WALLS;
		case T::Traditional:
			return EAST_TRADITIONAL_WALLS;
		case T::Civic:
			return EAST_CIVIC_WALLS;
		case T::Modern:
			return EAST_MODERN_WALLS;
		}
		break;
	case R::CentralAsia:
		if (type == T::House)
			return CENTRAL_ASIA_HOUSE_WALLS;
		if (type == T::Modern)
			return EAST_MODERN_WALLS;
		return CENTRAL_ASIA_TRADITIONAL_WALLS;
	case R::MiddleEast:
		if (type == T::House || type == T::Traditional)
			return MIDDLE_EAST_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return MIDDLE_EAST_CIVIC_WALLS;
		return MIDDLE_EAST_MODERN_WALLS;
	case R::SubSaharanAfrica:
		if (type == T::House)
			return AFRICA_HOUSE_WALLS;
		if (type == T::Traditional)
			return AFRICA_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return AFRICA_CIVIC_WALLS;
		return AFRICA_MODERN_WALLS;
	case R::SouthAsia:
		if (type == T::House)
			return SOUTH_ASIA_HOUSE_WALLS;
		if (type == T::Traditional)
			return SOUTH_ASIA_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return SOUTH_ASIA_CIVIC_WALLS;
		return SOUTH_ASIA_MODERN_WALLS;
	case R::SoutheastAsia:
		if (type == T::House)
			return SOUTHEAST_ASIA_HOUSE_WALLS;
		if (type == T::Traditional)
			return SOUTHEAST_ASIA_TRADITIONAL_WALLS;
		return SOUTHEAST_ASIA_MODERN_WALLS;
	case R::EastAsia:
		if (type == T::House)
			return EAST_ASIA_HOUSE_WALLS;
		if (type == T::Traditional)
			return EAST_ASIA_TRADITIONAL_WALLS;
		return EAST_ASIA_MODERN_WALLS;
	case R::Korea:
		if (type == T::House)
			return KOREA_HOUSE_WALLS;
		if (type == T::Traditional)
			return KOREA_TRADITIONAL_WALLS;
		return EAST_ASIA_MODERN_WALLS;
	case R::Japan:
		return type == T::House ? WallPool(JAPAN_HOUSE_WALLS)
								: WallPool(JAPAN_URBAN_WALLS);
	case R::LatinAmerica:
		if (type == T::House)
			return LATAM_HOUSE_WALLS;
		if (type == T::Traditional)
			return LATAM_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return LATAM_CIVIC_WALLS;
		return LATAM_MODERN_WALLS;
	case R::SouthernCone:
		if (type == T::House)
			return SOUTHERN_CONE_HOUSE_WALLS;
		if (type == T::Traditional)
			return SOUTHERN_CONE_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return EU_CIVIC_WALLS;
		return LATAM_MODERN_WALLS;
	case R::Australasia:
		if (type == T::House)
			return AUSTRALASIA_HOUSE_WALLS;
		if (type == T::Traditional)
			return AUSTRALASIA_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return AUSTRALASIA_CIVIC_WALLS;
		return AUSTRALASIA_MODERN_WALLS;
	case R::NorthAmerica:
		if (type == T::House)
			return NA_HOUSE_WALLS;
		if (type == T::Traditional)
			return NA_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return NA_CIVIC_WALLS;
		return NA_MODERN_WALLS;
	case R::NorthAmericaSunbelt:
		if (type == T::House)
			return SUNBELT_HOUSE_WALLS;
		if (type == T::Traditional)
			return SUNBELT_TRADITIONAL_WALLS;
		if (type == T::Civic)
			return NA_CIVIC_WALLS;
		return NA_MODERN_WALLS;
	case R::BritishIsles:
		if (type == T::Modern)
			return EU_BRICK_MODERN_WALLS;
		if (type == T::Civic)
			return EU_CIVIC_WALLS;
		return BRITISH_TRADITIONAL_WALLS;
	case R::BrickNorth:
		if (type == T::Modern)
			return EU_BRICK_MODERN_WALLS;
		return BRICK_NORTH_TRADITIONAL_WALLS;
	case R::Nordic:
		if (type == T::Modern)
			return EU_BRICK_MODERN_WALLS;
		if (type == T::Civic)
			return EU_CIVIC_WALLS;
		return NORDIC_TRADITIONAL_WALLS;
	case R::France:
		if (type == T::Modern)
			return EU_MODERN_WALLS;
		if (type == T::Civic)
			return FRANCE_TRADITIONAL_WALLS;
		return FRANCE_TRADITIONAL_WALLS;
	case R::Central:
	case R::CentralEast:
		if (type == T::Modern)
			return EU_MODERN_WALLS;
		return CENTRAL_TRADITIONAL_WALLS;
	case R::Mediterranean:
		if (type == T::Modern)
			return EU_MODERN_WALLS;
		if (type == T::Civic)
			return EU_CIVIC_WALLS;
		return MEDITERRANEAN_TRADITIONAL_WALLS;
	}
	return {};
}

std::optional<WallTypology> wall_typology(BuildingCategory category, ArchEra era,
		int height, double scale, building_style::MaterialRegion region, bool attached)
{
	using R = building_style::MaterialRegion;
	const bool modern = era == ArchEra::PostWarPanel || era == ArchEra::Contemporary;
	const bool old = era == ArchEra::HistoricOrnate || era == ArchEra::TraditionalPreWar;
	const bool high_rise = static_cast<double>(height) >= 30.0 * scale;
	const bool american = region == R::NorthAmerica || region == R::NorthAmericaSunbelt ||
						  region == R::Canada;
	const bool small_house = category == BuildingCategory::Default &&
							 building_style::roof_habit(region).has_value() &&
							 !attached && static_cast<double>(height) <= 11.0 * scale;
	switch (category) {
	case BuildingCategory::House:
		return modern && !american ? WallTypology::Modern : WallTypology::House;
	case BuildingCategory::Default:
		if (small_house && !modern)
			return WallTypology::House;
		[[fallthrough]];
	case BuildingCategory::Residential:
	case BuildingCategory::Commercial:
	case BuildingCategory::Hotel:
		if (modern)
			return WallTypology::Modern;
		if (old || !high_rise)
			return WallTypology::Traditional;
		return american ? WallTypology::Civic : WallTypology::Modern;
	case BuildingCategory::Office:
	case BuildingCategory::TallBuilding:
		return modern || (!old && !american && high_rise) ? WallTypology::Modern
														  : WallTypology::Civic;
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		return modern ? WallTypology::Modern : WallTypology::Civic;
	case BuildingCategory::Historic:
		return WallTypology::Civic;
	default:
		return std::nullopt;
	}
}

std::uint64_t weighted_total(const std::vector<WallEntry> &pool)
{
	std::uint64_t total = 0;
	for (const auto &[_, weight] : pool)
		total += weight;
	return total;
}
std::size_t weighted_pick(const std::vector<WallEntry> &pool, ChaCha8Rng &rng)
{
	const auto total = static_cast<std::uint32_t>(weighted_total(pool));
	std::uint32_t roll = rng.uniform(std::max<std::uint32_t>(1, total));
	for (std::size_t i = 0; i < pool.size(); ++i) {
		if (roll < pool[i].second)
			return i;
		roll -= pool[i].second;
	}
	return 0;
}

Block pick_district_wall(WallPool pool, BuildingCategory category, biome::Climate climate,
		std::uint64_t district, ChaCha8Rng &rng)
{
	const bool reweight =
			climate != biome::Climate::Temperate && climate_applies(climate, category);
	std::vector<WallEntry> weighted;
	weighted.reserve(pool.size() + 3);
	for (const auto &[block, weight] : pool) {
		const auto factor = reweight ? climate_wall_weight(climate, category, block) : 1;
		if (factor)
			weighted.emplace_back(block, static_cast<std::uint8_t>(weight * factor));
	}
	if (reweight &&
			(climate == biome::Climate::Boreal || climate == biome::Climate::Tundra ||
					climate == biome::Climate::IceCap)) {
		for (Block block : {SPRUCE_PLANKS, OAK_PLANKS, DARK_OAK_PLANKS}) {
			const auto factor = climate_wall_weight(climate, category, block);
			if (factor)
				weighted.emplace_back(block, static_cast<std::uint8_t>(4 * factor));
		}
	}
	if (weighted.size() < 2)
		weighted.assign(pool.begin(), pool.end());
	if (weighted.empty())
		return STONE;
	auto district_rng = element_rng(district ^ 0x5B9A1E77D1570002ULL);
	const auto keep = std::min<std::size_t>(3 + district_rng.uniform(3), weighted.size());
	auto rest = weighted;
	std::vector<WallEntry> sub;
	sub.reserve(keep);
	// Weighted sampling without replacement; swap_remove mirrors Rust's vector.
	for (std::size_t i = 0; i < keep; ++i) {
		const auto at = weighted_pick(rest, district_rng);
		sub.push_back(rest[at]);
		rest[at] = rest.back();
		rest.pop_back();
	}
	const auto &source = rng.random_bool(0.12) ? weighted : sub;
	return source[weighted_pick(source, rng)].first;
}
} // namespace

std::optional<Block> rust_regional_wall_block(BuildingCategory category, ArchEra era,
		biome::Climate climate, building_style::MaterialRegion region, int height,
		double scale, bool attached, bool historic_fortification, std::uint64_t district,
		ChaCha8Rng &rng)
{
	std::optional<WallTypology> typology;
	if (category == BuildingCategory::Historic) {
		if (!historic_fortification)
			typology = WallTypology::Civic;
	} else {
		typology = wall_typology(category, era, height, scale, region, attached);
	}
	if (!typology)
		return std::nullopt;
	const auto pool = regional_wall_pool(region, *typology);
	if (pool.empty())
		return std::nullopt;
	return pick_district_wall(pool, category, climate, district, rng);
}

} // namespace arnis::buildings
