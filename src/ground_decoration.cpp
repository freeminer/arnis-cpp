#include "ground_decoration.h"

#include "args.h"
#include "block_definitions.h"
#include "celestial.h"
#include "deterministic_rng.h"
#include "land_cover/land_cover.h"
#include "ecoregion.h"
#include "floodfill_cache.h"
#include "ground_generation.h"
#include "../../arnis_adapter.h"

#include <iterator>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace arnis::ground_decoration
{
namespace
{
using FlowerPalette = std::vector<std::pair<Block, std::uint32_t>>;
using TallFlowerPalette = std::vector<std::pair<std::pair<Block, Block>, std::uint32_t>>;

constexpr std::uint32_t SALT_SCATTER = 0x5CA77E11u;
constexpr std::uint32_t SALT_BED = 0x0BEDF10Eu;

const std::vector<Block> &loose_plants()
{
	static const std::vector<Block> blocks{GRASS, TALL_GRASS_BOTTOM, TALL_GRASS_TOP, FERN,
			LARGE_FERN_LOWER, LARGE_FERN_UPPER, DEAD_BUSH, RED_FLOWER, YELLOW_FLOWER,
			BLUE_FLOWER, WHITE_FLOWER, CORNFLOWER, OXEYE_DAISY, ALLIUM,
			LILY_OF_THE_VALLEY, RED_TULIP, ORANGE_TULIP, WHITE_TULIP, PINK_TULIP,
			SUNFLOWER_LOWER, SUNFLOWER_UPPER, LILAC_LOWER, LILAC_UPPER, ROSE_BUSH_LOWER,
			ROSE_BUSH_UPPER, PEONY_LOWER, PEONY_UPPER, SWEET_BERRY_BUSH, BROWN_MUSHROOM,
			RED_MUSHROOM, MOSS_CARPET, SUGAR_CANE, PUMPKIN, CACTUS, OAK_LEAVES};
	return blocks;
}

const std::vector<Block> &plant_lower_halves()
{
	static const std::vector<Block> blocks{TALL_GRASS_BOTTOM, LARGE_FERN_LOWER,
			SUNFLOWER_LOWER, LILAC_LOWER, ROSE_BUSH_LOWER, PEONY_LOWER};
	return blocks;
}

bool open_above(WorldEditor &e, int x, int y, int z)
{
	const bool first_open = !e.block_exists_absolute(x, y + 1, z) ||
							e.check_for_block_absolute(x, y + 1, z,
									std::optional<std::vector<Block>>{{GRASS}});
	const std::vector<Block> wood{OAK_LOG, SPRUCE_LOG, BIRCH_LOG, DARK_OAK_LOG,
			JUNGLE_LOG, ACACIA_LOG, CHERRY_LOG};
	return first_open && !e.check_for_block_absolute(x, y + 2, z, wood);
}

bool is_undergrowth_impl(const Block &block)
{
	return block != OAK_LEAVES && std::find(loose_plants().begin(), loose_plants().end(),
										  block) != loose_plants().end();
}

void clear_undergrowth_under_trunk_impl(WorldEditor &editor, int x, int y, int z)
{
	const auto below = editor.get_block_absolute(x, y - 1, z);
	if (!below || !is_undergrowth_impl(*below))
		return;
	const auto &loose = loose_plants();
	const auto &lower = plant_lower_halves();
	editor.set_block_absolute(AIR, x, y - 1, z, &loose, nullptr);
	editor.set_block_absolute(AIR, x, y - 2, z, &lower, nullptr);
}

void plant(WorldEditor &e, int x, int y, int z, const Block &soil, const Block &block)
{
	if (!e.check_for_block_absolute(
				x, y, z, std::optional<std::vector<Block>>(std::vector<Block>{soil})) ||
			!open_above(e, x, y, z))
		return;
	e.set_block_absolute(block, x, y + 1, z,
			std::optional<std::vector<Block>>(std::vector<Block>{GRASS}), std::nullopt);
}

void tall_plant(
		WorldEditor &e, int x, int y, int z, const Block &lower, const Block &upper)
{
	if (!e.check_for_block_absolute(x, y, z,
				std::optional<std::vector<Block>>(
						std::vector<Block>{GRASS_BLOCK, DIRT})) ||
			!open_above(e, x, y, z) || e.block_exists_absolute(x, y + 2, z))
		return;
	e.set_block_absolute(lower, x, y + 1, z,
			std::optional<std::vector<Block>>(std::vector<Block>{GRASS}), std::nullopt);
	e.set_block_absolute(upper, x, y + 2, z, std::nullopt, std::nullopt);
}

void stack(WorldEditor &e, int x, int y, int z, const Block &soil,
		const Block &plant_block, int height, bool clear_sides = false)
{
	if (!e.check_for_block_absolute(
				x, y, z, std::optional<std::vector<Block>>(std::vector<Block>{soil})) ||
			!open_above(e, x, y, z))
		return;
	for (int dy = 1; dy <= height; ++dy) {
		if (e.block_exists_absolute(x, y + dy, z))
			break;
		if (clear_sides && (e.block_exists_absolute(x + 1, y + dy, z) ||
								   e.block_exists_absolute(x - 1, y + dy, z) ||
								   e.block_exists_absolute(x, y + dy, z + 1) ||
								   e.block_exists_absolute(x, y + dy, z - 1)))
			break;
		e.set_block_absolute(plant_block, x, y + dy, z, std::nullopt, std::nullopt);
	}
}

bool beside_water(WorldEditor &e, int x, int y, int z)
{
	const std::vector<Block> water{WATER};
	return e.check_for_block_absolute(x + 1, y, z, water) ||
		   e.check_for_block_absolute(x - 1, y, z, water) ||
		   e.check_for_block_absolute(x, y, z + 1, water) ||
		   e.check_for_block_absolute(x, y, z - 1, water);
}

template <typename T>
T pick_weighted(const std::vector<std::pair<T, std::uint32_t>> &palette,
		std::uint32_t roll_permille);

Block flower_for(int x, int z, FlowerSetting setting, bool bed = false)
{
	static const FlowerPalette meadow{{YELLOW_FLOWER, 30}, {RED_FLOWER, 25},
			{OXEYE_DAISY, 20}, {CORNFLOWER, 12}, {WHITE_FLOWER, 12}, {ALLIUM, 4},
			{RED_TULIP, 3}, {ORANGE_TULIP, 2}, {WHITE_TULIP, 2}, {PINK_TULIP, 2}};
	static const FlowerPalette forest{{RED_FLOWER, 30}, {YELLOW_FLOWER, 25},
			{LILY_OF_THE_VALLEY, 25}, {ALLIUM, 5}, {OXEYE_DAISY, 5}};
	static const FlowerPalette garden{{RED_TULIP, 18}, {PINK_TULIP, 14},
			{WHITE_TULIP, 12}, {ORANGE_TULIP, 12}, {ALLIUM, 12}, {CORNFLOWER, 10},
			{OXEYE_DAISY, 12}, {RED_FLOWER, 10}};
	const FlowerPalette &palette = setting == FlowerSetting::Forest	  ? forest
								   : setting == FlowerSetting::Garden ? garden
																	  : meadow;
	const int scale = bed ? 4 : 24;
	const std::uint32_t salt = bed ? SALT_BED : SALT_SCATTER;
	const auto zone = ground_generation::patch_noise(x, z, scale, salt);
	const auto variation = land_cover::coord_hash(x ^ 0x5ca7, z ^ 0x7e11);
	const auto roll = bed || variation % 100 < 85
							  ? static_cast<std::uint32_t>(zone * 999.0)
							  : static_cast<std::uint32_t>((variation >> 8) % 1000);
	return pick_weighted(palette, roll);
}
}

bool is_undergrowth(const Block &block)
{
	return is_undergrowth_impl(block);
}

void clear_undergrowth_under_trunk(WorldEditor &editor, int x, int y, int z)
{
	clear_undergrowth_under_trunk_impl(editor, x, y, z);
}

std::optional<Habitat> habitat(std::uint8_t cover, biome::Climate climate,
		double absolute_latitude, bool alpine, std::optional<ecoregion::Ecoregion> eco)
{
	using biome::Climate;
	using ecoregion::EcoBiome;
	using land_cover::LC_BARE;
	using land_cover::LC_GRASSLAND;
	using land_cover::LC_MANGROVES;
	using land_cover::LC_MOSS;
	using land_cover::LC_SHRUBLAND;
	using land_cover::LC_TREE_COVER;
	using land_cover::LC_WETLAND;

	if (eco && !alpine && climate != Climate::HotDesert &&
			climate != Climate::ColdDesert && climate != Climate::Boreal &&
			climate != Climate::Tundra && climate != Climate::IceCap) {
		const bool steppe_climate =
				climate == Climate::HotSteppe || climate == Climate::ColdSteppe;
		switch (cover) {
		case LC_TREE_COVER:
			switch (eco->biome) {
			case EcoBiome::MoistTropical:
				return Habitat::Jungle;
			case EcoBiome::DryTropical:
			case EcoBiome::TropicalGrassland:
			case EcoBiome::Mediterranean:
			case EcoBiome::Desert:
				return Habitat::Shrub;
			case EcoBiome::TemperateConifer:
			case EcoBiome::Boreal:
			case EcoBiome::Tundra:
				return Habitat::Taiga;
			case EcoBiome::TemperateBroadleaf:
			case EcoBiome::TropicalConifer:
			case EcoBiome::TemperateGrassland:
			case EcoBiome::MontaneGrassland:
				return Habitat::Forest;
			case EcoBiome::Flooded:
			case EcoBiome::Mangroves:
				break;
			}
			break;
		case LC_GRASSLAND:
		case LC_SHRUBLAND:
			switch (eco->biome) {
			case EcoBiome::TemperateGrassland:
				if (!steppe_climate)
					return Habitat::Prairie;
				break;
			case EcoBiome::Mediterranean:
				return Habitat::Maquis;
			case EcoBiome::TropicalGrassland:
				return Habitat::Savanna;
			case EcoBiome::MontaneGrassland:
				return Habitat::Alpine;
			case EcoBiome::Desert:
				return Habitat::Steppe;
			default:
				break;
			}
			break;
		default:
			break;
		}
	}

	const bool arid = climate == Climate::HotDesert || climate == Climate::ColdDesert;
	const bool dry = climate == Climate::HotSteppe || climate == Climate::ColdSteppe ||
					 climate == Climate::TropicalSavanna ||
					 climate == Climate::DryContinental;
	const bool polar = climate == Climate::Tundra || climate == Climate::IceCap;
	switch (cover) {
	case LC_TREE_COVER:
		if (climate == Climate::Boreal || polar || absolute_latitude > 55.0)
			return Habitat::Taiga;
		if (arid || dry)
			return Habitat::Shrub;
		if (absolute_latitude < 23.5)
			return Habitat::Jungle;
		return Habitat::Forest;
	case LC_SHRUBLAND:
	case LC_GRASSLAND:
		if (alpine && !arid)
			return Habitat::Alpine;
		if (cover == LC_SHRUBLAND && (arid || dry))
			return Habitat::Steppe;
		if (cover == LC_SHRUBLAND && polar)
			return Habitat::Tundra;
		if (cover == LC_SHRUBLAND)
			return Habitat::Shrub;
		if (arid)
			return Habitat::Desert;
		if (dry)
			return Habitat::Steppe;
		if (polar)
			return Habitat::Tundra;
		return Habitat::Meadow;
	case LC_MOSS:
		return Habitat::Tundra;
	case LC_WETLAND:
	case LC_MANGROVES:
		return Habitat::Wetland;
	case LC_BARE:
		if (arid || dry)
			return Habitat::Desert;
		return std::nullopt;
	default:
		return std::nullopt;
	}
}

namespace
{
enum class PatchFeature
{
	Flowers,
	TallFlowers,
	TallGrass,
	Ferns,
	Mushrooms,
	SugarCane,
	Pumpkins,
	SweetBerries,
	DeadBushes,
	Cactus,
	LilyPads,
	Moss
};

struct WeightedFeature
{
	PatchFeature feature;
	std::uint32_t weight;
};

std::vector<WeightedFeature> features(Habitat h)
{
	using F = PatchFeature;
	using H = Habitat;
	switch (h) {
	case H::Meadow:
		return {{F::Flowers, 90}, {F::TallGrass, 100}, {F::TallFlowers, 6},
				{F::Pumpkins, 1}, {F::SugarCane, 70}};
	case H::Alpine:
		return {{F::Flowers, 160}, {F::TallGrass, 50}};
	case H::Forest:
		return {{F::Flowers, 50}, {F::TallFlowers, 35}, {F::Ferns, 80},
				{F::Mushrooms, 60}, {F::SugarCane, 50}, {F::Pumpkins, 1}};
	case H::Taiga:
		return {{F::Ferns, 200}, {F::SweetBerries, 60}, {F::Mushrooms, 80},
				{F::Flowers, 20}, {F::Moss, 40}};
	case H::Jungle:
		return {{F::Ferns, 180}, {F::Flowers, 30}, {F::SugarCane, 100},
				{F::TallGrass, 60}};
	case H::Shrub:
		return {{F::Flowers, 50}, {F::TallGrass, 70}};
	case H::Steppe:
		return {{F::TallGrass, 140}, {F::DeadBushes, 80}, {F::Flowers, 20},
				{F::SugarCane, 60}, {F::Cactus, 30}};
	case H::Desert:
		return {{F::DeadBushes, 160}, {F::Cactus, 80}, {F::SugarCane, 100}};
	case H::Tundra:
		return {{F::Flowers, 30}, {F::Moss, 60}, {F::SweetBerries, 20}};
	case H::Wetland:
		return {{F::LilyPads, 180}, {F::Flowers, 60}, {F::SugarCane, 100},
				{F::TallGrass, 60}, {F::Mushrooms, 20}};
	case H::Prairie:
		return {{F::TallGrass, 180}, {F::Flowers, 70}, {F::TallFlowers, 25},
				{F::SugarCane, 40}};
	case H::Maquis:
		return {{F::Flowers, 60}, {F::TallGrass, 60}, {F::DeadBushes, 40},
				{F::SweetBerries, 30}, {F::SugarCane, 30}};
	case H::Savanna:
		return {{F::TallGrass, 220}, {F::DeadBushes, 30}, {F::Flowers, 20},
				{F::SugarCane, 40}};
	}
	return {};
}

FlowerPalette flower_palette(Habitat h)
{
	using H = Habitat;
	switch (h) {
	case H::Meadow:
		return {{YELLOW_FLOWER, 30}, {RED_FLOWER, 25}, {OXEYE_DAISY, 20},
				{CORNFLOWER, 12}, {WHITE_FLOWER, 12}, {ALLIUM, 4}, {RED_TULIP, 3},
				{ORANGE_TULIP, 2}, {WHITE_TULIP, 2}, {PINK_TULIP, 2}};
	case H::Forest:
		return {{RED_FLOWER, 30}, {YELLOW_FLOWER, 25}, {LILY_OF_THE_VALLEY, 25},
				{ALLIUM, 5}, {OXEYE_DAISY, 5}};
	case H::Taiga:
		return {{YELLOW_FLOWER, 40}, {LILY_OF_THE_VALLEY, 25}, {RED_FLOWER, 20},
				{WHITE_FLOWER, 15}};
	case H::Jungle:
		return {{RED_FLOWER, 45}, {BLUE_FLOWER, 30}, {YELLOW_FLOWER, 25}};
	case H::Shrub:
		return {{YELLOW_FLOWER, 30}, {RED_FLOWER, 30}, {ALLIUM, 20}, {WHITE_FLOWER, 20}};
	case H::Maquis:
		return {{RED_FLOWER, 35}, {YELLOW_FLOWER, 25}, {ALLIUM, 20}, {WHITE_FLOWER, 20}};
	case H::Steppe:
		return {{YELLOW_FLOWER, 40}, {ALLIUM, 30}, {RED_FLOWER, 30}};
	case H::Tundra:
		return {{WHITE_FLOWER, 40}, {YELLOW_FLOWER, 30}, {OXEYE_DAISY, 30}};
	case H::Wetland:
		return {{BLUE_FLOWER, 70}, {OXEYE_DAISY, 15}, {YELLOW_FLOWER, 15}};
	case H::Prairie:
		return {{YELLOW_FLOWER, 35}, {OXEYE_DAISY, 20}, {ALLIUM, 15}, {CORNFLOWER, 15},
				{RED_FLOWER, 10}, {ORANGE_TULIP, 5}};
	case H::Alpine:
		return {{WHITE_FLOWER, 30}, {CORNFLOWER, 20}, {ALLIUM, 20}, {OXEYE_DAISY, 15},
				{YELLOW_FLOWER, 15}};
	case H::Savanna:
		return {{YELLOW_FLOWER, 30}, {RED_FLOWER, 30}, {ALLIUM, 20}, {WHITE_FLOWER, 20}};
	case H::Desert:
		return {{YELLOW_FLOWER, 40}, {ALLIUM, 30}, {RED_FLOWER, 30}};
	}
	return {{YELLOW_FLOWER, 1}};
}

TallFlowerPalette tall_flower_palette(Habitat h)
{
	if (h == Habitat::Forest)
		return {{{LILAC_LOWER, LILAC_UPPER}, 35},
				{{ROSE_BUSH_LOWER, ROSE_BUSH_UPPER}, 35},
				{{PEONY_LOWER, PEONY_UPPER}, 30}};
	return {{{SUNFLOWER_LOWER, SUNFLOWER_UPPER}, 1}};
}

template <typename T>
T pick_weighted(const std::vector<std::pair<T, std::uint32_t>> &palette,
		std::uint32_t roll_permille)
{
	std::uint32_t total = 0;
	for (const auto &[_, weight] : palette)
		total += weight;
	std::uint32_t roll = roll_permille * total / 1000;
	for (const auto &[value, weight] : palette) {
		if (roll < weight)
			return value;
		roll -= weight;
	}
	return palette.back().first;
}

int div_euclid(int value, int divisor)
{
	const int quotient = value / divisor;
	return value % divisor < 0 ? quotient - 1 : quotient;
}

bool same_ground(Habitat a, Habitat b)
{
	return a == b || (a == Habitat::Meadow && b == Habitat::Alpine) ||
		   (a == Habitat::Alpine && b == Habitat::Meadow);
}

std::optional<WeightedFeature> choose_feature(
		const std::vector<WeightedFeature> &table, std::uint32_t roll)
{
	for (const auto &entry : table) {
		if (roll < entry.weight)
			return entry;
		roll -= entry.weight;
	}
	return std::nullopt;
}

std::pair<int, int> patch_shape(PatchFeature feature)
{
	switch (feature) {
	case PatchFeature::Flowers:
		return {48, 6};
	case PatchFeature::TallFlowers:
		return {24, 5};
	case PatchFeature::TallGrass:
		return {32, 5};
	case PatchFeature::Ferns:
		return {40, 6};
	case PatchFeature::Mushrooms:
		return {12, 3};
	case PatchFeature::SugarCane:
		return {20, 4};
	case PatchFeature::Pumpkins:
		return {8, 3};
	case PatchFeature::SweetBerries:
		return {16, 4};
	case PatchFeature::DeadBushes:
		return {6, 4};
	case PatchFeature::Cactus:
		return {6, 5};
	case PatchFeature::LilyPads:
		return {28, 6};
	case PatchFeature::Moss:
		return {32, 4};
	}
	return {0, 0};
}

void grow_patch(WorldEditor &editor, const Ground &ground, const ::XZBBox &bbox,
		const Args &args, int min_x, int max_x, int min_z, int max_z,
		double absolute_latitude, int alpine_from_y, bool cactus_country, ChaCha8Rng &rng,
		WeightedFeature selected, Habitat origin_habitat, int origin_x, int origin_z)
{
	auto [tries, spread] = patch_shape(selected.feature);
	if ((selected.feature == PatchFeature::Flowers ||
				selected.feature == PatchFeature::TallFlowers) &&
			rng.uniform(100) >= 12)
		tries = tries * 2 / 5;
	const auto flowers = flower_palette(origin_habitat);
	const auto tall_flowers = tall_flower_palette(origin_habitat);
	Block main_flower = GRASS;
	Block other_flower = GRASS;
	if (selected.feature == PatchFeature::Flowers) {
		auto drift = coord_rng(div_euclid(origin_x, 40), div_euclid(origin_z, 40),
				0x9A7C4E11D00D0002ULL);
		main_flower = pick_weighted(flowers, drift.uniform(1000));
		other_flower = pick_weighted(flowers, rng.uniform(1000));
	}

	const auto point_cover = [&](int x, int z) {
		return ground.cover_class({x - bbox.min_x(), z - bbox.min_z()});
	};
	const auto point_eco = [&](int x, int z) {
		return ground.ecoregion_at({x - bbox.min_x(), z - bbox.min_z()});
	};
	const auto point_habitat = [&](int x, int z) {
		const int terrain_y = ground.elevation_enabled
									  ? ground.level({x - bbox.min_x(), z - bbox.min_z()})
									  : args.ground_level;
		return habitat(point_cover(x, z), ground.climate(), absolute_latitude,
				terrain_y >= alpine_from_y, point_eco(x, z));
	};
	const std::vector<Block> soil{GRASS_BLOCK, PODZOL, DIRT, COARSE_DIRT, MOSS_BLOCK};
	const std::vector<Block> shade_soil{
			GRASS_BLOCK, PODZOL, DIRT, COARSE_DIRT, MOSS_BLOCK, MYCELIUM};
	const std::vector<Block> cane_soil{GRASS_BLOCK, DIRT, COARSE_DIRT, PODZOL, SAND, MUD};
	const std::vector<Block> dry_soil{SAND, COARSE_DIRT, DIRT, TERRACOTTA, RED_TERRACOTTA,
			ORANGE_TERRACOTTA, YELLOW_TERRACOTTA};
	const auto open = [&](int x, int y, int z) {
		return (!editor.block_exists_absolute(x, y + 1, z) ||
					   editor.check_for_block_absolute(
							   x, y + 1, z, std::vector<Block>{GRASS})) &&
			   !editor.check_for_block_absolute(x, y + 2, z,
					   std::vector<Block>{OAK_LOG, SPRUCE_LOG, BIRCH_LOG, DARK_OAK_LOG,
							   JUNGLE_LOG, ACACIA_LOG, CHERRY_LOG});
	};
	const auto plant_on = [&](int x, int y, int z, const std::vector<Block> &blocks,
								  const Block &plant) {
		if (editor.check_for_block_absolute(x, y, z, blocks) && open(x, y, z))
			editor.set_block_absolute(plant, x, y + 1, z,
					std::optional<std::vector<Block>>({GRASS}), std::nullopt);
	};
	const auto tall_on = [&](int x, int y, int z) {
		if (editor.check_for_block_absolute(x, y, z, soil) && open(x, y, z) &&
				!editor.block_exists_absolute(x, y + 2, z)) {
			editor.set_block_absolute(TALL_GRASS_BOTTOM, x, y + 1, z,
					std::optional<std::vector<Block>>({GRASS}), std::nullopt);
			editor.set_block_absolute(
					TALL_GRASS_TOP, x, y + 2, z, std::nullopt, std::nullopt);
		}
	};
	const auto beside_water = [&](int x, int y, int z) {
		return editor.check_for_block_type_absolute(x + 1, y, z, WATER) ||
			   editor.check_for_block_type_absolute(x - 1, y, z, WATER) ||
			   editor.check_for_block_type_absolute(x, y, z + 1, WATER) ||
			   editor.check_for_block_type_absolute(x, y, z - 1, WATER);
	};

	for (int i = 0; i < tries; ++i) {
		const int x =
				origin_x + int(rng.uniform(spread + 1)) - int(rng.uniform(spread + 1));
		const int z =
				origin_z + int(rng.uniform(spread + 1)) - int(rng.uniform(spread + 1));
		const auto pick = rng.uniform(100);
		const auto variant = rng.uniform(1000);
		(void)variant;
		if (x < min_x || x > max_x || z < min_z || z > max_z ||
				!ground.inside_rotation_mask(x, z) || editor.surface_is_sealed(x, z))
			continue;
		const auto current_habitat = point_habitat(x, z);
		const auto cover = point_cover(x, z);
		const auto eco = point_eco(x, z);
		const bool point_cactus_country =
				eco ? ecoregion::realm_is_americas(eco->realm) : cactus_country;
		const bool in_feature_habitat =
				selected.feature == PatchFeature::LilyPads
						? cover == land_cover::LC_WETLAND ||
								  cover == land_cover::LC_MANGROVES ||
								  cover == land_cover::LC_WATER
				: selected.feature == PatchFeature::SugarCane
						? current_habitat.has_value() || cover == land_cover::LC_BARE
						: current_habitat &&
								  same_ground(*current_habitat, origin_habitat);
		if (!in_feature_habitat ||
				(selected.feature == PatchFeature::Cactus && !point_cactus_country))
			continue;
		const int y = ground.elevation_enabled ? editor.get_ground_level(x, z)
											   : args.ground_level;
		switch (selected.feature) {
		case PatchFeature::Flowers:
			plant_on(x, y, z, soil, pick < 85 ? main_flower : other_flower);
			break;
		case PatchFeature::TallFlowers: {
			const auto [lower, upper] = pick_weighted(tall_flowers, variant);
			if (editor.check_for_block_absolute(x, y, z, soil) && open(x, y, z) &&
					!editor.block_exists_absolute(x, y + 2, z)) {
				editor.set_block_absolute(lower, x, y + 1, z,
						std::optional<std::vector<Block>>({GRASS}), std::nullopt);
				editor.set_block_absolute(upper, x, y + 2, z, std::nullopt, std::nullopt);
			}
			break;
		}
		case PatchFeature::TallGrass:
			tall_on(x, y, z);
			break;
		case PatchFeature::Ferns:
			if (pick < 30 && editor.check_for_block_absolute(x, y, z, soil) &&
					open(x, y, z) && !editor.block_exists_absolute(x, y + 2, z)) {
				editor.set_block_absolute(LARGE_FERN_LOWER, x, y + 1, z,
						std::optional<std::vector<Block>>({GRASS}), std::nullopt);
				editor.set_block_absolute(
						LARGE_FERN_UPPER, x, y + 2, z, std::nullopt, std::nullopt);
			} else {
				plant_on(x, y, z, soil, FERN);
			}
			break;
		case PatchFeature::Mushrooms:
			if (editor.highest_block_between(x, z, y + 3, y + 24))
				plant_on(x, y, z, shade_soil, pick < 70 ? BROWN_MUSHROOM : RED_MUSHROOM);
			break;
		case PatchFeature::SugarCane:
			if (beside_water(x, y, z) &&
					editor.check_for_block_absolute(x, y, z, cane_soil) &&
					open(x, y, z)) {
				const int height = 1 + int(pick % 3);
				for (int dy = 1; dy <= height; ++dy) {
					if (editor.block_exists_absolute(x, y + dy, z) &&
							!editor.check_for_block_absolute(
									x, y + dy, z, std::vector<Block>{GRASS}))
						break;
					editor.set_block_absolute(SUGAR_CANE, x, y + dy, z,
							std::optional<std::vector<Block>>({GRASS}), std::nullopt);
				}
			}
			break;
		case PatchFeature::Pumpkins:
			plant_on(x, y, z, std::vector<Block>{GRASS_BLOCK}, PUMPKIN);
			break;
		case PatchFeature::SweetBerries:
			plant_on(x, y, z, soil, SWEET_BERRY_BUSH);
			break;
		case PatchFeature::DeadBushes:
			plant_on(x, y, z, dry_soil, DEAD_BUSH);
			break;
		case PatchFeature::Cactus:
			if (editor.check_for_block_absolute(x, y, z, std::vector<Block>{SAND}) &&
					open(x, y, z)) {
				const int height = pick % 4 == 0 ? 1 : pick % 4 == 3 ? 3 : 2;
				for (int dy = 1; dy <= height; ++dy) {
					if ((dy > 1 && editor.block_exists_absolute(x, y + dy, z)) ||
							editor.block_exists_absolute(x + 1, y + dy, z) ||
							editor.block_exists_absolute(x - 1, y + dy, z) ||
							editor.block_exists_absolute(x, y + dy, z + 1) ||
							editor.block_exists_absolute(x, y + dy, z - 1))
						break;
					editor.set_block_absolute(CACTUS, x, y + dy, z,
							std::optional<std::vector<Block>>({GRASS}), std::nullopt);
				}
			}
			break;
		case PatchFeature::LilyPads:
			if ((point_cover(x, z) == land_cover::LC_WETLAND ||
						point_cover(x, z) == land_cover::LC_MANGROVES ||
						point_cover(x, z) == land_cover::LC_WATER) &&
					editor.check_for_block_type_absolute(x, y, z, WATER) &&
					!editor.block_exists_absolute(x, y + 1, z))
				editor.set_block_absolute(
						LILY_PAD, x, y + 1, z, std::nullopt, std::nullopt);
			break;
		case PatchFeature::Moss:
			plant_on(x, y, z, soil, MOSS_CARPET);
			break;
		}
	}
}
}

void place_scattered_flower(WorldEditor &editor, int x, int z, FlowerSetting setting)
{
	if (ground_generation::patch_noise(x, z, 11, 0x00C10A9Fu) < .6 ||
			editor.surface_is_sealed(x, z))
		return;
	const int y = editor.get_ground_level(x, z);
	if (editor.check_for_block_absolute(
				x, y, z, std::optional<std::vector<Block>>({GRASS_BLOCK, DIRT})))
		editor.set_block_absolute(flower_for(x, z, setting), x, y + 1, z,
				std::optional<std::vector<Block>>({GRASS}), std::nullopt);
}

void place_bed_flower(WorldEditor &editor, int x, int z)
{
	const auto h = land_cover::coord_hash(x ^ 0x0bed, z ^ 0x5eed);
	if (h % 100 < 12 || editor.surface_is_sealed(x, z))
		return;
	const int y = editor.get_ground_level(x, z);
	if (editor.check_for_block_absolute(x, y, z,
				std::optional<std::vector<Block>>({GRASS_BLOCK, DIRT, FARMLAND})))
		editor.set_block_absolute(flower_for(x, z, FlowerSetting::Garden, true), x, y + 1,
				z, std::optional<std::vector<Block>>({GRASS, FARMLAND}), std::nullopt);
}

void decorate_region(WorldEditor &editor, const Args &args, const ::XZBBox &bbox,
		int min_x, int max_x, int min_z, int max_z)
{
	if (!editor.ground || !editor.ground->has_land_cover() || !is_earth(args.body) ||
			min_x > max_x || min_z > max_z)
		return;
	constexpr int max_spread = 7;
	const int chunk_min_x = (min_x - max_spread) >> 4;
	const int chunk_max_x = (max_x + max_spread) >> 4;
	const int chunk_min_z = (min_z - max_spread) >> 4;
	const int chunk_max_z = (max_z + max_spread) >> 4;
	const auto geographic_bounds = editor.geographic_bounds();
	const double absolute_latitude =
			std::abs((geographic_bounds[0] + geographic_bounds[1]) * .5);
	const double center_longitude = (geographic_bounds[2] + geographic_bounds[3]) * .5;
	const int snow_y = editor.ground->snow_threshold();
	const int alpine_from_y =
			snow_y == std::numeric_limits<int>::max() ||
							snow_y == std::numeric_limits<int>::min()
					? snow_y
					: snow_y - static_cast<int>(std::lround(
									   1000.0 * editor.ground->blocks_per_meter()));
	for (int chunk_x = chunk_min_x; chunk_x <= chunk_max_x; ++chunk_x) {
		for (int chunk_z = chunk_min_z; chunk_z <= chunk_max_z; ++chunk_z) {
			for (std::uint64_t origin = 0; origin < 4; ++origin) {
				ChaCha8Rng rng = coord_rng(
						chunk_x, chunk_z, 0x9A7C4E11D00D0001ULL ^ (origin << 48));
				const int origin_x = (chunk_x << 4) + int(rng.uniform(16));
				const int origin_z = (chunk_z << 4) + int(rng.uniform(16));
				const auto origin_coord =
						XZPoint{origin_x - bbox.min_x(), origin_z - bbox.min_z()};
				const auto origin_eco = editor.ground->ecoregion_at(origin_coord);
				const int origin_y = editor.ground->elevation_enabled
											 ? editor.ground->level(origin_coord)
											 : args.ground_level;
				const auto habitat_origin =
						habitat(editor.ground->cover_class(origin_coord),
								editor.ground->climate(), absolute_latitude,
								origin_y >= alpine_from_y, origin_eco);
				if (!habitat_origin)
					continue;
				const auto selected = choose_feature(features(*habitat_origin),
						static_cast<std::uint32_t>(rng.uniform(1000)));
				if (!selected)
					continue;
				const bool cactus_country =
						origin_eco
								? ecoregion::realm_is_americas(origin_eco->realm)
								: center_longitude >= -170.0 && center_longitude <= -30.0;
				if (selected->feature == PatchFeature::Cactus && !cactus_country)
					continue;
				grow_patch(editor, *editor.ground, bbox, args, min_x, max_x, min_z, max_z,
						absolute_latitude, alpine_from_y, cactus_country, rng, *selected,
						*habitat_origin, origin_x, origin_z);
			}
		}
	}
}
} // namespace arnis::ground_decoration
