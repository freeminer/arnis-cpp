#include "ground_generation.h"

#include "block_definitions.h"
#include "element_processing/tree.h"
#include "land_cover/land_cover.h"
#include "trees/schematic.h"
#include "climate.h"
#include "celestial.h"
#include "deterministic_rng.h"
#include "world_editor/floor_state.h"
#include "terrain_surface.h"
#include "ecoregion.h"
#include "trees/mapped.h"
#include "biome.h"
#include "ground_decoration.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>
#include <map>
#include <vector>
#include <iostream>

namespace arnis::ground_generation
{

namespace
{

constexpr std::uint32_t SALT_FOREST_FLOOR = 0xF0E57F10u;
constexpr std::uint32_t SALT_SHRUB_FLOOR = 0x5B7BF10Au;
constexpr std::uint32_t SALT_SWARD = 0x5A7D0001u;
constexpr std::uint32_t SALT_TALL_SWARD = 0x5A7D0002u;
constexpr std::uint32_t SALT_WETLAND_POOLS = 0x90015E75u;

double climate_sward(const biome::Climate climate)
{
	switch (climate) {
	case biome::Climate::HotDesert:
		return .25;
	case biome::Climate::ColdDesert:
	case biome::Climate::IceCap:
		return .3;
	case biome::Climate::HotSteppe:
		return .55;
	case biome::Climate::ColdSteppe:
	case biome::Climate::Tundra:
		return .7;
	case biome::Climate::DryContinental:
		return .85;
	case biome::Climate::Boreal:
		return .9;
	case biome::Climate::Temperate:
	case biome::Climate::TropicalSavanna:
		return 1.0;
	}
	return 1.0;
}

bool undergrowth_roll(int x, int z, double mean, std::uint32_t salt)
{
	const double density = std::min(.95, mean * (.3 + 1.4 * patch_noise(x, z, 11, salt)));
	const auto roll = land_cover::coord_hash(x ^ static_cast<int>(salt),
							  z ^ static_cast<int>((salt << 9) | (salt >> 23))) %
					  1000;
	return static_cast<double>(roll) < density * 1000.0;
}

Block rocky_surface_for(int x, int z)
{
	const auto h = land_cover::coord_hash(x, z) % 12;
	if (h <= 4)
		return STONE;
	if (h <= 6)
		return ANDESITE;
	if (h <= 8)
		return COBBLESTONE;
	if (h == 9)
		return GRAVEL;
	if (h == 10)
		return TUFF;
	return COARSE_DIRT;
}

double value_noise_01_impl(int x, int z, int scale)
{
	const int s = std::max(1, scale);
	const auto floor_to = [s](int v) {
		return (v >= 0 ? v / s : -(((-v) + s - 1) / s)) * s;
	};
	const int x0 = floor_to(x), z0 = floor_to(z), x1 = x0 + s, z1 = z0 + s;
	const double tx = double(x - x0) / s, tz = double(z - z0) / s;
	const auto smooth = [](double t) { return t * t * (3.0 - 2.0 * t); };
	const double fx = smooth(tx), fz = smooth(tz);
	const auto sample = [](int sx, int sz) {
		return double(land_cover::coord_hash(sx, sz) % 1000) / 1000.0;
	};
	const double a = sample(x0, z0) * (1 - fx) + sample(x1, z0) * fx;
	const double b = sample(x0, z1) * (1 - fx) + sample(x1, z1) * fx;
	return a * (1 - fz) + b * fz;
}

std::pair<Block, Block> slope_palette(int slope, int x, int z)
{
	const auto h = land_cover::coord_hash(x, z);
	if (slope > 8)
		return h % 2 ? std::make_pair(DEEPSLATE, DEEPSLATE)
					 : std::make_pair(COBBLED_DEEPSLATE, COBBLED_DEEPSLATE);
	if (slope > 6) {
		const auto k = h % 20;
		return k < 12	? std::make_pair(STONE, DEEPSLATE)
			   : k < 17 ? std::make_pair(COBBLESTONE, DEEPSLATE)
						: std::make_pair(ANDESITE, DEEPSLATE);
	}
	if (slope > 4) {
		switch (h % 12) {
		case 0:
		case 1:
		case 2:
		case 3:
			return {ANDESITE, STONE};
		case 4:
		case 5:
			return {TUFF, STONE};
		case 6:
		case 7:
			return {STONE, STONE};
		case 8:
		case 9:
			return {COBBLESTONE, STONE};
		default:
			return {GRAVEL, STONE};
		}
	}
	return {GRASS_BLOCK, DIRT};
}

int local_slope(WorldEditor &editor, int x, int z)
{
	// Ground::slope is the Rust terrain metric: cardinal samples at a four
	// block step, corrected from compressed elevation space into the documented
	// world-scale threshold units.  Keep the small fallback for library callers
	// that construct an editor without a Ground object.
	if (editor.ground)
		return editor.ground->slope({x, z});
	const int center = editor.get_ground_level(x, z);
	int max_delta = 0;
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz) {
			if (dx == 0 && dz == 0)
				continue;
			max_delta = std::max(max_delta,
					std::abs(editor.get_ground_level(x + dx, z + dz) - center));
		}
	}
	return max_delta;
}

bool has_nearby_water(WorldEditor &editor, int x, int ground_y, int z)
{
	for (int dx = -2; dx <= 2; ++dx) {
		for (int dz = -2; dz <= 2; ++dz) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (editor.check_for_block_type_absolute(
							x + dx, ground_y + dy, z + dz, WATER)) {
					return true;
				}
			}
		}
	}
	return false;
}

bool is_protected_surface(WorldEditor &editor, int x, int y, int z)
{
	return editor.check_for_block_absolute(x, y, z,
			std::optional<std::vector<Block>>(std::vector<Block>{
					BLACK_CONCRETE,
					GRAY_CONCRETE_POWDER,
					CYAN_TERRACOTTA,
					GRAY_CONCRETE,
					LIGHT_GRAY_CONCRETE,
					WHITE_CONCRETE,
					DIRT_PATH,
					SMOOTH_STONE,
					WATER,
			}));
}

bool is_replaceable_surface(WorldEditor &editor, int x, int y, int z)
{
	// Match the Rust natural-surface palette while protecting authored OSM blocks.
	return !editor.block_exists_absolute(x, y, z) ||
		   editor.check_for_block_absolute(x, y, z,
				   std::optional<std::vector<Block>>(std::vector<Block>{
						   STONE,
						   DIRT,
						   GRASS_BLOCK,
						   GRASS,
						   SAND,
						   SANDSTONE,
						   GRAVEL,
						   CLAY,
						   COARSE_DIRT,
						   PODZOL,
						   MUD,
						   ANDESITE,
						   COBBLESTONE,
						   TUFF,
						   DEEPSLATE,
						   COBBLED_DEEPSLATE,
						   MOSS_BLOCK,
						   SNOW_BLOCK,
						   ICE,
						   PACKED_ICE,
						   BLACKSTONE,
				   }));
}

Block natural_surface_for(WorldEditor &editor, int x, int ground_y, int z)
{
	const auto cover = editor.ground
							   ? editor.ground->cover_class({x - editor.mg->node_min.X,
										 z - editor.mg->node_min.Z})
							   : 0;
	const int slope = local_slope(editor, x, z);
	if (slope > 4)
		return terrain_surface::steep_palette(x, z, ground_y, slope, cover).first;
	if (cover == land_cover::LC_BEACH)
		return SAND;
	if (editor.ground) {
		auto climate_palette =
				climate::surface_palette(editor.ground->climate(), cover, x, z);
		if (climate_palette)
			return climate_palette->first;
	}
	if (cover == land_cover::LC_CROPLAND)
		return FARMLAND;
	if (cover == land_cover::LC_BUILT_UP) {
		const auto h = land_cover::coord_hash(x, z) % 100;
		return h < 72	? STONE_BRICKS
			   : h < 87 ? CRACKED_STONE_BRICKS
			   : h < 92 ? STONE
						: COBBLESTONE;
	}
	if (cover == land_cover::LC_BARE || cover == land_cover::LC_BEACH ||
			cover == land_cover::LC_SNOW_ICE) {
		int nearby = 0;
		if (editor.ground)
			for (auto [dx, dz] :
					std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
				auto c = editor.ground->cover_class(
						{x + dx - editor.mg->node_min.X, z + dz - editor.mg->node_min.Z});
				nearby += c == land_cover::LC_BARE || c == land_cover::LC_BEACH ||
						  c == land_cover::LC_SNOW_ICE;
			}
		if (!nearby)
			return GRASS_BLOCK;
		const auto h = land_cover::coord_hash(x, z);
		if (patch_noise(x, z, 6, 0xBA4E40C3u) < .45)
			return h % 10 < 8 ? COARSE_DIRT : STONE;
		return rocky_surface_for(x, z);
	}
	if (cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES)
		return MUD;
	if (cover == land_cover::LC_SHRUBLAND)
		return patch_noise(x, z, 5, 0x5EED0A11u) < .4 && land_cover::coord_hash(x, z) % 5
					   ? COARSE_DIRT
					   : GRASS_BLOCK;
	if (has_nearby_water(editor, x, ground_y, z))
		return SAND;

	const auto h = land_cover::coord_hash(x, z) % 100;
	if (h < 2)
		return COARSE_DIRT;
	if (h < 5)
		return PODZOL;
	return GRASS_BLOCK;
}

std::optional<bool> canopy_tree_verdict(
		WorldEditor &editor, int x, int z, int origin_x, int origin_z)
{
	if (!editor.ground || !editor.ground->has_canopy())
		return std::nullopt;
	const int spacing = std::max(1, editor.get_tree_slot_spacing());
	const auto floor_div = [spacing](int v) {
		return v >= 0 ? v / spacing : -(((-v) + spacing - 1) / spacing);
	};
	const int cell_x = floor_div(x) * spacing;
	const int cell_z = floor_div(z) * spacing;
	const auto fraction = editor.ground->canopy_fraction(
			XZPoint{cell_x - origin_x, cell_z - origin_z}, spacing);
	if (!fraction)
		return std::nullopt;
	const auto [slot_x, slot_z] = trees::trunk_slot_s(x, z, spacing);
	if (x != slot_x || z != slot_z)
		return false;
	const double p =
			canopy::slot_probability(*fraction, spacing, editor.place_schematics());
	const double roll =
			double(land_cover::coord_hash(x ^ 0x434d, z ^ 0x484d) % 10000) / 10000.0;
	return roll < p;
}

void maybe_place_vegetation(WorldEditor &editor, int x, int ground_y, int z,
		const BuildingFootprintBitmap &building_footprints, int origin_x, int origin_z,
		const bridges::BridgeSurfaceMap *bridge_surface, double scale,
		double absolute_latitude, int alpine_from_y)
{
	if (editor.surface_is_sealed(x, z) || building_footprints.contains(x, z) ||
			(bridge_surface && bridge_surface->contains(x, z)) ||
			(editor.mapped_trunks && editor.mapped_trunks->under_crown(x, z)) ||
			editor.check_for_block_absolute(x, ground_y + 1, z))
		return;
	if (!editor.check_for_block_absolute(x, ground_y, z,
				std::optional<std::vector<Block>>(std::vector<Block>{GRASS_BLOCK, PODZOL,
						COARSE_DIRT, DIRT, MUD, MYCELIUM, FARMLAND})))
		return;

	const auto cover = editor.ground
							   ? editor.ground->cover_class({x - editor.mg->node_min.X,
										 z - editor.mg->node_min.Z})
							   : 0;
	auto rng = coord_rng(x, z, 0);
	const int patch_x = x >= 0 ? x / 8 : -(((-x) + 7) / 8);
	const int patch_z = z >= 0 ? z / 8 : -(((-z) + 7) / 8);
	const auto patch_roll = land_cover::coord_hash(patch_x, patch_z) % 100;
	const auto eco = editor.ground
							 ? editor.ground->ecoregion_at({x - editor.mg->node_min.X,
									   z - editor.mg->node_min.Z})
							 : std::nullopt;
	const auto climate =
			editor.ground ? editor.ground->climate() : biome::Climate::Temperate;
	const auto site_habitat = ground_decoration::habitat(
			cover, climate, absolute_latitude, ground_y >= alpine_from_y, eco);
	const double sward =
			climate_sward(climate) *
			(editor.check_for_block_type_absolute(x, ground_y, z, COARSE_DIRT) ? .5
																			   : 1.0);
	// Rust's decoration pass only installs a plant when the space above the
	// finished surface is still open.  Keep this invariant for all lightweight
	// vegetation choices below; structures and schematics remain authoritative.
	const auto place_decoration = [&](const Block &block, int y) {
		if (!editor.block_exists_absolute(x, y, z))
			editor.set_block_if_absent_absolute(block, x, y, z);
	};
	if (cover == land_cover::LC_MANGROVES && eco &&
			eco->biome == ecoregion::EcoBiome::Mangroves && rng.uniform(4) == 0) {
		// Rust's ecoregion decorator gives mapped mangrove habitat priority
		// over the generic canopy fallback.
		Tree::create_of_type(editor, Coord{x, 1, z}, TreeType::Mangrove,
				&building_footprints, bridge_surface, true);
	} else if (cover == land_cover::LC_TREE_COVER) {
		// Measured canopy owns density where available.  On no-data cells retain
		// the old land-cover-only probability, matching Rust's fallback contract.
		const auto canopy_wants_tree =
				canopy_tree_verdict(editor, x, z, origin_x, origin_z);
		constexpr double micro_tree_max_scale = 0.35;
		const auto tree_rate = scale < micro_tree_max_scale ? 4u : 30u;
		const auto choice = rng.uniform(tree_rate);
		if (canopy_wants_tree.value_or(choice == 0)) {
			bool placed = false;
			if (eco) {
				switch (eco->biome) {
				case ecoregion::EcoBiome::TemperateConifer:
				case ecoregion::EcoBiome::Boreal:
					Tree::create_of_type(editor, Coord{x, 1, z}, TreeType::Spruce,
							&building_footprints, bridge_surface);
					placed = true;
					break;
				case ecoregion::EcoBiome::DryTropical:
					Tree::create_of_type(editor, Coord{x, 1, z}, TreeType::Acacia,
							&building_footprints, bridge_surface);
					placed = true;
					break;
				case ecoregion::EcoBiome::MoistTropical:
					Tree::create_of_type(editor, Coord{x, 1, z}, TreeType::Jungle,
							&building_footprints, bridge_surface);
					placed = true;
					break;
				case ecoregion::EcoBiome::TropicalGrassland:
				case ecoregion::EcoBiome::Mediterranean:
					Tree::create_of_type(editor, Coord{x, 1, z}, TreeType::Oak,
							&building_footprints, bridge_surface);
					placed = true;
					break;
				default:
					break;
				}
			}
			if (!placed && !editor.place_regional_tree(x, ground_y + 1, z, cover))
				Tree::create(
						editor, Coord{x, 1, z}, &building_footprints, bridge_surface);
		} else if (choice == 1 && patch_roll < 72) {
			// Rust's habitat tables bias prairie and Mediterranean patches toward
			// their characteristic flowers instead of using a global palette.
			const auto flower_choice =
					eco && eco->biome == ecoregion::EcoBiome::TemperateGrassland ? 2u
					: eco && eco->biome == ecoregion::EcoBiome::Mediterranean
							? 0u
							: rng.uniform(4);
			const Block flower = flower_choice == 0	  ? RED_FLOWER
								 : flower_choice == 1 ? BLUE_FLOWER
								 : flower_choice == 2 ? YELLOW_FLOWER
													  : WHITE_FLOWER;
			place_decoration(flower, ground_y + 1);
		} else if (choice == 2 && eco &&
				   (eco->biome == ecoregion::EcoBiome::Boreal ||
						   eco->biome == ecoregion::EcoBiome::TemperateConifer) &&
				   editor.highest_block_between(x, z, ground_y + 3, ground_y + 24)) {
			place_decoration(
					(patch_roll & 1) ? RED_MUSHROOM : BROWN_MUSHROOM, ground_y + 1);
		} else if (undergrowth_roll(x, z, .4 * sward, SALT_FOREST_FLOOR)) {
			const double fern_share =
					site_habitat == ground_decoration::Habitat::Taiga	 ? .45
					: site_habitat == ground_decoration::Habitat::Jungle ? .3
																		 : .12;
			const double fern_roll =
					double(land_cover::coord_hash(x ^ 0xfe, z ^ 0x4e) % 100) / 100.0;
			place_decoration(fern_roll < fern_share ? FERN : GRASS, ground_y + 1);
		}
	} else if (cover == land_cover::LC_CROPLAND) {
		const bool farmland =
				editor.check_for_block_type_absolute(x, ground_y, z, FARMLAND);
		const bool enclosed = editor.get_ground_level(x + 1, z) >= ground_y &&
							  editor.get_ground_level(x - 1, z) >= ground_y &&
							  editor.get_ground_level(x, z + 1) >= ground_y &&
							  editor.get_ground_level(x, z - 1) >= ground_y;
		if (farmland && x % 9 == 0 && z % 9 == 0 && enclosed)
			editor.set_block_absolute(WATER, x, ground_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{FARMLAND}),
					std::nullopt);
		else if (farmland && rng.uniform(76) == 0 && rng.uniform(10) < 4)
			place_decoration(HAY_BALE, ground_y + 1);
		else if (farmland && rng.uniform(40) == 0)
			place_decoration(PUMPKIN, ground_y + 1);
		else if (farmland) {
			const auto crop_choice = rng.uniform(3);
			const Block crop = crop_choice == 0	  ? WHEAT
							   : crop_choice == 1 ? CARROTS
												  : POTATOES;
			place_decoration(crop, ground_y + 1);
		}
	} else if (cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES) {
		const auto choice = rng.uniform(100);
		const bool water_surface =
				editor.check_for_block_type_absolute(x, ground_y, z, WATER);
		if (water_surface && choice < 8) {
			const bool kelp = choice == 0;
			if (kelp) {
				const int height = 2 + int(rng.uniform(3));
				for (int dy = 1; dy <= height; ++dy)
					place_decoration(KELP_PLANT, ground_y + dy);
			} else {
				place_decoration(
						choice < 3 ? TALL_SEAGRASS_BOTTOM : SEAGRASS, ground_y + 1);
			}
			if (!kelp && choice < 3)
				place_decoration(TALL_SEAGRASS_TOP, ground_y + 2);
		} else if (choice < 12 && has_nearby_water(editor, x, ground_y, z)) {
			// Rust's sugar-cane feature is a short stack next to water.  Keep
			// every segment conditional so existing structures remain untouched.
			const int height = 1 + int(rng.uniform(3));
			for (int dy = 1; dy <= height; ++dy)
				place_decoration(SUGAR_CANE, ground_y + dy);
		} else if (choice < 22 && water_surface) {
			place_decoration(LILY_PAD, ground_y + 1);
		} else if (!water_surface && patch_noise(x, z, 5, SALT_WETLAND_POOLS) < .28 &&
				   editor.water_source_is_enclosed(x, z))
			editor.set_block_absolute(WATER, x, ground_y, z,
					std::optional<std::vector<Block>>(
							std::vector<Block>{MUD, GRASS_BLOCK}),
					std::nullopt);
		else if (choice < 50)
			place_decoration(GRASS, ground_y + 1);
		else if (choice < 64) {
			place_decoration(TALL_GRASS_BOTTOM, ground_y + 1);
			place_decoration(TALL_GRASS_TOP, ground_y + 2);
		} else if (choice < 80)
			place_decoration(MOSS_CARPET, ground_y + 1);
	} else if (cover == land_cover::LC_MOSS) {
		// Rust treats moss cover as its own tundra habitat rather than falling
		// through to bare-rock decoration.
		const auto choice = rng.uniform(100);
		if (choice < 55)
			place_decoration(MOSS_CARPET, ground_y + 1);
		else if (choice < 68)
			place_decoration(FERN, ground_y + 1);
	} else if (cover == land_cover::LC_BARE) {
		const bool coarse =
				editor.check_for_block_type_absolute(x, ground_y, z, COARSE_DIRT);
		const auto choice = rng.uniform(100);
		const bool sandy = editor.check_for_block_type_absolute(x, ground_y, z, SAND);
		bool cactus_country = true;
		if (editor.mg) {
			const auto [lat, lon] = editor.mg->pos_to_ll(x, z);
			(void)lat;
			cactus_country = lon >= -170.0 && lon <= -30.0;
		}
		if (eco)
			cactus_country = ecoregion::realm_is_americas(eco->realm);
		if (sandy && cactus_country && choice < 4) {
			const int height = 1 + int(rng.uniform(3));
			for (int dy = 1; dy <= height; ++dy)
				place_decoration(CACTUS, ground_y + dy);
		} else if (coarse && choice < 6)
			place_decoration(GRASS, ground_y + 1);
		else if (coarse && choice < 9)
			place_decoration(OAK_LEAVES, ground_y + 1);
		else if ((!coarse && choice == 0) || (coarse && choice == 9))
			place_decoration(DEAD_BUSH, ground_y + 1);
	} else if (cover == land_cover::LC_SHRUBLAND) {
		if (rng.uniform(100) < 2) {
			if (eco && eco->biome == ecoregion::EcoBiome::Tundra)
				place_decoration(MOSS_CARPET, ground_y + 1);
			else if (eco && (eco->biome == ecoregion::EcoBiome::TemperateGrassland ||
									eco->biome == ecoregion::EcoBiome::Boreal))
				place_decoration(SWEET_BERRY_BUSH, ground_y + 1);
			else
				place_decoration(OAK_LEAVES, ground_y + 1);
		} else if (undergrowth_roll(x, z, .28 * sward, SALT_SHRUB_FLOOR)) {
			place_decoration(GRASS, ground_y + 1);
		}
	} else if (cover == land_cover::LC_GRASSLAND) {
		if (undergrowth_roll(x, z, .55 * sward, SALT_SWARD)) {
			const bool stand = patch_noise(x, z, 9, SALT_TALL_SWARD) > .8;
			const auto tall_share = stand ? 35u : 4u;
			if (rng.uniform(100) < tall_share) {
				place_decoration(TALL_GRASS_BOTTOM, ground_y + 1);
				place_decoration(TALL_GRASS_TOP, ground_y + 2);
			} else {
				place_decoration(GRASS, ground_y + 1);
			}
		} else if (patch_roll < 82 && rng.uniform(100) < 6) {
			const auto flower_choice =
					eco && eco->biome == ecoregion::EcoBiome::TemperateGrassland ? 2u
					: eco && eco->biome == ecoregion::EcoBiome::Mediterranean
							? 0u
							: rng.uniform(4);
			const Block flower = flower_choice == 0	  ? RED_FLOWER
								 : flower_choice == 1 ? BLUE_FLOWER
								 : flower_choice == 2 ? YELLOW_FLOWER
													  : WHITE_FLOWER;
			place_decoration(flower, ground_y + 1);
		} else if (patch_roll < 90 && rng.uniform(100) < 5) {
			// Rust's meadow/taiga habitat table reserves a small share of
			// grassland patches for ferns.  Keep the upper half conditional so
			// an existing canopy or authored block is never overwritten.
			if (rng.uniform(100) < 20 &&
					editor.highest_block_between(x, z, ground_y + 3, ground_y + 24)) {
				place_decoration(LARGE_FERN_LOWER, ground_y + 1);
				place_decoration(LARGE_FERN_UPPER, ground_y + 2);
			} else
				place_decoration(FERN, ground_y + 1);
		}
	}
}

void clear_road_vegetation(WorldEditor &editor, int x, int y, int z)
{
	const std::vector<Block> stray_surface{BLACK_CONCRETE, GRAY_CONCRETE_POWDER,
			CYAN_TERRACOTTA, GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE,
			DIRT_PATH, WATER};
	const std::vector<Block> loose_plants{GRASS, TALL_GRASS_BOTTOM, TALL_GRASS_TOP, FERN,
			LARGE_FERN_LOWER, LARGE_FERN_UPPER, DEAD_BUSH, RED_FLOWER, YELLOW_FLOWER,
			BLUE_FLOWER, WHITE_FLOWER, SWEET_BERRY_BUSH, BROWN_MUSHROOM, RED_MUSHROOM,
			MOSS_CARPET, SUGAR_CANE, PUMPKIN, CACTUS};
	const std::vector<Block> wood{OAK_LOG, SPRUCE_LOG, BIRCH_LOG, DARK_OAK_LOG,
			JUNGLE_LOG, ACACIA_LOG, CHERRY_LOG};
	const std::vector<Block> stacked{
			TALL_GRASS_TOP, LARGE_FERN_UPPER, SUGAR_CANE, CACTUS};
	const bool surface = editor.check_for_block_absolute(x, y, z, stray_surface);
	const bool under_wood = editor.check_for_block_absolute(x, y + 2, z, wood);
	if (!editor.check_for_block_absolute(x, y + 1, z, loose_plants) ||
			(!surface && !under_wood))
		return;
	editor.set_block_absolute(AIR, x, y + 1, z, loose_plants, std::nullopt);
	for (int yy = y + 2; yy <= y + 3; ++yy) {
		if (!editor.check_for_block_absolute(x, yy, z, stacked))
			break;
		editor.set_block_absolute(AIR, x, yy, z, stacked, std::nullopt);
	}
}

}

double value_noise_01(int x, int z, int scale)
{
	return value_noise_01_impl(x, z, scale);
}

double patch_noise(int x, int z, int scale, std::uint32_t salt)
{
	constexpr double c = 0.891006524188368, s = 0.453990499739547;
	const double u = (x * c - z * s) / std::max(1, scale);
	const double v = (x * s + z * c) / std::max(1, scale);
	const int u0 = static_cast<int>(std::floor(u)), v0 = static_cast<int>(std::floor(v));
	const double tu = u - u0, tv = v - v0;
	const double su = tu * tu * (3.0 - 2.0 * tu), sv = tv * tv * (3.0 - 2.0 * tv);
	const int sx = static_cast<int>(salt),
			  sz = static_cast<int>((salt << 16) | (salt >> 16));
	const auto sample = [&](int a, int b) {
		return double(land_cover::coord_hash(a ^ sx, b ^ sz) % 1000) / 1000.0;
	};
	const double a = sample(u0, v0) * (1.0 - su) + sample(u0 + 1, v0) * su;
	const double b = sample(u0, v0 + 1) * (1.0 - su) + sample(u0 + 1, v0 + 1) * su;
	const double value = a * (1.0 - sv) + b * sv;
	constexpr std::array<std::pair<double, double>, 13> quantiles{
			{{0.0, 0.0}, {0.15, 0.05}, {0.21, 0.1}, {0.30, 0.2}, {0.37, 0.3},
					{0.435, 0.4}, {0.496, 0.5}, {0.558, 0.6}, {0.625, 0.7}, {0.702, 0.8},
					{0.796, 0.9}, {0.858, 0.95}, {1.0, 1.0}}};
	for (std::size_t i = 1; i < quantiles.size(); ++i)
		if (value <= quantiles[i].first) {
			const auto [x0, y0] = quantiles[i - 1];
			const auto [x1, y1] = quantiles[i];
			return y0 + (value - x0) * (y1 - y0) / std::max(1e-12, x1 - x0);
		}
	return 1.0;
}

void generate_ground_region(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints, int iter_min_x,
		int iter_max_x, int iter_min_z, int iter_max_z,
		const CoordinateBitmap *tunnel_footprint,
		const bridges::BridgeSurfaceMap *bridge_surface, bool show_progress)
{
	// Rust parity: src/ground_generation.rs::generate_ground_layer ordering.
	// xzbbox remains the shared-grid origin; callers may supply strict tile
	// bounds so streamed passes sample the same land-cover/canopy cells.
	const int min_x = std::max(iter_min_x, xzbbox.min_x());
	const int max_x = std::min(iter_max_x, xzbbox.max_x());
	const int min_z = std::max(iter_min_z, xzbbox.min_z());
	const int max_z = std::min(iter_max_z, xzbbox.max_z());
	if (min_x > max_x || min_z > max_z)
		return;
	if (show_progress)
		std::cout << "[6/7] Generating ground...\n";
	const bool terrain_enabled = generation_mode_terrain(args.mode);
	const auto geographic_bounds = editor.geographic_bounds();
	const double center_latitude = (geographic_bounds[0] + geographic_bounds[1]) * .5;
	const terrain_surface::SnowLine snow_line(editor.ground
													  ? editor.ground->snow_threshold()
													  : std::numeric_limits<int>::max(),
			editor.ground ? editor.ground->blocks_per_meter() : 0.0, center_latitude,
			args.rotation);
	const int snow_threshold = editor.ground ? editor.ground->snow_threshold()
											 : std::numeric_limits<int>::max();
	const int alpine_from_y =
			snow_threshold == std::numeric_limits<int>::max() ||
							snow_threshold == std::numeric_limits<int>::min()
					? snow_threshold
					: snow_threshold -
							  static_cast<int>(std::lround(
									  1000.0 * editor.ground->blocks_per_meter()));
	std::map<std::pair<int, int>, std::optional<terrain_surface::TalusField>>
			talus_fields;
	for (int x = min_x; x <= max_x; ++x) {
		for (int z = min_z; z <= max_z; ++z) {
			// Rotation expands the output AABB. Rust masks columns whose inverse
			// rotated coordinate lies outside the original source bbox; otherwise
			// the expanded corners receive fabricated terrain and vegetation.
			if (editor.ground && !editor.ground->inside_rotation_mask(x, z))
				continue;
			const bool in_tunnel = tunnel_footprint && tunnel_footprint->contains(x, z);
			// Rust's geo-only mode deliberately bypasses elevation and uses the
			// configured flat level.  Keep all downstream surface/water decisions
			// on that same height rather than merely disabling the provider fetch.
			const int ground_y =
					terrain_enabled ? editor.get_ground_level(x, z) : args.ground_level;
			const int slope = terrain_enabled ? local_slope(editor, x, z) : 0;
			// Rust treats slopes above four as an explicit rock override.  This
			// deliberately bypasses authored surface blocks (quarry/park/etc.) so
			// steep faces cannot retain grass or paving, while still allowing the
			// lower tiers of the slope palette to choose scree materials.
			const bool steep_override = terrain_enabled && slope > 4;
			const auto relative = XZPoint{x - xzbbox.min_x(), z - xzbbox.min_z()};
			const bool planetary = !is_earth(args.body);
			const bool has_cover =
					!planetary && editor.ground && editor.ground->has_land_cover();
			const std::uint8_t cover_here =
					has_cover ? editor.ground->cover_class(relative) : 0;
			std::optional<std::pair<Block, Block>> talus_block;
			if (terrain_enabled && !planetary && editor.ground && slope <= 6 &&
					terrain_surface::takes_talus(cover_here)) {
				const std::pair<int, int> chunk{x >> 4, z >> 4};
				auto [it, inserted] = talus_fields.try_emplace(chunk);
				if (inserted)
					it->second = terrain_surface::TalusField::build(*editor.ground,
							chunk.first, chunk.second, xzbbox.min_x(), xzbbox.min_z());
				if (it->second)
					talus_block = terrain_surface::talus_palette(x, z,
							it->second->near(x, z, editor.ground->level_exact(relative)),
							cover_here);
			}
			const double water_blend =
					has_cover ? editor.ground->water_blend(relative) : 0.0;
			const bool grid_water =
					has_cover && editor.ground->water_distance(relative) > 0;
			// Probe each column at its own terrain level. Using the outer
			// ground_y for neighbours misses OSM water on sloped terrain.
			auto has_water_in_column = [&](int wx, int wz) {
				const int neighbour_ground = terrain_enabled
													 ? editor.get_ground_level(wx, wz)
													 : args.ground_level;
				for (int dy = 0; dy <= 2; ++dy)
					if (editor.check_for_block_type_absolute(
								wx, neighbour_ground + dy, wz, WATER))
						return true;
				return false;
			};
			const bool existing_water = has_water_in_column(x, z);
			const bool osm_gap =
					!existing_water &&
					((has_water_in_column(x, z - 1) && has_water_in_column(x, z + 1)) ||
							(has_water_in_column(x - 1, z) &&
									has_water_in_column(x + 1, z)) ||
							(has_water_in_column(x + 1, z - 1) &&
									has_water_in_column(x - 1, z + 1)) ||
							(has_water_in_column(x - 1, z - 1) &&
									has_water_in_column(x + 1, z + 1)));
			const bool has_existing_stone =
					editor.check_for_block_type_absolute(x, ground_y, z, STONE);
			const bool process_surface = steep_override || !has_existing_stone;
			std::optional<Block> column_under;

			// Rust uses a smoothed ESA water mask, but never retracts a hard water
			// cell.  Keep OSM water too, and avoid flooding cliff faces.
			if (process_surface && !planetary && !in_tunnel && !steep_override &&
					(grid_water || existing_water || osm_gap || water_blend > .5) &&
					slope <= 4) {
				const int water_y = editor.get_water_level(x, z);
				if (ground_y <= water_y && !is_protected_surface(editor, x, water_y, z)) {
					editor.set_block_absolute(
							WATER, x, water_y, z, std::nullopt, std::nullopt);
					if (water_y - 1 > world_editor::min_y())
						editor.set_block_absolute(
								SAND, x, water_y - 1, z, std::nullopt, std::nullopt);
					if (water_y - 2 > world_editor::min_y())
						editor.set_block_absolute(
								SANDSTONE, x, water_y - 2, z, std::nullopt, std::nullopt);
					// Match Rust: only skip normal ground generation when this
					// column was actually converted to water.  A classified cell
					// above its water surface must remain terrain, otherwise the
					// later water/depth pass leaves false flooded areas.
					continue;
				}
			}
			// On a steep DEM edge a snapped water level may sit below the
			// current column.  Rust keeps an interior water cell in that case;
			// otherwise the ground pass paints a grass/rock line through the
			// middle of a lake and the later depth pass cannot restore it.
			if (process_surface && !planetary && !in_tunnel && steep_override &&
					grid_water && editor.ground->is_interior_water(relative)) {
				const int water_y = ground_y;
				if (!is_protected_surface(editor, x, water_y, z)) {
					editor.set_block_absolute(
							WATER, x, water_y, z, std::nullopt, std::nullopt);
					continue;
				}
			}

			if (process_surface && !in_tunnel &&
					!is_protected_surface(editor, x, ground_y, z) &&
					(steep_override || is_replaceable_surface(editor, x, ground_y, z))) {
				const auto planetary_palette =
						planetary
								? celestial_surface_palette(args.body, slope,
										  args.celestial_latitude_degrees, ground_y, x, z)
								: std::pair<Block, Block>{};
				Block surface = planetary ? planetary_palette.first
										  : (talus_block ? talus_block->first
														 : natural_surface_for(editor, x,
																   ground_y, z));
				std::optional<Block> climate_under;
				if (planetary)
					climate_under = planetary_palette.second;
				else if (talus_block)
					climate_under = talus_block->second;
				else if (steep_override)
					climate_under = terrain_surface::steep_palette(
							x, z, ground_y, slope, cover_here)
											.second;
				else if (has_cover && editor.ground) {
					auto palette = climate::surface_palette(editor.ground->climate(),
							editor.ground->cover_class(relative), x, z);
					if (palette)
						climate_under = palette->second;
				}
				// Rust shoreline parity: blend the immediate ring around ESA or
				// already-rendered OSM water to sand on gentle terrain. This keeps
				// water boundaries from exposing abrupt grass/clay edges.
				if (!planetary && surface != WATER && slope <= 3) {
					bool near_esa_water = false;
					if (has_cover) {
						for (int dz = -1; dz <= 1 && !near_esa_water; ++dz)
							for (int dx = -1; dx <= 1; ++dx)
								if ((dx || dz) &&
										editor.ground->cover_class(
												{x + dx - xzbbox.min_x(),
														z + dz - xzbbox.min_z()}) ==
												land_cover::LC_WATER) {
									near_esa_water = true;
									break;
								}
					}
					bool near_placed_water = false;
					for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
								 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}})
						if (editor.check_for_block_type_absolute(
									x + dx, ground_y, z + dz, WATER)) {
							near_placed_water = true;
							break;
						}
					if (near_esa_water || near_placed_water) {
						surface = SAND;
						climate_under = SANDSTONE;
					}
				}
				if (surface == SAND && !climate_under)
					climate_under = SANDSTONE;
				else if (!climate_under &&
						 (surface == STONE || surface == ANDESITE ||
								 surface == COBBLESTONE || surface == TUFF))
					climate_under = STONE;
				else if (!climate_under)
					climate_under = DIRT;
				column_under = climate_under;
				if (steep_override) {
					// Match Rust's steep-face blacklist: roads, structures, bedrock,
					// and water remain authored even when terrain rock is forced.
					editor.set_block_absolute(surface, x, ground_y, z, std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{WATER,
									BEDROCK, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA,
									GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE,
									DIRT_PATH, STONE_BRICKS, BRICK, OAK_PLANKS,
									BLACK_CONCRETE}));
				} else if (talus_block && surface == talus_block->first) {
					editor.set_block_absolute(surface, x, ground_y, z,
							std::optional<std::vector<Block>>(
									terrain_surface::talus_buries()),
							std::nullopt);
				} else {
					editor.set_block_absolute(
							surface, x, ground_y, z, std::nullopt, std::nullopt);
				}
				const bool surface_is_water =
						editor.check_for_block_type_absolute(x, ground_y, z, WATER);
				auto set_under_if_absent = [&](Block block) {
					if (!editor.check_for_block_absolute(x, ground_y - 1, z))
						editor.set_block_absolute(
								block, x, ground_y - 1, z, std::nullopt, std::nullopt);
				};

				// Rust leaves authored/placed water untouched: under-materials must
				// not be written below a water surface and expose through shallow
				// rivers or lakes.
				if (!surface_is_water && climate_under) {
					set_under_if_absent(*climate_under);
				}
			}

			if (!planetary && !in_tunnel)
				maybe_place_vegetation(editor, x, ground_y, z, building_footprints,
						xzbbox.min_x(), xzbbox.min_z(), bridge_surface, args.scale,
						std::abs(center_latitude), alpine_from_y);

			// Rust's universal depth pass closes visible gaps below all terrain
			// columns, including ones whose surface was supplied by OSM.
			if (!in_tunnel &&
					!editor.check_for_block_type_absolute(x, ground_y, z, WATER)) {
				int lowest = ground_y;
				for (int dx = -1; dx <= 1; ++dx)
					for (int dz = -1; dz <= 1; ++dz)
						if (dx || dz)
							lowest = std::min(lowest,
									terrain_enabled
											? editor.get_ground_level(x + dx, z + dz)
											: args.ground_level);
				const int depth = std::clamp(ground_y - lowest + 1, 2, 64);
				const int fill_min =
						std::max(world_editor::terrain_floor_y() + 1, ground_y - depth);
				const int fill_max = ground_y - 1;
				const Block under = column_under.value_or(STONE);
				if (terrain_enabled && !planetary && steep_override && under == STONE)
					terrain_surface::fill_strata(
							editor, x, z, fill_min, fill_max, slope > 6);
				else
					editor.fill_column_absolute(under, x, z, fill_min, fill_max, true);
			}

			// Snow is a separate cap, so climate/land-cover material selection is
			// preserved below it just as in the Rust ground pass.
			// Match Rust's softened climatic snow edge.  A small deterministic
			// jitter avoids an artificial contour line at exactly the threshold;
			// MAX_INT disables snow for low/flat worlds.
			if (!planetary && !in_tunnel && editor.ground && water_blend <= .5 &&
					!editor.check_for_block_type_absolute(x, ground_y, z, WATER)) {
				const auto surface_block =
						editor.get_block_absolute(x, ground_y, z).value_or(AIR);
				double snow_depth = snow_line.depth(x, z, ground_y);
				const auto cover = editor.ground->cover_class(relative);
				if (terrain_surface::is_glacier_cover(cover) &&
						terrain_surface::is_plausible_ice(
								snow_depth, editor.ground->climate()))
					snow_depth = terrain_surface::glacier_depth(snow_depth);
				if (snow_depth >= -1.5) {
					const auto [snow_slope, gradient] =
							editor.ground->slope_and_gradient(relative);
					const auto snow = terrain_surface::snow_cover(
							snow_depth, snow_slope,
							[&] { return editor.ground->convexity(relative); },
							snow_line.shade(gradient.first, gradient.second, snow_slope),
							x, z);
					// Snow on ice cannot be represented as a floating partial layer;
					// Rust promotes any non-empty snow cover on ice to a full cap.
					if ((snow == terrain_surface::Snow::Block ||
								(snow != terrain_surface::Snow::None &&
										terrain_surface::is_ice(surface_block))) &&
							water_blend <= .5 && surface_block != WATER)
						editor.set_block_absolute(SNOW_BLOCK, x, ground_y, z,
								std::optional<std::vector<Block>>(
										std::vector<Block>{surface_block}),
								std::nullopt);
					else if (snow != terrain_surface::Snow::None)
						terrain_surface::place_snow_layer(editor, x, ground_y, z,
								terrain_surface::snow_eighths(snow,
										terrain_enabled
												? editor.ground->level_exact(relative) -
														  ground_y + .5
												: 0.0));
				}
			}

			if (!planetary && !in_tunnel)
				clear_road_vegetation(editor, x, ground_y, z);

			if (!in_tunnel && args.fillground) {
				const int floor_y = world_editor::terrain_floor_y();
				for (int y = floor_y + 1; y < ground_y; ++y) {
					if (!editor.check_for_block_absolute(x, y, z))
						editor.set_block_absolute(y < ground_y - 8 ? STONE : DIRT, x, y,
								z, std::nullopt, std::nullopt);
				}
				editor.set_block_absolute(
						BEDROCK, x, floor_y, z, std::nullopt, std::nullopt);
			}
		}
	}
}

void generate_ground_layer(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints,
		const CoordinateBitmap *tunnel_footprint,
		const bridges::BridgeSurfaceMap *bridge_surface, bool show_progress)
{
	generate_ground_region(editor, args, xzbbox, building_footprints, xzbbox.min_x(),
			xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z(), tunnel_footprint,
			bridge_surface, show_progress);
}

}
