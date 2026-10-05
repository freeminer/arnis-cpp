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
#include <cstdint>
#include <optional>
#include <limits>
#include <map>
#include <utility>
#include <unordered_map>
#include <vector>
#include <iostream>

namespace arnis::ground_generation
{

namespace
{

class WaterColumnMemo
{
	std::unordered_map<std::uint64_t, bool> state_;

	static std::uint64_t key(int x, int z)
	{
		return (std::uint64_t(static_cast<std::uint32_t>(x)) << 32) |
			   static_cast<std::uint32_t>(z);
	}

public:
	template <typename Probe>
	bool get(int x, int z, Probe &&probe)
	{
		const auto encoded = key(x, z);
		if (const auto found = state_.find(encoded); found != state_.end())
			return found->second;
		const bool has_water = std::forward<Probe>(probe)();
		state_.emplace(encoded, has_water);
		return has_water;
	}

	void forget(int x, int z) { state_.erase(key(x, z)); }
};

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
	// Rust thresholds the rounded, unrounded-height slope metric. Use the
	// editor's ground origin: passing world coordinates directly clamps DEM
	// sampling to its edge for non-zero-origin maps.
	if (editor.ground)
		return static_cast<int>(std::lround(
				editor.ground->slope_and_gradient(editor.ground_point(x, z)).first));
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

Block natural_surface_for(WorldEditor &editor, int x, int ground_y, int z)
{
	const auto cover =
			editor.ground ? editor.ground->cover_class(editor.ground_point(x, z)) : 0;
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
				auto c = editor.ground->cover_class(editor.ground_point(x + dx, z + dz));
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

std::pair<bool, bool> canopy_verdict(
		WorldEditor &editor, int x, int z, int origin_x, int origin_z)
{
	if (!editor.ground || !editor.ground->has_canopy() ||
			!editor.ground->has_land_cover())
		return {false, false};
	const int spacing = std::max(1, editor.get_tree_slot_spacing());
	const auto floor_div = [spacing](int v) {
		return v >= 0 ? v / spacing : -(((-v) + spacing - 1) / spacing);
	};
	const int cell_x = floor_div(x) * spacing;
	const int cell_z = floor_div(z) * spacing;
	const auto fraction = editor.ground->canopy_fraction(
			XZPoint{cell_x - origin_x, cell_z - origin_z}, spacing);
	if (!fraction)
		return {false, false};
	const auto [slot_x, slot_z] = trees::trunk_slot_s(x, z, spacing);
	if (x != slot_x || z != slot_z)
		return {true, false};
	const double p =
			canopy::slot_probability(*fraction, spacing, editor.place_schematics());
	const double roll =
			double(land_cover::coord_hash(x ^ 0x434d, z ^ 0x484d) % 10000) / 10000.0;
	return {true, roll < p};
}

void maybe_place_vegetation(WorldEditor &editor, int x, int ground_y, int z,
		const BuildingFootprintBitmap &building_footprints, int origin_x, int origin_z,
		const bridges::BridgeSurfaceMap *bridge_surface, double scale,
		double absolute_latitude, int alpine_from_y, int slope)
{
	if (editor.surface_is_sealed(x, z) || building_footprints.contains(x, z) ||
			(bridge_surface && bridge_surface->contains(x, z)) ||
			editor.check_for_block_absolute(x, ground_y + 1, z))
		return;
	const auto natural_ground = [&] {
		return editor.check_for_block_absolute(x, ground_y, z,
				std::optional<std::vector<Block>>(std::vector<Block>{GRASS_BLOCK,
						COARSE_DIRT, DIRT, MUD, FARMLAND, PODZOL, MOSS_BLOCK}));
	};
	const bool ground_is_natural = natural_ground();
	const bool ground_allows_trees =
			ground_is_natural ||
			editor.check_for_block_absolute(x, ground_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{
							SMOOTH_STONE, STONE_BRICKS, CRACKED_STONE_BRICKS}));

	const auto cover =
			editor.ground ? editor.ground->cover_class(editor.ground_point(x, z)) : 0;
	auto rng = coord_rng(x, z, 0);
	const int patch_x = x >= 0 ? x / 8 : -(((-x) + 7) / 8);
	const int patch_z = z >= 0 ? z / 8 : -(((-z) + 7) / 8);
	const auto patch_roll = land_cover::coord_hash(patch_x, patch_z) % 100;
	const auto eco = editor.ground
							 ? editor.ground->ecoregion_at(editor.ground_point(x, z))
							 : std::nullopt;
	const auto climate =
			editor.ground ? editor.ground->climate() : biome::Climate::Temperate;
	const auto site_habitat = ground_decoration::habitat(
			cover, climate, absolute_latitude, ground_y >= alpine_from_y, eco);
	const auto [canopy_covered, canopy_tree] =
			canopy_verdict(editor, x, z, origin_x, origin_z);
	if (canopy_tree && slope <= 4 && ground_allows_trees &&
			!editor.check_for_block_absolute(x, ground_y + 1, z) &&
			!(editor.mapped_trunks && editor.mapped_trunks->under_crown(x, z)))
		Tree::create_from_canopy(
				editor, Coord{x, 1, z}, &building_footprints, bridge_surface);
	// Rust runs its land-cover decoration after canopy trees. A placed canopy
	// trunk now occupies this column; otherwise the same open-column gate applies.
	if (editor.check_for_block_absolute(x, ground_y + 1, z))
		return;
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
	if (cover == land_cover::LC_TREE_COVER && slope <= 4 && ground_allows_trees) {
		// The canopy map adds a separately selected tree; it suppresses only the
		// land-cover tree roll in measured cells, never the forest-floor plants.
		constexpr double micro_tree_max_scale = 0.35;
		const auto tree_rate = scale < micro_tree_max_scale ? 4u : 30u;
		const auto choice = rng.uniform(tree_rate);
		if (choice == 0 && !canopy_covered &&
				!(editor.mapped_trunks && editor.mapped_trunks->under_crown(x, z))) {
			Tree::create(editor, Coord{x, 1, z}, &building_footprints, bridge_surface);
		} else if (ground_is_natural &&
				   undergrowth_roll(x, z, .4 * sward, SALT_FOREST_FLOOR)) {
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
	WaterColumnMemo water_columns;
	for (int x = min_x; x <= max_x; ++x) {
		for (int z = min_z; z <= max_z; ++z) {
			// Rotation expands the output AABB. Rust masks columns whose inverse
			// rotated coordinate lies outside the original source bbox; otherwise
			// the expanded corners receive fabricated terrain and vegetation.
			if (editor.ground && !editor.ground->inside_rotation_mask(x, z)) {
				water_columns.forget(x, z);
				continue;
			}
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
			const bool mapped_sand =
					editor.check_for_block_type_absolute(x, ground_y, z, SAND);
			const bool steep_override =
					terrain_enabled && slope > 4 && (slope > 6 || !mapped_sand);
			const auto relative = editor.ground_point(x, z);
			const bool planetary = !is_earth(args.body);
			const bool has_cover =
					!planetary && editor.ground && editor.ground->has_land_cover();
			const std::uint8_t cover_here =
					has_cover ? editor.ground->cover_class(relative) : 0;
			// Rust treats plausible ESA snow/ice cells as glacier even below the
			// ordinary snow line. Compute this before choosing the base surface so
			// the packed-ice substrate and the later snow cap use the same verdict.
			double snow_depth = snow_line.depth(x, z, ground_y);
			const bool glacier = !planetary &&
								 terrain_surface::is_glacier_cover(cover_here) &&
								 editor.ground &&
								 terrain_surface::is_plausible_ice(
										 snow_depth, editor.ground->climate());
			if (glacier)
				snow_depth = terrain_surface::glacier_depth(snow_depth);
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
				return water_columns.get(wx, wz, [&] {
					const int neighbour_ground = terrain_enabled
														 ? editor.get_ground_level(wx, wz)
														 : args.ground_level;
					for (int dy = 0; dy <= 2; ++dy)
						if (editor.check_for_block_type_absolute(
									wx, neighbour_ground + dy, wz, WATER))
							return true;
					return false;
				});
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
					(grid_water || existing_water || osm_gap || water_blend > .5)) {
				const int water_y = editor.get_water_level(x, z);
				if (ground_y <= water_y && !is_protected_surface(editor, x, water_y, z)) {
					editor.set_block_if_absent_absolute(WATER, x, water_y, z);
					if (water_y - 1 > world_editor::min_y())
						editor.set_block_if_absent_absolute(SAND, x, water_y - 1, z);
					if (water_y - 2 > world_editor::min_y())
						editor.set_block_if_absent_absolute(SANDSTONE, x, water_y - 2, z);
					// Match Rust: only skip normal ground generation when this
					// column was actually converted to water.  A classified cell
					// above its water surface must remain terrain, otherwise the
					// later water/depth pass leaves false flooded areas.
					water_columns.forget(x, z);
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
					editor.set_block_if_absent_absolute(WATER, x, water_y, z);
					water_columns.forget(x, z);
					continue;
				}
			}

			// Rust computes the natural under-material even when an authored
			// surface already occupies the cell; its absent-only setter protects
			// that surface while the selected under-block still closes the terrain.
			if (process_surface && !in_tunnel) {
				const auto planetary_palette =
						planetary
								? celestial_surface_palette(args.body, slope,
										  args.celestial_latitude_degrees, ground_y, x, z)
								: std::pair<Block, Block>{};
				Block surface =
						planetary ? planetary_palette.first
								  : (talus_block ? talus_block->first
											: slope > 4
													? terrain_surface::steep_palette(x, z,
															  ground_y, slope, cover_here)
															  .first
											: glacier ? PACKED_ICE
													  : natural_surface_for(
																editor, x, ground_y, z));
				std::optional<Block> climate_under;
				if (planetary)
					climate_under = planetary_palette.second;
				else if (talus_block)
					climate_under = talus_block->second;
				else if (slope > 4)
					climate_under = terrain_surface::steep_palette(
							x, z, ground_y, slope, cover_here)
											.second;
				else if (glacier)
					climate_under = PACKED_ICE;
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
												editor.ground_point(x + dx, z + dz)) ==
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
					editor.set_block_if_absent_absolute(surface, x, ground_y, z);
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
				} else if (surface_is_water) {
					// Match Rust's OSM-water seabed: stop at the bottom of the
					// contiguous water column, then cap it with a stable sand /
					// gravel / clay choice and sandstone underneath. ESA water has
					// its own depth-aware bed in water_depth::carve_lc_water_pass.
					int water_bottom = ground_y;
					while (water_bottom - 1 > world_editor::min_y() &&
							editor.check_for_block_type_absolute(
									x, water_bottom - 1, z, WATER))
						--water_bottom;
					const int floor_y = water_bottom - 1;
					if (floor_y > world_editor::min_y()) {
						const auto hash = land_cover::coord_hash(x, z);
						const Block floor = hash % 5 == 0	? GRAVEL
											: hash % 5 == 1 ? CLAY
															: SAND;
						editor.set_block_if_absent_absolute(floor, x, floor_y, z);
						if (floor_y - 1 > world_editor::min_y())
							editor.set_block_if_absent_absolute(
									SANDSTONE, x, floor_y - 1, z);
					}
				}
			}

			if (!planetary && !in_tunnel && has_cover)
				maybe_place_vegetation(editor, x, ground_y, z, building_footprints,
						xzbbox.min_x(), xzbbox.min_z(), bridge_surface, args.scale,
						std::abs(center_latitude), alpine_from_y, slope);

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
			// This column may now contain water placed by the ground pass; do not
			// let a neighbour reuse the pre-write answer.
			water_columns.forget(x, z);
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
