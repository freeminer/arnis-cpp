#include "ground_generation.h"

#include "block_definitions.h"
#include "element_processing/tree.h"
#include "element_processing/bush.h"
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
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>
#include <utility>
#include <vector>
#include <iostream>

namespace arnis::ground_generation
{

namespace
{

class WaterColumnMemo
{
	int base_x_{};
	int base_z_{};
	std::array<std::uint8_t, 18 * 18> state_{};
	std::optional<std::size_t> slot(int x, int z) const
	{
		const int dx = x - base_x_;
		const int dz = z - base_z_;
		if (dx < 0 || dx >= 18 || dz < 0 || dz >= 18)
			return std::nullopt;
		return static_cast<std::size_t>(dz * 18 + dx);
	}

public:
	WaterColumnMemo(int chunk_x, int chunk_z) :
			base_x_((chunk_x << 4) - 1), base_z_((chunk_z << 4) - 1)
	{
	}

	template <typename Probe>
	bool get(int x, int z, Probe &&probe)
	{
		const auto i = slot(x, z);
		if (!i)
			return std::forward<Probe>(probe)();
		if (state_[*i] != 0)
			return state_[*i] == 2;
		const bool has_water = std::forward<Probe>(probe)();
		state_[*i] = has_water ? 2 : 1;
		return has_water;
	}

	void forget(int x, int z)
	{
		if (const auto i = slot(x, z))
			state_[*i] = 0;
	}
};

struct ChunkGroundCache
{
	std::array<int, 256> grid{};
	int base_x{}, base_z{}, min_x{}, max_x{}, min_z{}, max_z{};

	static ChunkGroundCache populate(WorldEditor &editor, int chunk_x, int chunk_z,
			int min_x, int max_x, int min_z, int max_z)
	{
		ChunkGroundCache cache;
		cache.base_x = chunk_x << 4;
		cache.base_z = chunk_z << 4;
		cache.min_x = min_x;
		cache.max_x = max_x;
		cache.min_z = min_z;
		cache.max_z = max_z;
		for (int x = min_x; x <= max_x; ++x)
			for (int z = min_z; z <= max_z; ++z) {
				const auto lx = static_cast<std::size_t>(x - cache.base_x);
				const auto lz = static_cast<std::size_t>(z - cache.base_z);
				cache.grid[lz * 16 + lx] = editor.get_ground_level(x, z);
			}
		return cache;
	}

	int get(WorldEditor &editor, int x, int z) const
	{
		if (x >= min_x && x <= max_x && z >= min_z && z <= max_z) {
			const auto lx = static_cast<std::size_t>(x - base_x);
			const auto lz = static_cast<std::size_t>(z - base_z);
			return grid[lz * 16 + lx];
		}
		return editor.get_ground_level(x, z);
	}
};

int ground_level_at(WorldEditor &editor, const ChunkGroundCache *cache,
		bool terrain_enabled, int fallback_y, int x, int z)
{
	if (cache)
		return cache->get(editor, x, z);
	return terrain_enabled ? editor.get_ground_level(x, z) : fallback_y;
}

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

int local_slope(WorldEditor &editor, int x, int z, const ChunkGroundCache *ground_cache,
		int fallback_y)
{
	// Rust thresholds the rounded, unrounded-height slope metric. Use the
	// editor's ground origin: passing world coordinates directly clamps DEM
	// sampling to its edge for non-zero-origin maps.
	if (editor.ground)
		return static_cast<int>(std::lround(
				editor.ground->slope_and_gradient(editor.ground_point(x, z)).first));
	const auto level_at = [&](int sx, int sz) {
		return ground_level_at(editor, ground_cache, true, fallback_y, sx, sz);
	};
	// Match Ground::slope's cardinal four-block stencil even when the host
	// editor has no Arnis Ground object. The previous eight-neighbour,
	// one-block estimate classified a visibly different set of columns as
	// steep, changing the terrain palette in this fallback path.
	constexpr int step = 4;
	const std::array<int, 4> samples{{level_at(x + step, z), level_at(x - step, z),
			level_at(x, z - step), level_at(x, z + step)}};
	const auto [min_it, max_it] = std::minmax_element(samples.begin(), samples.end());
	const auto rise = static_cast<std::int64_t>(*max_it) - *min_it;
	return rise >= std::numeric_limits<int>::max() ? std::numeric_limits<int>::max()
												   : static_cast<int>(rise);
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
					YELLOW_CONCRETE,
					DIRT_PATH,
					SMOOTH_STONE,
					WATER,
			}));
}

std::pair<Block, Block> natural_surface_palette(WorldEditor &editor, int x, int z,
		int ground_y, int slope, std::uint8_t cover, bool has_cover)
{
	if (slope > 4)
		return terrain_surface::steep_palette(x, z, ground_y, slope, cover);
	if (has_cover && editor.ground) {
		auto climate_palette =
				climate::surface_palette(editor.ground->climate(), cover, x, z);
		if (climate_palette)
			return *climate_palette;
	}
	if (cover == land_cover::LC_BEACH) {
		const auto climate =
				editor.ground ? editor.ground->climate() : biome::Climate::Temperate;
		if ((climate == biome::Climate::Tundra || climate == biome::Climate::IceCap) &&
				value_noise_01(x + 41, z + 5, 6) < .7)
			return {GRAVEL, STONE};
		return {SAND, SANDSTONE};
	}
	if (cover == land_cover::LC_TREE_COVER || cover == land_cover::LC_GRASSLAND)
		return {GRASS_BLOCK, DIRT};
	if (cover == land_cover::LC_CROPLAND)
		return {FARMLAND, DIRT};
	if (cover == land_cover::LC_BUILT_UP) {
		const auto h = land_cover::coord_hash(x, z) % 100;
		return {h < 72	 ? STONE_BRICKS
				: h < 87 ? CRACKED_STONE_BRICKS
				: h < 92 ? STONE
						 : COBBLESTONE,
				STONE};
	}
	if (cover == land_cover::LC_BARE || cover == land_cover::LC_SNOW_ICE) {
		int neighboring_bare = 0;
		if (has_cover && editor.ground)
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
						 {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}}) {
				const auto neighbor =
						editor.ground->cover_class(editor.ground_point(x + dx, z + dz));
				neighboring_bare += neighbor == land_cover::LC_BARE ||
									neighbor == land_cover::LC_SNOW_ICE ||
									neighbor == land_cover::LC_BEACH;
			}
		if (neighboring_bare == 0)
			return {GRASS_BLOCK, DIRT};
		if (value_noise_01(x, z, 6) < .45)
			return {COARSE_DIRT, DIRT};
		return terrain_surface::bare_rock_palette(x, z);
	}
	if (cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES)
		return {MUD, DIRT};
	if (cover == land_cover::LC_SHRUBLAND) {
		const auto h = land_cover::coord_hash(x, z);
		return value_noise_01(x, z, 5) < .4 && h % 5 != 0 ? std::pair{COARSE_DIRT, DIRT}
														  : std::pair{GRASS_BLOCK, DIRT};
	}
	return {GRASS_BLOCK, DIRT};
}

std::pair<bool, bool> canopy_verdict(WorldEditor &editor, int x, int z, int origin_x,
		int origin_z, int spacing, bool schematic_trees)
{
	if (!editor.ground || !editor.ground->has_canopy() ||
			!editor.ground->has_land_cover())
		return {false, false};
	const auto floor_div = [spacing](int v) {
		const int quotient = v / spacing;
		return quotient - (v % spacing < 0 ? 1 : 0);
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
	const double p = canopy::slot_probability(*fraction, spacing, schematic_trees);
	const double roll =
			double(land_cover::coord_hash(x ^ 0x434d, z ^ 0x484d) % 10000) / 10000.0;
	return {true, roll < p};
}

void maybe_place_vegetation(WorldEditor &editor, int x, int ground_y, int z,
		const BuildingFootprintBitmap &building_footprints, int origin_x, int origin_z,
		const CoordinateBitmap *tunnel_footprint,
		const bridges::BridgeSurfaceMap *bridge_surface, double scale, int slope,
		double forest_fern_share, int tree_spacing, bool schematic_trees)
{
	const bool sealed = editor.surface_is_sealed(x, z);
	if (editor.check_for_block_absolute(x, ground_y + 1, z))
		return;
	const auto natural_ground = [&] {
		return !sealed &&
			   editor.check_for_block_absolute(x, ground_y, z,
					   std::optional<std::vector<Block>>(std::vector<Block>{GRASS_BLOCK,
							   COARSE_DIRT, DIRT, MUD, FARMLAND, PODZOL, MOSS_BLOCK}));
	};
	const bool ground_is_natural = natural_ground();
	const bool ground_allows_trees =
			ground_is_natural ||
			(!sealed &&
					editor.check_for_block_absolute(x, ground_y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{
									SMOOTH_STONE, STONE_BRICKS, CRACKED_STONE_BRICKS})));

	const auto cover =
			editor.ground ? editor.ground->cover_class(editor.ground_point(x, z)) : 0;
	auto rng = coord_rng(x, z, 0);
	const auto climate =
			editor.ground ? editor.ground->climate() : biome::Climate::Temperate;
	const auto [canopy_covered, canopy_tree] = canopy_verdict(
			editor, x, z, origin_x, origin_z, tree_spacing, schematic_trees);
	if (canopy_tree && slope <= 4 && ground_allows_trees &&
			!(tunnel_footprint && tunnel_footprint->contains(x, z)) &&
			!editor.check_for_block_absolute(x, ground_y + 1, z) &&
			!(editor.mapped_trunks && editor.mapped_trunks->under_crown(x, z)))
		Tree::create_from_canopy(
				editor, Coord{x, 1, z}, &building_footprints, bridge_surface);
	// Rust runs its land-cover decoration after canopy trees. A placed canopy
	// trunk now occupies this column; otherwise the same open-column gate applies.
	if (sealed || editor.check_for_block_absolute(x, ground_y + 1, z))
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
	if (cover == land_cover::LC_TREE_COVER && slope <= 4 && ground_allows_trees &&
			!(tunnel_footprint && tunnel_footprint->contains(x, z))) {
		// The canopy map adds a separately selected tree; it suppresses only the
		// land-cover tree roll in measured cells, never the forest-floor plants.
		const auto tree_rate = scale < MICRO_TREE_MAX_SCALE ? 4u : 30u;
		const auto choice = rng.uniform(tree_rate);
		if (choice == 0 && !canopy_covered &&
				!(editor.mapped_trunks && editor.mapped_trunks->under_crown(x, z))) {
			Tree::create(editor, Coord{x, 1, z}, &building_footprints, bridge_surface);
		} else if (ground_is_natural &&
				   undergrowth_roll(x, z, .4 * sward, SALT_FOREST_FLOOR)) {
			const double fern_roll =
					double(land_cover::coord_hash(x ^ 0xfe, z ^ 0x4e) % 100) / 100.0;
			place_decoration(fern_roll < forest_fern_share ? FERN : GRASS, ground_y + 1);
		}
	} else if (cover == land_cover::LC_CROPLAND) {
		const bool farmland =
				editor.check_for_block_type_absolute(x, ground_y, z, FARMLAND);
		if (farmland && x % 9 == 0 && z % 9 == 0 && editor.water_source_is_enclosed(x, z))
			editor.set_block_absolute(WATER, x, ground_y, z,
					std::optional<std::vector<Block>>(std::vector<Block>{FARMLAND}),
					std::nullopt);
		else if (farmland && rng.uniform(76) == 0) {
			if (rng.uniform(10) < 4)
				place_decoration(HAY_BALE, ground_y + 1);
		} else if (farmland) {
			const auto crop_choice = rng.uniform(3);
			const Block crop = crop_choice == 0	  ? WHEAT
							   : crop_choice == 1 ? CARROTS
												  : POTATOES;
			place_decoration(crop, ground_y + 1);
		}
	} else if ((cover == land_cover::LC_WETLAND || cover == land_cover::LC_MANGROVES) &&
			   ground_is_natural) {
		const auto choice = rng.uniform(100);
		if (patch_noise(x, z, 5, SALT_WETLAND_POOLS) < .28 &&
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
		}
	} else if (cover == land_cover::LC_BARE && ground_is_natural) {
		const bool coarse =
				editor.check_for_block_type_absolute(x, ground_y, z, COARSE_DIRT);
		if (coarse) {
			const auto choice = rng.uniform(100);
			if (choice < 6)
				place_decoration(GRASS, ground_y + 1);
			else if (choice < 9)
				bush::place(editor, x, z, bush::Kind::Low);
			else if (choice == 9)
				place_decoration(DEAD_BUSH, ground_y + 1);
		} else if (rng.uniform(100) == 0)
			place_decoration(DEAD_BUSH, ground_y + 1);
	} else if (cover == land_cover::LC_SHRUBLAND && ground_is_natural) {
		if (rng.uniform(100) < 2) {
			bush::place(editor, x, z, bush::Kind::Wild);
		} else if (undergrowth_roll(x, z, .28 * sward, SALT_SHRUB_FLOOR)) {
			place_decoration(GRASS, ground_y + 1);
		}
	} else if (cover == land_cover::LC_GRASSLAND && ground_is_natural) {
		if (undergrowth_roll(x, z, .55 * sward, SALT_SWARD)) {
			const bool stand = patch_noise(x, z, 9, SALT_TALL_SWARD) > .8;
			const auto tall_share = stand ? 35u : 4u;
			if (rng.uniform(100) < tall_share) {
				place_decoration(TALL_GRASS_BOTTOM, ground_y + 1);
				place_decoration(TALL_GRASS_TOP, ground_y + 2);
			} else {
				place_decoration(GRASS, ground_y + 1);
			}
		}
	}
}

void clear_road_vegetation(WorldEditor &editor, int x, int y, int z)
{
	static const std::vector<Block> stray_surface{BLACK_CONCRETE, GRAY_CONCRETE_POWDER,
			CYAN_TERRACOTTA, GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE,
			YELLOW_CONCRETE, DIRT_PATH, WATER};
	const auto &loose_plants = ground_decoration::loose_plant_blocks();
	static const std::vector<Block> wood{OAK_LOG, SPRUCE_LOG, BIRCH_LOG, DARK_OAK_LOG,
			JUNGLE_LOG, ACACIA_LOG, CHERRY_LOG, MANGROVE_LOG};
	const auto &stacked = ground_decoration::stacked_plant_parts();
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

double value_noise_salted(int x, int z, int scale, std::uint32_t salt)
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
	return a * (1.0 - sv) + b * sv;
}

double patch_noise(int x, int z, int scale, std::uint32_t salt)
{
	const double value = value_noise_salted(x, z, scale, salt);
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
	// Rust's water_blend accessor materializes its cached shoreline field on
	// demand. C++ normally warms it during data_processing, but this generator
	// is also a public standalone entry point, so ensure direct callers see the
	// same smoothed ESA/OSM water classification.
	if (editor.ground)
		editor.ground->warm_water_blend();
	if (show_progress)
		std::cout << "[6/7] Generating ground...\n";
	const bool terrain_enabled = generation_mode_terrain(args.mode);
	const int tree_spacing = std::max(1, editor.get_tree_slot_spacing());
	const bool schematic_trees = editor.has_schematic_tree_pack();
	const auto geographic_bounds = editor.geographic_bounds();
	const double center_latitude = (geographic_bounds[0] + geographic_bounds[1]) * .5;
	const terrain_surface::SnowLine snow_line(editor.ground
													  ? editor.ground->snow_threshold()
													  : std::numeric_limits<int>::max(),
			editor.ground ? editor.ground->blocks_per_meter() : 0.0, center_latitude,
			args.rotation);
	const auto chunk_of = [](int v) { return v >= 0 ? v / 16 : -(((-v) + 15) / 16); };
	for (int chunk_x = chunk_of(min_x); chunk_x <= chunk_of(max_x); ++chunk_x) {
		const int chunk_min_x = std::max(min_x, chunk_x * 16);
		const int chunk_max_x = std::min(max_x, chunk_x * 16 + 15);
		for (int chunk_z = chunk_of(min_z); chunk_z <= chunk_of(max_z); ++chunk_z) {
			const int chunk_min_z = std::max(min_z, chunk_z * 16);
			const int chunk_max_z = std::min(max_z, chunk_z * 16 + 15);
			double forest_fern_share = .12;
			if (editor.ground) {
				const int sample_x = (chunk_x << 4) + 8;
				const int sample_z = (chunk_z << 4) + 8;
				const auto eco = editor.ground->ecoregion_at(
						editor.ground_point(sample_x, sample_z));
				const auto habitat = ground_decoration::habitat(land_cover::LC_TREE_COVER,
						editor.ground->climate(), std::abs(center_latitude), false, eco);
				if (habitat == ground_decoration::Habitat::Taiga)
					forest_fern_share = .45;
				else if (habitat == ground_decoration::Habitat::Jungle)
					forest_fern_share = .3;
			}
			std::optional<ChunkGroundCache> ground_cache;
			if (terrain_enabled)
				ground_cache = ChunkGroundCache::populate(editor, chunk_x, chunk_z,
						chunk_min_x, chunk_max_x, chunk_min_z, chunk_max_z);
			WaterColumnMemo water_columns(chunk_x, chunk_z);
			std::optional<terrain_surface::TalusField> talus_field;
			bool talus_checked = false;
			const auto ground_at = [&](int gx, int gz) {
				return ground_level_at(editor, ground_cache ? &*ground_cache : nullptr,
						terrain_enabled, args.ground_level, gx, gz);
			};
			int column_fill_y_min = world_editor::terrain_floor_y() + 1;
			if (args.fillground) {
				const bool chunk_fully_in_bbox = chunk_min_x == (chunk_x << 4) &&
												 chunk_max_x == (chunk_x << 4) + 15 &&
												 chunk_min_z == (chunk_z << 4) &&
												 chunk_max_z == (chunk_z << 4) + 15;
				const bool rotated_in =
						chunk_fully_in_bbox &&
						(!editor.ground || (editor.ground->is_in_rotated_bounds(
													chunk_min_x, chunk_min_z) &&
												   editor.ground->is_in_rotated_bounds(
														   chunk_max_x, chunk_min_z) &&
												   editor.ground->is_in_rotated_bounds(
														   chunk_min_x, chunk_max_z) &&
												   editor.ground->is_in_rotated_bounds(
														   chunk_max_x, chunk_max_z)));
				if (rotated_in) {
					int min_ground_y = args.ground_level;
					if (ground_cache)
						min_ground_y = *std::min_element(
								ground_cache->grid.begin(), ground_cache->grid.end());
					const int bottom_section = chunk_of(world_editor::terrain_floor_y());
					// section_top = section_y*16 + 15 <= min_ground_y - 3.
					const int top_section = chunk_of(min_ground_y - 18);
					const int section_min = std::numeric_limits<std::int8_t>::min();
					const int section_max = std::numeric_limits<std::int8_t>::max();
					if (bottom_section >= section_min && bottom_section <= section_max &&
							top_section >= section_min && top_section <= section_max &&
							top_section >= bottom_section) {
						const bool all_clean = editor.bulk_fill_chunk_sections_below(
								chunk_x, chunk_z, bottom_section, top_section, STONE);
						if (all_clean)
							column_fill_y_min = (top_section + 1) * 16;
					}
				}
			}
			for (int x = chunk_min_x; x <= chunk_max_x; ++x) {
				for (int z = chunk_min_z; z <= chunk_max_z; ++z) {
					// Rotation expands the output AABB. Rust masks columns whose inverse
					// rotated coordinate lies outside the original source bbox; otherwise
					// the expanded corners receive fabricated terrain and vegetation.
					if (editor.ground && !editor.ground->inside_rotation_mask(x, z)) {
						water_columns.forget(x, z);
						continue;
					}
					// Rust's geo-only mode deliberately bypasses elevation and uses the
					// configured flat level.  Keep all downstream surface/water decisions
					// on that same height rather than merely disabling the provider fetch.
					const int ground_y = ground_at(x, z);
					const int slope =
							terrain_enabled
									? local_slope(editor, x, z,
											  ground_cache ? &*ground_cache : nullptr,
											  args.ground_level)
									: 0;
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
					const bool has_cover = !planetary && editor.ground &&
										   editor.ground->has_land_cover();
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
					auto snow = terrain_surface::Snow::None;
					if (!planetary && editor.ground && snow_depth >= -1.5) {
						const auto [snow_slope, gradient] =
								editor.ground->slope_and_gradient(relative);
						snow = terrain_surface::snow_cover(
								snow_depth, snow_slope,
								[&] { return editor.ground->convexity(relative); },
								snow_line.shade(
										gradient.first, gradient.second, snow_slope),
								x, z);
					}
					std::optional<Block> natural_snow_surface;
					std::optional<std::pair<Block, Block>> talus_block;
					if (terrain_enabled && !planetary && editor.ground && slope <= 6 &&
							terrain_surface::takes_talus(cover_here)) {
						if (!talus_checked) {
							talus_field = terrain_surface::TalusField::build(
									*editor.ground, chunk_x, chunk_z, xzbbox.min_x(),
									xzbbox.min_z());
							talus_checked = true;
						}
						if (talus_field)
							talus_block = terrain_surface::talus_palette(x, z,
									talus_field->nearr(
											x, z, editor.ground->level_exact(relative)),
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
							const int neighbour_ground = ground_at(wx, wz);
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
							((has_water_in_column(x, z - 1) &&
									 has_water_in_column(x, z + 1)) ||
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

					// Follow Rust's smoothed ESA water mask while preserving every hard
					// water cell. Interior water remains at terrain height when snapping
					// its surface down to the local water level would leave it submerged.
					const bool esa_water =
							grid_water || existing_water || osm_gap || water_blend > .5;
					bool place_esa_water = false;
					int water_y = 0;
					if (process_surface && !planetary && esa_water && !steep_override) {
						water_y = editor.get_water_level(x, z);
						if (ground_y <= water_y) {
							place_esa_water = true;
						} else if (grid_water && editor.ground &&
								   editor.ground->is_interior_water(relative)) {
							water_y = ground_y;
							place_esa_water = true;
						}
					}
					if (place_esa_water) {
						editor.set_block_if_absent_absolute(WATER, x, water_y, z);
						if (water_y - 1 > world_editor::min_y())
							editor.set_block_if_absent_absolute(SAND, x, water_y - 1, z);
						if (water_y - 2 > world_editor::min_y())
							editor.set_block_if_absent_absolute(
									SANDSTONE, x, water_y - 2, z);
						water_columns.forget(x, z);
					} else {

						// Rust computes the natural under-material even when an authored
						// surface already occupies the cell; its absent-only setter protects
						// that surface while the selected under-block still closes the terrain.
						if (process_surface) {
							const auto planetary_palette =
									planetary
											? celestial_surface_palette(args.body, slope,
													  args.celestial_latitude_degrees,
													  ground_y, x, z)
											: std::pair<Block, Block>{};
							const std::pair<Block, Block> palette =
									planetary	  ? planetary_palette
									: talus_block ? *talus_block
									: slope > 4	  ? terrain_surface::steep_palette(x, z,
															ground_y, slope, cover_here)
									: glacier	  ? std::pair{PACKED_ICE, PACKED_ICE}
												  : natural_surface_palette(editor, x, z,
															ground_y, slope, cover_here,
															has_cover);
							Block surface = palette.first;
							std::optional<Block> climate_under = palette.second;
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
															editor.ground_point(
																	x + dx, z + dz)) ==
															land_cover::LC_WATER) {
												near_esa_water = true;
												break;
											}
								}
								bool near_placed_water = false;
								for (const auto &[dx, dz] :
										std::array<std::pair<int, int>, 4>{
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
							// Select snow before the protected/absent-only surface write, so
							// mapped roads and structures are not overwritten by a later cap.
							if ((snow == terrain_surface::Snow::Block ||
										(snow != terrain_surface::Snow::None &&
												terrain_surface::is_ice(surface))) &&
									water_blend <= .5 && surface != WATER)
								surface = SNOW_BLOCK;
							natural_snow_surface = surface;
							if (steep_override) {
								// Match Rust's steep-face blacklist: roads, structures, bedrock,
								// and water remain authored even when terrain rock is forced.
								editor.set_block_absolute(surface, x, ground_y, z,
										std::nullopt,
										std::optional<std::vector<Block>>(
												std::vector<Block>{WATER, BEDROCK,
														GRAY_CONCRETE_POWDER,
														CYAN_TERRACOTTA, GRAY_CONCRETE,
														LIGHT_GRAY_CONCRETE,
														WHITE_CONCRETE, YELLOW_CONCRETE,
														DIRT_PATH, STONE_BRICKS, BRICK,
														OAK_PLANKS, BLACK_CONCRETE}));
							} else if (talus_block && surface == talus_block->first) {
								editor.set_block_absolute(surface, x, ground_y, z,
										std::optional<std::vector<Block>>(
												terrain_surface::talus_buries()),
										std::nullopt);
							} else {
								editor.set_block_if_absent_absolute(
										surface, x, ground_y, z);
							}
							const bool surface_is_water =
									editor.check_for_block_type_absolute(
											x, ground_y, z, WATER);
							auto set_under_if_absent = [&](Block block) {
								if (!editor.check_for_block_absolute(x, ground_y - 1, z))
									editor.set_block_absolute(block, x, ground_y - 1, z,
											std::nullopt, std::nullopt);
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
									editor.set_block_if_absent_absolute(
											floor, x, floor_y, z);
									if (floor_y - 1 > world_editor::min_y())
										editor.set_block_if_absent_absolute(
												SANDSTONE, x, floor_y - 1, z);
								}
							}
						}

						if (!planetary && has_cover)
							maybe_place_vegetation(editor, x, ground_y, z,
									building_footprints, xzbbox.min_x(), xzbbox.min_z(),
									tunnel_footprint, bridge_surface, args.scale, slope,
									forest_fern_share, tree_spacing, schematic_trees);

						// Snow is a separate cap, so climate/land-cover material selection is
						// preserved below it just as in the Rust ground pass.
						// Match Rust's softened climatic snow edge.  A small deterministic
						// jitter avoids an artificial contour line at exactly the threshold;
						// MAX_INT disables snow for low/flat worlds.
						if (!planetary && editor.ground && water_blend <= .5 &&
								!editor.check_for_block_type_absolute(
										x, ground_y, z, WATER)) {
							const auto surface_block =
									editor.get_block_absolute(x, ground_y, z)
											.value_or(AIR);
							if (snow != terrain_surface::Snow::None) {
								const bool natural_top =
										natural_snow_surface == surface_block ||
										surface_block == PACKED_ICE;
								const unsigned eighths =
										natural_top
												? terrain_surface::snow_eighths(snow,
														  terrain_enabled
																  ? editor.ground->level_exact(
																			relative) -
																			ground_y + .5
																  : 0.0)
												: 1;
								terrain_surface::place_snow_layer(
										editor, x, ground_y, z, eighths);
							}
						}
					}

					// Rust's universal depth pass closes visible gaps below all terrain
					// columns, including ones whose surface was supplied by OSM.
					if (terrain_enabled && !editor.check_for_block_type_absolute(
												   x, ground_y, z, WATER)) {
						int lowest = ground_y;
						for (int dx = -1; dx <= 1; ++dx)
							for (int dz = -1; dz <= 1; ++dz)
								if (dx || dz)
									lowest = std::min(lowest, ground_at(x + dx, z + dz));
						const int depth = std::clamp(ground_y - lowest + 1, 2, 64);
						const int fill_min = std::max(
								world_editor::terrain_floor_y() + 1, ground_y - depth);
						const int fill_max = ground_y - 1;
						const Block under = column_under.value_or(STONE);
						if (terrain_enabled && !planetary && slope > 4 && under == STONE)
							terrain_surface::fill_strata(
									editor, x, z, fill_min, fill_max, slope > 6);
						else
							editor.fill_column_absolute(
									under, x, z, fill_min, fill_max, true);
					}

					// Rust performs this post-pass for every body and column: an
					// authored road or water surface can receive vegetation from an
					// overlapping element regardless of the active terrain provider.
					clear_road_vegetation(editor, x, ground_y, z);

					if (args.fillground)
						editor.fill_column_absolute(
								STONE, x, z, column_fill_y_min, ground_y - 3, true);
					// Rust keeps bedrock as a flat floor independently of fillground,
					// replacing any generated block there except existing bedrock.
					editor.set_block_absolute(BEDROCK, x, world_editor::terrain_floor_y(),
							z, std::nullopt,
							std::optional<std::vector<Block>>(
									std::vector<Block>{BEDROCK}));
					// This column may now contain water placed by the ground pass; do not
					// let a neighbour reuse the pre-write answer.
					water_columns.forget(x, z);
				}
			}
		}
	}

	// Rust places deterministic, chunk-origin plant patches only after every
	// ground column in this region has its final surface. Keep the caller's
	// region bounds here so streamed tile generation uses the same patch origins
	// while clipping writes to the tile currently being generated.
	ground_decoration::decorate_region(editor, args, xzbbox, min_x, max_x, min_z, max_z);
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
