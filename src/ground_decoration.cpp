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

namespace arnis::ground_decoration
{
namespace
{
bool open_above(WorldEditor &e, int x, int y, int z)
{
	const bool first_open = !e.block_exists_absolute(x, y + 1, z) ||
							e.check_for_block_absolute(x, y + 1, z,
									std::optional<std::vector<Block>>{{GRASS}});
	const std::vector<Block> wood{OAK_LOG, SPRUCE_LOG, BIRCH_LOG, DARK_OAK_LOG,
			JUNGLE_LOG, ACACIA_LOG, CHERRY_LOG};
	return first_open && !e.check_for_block_absolute(x, y + 2, z, wood);
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

Block flower_for(int x, int z, FlowerSetting setting, int field_scale = 24)
{
	const auto h = land_cover::coord_hash(x ^ 0x5ca7, z ^ 0x7e11);
	static const Block meadow[] = {YELLOW_FLOWER, RED_FLOWER, WHITE_FLOWER, BLUE_FLOWER};
	static const Block forest[] = {RED_FLOWER, YELLOW_FLOWER, WHITE_FLOWER, BLUE_FLOWER};
	static const Block garden[] = {RED_FLOWER, BLUE_FLOWER, WHITE_FLOWER, YELLOW_FLOWER};
	const Block *palette = meadow;
	std::size_t size = std::size(meadow);
	if (setting == FlowerSetting::Forest) {
		palette = forest;
		size = std::size(forest);
	} else if (setting == FlowerSetting::Garden) {
		palette = garden;
		size = std::size(garden);
	}
	// Smooth fields make neighbouring flowers form drifts instead of white
	// noise.  Keep the hash as a minority fallback, matching Rust's 85/15
	// patch-vs-variation split.
	const auto field = ground_generation::patch_noise(
			x, z, field_scale, 0x9A7C4E11u ^ static_cast<std::uint32_t>(field_scale));
	const auto index = (h % 100 < 85) ? static_cast<std::size_t>(field * size) % size
									  : static_cast<std::size_t>((h >> 8) % size);
	return palette[index];
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
		editor.set_block_absolute(flower_for(x, z, FlowerSetting::Garden, 4), x, y + 1, z,
				std::optional<std::vector<Block>>({GRASS, FARMLAND}), std::nullopt);
}

void decorate_region(WorldEditor &editor, const Args &args, const ::XZBBox &bbox,
		int min_x, int max_x, int min_z, int max_z)
{
	if (!editor.ground || !editor.ground->has_land_cover() || !is_earth(args.body) ||
			min_x > max_x || min_z > max_z)
		return;
	constexpr int spread = 8;
	const int cx0 = (min_x - spread) >> 4, cx1 = (max_x + spread) >> 4;
	const int cz0 = (min_z - spread) >> 4, cz1 = (max_z + spread) >> 4;
	for (int cz = cz0; cz <= cz1; ++cz)
		for (int cx = cx0; cx <= cx1; ++cx) {
			for (int origin = 0; origin < 4; ++origin) {
				// Rust keys every origin independently.  Reusing one stream here
				// changes all later origins when an earlier feature is skipped and
				// makes decoration differ at tile boundaries.
				ChaCha8Rng rng = coord_rng(
						cx, cz, 0x9A7C4E11D00D0001ULL ^ (std::uint64_t(origin) << 48));
				const int ox = (cx << 4) + int(rng.uniform(16));
				const int oz = (cz << 4) + int(rng.uniform(16));
				const auto roll = rng.uniform(1000);
				if (ox < min_x - spread || ox > max_x + spread || oz < min_z - spread ||
						oz > max_z + spread || editor.surface_is_sealed(ox, oz))
					continue;
				const int x = ox + int(rng.uniform(17)) - int(rng.uniform(17));
				const int z = oz + int(rng.uniform(17)) - int(rng.uniform(17));
				if (x < min_x || x > max_x || z < min_z || z > max_z ||
						editor.surface_is_sealed(x, z))
					continue;
				const auto cover =
						editor.ground->cover_class({x - bbox.min_x(), z - bbox.min_z()});
				const auto eco =
						editor.ground->ecoregion_at({x - bbox.min_x(), z - bbox.min_z()});
				const int y = editor.get_ground_level(x, z);
				if (cover == land_cover::LC_WETLAND ||
						cover == land_cover::LC_MANGROVES) {
					const bool water = editor.check_for_block_absolute(x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{WATER}));
					if (water && roll < 110)
						stack(editor, x, y, z, WATER, KELP_PLANT, 2 + int(roll % 3));
					else if (water && roll < 240)
						editor.set_block_absolute(
								LILY_PAD, x, y + 1, z, std::nullopt, std::nullopt);
					else if (roll < 400 && beside_water(editor, x, y, z))
						stack(editor, x, y, z, GRASS_BLOCK, SUGAR_CANE,
								1 + int(roll % 3));
					else if (roll < 700)
						plant(editor, x, y, z, GRASS_BLOCK, GRASS);
				} else if (cover == land_cover::LC_CROPLAND && roll < 180) {
					plant(editor, x, y, z, FARMLAND, PUMPKIN);
				} else if (cover == land_cover::LC_MOSS && roll < 680) {
					plant(editor, x, y, z, GRASS_BLOCK, MOSS_CARPET);
				} else if (cover == land_cover::LC_BARE && roll < 45 &&
						   editor.check_for_block_absolute(x, y, z,
								   std::optional<std::vector<Block>>(
										   std::vector<Block>{SAND})) &&
						   editor.mg && [&] {
							   const auto [lat, lon] = editor.mg->pos_to_ll(x, z);
							   (void)lat;
							   return lon >= -170.0 && lon <= -30.0;
						   }()) {
					stack(editor, x, y, z, SAND, CACTUS, 1 + int(roll % 3), true);
				} else if (cover == land_cover::LC_SHRUBLAND && roll < 170 && eco &&
						   (eco->biome == ecoregion::EcoBiome::Boreal ||
								   eco->biome ==
										   ecoregion::EcoBiome::TemperateGrassland)) {
					plant(editor, x, y, z, GRASS_BLOCK, SWEET_BERRY_BUSH);
				} else if (cover == land_cover::LC_BARE && roll < 210) {
					plant(editor, x, y, z, COARSE_DIRT, DEAD_BUSH);
				} else if (cover == land_cover::LC_GRASSLAND ||
						   cover == land_cover::LC_SHRUBLAND) {
					if (roll < 520)
						plant(editor, x, y, z, GRASS_BLOCK, GRASS);
					else if (roll < 650)
						tall_plant(editor, x, y, z, TALL_GRASS_BOTTOM, TALL_GRASS_TOP);
					else if (roll < 730)
						place_scattered_flower(editor, x, z, FlowerSetting::Meadow);
					else if (roll < 780)
						plant(editor, x, y, z, GRASS_BLOCK,
								eco && (eco->biome == ecoregion::EcoBiome::Boreal ||
											   eco->biome == ecoregion::EcoBiome::Tundra)
										? MOSS_CARPET
										: FERN);
				} else if (cover == land_cover::LC_TREE_COVER) {
					if (roll < 360)
						plant(editor, x, y, z, GRASS_BLOCK, FERN);
					else if (roll < 430)
						place_scattered_flower(editor, x, z, FlowerSetting::Forest);
					else if (roll < 500 && beside_water(editor, x, y, z))
						stack(editor, x, y, z, GRASS_BLOCK, SUGAR_CANE,
								1 + int(roll % 3));
					else if (roll < 590 &&
							 editor.highest_block_between(x, z, y + 3, y + 24))
						plant(editor, x, y, z, GRASS_BLOCK,
								(roll & 1) ? RED_MUSHROOM : BROWN_MUSHROOM);
				}
			}
		}
}
} // namespace arnis::ground_decoration
