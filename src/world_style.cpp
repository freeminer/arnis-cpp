#include "world_style.h"

#include "block_definitions.h"
#include "ground_generation.h"
#include "land_cover/land_cover.h"
#include "../../emerge.h"
#include "../../arnis_world_editor.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <string_view>
#include <vector>

namespace arnis::world_style
{
namespace
{
constexpr std::uint32_t SALT_COLLAPSE = 0x5255'4953, SALT_JAG = 0x4A41'4747,
						SALT_HOLE = 0x484F'4C45, SALT_CRACK = 0x4352'4B53,
						SALT_RUBBLE = 0x5242'4C45, SALT_DEPTH = 0x534E'4F57,
						SALT_PLOW = 0x504C'4F57, SALT_ICE = 0x4943'4521,
						SALT_VINE = 0x5649'4E45, SALT_MOSS = 0x4D4F'5353,
						SALT_ROOF = 0x524F'4F46;
constexpr int RUBBLE_REACH = 2, MOSS_REACH = 6, LAMP_REACH = 12;
constexpr double VINE_SHARE = .35, WHITE_ROOF_SHARE = .85;

enum class Kind
{
	Air,
	Water,
	Snowy,
	Plant,
	TallPlant,
	Leaves,
	Glass,
	Light,
	Thin,
	Shaped,
	Solid
};

std::string_view short_name(const std::string &name)
{
	const auto colon = name.find(':');
	return colon == std::string::npos ? std::string_view(name)
									  : std::string_view(name).substr(colon + 1);
}

Block named(
		WorldEditor &editor, std::initializer_list<const char *> names, Block fallback)
{
	if (!editor.mg || !editor.mg->m_emerge || !editor.mg->m_emerge->ndef)
		return fallback;
	for (const auto *name : names) {
		const std::array<std::string, 3> candidates = {std::string{name},
				"mcl_core:" + std::string{name}, "default:" + std::string{name}};
		for (const auto &candidate : candidates) {
			const auto id = editor.mg->m_emerge->ndef->getId(candidate);
			if (id != CONTENT_IGNORE && id != CONTENT_AIR)
				return Block{id};
		}
	}
	return fallback;
}

Kind kind(WorldEditor &editor, const Block &block)
{
	const auto full = editor.block_name(block);
	if (!full)
		return Kind::Solid;
	const auto n = short_name(*full);
	if (n == "air" || n == "cave_air" || n == "void_air" || n == "light" ||
			n == "barrier" || n == "structure_void")
		return Kind::Air;
	if (n == "water" || n == "bubble_column")
		return Kind::Water;
	if (n == "snow" || n == "snow_block" || n == "powder_snow" || n == "ice" ||
			n == "packed_ice" || n == "blue_ice")
		return Kind::Snowy;
	static constexpr std::array<std::string_view, 8> tall = {"tall_grass", "large_fern",
			"sunflower", "lilac", "rose_bush", "peony", "tall_dry_grass",
			"pitcher_plant"};
	if (std::find(tall.begin(), tall.end(), n) != tall.end())
		return Kind::TallPlant;
	static constexpr std::array<std::string_view, 32> plants = {"short_grass", "grass",
			"fern", "dead_bush", "bush", "firefly_bush", "short_dry_grass",
			"sweet_berry_bush", "wheat", "carrots", "potatoes", "beetroots", "dandelion",
			"poppy", "blue_orchid", "allium", "azure_bluet", "red_tulip", "orange_tulip",
			"white_tulip", "pink_tulip", "oxeye_daisy", "cornflower",
			"lily_of_the_valley", "pink_petals", "wildflowers", "leaf_litter",
			"torchflower", "open_eyeblossom", "closed_eyeblossom", "brown_mushroom",
			"red_mushroom"};
	if (std::find(plants.begin(), plants.end(), n) != plants.end() ||
			n.ends_with("_sapling"))
		return Kind::Plant;
	if (n.ends_with("_leaves"))
		return Kind::Leaves;
	if (n.find("glass") != std::string_view::npos)
		return Kind::Glass;
	if (n == "fire" || n == "soul_fire" || n == "glowstone" || n == "sea_lantern" ||
			n == "shroomlight" || n == "redstone_lamp" || n == "jack_o_lantern" ||
			n == "end_rod" || n.ends_with("lantern") || n.ends_with("froglight") ||
			n.find("torch") != std::string_view::npos || n.ends_with("candle"))
		return Kind::Light;
	static constexpr std::array<std::string_view, 50> thin = {"fence", "_wall", "door",
			"sign", "banner", "button", "lever", "rail", "carpet", "pressure_plate",
			"ladder", "vine", "chain", "bars", "_head", "skull", "potted", "flower_pot",
			"bell", "anvil", "_bed", "chest", "cake", "campfire", "hopper", "cauldron",
			"grindstone", "lectern", "scaffolding", "cobweb", "lily_pad", "kelp",
			"seagrass", "coral", "pickle", "lichen", "bamboo", "sugar_cane", "cactus",
			"dripstone", "dripleaf", "azalea", "amethyst_bud", "amethyst_cluster",
			"sculk_vein", "sculk_sensor", "sculk_shrieker", "daylight_detector",
			"brewing_stand", "composter"};
	if (std::any_of(thin.begin(), thin.end(),
				[&](auto part) { return n.find(part) != std::string_view::npos; }))
		return Kind::Thin;
	if (n.ends_with("_stairs") || n.ends_with("_slab"))
		return Kind::Shaped;
	return Kind::Solid;
}

bool flammable(WorldEditor &editor, const Block &block)
{
	const auto full = editor.block_name(block);
	if (!full)
		return false;
	const auto n = short_name(*full);
	if (kind(editor, block) == Kind::Plant || kind(editor, block) == Kind::TallPlant ||
			kind(editor, block) == Kind::Leaves)
		return true;
	static constexpr std::array<std::string_view, 27> parts = {"oak", "spruce", "birch",
			"jungle", "acacia", "mangrove", "cherry", "bamboo", "planks", "log", "wood",
			"wool", "carpet", "vine", "hay", "bookshelf", "barrel", "scaffolding",
			"lectern", "composter", "beehive", "bee_nest", "target", "tnt", "coal_block",
			"moss", "azalea"};
	return std::any_of(parts.begin(), parts.end(),
			[&](auto part) { return n.find(part) != std::string_view::npos; });
}

double noise(int x, int z, int scale, std::uint32_t salt)
{
	return ground_generation::patch_noise(x, z, scale, salt);
}

void put(WorldEditor &editor, const Block &block, int x, int y, int z)
{
	static const std::optional<std::vector<Block>> unrestricted = std::vector<Block>{};
	editor.set_block_absolute(block, x, y, z, std::nullopt, unrestricted);
}

void put(WorldEditor &editor, const BlockWithProperties &block, int x, int y, int z)
{
	static const std::optional<std::vector<Block>> unrestricted = std::vector<Block>{};
	editor.set_block_with_properties_absolute(block, x, y, z, std::nullopt, unrestricted);
}

struct Column
{
	int x, z, ground;
};
void swap_material(WorldEditor &, const Column &, int, const Block &, Block);

std::optional<std::pair<int, Block>> top(
		WorldEditor &editor, const Column &c, int min_y, int max_y)
{
	const auto y = editor.highest_block_between(c.x, c.z, min_y, max_y);
	if (!y)
		return std::nullopt;
	const auto b = editor.get_block_absolute(c.x, *y, c.z);
	return b ? std::optional<std::pair<int, Block>>{std::pair<int, Block>{*y, *b}}
			 : std::nullopt;
}

std::uint64_t col_hash(const Column &c, int y)
{
	return land_cover::coord_hash(c.x ^ (y * 0x9E37), c.z ^ (y * 0x85EB));
}

unsigned snow_depth(const Column &c, bool on_ground, bool thin)
{
	if (thin)
		return 1;
	const double drift = noise(c.x, c.z, 20, SALT_DEPTH);
	return on_ground ? 1 + unsigned(drift * 3.99) : 1 + unsigned(drift > .6);
}

void snowy_ground(WorldEditor &editor, const Column &c, int y)
{
	const auto block = editor.get_block_absolute(c.x, y, c.z);
	if (!block)
		return;
	if (block->id() == GRASS_BLOCK.id())
		swap_material(editor, c, y, *block, SNOWY_GRASS_BLOCK);
	else if (block->id() == PODZOL.id())
		swap_material(editor, c, y, *block, SNOWY_PODZOL);
	else if (block->id() == FARMLAND.id() || block->id() == DIRT_PATH.id() ||
			 block->id() == MUD.id())
		swap_material(editor, c, y, *block, DIRT);
}

bool full_top(const Block &block)
{
	const auto props = block.properties();
	if (!props)
		return false;
	const auto type = props->find("type"), half = props->find("half");
	return (type != props->end() &&
				   (type->second == "top" || type->second == "double")) ||
		   (half != props->end() && half->second == "top");
}

void swap_material(
		WorldEditor &editor, const Column &c, int y, const Block &old, Block replacement)
{
	replacement.setParam2(old.getParam2());
	put(editor, replacement, c.x, y, c.z);
}

Block weathered(WorldEditor &editor, const Block &b, std::uint64_t h)
{
	const auto full_name = editor.block_name(b).value_or("");
	const auto n = short_name(full_name);
	const auto roll = h % 100;
	if (n == "stone_bricks" && roll < 30)
		return named(editor, {"cracked_stone_bricks"}, b);
	if (n == "stone_bricks" && roll < 55)
		return named(editor, {"mossy_stone_bricks"}, MOSSY_STONE_BRICKS);
	if (n.ends_with("stone_brick_stairs") && roll < 35)
		return named(editor, {"mossy_stone_brick_stairs"}, b);
	if (n.ends_with("stone_brick_slab") && roll < 35)
		return named(editor, {"mossy_stone_brick_slab"}, b);
	if (n.ends_with("stone_brick_wall") && roll < 35)
		return named(editor, {"mossy_stone_brick_wall"}, b);
	if (n == "cobblestone" && roll < 40)
		return named(editor, {"mossy_cobblestone"}, MOSSY_COBBLESTONE);
	if (n.ends_with("cobblestone_stairs") && roll < 35)
		return named(editor, {"mossy_cobblestone_stairs"}, b);
	if (n.ends_with("cobblestone_slab") && roll < 35)
		return named(editor, {"mossy_cobblestone_slab"}, b);
	if (n.ends_with("cobblestone_wall") && roll < 35)
		return named(editor, {"mossy_cobblestone_wall"}, MOSSY_COBBLESTONE_WALL);
	if (n == "deepslate_bricks" && roll < 40)
		return named(editor, {"cracked_deepslate_bricks"}, b);
	if (n == "deepslate_tiles" && roll < 40)
		return named(editor, {"cracked_deepslate_tiles"}, b);
	if (n == "polished_blackstone_bricks" && roll < 40)
		return named(editor, {"cracked_polished_blackstone_bricks"}, b);
	if (n == "nether_bricks" && roll < 40)
		return named(editor, {"cracked_nether_bricks"}, b);
	if ((n == "stone" || n == "smooth_stone" || n == "polished_andesite") && roll < 18)
		return roll < 12 ? COBBLESTONE : MOSSY_COBBLESTONE;
	return b;
}

Block rubble(WorldEditor &editor, std::uint64_t h)
{
	switch (h % 6) {
	case 0:
	case 1:
		return COBBLESTONE;
	case 2:
		return MOSSY_COBBLESTONE;
	case 3:
		return GRAVEL;
	case 4:
		return ANDESITE;
	default:
		return named(editor, {"cobblestone_slab"}, COBBLESTONE);
	}
}

struct FireSpot
{
	Column column;
	int y;
	std::uint64_t hash;
};

void rubble_or_fire(WorldEditor &editor, const Column &c, int y, std::uint64_t h,
		std::vector<FireSpot> &fires)
{
	if (((h >> 24) % 100) == 0)
		fires.push_back({c, y, h});
	else
		put(editor, rubble(editor, h), c.x, y, c.z);
}

bool fire_fits(WorldEditor &editor, const Column &c, int y, int bx, int bz, int max_y)
{
	if (y + 1 > max_y || c.x - bx <= 0 || c.x - bx >= 15 || c.z - bz <= 0 ||
			c.z - bz >= 15 || editor.block_at(c.x, y, c.z) ||
			editor.block_at(c.x, y + 1, c.z))
		return false;
	for (int dz = -1; dz <= 1; ++dz)
		for (int dx = -1; dx <= 1; ++dx)
			for (int yy = y; yy <= std::min(max_y, y + 5); ++yy) {
				const auto b = editor.get_block_absolute(c.x + dx, yy, c.z + dz);
				if (b && flammable(editor, *b))
					return false;
			}
	return true;
}

void light_fire(WorldEditor &editor, const FireSpot &spot, int bx, int bz, int max_y)
{
	const auto netherrack = named(editor, {"netherrack"}, ANDESITE);
	const auto fire = named(editor, {"fire"}, AIR);
	if (fire.id() != AIR.id() && fire_fits(editor, spot.column, spot.y, bx, bz, max_y)) {
		put(editor, netherrack, spot.column.x, spot.y, spot.column.z);
		put(editor, fire, spot.column.x, spot.y + 1, spot.column.z);
	} else if (!editor.block_at(spot.column.x, spot.y, spot.column.z)) {
		put(editor, rubble(editor, spot.hash), spot.column.x, spot.y, spot.column.z);
	}
}

void ruin_building(WorldEditor &editor, const Column &c, int min_y, int max_y,
		std::vector<FireSpot> &fires)
{
	const auto top_block = top(editor, c, min_y, max_y);
	if (!top_block)
		return;
	const int top_y = top_block->first;
	const int height = top_y - c.ground;
	if (height < 2)
		return;
	const double standing = noise(c.x, c.z, 26, SALT_COLLAPSE);
	const double share = standing > .72 ? 1.0 : .1 + standing / .72 * .8;
	const int cut =
			c.ground +
			std::clamp(
					static_cast<int>(std::round(
							height * share + (noise(c.x, c.z, 5, SALT_JAG) - .5) * 4.0)),
					1, height);
	for (int y = cut + 1; y <= top_y; ++y)
		put(editor, AIR, c.x, y, c.z);
	for (int y = c.ground + 1; y <= cut; ++y) {
		const auto block = editor.get_block_absolute(c.x, y, c.z);
		if (!block || kind(editor, *block) == Kind::Air)
			continue;
		const auto k = kind(editor, *block);
		const auto h = col_hash(c, y);
		const auto roll = h % 100;
		const bool hole = noise(c.x + y * 3, c.z - y * 2, 4, SALT_HOLE) < .1;
		if (hole || k == Kind::Light || (k == Kind::Glass && roll >= 4 && roll < 85)) {
			put(editor, AIR, c.x, y, c.z);
		} else if (k == Kind::Glass && roll < 4) {
			swap_material(editor, c, y, *block, COBWEB);
		} else if (k == Kind::Solid && y - c.ground <= MOSS_REACH &&
				   noise(c.x + y, c.z - y, 5, SALT_MOSS) <
						   .22 * (1.0 - double(y - c.ground - 1) / MOSS_REACH)) {
			swap_material(editor, c, y, *block, MOSS_BLOCK);
		} else {
			const auto variant = weathered(editor, *block, h);
			if (variant.id() != block->id())
				swap_material(editor, c, y, *block, variant);
		}
	}
	if (cut < top_y && cut < max_y) {
		const auto cap = editor.get_block_absolute(c.x, cut, c.z);
		if (cap && kind(editor, *cap) == Kind::Solid) {
			const auto h = col_hash(c, cut + 1);
			if (h % 6 < 2)
				rubble_or_fire(editor, c, cut + 1, col_hash(c, cut + 2), fires);
			else if (h % 6 == 2)
				put(editor, MOSS_CARPET, c.x, cut + 1, c.z);
		}
	}
}

void grow(WorldEditor &editor, const Column &c, int y, std::uint64_t h, unsigned percent,
		int max_y)
{
	if (h % 100 >= percent || editor.block_at(c.x, y, c.z))
		return;
	switch ((h >> 7) % 10) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		put(editor, GRASS, c.x, y, c.z);
		break;
	case 6:
	case 7:
	case 8:
		if (y < max_y) {
			put(editor, TALL_GRASS_BOTTOM, c.x, y, c.z);
			put(editor, TALL_GRASS_TOP, c.x, y + 1, c.z);
		}
		break;
	default:
		put(editor, DEAD_BUSH, c.x, y, c.z);
		break;
	}
}

void ruin_open_ground(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const BuildingFootprintBitmap &footprints, std::vector<FireSpot> &fires)
{
	const auto surface = top(editor, c, min_y, max_y);
	if (!surface)
		return;
	const auto [top_y, block] = *surface;
	for (int y = std::min(top_y, c.ground + LAMP_REACH); y > c.ground; --y) {
		const auto b = editor.get_block_absolute(c.x, y, c.z);
		if (b && kind(editor, *b) == Kind::Light)
			put(editor, AIR, c.x, y, c.z);
	}
	if (top_y != c.ground || kind(editor, block) != Kind::Solid || top_y + 1 > max_y)
		return;
	const auto h = col_hash(c, top_y);
	bool near_ruin = false;
	for (int dz = -RUBBLE_REACH; dz <= RUBBLE_REACH && !near_ruin; ++dz)
		for (int dx = -RUBBLE_REACH; dx <= RUBBLE_REACH; ++dx)
			near_ruin |= footprints.contains(c.x + dx, c.z + dz);
	if (near_ruin && noise(c.x, c.z, 6, SALT_RUBBLE) < .45 && h % 3 == 0) {
		rubble_or_fire(editor, c, top_y + 1, h >> 8, fires);
		return;
	}
	if (editor.surface_is_sealed(c.x, c.z) && noise(c.x, c.z, 7, SALT_CRACK) < .32) {
		Block ground;
		switch (h % 10) {
		case 0:
		case 1:
		case 2:
		case 3:
			ground = GRASS_BLOCK;
			break;
		case 4:
		case 5:
			ground = COARSE_DIRT;
			break;
		case 6:
		case 7:
			ground = GRAVEL;
			break;
		case 8:
			ground = MOSS_BLOCK;
			break;
		default:
			ground = COBBLESTONE;
			break;
		}
		swap_material(editor, c, top_y, block, ground);
		if (ground.id() == GRASS_BLOCK.id() || ground.id() == MOSS_BLOCK.id())
			grow(editor, c, top_y + 1, h >> 8, 70, max_y);
	} else if (block.id() == GRASS_BLOCK.id()) {
		grow(editor, c, top_y + 1, h >> 8, 30, max_y);
	}
}

void hang_vines(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const BuildingFootprintBitmap &footprints, int bx, int bz)
{
	const double patch = noise(c.x, c.z, 9, SALT_VINE);
	if (patch > VINE_SHARE)
		return;
	const int height = 2 + int((VINE_SHARE - patch) / VINE_SHARE * 14.0);
	constexpr int dx[] = {0, 1, 0, -1};
	constexpr int dz[] = {-1, 0, 1, 0};
	constexpr const char *facing[] = {"north", "east", "south", "west"};
	const auto vine = named(editor, {"vine"}, AIR);
	if (vine.id() == AIR.id())
		return;
	for (int i = 0; i < 4; ++i) {
		const int wx = c.x + dx[i], wz = c.z + dz[i];
		if (wx < bx || wx >= bx + 16 || wz < bz || wz >= bz + 16 ||
				!footprints.contains(wx, wz))
			continue;
		const auto wall = top(editor, {wx, wz, c.ground}, min_y, max_y);
		const int wall_top = wall ? wall->first : c.ground;
		const int end_y = std::min({c.ground + height, wall_top, max_y});
		for (int y = c.ground + 1; y <= end_y; ++y) {
			const auto wb = editor.get_block_absolute(wx, y, wz);
			if (wb && kind(editor, *wb) == Kind::Solid && !editor.block_at(c.x, y, c.z)) {
				BlockWithProperties state{
						vine, BlockWithProperties::Properties{{facing[i], "true"}}};
				put(editor, state, c.x, y, c.z);
			}
		}
		break;
	}
}

void snow_over(WorldEditor &editor, const Column &c, int min_y, int max_y)
{
	const auto surface = top(editor, c, min_y, max_y);
	if (!surface)
		return;
	const auto [y, block] = *surface;
	const bool on_ground = y <= c.ground + 1;
	const auto k = kind(editor, block);
	if (k == Kind::Water) {
		const int reach = 5 + int(noise(c.x, c.z, 16, SALT_ICE) * 10.0);
		const auto d = editor.water_distance(c.x, c.z);
		if (!editor.is_lc_water(c.x, c.z) || (d != 0 && d <= reach))
			swap_material(editor, c, y, block, named(editor, {"ice"}, ICE));
		return;
	}
	if (k == Kind::Plant || k == Kind::TallPlant) {
		int snow_y = y;
		if (k == Kind::TallPlant) {
			const auto below = editor.get_block_absolute(c.x, y - 1, c.z);
			if (below && (below->id() == block.id() ||
								 kind(editor, *below) == Kind::TallPlant)) {
				put(editor, AIR, c.x, y, c.z);
				--snow_y;
			}
		}
		put(editor, snow_layer_with_depth(snow_depth(c, on_ground, false)), c.x, snow_y,
				c.z);
		snowy_ground(editor, c, snow_y - 1);
		return;
	}
	if (k != Kind::Leaves && k != Kind::Solid && k != Kind::Shaped)
		return;
	if (y >= max_y)
		return;
	const bool road = on_ground && editor.surface_is_sealed(c.x, c.z);
	if (road && noise(c.x, c.z, 3, SALT_PLOW) < .92)
		return;
	if (k == Kind::Shaped) {
		if (!full_top(block)) {
			if (noise(c.x, c.z, 10, SALT_ROOF) < WHITE_ROOF_SHARE) {
				const auto full_name = editor.block_name(block).value_or("");
				const auto n = short_name(full_name);
				Block white = block;
				if (n.ends_with("_stairs"))
					white = named(editor, {"smooth_quartz_stairs"}, block);
				else if (n.ends_with("_slab"))
					white = named(editor, {"smooth_quartz_slab"}, block);
				if (white.id() != block.id())
					swap_material(editor, c, y, block, white);
			}
			return;
		}
	}
	const unsigned layers =
			road || k == Kind::Leaves ? 1 : snow_depth(c, on_ground, false);
	put(editor, snow_layer_with_depth(layers), c.x, y + 1, c.z);
	snowy_ground(editor, c, y);
}

void style_chunk(WorldEditor &editor, int bx, int bz, int min_x, int min_z, int max_x,
		int max_z, int min_y, int max_y, const Args &args,
		const BuildingFootprintBitmap &footprints)
{
	std::vector<Column> columns;
	columns.reserve(256);
	for (int lz = 0; lz < 16; ++lz)
		for (int lx = 0; lx < 16; ++lx) {
			const int x = bx + lx, z = bz + lz;
			if (x >= min_x && x <= max_x && z >= min_z && z <= max_z)
				columns.push_back({x, z, editor.get_ground_level(x, z)});
		}
	if (args.post_apocalyptic) {
		std::vector<FireSpot> fires;
		for (const auto &c : columns) {
			if (footprints.contains(c.x, c.z))
				ruin_building(editor, c, min_y, max_y, fires);
			else
				ruin_open_ground(editor, c, min_y, max_y, footprints, fires);
		}
		for (const auto &c : columns)
			if (!footprints.contains(c.x, c.z))
				hang_vines(editor, c, min_y, max_y, footprints, bx, bz);
		for (const auto &fire : fires)
			light_fire(editor, fire, bx, bz, max_y);
	}
	if (args.winter)
		for (const auto &c : columns)
			snow_over(editor, c, min_y, max_y);
}
}

void apply(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints)
{
	if ((!args.winter && !args.post_apocalyptic) || !editor.mg)
		return;
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	const auto [min_y, max_y] = editor.writable_y_bounds();
	if (min_x > max_x || min_z > max_z || min_y > max_y)
		return;
	const int first_cx = min_x >> 4, last_cx = max_x >> 4;
	const int first_cz = min_z >> 4, last_cz = max_z >> 4;
	for (int cz = first_cz; cz <= last_cz; ++cz)
		for (int cx = first_cx; cx <= last_cx; ++cx)
			style_chunk(editor, cx << 4, cz << 4, min_x, min_z, max_x, max_z, min_y,
					max_y, args, building_footprints);
}
}
