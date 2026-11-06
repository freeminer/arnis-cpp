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
#include <map>
#include <unordered_map>
#include <mutex>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace arnis::world_style
{
bool is_active(const Args &args)
{
	return args.winter || args.post_apocalyptic;
}

namespace
{
constexpr std::uint32_t SALT_JAG = 0x4A41'4747, SALT_HOLE = 0x484F'4C45,
						SALT_CRACK = 0x4352'4B53, SALT_RUBBLE = 0x5242'4C45,
						SALT_DEPTH = 0x534E'4F57, SALT_PLOW = 0x504C'4F57,
						SALT_ICE = 0x4943'4521, SALT_VINE = 0x5649'4E45,
						SALT_MOSS = 0x4D4F'5353, SALT_ROOF = 0x524F'4F46,
						SALT_DECAY = 0x4445'4359, SALT_BREAK = 0x4252'4B31,
						SALT_FLOORS = 0x464C'5253, SALT_ROOF_HOLE = 0x5248'4F4C,
						SALT_GROWTH = 0x4752'4F57, SALT_WRECK = 0x5743'4152;
constexpr int RUBBLE_REACH = 3, FIRE_ODDS = 500, MOSS_REACH = 9, CORNER_WALK = 512,
			  TOWER_HEIGHT = 40, RUIN_TREE_ODDS = 170, WRECK_ODDS = 5, LAMP_REACH = 12;
constexpr double VINE_SHARE = .45, WHITE_ROOF_SHARE = .85, VINE_REACH = 34.0,
				 TOWER_SHARE = .6;

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

Kind kind_uncached(WorldEditor &editor, const Block &block)
{
	const auto full = editor.block_name(block);
	if (!full)
		return Kind::Solid;
	const auto n = short_name(*full);
	if (n == "air" || n == "cave_air" || n == "void_air" || n == "light" ||
			n == "barrier" || n == "structure_void")
		return Kind::Air;
	// Rust classifies the canonical WATER block by id. Backends expose that
	// same liquid under several node names, so recognize their semantic suffixes
	// here instead of making winter styling depend on the selected game's alias.
	if (n == "water" || n == "water_source" || n == "flowing_water" ||
			n == "water_flowing" || n == "river_water_source" ||
			n == "flowing_river_water" || n == "river_water_flowing" ||
			n == "bubble_column")
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
	if (std::find(plants.begin(), plants.end(), n) != plants.end() || n == "grass_3" ||
			n.ends_with("_sapling"))
		return Kind::Plant;
	if (n.ends_with("_leaves") || n == "jungleleaves" || n == "pine_needles")
		return Kind::Leaves;
	if (n.find("glass") != std::string_view::npos)
		return Kind::Glass;
	if (n == "fire" || n == "soul_fire" || n == "glowstone" || n == "sea_lantern" ||
			n == "shroomlight" || n == "redstone_lamp" || n == "jack_o_lantern" ||
			n == "end_rod" || n.ends_with("lantern") || n.ends_with("froglight") ||
			n.find("torch") != std::string_view::npos || n.ends_with("candle"))
		return Kind::Light;
	static constexpr std::array<std::string_view, 51> thin = {"fence", "_wall", "door",
			"sign", "banner", "button", "lever", "rail", "carpet", "pressure_plate",
			"ladder", "vine", "chain", "bars", "_head", "skull", "potted", "flower_pot",
			"bell", "anvil", "_bed", "chest", "cake", "campfire", "hopper", "cauldron",
			"grindstone", "lectern", "scaffolding", "cobweb", "lily_pad", "kelp",
			"seagrass", "coral", "pickle", "lichen", "bamboo", "sugar_cane", "cactus",
			"dripstone", "dripleaf", "azalea", "amethyst_bud", "amethyst_cluster",
			"sculk_vein", "sculk_sensor", "sculk_shrieker", "daylight_detector",
			"brewing_stand", "composter", "papyrus"};
	if (std::any_of(thin.begin(), thin.end(),
				[&](auto part) { return n.find(part) != std::string_view::npos; }))
		return Kind::Thin;
	if (n.ends_with("_stairs") || n.ends_with("_slab"))
		return Kind::Shaped;
	return Kind::Solid;
}

struct BlockCacheKey
{
	const void *definitions;
	std::uint16_t id;
	bool operator==(const BlockCacheKey &) const = default;
};

struct BlockCacheKeyHash
{
	std::size_t operator()(const BlockCacheKey &key) const noexcept
	{
		const auto pointer = std::hash<const void *>{}(key.definitions);
		return pointer ^ (std::hash<std::uint16_t>{}(key.id) + 0x9e3779b9u +
								 (pointer << 6) + (pointer >> 2));
	}
};

const void *definitions_key(const WorldEditor &editor)
{
	return editor.mg && editor.mg->m_emerge ? editor.mg->m_emerge->ndef : nullptr;
}

Kind kind(WorldEditor &editor, const Block &block)
{
	const auto *definitions = definitions_key(editor);
	if (!definitions)
		return kind_uncached(editor, block);
	static std::mutex mutex;
	static std::unordered_map<BlockCacheKey, Kind, BlockCacheKeyHash> cache;
	const BlockCacheKey key{definitions, block.id()};
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (const auto it = cache.find(key); it != cache.end())
			return it->second;
	}
	const auto value = kind_uncached(editor, block);
	std::lock_guard<std::mutex> lock(mutex);
	return cache.emplace(key, value).first->second;
}

bool flammable(WorldEditor &editor, const Block &block)
{
	const auto *definitions = definitions_key(editor);
	static std::mutex mutex;
	static std::unordered_map<BlockCacheKey, bool, BlockCacheKeyHash> cache;
	const BlockCacheKey key{definitions, block.id()};
	if (definitions) {
		std::lock_guard<std::mutex> lock(mutex);
		if (const auto it = cache.find(key); it != cache.end())
			return it->second;
	}
	const auto full = editor.block_name(block);
	const auto n = full ? short_name(*full) : std::string_view{};
	bool result = kind(editor, block) == Kind::Plant ||
				  kind(editor, block) == Kind::TallPlant ||
				  kind(editor, block) == Kind::Leaves;
	static constexpr std::array<std::string_view, 27> parts = {"oak", "spruce", "birch",
			"jungle", "acacia", "mangrove", "cherry", "bamboo", "planks", "log", "wood",
			"wool", "carpet", "vine", "hay", "bookshelf", "barrel", "scaffolding",
			"lectern", "composter", "beehive", "bee_nest", "target", "tnt", "coal_block",
			"moss", "azalea"};
	result = result || n == "tree" || n == "jungletree" || n.ends_with("_tree") ||
			 std::any_of(parts.begin(), parts.end(),
					 [&](auto part) { return n.find(part) != std::string_view::npos; });
	if (definitions) {
		std::lock_guard<std::mutex> lock(mutex);
		return cache.emplace(key, result).first->second;
	}
	return result;
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
	int search_max = max_y;
	while (search_max >= min_y) {
		const auto y = editor.highest_block_between(c.x, c.z, min_y, search_max);
		if (!y)
			return std::nullopt;
		if (const auto block = editor.get_block_absolute(c.x, *y, c.z);
				block && kind(editor, *block) != Kind::Air)
			return std::pair<int, Block>{*y, *block};
		if (*y <= min_y)
			break;
		search_max = *y - 1;
	}
	return std::nullopt;
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
		put(editor, SNOWY_GRASS_BLOCK, c.x, y, c.z);
	else if (block->id() == PODZOL.id())
		put(editor, SNOWY_PODZOL, c.x, y, c.z);
	else if (block->id() == FARMLAND.id() || block->id() == DIRT_PATH.id() ||
			 block->id() == MUD.id())
		put(editor, DIRT, c.x, y, c.z);
}

bool full_top(WorldEditor &editor, const Block &block, int x, int y, int z)
{
	if (editor.has_full_top_state(x, y, z))
		return true;
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
	// Rust's Column::swap keeps the generated block's actual per-cell state;
	// fall back to the block definition's baked-in state when none was emitted.
	if (const auto previous = editor.get_block_with_properties_absolute(c.x, y, c.z))
		put(editor, BlockWithProperties{replacement, previous->property_values()}, c.x, y,
				c.z);
	else
		put(editor, replacement, c.x, y, c.z);
}

Block weathered(WorldEditor &editor, const Block &b, std::uint64_t h)
{
	const auto roll = h % 100;
	const std::pair<const Block *, const char *> weather[] = {
			{&STONE_BRICK_STAIRS, "mossy_stone_brick_stairs"},
			{&STONE_BRICK_SLAB, "mossy_stone_brick_slab"},
			{&STONE_BRICK_WALL, "mossy_stone_brick_wall"},
			{&COBBLESTONE, "mossy_cobblestone"},
			{&COBBLESTONE_STAIRS, "mossy_cobblestone_stairs"},
			{&COBBLESTONE_SLAB, "mossy_cobblestone_slab"},
			{&COBBLESTONE_WALL, "mossy_cobblestone_wall"},
			{&DEEPSLATE_BRICKS, "cracked_deepslate_bricks"},
			{&DEEPSLATE_TILES, "cracked_deepslate_tiles"},
			{&POLISHED_BLACKSTONE_BRICKS, "cracked_polished_blackstone_bricks"},
			{&NETHER_BRICK, "cracked_nether_bricks"}, {&CALCITE, "tuff"},
			{&DIORITE, "cobblestone"}};
	if (b == STONE_BRICKS && roll < 30)
		return named(editor, {"cracked_stone_bricks"}, b);
	if (b == STONE_BRICKS)
		return named(editor, {"mossy_stone_bricks"}, roll < 55 ? MOSSY_STONE_BRICKS : b);
	for (const auto &[block, name] : weather) {
		const unsigned share =
				block == &CALCITE || block == &DIORITE
						? 18
						: (block == &STONE_BRICK_STAIRS || block == &STONE_BRICK_SLAB ||
												  block == &STONE_BRICK_WALL ||
												  block == &COBBLESTONE_STAIRS ||
												  block == &COBBLESTONE_SLAB
										  ? 35
										  : 40);
		if (b == *block && roll < share)
			return named(editor, {name}, b);
	}
	if ((b == STONE || b == SMOOTH_STONE || b == POLISHED_ANDESITE) && roll < 18)
		return roll < 12 ? COBBLESTONE : MOSSY_COBBLESTONE;
	if ((b == WHITE_CONCRETE || b == GRAY_CONCRETE) && roll < 18)
		return named(editor, {"light_gray_concrete"}, b);
	if (b == LIGHT_GRAY_CONCRETE && roll < 12)
		return named(editor, {"gray_concrete"}, b);
	if ((b == WHITE_TERRACOTTA || b == BROWN_TERRACOTTA) && roll < 18)
		return b;
	if (b == GRAY_TERRACOTTA && roll < 16)
		return named(editor, {"brown_terracotta"}, b);
	if (b == RED_TERRACOTTA && roll < 12)
		return COARSE_DIRT;
	if (b == BLACK_CONCRETE && roll < 10)
		return named(editor, {"polished_deepslate"}, b);
	if (b == QUARTZ_BLOCK && roll < 15)
		return named(editor, {"smooth_quartz"}, b);
	if (b == SMOOTH_QUARTZ && roll < 10)
		return AIR;
	if (b == BRICK && roll < 22)
		return roll < 14 ? named(editor, {"granite"}, b) : COBBLESTONE;
	if (b == SMOOTH_SANDSTONE && roll < 40)
		return SANDSTONE;
	if (b == OAK_PLANKS && roll < 25)
		return named(editor, {"spruce_planks"}, b);
	if (b == TERRACOTTA && roll < 12)
		return COARSE_DIRT;
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

enum class Decay
{
	Shell,
	Roofless,
	Broken,
	Fallen
};

struct RuinPlan
{
	Decay decay;
	bool wall;
	int eave;
	int top;
};

struct RegionTops
{
	std::unordered_map<std::uint64_t, int> tops;
	static std::uint64_t key(int x, int z)
	{
		return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
	}
	void build(const BuildingFootprintBitmap &footprints, int min_x, int min_z, int max_x,
			int max_z, int min_y, int max_y, WorldEditor &editor)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x) {
				if (!footprints.contains(x, z))
					continue;
				if (const auto column_top = top(editor,
							Column{x, z, editor.get_ground_level(x, z)}, min_y, max_y))
					tops[key(x, z)] = column_top->first;
			}
	}
	std::optional<int> get(int x, int z) const
	{
		const auto it = tops.find(key(x, z));
		return it == tops.end() ? std::nullopt : std::optional(it->second);
	}
};

std::optional<std::pair<int, int>> corner_step(
		const BuildingFootprintBitmap &footprints, int x, int z)
{
	static constexpr int slide[] = {
			0, -1, 1, -2, 2, -3, 3, -4, 4, -5, 5, -6, 6, -7, 7, -8, 8};
	for (const int dz : slide) {
		bool inside = true;
		for (int t = std::min(z, z + dz); t <= std::max(z, z + dz); ++t)
			inside &= footprints.contains(x, t);
		if (inside && footprints.contains(x - 1, z + dz))
			return std::pair{x - 1, z + dz};
	}
	if (footprints.contains(x, z - 1))
		return std::pair{x, z - 1};
	return std::nullopt;
}

std::pair<int, int> building_corner(
		const BuildingFootprintBitmap &footprints, int x, int z)
{
	std::pair at{x, z};
	for (int i = 0; i < CORNER_WALK; ++i) {
		const auto next = corner_step(footprints, at.first, at.second);
		if (!next)
			break;
		at = *next;
	}
	return at;
}

Decay decay_of(std::pair<int, int> corner, int x, int z)
{
	const double own =
			((land_cover::coord_hash(corner.first, corner.second) >> 16) % 1000) / 1000.0;
	const double value = .75 * own + .25 * noise(x, z, 40, SALT_DECAY);
	if (value < .17)
		return Decay::Fallen;
	if (value < .39)
		return Decay::Broken;
	if (value < .72)
		return Decay::Roofless;
	return Decay::Shell;
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
	if (((h >> 24) % FIRE_ODDS) == 0)
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

using RuinPlans = std::array<std::array<std::optional<RuinPlan>, 16>, 16>;

RuinPlans plan_ruins(const std::vector<Column> &columns,
		const BuildingFootprintBitmap &footprints, const RegionTops &tops)
{
	RuinPlans plans;
	const auto wall = [&](int x, int z) {
		return footprints.contains(x, z) &&
			   (!footprints.contains(x + 1, z) || !footprints.contains(x - 1, z) ||
					   !footprints.contains(x, z + 1) || !footprints.contains(x, z - 1));
	};
	for (const auto &column : columns) {
		if (!footprints.contains(column.x, column.z))
			continue;
		const auto column_top = tops.get(column.x, column.z);
		if (!column_top)
			continue;
		const bool is_wall = wall(column.x, column.z);
		int eave = *column_top;
		if (!is_wall)
			for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
				for (int reach = 1; reach <= 48; ++reach) {
					const int x = column.x + dx * reach, z = column.z + dz * reach;
					if (!footprints.contains(x, z))
						break;
					if (wall(x, z)) {
						if (const auto y = tops.get(x, z))
							eave = std::min(eave, *y);
						break;
					}
				}
		const auto corner = building_corner(footprints, column.x, column.z);
		plans[column.z & 15][column.x & 15] = {
				decay_of(corner, column.x, column.z), is_wall, eave, *column_top};
	}
	return plans;
}
double hole_share(Decay d)
{
	return d == Decay::Shell ? .04 : d == Decay::Roofless ? .07 : .1;
}
double floor_share(Decay d)
{
	return d == Decay::Shell ? .12 : d == Decay::Roofless ? .3 : .55;
}
double rubble_share(Decay d)
{
	return d == Decay::Shell	  ? .04
		   : d == Decay::Roofless ? .16
		   : d == Decay::Broken	  ? .42
								  : .6;
}

int standing_height(const Column &c, int height, const RuinPlan &plan)
{
	const double line = noise(c.x, c.z, 11, SALT_BREAK) - .5;
	const double jag = (noise(c.x, c.z, 5, SALT_JAG) - .5) * 4;
	const auto of = [&](int h, double share) {
		return h * (height >= TOWER_HEIGHT ? std::max(share, TOWER_SHARE) : share);
	};
	double standing = 0;
	if (plan.wall)
		switch (plan.decay) {
		case Decay::Shell:
			standing = height;
			break;
		case Decay::Roofless:
			standing = of(height, .96 + line * .08) + jag;
			break;
		case Decay::Broken:
			standing = of(height, .62 + line * .5) + jag;
			break;
		case Decay::Fallen:
			standing = of(height, .22 + line * .3) + jag;
			break;
		}
	else {
		const int eave = std::min(plan.eave - c.ground, height);
		const double floors = noise(c.x, c.z, 9, SALT_FLOORS);
		switch (plan.decay) {
		case Decay::Shell:
			standing = noise(c.x, c.z, 7, SALT_ROOF_HOLE) < .22
							   ? height - 1 - col_hash(c, 0) % 3
							   : height;
			break;
		case Decay::Roofless:
			standing = eave - 1 - col_hash(c, 1) % 2;
			break;
		case Decay::Broken:
			standing = of(eave, .2 + floors * .4);
			break;
		case Decay::Fallen:
			standing = 0;
			break;
		}
	}
	return std::clamp(static_cast<int>(std::round(standing)), 1, height);
}
void grow(WorldEditor &, const Column &, int, std::uint64_t, unsigned, int);

void overgrow(WorldEditor &editor, const Column &c, int y, std::uint64_t h, int max_y)
{
	if (y >= max_y)
		return;
	const auto block = editor.get_block_absolute(c.x, y, c.z);
	if (!block)
		return;
	if (kind(editor, *block) == Kind::Solid) {
		put(editor, MOSS_BLOCK, c.x, y, c.z);
		grow(editor, c, y + 1, h >> 8, 55, max_y);
	} else if (kind(editor, *block) == Kind::Shaped && h % 2 == 0)
		put(editor, MOSS_CARPET, c.x, y + 1, c.z);
}
Block debris(WorldEditor &editor, const std::optional<Block> &material, std::uint64_t h)
{
	return material && h % 5 < 2 ? *material : rubble(editor, h >> 3);
}

void pile_up(WorldEditor &editor, const Column &c, int y, int height,
		const std::optional<Block> &material, std::uint64_t h, int max_y,
		std::vector<FireSpot> &fires)
{
	for (int i = 0; i < height; ++i) {
		const int at = y + i;
		if (at > max_y || editor.block_at(c.x, at, c.z))
			return;
		const auto mix = h + std::uint64_t(std::uint32_t(i)) * 0x9E3779B9ULL;
		if (i == height - 1) {
			if (mix % 3 == 0)
				put(editor, COBBLESTONE_SLAB, c.x, at, c.z);
			else
				rubble_or_fire(editor, c, at, mix, fires);
		} else
			put(editor, debris(editor, material, mix), c.x, at, c.z);
	}
}
void grow_tree(WorldEditor &editor, const Column &c, int bx, int bz, int surface,
		std::uint64_t h, int max_y)
{
	if (c.x - bx < 2 || c.x - bx > 13 || c.z - bz < 2 || c.z - bz > 13)
		return;
	const int trunk = 3 + ((h >> 4) % 3), crown = surface + trunk;
	if (crown + 2 > max_y)
		return;
	for (int dz = -2; dz <= 2; ++dz)
		for (int dx = -2; dx <= 2; ++dx)
			for (int y = surface + 1; y <= crown + 1; ++y)
				if (editor.block_at(c.x + dx, y, c.z + dz))
					return;
	for (int y = surface + 1; y <= crown; ++y)
		put(editor, OAK_LOG, c.x, y, c.z);
	for (int dz = -2; dz <= 2; ++dz)
		for (int dx = -2; dx <= 2; ++dx)
			for (const auto &[y, reach] :
					{std::pair{crown - 1, 3}, {crown, 3}, {crown + 1, 1}})
				if (std::abs(dx) + std::abs(dz) <= reach &&
						!(dx == 0 && dz == 0 && y <= crown))
					put(editor, OAK_LEAVES, c.x + dx, y, c.z + dz);
}

void ruin_building(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const RuinPlan &plan, int bx, int bz, std::vector<FireSpot> &fires)
{
	const auto top_block = top(editor, c, min_y, max_y);
	if (!top_block)
		return;
	const int top_y = top_block->first, height = top_y - c.ground;
	if (height < 2)
		return;
	const int cut = c.ground + standing_height(c, height, plan);
	std::optional<Block> material;
	for (int y = top_y; y > c.ground && !material; --y) {
		const auto b = editor.get_block_absolute(c.x, y, c.z);
		if (b && kind(editor, *b) == Kind::Solid)
			material = b;
	}
	int fallen = top_y - cut;
	for (int y = cut + 1; y <= top_y; ++y)
		put(editor, AIR, c.x, y, c.z);
	for (int y = c.ground + 1; y <= cut; ++y) {
		const auto block = editor.get_block_absolute(c.x, y, c.z);
		if (!block)
			continue;
		const auto k = kind(editor, *block);
		const auto h = col_hash(c, y);
		const auto roll = h % 100;
		const bool hole =
				noise(c.x + y * 3, c.z - y * 2, 4, SALT_HOLE) < hole_share(plan.decay);
		const auto below = editor.get_block_absolute(c.x, y - 1, c.z);
		const bool floor_gone = !plan.wall && y > c.ground + 1 &&
								(k == Kind::Solid || k == Kind::Shaped) && !below &&
								noise(c.x, c.z, 6, SALT_FLOORS + std::uint32_t(y)) <
										floor_share(plan.decay);
		if (floor_gone) {
			++fallen;
			put(editor, AIR, c.x, y, c.z);
			if (y < cut) {
				const auto above = editor.get_block_absolute(c.x, y + 1, c.z);
				if (above && kind(editor, *above) != Kind::Solid)
					put(editor, AIR, c.x, y + 1, c.z);
			}
		} else if (hole || k == Kind::Light ||
				   (k == Kind::Glass && roll >= 4 && roll < 85)) {
			put(editor, AIR, c.x, y, c.z);
		} else if (k == Kind::Glass && roll < 4) {
			put(editor, COBWEB, c.x, y, c.z);
		} else if (k == Kind::Solid && y - c.ground <= MOSS_REACH &&
				   noise(c.x + y, c.z - y, 5, SALT_MOSS) <
						   .22 * (1 - double(y - c.ground - 1) / MOSS_REACH)) {
			put(editor, MOSS_BLOCK, c.x, y, c.z);
		} else {
			const auto variant = weathered(editor, *block, h);
			if (variant != *block)
				swap_material(editor, c, y, *block, variant);
		}
	}
	if (plan.wall) {
		if (fallen > 0 && cut < max_y) {
			const auto cap = editor.get_block_absolute(c.x, cut, c.z);
			if (cap && kind(editor, *cap) == Kind::Solid)
				switch (col_hash(c, cut + 1) % 6) {
				case 0:
					put(editor, rubble(editor, col_hash(c, cut + 2)), c.x, cut + 1, c.z);
					break;
				case 1:
				case 2:
					put(editor, MOSS_CARPET, c.x, cut + 1, c.z);
					break;
				}
		}
		return;
	}
	const auto h = col_hash(c, cut + 3);
	const double lying = noise(c.x, c.z, 7, SALT_RUBBLE);
	const int pile = plan.decay == Decay::Shell		 ? 0
					 : plan.decay == Decay::Roofless ? int(lying < .25)
					 : plan.decay == Decay::Broken	 ? int(fallen >= 3 ? lying * 3.5 : 0)
													 : 1 + int(lying * 4);
	int floor = max_y;
	for (int y = c.ground + 1; y <= max_y; ++y)
		if (!editor.block_at(c.x, y, c.z)) {
			floor = y;
			break;
		}
	if (pile > 0)
		pile_up(editor, c, floor, pile, material, h, max_y, fires);
	if (plan.decay == Decay::Shell) {
		if (fallen > 0 && col_hash(c, cut + 1) % 4 == 0)
			overgrow(editor, c, cut, h, max_y);
		return;
	}
	const auto surface = top(editor, c, min_y, max_y);
	if (!surface)
		return;
	if (noise(c.x, c.z, 8, SALT_GROWTH) < .6)
		overgrow(editor, c, surface->first, h, max_y);
	if (plan.decay != Decay::Roofless && (h >> 20) % RUIN_TREE_ODDS == 0)
		grow_tree(editor, c, bx, bz, surface->first, h, max_y);
}
void drop_overhangs(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const RuinPlans &plans)
{
	const auto surface = top(editor, c, min_y, max_y);
	if (!surface)
		return;
	const int top_y = surface->first;
	const auto solid = [&](int y) {
		const auto block = editor.get_block_absolute(c.x, y, c.z);
		return block && (kind(editor, *block) == Kind::Solid ||
								kind(editor, *block) == Kind::Shaped);
	};
	if (top_y <= c.ground + 2 || !solid(top_y))
		return;
	int bottom = top_y;
	while (bottom > top_y - 1 && solid(bottom - 1))
		--bottom;
	if (editor.block_at(c.x, bottom - 1, c.z) || bottom <= c.ground + 2)
		return;
	for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
		const int x = c.x + dx, z = c.z + dz;
		const auto &plan = plans[z & 15][x & 15];
		if (!plan)
			continue;
		const auto now = top(editor, Column{x, z, c.ground}, min_y, max_y);
		if (top_y <= plan->top + 1 && bottom > (now ? now->first : c.ground)) {
			for (int y = bottom; y <= top_y; ++y)
				put(editor, AIR, c.x, y, c.z);
			return;
		}
	}
}

static Block street_bush(std::uint64_t h)
{
	return (h >> 24) % 3 == 0 ? BIRCH_LEAVES : OAK_LEAVES;
}

void wreck_a_car(WorldEditor &editor, int bx, int bz, const std::vector<Column> &columns,
		int min_y, int max_y, const BuildingFootprintBitmap &footprints)
{
	const auto h = land_cover::coord_hash(bx >> 4, bz >> 4);
	if (h % WRECK_ODDS != 0)
		return;
	const int lx = 4 + (h >> 8) % 8, lz = 4 + (h >> 16) % 8;
	const auto found = std::find_if(columns.begin(), columns.end(),
			[&](const auto &c) { return (c.x - bx) == lx && (c.z - bz) == lz; });
	if (found == columns.end())
		return;
	const auto centre = *found;
	const auto run = [&](int dx, int dz) {
		int n = 0;
		for (int i = 1;
				i <= 8 && editor.surface_is_sealed(centre.x + dx * i, centre.z + dz * i);
				++i)
			++n;
		return n;
	};
	const bool along_x = run(1, 0) + run(-1, 0) >= run(0, 1) + run(0, -1);
	const int ax = along_x ? 1 : 0, az = 1 - ax;
	const auto cell = [&](int along, int across) {
		return Column{centre.x + along * ax + across * az,
				centre.z + along * az + across * ax, centre.ground};
	};
	if (centre.ground + 3 > max_y)
		return;
	for (int along = -2; along <= 1; ++along)
		for (int across = 0; across <= 1; ++across) {
			const auto c = cell(along, across);
			if (!editor.surface_is_sealed(c.x, c.z) || footprints.contains(c.x, c.z) ||
					editor.get_ground_level(c.x, c.z) != centre.ground)
				return;
			const auto surface = top(editor, c, min_y, max_y);
			if (!surface || surface->first != centre.ground ||
					kind(editor, surface->second) != Kind::Solid)
				return;
		}
	static const Block paint[] = {
			BROWN_TERRACOTTA, GRAY_TERRACOTTA, RED_TERRACOTTA, BLACK_CONCRETE};
	for (int along = -2; along <= 1; ++along)
		for (int across = 0; across <= 1; ++across) {
			const auto c = cell(along, across);
			put(editor, paint[(h >> 24) % 4], c.x, centre.ground + 1, c.z);
			if (along == -1 || along == 0)
				put(editor, TINTED_GLASS, c.x, centre.ground + 2, c.z);
			else if ((h >> 28) % 2 == 0)
				put(editor, MOSS_CARPET, c.x, centre.ground + 2, c.z);
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

std::optional<std::tuple<int, int, int>> nearest_footprint(
		const BuildingFootprintBitmap &footprints, int x, int z)
{
	for (int distance = 1; distance <= RUBBLE_REACH; ++distance)
		for (int dz = -distance; dz <= distance; ++dz)
			for (int dx = -distance; dx <= distance; ++dx)
				if ((std::abs(dx) == distance || std::abs(dz) == distance) &&
						footprints.contains(x + dx, z + dz))
					return std::tuple{distance, x + dx, z + dz};
	return std::nullopt;
}

int heap_height(const Column &c, int distance, Decay decay)
{
	const double share = rubble_share(decay), patch = noise(c.x, c.z, 6, SALT_RUBBLE);
	if (patch >= share)
		return 0;
	const int most = RUBBLE_REACH + 1 - distance;
	return std::min(most, 1 + int((share - patch) / share * most));
}

void ruin_open_ground(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const BuildingFootprintBitmap &footprints, int bx, int bz,
		std::vector<FireSpot> &fires)
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
	if (const auto nearest = nearest_footprint(footprints, c.x, c.z)) {
		const auto &[distance, wx, wz] = *nearest;
		const auto corner = building_corner(footprints, wx, wz);
		const int height = heap_height(c, distance, decay_of(corner, wx, wz));
		if (height > 0) {
			put(editor, AIR, c.x, top_y, c.z);
			std::optional<Block> material;
			if (wx >= bx && wx < bx + 16 && wz >= bz && wz < bz + 16)
				for (int y = c.ground + 1; y <= c.ground + 4 && !material; ++y) {
					const auto b = editor.get_block_absolute(wx, y, wz);
					if (b && kind(editor, *b) == Kind::Solid)
						material = b;
				}
			pile_up(editor, c, top_y, height, material, h, max_y, fires);
			return;
		}
	}
	if (editor.surface_is_sealed(c.x, c.z)) {
		const double patch = noise(c.x, c.z, 7, SALT_CRACK);
		if (patch < .25) {
			put(editor, street_bush(h), c.x, top_y + 1, c.z);
		} else if (patch < .33 || patch < .48 && h % 4 == 0) {
			put(editor, COARSE_DIRT, c.x, top_y, c.z);
			put(editor, MOSS_CARPET, c.x, top_y + 1, c.z);
			grow(editor, c, top_y + 2, h >> 8, 45, max_y);
		} else if (patch < .46) {
			put(editor, COARSE_DIRT, c.x, top_y, c.z);
			grow(editor, c, top_y + 1, h >> 8, 60, max_y);
		} else if (patch < .52) {
			put(editor, GRASS_BLOCK, c.x, top_y, c.z);
			grow(editor, c, top_y + 1, h >> 8, 80, max_y);
		}
		return;
	}
	int overgrown = 0;
	for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
		const auto neighbour = editor.get_block_absolute(c.x + dx, top_y + 1, c.z + dz);
		if (neighbour && (*neighbour == MOSS_BLOCK || *neighbour == VINE))
			++overgrown;
	}
	if (overgrown >= 3)
		put(editor, MOSS_BLOCK, c.x, top_y, c.z);
	const auto moss = noise(c.x, c.z, 11, SALT_MOSS) < .75 ? h : h >> 8;
	if (noise(c.x, c.z, 8, SALT_GROWTH) < .55 ||
			(block.id() == GRASS_BLOCK.id() && c.ground % 3 == 0))
		grow(editor, c, top_y + 1, moss, 65, max_y);
}

void hang_vines(WorldEditor &editor, const Column &c, int min_y, int max_y,
		const BuildingFootprintBitmap &footprints, int bx, int bz)
{
	const double patch = noise(c.x, c.z, 9, SALT_VINE);
	if (patch > VINE_SHARE)
		return;
	const int height = 2 + int((VINE_SHARE - patch) / VINE_SHARE * VINE_REACH);
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

void snow_over(WorldEditor &editor, const Column &c, int min_y, int max_y,
		std::optional<Block> bare)
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
			put(editor, named(editor, {"ice"}, ICE), c.x, y, c.z);
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
	// Rust leaves the blueprint's outline marker visible in winter worlds;
	// otherwise ordinary snow placement would obscure the construction guide.
	if (on_ground && bare && block == *bare)
		return;
	if (y >= max_y)
		return;
	const bool road = on_ground && editor.surface_is_sealed(c.x, c.z);
	if (road && noise(c.x, c.z, 3, SALT_PLOW) < .92)
		return;
	if (k == Kind::Shaped) {
		if (!full_top(editor, block, c.x, y, c.z)) {
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
		const BuildingFootprintBitmap &footprints, const RegionTops *tops)
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
		const auto plans = tops ? plan_ruins(columns, footprints, *tops) : RuinPlans{};
		for (const auto &c : columns) {
			const auto &plan = plans[c.z & 15][c.x & 15];
			if (footprints.contains(c.x, c.z) && plan)
				ruin_building(editor, c, min_y, max_y, *plan, bx, bz, fires);
			else
				ruin_open_ground(editor, c, min_y, max_y, footprints, bx, bz, fires);
		}
		for (const auto &c : columns)
			if (!footprints.contains(c.x, c.z)) {
				drop_overhangs(editor, c, min_y, max_y, plans);
				hang_vines(editor, c, min_y, max_y, footprints, bx, bz);
			}
		wreck_a_car(editor, bx, bz, columns, min_y, max_y, footprints);
		for (const auto &fire : fires)
			light_fire(editor, fire, bx, bz, max_y);
	}
	if (args.winter)
		for (const auto &c : columns)
			snow_over(editor, c, min_y, max_y,
					args.mode == GenerationMode::Blueprint
							? std::optional<Block>(block_definitions::LIGHT_BLUE_CONCRETE)
							: std::nullopt);
}

int region_floor(int coordinate)
{
	int region = coordinate / 512;
	if (coordinate % 512 < 0)
		--region;
	return region;
}

void apply_chunks(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints, int min_x, int min_z,
		int max_x, int max_z, const std::set<std::pair<int, int>> *skip_regions)
{
	const auto [min_y, max_y] = editor.writable_y_bounds();
	if (min_x > max_x || min_z > max_z || min_y > max_y)
		return;
	const int first_cx = min_x >> 4, last_cx = max_x >> 4;
	const int first_cz = min_z >> 4, last_cz = max_z >> 4;
	std::map<std::pair<int, int>, RegionTops> region_tops;
	if (args.post_apocalyptic && !building_footprints.is_empty()) {
		for (int rz = min_z >> 9; rz <= max_z >> 9; ++rz)
			for (int rx = min_x >> 9; rx <= max_x >> 9; ++rx) {
				const std::pair key{rx, rz};
				auto &tops = region_tops[key];
				const int rx0 = rx * 512, rz0 = rz * 512;
				tops.build(building_footprints, std::max(min_x, rx0),
						std::max(min_z, rz0), std::min(max_x, rx0 + 511),
						std::min(max_z, rz0 + 511), min_y, max_y, editor);
			}
	}
	for (int cz = first_cz; cz <= last_cz; ++cz)
		for (int cx = first_cx; cx <= last_cx; ++cx) {
			const int bx = cx << 4, bz = cz << 4;
			if (skip_regions &&
					skip_regions->contains({region_floor(bx), region_floor(bz)}))
				continue;
			style_chunk(editor, bx, bz, min_x, min_z, max_x, max_z, min_y, max_y, args,
					building_footprints,
					args.post_apocalyptic
							? &region_tops[{region_floor(bx), region_floor(bz)}]
							: nullptr);
		}
}
}

void apply(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints)
{
	if (!is_active(args) || !editor.mg)
		return;
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	apply_chunks(editor, args, building_footprints, min_x, min_z, max_x, max_z, nullptr);
}

void apply_region(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints, int region_x, int region_z)
{
	if (!is_active(args) || !editor.mg)
		return;
	const auto [world_min_x, world_min_z] = editor.get_min_coords();
	const auto [world_max_x, world_max_z] = editor.get_max_coords();
	const std::int64_t region_min_x = std::int64_t(region_x) * 512;
	const std::int64_t region_min_z = std::int64_t(region_z) * 512;
	const auto min_x =
			static_cast<int>(std::max<std::int64_t>(world_min_x, region_min_x));
	const auto min_z =
			static_cast<int>(std::max<std::int64_t>(world_min_z, region_min_z));
	const auto max_x =
			static_cast<int>(std::min<std::int64_t>(world_max_x, region_min_x + 511));
	const auto max_z =
			static_cast<int>(std::min<std::int64_t>(world_max_z, region_min_z + 511));
	apply_chunks(editor, args, building_footprints, min_x, min_z, max_x, max_z, nullptr);
}

void apply_remaining(WorldEditor &editor, const Args &args,
		const BuildingFootprintBitmap &building_footprints,
		const std::set<std::pair<int, int>> &already_styled)
{
	if (!is_active(args) || !editor.mg)
		return;
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	apply_chunks(editor, args, building_footprints, min_x, min_z, max_x, max_z,
			&already_styled);
}
}
