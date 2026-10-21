#include <mutex>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "colors.h"
#include "block_palette.h"

#include "../../arnis_adapter.h"
namespace arnis
{

BlockWithProperties::Properties default_block_properties(const Block &block)
{
	using Properties = BlockWithProperties::Properties;
	const auto one = [](const char *key, const char *value) {
		return Properties{{key, value}};
	};
	switch (block.id()) {
	case 3:
	case 49:
	case 220:
	case 222:
	case 224:
	case 225:
	case 231:
	case 233:
	case 234:
	case 480:
		return one("persistent", "true");
	case 70:
	case 106:
	case 137:
	case 197:
	case 293:
	case 364:
	case 415:
	case 459:
	case 461:
	case 463:
	case 465:
		return one("half", "lower");
	case 93:
	case 107:
	case 138:
	case 198:
	case 299:
	case 365:
	case 366:
	case 416:
	case 460:
	case 462:
	case 464:
	case 466:
		return one("half", "upper");
	case 95:
		return {{"pickles", "2"}, {"waterlogged", "true"}};
	case 105:
	case 108:
	case 109:
		return one("age", "7");
	case 116:
		return one("shape", "north_south");
	case 117:
		return one("shape", "east_west");
	case 118:
		return one("shape", "ascending_east");
	case 119:
		return one("shape", "ascending_west");
	case 120:
		return one("shape", "ascending_north");
	case 121:
		return one("shape", "ascending_south");
	case 122:
		return one("shape", "north_east");
	case 123:
		return one("shape", "north_west");
	case 124:
		return one("shape", "south_east");
	case 125:
		return one("shape", "south_west");
	case 161:
		return {{"facing", "north"}, {"part", "head"}, {"occupied", "false"}};
	case 162:
		return {{"facing", "north"}, {"part", "foot"}, {"occupied", "false"}};
	case 165:
		return {{"facing", "south"}, {"part", "head"}, {"occupied", "false"}};
	case 166:
		return {{"facing", "south"}, {"part", "foot"}, {"occupied", "false"}};
	case 167:
		return {{"facing", "west"}, {"part", "head"}, {"occupied", "false"}};
	case 168:
		return {{"facing", "west"}, {"part", "foot"}, {"occupied", "false"}};
	case 182:
		return {{"north", "true"}, {"south", "true"}, {"east", "true"}, {"west", "true"},
				{"up", "true"}, {"down", "true"}};
	case 210:
		return one("axis", "x");
	case 214:
		return one("type", "bottom");
	case 217:
	case 240:
		return one("type", "top");
	case 236:
		return {{"facing", "north"}, {"open", "true"}, {"half", "top"}};
	case 274:
		return {{"rotation", "6"}, {"waterlogged", "false"}};
	case 294:
		return {{"facing", "east"}, {"part", "head"}, {"occupied", "false"}};
	case 295:
		return {{"facing", "east"}, {"part", "foot"}, {"occupied", "false"}};
	case 305:
		return one("facing", "north");
	case 316:
		return one("facing", "east");
	case 320:
		return one("facing", "south");
	case 326:
		return one("facing", "west");
	case 362:
		return one("axis", "z");
	case 471:
	case 472:
		return one("snowy", "true");
	case 474:
	case 475:
	case 476:
	case 477:
	case 478:
	case 479:
		return {{"layers", std::to_string(block.id() - 472)}};
	case 467:
		return one("age", "3");
	default:
		return {};
	}
}

std::optional<BlockProperties> Block::properties() const
{
	auto properties = default_block_properties(*this);
	if (properties.empty())
		return std::nullopt;
	return properties;
}

/*
enum class Block {
    GLASS,
    GRAY_STAINED_GLASS,
    LIGHT_GRAY_STAINED_GLASS,
    BROWN_STAINED_GLASS,
    WHITE_STAINED_GLASS,
    TINTED_GLASS,
    WHITE_CONCRETE,
    GRAY_CONCRETE,
    LIGHT_GRAY_CONCRETE,
    POLISHED_ANDESITE,
    SMOOTH_STONE,
    STONE_BRICKS,
    MUD_BRICKS,
    OAK_PLANKS,
    BRICK,
    NETHER_BRICK,
    POLISHED_BLACKSTONE_BRICKS,
    BLACKSTONE,
    DEEPSLATE_BRICKS,
    LIGHT_BLUE_TERRACOTTA,
    POLISHED_BLACKSTONE,
    END_STONE_BRICKS,
    SANDSTONE,
    SMOOTH_SANDSTONE,
    BROWN_TERRACOTTA,
    BROWN_CONCRETE,
    GRAY_TERRACOTTA,
    NETHERITE_BLOCK,
    POLISHED_DEEPSLATE,
    POLISHED_GRANITE,
    QUARTZ_BRICKS,
    BLUE_TERRACOTTA,
    QUARTZ_BLOCK,
    WHITE_TERRACOTTA,
    BLACK_TERRACOTTA,
    CHISELED_STONE_BRICKS,
    CRACKED_STONE_BRICKS,
    COBBLESTONE,
    MOSSY_COBBLESTONE,
    ANDESITE
};
struct RGB {
    int r;
    int g;
    int b;
};
*/

namespace colors
{
inline std::int64_t rgb_distance(const RGB &a, const RGB &b)
{
	/*
        std::int64_t dr = static_cast<std::int64_t>(a.r) - static_cast<std::int64_t>(b.r);
        std::int64_t dg = static_cast<std::int64_t>(a.g) - static_cast<std::int64_t>(b.g);
        std::int64_t db = static_cast<std::int64_t>(a.b) - static_cast<std::int64_t>(b.b);
        return dr * dr + dg * dg + db * db;
*/
	auto [ar, ag, ab] = a;
	auto [br, bg, bb] = b;
	// Simple Euclidean or squared difference
	int dr = int(ar) - int(br);
	int dg = int(ag) - int(bg);
	int db = int(ab) - int(bb);
	return dr * dr + dg * dg + db * db;
}
}

static std::mt19937 &global_rng()
{
	static std::random_device rd;
	static std::mt19937 rng(rd());
	return rng;
}

//template <typename BlockT, typename ValueT>
class _______BlockWithProperties
{
public:
	Block block;
	//std::optional<ValueT> properties;
	/*
    BlockWithProperties() = default;
    BlockWithProperties(BlockWithProperties const&) = default;
    BlockWithProperties(BlockWithProperties&&) = default;
    BlockWithProperties& operator=(BlockWithProperties const&) = default;
    BlockWithProperties& operator=(BlockWithProperties&&) = default;
    ~BlockWithProperties() = default;

    BlockWithProperties(BlockT const& block_, std::optional<ValueT> properties_)
        : block(block_), properties(std::move(properties_)) {}

    BlockWithProperties(BlockT&& block_, std::optional<ValueT> properties_)
        : block(std::move(block_)), properties(std::move(properties_)) {}

    static BlockWithProperties simple(BlockT const& block_) {
        return BlockWithProperties(block_, std::nullopt);
    }

    static BlockWithProperties simple(BlockT&& block_) {
        return BlockWithProperties(std::move(block_), std::nullopt);
    }
        */
};

struct StairKeyHash
{
	std::size_t operator()(
			std::tuple<content_t, StairFacing, StairShape> const &t) const noexcept
	{
		std::size_t h1 = std::hash<content_t>{}(std::get<0>(t));
		std::size_t h2 = std::hash<int>{}(static_cast<int>(std::get<1>(t)));
		std::size_t h3 = std::hash<int>{}(static_cast<int>(std::get<2>(t)));
		// boost::hash_combine style
		h1 ^= h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2);
		h1 ^= h3 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2);
		return h1;
	}
};

static std::mutex STAIR_CACHE_MUTEX;
static std::unordered_map<std::tuple<content_t, StairFacing, StairShape>,
		BlockWithProperties, StairKeyHash>
		STAIR_CACHE;
static std::mutex TOP_STAIR_CACHE_MUTEX;
static std::unordered_map<std::tuple<content_t, StairFacing, StairShape>,
		std::shared_ptr<const BlockWithProperties::Properties>, StairKeyHash>
		TOP_STAIR_CACHE;
struct Value
{
	static Value String(std::string);
	static Value Compound(std::unordered_map<std::string, Value>);
};

BlockWithProperties create_stair_with_properties(
		const Block &base_stair_block, StairFacing facing, StairShape shape)
{
	auto cache_key = std::make_tuple(base_stair_block.id(), facing, shape);
	// Roofs may ask for the same stair on every cell. Keep each worker's hot
	// keys locally so they don't all serialize on the shared cache mutex.
	static thread_local std::unordered_map<std::tuple<content_t, StairFacing, StairShape>,
			BlockWithProperties, StairKeyHash>
			local_cache;
	if (const auto it = local_cache.find(cache_key); it != local_cache.end())
		return it->second;

	// Share the immutable property object between workers on a cold local hit.
	{
		std::lock_guard<std::mutex> lock(STAIR_CACHE_MUTEX);
		auto it = STAIR_CACHE.find(cache_key);
		if (it != STAIR_CACHE.end()) {
			local_cache.emplace(cache_key, it->second);
			return it->second;
		}
	}
	BlockWithProperties::Properties properties;
	properties.emplace("facing", StairFacing_as_str(facing));
	if (shape != StairShape::Straight)
		properties.emplace("shape", StairShape_as_str(shape));
	BlockWithProperties block_with_props(base_stair_block, std::move(properties));

	// Another worker may have filled this key while we built the properties.
	{
		std::lock_guard<std::mutex> lock(STAIR_CACHE_MUTEX);
		auto [it, inserted] = STAIR_CACHE.emplace(cache_key, block_with_props);
		(void)inserted;
		block_with_props = it->second;
	}
	local_cache.emplace(cache_key, block_with_props);

	return block_with_props;
}

BlockWithProperties top_stair(BlockWithProperties stair)
{
	if (!stair.properties)
		return stair;
	const auto &properties = *stair.properties;
	const auto add_top_half = [&]() {
		auto top_properties = *stair.properties;
		top_properties["half"] = "top";
		stair.properties =
				BlockWithProperties::intern_properties(std::move(top_properties));
		return stair;
	};
	const auto facing_it = properties.find("facing");
	if (facing_it == properties.end() || properties.size() > 2)
		return add_top_half();
	std::optional<StairFacing> facing;
	for (const auto candidate : {StairFacing::North, StairFacing::East,
				 StairFacing::South, StairFacing::West})
		if (facing_it->second == StairFacing_as_str(candidate)) {
			facing = candidate;
			break;
		}
	if (!facing)
		return add_top_half();
	StairShape shape = StairShape::Straight;
	if (const auto shape_it = properties.find("shape"); shape_it != properties.end()) {
		if (properties.size() != 2)
			return add_top_half();
		bool found = false;
		for (const auto candidate :
				{StairShape::Straight, StairShape::InnerLeft, StairShape::InnerRight,
						StairShape::OuterLeft, StairShape::OuterRight})
			if (shape_it->second == StairShape_as_str(candidate)) {
				shape = candidate;
				found = true;
				break;
			}
		if (!found)
			return add_top_half();
	} else if (properties.size() != 1) {
		return add_top_half();
	}
	const auto key = std::make_tuple(stair.block.id(), *facing, shape);
	{
		std::lock_guard<std::mutex> lock(TOP_STAIR_CACHE_MUTEX);
		if (const auto it = TOP_STAIR_CACHE.find(key); it != TOP_STAIR_CACHE.end()) {
			stair.properties = it->second;
			return stair;
		}
	}
	auto top_properties = properties;
	top_properties["half"] = "top";
	auto shared = BlockWithProperties::intern_properties(std::move(top_properties));
	{
		std::lock_guard<std::mutex> lock(TOP_STAIR_CACHE_MUTEX);
		auto [it, inserted] = TOP_STAIR_CACHE.emplace(key, std::move(shared));
		(void)inserted;
		stair.properties = it->second;
	}
	return stair;
}

Block get_stair_block_for_material(const Block &material)
{
	if (material == STONE || material == COBBLESTONE || material == GRAVEL)
		return COBBLESTONE_STAIRS;
	if (material == STONE_BRICKS || material == CRACKED_STONE_BRICKS ||
			material == CHISELED_STONE_BRICKS)
		return STONE_BRICK_STAIRS;
	if (material == MOSSY_COBBLESTONE)
		return MOSSY_COBBLESTONE_STAIRS;
	if (material == MOSSY_STONE_BRICKS)
		return MOSSY_STONE_BRICK_STAIRS;
	if (material == TUFF)
		return COBBLESTONE_STAIRS;
	if (material == END_STONE_BRICKS)
		return END_STONE_BRICK_STAIRS;
	if (material == ANDESITE)
		return ANDESITE_STAIRS;
	if (material == DIORITE || material == POLISHED_DIORITE ||
			material == LIGHT_BLUE_TERRACOTTA || material == QUARTZ_BRICKS ||
			material == IRON_BLOCK)
		return POLISHED_DIORITE_STAIRS;
	if (material == GRANITE || material == TERRACOTTA)
		return GRANITE_STAIRS;
	if (material == POLISHED_GRANITE)
		return POLISHED_GRANITE_STAIRS;
	if (material == DEEPSLATE || material == DEEPSLATE_BRICKS)
		return DEEPSLATE_BRICK_STAIRS;
	if (material == POLISHED_DEEPSLATE)
		return POLISHED_DEEPSLATE_STAIRS;
	if (material == COBBLED_DEEPSLATE)
		return COBBLED_DEEPSLATE_STAIRS;
	if (material == DEEPSLATE_TILES)
		return DEEPSLATE_TILE_STAIRS;
	if (material == BLACKSTONE || material == POLISHED_BLACKSTONE ||
			material == POLISHED_BLACKSTONE_BRICKS || material == BLACK_TERRACOTTA ||
			material == NETHERITE_BLOCK || material == BLACK_CONCRETE)
		return POLISHED_BLACKSTONE_BRICK_STAIRS;
	if (material == MUD_BRICKS || material == MUD || material == BROWN_CONCRETE ||
			material == BROWN_CONCRETE_POWDER || material == BROWN_TERRACOTTA ||
			material == GRAY_TERRACOTTA || material == LIGHT_GRAY_TERRACOTTA)
		return MUD_BRICK_STAIRS;
	if (material == BRICK || material == ORANGE_TERRACOTTA || material == RED_TERRACOTTA)
		return BRICK_STAIRS;
	if (material == BLUE_TERRACOTTA || material == CYAN_TERRACOTTA ||
			material == CYAN_CONCRETE)
		return DEEPSLATE_BRICK_STAIRS;
	if (material == OAK_PLANKS || material == OAK_LOG || material == HAY_BALE)
		return OAK_STAIRS;
	if (material == SPRUCE_PLANKS || material == SPRUCE_LOG)
		return SPRUCE_STAIRS;
	if (material == DARK_OAK_PLANKS)
		return DARK_OAK_STAIRS;
	if (material == MANGROVE_PLANKS)
		return MANGROVE_STAIRS;
	if (material == CHERRY_PLANKS)
		return CHERRY_STAIRS;
	if (material == BIRCH_PLANKS || material == YELLOW_TERRACOTTA ||
			material == SANDSTONE || material == SMOOTH_SANDSTONE)
		return SMOOTH_SANDSTONE_STAIRS;
	if (material == POLISHED_ANDESITE || material == SMOOTH_STONE ||
			material == LIGHT_GRAY_CONCRETE_POWDER)
		return POLISHED_ANDESITE_STAIRS;
	if (material == PINK_TERRACOTTA || material == LIME_TERRACOTTA ||
			material == WHITE_TERRACOTTA || material == MUSHROOM_STEM ||
			material == CALCITE || material == QUARTZ_BLOCK ||
			material == WHITE_CONCRETE || material == GLASS)
		return QUARTZ_STAIRS;
	if (material == SMOOTH_QUARTZ)
		return SMOOTH_QUARTZ_STAIRS;
	if (material == LIGHT_BLUE_CONCRETE)
		return POLISHED_DIORITE_STAIRS;
	if (material == GRAY_CONCRETE)
		return POLISHED_BLACKSTONE_BRICK_STAIRS;
	if (material == LIGHT_GRAY_CONCRETE)
		return STONE_BRICK_STAIRS;
	if (material == GREEN_CONCRETE || material == GRASS_BLOCK || material == MOSS_BLOCK)
		return MOSSY_COBBLESTONE_STAIRS;
	if (material == NETHER_BRICK)
		return NETHER_BRICK_STAIRS;
	if (material == RED_NETHER_BRICKS)
		return RED_NETHER_BRICK_STAIRS;
	if (material == DARK_PRISMARINE)
		return DARK_PRISMARINE_STAIRS;
	if (material == WAXED_OXIDIZED_COPPER)
		return WAXED_OXIDIZED_CUT_COPPER_STAIRS;
	if (material == WAXED_COPPER_BLOCK)
		return WAXED_CUT_COPPER_STAIRS;
	if (material == WAXED_EXPOSED_COPPER || material == WAXED_EXPOSED_CHISELED_COPPER ||
			material == WAXED_EXPOSED_CUT_COPPER)
		return WAXED_EXPOSED_CUT_COPPER_STAIRS;
	return STONE_BRICK_STAIRS;
}

Block get_slab_block_for_material(const Block &material)
{
	if (material == STONE_BRICKS || material == CHISELED_STONE_BRICKS ||
			material == CRACKED_STONE_BRICKS)
		return STONE_BRICK_SLAB;
	if (material == BRICK || material == BROWN_TERRACOTTA ||
			material == BROWN_CONCRETE_POWDER || material == RED_TERRACOTTA)
		return BRICK_SLAB;
	if (material == GRANITE || material == POLISHED_GRANITE || material == TERRACOTTA)
		return POLISHED_GRANITE_SLAB;
	if (material == MUD_BRICKS || material == WHITE_TERRACOTTA ||
			material == GRAY_TERRACOTTA || material == LIGHT_BLUE_TERRACOTTA)
		return MUD_BRICK_SLAB;
	if (material == OAK_PLANKS || material == OAK_LOG)
		return OAK_SLAB;
	if (material == SPRUCE_PLANKS)
		return SPRUCE_SLAB;
	if (material == DARK_OAK_PLANKS)
		return DARK_OAK_SLAB;
	if (material == QUARTZ_BLOCK || material == QUARTZ_BRICKS ||
			material == WHITE_CONCRETE || material == MUSHROOM_STEM ||
			material == CALCITE || material == PINK_TERRACOTTA ||
			material == LIME_TERRACOTTA)
		return QUARTZ_SLAB_TOP;
	if (material == POLISHED_ANDESITE)
		return POLISHED_ANDESITE_SLAB;
	if (material == SMOOTH_STONE || material == ANDESITE || material == GRAY_CONCRETE ||
			material == LIGHT_GRAY_CONCRETE || material == LIGHT_GRAY_CONCRETE_POWDER)
		return SMOOTH_STONE_SLAB;
	if (material == DEEPSLATE_BRICKS || material == POLISHED_DEEPSLATE)
		return POLISHED_DEEPSLATE_SLAB;
	if (material == DEEPSLATE_TILES)
		return DEEPSLATE_TILE_SLAB;
	if (material == COBBLED_DEEPSLATE)
		return COBBLED_DEEPSLATE_SLAB;
	if (material == POLISHED_BLACKSTONE_BRICKS || material == POLISHED_BLACKSTONE)
		return POLISHED_BLACKSTONE_SLAB;
	if (material == BLACKSTONE)
		return BLACKSTONE_SLAB;
	if (material == RED_NETHER_BRICKS)
		return RED_NETHER_BRICK_SLAB;
	if (material == NETHER_BRICK)
		return NETHER_BRICK_SLAB;
	if (material == DARK_PRISMARINE)
		return DARK_PRISMARINE_SLAB;
	if (material == WAXED_OXIDIZED_COPPER)
		return WAXED_OXIDIZED_CUT_COPPER_SLAB;
	if (material == MANGROVE_PLANKS)
		return MANGROVE_SLAB;
	if (material == CHERRY_PLANKS)
		return CHERRY_SLAB;
	if (material == BIRCH_PLANKS)
		return CUT_SANDSTONE_SLAB;
	if (material == SANDSTONE || material == SMOOTH_SANDSTONE)
		return STONE_BLOCK_SLAB;
	return STONE_BRICK_SLAB;
}

Block get_wall_piece_for_material(const Block &material)
{
	if (material == BRICK || material == BROWN_TERRACOTTA ||
			material == BROWN_CONCRETE_POWDER || material == MUD_BRICKS ||
			material == WHITE_TERRACOTTA)
		return BRICK_WALL;
	if (material == ANDESITE || material == GRAY_CONCRETE ||
			material == LIGHT_GRAY_CONCRETE || material == LIGHT_GRAY_CONCRETE_POWDER ||
			material == CYAN_TERRACOTTA)
		return ANDESITE_WALL;
	if (material == STONE_BRICKS || material == CHISELED_STONE_BRICKS ||
			material == CRACKED_STONE_BRICKS || material == POLISHED_ANDESITE ||
			material == SMOOTH_STONE || material == POLISHED_DEEPSLATE ||
			material == DEEPSLATE_BRICKS)
		return STONE_BRICK_WALL;
	if (material == SANDSTONE || material == SMOOTH_SANDSTONE)
		return SANDSTONE_WALL;
	if (material == MUSHROOM_STEM || material == CALCITE)
		return DIORITE_WALL;
	return COBBLESTONE_WALL;
}

/*
Block get_stair_block_for_material(const Block &material) {
    switch (material.g) {
        case STONE_BRICKS:
            return STONE_BRICK_STAIRS;
        case MUD_BRICKS:
            return MUD_BRICK_STAIRS;
        case OAK_PLANKS:
            return OAK_STAIRS;
        case POLISHED_ANDESITE:
            return STONE_BRICK_STAIRS;
        case SMOOTH_STONE:
            return POLISHED_ANDESITE_STAIRS;
        case ANDESITE:
            return STONE_BRICK_STAIRS;
        case CHISELED_STONE_BRICKS:
            return STONE_BRICK_STAIRS;
        case BLACK_TERRACOTTA:
            return POLISHED_BLACKSTONE_BRICK_STAIRS;
        case BLACKSTONE:
            return POLISHED_BLACKSTONE_BRICK_STAIRS;
        case BLUE_TERRACOTTA:
            return MUD_BRICK_STAIRS;
        case BRICK:
            return BRICK_STAIRS;
        case BROWN_CONCRETE:
            return MUD_BRICK_STAIRS;
        case BROWN_TERRACOTTA:
            return MUD_BRICK_STAIRS;
        case DEEPSLATE_BRICKS:
            return STONE_BRICK_STAIRS;
        case END_STONE_BRICKS:
            return END_STONE_BRICK_STAIRS;
        case GRAY_CONCRETE:
            return POLISHED_BLACKSTONE_BRICK_STAIRS;
        case GRAY_TERRACOTTA:
            return MUD_BRICK_STAIRS;
        case LIGHT_BLUE_TERRACOTTA:
            return STONE_BRICK_STAIRS;
        case LIGHT_GRAY_CONCRETE:
            return STONE_BRICK_STAIRS;
        case NETHER_BRICK:
            return NETHER_BRICK_STAIRS;
        case POLISHED_BLACKSTONE:
            return POLISHED_BLACKSTONE_BRICK_STAIRS;
        case POLISHED_BLACKSTONE_BRICKS:
            return POLISHED_BLACKSTONE_BRICK_STAIRS;
        case POLISHED_DEEPSLATE:
            return STONE_BRICK_STAIRS;
        case POLISHED_GRANITE:
            return POLISHED_GRANITE_STAIRS;
        case QUARTZ_BLOCK:
            return POLISHED_DIORITE_STAIRS;
        case QUARTZ_BRICKS:
            return POLISHED_DIORITE_STAIRS;
        case SANDSTONE:
            return SMOOTH_SANDSTONE_STAIRS;
        case SMOOTH_SANDSTONE:
            return SMOOTH_SANDSTONE_STAIRS;
        case WHITE_CONCRETE:
            return QUARTZ_STAIRS;
        case WHITE_TERRACOTTA:
            return MUD_BRICK_STAIRS;

            default:
            return STONE;
            //return STONE_BRICK_STAIRS;
    }
}
*/

static const std::array<Block, 5> WINDOW_VARIATIONS = {GRAY_STAINED_GLASS,
		LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS};

namespace
{
const std::vector<Block> &window_options_for_building_type(const std::string &type)
{
	static const std::vector<Block> residential{GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
			GLASS};
	static const std::vector<Block> institutional{GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, BLACK_STAINED_GLASS};
	static const std::vector<Block> hospitality{GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, BLACK_STAINED_GLASS};
	static const std::vector<Block> industrial{
			GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS};
	static const std::vector<Block> tower{GRAY_STAINED_GLASS, GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, BLACK_STAINED_GLASS, CYAN_STAINED_GLASS};
	static const std::vector<Block> religious{BLUE_STAINED_GLASS, CYAN_STAINED_GLASS,
			LIGHT_BLUE_STAINED_GLASS, BROWN_STAINED_GLASS, WHITE_STAINED_GLASS};
	static const std::vector<Block> farm{
			GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS};
	static const std::vector<Block> historic{
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, GLASS};
	static const std::vector<Block> variations(
			WINDOW_VARIATIONS.begin(), WINDOW_VARIATIONS.end());
	if (type == "residential" || type == "house" || type == "apartment")
		return residential;
	if (type == "hospital" || type == "school" || type == "university" ||
			type == "office" || type == "commercial")
		return institutional;
	if (type == "hotel" || type == "restaurant")
		return hospitality;
	if (type == "industrial" || type == "warehouse")
		return industrial;
	if (type == "tower" || type == "skyscraper")
		return tower;
	if (type == "religious" || type == "church" || type == "chapel" || type == "mosque" ||
			type == "temple")
		return religious;
	if (type == "farm")
		return farm;
	if (type == "historic" || type == "heritage")
		return historic;
	return variations;
}
} // namespace

static const std::vector<std::pair<RGB, std::vector<Block *>>> DEFINED_COLORS = {
		{{233, 107, 57}, {&BRICK, &NETHER_BRICK}},
		{{18, 12, 13}, {&POLISHED_BLACKSTONE_BRICKS, &BLACKSTONE, &DEEPSLATE_BRICKS}},
		{{76, 127, 153}, {&LIGHT_BLUE_TERRACOTTA}},
		{{0, 0, 0}, {&DEEPSLATE_BRICKS, &BLACKSTONE, &POLISHED_BLACKSTONE}},
		{{186, 195, 142},
				{&END_STONE_BRICKS, &SANDSTONE, &SMOOTH_SANDSTONE, &LIGHT_GRAY_CONCRETE}},
		{{57, 41, 35}, {&BROWN_TERRACOTTA, &BROWN_CONCRETE, &MUD_BRICKS, &BRICK}},
		{{112, 108, 138}, {&LIGHT_BLUE_TERRACOTTA, &GRAY_TERRACOTTA, &GRAY_CONCRETE}},
		{{122, 92, 66}, {&MUD_BRICKS, &BROWN_TERRACOTTA, &SANDSTONE, &BRICK}},
		{{24, 13, 14}, {&NETHER_BRICK, &BLACKSTONE, &DEEPSLATE_BRICKS}},
		{{159, 82, 36}, {&BROWN_TERRACOTTA, &BRICK, &POLISHED_GRANITE, &BROWN_CONCRETE,
								&NETHERITE_BLOCK, &POLISHED_DEEPSLATE}},
		{{128, 128, 128},
				{&POLISHED_ANDESITE, &LIGHT_GRAY_CONCRETE, &SMOOTH_STONE, &STONE_BRICKS}},
		{{174, 173, 174},
				{&POLISHED_ANDESITE, &LIGHT_GRAY_CONCRETE, &SMOOTH_STONE, &STONE_BRICKS}},
		{{141, 101, 142}, {&STONE_BRICKS, &BRICK, &MUD_BRICKS}},
		{{142, 60, 46},
				{&BLACK_TERRACOTTA, &NETHERITE_BLOCK, &NETHER_BRICK, &POLISHED_GRANITE,
						&POLISHED_DEEPSLATE, &BROWN_TERRACOTTA}},
		{{153, 83, 28}, {&BLACK_TERRACOTTA, &POLISHED_GRANITE, &BROWN_CONCRETE,
								&BROWN_TERRACOTTA, &STONE_BRICKS}},
		{{224, 216, 175}, {&SMOOTH_SANDSTONE, &LIGHT_GRAY_CONCRETE, &POLISHED_ANDESITE,
								  &SMOOTH_STONE}},
		{{188, 182, 179}, {&SMOOTH_SANDSTONE, &LIGHT_GRAY_CONCRETE, &QUARTZ_BRICKS,
								  &POLISHED_ANDESITE, &SMOOTH_STONE}},
		{{35, 86, 85},
				{&POLISHED_BLACKSTONE_BRICKS, &BLUE_TERRACOTTA, &LIGHT_BLUE_TERRACOTTA}},
		{{255, 255, 255}, {&WHITE_CONCRETE, &QUARTZ_BRICKS, &QUARTZ_BLOCK}},
		{{209, 177, 161}, {&WHITE_TERRACOTTA, &SMOOTH_SANDSTONE, &SMOOTH_STONE,
								  &SANDSTONE, &LIGHT_GRAY_CONCRETE}},
		{{191, 147, 42}, {&SMOOTH_SANDSTONE, &SANDSTONE, &SMOOTH_STONE}},
		{{36, 137, 199}, {&LIGHT_BLUE_CONCRETE}},
		{{45, 47, 143}, {&BLUE_CONCRETE, &BLUE_TERRACOTTA}},
		{{21, 119, 136}, {&CYAN_CONCRETE}}, {{73, 91, 36}, {&GREEN_CONCRETE}},
		{{120, 200, 30}, {&LIME_CONCRETE}}, {{241, 175, 21}, {&YELLOW_CONCRETE}},
		{{186, 133, 35}, {&YELLOW_TERRACOTTA}}, {{200, 30, 30}, {&RED_CONCRETE}},
		{{100, 32, 156}, {&PURPLE_CONCRETE}}, {{169, 48, 159}, {&MAGENTA_CONCRETE}},
		{{224, 97, 1}, {&ORANGE_CONCRETE}},
		{{90, 90, 90}, {&GRAY_CONCRETE, &POLISHED_DEEPSLATE}}};

/*
inline Block get_fallback_building_block();

inline Block random_choice_from_vector(const std::vector<Block>& v) {
    if (v.empty()) return get_fallback_building_block();
    std::mt19937& rng = global_rng();
    std::uniform_int_distribution<std::size_t> dist(0, v.size() - 1);
    return v[dist(rng)];
}

inline Block random_choice_from_array(const std::vector<Block>::const_iterator begin_it, std::size_t len) {
    if (len == 0) return get_fallback_building_block();
    std::mt19937& rng = global_rng();
    std::uniform_int_distribution<std::size_t> dist(0, len - 1);
    return *(begin_it + dist(rng));
}
*/

Block get_window_block_for_building_type(const std::string &building_type)
{
	std::mt19937 &rng = global_rng();
	const auto &options = window_options_for_building_type(building_type);
	std::uniform_int_distribution<std::size_t> dist(0, options.size() - 1);
	return options[dist(rng)];
}

Block get_window_block_for_building_type(
		const std::string &building_type, ChaCha8Rng &rng)
{
	const auto &options = window_options_for_building_type(building_type);
	return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
}

Block get_random_floor_block()
{
	const std::vector<Block> floor_options = {WHITE_CONCRETE, GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE, STONE_BRICKS,
			MUD_BRICKS, OAK_PLANKS};
	std::mt19937 &rng = global_rng();
	std::uniform_int_distribution<std::size_t> dist(0, floor_options.size() - 1);
	return floor_options[dist(rng)];
}

Block get_random_floor_block(ChaCha8Rng &rng)
{
	//const std::array<Block, 8> floor_options = {
	const std::vector<Block> floor_options = {WHITE_CONCRETE, GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE, STONE_BRICKS,
			MUD_BRICKS, OAK_PLANKS};
	return floor_options[rng.uniform(static_cast<std::uint32_t>(floor_options.size()))];
}

Block get_building_wall_block_for_color(const RGB &color)
{
	if (DEFINED_COLORS.empty()) {
		return get_fallback_building_block();
	}
	auto best_it = std::min_element(DEFINED_COLORS.begin(), DEFINED_COLORS.end(),
			[&color](const std::pair<RGB, std::vector<Block *>> &a,
					const std::pair<RGB, std::vector<Block *>> &b) {
				return colors::rgb_distance(color, a.first) <
					   colors::rgb_distance(color, b.first);
			});
	if (best_it != DEFINED_COLORS.end() && !best_it->second.empty()) {
		std::mt19937 &rng = global_rng();
		std::uniform_int_distribution<std::size_t> dist(0, best_it->second.size() - 1);
		return *best_it->second[dist(rng)];
	}
	return get_fallback_building_block();
}

Block get_building_wall_block_for_color(const RGB &color, ChaCha8Rng &rng)
{
	return block_palette::wall_block_for_color(color, rng);
}

Block get_fallback_building_block()
{
	// Keep this order identical to Rust's get_fallback_building_block palette.
	const std::vector<Block> fallback_options = {BRICK, BROWN_CONCRETE, BROWN_TERRACOTTA,
			CYAN_TERRACOTTA, GRAY_CONCRETE, GRAY_TERRACOTTA, LIGHT_GRAY_CONCRETE,
			MUD_BRICKS, MUSHROOM_STEM, POLISHED_ANDESITE, POLISHED_GRANITE, QUARTZ_BLOCK,
			QUARTZ_BRICKS, SANDSTONE, SMOOTH_SANDSTONE, SMOOTH_STONE, STONE_BRICKS,
			WHITE_CONCRETE, WHITE_TERRACOTTA};
	std::mt19937 &rng = global_rng();
	std::uniform_int_distribution<std::size_t> dist(0, fallback_options.size() - 1);
	return fallback_options[dist(rng)];
}

Block get_fallback_building_block(ChaCha8Rng &rng)
{
	// Keep this order identical to Rust's get_fallback_building_block palette.
	const std::vector<Block> options = {BRICK, BROWN_CONCRETE, BROWN_TERRACOTTA,
			CYAN_TERRACOTTA, GRAY_CONCRETE, GRAY_TERRACOTTA, LIGHT_GRAY_CONCRETE,
			MUD_BRICKS, MUSHROOM_STEM, POLISHED_ANDESITE, POLISHED_GRANITE, QUARTZ_BLOCK,
			QUARTZ_BRICKS, SANDSTONE, SMOOTH_SANDSTONE, SMOOTH_STONE, STONE_BRICKS,
			WHITE_CONCRETE, WHITE_TERRACOTTA};
	return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
}

Block get_castle_wall_block()
{
	//const std::array<Block, 10> castle_wall_options = {
	const std::vector<Block> castle_wall_options = {STONE_BRICKS, CHISELED_STONE_BRICKS,
			CRACKED_STONE_BRICKS,

			COBBLESTONE, MOSSY_COBBLESTONE, DEEPSLATE_BRICKS, POLISHED_ANDESITE, ANDESITE,

			SMOOTH_STONE, BRICK};
	std::mt19937 &rng = global_rng();
	std::uniform_int_distribution<std::size_t> dist(0, castle_wall_options.size() - 1);
	return castle_wall_options[dist(rng)];
}

Block get_castle_wall_block(ChaCha8Rng &rng)
{
	const std::vector<Block> options = {STONE_BRICKS, CHISELED_STONE_BRICKS,
			CRACKED_STONE_BRICKS, COBBLESTONE, MOSSY_COBBLESTONE, DEEPSLATE_BRICKS,
			POLISHED_ANDESITE, ANDESITE, SMOOTH_STONE, BRICK};
	return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
}

BlockWithProperties snow_layer_with_depth(unsigned layers)
{
	return BlockWithProperties{
			SNOW_LAYER, {{"layers", std::to_string(std::clamp(layers, 1u, 7u))}}};
}

}
