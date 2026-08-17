#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"
#include "../floodfill.h"
#include "../floodfill_cache.h"
#include "buildings.h"
#include "historic.h"
#include "subprocessor/buildings_interior.h"
#include "../deterministic_rng.h"
namespace arnis
{

namespace man_made
{
bool is_tank_structure(const ProcessedWay &way);
void generate_tank_structure(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
}

namespace buildings
{

/*

// Basic placeholder types and constants
using Block = int;

struct XZPoint {
    int x;
    int z;
    XZPoint(int x_, int z_) : x(x_), z(z_) {}
    static XZPoint new_point(int x_, int z_) { return XZPoint(x_, z_); }
};

struct Ground {
    int level(const XZPoint& p) const;
};

struct WorldEditor {
    std::pair<int,int> get_min_coords() const;
    Ground* get_ground() const; // may return nullptr
    void set_block(Block b, int x, int y, int z, const Block* alternatives = nullptr, std::size_t alt_count = 0) const;
    void set_block_absolute(Block b, int x, int y, int z, const Block* alternatives = nullptr, std::size_t alt_count = 0) const;
};

struct Node { int x; int z; };

struct ProcessedWay {
    std::vector<Node> nodes;
    std::unordered_map<std::string, std::string> tags;
};

struct Args {
    bool terrain;
    int ground_level;
    double scale;
    bool roof;
    bool interior;
    std::optional<int> timeout;
    std::optional<int> timeout_ref() const { return timeout; }
};

// Example block constants (placeholders)
constexpr Block POLISHED_ANDESITE = 1;
constexpr Block SMOOTH_STONE = 2;
constexpr Block STONE_BRICKS = 3;
constexpr Block MUD_BRICKS = 4;
constexpr Block ANDESITE = 5;
constexpr Block CHISELED_STONE_BRICKS = 6;
constexpr Block STONE_BRICK_SLAB = 7;
constexpr Block OAK_FENCE = 8;
constexpr Block OAK_PLANKS = 9;
constexpr Block STONE_BLOCK_SLAB = 10;
constexpr Block SMOOTH_STONE_BLOCK = 11;
constexpr Block COBBLESTONE = 12;
constexpr Block COBBLESTONE_WALL = 13;
constexpr Block STONE_BRICKS_BLOCK = 14;
constexpr Block GLOWSTONE = 15;
constexpr Block COBBLESTONE_BLOCK = 16;
// Helper function declarations (assumed implemented elsewhere)
int multiply_scale(int value, double scale);
std::vector<std::pair<int,int>> flood_fill_area(const std::vector<std::pair<int,int>>& polygon_coords, const std::optional<int>& timeout);
Block get_castle_wall_block();
std::optional<std::tuple<uint8_t,uint8_t,uint8_t>> color_text_to_rgb_tuple(const std::string& text);
Block get_building_wall_block_for_color(const std::tuple<uint8_t,uint8_t,uint8_t>& rgb);
Block get_fallback_building_block();
Block get_random_floor_block();
Block get_window_block_for_building_type(const std::string& building_type);
void generate_bridge(WorldEditor* editor, const ProcessedWay& element, const std::optional<int>& timeout);
std::vector<std::tuple<int,int,int>> bresenham_line(int x1, int y1, int z1, int x2, int y2, int z2);
void generate_building_interior(WorldEditor* editor,
                                const std::vector<std::pair<int,int>>& floor_area,
                                int min_x, int min_z, int max_x, int max_z,
                                int start_y_offset, int building_height,
                                Block wall_block,
                                const std::vector<int>& floor_levels,
                                const Args& args,
                                const ProcessedWay& element,
                                int abs_terrain_offset,
                                bool is_abandoned_building,
                                const CoordinateBitmap& building_passages,
                                bool has_sloped_roof);
void generate_roof(WorldEditor* editor,
                   const ProcessedWay& element,
                   int start_y_offset,
                   int building_height,
                   Block floor_block,
                   Block wall_block,
                   Block accent_block,
                   RoofType roof_type,
                   const std::vector<std::pair<int,int>>& cached_floor_area,
                   int abs_terrain_offset);

*/

// RoofType enum
enum class RoofType
{
	Gabled,
	Hipped,
	Skillion,
	Pyramidal,
	Dome,
	Cone,
	Onion,
	Flat
};

constexpr int BUILDING_PASSAGE_HEIGHT = 4;

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset);

// Hash for pair<int,int>
struct PairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		std::size_t h1 = std::hash<int>()(p.first);
		std::size_t h2 = std::hash<int>()(p.second);
		return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
	}
};
using pair_hash = PairHash;

namespace
{

std::optional<int> parse_i32_tag(const tags_t &tags, const std::string &key)
{
	auto it = tags.find(key);
	if (it == tags.end())
		return std::nullopt;
	try {
		return std::stoi(it->second);
	} catch (...) {
		return std::nullopt;
	}
}

bool has_tag_value(const tags_t &tags, const std::string &key, const std::string &value)
{
	auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

bool should_skip_underground_tags(const tags_t &tags)
{
	if (auto layer = parse_i32_tag(tags, "layer"); layer.has_value() && *layer < 0)
		return true;
	if (auto level = parse_i32_tag(tags, "level"); level.has_value() && *level < 0)
		return true;
	if (has_tag_value(tags, "location", "underground") ||
			has_tag_value(tags, "location", "subway"))
		return true;
	return tags.contains("building:levels:underground") &&
		   !tags.contains("building:levels");
}

std::vector<std::pair<int, int>> way_polygon_coords(const ProcessedWay &way)
{
	std::vector<std::pair<int, int>> polygon_coords;
	polygon_coords.reserve(way.nodes.size());
	for (const auto &n : way.nodes)
		polygon_coords.emplace_back(n.x, n.z);
	return polygon_coords;
}

std::vector<std::pair<int, int>> compute_floor_area(
		const FloodFillCache *flood_fill_cache, const ProcessedWay &way, const Args &args)
{
	if (flood_fill_cache)
		return flood_fill_cache->get_or_compute(way, args.timeout);
	return flood_fill_area(way_polygon_coords(way), args.timeout_ref());
}

void merge_way_segments(std::vector<std::vector<ProcessedNode>> &rings)
{
	bool changed = true;
	while (changed) {
		changed = false;
		for (std::size_t i = 0; i < rings.size() && !changed; ++i) {
			if (rings[i].empty())
				continue;
			for (std::size_t j = i + 1; j < rings.size(); ++j) {
				if (rings[j].empty())
					continue;

				auto &a = rings[i];
				auto &b = rings[j];
				auto same_point = [](const ProcessedNode &lhs, const ProcessedNode &rhs) {
					return lhs.id == rhs.id || (lhs.x == rhs.x && lhs.z == rhs.z);
				};

				if (same_point(a.back(), b.front())) {
					a.insert(a.end(), std::next(b.begin()), b.end());
				} else if (same_point(a.back(), b.back())) {
					a.insert(a.end(), std::next(b.rbegin()), b.rend());
				} else if (same_point(a.front(), b.back())) {
					a.insert(a.begin(), b.begin(), std::prev(b.end()));
				} else if (same_point(a.front(), b.front())) {
					a.insert(a.begin(), std::next(b.rbegin()), b.rend());
				} else {
					continue;
				}

				rings.erase(rings.begin() + static_cast<std::ptrdiff_t>(j));
				changed = true;
				break;
			}
		}
	}
}

bool close_ring_if_near(std::vector<ProcessedNode> &ring)
{
	if (ring.size() < 3)
		return false;
	const auto &first = ring.front();
	const auto &last = ring.back();
	if (first.id != last.id && std::abs(first.x - last.x) <= 1 &&
			std::abs(first.z - last.z) <= 1)
		ring.push_back(first);
	const auto &closed_last = ring.back();
	return first.id == closed_last.id || (std::abs(first.x - closed_last.x) <= 1 &&
												 std::abs(first.z - closed_last.z) <= 1);
}

std::vector<std::vector<ProcessedNode>> collect_merged_rings(
		const ProcessedRelation &relation, ProcessedMemberRole role)
{
	std::vector<std::vector<ProcessedNode>> rings;
	for (const auto &member : relation.members) {
		if (member.role == role)
			rings.push_back(member.way.nodes);
	}
	merge_way_segments(rings);
	std::vector<std::vector<ProcessedNode>> out;
	for (auto &ring : rings) {
		if (close_ring_if_near(ring) && ring.size() >= 4)
			out.push_back(std::move(ring));
	}
	return out;
}

bool passage_at(const CoordinateBitmap *building_passages, int x, int z)
{
	return building_passages && building_passages->contains(x, z);
}

double parse_tag_meters(const std::string &value, double fallback = 0.0)
{
	std::string s = value;
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
		s.pop_back();
	if (!s.empty() && s.back() == 'm')
		s.pop_back();
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
		s.pop_back();
	try {
		return std::stod(s);
	} catch (...) {
		return fallback;
	}
}

std::string normalized_material(std::string_view material)
{
	std::string out;
	out.reserve(material.size());
	for (unsigned char c : material) {
		if (std::isspace(c) || c == '_' || c == '-')
			continue;
		out.push_back(static_cast<char>(std::tolower(c)));
	}
	return out;
}

Block choose_block(const std::vector<Block> &options, ChaCha8Rng &rng)
{
	return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
}

std::optional<Block> get_wall_block_for_material_cpp(
		const std::string &material, ChaCha8Rng &rng)
{
	const std::string m = normalized_material(material);
	if (m == "brick" || m == "bricks" || m == "redbrick")
		return choose_block({BRICK, NETHER_BRICK}, rng);
	if (m == "stone" || m == "naturalstone" || m == "hard")
		return choose_block({STONE_BRICKS, COBBLESTONE, SMOOTH_STONE, ANDESITE}, rng);
	if (m == "limestone")
		return choose_block({SMOOTH_STONE, POLISHED_ANDESITE, WHITE_TERRACOTTA}, rng);
	if (m == "sandstone")
		return choose_block({SANDSTONE, SMOOTH_SANDSTONE}, rng);
	if (m == "marble")
		return choose_block({QUARTZ_BLOCK, POLISHED_DIORITE, WHITE_CONCRETE}, rng);
	if (m == "granite")
		return choose_block({POLISHED_GRANITE, POLISHED_DIORITE, QUARTZ_BLOCK}, rng);
	if (m == "slate")
		return choose_block({POLISHED_BLACKSTONE, DEEPSLATE_BRICKS, BLACKSTONE}, rng);
	if (m == "concrete" || m == "reinforcedconcrete" || m == "cementblock" ||
			m == "cement" || m == "breezeblock" || m == "concreteblock" ||
			m == "concreteblocks" || m == "block" || m == "concretemasonryunit")
		return choose_block(
				{GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, SMOOTH_STONE}, rng);
	if (m == "plaster" || m == "stucco" || m == "render" || m == "rendering" ||
			m == "limerender" || m == "plastered")
		return choose_block(
				{WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, QUARTZ_BLOCK, SMOOTH_SANDSTONE},
				rng);
	if (m == "wood" || m == "timber" || m == "timberframe" || m == "halftimber" ||
			m == "halftimbered" || m == "loghouse" || m == "logs" || m == "bamboo")
		return choose_block({OAK_PLANKS, SPRUCE_PLANKS, DARK_OAK_PLANKS, OAK_LOG}, rng);
	if (m == "reed" || m == "thatch" || m == "straw")
		return HAY_BALE;
	if (m == "metal" || m == "steel" || m == "iron" || m == "aluminium" ||
			m == "aluminum" || m == "corrugatedsteel" || m == "corrugatediron" ||
			m == "corrugatedmetal" || m == "tin" || m == "sheetmetal" ||
			m == "metalsheet" || m == "metalplates")
		return choose_block({IRON_BLOCK, LIGHT_GRAY_CONCRETE, GRAY_CONCRETE}, rng);
	if (m == "copper" || m == "oxidisedcopper" || m == "oxidizedcopper" ||
			m == "patina" || m == "verdigris")
		return choose_block(
				{WAXED_OXIDIZED_COPPER, WAXED_EXPOSED_COPPER, WAXED_COPPER_BLOCK}, rng);
	if (m == "glass")
		return choose_block(
				{GLASS, LIGHT_GRAY_STAINED_GLASS, WHITE_STAINED_GLASS, TINTED_GLASS},
				rng);
	if (m == "mirror" || m == "solarpanels")
		return choose_block({GLASS, BLUE_STAINED_GLASS, LIGHT_BLUE_STAINED_GLASS}, rng);
	if (m == "tiles" || m == "tile" || m == "rooftiles" || m == "ceramictiles" ||
			m == "ceramic" || m == "terracotta")
		return choose_block(
				{WHITE_TERRACOTTA, BROWN_TERRACOTTA, RED_TERRACOTTA, ORANGE_TERRACOTTA},
				rng);
	if (m == "mud" || m == "adobe" || m == "earth" || m == "clay" || m == "rammedearth" ||
			m == "cob" || m == "loam")
		return choose_block({MUD_BRICKS, BROWN_TERRACOTTA, BROWN_CONCRETE}, rng);
	if (m == "asbestos" || m == "asbestoscement" || m == "fibrecement" ||
			m == "fibercement")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE}, rng);
	if (m == "vinyl" || m == "siding" || m == "vinylsiding" || m == "weatherboard" ||
			m == "weatherboarding" || m == "clapboard")
		return choose_block({OAK_PLANKS, SPRUCE_PLANKS, WHITE_CONCRETE}, rng);
	if (m == "panel" || m == "panels" || m == "panelling" || m == "paneling" ||
			m == "panelhouse" || m == "prefab" || m == "prefabricated")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, WHITE_CONCRETE}, rng);
	if (m == "plastic" || m == "light")
		return choose_block(
				{WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, QUARTZ_BLOCK, GLASS}, rng);
	if (m == "mixed" || m == "masonry")
		return choose_block({STONE_BRICKS, BRICK, SMOOTH_STONE, COBBLESTONE}, rng);
	if (m == "pebbledash")
		return choose_block({ANDESITE, COBBLESTONE, STONE_BRICKS, GRAVEL}, rng);
	return std::nullopt;
}

std::optional<Block> get_roof_block_for_material_cpp(
		const std::string &material, ChaCha8Rng &rng)
{
	const std::string normalized = normalized_material(material);
	if (normalized == "glass" || normalized == "glazing" || normalized == "acrylicglass")
		return choose_block({GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS}, rng);
	if (normalized == "tile" || normalized == "tiles" || normalized == "rooftiles" ||
			normalized == "ceramic" || normalized == "ceramictiles" ||
			normalized == "claytile" || normalized == "claytiles" ||
			normalized == "terracotta")
		return choose_block({BRICK, NETHER_BRICK, RED_NETHER_BRICKS, MUD_BRICKS}, rng);
	if (normalized == "slate" || normalized == "slates")
		return choose_block({POLISHED_BLACKSTONE, DEEPSLATE_BRICKS, BLACKSTONE}, rng);
	if (normalized == "metal" || normalized == "steel" || normalized == "aluminium" ||
			normalized == "aluminum" || normalized == "corrugatedsteel" ||
			normalized == "corrugatediron" || normalized == "corrugatedmetal" ||
			normalized == "tin" || normalized == "zinc" || normalized == "lead" ||
			normalized == "sheetmetal" || normalized == "metalsheet")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, IRON_BLOCK}, rng);
	if (normalized == "copper")
		return choose_block(
				{WAXED_OXIDIZED_COPPER, WAXED_EXPOSED_COPPER, WAXED_COPPER_BLOCK}, rng);
	if (normalized == "concrete" || normalized == "reinforcedconcrete")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, SMOOTH_STONE}, rng);
	if (normalized == "wood" || normalized == "timber" || normalized == "shingle" ||
			normalized == "shingles" || normalized == "woodshingle" ||
			normalized == "woodshingles")
		return choose_block({OAK_PLANKS, SPRUCE_PLANKS, DARK_OAK_PLANKS}, rng);
	if (normalized == "thatch" || normalized == "straw" || normalized == "reed" ||
			normalized == "reeds" || normalized == "palmleaves")
		return HAY_BALE;
	if (normalized == "asphalt" || normalized == "bitumen" || normalized == "tar" ||
			normalized == "tarpaper" || normalized == "rolledasphalt" ||
			normalized == "rolledroofing" || normalized == "asphaltshingle")
		return choose_block(
				{BLACKSTONE, POLISHED_BLACKSTONE, POLISHED_BLACKSTONE_BRICKS}, rng);
	if (normalized == "stone")
		return choose_block({STONE_BRICKS, SMOOTH_STONE, ANDESITE}, rng);
	if (normalized == "gravel")
		return GRAVEL;
	if (normalized == "grass" || normalized == "green" || normalized == "vegetation" ||
			normalized == "greenroof" || normalized == "sod")
		return choose_block({GRASS_BLOCK, MOSS_BLOCK}, rng);
	if (normalized == "eternit" || normalized == "asbestos" ||
			normalized == "fibrecement" || normalized == "fibercement")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE}, rng);
	if (normalized == "plastic")
		return choose_block(
				{LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, WHITE_CONCRETE, GLASS}, rng);
	return std::nullopt;
}

Block roof_friendly_block(Block block)
{
	if (block == OAK_LOG)
		return OAK_PLANKS;
	if (block == SPRUCE_LOG)
		return SPRUCE_PLANKS;
	return block;
}

std::optional<Block> roof_block_from_tags(const ProcessedWay &element, ChaCha8Rng &rng)
{
	for (const auto *key : {"roof:material", "material"}) {
		if (auto it = element.tags.find(key); it != element.tags.end()) {
			if (auto block = get_roof_block_for_material_cpp(it->second, rng))
				return block;
		}
	}
	for (const auto *key : {"roof:colour", "building:colour", "colour"}) {
		if (auto it = element.tags.find(key); it != element.tags.end()) {
			if (auto rgb = color_text_to_rgb_tuple(it->second))
				return roof_friendly_block(get_building_wall_block_for_color(*rgb));
		}
	}
	return std::nullopt;
}

RoofType parse_roof_type(const std::string &roof_shape)
{
	if (roof_shape == "gabled" || roof_shape == "gable" || roof_shape == "pitched" ||
			roof_shape == "saltbox" || roof_shape == "double_saltbox" ||
			roof_shape == "quadruple_saltbox" || roof_shape == "gabled_row")
		return RoofType::Gabled;
	if (roof_shape == "hipped" || roof_shape == "hip" || roof_shape == "half-hipped" ||
			roof_shape == "gambrel" || roof_shape == "mansard" || roof_shape == "round" ||
			roof_shape == "side_hipped" || roof_shape == "side_half-hipped")
		return RoofType::Hipped;
	if (roof_shape == "skillion" || roof_shape == "shed" || roof_shape == "lean_to" ||
			roof_shape == "monopitch")
		return RoofType::Skillion;
	if (roof_shape == "pyramidal" || roof_shape == "pyramid")
		return RoofType::Pyramidal;
	if (roof_shape == "dome" || roof_shape == "spherical")
		return RoofType::Dome;
	if (roof_shape == "cone" || roof_shape == "conical" || roof_shape == "circular" ||
			roof_shape == "spire")
		return RoofType::Cone;
	if (roof_shape == "onion")
		return RoofType::Onion;
	return RoofType::Flat;
}

int scaled_blocks(int value, double scale_factor)
{
	if (scale_factor == 1.0)
		return value;
	if (scale_factor == 2.0)
		return value << 1;
	if (scale_factor == 4.0)
		return value << 2;
	return static_cast<int>(std::floor(static_cast<double>(value) * scale_factor));
}

void generate_roof_only_structure(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &cached_floor_area, const Args &args)
{
	const double scale_factor = args.scale;
	const int abs_terrain_offset = !args.terrain ? args.ground_level : 0;

	int min_level_offset = 0;
	if (auto it = element.tags.find("min_height"); it != element.tags.end()) {
		min_level_offset = static_cast<int>(parse_tag_meters(it->second) * scale_factor);
	} else if (auto it = element.tags.find("building:min_level");
			   it != element.tags.end()) {
		if (auto level = parse_i32_tag(element.tags, "building:min_level"))
			min_level_offset = scaled_blocks(*level * 4, scale_factor);
	} else if (auto it = element.tags.find("layer"); it != element.tags.end()) {
		if (auto layer = parse_i32_tag(element.tags, "layer"); layer && *layer > 0)
			min_level_offset = scaled_blocks(*layer * 4, scale_factor);
	}

	int start_y_offset = min_level_offset;
	if (args.terrain) {
		int max_ground_level = args.ground_level;
		for (const auto &node : element.nodes)
			max_ground_level =
					std::max(max_ground_level, editor.get_ground_level(node.x, node.z));
		start_y_offset = max_ground_level + min_level_offset;
	}

	int roof_thickness = 5;
	if (auto it = element.tags.find("height"); it != element.tags.end()) {
		const int total =
				static_cast<int>(parse_tag_meters(it->second, 5.0) * scale_factor);
		roof_thickness = element.tags.contains("min_height")
								 ? std::max(3, total - min_level_offset)
								 : std::max(3, total);
	} else if (auto levels = parse_i32_tag(element.tags, "building:levels")) {
		roof_thickness = std::max(3, scaled_blocks(*levels * 4 + 2, scale_factor));
	}

	auto rng = element_rng_salted(static_cast<std::uint64_t>(element.id), 0x726f6f66);
	const Block roof_block =
			roof_block_from_tags(element, rng).value_or(STONE_BRICK_SLAB);

	const RoofType roof_type = [&]() {
		if (auto it = element.tags.find("roof:shape"); it != element.tags.end())
			return parse_roof_type(it->second);
		return RoofType::Flat;
	}();

	if ((roof_type == RoofType::Dome || roof_type == RoofType::Hipped ||
				roof_type == RoofType::Pyramidal || roof_type == RoofType::Cone ||
				roof_type == RoofType::Onion) &&
			!cached_floor_area.empty()) {
		generate_roof(editor, element, start_y_offset - 4, 3, roof_block, roof_block,
				roof_block,
				roof_type == RoofType::Cone || roof_type == RoofType::Onion
						? RoofType::Dome
						: roof_type,
				cached_floor_area, abs_terrain_offset);
		return;
	}

	const int slab_y = start_y_offset + roof_thickness;
	std::optional<std::pair<int, int>> previous_node;
	for (const auto &node : element.nodes) {
		if (previous_node) {
			auto points = bresenham_line(previous_node->first, slab_y,
					previous_node->second, node.x, slab_y, node.z);
			for (const auto &point : points)
				editor.set_block_absolute(roof_block, std::get<0>(point),
						slab_y + abs_terrain_offset, std::get<2>(point));
		}

		const int pillar_base =
				args.terrain ? editor.get_ground_level(node.x, node.z) : 0;
		for (int y = pillar_base + 1; y < slab_y; ++y)
			editor.set_block_absolute(
					COBBLESTONE_WALL, node.x, y + abs_terrain_offset, node.z);

		previous_node = {node.x, node.z};
	}

	for (const auto &point : cached_floor_area)
		editor.set_block_absolute(
				roof_block, point.first, slab_y + abs_terrain_offset, point.second);
}

}

inline int32_t multiply_scale(int32_t value, double scale_factor);
void generate_bridge(WorldEditor &editor, const ProcessedWay &element,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout);

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset);

void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels)
{
	FloodFillCache empty_cache;
	CoordinateBitmap empty_passages = CoordinateBitmap::new_empty();
	generate_buildings(
			editor, element, args, relation_levels, empty_cache, empty_passages, nullptr);
}

void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels,
		const FloodFillCache &flood_fill_cache, const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons,
		std::optional<std::uint64_t> style_seed)
{
	const auto visual_seed = style_seed.value_or(element.id);
	if (should_skip_underground_tags(element.tags)) {
		return;
	}

	if (element.tags.get("tomb") == "pyramid") {
		historic::generate_pyramid(*editor, element, args);
		return;
	}

	// min_level
	int min_level = 0;
	{
		auto it = element.tags.find("building:min_level");
		if (it != element.tags.end()) {
			try {
				min_level = std::stoi(it->second);
			} catch (...) {
				min_level = 0;
			}
		}
	}

	static const std::unordered_set<uint64_t> skip_way_ids = {
			5013364, 204068874, 32920861};
	if (skip_way_ids.find(element.id) != skip_way_ids.end())
		return;

	if (arnis::man_made::is_tank_structure(element)) {
		arnis::man_made::generate_tank_structure(
				*editor, ProcessedElement(element), args);
		return;
	}

	int abs_terrain_offset = (!args.terrain) ? args.ground_level : 0;
	double scale_factor = args.scale;
	int min_level_offset = multiply_scale(min_level * 4, scale_factor);
	if (auto it = element.tags.find("min_height"); it != element.tags.end()) {
		std::string s = it->second;
		while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
			s.pop_back();
		if (!s.empty() && s.back() == 'm')
			s.pop_back();
		while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
			s.pop_back();
		try {
			min_level_offset = static_cast<int>(std::stod(s) * scale_factor);
		} catch (...) {
			min_level_offset = 0;
		}
	}
	const CoordinateBitmap *effective_passages =
			(min_level_offset == 0) ? &building_passages : nullptr;

	std::vector<std::pair<int, int>> cached_floor_area =
			compute_floor_area(&flood_fill_cache, element, args);
	if (hole_polygons && !hole_polygons->empty() && !cached_floor_area.empty()) {
		std::unordered_set<std::pair<int, int>, PairHash> outer_area(
				cached_floor_area.begin(), cached_floor_area.end());
		std::unordered_set<std::pair<int, int>, PairHash> hole_points;
		for (const auto &hole : *hole_polygons) {
			if (hole.way.nodes.size() < 3)
				continue;
			auto hole_area = compute_floor_area(&flood_fill_cache, hole.way, args);
			bool overlaps_outer = false;
			for (const auto &point : hole_area) {
				if (outer_area.contains(point)) {
					overlaps_outer = true;
					break;
				}
			}
			if (!overlaps_outer)
				continue;
			for (const auto &point : hole_area)
				hole_points.insert(point);
		}
		if (!hole_points.empty()) {
			cached_floor_area.erase(
					std::remove_if(cached_floor_area.begin(), cached_floor_area.end(),
							[&](const auto &point) {
								return hole_points.contains(point);
							}),
					cached_floor_area.end());
		}
	}
	std::size_t cached_footprint_size = cached_floor_area.size();
	if (cached_footprint_size == 0)
		return;

	int start_y_offset = 0;
	if (args.terrain) {
		std::vector<XZPoint> building_points;
		building_points.reserve(element.nodes.size());
		auto min_coords = editor->get_min_coords();
		for (const auto &n : element.nodes) {
			building_points.emplace_back(
					XZPoint::new_point(n.x - min_coords.first, n.z - min_coords.second));
		}

		int max_ground_level = args.ground_level;
		Ground *grd = editor->get_ground();
		for (const auto &point : building_points) {
			if (grd) {
				int lvl = grd->level(point);
				if (lvl > max_ground_level)
					max_ground_level = lvl;
			}
		}

		start_y_offset = max_ground_level + min_level_offset;
	} else {
		start_y_offset = min_level_offset;
	}

	int min_x = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &n : element.nodes) {
		if (n.x < min_x)
			min_x = n.x;
		if (n.x > max_x)
			max_x = n.x;
		if (n.z < min_z)
			min_z = n.z;
		if (n.z > max_z)
			max_z = n.z;
	}
	if (min_x == std::numeric_limits<int>::max()) {
		min_x = 0;
	}
	if (min_z == std::numeric_limits<int>::max()) {
		min_z = 0;
	}
	if (max_x == std::numeric_limits<int>::min()) {
		max_x = 0;
	}
	if (max_z == std::numeric_limits<int>::min()) {
		max_z = 0;
	}

	std::optional<std::pair<int, int>> previous_node;
	std::tuple<int, int, int> corner_addup = std::make_tuple(0, 0, 0);
	std::vector<std::pair<int, int>> current_building;

	// building type
	std::string building_type = "yes";
	{
		auto it = element.tags.find("building");
		if (it != element.tags.end()) {
			building_type = it->second;
		} else {
			auto it2 = element.tags.find("building:part");
			if (it2 != element.tags.end())
				building_type = it2->second;
		}
	}
	// Consume all unconstrained façade choices from the element-local stream.
	auto rng = element_rng_salted(visual_seed, 0x66616361);

	Block wall_block;
	{
		auto it_hist = element.tags.find("historic");
		auto style_value = [&](const char *key) {
			auto it = element.tags.find(key);
			return it == element.tags.end() ? std::string{}
											: normalized_material(it->second);
		};
		const std::string material = style_value("building:material");
		const std::string facade = style_value("building:facade:material");
		const std::string architecture = style_value("building:architecture");
		const bool historic_style = it_hist != element.tags.end() &&
									it_hist->second != "" && it_hist->second != "no";
		const bool glass_style = material == "glass" || material == "mirror" ||
								 facade == "glass" ||
								 style_value("roof:material") == "glass";
		const bool modern_style =
				architecture == "modern" || architecture == "contemporary" ||
				architecture == "modernism" || architecture == "functionalism" ||
				material == "concrete" || material == "reinforcedconcrete";
		if (it_hist != element.tags.end() && it_hist->second == "castle") {
			wall_block = get_castle_wall_block(rng);
		} else {
			auto it_col = element.tags.find("building:colour");
			if (it_col != element.tags.end()) {
				auto rgb = color_text_to_rgb_tuple(it_col->second);
				if (rgb.has_value()) {
					wall_block = get_building_wall_block_for_color(rgb.value(), rng);
				} else {
					wall_block = get_fallback_building_block(rng);
				}
			} else if (auto it_material = element.tags.find("building:material");
					   it_material != element.tags.end()) {
				auto material_rng = element_rng_salted(visual_seed, 0x6d617465);
				wall_block =
						get_wall_block_for_material_cpp(it_material->second, material_rng)
								.value_or(
										glass_style
												? GLASS
												: (modern_style ? get_building_wall_block_for_color(
																		  RGB{180, 180,
																				  180},
																		  rng)
																: (historic_style ? get_castle_wall_block(
																							rng)
																				  : get_fallback_building_block(
																							rng))));
			} else if (glass_style) {
				wall_block = GLASS;
			} else if (historic_style) {
				wall_block = get_castle_wall_block(rng);
			} else if (modern_style) {
				wall_block = get_building_wall_block_for_color(RGB{180, 180, 180}, rng);
			} else {
				wall_block = get_fallback_building_block(rng);
			}
		}
	}

	Block floor_block = get_random_floor_block(rng);
	Block window_block = get_window_block_for_building_type(building_type, rng);

	std::unordered_set<std::pair<int, int>, PairHash> processed_points;
	int building_height = std::max(3, static_cast<int>(6.0 * scale_factor));
	bool is_tall_building = false;

	bool use_vertical_windows = rng.random_bool(0.7);
	bool use_accent_roof_line = rng.random_bool(0.25);

	Block accent_blocks_arr[] = {
			//POLISHED_ANDESITE,
			SMOOTH_STONE, STONE_BRICKS,
			//MUD_BRICKS,
			//ANDESITE,
			//CHISELED_STONE_BRICKS
	};
	Block accent_block = accent_blocks_arr[rng.uniform(static_cast<std::uint32_t>(
			sizeof(accent_blocks_arr) / sizeof(accent_blocks_arr[0])))];

	// building:levels
	{
		auto it = element.tags.find("building:levels");
		if (it != element.tags.end()) {
			try {
				int levels = std::stoi(it->second);
				int lev = levels - min_level;
				if (lev >= 1) {
					building_height = multiply_scale(levels * 4 + 2, scale_factor);
					if (building_height < 3)
						building_height = 3;
					if (levels > 7)
						is_tall_building = true;
				}
			} catch (...) {
			}
		}
	}

	// height tag
	{
		auto it = element.tags.find("height");
		if (it != element.tags.end()) {
			try {
				std::string s = it->second;
				// trim trailing 'm' and whitespace
				while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
					s.pop_back();
				if (!s.empty() && s.back() == 'm')
					s.pop_back();
				while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
					s.pop_back();
				double height = std::stod(s);
				building_height = static_cast<int>(height * scale_factor);
				if (building_height < 3)
					building_height = 3;
				if (height > 28.0)
					is_tall_building = true;
			} catch (...) {
			}
		}
	}

	if (relation_levels.has_value()) {
		int levels = relation_levels.value();
		building_height = multiply_scale(levels * 4 + 2, scale_factor);
		if (building_height < 3)
			building_height = 3;
		if (levels > 7)
			is_tall_building = true;
	}

	bool has_multiple_floors = building_height > 6;
	bool use_accent_lines = has_multiple_floors && rng.random_bool(0.2);
	bool use_vertical_accent =
			has_multiple_floors && !use_accent_lines && rng.random_bool(0.1);

	{
		auto it = element.tags.find("amenity");
		if (it != element.tags.end() && it->second == "shelter") {
			Block roof_block = STONE_BRICK_SLAB;
			const std::vector<std::pair<int, int>> &roof_area = cached_floor_area;
			for (const auto &node : element.nodes) {
				int x = node.x;
				int z = node.z;
				for (int shelter_y = 1; shelter_y <= multiply_scale(4, scale_factor);
						++shelter_y) {
					editor->set_block(OAK_FENCE, x, shelter_y, z);
				}
				editor->set_block(roof_block, x, 5, z);
			}
			for (const auto &p : roof_area) {
				editor->set_block(roof_block, p.first, 5, p.second);
			}
			return;
		}
	}

	{
		auto it = element.tags.find("building");
		if (it != element.tags.end()) {
			const std::string &btype = it->second;
			if (btype == "garage") {
				building_height = std::max(3, static_cast<int>(2.0 * scale_factor));
			} else if (btype == "shed") {
				building_height = std::max(3, static_cast<int>(2.0 * scale_factor));
				if (element.tags.find("bicycle_parking") != element.tags.end()) {
					Block ground_block = OAK_PLANKS;
					Block roof_block = STONE_BLOCK_SLAB;
					const std::vector<std::pair<int, int>> &floor_area =
							cached_floor_area;
					for (const auto &p : floor_area) {
						editor->set_block(ground_block, p.first, 0, p.second);
					}
					for (const auto &node : element.nodes) {
						int x = node.x;
						int z = node.z;
						for (int dy = 1; dy <= 4; ++dy) {
							editor->set_block(OAK_FENCE, x, dy, z);
						}
						editor->set_block(roof_block, x, 5, z);
					}
					for (const auto &p : floor_area) {
						editor->set_block(roof_block, p.first, 5, p.second);
					}
					return;
				}
			} else if (btype == "parking" ||
					   (element.tags.find("parking") != element.tags.end() &&
							   element.tags.at("parking") == "multi-storey")) {
				building_height = std::max(building_height, 16);
				const std::vector<std::pair<int, int>> &floor_area = cached_floor_area;
				int top_level = building_height / 4;
				for (int level = 0; level <= top_level; ++level) {
					int current_level_y = level * 4;
					for (const auto &node : element.nodes) {
						int x = node.x;
						int z = node.z;
						for (int y = current_level_y + 1; y <= current_level_y + 4; ++y) {
							editor->set_block(STONE_BRICKS, x, y, z);
						}
					}
					for (const auto &p : floor_area) {
						if (level == 0) {
							editor->set_block(
									SMOOTH_STONE, p.first, current_level_y, p.second);
						} else {
							editor->set_block(
									COBBLESTONE, p.first, current_level_y, p.second);
						}
					}
				}
				for (int level = 0; level <= top_level; ++level) {
					int current_level_y = level * 4;
					std::optional<std::pair<int, int>> prev_outline;
					for (const auto &node : element.nodes) {
						int x = node.x;
						int z = node.z;
						if (prev_outline.has_value()) {
							auto outline_points =
									bresenham_line(prev_outline->first, current_level_y,
											prev_outline->second, x, current_level_y, z);
							for (const auto &t : outline_points) {
								int bx = std::get<0>(t);
								int bz = std::get<2>(t);
								std::vector<Block> alts = {COBBLESTONE, COBBLESTONE_WALL};
								editor->set_block(SMOOTH_STONE_BLOCK, bx, current_level_y,
										bz, alts);
								editor->set_block(
										STONE_BRICK_SLAB, bx, current_level_y + 2, bz);
								if ((bx % 2) == 0) {
									editor->set_block(COBBLESTONE_WALL, bx,
											current_level_y + 1, bz);
								}
							}
						}
						prev_outline = std::make_pair(x, z);
					}
				}
				return;
			} else if (btype == "roof") {
				generate_roof_only_structure(*editor, element, cached_floor_area, args);
				return;
			} else if (btype == "apartments") {
				if (building_height ==
						std::max(3, static_cast<int>(6.0 * scale_factor))) {
					building_height = std::max(3, static_cast<int>(15.0 * scale_factor));
				}
			} else if (btype == "hospital") {
				if (building_height ==
						std::max(3, static_cast<int>(6.0 * scale_factor))) {
					building_height = std::max(3, static_cast<int>(23.0 * scale_factor));
				}
			} else if (btype == "bridge") {
				generate_bridge(*editor, element, args.timeout_ref());
				return;
			}
		}
	}

	// Process nodes to create walls and corners
	for (const auto &node : element.nodes) {
		int x = node.x;
		int z = node.z;
		if (previous_node.has_value()) {
			auto prev = previous_node.value();
			auto bresenham_points = bresenham_line(
					prev.first, start_y_offset, prev.second, x, start_y_offset, z);
			for (const auto &t : bresenham_points) {
				int bx = std::get<0>(t);
				int bz = std::get<2>(t);
				const bool is_passage = passage_at(effective_passages, bx, bz);

				if (args.terrain && min_level == 0 && !is_passage) {
					int local_ground_level = args.ground_level;
					Ground *grd = editor->get_ground();
					if (grd) {
						auto min_coords = editor->get_min_coords();
						local_ground_level = grd->level(XZPoint::new_point(
								bx - min_coords.first, bz - min_coords.second));
					}
					for (int y = local_ground_level; y <= start_y_offset; ++y) {
						editor->set_block_absolute(
								wall_block, bx, y + abs_terrain_offset, bz);
					}
				}

				const int passage_height =
						std::min(BUILDING_PASSAGE_HEIGHT, building_height);
				const int wall_start = is_passage ? start_y_offset + passage_height + 1
												  : start_y_offset + 1;
				for (int h = wall_start; h <= start_y_offset + building_height; ++h) {
					if (is_tall_building && use_vertical_windows) {
						if (h > start_y_offset + 1 && ((bx + bz) % 3) == 0) {
							editor->set_block_absolute(
									window_block, bx, h + abs_terrain_offset, bz);
						} else {
							editor->set_block_absolute(
									wall_block, bx, h + abs_terrain_offset, bz);
						}
					} else {
						if (h > start_y_offset + 1 && (h % 4) != 0 &&
								((bx + bz) % 6) < 3) {
							editor->set_block_absolute(
									window_block, bx, h + abs_terrain_offset, bz);
						} else {
							bool use_accent_line = use_accent_lines &&
												   h > start_y_offset + 1 && (h % 4) == 0;
							bool use_vertical_accent_here =
									use_vertical_accent && h > start_y_offset + 1 &&
									(h % 4) == 0 && ((bx + bz) % 6) < 3;
							if (use_accent_line || use_vertical_accent_here) {
								editor->set_block_absolute(
										accent_block, bx, h + abs_terrain_offset, bz);
							} else {
								editor->set_block_absolute(
										wall_block, bx, h + abs_terrain_offset, bz);
							}
						}
					}
				}
				if (is_passage && passage_height < building_height) {
					editor->set_block_absolute(floor_block, bx,
							start_y_offset + passage_height + abs_terrain_offset, bz);
				}

				Block roof_line_block = use_accent_roof_line ? accent_block : wall_block;
				editor->set_block_absolute(roof_line_block, bx,
						start_y_offset + building_height + abs_terrain_offset + 1, bz);

				current_building.emplace_back(bx, bz);
				std::get<0>(corner_addup) += bx;
				std::get<1>(corner_addup) += bz;
				std::get<2>(corner_addup) += 1;
			}
		}
		previous_node = std::make_pair(x, z);
	}

	if (std::get<2>(corner_addup) != 0) {
		const std::vector<std::pair<int, int>> &floor_area = cached_floor_area;

		std::vector<int> floor_levels;
		floor_levels.push_back(start_y_offset);
		if (building_height > 6) {
			int num_upper_floors = std::max(1, building_height / 4);
			for (int floor = 1; floor < num_upper_floors; ++floor) {
				floor_levels.push_back(start_y_offset + 2 + (floor * 4));
			}
		}

		for (const auto &p : floor_area) {
			int x = p.first;
			int z = p.second;
			if (processed_points.insert(p).second) {
				const bool is_passage = passage_at(effective_passages, x, z);
				if (args.terrain) {
					Ground *grd = editor->get_ground();
					if (grd) {
						auto min_coords = editor->get_min_coords();
						(void)grd->level(XZPoint::new_point(
								x - min_coords.first, z - min_coords.second));
					} else {
						(void)args.ground_level;
					}
				}

				if (!is_passage) {
					editor->set_block_absolute(
							floor_block, x, start_y_offset + abs_terrain_offset, z);
				}

				if (building_height > 4) {
					const int passage_ceiling =
							start_y_offset +
							std::min(BUILDING_PASSAGE_HEIGHT, building_height);
					for (int h = start_y_offset + 2 + 4;
							h < start_y_offset + building_height; h += 4) {
						if (is_passage && h <= passage_ceiling)
							continue;
						if ((x % 5) == 0 && (z % 5) == 0) {
							editor->set_block_absolute(
									GLOWSTONE, x, h + abs_terrain_offset, z);
						} else {
							editor->set_block_absolute(
									floor_block, x, h + abs_terrain_offset, z);
						}
					}
				} else if ((x % 5) == 0 && (z % 5) == 0) {
					editor->set_block_absolute(GLOWSTONE, x,
							start_y_offset + building_height + abs_terrain_offset, z);
				}

				if (!args.roof || element.tags.find("roof:shape") == element.tags.end() ||
						element.tags.at("roof:shape") == "flat") {
					editor->set_block_absolute(floor_block, x,
							start_y_offset + building_height + abs_terrain_offset + 1, z);
				}
			}
		}

		if (args.interior) {
			std::string btype = "yes";
			auto it = element.tags.find("building");
			if (it != element.tags.end())
				btype = it->second;
			bool skip_interior =
					(btype == "garage" || btype == "shed" || btype == "parking" ||
							btype == "roof" || btype == "bridge");

			// Detect abandoned buildings (similar to Rust version)
			bool is_abandoned_building = false;
			auto it_abandoned = element.tags.find("abandoned");
			if (it_abandoned != element.tags.end() && it_abandoned->second == "yes") {
				is_abandoned_building = true;
			} else {
				auto it_abandoned_building = element.tags.find("abandoned:building");
				if (it_abandoned_building != element.tags.end()) {
					is_abandoned_building = true;
				}
			}

			if (!skip_interior && floor_area.size() > 100) {
				bool has_sloped_roof = false;
				if (args.roof) {
					auto roof_shape = element.tags.find("roof:shape");
					if (roof_shape != element.tags.end())
						has_sloped_roof =
								parse_roof_type(roof_shape->second) != RoofType::Flat;
				}
				CoordinateBitmap no_passages = CoordinateBitmap::new_empty();
				const CoordinateBitmap &interior_passages =
						effective_passages ? *effective_passages : no_passages;
				generate_building_interior(*editor, floor_area, min_x, min_z, max_x,
						max_z, start_y_offset, building_height, wall_block, floor_levels,
						args, element, abs_terrain_offset, is_abandoned_building,
						interior_passages, has_sloped_roof);
			}
		}
	}

	if (args.roof) {
		auto it_shape = element.tags.find("roof:shape");
		if (it_shape != element.tags.end()) {
			RoofType roof_type = parse_roof_type(it_shape->second);
			if (roof_type == RoofType::Cone || roof_type == RoofType::Onion)
				roof_type = RoofType::Dome;

			auto roof_rng = element_rng_salted(visual_seed, 0xF00FC010BA5EF00DULL);
			const Block roof_floor_block =
					roof_block_from_tags(element, roof_rng).value_or(floor_block);

			generate_roof(*editor, element, start_y_offset, building_height,
					roof_floor_block, wall_block, accent_block, roof_type,
					cached_floor_area, abs_terrain_offset);
		} else {
			std::string btype = "yes";
			auto it = element.tags.find("building");
			if (it != element.tags.end())
				btype = it->second;

			if (btype == "apartments" || btype == "residential" || btype == "house" ||
					btype == "yes") {
				std::size_t footprint_size = cached_footprint_size;
				const std::size_t max_footprint_for_gabled = 800;
				if (footprint_size <= max_footprint_for_gabled && rng.random_bool(0.9)) {
					generate_roof(*editor, element, start_y_offset, building_height,
							floor_block, wall_block, accent_block, RoofType::Gabled,
							cached_floor_area, abs_terrain_offset);
				}
			}
		}
	} else {
		// flat roof default - already applied
	}
}

/*
#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <random>
#include <utility>

namespace heck {

// Basic types and helpers
enum class StairFacing { North, South, East, West };
enum class StairShape { Straight, OuterLeft, OuterRight };

struct Block {
    int id;
};

struct BlockWithProperties {
    Block base;
    StairFacing facing;
    StairShape shape;
    bool has_props;

    static BlockWithProperties simple(Block b) {
        return BlockWithProperties{b, StairFacing::North, StairShape::Straight, false};
    }
};

struct Node {
    int32_t x;
    int32_t z;
};

struct ProcessedWay {
    std::vector<Node> nodes;
};

enum class RoofType { Flat, Gabled, Hipped, Skillion, Pyramidal, Dome };

struct WorldEditor {
    void set_block_absolute(Block block, int32_t x, int32_t y, int32_t z, void* a, void* b);
    void set_block_with_properties_absolute(BlockWithProperties bwp, int32_t x, int32_t y, int32_t z, void* a, void* b);
};

// Helpers referenced in the original code (stubs)
inline Block get_stair_block_for_material(Block material) {
    return material;
}

inline BlockWithProperties create_stair_with_properties(Block material, StairFacing facing, StairShape shape) {
    return BlockWithProperties{material, facing, shape, true};
}

// Hash for pair<int32_t,int32_t>
struct pair_hash {
    std::size_t operator()(std::pair<int32_t,int32_t> const& p) const noexcept {
        std::uint64_t a = static_cast<std::uint32_t>(p.first);
        std::uint64_t b = static_cast<std::uint32_t>(p.second);
        std::uint64_t res = (a << 32) ^ b;
        return static_cast<std::size_t>(res ^ (res >> 33));
    }
};
*/

// multiply_scale implementation
inline int32_t multiply_scale(int32_t value, double scale_factor)
{
	if (scale_factor == 1.0) {
		return value;
	} else if (scale_factor == 2.0) {
		return value << 1;
	} else if (scale_factor == 4.0) {
		return value << 2;
	} else {
		double result = static_cast<double>(value) * scale_factor;
		return static_cast<int32_t>(std::floor(result));
	}
}

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset)
{
	using std::int32_t;
	const auto &floor_area = cached_floor_area;

	// Pre-calculate bounds
	int32_t min_x = std::numeric_limits<int32_t>::max();
	int32_t max_x = std::numeric_limits<int32_t>::min();
	int32_t min_z = std::numeric_limits<int32_t>::max();
	int32_t max_z = std::numeric_limits<int32_t>::min();

	for (auto const &n : element.nodes) {
		min_x = std::min(min_x, n.x);
		max_x = std::max(max_x, n.x);
		min_z = std::min(min_z, n.z);
		max_z = std::max(max_z, n.z);
	}

	int32_t center_x = (min_x + max_x) >> 1;
	int32_t center_z = (min_z + max_z) >> 1;

	int32_t base_height = start_y_offset + building_height + 1;

	// Random generator
	auto rng = element_rng_salted(static_cast<std::uint64_t>(element.id), 0x726f6f66);
	const std::optional<Block> tagged_roof_block = roof_block_from_tags(element, rng);

	if (roof_type == RoofType::Flat) {
		for (auto const &p : floor_area) {
			editor.set_block_absolute(floor_block, p.first,
					base_height + abs_terrain_offset, p.second, nullptr, nullptr);
		}
		return;
	}

	if (roof_type == RoofType::Gabled) {
		int32_t width = max_x - min_x;
		int32_t length = max_z - min_z;
		int32_t building_size = std::max(width, length);

		int32_t roof_height_boost = static_cast<int32_t>(
				3.0 + std::max(1.0, std::log(std::max(1.0,
											static_cast<double>(building_size) * 0.15))));
		int32_t roof_peak_height = base_height + roof_height_boost;

		bool is_wider_than_long = width > length;
		int32_t max_distance = is_wider_than_long ? (length >> 1) : (width >> 1);

		Block roof_block =
				tagged_roof_block.value_or(rng.random_bool() ? accent_block : wall_block);

		std::vector<std::pair<std::pair<int32_t, int32_t>, int32_t>> roof_heights;
		roof_heights.reserve(floor_area.size());

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			int32_t distance_to_ridge =
					is_wider_than_long ? std::abs(z - center_z) : std::abs(x - center_x);

			int32_t roof_height;
			if (distance_to_ridge == 0 &&
					((is_wider_than_long && z == center_z) ||
							(!is_wider_than_long && x == center_x))) {
				roof_height = roof_peak_height;
			} else {
				double slope_ratio = static_cast<double>(distance_to_ridge) /
									 static_cast<double>(std::max(1, max_distance));
				roof_height = static_cast<int32_t>(
						static_cast<double>(roof_peak_height) -
						(slope_ratio * static_cast<double>(roof_height_boost)));
			}
			roof_height = std::max(base_height, roof_height);
			roof_heights.push_back({{x, z}, roof_height});
		}

		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash> roof_map;
		roof_map.reserve(roof_heights.size() * 2);
		for (auto const &kv : roof_heights) {
			roof_map[kv.first] = kv.second;
		}

		Block stair_block_material = get_stair_block_for_material(roof_block);
		std::vector<std::tuple<int32_t, int32_t, int32_t, Block,
				std::optional<BlockWithProperties>>>
				blocks_to_place;
		blocks_to_place.reserve(floor_area.size() * 2);

		for (auto const &kv : roof_heights) {
			int32_t x = kv.first.first;
			int32_t z = kv.first.second;
			int32_t roof_height = kv.second;

			bool has_lower_neighbor = false;
			std::pair<int32_t, int32_t> ncoords[4] = {
					{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
			for (auto const &nc : ncoords) {
				auto it = roof_map.find(nc);
				if (it != roof_map.end() && it->second < roof_height) {
					has_lower_neighbor = true;
					break;
				}
			}

			for (int32_t y = base_height; y <= roof_height; ++y) {
				if (y == roof_height && has_lower_neighbor) {
					BlockWithProperties stair_block_with_props;
					if (is_wider_than_long) {
						if (z < center_z) {
							stair_block_with_props =
									create_stair_with_properties(stair_block_material,
											StairFacing::South, StairShape::Straight);
						} else {
							stair_block_with_props =
									create_stair_with_properties(stair_block_material,
											StairFacing::North, StairShape::Straight);
						}
					} else if (x < center_x) {
						stair_block_with_props =
								create_stair_with_properties(stair_block_material,
										StairFacing::East, StairShape::Straight);
					} else {
						stair_block_with_props =
								create_stair_with_properties(stair_block_material,
										StairFacing::West, StairShape::Straight);
					}
					blocks_to_place.emplace_back(x, y, z, roof_block,
							std::optional<BlockWithProperties>(stair_block_with_props));
				} else {
					blocks_to_place.emplace_back(
							x, y, z, roof_block, std::optional<BlockWithProperties>());
				}
			}
		}

		for (auto const &t : blocks_to_place) {
			int32_t x, y, z;
			Block block;
			std::optional<BlockWithProperties> maybe_bwp;
			std::tie(x, y, z, block, maybe_bwp) = t;
			if (maybe_bwp.has_value()) {
				editor.set_block_with_properties_absolute(maybe_bwp.value(), x,
						y + abs_terrain_offset, z, nullptr, nullptr);
			} else {
				editor.set_block_absolute(
						block, x, y + abs_terrain_offset, z, nullptr, nullptr);
			}
		}

		return;
	}

	if (roof_type == RoofType::Hipped) {
		int32_t width = max_x - min_x;
		int32_t length = max_z - min_z;

		bool is_rectangular = ((static_cast<double>(width) / std::max(1, length) > 1.3) ||
							   (static_cast<double>(length) / std::max(1, width) > 1.3));
		bool long_axis_is_x = width > length;

		int32_t roof_peak_height = base_height + ((std::max(width, length) > 20) ? 7 : 5);

		Block roof_block =
				tagged_roof_block.value_or(rng.random_bool() ? accent_block : wall_block);

		if (is_rectangular) {
			std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash>
					roof_heights;
			roof_heights.reserve(floor_area.size() * 2);

			for (auto const &p : floor_area) {
				int32_t x = p.first;
				int32_t z = p.second;

				int32_t distance_to_ridge =
						long_axis_is_x ? std::abs(z - center_z) : std::abs(x - center_x);

				int32_t max_distance_from_ridge =
						long_axis_is_x ? ((max_z - min_z) / 2) : ((max_x - min_x) / 2);

				double slope_factor =
						(max_distance_from_ridge > 0)
								? static_cast<double>(distance_to_ridge) /
										  static_cast<double>(max_distance_from_ridge)
								: 0.0;

				int32_t roof_height =
						roof_peak_height -
						static_cast<int32_t>(
								slope_factor *
								static_cast<double>(roof_peak_height - base_height));
				int32_t roof_y = std::max(base_height, roof_height);
				roof_heights[{x, z}] = roof_y;
			}

			Block stair_block_material = get_stair_block_for_material(roof_block);

			for (auto const &p : floor_area) {
				int32_t x = p.first;
				int32_t z = p.second;
				int32_t roof_height = roof_heights[{x, z}];

				for (int32_t y = base_height; y <= roof_height; ++y) {
					if (y == roof_height) {
						bool has_lower_neighbor = false;
						std::pair<int32_t, int32_t> ncoords[4] = {
								{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
						for (auto const &nc : ncoords) {
							auto it = roof_heights.find(nc);
							if (it != roof_heights.end() && it->second < roof_height) {
								has_lower_neighbor = true;
								break;
							}
						}

						if (has_lower_neighbor) {
							BlockWithProperties stair_block_with_props;
							if (long_axis_is_x) {
								if (z < center_z) {
									stair_block_with_props = create_stair_with_properties(
											stair_block_material, StairFacing::South,
											StairShape::Straight);
								} else {
									stair_block_with_props = create_stair_with_properties(
											stair_block_material, StairFacing::North,
											StairShape::Straight);
								}
							} else {
								if (x < center_x) {
									stair_block_with_props = create_stair_with_properties(
											stair_block_material, StairFacing::East,
											StairShape::Straight);
								} else {
									stair_block_with_props = create_stair_with_properties(
											stair_block_material, StairFacing::West,
											StairShape::Straight);
								}
							}
							editor.set_block_with_properties_absolute(
									stair_block_with_props, x, y + abs_terrain_offset, z,
									nullptr, nullptr);
						} else {
							editor.set_block_absolute(roof_block, x,
									y + abs_terrain_offset, z, nullptr, nullptr);
						}
					} else {
						editor.set_block_absolute(roof_block, x, y + abs_terrain_offset,
								z, nullptr, nullptr);
					}
				}
			}
		} else {
			std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash>
					roof_heights;
			roof_heights.reserve(floor_area.size() * 2);

			for (auto const &p : floor_area) {
				int32_t x = p.first;
				int32_t z = p.second;
				double dx = static_cast<double>(x - center_x);
				double dz = static_cast<double>(z - center_z);
				double distance_from_center = std::sqrt(dx * dx + dz * dz);

				double corner_sq[4] = {
						static_cast<double>((min_x - center_x) * (min_x - center_x) +
											(min_z - center_z) * (min_z - center_z)),
						static_cast<double>((min_x - center_x) * (min_x - center_x) +
											(max_z - center_z) * (max_z - center_z)),
						static_cast<double>((max_x - center_x) * (max_x - center_x) +
											(min_z - center_z) * (min_z - center_z)),
						static_cast<double>((max_x - center_x) * (max_x - center_x) +
											(max_z - center_z) * (max_z - center_z))};
				double max_distance = 0.0;
				for (int i = 0; i < 4; ++i)
					max_distance = std::max(max_distance, corner_sq[i]);
				max_distance = std::sqrt(max_distance);

				double distance_factor =
						(max_distance > 0.0)
								? std::min(1.0, distance_from_center / max_distance)
								: 0.0;

				int32_t roof_height =
						roof_peak_height -
						static_cast<int32_t>(
								distance_factor *
								static_cast<double>(roof_peak_height - base_height));
				int32_t roof_y = std::max(base_height, roof_height);
				roof_heights[{x, z}] = roof_y;
			}

			Block stair_block_material = get_stair_block_for_material(roof_block);

			for (auto const &p : floor_area) {
				int32_t x = p.first;
				int32_t z = p.second;
				int32_t roof_height = roof_heights[{x, z}];

				for (int32_t y = base_height; y <= roof_height; ++y) {
					if (y == roof_height) {
						bool has_lower_neighbor = false;
						std::pair<int32_t, int32_t> ncoords[4] = {
								{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
						for (auto const &nc : ncoords) {
							auto it = roof_heights.find(nc);
							if (it != roof_heights.end() && it->second < roof_height) {
								has_lower_neighbor = true;
								break;
							}
						}

						if (has_lower_neighbor) {
							int32_t center_dx = x - center_x;
							int32_t center_dz = z - center_z;
							BlockWithProperties stair_block;
							if (std::abs(center_dx) > std::abs(center_dz)) {
								if (center_dx > 0) {
									stair_block = create_stair_with_properties(
											stair_block_material, StairFacing::West,
											StairShape::Straight);
								} else {
									stair_block = create_stair_with_properties(
											stair_block_material, StairFacing::East,
											StairShape::Straight);
								}
							} else {
								if (center_dz > 0) {
									stair_block = create_stair_with_properties(
											stair_block_material, StairFacing::North,
											StairShape::Straight);
								} else {
									stair_block = create_stair_with_properties(
											stair_block_material, StairFacing::South,
											StairShape::Straight);
								}
							}
							editor.set_block_with_properties_absolute(stair_block, x,
									y + abs_terrain_offset, z, nullptr, nullptr);
						} else {
							editor.set_block_absolute(roof_block, x,
									y + abs_terrain_offset, z, nullptr, nullptr);
						}
					} else {
						editor.set_block_absolute(roof_block, x, y + abs_terrain_offset,
								z, nullptr, nullptr);
					}
				}
			}
		}

		return;
	}

	if (roof_type == RoofType::Skillion) {
		int32_t width = std::max(1, max_x - min_x);
		int32_t building_size = std::max(max_x - min_x, max_z - min_z);

		int32_t max_roof_height = std::clamp(building_size / 3, 4, 10);

		Block roof_block =
				tagged_roof_block.value_or(rng.random_bool() ? accent_block : wall_block);

		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash> roof_heights;
		roof_heights.reserve(floor_area.size() * 2);

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			double slope_progress =
					static_cast<double>(x - min_x) / static_cast<double>(width);
			int32_t roof_height =
					base_height +
					static_cast<int32_t>(
							slope_progress * static_cast<double>(max_roof_height));
			roof_heights[{x, z}] = roof_height;
		}

		Block stair_block_material = get_stair_block_for_material(roof_block);

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			int32_t roof_height = roof_heights[{x, z}];

			for (int32_t y = base_height; y <= roof_height; ++y) {
				if (y == roof_height) {
					bool has_lower_neighbor = false;
					std::pair<int32_t, int32_t> ncoords[4] = {
							{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
					for (auto const &nc : ncoords) {
						auto it = roof_heights.find(nc);
						if (it != roof_heights.end() && it->second < roof_height) {
							has_lower_neighbor = true;
							break;
						}
					}

					if (has_lower_neighbor) {
						BlockWithProperties stair_block_with_props =
								create_stair_with_properties(stair_block_material,
										StairFacing::East, StairShape::Straight);
						editor.set_block_with_properties_absolute(stair_block_with_props,
								x, y + abs_terrain_offset, z, nullptr, nullptr);
					} else {
						editor.set_block_absolute(roof_block, x, y + abs_terrain_offset,
								z, nullptr, nullptr);
					}
				} else {
					editor.set_block_absolute(
							roof_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
				}
			}
		}

		return;
	}

	if (roof_type == RoofType::Pyramidal) {
		int32_t building_size = std::max(max_x - min_x, max_z - min_z);

		int32_t peak_height = base_height + std::clamp(building_size / 3, 3, 8);

		Block roof_block =
				tagged_roof_block.value_or(rng.random_bool() ? accent_block : wall_block);

		//std::unordered_map<std::pair<int32_t,int32_t>, int32_t, pair_hash> roof_heights;
		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, PairHash> roof_heights;
		roof_heights.reserve(floor_area.size() * 2);

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;

			double dx = static_cast<double>(std::abs(x - center_x));
			double dz = static_cast<double>(std::abs(z - center_z));
			double distance_to_edge = std::max(dx, dz);

			double max_distance = static_cast<double>(
					std::max((max_x - min_x) / 2, (max_z - min_z) / 2));

			double height_factor =
					(max_distance > 0.0)
							? std::max(0.0, 1.0 - (distance_to_edge / max_distance))
							: 1.0;

			int32_t roof_height =
					base_height +
					static_cast<int32_t>(height_factor *
										 static_cast<double>(peak_height - base_height));
			roof_heights[{x, z}] = std::max(base_height, roof_height);
		}

		Block stair_block_material = get_stair_block_for_material(roof_block);

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			int32_t roof_height = roof_heights[{x, z}];

			for (int32_t y = base_height; y <= roof_height; ++y) {
				if (y == roof_height) {
					int32_t dx = x - center_x;
					int32_t dz = z - center_z;

					int32_t north_height = roof_heights.count({x, z - 1})
												   ? roof_heights[{x, z - 1}]
												   : base_height;
					int32_t south_height = roof_heights.count({x, z + 1})
												   ? roof_heights[{x, z + 1}]
												   : base_height;
					int32_t west_height = roof_heights.count({x - 1, z})
												  ? roof_heights[{x - 1, z}]
												  : base_height;
					int32_t east_height = roof_heights.count({x + 1, z})
												  ? roof_heights[{x + 1, z}]
												  : base_height;

					bool has_lower_north = north_height < roof_height;
					bool has_lower_south = south_height < roof_height;
					bool has_lower_west = west_height < roof_height;
					bool has_lower_east = east_height < roof_height;

					BlockWithProperties stair_block;
					if (has_lower_north && has_lower_west) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::East, StairShape::OuterRight);
					} else if (has_lower_north && has_lower_east) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::South, StairShape::OuterRight);
					} else if (has_lower_south && has_lower_west) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::East, StairShape::OuterLeft);
					} else if (has_lower_south && has_lower_east) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::North, StairShape::OuterLeft);
					} else {
						if (std::abs(dx) > std::abs(dz)) {
							if (dx > 0 && east_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::West, StairShape::Straight);
							} else if (dx < 0 && west_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::East, StairShape::Straight);
							} else if (dz > 0 && south_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::North, StairShape::Straight);
							} else if (dz < 0 && north_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::South, StairShape::Straight);
							} else {
								stair_block = BlockWithProperties::simple(roof_block);
							}
						} else {
							if (dz > 0 && south_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::North, StairShape::Straight);
							} else if (dz < 0 && north_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::South, StairShape::Straight);
							} else if (dx > 0 && east_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::West, StairShape::Straight);
							} else if (dx < 0 && west_height < roof_height) {
								stair_block =
										create_stair_with_properties(stair_block_material,
												StairFacing::East, StairShape::Straight);
							} else {
								stair_block = BlockWithProperties::simple(roof_block);
							}
						}
					}

					editor.set_block_with_properties_absolute(
							stair_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
				} else {
					editor.set_block_absolute(
							roof_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
				}
			}
		}

		return;
	}

	if (roof_type == RoofType::Dome) {
		double radius = static_cast<double>(std::max(max_x - min_x, max_z - min_z)) / 2.0;

		Block roof_block =
				tagged_roof_block.value_or(rng.random_bool() ? accent_block : wall_block);

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			double distance_from_center_sq = static_cast<double>(
					(x - center_x) * (x - center_x) + (z - center_z) * (z - center_z));
			double normalized_distance = std::min(
					1.0, std::sqrt(distance_from_center_sq) / std::max(1.0, radius));

			double height_factor = std::sqrt(
					std::max(0.0, 1.0 - normalized_distance * normalized_distance));
			int32_t surface_height =
					base_height + static_cast<int32_t>(height_factor * (radius * 0.8));

			for (int32_t y = base_height; y <= surface_height; ++y) {
				editor.set_block_absolute(
						roof_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
			}
		}

		return;
	}
}

/*
#include <vector>
#include <string>
#include <optional>
#include <tuple>
#include <chrono>
#include <unordered_map>
#include <utility>
#include <stdexcept>
*/

void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args)
{
	FloodFillCache empty_cache;
	CoordinateBitmap empty_passages = CoordinateBitmap::new_empty();
	auto min_coords = editor.get_min_coords();
	auto max_coords = editor.get_max_coords();
	XZBBox xzbbox(
			min_coords.first, min_coords.second, max_coords.first, max_coords.second);
	generate_building_from_relation(
			editor, relation, args, empty_cache, xzbbox, empty_passages);
}

void generate_building_from_relation(WorldEditor &editor,
		const ProcessedRelation &relation, const Args &args,
		const FloodFillCache &flood_fill_cache, const XZBBox &xzbbox,
		const CoordinateBitmap &building_passages)
{
	(void)xzbbox;
	if (should_skip_underground_tags(relation.tags)) {
		return;
	}

	int relation_levels = parse_i32_tag(relation.tags, "building:levels").value_or(2);
	const bool is_building_type = relation.tags.get("type") == "building";
	bool has_parts = false;
	if (is_building_type) {
		for (const auto &member : relation.members) {
			if (member.role == ProcessedMemberRole::Part) {
				has_parts = true;
				break;
			}
		}
	}
	if (has_parts) {
		return;
	}

	auto outer_rings = collect_merged_rings(relation, ProcessedMemberRole::Outer);
	auto inner_rings = collect_merged_rings(relation, ProcessedMemberRole::Inner);

	std::vector<HolePolygon> hole_polygons;
	hole_polygons.reserve(inner_rings.size());
	for (std::size_t i = 0; i < inner_rings.size(); ++i) {
		ProcessedWay way;
		way.id = static_cast<std::int64_t>(
				(1ULL << 63) |
				((static_cast<std::uint64_t>(relation.id) & 0x7FFF'FFFFULL) << 16) |
				(0x8000ULL | (i & 0x7FFFULL)));
		way.nodes = std::move(inner_rings[i]);
		hole_polygons.push_back(HolePolygon{std::move(way), true});
	}

	for (std::size_t i = 0; i < outer_rings.size(); ++i) {
		ProcessedWay merged_way;
		merged_way.id = static_cast<std::int64_t>(
				(1ULL << 63) |
				((static_cast<std::uint64_t>(relation.id) & 0x7FFF'FFFFULL) << 16) |
				(i & 0xFFFFULL));
		merged_way.tags = relation.tags;
		merged_way.nodes = std::move(outer_rings[i]);
		generate_buildings(&editor, merged_way, args, std::optional<int>(relation_levels),
				flood_fill_cache, building_passages,
				hole_polygons.empty() ? nullptr : &hole_polygons);
	}
}

void generate_bridge(WorldEditor &editor, const ProcessedWay &element,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout)
{
	// Need at least 2 nodes for a bridge
	if (element.nodes.size() < 2) {
		return;
	}

	// Get start and end node elevations and use MAX for level bridge deck
	// Using MAX ensures bridges don't dip when multiple bridge ways meet in a valley
	int bridge_deck_ground_y = 0;
	if (!element.nodes.empty()) {
		const auto &start_node = element.nodes.front();
		const auto &end_node = element.nodes.back();
		// Get ground reference from editor
		auto *ground = editor.get_ground();
		if (ground) {
			int start_y = ground->level(XZPoint(start_node.x, start_node.z));
			int end_y = ground->level(XZPoint(end_node.x, end_node.z));
			bridge_deck_ground_y = std::max(start_y, end_y);
		}
	}

	Block floor_block = STONE;
	Block railing_block = STONE_BRICKS;

	std::optional<std::pair<int, int>> previous_node = std::nullopt;
	for (const auto &node : element.nodes) {
		int x = node.x;
		int z = node.z;

		int bridge_y_offset = 1;
		auto it = element.tags.find(std::string("level"));
		if (it != element.tags.end()) {
			try {
				int level = std::stoi(it->second);
				bridge_y_offset = (level * 3) + 1;
			} catch (const std::exception &) {
				bridge_y_offset = 1;
			}
		}

		if (previous_node.has_value()) {
			auto prev = previous_node.value();
			std::vector<std::tuple<int, int, int>> bridge_points = bresenham_line(
					prev.first, bridge_y_offset, prev.second, x, bridge_y_offset, z);

			for (const auto &tp : bridge_points) {
				int bx = std::get<0>(tp);
				int bz = std::get<2>(tp);
				// Use fixed bridge deck height (max of endpoints)
				int bridge_y = bridge_deck_ground_y + bridge_y_offset;
				editor.set_block(
						railing_block, bx, bridge_y + 1, bz, std::nullopt, std::nullopt);
				editor.set_block(
						railing_block, bx, bridge_y, bz, std::nullopt, std::nullopt);
			}
		}

		previous_node = std::make_pair(x, z);
	}

	std::vector<std::pair<int, int>> polygon_coords;
	polygon_coords.reserve(element.nodes.size());
	for (const auto &n : element.nodes)
		polygon_coords.emplace_back(n.x, n.z);

	std::vector<std::pair<int, int>> bridge_area =
			flood_fill_area(polygon_coords, floodfill_timeout);

	int bridge_y_offset = 1;
	auto it2 = element.tags.find(std::string("level"));
	if (it2 != element.tags.end()) {
		try {
			int level = std::stoi(it2->second);
			bridge_y_offset = (level * 3) + 1;
		} catch (const std::exception &) {
			bridge_y_offset = 1;
		}
	}

	// Use the same level bridge deck height for filled areas
	int floor_y = bridge_deck_ground_y + bridge_y_offset;

	for (const auto &p : bridge_area) {
		int x = p.first;
		int z = p.second;
		editor.set_block(floor_block, x, floor_y, z, std::nullopt, std::nullopt);
	}
}

}
}
