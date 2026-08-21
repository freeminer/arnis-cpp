#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <iterator>
#include <limits>
#include <numbers>
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
#include "building_facade.h"
#include "signage.h"
#include "../decals/pictograms.h"
#include "../decals/font.h"
#include "historic.h"
#include "subprocessor/buildings_interior.h"
#include "../deterministic_rng.h"
#include "../block_palette.h"
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


// RoofType enum
enum class RoofType
{
	Mansard,
	Gambrel,
	HalfHipped,
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

BuildingCondition building_condition_from_tags(const tags_t &tags)
{
	const auto building = tags.get("building");
	if (building == "ruins" || building == "collapsed" ||
			tags.get("historic") == "ruins" || tags.get("ruins") == "yes" ||
			tags.contains("ruins:building"))
		return BuildingCondition::Ruined;
	if (tags.get("abandoned") == "yes" || tags.contains("abandoned:building"))
		return BuildingCondition::Abandoned;
	if (tags.contains("disused:building"))
		return BuildingCondition::Disused;
	if (building == "construction" || tags.contains("construction:building"))
		return BuildingCondition::Construction;
	return BuildingCondition::Normal;
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

ArchEra building_arch_era(const tags_t &tags)
{
	auto positive = [&](const char *key) {
		auto it = tags.find(key);
		return it != tags.end() && normalized_material(it->second) != "no";
	};
	if (positive("historic") || positive("heritage") || tags.contains("ref:nrhp") ||
			positive("listed_status"))
		return ArchEra::HistoricOrnate;
	const auto architecture = normalized_material(
			!tags.get("building:architecture").empty() ? tags.get("building:architecture")
													   : tags.get("architecture"));
	if (architecture == "gothic" || architecture == "baroque" ||
			architecture == "renaissance" || architecture == "neoclassical" ||
			architecture == "artnouveau")
		return ArchEra::HistoricOrnate;
	if (architecture == "brutalist" || architecture == "brutalism" ||
			architecture == "prefabricated" || architecture == "panel")
		return ArchEra::PostWarPanel;
	if (architecture == "modern" || architecture == "modernism" ||
			architecture == "contemporary" || architecture == "functionalism")
		return ArchEra::Contemporary;
	for (const char *key : {"start_date", "construction_date", "year_of_construction"}) {
		const auto value = tags.get(key);
		for (std::size_t i = 0; i + 3 < value.size(); ++i)
			if (std::isdigit(static_cast<unsigned char>(value[i])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 1])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 2])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 3]))) {
				const int year = std::stoi(value.substr(i, 4));
				return year < 1945	 ? ArchEra::TraditionalPreWar
					   : year < 1980 ? ArchEra::PostWarPanel
									 : ArchEra::Contemporary;
			}
	}
	const auto material =
			normalized_material(!tags.get("building:material").empty()
										? tags.get("building:material")
										: tags.get("building:facade:material"));
	if (material == "prefabricated" || material == "panel" || material == "plates")
		return ArchEra::PostWarPanel;
	if (material == "brick" || material == "stone" || material == "masonry" ||
			material == "sandstone" || material == "terracotta" || material == "wood")
		return ArchEra::TraditionalPreWar;
	if (material == "concrete" || material == "reinforcedconcrete" ||
			material == "glass" || material == "mirror")
		return ArchEra::Contemporary;
	return ArchEra::Unknown;
}

BuildingCategory building_category(const ProcessedWay &element, bool tall, int height,
		double scale, std::uint64_t seed)
{
	if (element.tags.get("man_made") == "tower")
		return BuildingCategory::Tower;
	if (tall) {
		const auto material = normalized_material(element.tags.get("building:material"));
		const auto architecture =
				normalized_material(element.tags.get("building:architecture"));
		if (material == "glass" || material == "mirror") {
			switch ((seed ^ 0x6C07A55EULL) * 2654435761ULL % 3) {
			case 0:
				return BuildingCategory::GlassySkyscraper;
			case 1:
				return BuildingCategory::GridSkyscraper;
			default:
				return BuildingCategory::GlassCornerSkyscraper;
			}
		}
		if (architecture == "artdeco" || architecture == "gothic" ||
				material == "brick" || material == "stone")
			return BuildingCategory::MasonrySkyscraper;
		int min_x = INT_MAX, max_x = INT_MIN, min_z = INT_MAX, max_z = INT_MIN;
		for (const auto &node : element.nodes) {
			min_x = std::min(min_x, node.x);
			max_x = std::max(max_x, node.x);
			min_z = std::min(min_z, node.z);
			max_z = std::max(max_z, node.z);
		}
		const bool skyscraper = height >= int(120 * scale) &&
								height >= 2 * std::max(max_x - min_x, max_z - min_z);
		if (skyscraper) {
			const auto variant = (seed * 2654435761ULL) % 100;
			if (variant < 18)
				return BuildingCategory::GlassySkyscraper;
			if (variant < 30)
				return BuildingCategory::GlassCornerSkyscraper;
			if (variant < 45)
				return BuildingCategory::GridSkyscraper;
			if (variant < 70)
				return BuildingCategory::ContemporarySkyscraper;
			if (variant < 85)
				return BuildingCategory::ModernSkyscraper;
		}
		return BuildingCategory::TallBuilding;
	}
	const auto type = !element.tags.get("building").empty()
							  ? element.tags.get("building")
							  : element.tags.get("building:part");
	if (type == "religious" || type == "church" || type == "cathedral" ||
			type == "chapel" || type == "mosque" || type == "synagogue" ||
			type == "temple" || element.tags.get("amenity") == "place_of_worship")
		return BuildingCategory::Religious;
	if (element.tags.contains("historic"))
		return BuildingCategory::Historic;
	if (type == "house" || type == "detached" || type == "semidetached_house" ||
			type == "terrace" || type == "bungalow" || type == "villa" ||
			type == "cabin" || type == "hut")
		return BuildingCategory::House;
	if (type == "residential" || type == "apartments" || type == "dormitory")
		return BuildingCategory::Residential;
	if (type == "farm" || type == "farm_auxiliary" || type == "barn" ||
			type == "stable" || type == "cowshed")
		return BuildingCategory::Farm;
	if (type == "commercial" || type == "retail" || type == "supermarket" ||
			type == "kiosk" || type == "shop")
		return BuildingCategory::Commercial;
	if (type == "office")
		return BuildingCategory::Office;
	if (type == "hotel")
		return BuildingCategory::Hotel;
	if (type == "industrial" || type == "factory" || type == "manufacture" ||
			type == "hangar")
		return BuildingCategory::Industrial;
	if (type == "warehouse" || type == "storage_tank")
		return BuildingCategory::Warehouse;
	if (type == "school" || type == "kindergarten" || type == "college" ||
			type == "university" || type == "public" || type == "government" ||
			type == "civic")
		return BuildingCategory::School;
	if (type == "hospital")
		return BuildingCategory::Hospital;
	if (type == "tower" || type == "clock_tower" || type == "transformer_tower")
		return BuildingCategory::Tower;
	if (type == "castle" || type == "ruins" || type == "fort" || type == "bunker")
		return BuildingCategory::Historic;
	if (type == "garage" || type == "garages" || type == "carport")
		return BuildingCategory::Garage;
	if (type == "shed")
		return BuildingCategory::Shed;
	if (type == "greenhouse" || type == "glasshouse")
		return BuildingCategory::Greenhouse;
	return BuildingCategory::Default;
}

Block category_wall_block(
		BuildingCategory category, ArchEra era, biome::Climate climate, ChaCha8Rng &rng)
{
	std::vector<Block> palette;
	switch (category) {
	case BuildingCategory::House:
	case BuildingCategory::Residential:
		palette = {BRICK, STONE_BRICKS, WHITE_TERRACOTTA, BROWN_TERRACOTTA, SANDSTONE,
				SMOOTH_SANDSTONE, QUARTZ_BRICKS, MUD_BRICKS, POLISHED_GRANITE,
				END_STONE_BRICKS, GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, RED_TERRACOTTA};
		if (climate == biome::Climate::Boreal || climate == biome::Climate::Tundra ||
				climate == biome::Climate::IceCap)
			palette.insert(palette.end(), {SPRUCE_PLANKS, SPRUCE_PLANKS, OAK_PLANKS});
		break;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::Hotel:
		palette = {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, POLISHED_ANDESITE,
				SMOOTH_STONE, QUARTZ_BLOCK, QUARTZ_BRICKS, STONE_BRICKS};
		break;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		palette = {GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, STONE, SMOOTH_STONE,
				POLISHED_ANDESITE, DEEPSLATE_BRICKS, BLACKSTONE};
		break;
	case BuildingCategory::Religious:
		palette = {STONE_BRICKS, CHISELED_STONE_BRICKS, QUARTZ_BLOCK, WHITE_CONCRETE,
				SANDSTONE, SMOOTH_SANDSTONE, POLISHED_DIORITE, END_STONE_BRICKS,
				WAXED_OXIDIZED_COPPER};
		break;
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		palette = {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, QUARTZ_BRICKS, STONE_BRICKS,
				POLISHED_ANDESITE, SMOOTH_STONE, SANDSTONE, END_STONE_BRICKS};
		break;
	case BuildingCategory::Farm:
		palette = {OAK_PLANKS, SPRUCE_PLANKS, DARK_OAK_PLANKS, COBBLESTONE, STONE,
				MUD_BRICKS, MOSSY_COBBLESTONE, BROWN_TERRACOTTA};
		break;
	case BuildingCategory::Historic:
	case BuildingCategory::MasonrySkyscraper:
	case BuildingCategory::Tower:
		palette = {STONE_BRICKS, CRACKED_STONE_BRICKS, CHISELED_STONE_BRICKS, COBBLESTONE,
				SANDSTONE, SMOOTH_SANDSTONE, POLISHED_BLACKSTONE_BRICKS, DEEPSLATE_BRICKS,
				ANDESITE, BRICK};
		break;
	case BuildingCategory::Garage:
		palette = {BRICK, STONE_BRICKS, POLISHED_ANDESITE, COBBLESTONE, SMOOTH_STONE,
				LIGHT_GRAY_CONCRETE};
		break;
	case BuildingCategory::Shed:
		return OAK_LOG;
	case BuildingCategory::Greenhouse:
		palette = {
				GLASS, CYAN_STAINED_GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS};
		break;
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
		palette = {GRAY_STAINED_GLASS, CYAN_STAINED_GLASS, BLUE_STAINED_GLASS,
				LIGHT_BLUE_STAINED_GLASS};
		break;
	default:
		palette = {WHITE_CONCRETE, LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, POLISHED_ANDESITE,
				SMOOTH_STONE, QUARTZ_BLOCK};
		break;
	}
	const bool era_filtered = category == BuildingCategory::House ||
							  category == BuildingCategory::Residential ||
							  category == BuildingCategory::Commercial ||
							  category == BuildingCategory::Office ||
							  category == BuildingCategory::Hotel ||
							  category == BuildingCategory::Default;
	if (era_filtered && era != ArchEra::Unknown) {
		auto allowed = [&](Block b) {
			if (era == ArchEra::TraditionalPreWar)
				return b == BRICK || b == STONE_BRICKS || b == WHITE_TERRACOTTA ||
					   b == BROWN_TERRACOTTA || b == SANDSTONE || b == SMOOTH_SANDSTONE ||
					   b == MUD_BRICKS || b == GRANITE || b == POLISHED_GRANITE ||
					   b == TERRACOTTA || b == END_STONE_BRICKS || b == QUARTZ_BRICKS ||
					   b == ORANGE_TERRACOTTA || b == RED_TERRACOTTA ||
					   b == NETHER_BRICK || b == COBBLESTONE || b == OAK_PLANKS ||
					   b == SPRUCE_PLANKS;
			if (era == ArchEra::PostWarPanel)
				return b == GRAY_CONCRETE || b == LIGHT_GRAY_CONCRETE ||
					   b == WHITE_CONCRETE || b == BROWN_CONCRETE ||
					   b == GRAY_TERRACOTTA || b == LIGHT_GRAY_TERRACOTTA ||
					   b == POLISHED_ANDESITE || b == SMOOTH_STONE || b == BRICK ||
					   b == WHITE_TERRACOTTA;
			if (era == ArchEra::Contemporary)
				return b == WHITE_CONCRETE || b == LIGHT_GRAY_CONCRETE ||
					   b == GRAY_CONCRETE || b == QUARTZ_BLOCK || b == QUARTZ_BRICKS ||
					   b == POLISHED_ANDESITE || b == SMOOTH_STONE ||
					   b == POLISHED_DEEPSLATE || b == DEEPSLATE_BRICKS ||
					   b == POLISHED_BLACKSTONE || b == LIGHT_GRAY_TERRACOTTA;
			return b == SANDSTONE || b == SMOOTH_SANDSTONE || b == STONE_BRICKS ||
				   b == CHISELED_STONE_BRICKS || b == END_STONE_BRICKS ||
				   b == QUARTZ_BRICKS || b == WHITE_TERRACOTTA || b == BRICK ||
				   b == POLISHED_DIORITE || b == GRANITE || b == POLISHED_GRANITE;
		};
		std::vector<Block> filtered;
		std::copy_if(
				palette.begin(), palette.end(), std::back_inserter(filtered), allowed);
		if (!filtered.empty())
			palette = std::move(filtered);
	}
	Block picked = palette[rng.uniform(static_cast<std::uint32_t>(palette.size()))];
	for (int attempt = 0; attempt < 8 &&
						  (picked == NETHER_BRICK || picked == RED_NETHER_BRICK ||
								  picked == RED_TERRACOTTA) &&
						  !rng.random_bool(.2);
			++attempt)
		picked = palette[rng.uniform(static_cast<std::uint32_t>(palette.size()))];
	return picked;
}

DetailTier compute_detail_tier(const ProcessedWay &element, BuildingCategory category,
		std::size_t footprint, int height, bool street_facing)
{
	int score = std::min(25, int(footprint / 40)) + std::min(25, height * 2);
	if (category == BuildingCategory::Historic || category == BuildingCategory::Religious)
		score += 20;
	else if (category == BuildingCategory::Commercial ||
			 category == BuildingCategory::Hotel || category == BuildingCategory::Office)
		score += 10;
	else if (category == BuildingCategory::Garage || category == BuildingCategory::Shed ||
			 category == BuildingCategory::Greenhouse)
		score -= 20;
	if (element.tags.contains("wikidata"))
		score += 15;
	const auto tourism = element.tags.get("tourism");
	if (tourism == "attraction" || tourism == "museum" || tourism == "gallery" ||
			tourism == "viewpoint")
		score += 15;
	if (!element.tags.get("heritage").empty() && element.tags.get("heritage") != "no")
		score += 20;
	if (!element.tags.get("historic").empty() && element.tags.get("historic") != "no")
		score += 15;
	if (element.tags.contains("building:architecture") ||
			element.tags.contains("architecture"))
		score += 10;
	if (street_facing)
		score += 10;
	return score <= 24	 ? DetailTier::Minimal
		   : score <= 55 ? DetailTier::Standard
		   : score <= 80 ? DetailTier::Enhanced
						 : DetailTier::Landmark;
}

enum class WindowArchetype
{
	Standard3,
	PairedNarrow,
	VerticalStrip,
	WideHorizontal,
	ArchedTraditional
};

enum class FloorRole
{
	Ground,
	Body,
	Top
};

enum class BalconyBand
{
	Scattered,
	EveryBay,
	Alternating
};

enum class WindowFrameStyle
{
	SpruceCottage,
	DarkTimber,
	StoneOrnate,
	Blackstone,
	RusticMossy,
	TerracottaCopper,
	QuartzModern
};

Block window_frame_material(WindowFrameStyle style)
{
	switch (style) {
	case WindowFrameStyle::SpruceCottage:
		return SPRUCE_PLANKS;
	case WindowFrameStyle::DarkTimber:
		return DARK_OAK_PLANKS;
	case WindowFrameStyle::StoneOrnate:
		return POLISHED_DIORITE;
	case WindowFrameStyle::Blackstone:
		return POLISHED_BLACKSTONE;
	case WindowFrameStyle::RusticMossy:
		return MOSSY_COBBLESTONE;
	case WindowFrameStyle::TerracottaCopper:
		return WAXED_COPPER_BLOCK;
	case WindowFrameStyle::QuartzModern:
		return QUARTZ_BLOCK;
	}
	return STONE_BRICKS;
}

std::optional<WindowFrameStyle> pick_window_frame(BuildingCategory category, ArchEra era,
		DetailTier detail, Block wall, std::uint64_t seed)
{
	if (detail == DetailTier::Minimal ||
			(category != BuildingCategory::House &&
					category != BuildingCategory::Residential &&
					category != BuildingCategory::Commercial &&
					category != BuildingCategory::Hotel &&
					category != BuildingCategory::Historic))
		return std::nullopt;
	std::vector<WindowFrameStyle> pool;
	double chance = .9;
	switch (era) {
	case ArchEra::HistoricOrnate:
		pool = {WindowFrameStyle::StoneOrnate, WindowFrameStyle::RusticMossy};
		chance = .95;
		break;
	case ArchEra::TraditionalPreWar:
		pool = {WindowFrameStyle::SpruceCottage, WindowFrameStyle::DarkTimber,
				WindowFrameStyle::StoneOrnate, WindowFrameStyle::RusticMossy,
				WindowFrameStyle::TerracottaCopper};
		break;
	case ArchEra::PostWarPanel:
		pool = {WindowFrameStyle::QuartzModern};
		chance = .15;
		break;
	case ArchEra::Contemporary:
		pool = {WindowFrameStyle::QuartzModern, WindowFrameStyle::Blackstone};
		chance = .7;
		break;
	case ArchEra::Unknown:
		if (category == BuildingCategory::Commercial ||
				category == BuildingCategory::Hotel)
			pool = {WindowFrameStyle::QuartzModern, WindowFrameStyle::Blackstone,
					WindowFrameStyle::StoneOrnate};
		else
			pool = {WindowFrameStyle::SpruceCottage, WindowFrameStyle::DarkTimber,
					WindowFrameStyle::StoneOrnate, WindowFrameStyle::RusticMossy,
					WindowFrameStyle::TerracottaCopper};
		break;
	}
	if (detail == DetailTier::Enhanced)
		chance = std::min(.95, chance + .15);
	else if (detail == DetailTier::Landmark)
		chance = std::min(.95, chance + .3);
	auto fits_wall = [&](WindowFrameStyle style) {
		if (wall == WHITE_CONCRETE || wall == LIGHT_GRAY_CONCRETE ||
				wall == GRAY_CONCRETE || wall == QUARTZ_BLOCK || wall == QUARTZ_BRICKS ||
				wall == SMOOTH_STONE || wall == POLISHED_ANDESITE ||
				wall == POLISHED_DIORITE)
			return style == WindowFrameStyle::QuartzModern ||
				   style == WindowFrameStyle::Blackstone ||
				   style == WindowFrameStyle::StoneOrnate;
		if (wall == OAK_PLANKS || wall == SPRUCE_PLANKS || wall == DARK_OAK_PLANKS ||
				wall == OAK_LOG || wall == SPRUCE_LOG)
			return style == WindowFrameStyle::SpruceCottage ||
				   style == WindowFrameStyle::DarkTimber ||
				   style == WindowFrameStyle::RusticMossy;
		if (wall == BRICK || wall == RED_TERRACOTTA || wall == ORANGE_TERRACOTTA ||
				wall == TERRACOTTA || wall == BROWN_TERRACOTTA ||
				wall == RED_NETHER_BRICKS || wall == NETHER_BRICK || wall == GRANITE ||
				wall == POLISHED_GRANITE)
			return style == WindowFrameStyle::StoneOrnate ||
				   style == WindowFrameStyle::TerracottaCopper ||
				   style == WindowFrameStyle::DarkTimber;
		if (wall == SANDSTONE || wall == SMOOTH_SANDSTONE || wall == END_STONE_BRICKS ||
				wall == WHITE_TERRACOTTA)
			return style == WindowFrameStyle::StoneOrnate ||
				   style == WindowFrameStyle::TerracottaCopper ||
				   style == WindowFrameStyle::QuartzModern;
		return true;
	};
	pool.erase(std::remove_if(pool.begin(), pool.end(),
					   [&](auto style) { return !fits_wall(style); }),
			pool.end());
	if (pool.empty())
		return std::nullopt;
	auto rng = element_rng_salted(seed, 0xF7A3E00157BD2210ULL);
	if (!rng.random_bool(chance))
		return std::nullopt;
	return pool[rng.uniform(static_cast<std::uint32_t>(pool.size()))];
}

BalconyBand pick_balcony_band(BuildingCategory category, int height, int floor_cycle,
		bool has_street, std::uint64_t seed)
{
	if ((category != BuildingCategory::Residential &&
				category != BuildingCategory::House) ||
			height < 3 * floor_cycle || !has_street)
		return BalconyBand::Scattered;
	const auto roll = element_rng_salted(seed, 0xBA1C041700000009ULL).uniform(100);
	return roll < 30   ? BalconyBand::Scattered
		   : roll < 60 ? BalconyBand::EveryBay
					   : BalconyBand::Alternating;
}

Block base_course_for_wall(Block wall)
{
	if (wall == OAK_PLANKS || wall == SPRUCE_PLANKS || wall == OAK_LOG ||
			wall == SPRUCE_LOG)
		return COBBLESTONE;
	if (wall == ANDESITE || wall == GRAY_CONCRETE || wall == LIGHT_GRAY_CONCRETE)
		return POLISHED_ANDESITE;
	return STONE_BRICKS;
}

WallDepthStyle wall_depth_style_for(BuildingCategory category, ArchEra era,
		DetailTier detail, std::size_t footprint, std::uint64_t seed)
{
	if (footprint < 20 || detail == DetailTier::Minimal)
		return WallDepthStyle::None;
	if (era == ArchEra::HistoricOrnate &&
			(category == BuildingCategory::House ||
					category == BuildingCategory::Residential ||
					category == BuildingCategory::Commercial))
		return WallDepthStyle::HistoricOrnate;
	if (era == ArchEra::PostWarPanel &&
			(category == BuildingCategory::House ||
					category == BuildingCategory::Residential))
		return WallDepthStyle::None;
	if (era == ArchEra::Contemporary && category == BuildingCategory::Residential &&
			element_rng_salted(seed, 0xE5A011DE57A10003ULL).random_bool(.5))
		return WallDepthStyle::ModernPillars;
	switch (category) {
	case BuildingCategory::House:
	case BuildingCategory::Residential:
		return WallDepthStyle::SubtlePilasters;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::Hotel:
		return WallDepthStyle::ModernPillars;
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		return WallDepthStyle::InstitutionalBands;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		return WallDepthStyle::IndustrialBeams;
	case BuildingCategory::Historic:
	case BuildingCategory::MasonrySkyscraper:
		return WallDepthStyle::HistoricOrnate;
	case BuildingCategory::Religious:
		return WallDepthStyle::ReligiousButtress;
	case BuildingCategory::TallBuilding:
	case BuildingCategory::ModernSkyscraper:
		return WallDepthStyle::SkyscraperFins;
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
		return WallDepthStyle::GlassCurtain;
	default:
		return WallDepthStyle::None;
	}
}

Block weathered_variant(Block block, std::uint64_t hash)
{
	if (block == STONE_BRICKS) {
		const std::array options{MOSSY_STONE_BRICKS, CRACKED_STONE_BRICKS, STONE_BRICKS};
		return options[hash % options.size()];
	}
	if (block == COBBLESTONE)
		return MOSSY_COBBLESTONE;
	if (block == STONE)
		return (hash & 1) ? ANDESITE : COBBLESTONE;
	if (block == SMOOTH_STONE)
		return STONE;
	if (block == POLISHED_ANDESITE)
		return ANDESITE;
	if (block == OAK_PLANKS)
		return (hash & 1) ? SPRUCE_PLANKS : DARK_OAK_PLANKS;
	if (block == OAK_LOG)
		return SPRUCE_LOG;
	return block;
}

Block apply_condition_variation(Block chosen, int x, int y, int z, Block wall_block,
		Block window_block, bool has_windows, BuildingCondition condition,
		BuildingCategory category, ArchEra era, std::uint64_t seed)
{
	if (condition == BuildingCondition::Construction)
		return chosen;
	std::uint64_t hash = std::uint64_t(std::uint32_t(x)) * 0x9E3779B97F4A7C15ULL;
	hash ^= std::uint64_t(std::uint32_t(z)) * 0x517CC1B727220A95ULL;
	hash ^= seed ^ (std::uint64_t(std::uint32_t(y)) << 16);
	hash ^= hash >> 29;
	const double roll = double(hash % 10000) / 10000.0;
	const bool window = has_windows && chosen == window_block;
	const double board_rate = condition == BuildingCondition::Disused	  ? .30
							  : condition == BuildingCondition::Abandoned ? .50
																		  : 0.0;
	if (window && roll < board_rate)
		return wall_block;
	if (window)
		return chosen;
	double weather_rate = 0.0;
	if (condition == BuildingCondition::Abandoned)
		weather_rate = .05;
	else if (condition == BuildingCondition::Ruined)
		weather_rate = .50;
	else if (condition == BuildingCondition::Normal) {
		if (category == BuildingCategory::Historic ||
				category == BuildingCategory::Religious)
			weather_rate = .06;
		else if (category == BuildingCategory::Farm)
			weather_rate = .03;
		weather_rate = std::max(weather_rate, era == ArchEra::HistoricOrnate	  ? .05
											  : era == ArchEra::TraditionalPreWar ? .02
																				  : 0.0);
	}
	return roll < weather_rate ? weathered_variant(chosen, hash >> 17) : chosen;
}

bool archetype_allows_window(
		WindowArchetype archetype, int column, int floor_row, int floor_cycle)
{
	switch (archetype) {
	case WindowArchetype::Standard3:
	case WindowArchetype::ArchedTraditional:
		return column < 3;
	case WindowArchetype::PairedNarrow:
		return (column == 0 || column == 2) && floor_row < floor_cycle - 1;
	case WindowArchetype::VerticalStrip:
		return column == 1;
	case WindowArchetype::WideHorizontal:
		return column < 4 && floor_row > 1;
	}
	return false;
}

WindowArchetype pick_window_archetype(
		BuildingCategory category, ArchEra era, std::uint64_t seed)
{
	using W = WindowArchetype;
	std::vector<std::pair<W, std::uint32_t>> table;
	switch (category) {
	case BuildingCategory::House:
		table = {{W::Standard3, 40}, {W::PairedNarrow, 35}, {W::VerticalStrip, 5},
				{W::WideHorizontal, 10}, {W::ArchedTraditional, 10}};
		break;
	case BuildingCategory::Residential:
		table = {{W::Standard3, 35}, {W::PairedNarrow, 30}, {W::VerticalStrip, 10},
				{W::WideHorizontal, 15}, {W::ArchedTraditional, 10}};
		break;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
		table = {{W::Standard3, 25}, {W::PairedNarrow, 5}, {W::VerticalStrip, 20},
				{W::WideHorizontal, 45}, {W::ArchedTraditional, 5}};
		break;
	case BuildingCategory::Hotel:
		table = {{W::Standard3, 30}, {W::PairedNarrow, 10}, {W::VerticalStrip, 20},
				{W::WideHorizontal, 35}, {W::ArchedTraditional, 5}};
		break;
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		table = {{W::Standard3, 30}, {W::PairedNarrow, 10}, {W::VerticalStrip, 10},
				{W::WideHorizontal, 50}};
		break;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		table = {{W::Standard3, 20}, {W::VerticalStrip, 10}, {W::WideHorizontal, 70}};
		break;
	default:
		return W::Standard3;
	}
	auto rng = element_rng_salted(seed, 0x57A2C0DEA5C10007ULL);
	auto roll = rng.uniform(100);
	W picked = W::Standard3;
	for (const auto &[value, weight] : table) {
		if (roll < weight) {
			picked = value;
			break;
		}
		roll -= weight;
	}
	auto era_rng = element_rng_salted(seed, 0x57A2C0DEA5C10008ULL);
	if (picked == W::Standard3 && era == ArchEra::HistoricOrnate &&
			era_rng.random_bool(.75))
		return W::ArchedTraditional;
	if (picked == W::Standard3 && era == ArchEra::TraditionalPreWar &&
			era_rng.random_bool(.25))
		return W::ArchedTraditional;
	if (picked == W::PairedNarrow && era == ArchEra::PostWarPanel)
		return W::WideHorizontal;
	if (picked == W::Standard3 && era == ArchEra::Contemporary &&
			era_rng.random_bool(.25))
		return W::WideHorizontal;
	return picked;
}

Block category_window_block(BuildingCategory category, ChaCha8Rng &rng)
{
	std::vector<Block> pool;
	switch (category) {
	case BuildingCategory::House:
	case BuildingCategory::Residential:
		pool = {GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
				BROWN_STAINED_GLASS};
		break;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		pool = {GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, CYAN_STAINED_GLASS};
		break;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		pool = {GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, TINTED_GLASS};
		break;
	case BuildingCategory::Religious:
		pool = {BLUE_STAINED_GLASS, CYAN_STAINED_GLASS, LIGHT_BLUE_STAINED_GLASS};
		break;
	default:
		pool = {GLASS, LIGHT_GRAY_STAINED_GLASS, CYAN_STAINED_GLASS};
		break;
	}
	return pool[rng.uniform(static_cast<std::uint32_t>(pool.size()))];
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
	if (block == RED_CONCRETE)
		return RED_TERRACOTTA;
	if (block == ORANGE_CONCRETE)
		return ORANGE_TERRACOTTA;
	if (block == YELLOW_CONCRETE)
		return YELLOW_TERRACOTTA;
	if (block == LIME_CONCRETE)
		return GREEN_CONCRETE;
	if (block == BLUE_CONCRETE)
		return BLUE_TERRACOTTA;
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
				return block_palette::roof_block_for_color(*rgb, rng);
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
	if (roof_shape == "mansard")
		return RoofType::Mansard;
	if (roof_shape == "gambrel")
		return RoofType::Gambrel;
	if (roof_shape == "half-hipped" || roof_shape == "side_half-hipped")
		return RoofType::HalfHipped;
	if (roof_shape == "hipped" || roof_shape == "hip" || roof_shape == "round" ||
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

int floor_cycle_for(const std::string &building_type, const tags_t &tags)
{
	static const std::unordered_set<std::string> residential = {"house", "detached",
			"semidetached_house", "terrace", "bungalow", "villa", "cabin", "residential",
			"apartments", "dormitory", "farm", "hut", "shed", "static_caravan"};
	if (tags.contains("building:part"))
		return 4;
	return building_type == "yes" || residential.contains(building_type) ? 3 : 4;
}

struct InferredHeight
{
	bool hall{false};
	int value{1};
};

int pick_weighted_value(
		ChaCha8Rng &rng, std::initializer_list<std::pair<int, std::uint32_t>> options)
{
	std::uint32_t total = 0;
	for (const auto &[value, weight] : options)
		total += weight;
	auto roll = rng.uniform(total);
	for (const auto &[value, weight] : options) {
		if (roll < weight)
			return value;
		roll -= weight;
	}
	return options.end()[-1].first;
}

InferredHeight infer_building_height(const std::string &type, const tags_t &tags,
		std::size_t footprint, double scale, std::uint64_t group_seed)
{
	auto rng = element_rng_salted(group_seed, 0x48E16F001EA50001ULL);
	const std::size_t area =
			tags.contains("building:part")
					? 400
					: (scale > 0 ? std::size_t(footprint / (scale * scale)) : footprint);
	auto pick = [&](bool hall,
						std::initializer_list<std::pair<int, std::uint32_t>> values) {
		return InferredHeight{hall, pick_weighted_value(rng, values)};
	};
	if (tags.get("man_made") == "tower")
		return pick(true, {{12, 40}, {16, 35}, {20, 25}});
	if (type == "bungalow" || type == "static_caravan")
		return pick(false, {{1, 100}});
	if (type == "house" || type == "detached" || type == "villa" || type == "farm")
		return pick(false, {{1, 25}, {2, 55}, {3, 20}});
	if (type == "semidetached_house")
		return pick(false, {{2, 70}, {3, 30}});
	if (type == "terrace")
		return pick(false, {{2, 50}, {3, 40}, {4, 10}});
	if (type == "apartments" || type == "residential" || type == "dormitory")
		return pick(false, {{3, 30}, {4, 30}, {5, 25}, {6, 15}});
	if (type == "hotel")
		return pick(false, {{3, 25}, {4, 30}, {5, 25}, {6, 10}, {8, 10}});
	if (type == "office" || type == "commercial") {
		if (area <= 299)
			return pick(false, {{2, 40}, {3, 60}});
		if (area <= 999)
			return pick(false, {{3, 40}, {4, 35}, {5, 25}});
		return pick(false, {{4, 40}, {5, 30}, {6, 20}, {8, 10}});
	}
	if (type == "retail" || type == "shop" || type == "supermarket")
		return pick(false, {{1, 60}, {2, 40}});
	if (type == "kiosk")
		return pick(false, {{1, 100}});
	if (type == "parking")
		return pick(false, {{2, 30}, {3, 40}, {4, 20}, {5, 10}});
	if (type == "school" || type == "kindergarten" || type == "college" ||
			type == "university")
		return pick(false, {{2, 50}, {3, 40}, {4, 10}});
	if (type == "hospital")
		return pick(false, {{4, 40}, {5, 30}, {6, 20}, {7, 10}});
	if (type == "cathedral")
		return pick(true, {{18, 50}, {21, 30}, {24, 20}});
	if (type == "church" || type == "chapel" || type == "mosque" || type == "synagogue" ||
			type == "temple" || type == "religious")
		return pick(true, {{10, 50}, {12, 30}, {14, 20}});
	if (type == "industrial" || type == "warehouse" || type == "hangar" ||
			type == "barn" || type == "stable")
		return pick(true, {{5, 20}, {7, 50}, {9, 30}});
	if (type == "garage" || type == "garages" || type == "carport" || type == "shed" ||
			type == "hut")
		return pick(true, {{3, 100}});
	if (type == "cabin")
		return pick(false, {{1, 80}, {2, 20}});
	if (area <= 39) {
		const bool hall = rng.random_bool();
		return {hall, hall ? 3 : 1};
	}
	if (area <= 149)
		return pick(false, {{1, 15}, {2, 60}, {3, 25}});
	if (area <= 599)
		return pick(false, {{2, 40}, {3, 40}, {4, 20}});
	const auto roll = rng.uniform(100);
	return roll < 40 ? InferredHeight{true, 7} : InferredHeight{false, roll < 75 ? 3 : 4};
}

std::optional<double> parse_meter_tag(const tags_t &tags, const char *key)
{
	auto it = tags.find(key);
	if (it == tags.end())
		return std::nullopt;
	auto value = it->second;
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
		value.pop_back();
	if (!value.empty() && value.back() == 'm')
		value.pop_back();
	try {
		return std::stod(value);
	} catch (...) {
		return std::nullopt;
	}
}

std::pair<int, bool> calculate_building_height(const ProcessedWay &element,
		const std::string &building_type, int min_level, double scale,
		std::optional<int> relation_levels, int floor_cycle, std::size_t footprint,
		std::uint64_t group_seed)
{
	constexpr int ground_floor_bonus = 2;
	int height = std::max(3, int(10.0 * scale));
	bool tall = false, has_source = false, explicit_height = false;
	if (auto it = element.tags.find("building:levels"); it != element.tags.end())
		try {
			const double levels = std::stod(it->second), lev = levels - min_level;
			if (lev >= 1) {
				height = std::max(
						3, int((lev * floor_cycle + (min_level ? 0 : 2)) * scale));
				has_source = true;
				tall = levels > 7;
			}
		} catch (...) {
		}
	if (const auto tagged = parse_meter_tag(element.tags, "height")) {
		explicit_height = has_source = true;
		double effective = *tagged;
		bool elevated = false;
		if (const auto min_height = parse_meter_tag(element.tags, "min_height")) {
			elevated = *min_height > 0;
			effective = std::max(1.0, effective - *min_height);
		} else if (min_level > 0) {
			elevated = true;
			effective = std::max(
					1.0, effective - (min_level * floor_cycle + ground_floor_bonus));
		}
		if (element.tags.get("roof:shape") != "flat")
			if (const auto roof_height = parse_meter_tag(element.tags, "roof:height"))
				effective = std::max(std::min(3.0, effective),
						effective - std::max(0.0, *roof_height));
		height = std::max(elevated ? 1 : 3, int(effective * scale));
		tall = *tagged > 28;
	}
	if (!explicit_height && relation_levels) {
		height = std::max(
				3, int(((std::max(1, *relation_levels - min_level) * floor_cycle) +
							   (min_level ? 0 : ground_floor_bonus)) *
						   scale));
		has_source = true;
		tall = *relation_levels > 7;
	}
	if (!has_source) {
		const auto inferred = infer_building_height(
				building_type, element.tags, footprint, scale, group_seed);
		if (inferred.hall)
			height = std::max(3, int(inferred.value * scale));
		else {
			const int levels = std::max(1, inferred.value - min_level);
			height = std::max(
					3, int((levels * floor_cycle + (min_level ? 0 : 2)) * scale));
			tall = inferred.value > 7;
		}
	}
	return {height, tall};
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
				roof_type == RoofType::HalfHipped || roof_type == RoofType::Gambrel ||
				roof_type == RoofType::Mansard || roof_type == RoofType::Pyramidal ||
				roof_type == RoofType::Cone || roof_type == RoofType::Onion) &&
			!cached_floor_area.empty()) {
		const int springing = start_y_offset + roof_thickness;
		for (const auto &node : element.nodes) {
			const int pillar_base =
					args.terrain ? editor.get_ground_level(node.x, node.z) : 0;
			for (int y = pillar_base + 1; y < springing; ++y)
				editor.set_block_absolute(
						COBBLESTONE_WALL, node.x, y + abs_terrain_offset, node.z);
		}
		generate_roof(editor, element, springing - 4, 3, roof_block, roof_block,
				roof_block, roof_type, cached_floor_area, abs_terrain_offset);
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

void generate_rooftop_systems(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &area, BuildingCategory category,
		BuildingCondition condition, bool sloped, int start_y, int building_height,
		int abs_offset, int floor_cycle, Block wall_block, std::uint64_t seed,
		bool covered_by_sibling)
{
	if (area.empty() || condition == BuildingCondition::Construction ||
			condition == BuildingCondition::Ruined)
		return;
	std::unordered_set<std::pair<int, int>, PairHash> footprint(area.begin(), area.end());
	auto interior = [&](int x, int z, bool diagonal = false) {
		for (int dz = -1; dz <= 1; ++dz)
			for (int dx = -1; dx <= 1; ++dx)
				if ((dx || dz) && (diagonal || dx == 0 || dz == 0) &&
						!footprint.contains({x + dx, z + dz}))
					return false;
		return true;
	};
	const int roof_y = start_y + building_height + abs_offset + 1;
	int min_x = INT_MAX, max_x = INT_MIN, min_z = INT_MAX, max_z = INT_MIN;
	for (const auto &[x, z] : area) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const int center_x = (min_x + max_x) / 2, center_z = (min_z + max_z) / 2;
	if (sloped) {
		const bool residential = category == BuildingCategory::House ||
								 category == BuildingCategory::Residential ||
								 category == BuildingCategory::Farm;
		if (!residential || area.size() < 30 || area.size() > 400 ||
				!element_rng_salted(seed, 0xC11A4E9000000001ULL).random_bool(.4))
			return;
		auto best = std::min_element(
				area.begin(), area.end(), [&](const auto &a, const auto &b) {
					const int ad = std::abs(a.first - center_x - 2) +
								   std::abs(a.second - center_z);
					const int bd = std::abs(b.first - center_x - 2) +
								   std::abs(b.second - center_z);
					return ad < bd;
				});
		const int rise = std::min(std::max(1, std::min(max_x - min_x, max_z - min_z) / 2),
				std::max(1, int(std::lround(building_height * .6))));
		for (int dy = 0; dy < 3; ++dy)
			editor.set_block_absolute(BRICK, best->first, roof_y + rise + dy,
					best->second, std::vector<Block>{AIR});
		return;
	}
	if (covered_by_sibling)
		return;

	const bool parapet = building_height > floor_cycle + 2 &&
						 (category == BuildingCategory::Commercial ||
								 category == BuildingCategory::Office ||
								 category == BuildingCategory::Hotel ||
								 category == BuildingCategory::School ||
								 category == BuildingCategory::Hospital ||
								 category == BuildingCategory::TallBuilding ||
								 category == BuildingCategory::GlassySkyscraper ||
								 category == BuildingCategory::ModernSkyscraper);
	if (parapet)
		for (const auto &[x, z] : area)
			if (!interior(x, z))
				editor.set_block_absolute(
						wall_block, x, roof_y + 1, z, std::vector<Block>{AIR});

	if (category == BuildingCategory::Hospital) {
		bool fits = true;
		for (int dz = -3; dz <= 3; ++dz)
			for (int dx = -3; dx <= 3; ++dx)
				fits &= footprint.contains({center_x + dx, center_z + dz});
		if (fits)
			for (int dz = -3; dz <= 3; ++dz)
				for (int dx = -3; dx <= 3; ++dx) {
					const bool h = std::abs(dx) == 2 || (dz == 0 && std::abs(dx) <= 2);
					editor.set_block_absolute(h ? WHITE_CONCRETE : YELLOW_CONCRETE,
							center_x + dx, roof_y + 1, center_z + dz);
				}
	}

	const bool terrace = element.tags.contains("building:part") && building_height >= 28;
	if (terrace) {
		for (const auto &[x, z] : area)
			if (!interior(x, z))
				editor.set_block_absolute(
						STONE_BRICKS, x, roof_y + 1, z, std::vector<Block>{AIR});
		std::vector<std::pair<int, int>> terrace_interior;
		for (const auto &[x, z] : area)
			if (interior(x, z))
				terrace_interior.emplace_back(x, z);
		for (const auto &[x, z] : terrace_interior) {
			const auto roll =
					(std::uint64_t(std::uint32_t(x)) * 0x9E3779B97F4A7C15ULL ^
							std::uint64_t(std::uint32_t(z)) * 0x517CC1B727220A95ULL ^
							seed) %
					100;
			if (roll < 3) {
				editor.set_block_absolute(IRON_BLOCK, x, roof_y + 1, z);
				editor.set_block_absolute(SMOOTH_STONE_SLAB, x, roof_y + 2, z);
			} else if (roll < 6) {
				editor.set_block_absolute(CAULDRON, x, roof_y + 1, z);
				editor.set_block_absolute(SPRUCE_LEAVES, x, roof_y + 2, z);
			} else if (roll < 9) {
				editor.set_block_absolute(OAK_FENCE, x, roof_y + 1, z);
				editor.set_block_absolute(OAK_SLAB, x, roof_y + 2, z);
			} else if (roll < 11)
				editor.set_block_absolute(OAK_STAIRS, x, roof_y + 1, z);
			else if (roll < 13)
				editor.set_block_absolute(LIGHTNING_ROD, x, roof_y + 1, z);
			else if (roll == 13)
				editor.set_block_absolute(CAULDRON, x, roof_y + 1, z);
			else if (roll == 14)
				editor.set_block_absolute(SEA_LANTERN, x, roof_y + 1, z);
		}
		if (!terrace_interior.empty()) {
			const auto best = std::min_element(terrace_interior.begin(),
					terrace_interior.end(), [&](const auto &a, const auto &b) {
						return std::abs(a.first - center_x) +
									   std::abs(a.second - center_z) <
							   std::abs(b.first - center_x) +
									   std::abs(b.second - center_z);
					});
			for (int dy = 0; dy < 6; ++dy)
				editor.set_block_absolute(
						IRON_BARS, best->first, roof_y + 1 + dy, best->second);
			editor.set_block_absolute(
					LIGHTNING_ROD, best->first, roof_y + 7, best->second);
		}
		return;
	}

	std::optional<std::pair<int, int>> water_tower;
	const bool water_tower_eligible = building_height >= 16 && area.size() >= 300 &&
									  !element.tags.contains("building:part") &&
									  category != BuildingCategory::GlassySkyscraper &&
									  category != BuildingCategory::ModernSkyscraper &&
									  category != BuildingCategory::Religious &&
									  category != BuildingCategory::Hospital;
	if (water_tower_eligible &&
			element_rng_salted(seed, 0x3A7E12F000000002ULL).random_bool(.18)) {
		std::vector<std::pair<int, int>> spots;
		for (const auto &[x, z] : area) {
			bool fits = true;
			for (int dz = -2; dz <= 2 && fits; ++dz)
				for (int dx = -2; dx <= 2; ++dx)
					if (!footprint.contains({x + dx, z + dz})) {
						fits = false;
						break;
					}
			if (fits)
				spots.emplace_back(x, z);
		}
		if (!spots.empty()) {
			auto tower_rng = element_rng_salted(seed, 0x3A7E12F000000002ULL);
			water_tower =
					spots[tower_rng.uniform(static_cast<std::uint32_t>(spots.size()))];
			const auto [cx, cz] = *water_tower;
			const int base = roof_y + 1;
			for (const auto [dx, dz] : {std::pair{-1, -1}, std::pair{1, -1},
						 std::pair{-1, 1}, std::pair{1, 1}})
				for (int h = 0; h < 2; ++h)
					editor.set_block_absolute(OAK_FENCE, cx + dx, base + h, cz + dz);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					for (int h = 2; h < 5; ++h)
						editor.set_block_absolute(
								SPRUCE_PLANKS, cx + dx, base + h, cz + dz);
					editor.set_block_absolute(
							dx == 0 && dz == 0 ? SPRUCE_PLANKS : OAK_SLAB, cx + dx,
							base + 5, cz + dz);
				}
			editor.set_block_absolute(OAK_SLAB, cx, base + 6, cz);
		}
	}

	const bool equipment = building_height >= 8 && category != BuildingCategory::House &&
						   category != BuildingCategory::Farm &&
						   category != BuildingCategory::Garage &&
						   category != BuildingCategory::Shed &&
						   category != BuildingCategory::Greenhouse &&
						   category != BuildingCategory::Religious;
	if (!equipment)
		return;
	for (const auto &[x, z] : area) {
		if (!interior(x, z, true) ||
				(water_tower && std::abs(x - water_tower->first) <= 3 &&
						std::abs(z - water_tower->second) <= 3) ||
				(category == BuildingCategory::Hospital && std::abs(x - center_x) <= 4 &&
						std::abs(z - center_z) <= 4))
			continue;
		const std::uint64_t hash =
				(std::uint64_t(std::uint32_t(x)) * 0x9E3779B97F4A7C15ULL) ^
				(std::uint64_t(std::uint32_t(z)) * 0x517CC1B727220A95ULL) ^ seed;
		const unsigned roll = hash % 1200;
		if (roll < 3) {
			editor.set_block_absolute(
					IRON_BLOCK, x, roof_y + 2, z, std::vector<Block>{AIR});
			editor.set_block_absolute(
					SMOOTH_STONE_SLAB, x, roof_y + 3, z, std::vector<Block>{AIR});
		} else if (roll < 6) {
			editor.set_block_absolute(
					DAYLIGHT_DETECTOR, x, roof_y + 2, z, std::vector<Block>{AIR});
		} else if (roll == 6) {
			editor.set_block_absolute(
					IRON_BARS, x, roof_y + 2, z, std::vector<Block>{AIR});
			editor.set_block_absolute(
					LIGHTNING_ROD, x, roof_y + 3, z, std::vector<Block>{AIR});
		} else if (roll < 9) {
			editor.set_block_absolute(BARREL, x, roof_y + 2, z, std::vector<Block>{AIR});
			editor.set_block_absolute(
					CAULDRON, x, roof_y + 3, z, std::vector<Block>{AIR});
		} else if (roll < 12) {
			editor.set_block_absolute(
					COBBLESTONE_WALL, x, roof_y + 2, z, std::vector<Block>{AIR});
		}
	}
}

struct PodiumTowerPlan
{
	int podium_height;
	int inset;
	int full_height;
};

struct InsetTier
{
	int inset;
	int height;
	bool with_ceilings;
};

using RoofDistanceGrid = std::unordered_map<std::pair<int, int>, int, PairHash>;

RoofDistanceGrid roof_edge_distances(const std::vector<std::pair<int, int>> &roof_area)
{
	std::unordered_set<std::pair<int, int>, PairHash> cells(
			roof_area.begin(), roof_area.end());
	RoofDistanceGrid distances;
	std::deque<std::pair<int, int>> queue;
	for (const auto &[x, z] : roof_area) {
		for (const auto [dx, dz] :
				{std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
			if (!cells.contains({x + dx, z + dz})) {
				distances[{x, z}] = 0;
				queue.emplace_back(x, z);
				break;
			}
		}
	}
	while (!queue.empty()) {
		const auto [x, z] = queue.front();
		queue.pop_front();
		const int next = distances.at({x, z}) + 1;
		for (const auto [dx, dz] :
				{std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
			const std::pair neighbor{x + dx, z + dz};
			if (cells.contains(neighbor) && !distances.contains(neighbor)) {
				distances[neighbor] = next;
				queue.push_back(neighbor);
			}
		}
	}
	return distances;
}

std::optional<PodiumTowerPlan> plan_podium_tower(const ProcessedWay &element,
		bool is_tall_building, int building_height, int floor_cycle,
		BuildingCondition condition, bool flat_roof, bool generate_roof,
		const std::vector<std::pair<int, int>> &floor_area, std::uint64_t group_seed)
{
	const std::size_t footprint_size = floor_area.size();
	if (!is_tall_building || building_height < 30 || footprint_size < 600 || !flat_roof ||
			!generate_roof || condition != BuildingCondition::Normal ||
			element.tags.contains("building:part"))
		return std::nullopt;
	auto rng = element_rng_salted(group_seed, 0x90D10A7000000001ULL);
	if (!rng.random_bool(.4))
		return std::nullopt;
	const int inset = footprint_size < 1200 ? 3 : 4 + int(rng.uniform(2));
	const auto distances = roof_edge_distances(floor_area);
	const std::size_t tower_cells = std::count_if(floor_area.begin(), floor_area.end(),
			[&](const auto &cell) { return distances.at(cell) >= inset; });
	if (tower_cells < 150 || tower_cells * 4 < footprint_size)
		return std::nullopt;
	const int podium_floors = 2 + int(rng.uniform(2));
	const int podium_height = podium_floors * floor_cycle + 2;
	if (building_height - podium_height < 2 * floor_cycle)
		return std::nullopt;
	return PodiumTowerPlan{podium_height, inset, building_height};
}

std::optional<int> generate_inset_tiers(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &roof_area,
		const RoofDistanceGrid &distances, int base_y, int abs_terrain_offset,
		const std::vector<InsetTier> &tiers, int floor_cycle, Block wall_block,
		Block window_block, Block floor_block, bool has_windows)
{
	int current_base = base_y;
	bool placed = false;
	for (const auto &tier : tiers) {
		std::vector<std::pair<int, int>> tier_cells;
		for (const auto &cell : roof_area) {
			if (distances.at(cell) >= tier.inset)
				tier_cells.push_back(cell);
		}
		if (tier_cells.size() < 30)
			break;
		placed = true;
		for (const auto &[x, z] : tier_cells) {
			const std::pair<int, int> offsets[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
			const bool wall = std::any_of(
					std::begin(offsets), std::end(offsets), [&](const auto &offset) {
						const auto it =
								distances.find({x + offset.first, z + offset.second});
						return it == distances.end() || it->second < tier.inset;
					});
			for (int h = 0; h < tier.height; ++h) {
				const int y = current_base + h;
				const int row = ((y - base_y) % floor_cycle + floor_cycle) % floor_cycle;
				if (wall) {
					const bool window = has_windows && row >= 2 &&
										row < floor_cycle - 1 && ((x + z) % 3 != 0);
					editor.set_block_absolute(window ? window_block : wall_block, x,
							y + abs_terrain_offset, z);
				} else if (tier.with_ceilings && row == 0) {
					editor.set_block_absolute(
							(x % 3 == 0 && z % 3 == 0) ? GLOWSTONE : floor_block, x,
							y + abs_terrain_offset, z);
				}
			}
			editor.set_block_absolute(
					floor_block, x, current_base + tier.height + abs_terrain_offset, z);
		}
		current_base += tier.height;
	}
	return placed ? std::optional<int>(current_base) : std::nullopt;
}

bool generate_setback_crown(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &roof_area, BuildingCategory category,
		BuildingCondition condition, bool is_tall_building, int start_y,
		int building_height, int abs_terrain_offset, int floor_cycle, Block wall_block,
		Block window_block, Block floor_block, bool has_windows, std::uint64_t seed)
{
	if (!is_tall_building || condition != BuildingCondition::Normal ||
			roof_area.size() < 200 ||
			!element_rng_salted(seed, 0x5E7BAC4C00000001ULL).random_bool(.45))
		return false;
	const auto distances = roof_edge_distances(roof_area);
	const int tier_count = category == BuildingCategory::MasonrySkyscraper ? 3 : 2;
	std::vector<InsetTier> tiers;
	for (int tier = 0; tier < tier_count; ++tier)
		tiers.push_back({3 + tier * 3, floor_cycle, false});
	const auto top = generate_inset_tiers(editor, roof_area, distances,
			start_y + building_height + 1, abs_terrain_offset, tiers, floor_cycle,
			wall_block, window_block, floor_block, has_windows);
	if (!top)
		return false;
	const auto deepest = std::max_element(
			roof_area.begin(), roof_area.end(), [&](const auto &a, const auto &b) {
				return distances.at(a) < distances.at(b);
			});
	if (deepest != roof_area.end() && distances.at(*deepest) >= 6) {
		const int mast_y = *top + abs_terrain_offset + 1;
		for (int h = 0; h < 3; ++h)
			editor.set_block_absolute(
					IRON_BARS, deepest->first, mast_y + h, deepest->second);
		editor.set_block_absolute(
				LIGHTNING_ROD, deepest->first, mast_y + 3, deepest->second);
	}
	return true;
}

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
		std::optional<std::uint64_t> style_seed, const CoordinateBitmap *road_mask,
		const CoordinateBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members)
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
	const std::string early_building_type =
			!element.tags.get("building").empty()
					? element.tags.get("building")
					: (!element.tags.get("building:part").empty()
									  ? element.tags.get("building:part")
									  : "yes");
	int min_level_offset = multiply_scale(
			min_level * floor_cycle_for(early_building_type, element.tags) + 2,
			scale_factor);
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
	building_facade::PointSet current_footprint(
			cached_floor_area.begin(), cached_floor_area.end());
	building_facade::FacadePlan facade_plan = building_facade::FacadePlan::empty();
	building_facade::PointSet sibling_cells;
	static const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
			empty_groups;
	const auto &groups = group_members ? *group_members : empty_groups;
	if (style_seed) {
		if (const auto group = groups.find(*style_seed); group != groups.end()) {
			for (const auto sibling_id : group->second) {
				if (sibling_id == element.id)
					continue;
				if (const auto *fill = flood_fill_cache.get_cached(sibling_id))
					sibling_cells.insert(fill->begin(), fill->end());
			}
		}
	}
	if (road_mask && building_footprints &&
			cached_footprint_size >= building_facade::MIN_FACADE_FOOTPRINT) {
		building_facade::PointSet own_cells = current_footprint;
		own_cells.insert(sibling_cells.begin(), sibling_cells.end());
		const building_facade::BuildingContext context{flood_fill_cache,
				building_passages, *road_mask, *building_footprints, groups};
		facade_plan = building_facade::compute_facade_plan(
				element, context, args.scale, own_cells);
	}

	int start_y_offset = 0;
	if (args.terrain) {
		int max_ground_level = args.ground_level;
		for (const auto &node : element.nodes)
			max_ground_level = std::max(
					max_ground_level, editor->get_ground_level(node.x, node.z));

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
	const BuildingCondition condition = building_condition_from_tags(element.tags);
	const bool explicit_wall = element.tags.contains("building:material") ||
							   element.tags.contains("building:facade:material") ||
							   element.tags.contains("facade:material") ||
							   element.tags.contains("building:colour") ||
							   element.tags.contains("colour") ||
							   element.tags.get("historic") == "castle";
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
	if (condition == BuildingCondition::Construction)
		wall_block = SCAFFOLDING;
	const bool has_windows = condition != BuildingCondition::Construction &&
							 condition != BuildingCondition::Ruined;

	std::unordered_set<std::pair<int, int>, PairHash> processed_points;
	int floor_cycle = floor_cycle_for(building_type, element.tags);
	const int grammar_anchor = min_level_offset == 0 ? 2 : 0;
	auto [building_height, is_tall_building] =
			calculate_building_height(element, building_type, min_level, scale_factor,
					relation_levels, floor_cycle, cached_footprint_size, visual_seed);
	if (building_type == "yes" && !element.tags.contains("building:part") &&
			is_tall_building && floor_cycle != 4) {
		const int old_cycle = floor_cycle;
		floor_cycle = 4;
		if (min_level > 0 && !element.tags.contains("min_height")) {
			const int lift_delta =
					multiply_scale(min_level * (floor_cycle - old_cycle), scale_factor);
			min_level_offset += lift_delta;
			start_y_offset += lift_delta;
		}
		std::tie(building_height, is_tall_building) =
				calculate_building_height(element, building_type, min_level, scale_factor,
						relation_levels, floor_cycle, cached_footprint_size, visual_seed);
	}
	if (condition == BuildingCondition::Construction)
		building_height = std::max(3, building_height / 2);
	else if (condition == BuildingCondition::Ruined)
		building_height = std::max(3, int(building_height * .6));
	const BuildingCategory category = building_category(
			element, is_tall_building, building_height, scale_factor, visual_seed);
	const ArchEra era = building_arch_era(element.tags);
	const DetailTier detail =
			compute_detail_tier(element, category, cached_footprint_size, building_height,
					facade_plan.front_segment.has_value());
	const WallDepthStyle wall_depth_style = wall_depth_style_for(
			category, era, detail, cached_footprint_size, visual_seed);
	const auto climate = editor->get_ground() ? editor->get_ground()->climate()
											  : biome::Climate::Temperate;
	if (!explicit_wall && condition != BuildingCondition::Construction)
		wall_block = category_wall_block(category, era, climate, rng);
	const auto window_frame =
			pick_window_frame(category, era, detail, wall_block, visual_seed);
	window_block = category_window_block(category, rng);
	const WindowArchetype window_archetype =
			pick_window_archetype(category, era, visual_seed);
	const int window_phase =
			element.tags.contains("building:part")
					? 0
					: int(element_rng_salted(visual_seed, 0x77D0A3E19B1C5544ULL)
									  .uniform(6));
	const bool horizontal_windows = category == BuildingCategory::ModernSkyscraper;
	const bool storefront =
			(category == BuildingCategory::Commercial ||
					category == BuildingCategory::Hotel) &&
			has_windows && condition == BuildingCondition::Normal &&
			min_level_offset == 0 && facade_plan.front_segment.has_value() &&
			element_rng_salted(visual_seed, 0x5709EF9000000002ULL).random_bool(.6);
	const Block awning_options[] = {
			OAK_TRAPDOOR, SPRUCE_TRAPDOOR, DARK_OAK_TRAPDOOR, BIRCH_TRAPDOOR};
	const Block awning_block =
			awning_options[element_rng_salted(visual_seed, 0x0A3B11B60000000BULL)
								   .uniform(4)];
	const auto roof_shape = element.tags.get("roof:shape");
	const bool part_has_explicit_top =
			element.tags.contains("building:part") &&
			(element.tags.contains("height") || element.tags.contains("building:levels"));
	const bool sloped_roof =
			args.roof && roof_shape != "flat" &&
			(!roof_shape.empty() ||
					(!part_has_explicit_top &&
							(category == BuildingCategory::House ||
									category == BuildingCategory::Residential)));
	const auto podium_tower =
			plan_podium_tower(element, is_tall_building, building_height, floor_cycle,
					condition, !sloped_roof, args.roof, cached_floor_area, visual_seed);
	if (podium_tower)
		building_height = podium_tower->podium_height;
	const bool top_treatment =
			has_windows && building_height >= 4 * floor_cycle && !horizontal_windows &&
			category != BuildingCategory::GlassySkyscraper &&
			category != BuildingCategory::GlassCornerSkyscraper &&
			category != BuildingCategory::GridSkyscraper &&
			category != BuildingCategory::Tower &&
			element_rng_salted(visual_seed, 0xF10A401E00000001ULL).random_bool(.45);
	const bool attic_style =
			has_windows && sloped_roof && building_height >= 3 * floor_cycle &&
			(category == BuildingCategory::House ||
					category == BuildingCategory::Residential ||
					category == BuildingCategory::Historic) &&
			element_rng_salted(visual_seed, 0xF10A401E00000002ULL).random_bool(.55);
	const bool piano_nobile =
			has_windows && building_height >= 3 * floor_cycle &&
			((category == BuildingCategory::Historic &&
					 element_rng_salted(visual_seed, 0xF10A401E00000003ULL)
							 .random_bool(.5)) ||
					(category == BuildingCategory::Hotel &&
							element_rng_salted(visual_seed, 0xF10A401E00000003ULL)
									.random_bool(.2)));
	const BalconyBand balcony_band = pick_balcony_band(category, building_height,
			floor_cycle, facade_plan.has_any_street, visual_seed);
	const Block base_course_block = base_course_for_wall(wall_block);
	const bool has_base_course =
			min_level_offset == 0 && has_windows &&
			condition == BuildingCondition::Normal &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage &&
			category != BuildingCategory::GlassySkyscraper &&
			base_course_block != wall_block &&
			element_rng_salted(visual_seed, 0xBA5EC0A25E110001ULL).random_bool(.7);
	const int base_course_rows = building_height >= 3 * floor_cycle ? 2 : 1;
	const bool rustication =
			min_level_offset == 0 && condition == BuildingCondition::Normal &&
			detail >= DetailTier::Standard &&
			(category == BuildingCategory::House ||
					category == BuildingCategory::Residential ||
					category == BuildingCategory::Commercial ||
					category == BuildingCategory::Hotel ||
					category == BuildingCategory::Historic) &&
			(era == ArchEra::HistoricOrnate ||
					(era == ArchEra::TraditionalPreWar &&
							element_rng_salted(visual_seed, 0x0BA5E5A00000000AULL)
									.random_bool(.7)));

	bool use_vertical_windows = rng.random_bool(0.7);
	const double roof_line_chance = detail == DetailTier::Minimal	 ? .08
									: detail == DetailTier::Standard ? .25
																	 : .45;
	bool use_accent_roof_line = rng.random_bool(roof_line_chance);

	Block accent_blocks_arr[] = {
			//POLISHED_ANDESITE,
			SMOOTH_STONE, STONE_BRICKS,
			//MUD_BRICKS,
			//ANDESITE,
			//CHISELED_STONE_BRICKS
	};
	Block accent_block = accent_blocks_arr[rng.uniform(static_cast<std::uint32_t>(
			sizeof(accent_blocks_arr) / sizeof(accent_blocks_arr[0])))];

	bool has_multiple_floors = building_height > floor_cycle + 2;
	const double accent_chance = detail == DetailTier::Minimal	  ? .05
								 : detail == DetailTier::Standard ? .2
																  : .4;
	bool use_accent_lines = has_multiple_floors && rng.random_bool(accent_chance);
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
	for (std::size_t node_index = 0; node_index < element.nodes.size(); ++node_index) {
		const auto &node = element.nodes[node_index];
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
				const bool party_wall = facade_plan.is_party(bx, bz);
				std::pair<int, int> outward{0, 0};
				if (node_index > 0 && node_index - 1 < facade_plan.segments.size())
					if (const auto &segment = facade_plan.segments[node_index - 1])
						outward = segment->normal;
				const bool sibling_clear =
						!element.tags.contains("building:part") ||
						outward == std::pair<int, int>{0, 0} ||
						(!sibling_cells.contains(
								 {bx + outward.first, bz + outward.second}) &&
								!sibling_cells.contains({bx + 2 * outward.first,
										bz + 2 * outward.second}));
				auto foreign_building_at = [&](int px, int pz) {
					return building_footprints && building_footprints->contains(px, pz) &&
						   !current_footprint.contains({px, pz});
				};
				const bool depth_clear =
						sibling_clear &&
						(outward == std::pair<int, int>{0, 0} ||
								(!foreign_building_at(
										 bx + outward.first, bz + outward.second) &&
										!foreign_building_at(bx + 2 * outward.first,
												bz + 2 * outward.second)));

				if (args.terrain && min_level == 0 && !is_passage) {
					const int local_ground_level = editor->get_ground_level(bx, bz);
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
					const bool above_floor = h > start_y_offset + 1;
					const int floor_row =
							((h - start_y_offset - grammar_anchor) % floor_cycle +
									floor_cycle) %
							floor_cycle;
					const int window_col = ((bx + bz + window_phase) % 6 + 6) % 6;
					const int ground_top =
							start_y_offset + grammar_anchor - 1 + floor_cycle;
					const FloorRole role =
							h <= ground_top ? FloorRole::Ground
							: h > start_y_offset + building_height - floor_cycle
									? FloorRole::Top
									: FloorRole::Body;
					Block chosen = wall_block;
					if (!party_wall && has_windows) {
						if (horizontal_windows)
							chosen = !above_floor	  ? wall_block
									 : floor_row == 0 ? accent_block
													  : window_block;
						else if (category == BuildingCategory::Tower) {
							const bool slit = above_floor &&
											  (floor_row == 1 || floor_row == 2) &&
											  ((bx + bz + window_phase) % 4 + 4) % 4 == 1;
							chosen = slit ? window_block : wall_block;
						} else if (category == BuildingCategory::GridSkyscraper) {
							const bool mullion =
									!above_floor || floor_row == 0 ||
									((bx + bz + window_phase) % 5 + 5) % 5 == 0;
							chosen = mullion ? wall_block : window_block;
						} else if (is_tall_building && use_vertical_windows) {
							chosen = above_floor && ((bx + bz + window_phase) & 1) == 0
											 ? window_block
											 : wall_block;
						} else if (storefront && facade_plan.is_street(bx, bz) &&
								   above_floor && h <= ground_top && floor_row != 0 &&
								   window_col < 4) {
							chosen = GLASS;
						} else if (attic_style && role == FloorRole::Top) {
							chosen = above_floor && window_col == 1 && floor_row == 2
											 ? window_block
											 : wall_block;
						} else if (top_treatment && floor_row == 0 &&
								   h == start_y_offset + building_height - floor_cycle) {
							chosen = accent_block;
						} else {
							bool window =
									above_floor && floor_row != 0 &&
									(category == BuildingCategory::MasonrySkyscraper
													? window_col < 2
											: category == BuildingCategory::
																	ContemporarySkyscraper
													? window_col < 4
													: archetype_allows_window(
															  window_archetype,
															  window_col, floor_row,
															  floor_cycle));
							if (top_treatment && role == FloorRole::Top &&
									((window_archetype == WindowArchetype::Standard3 ||
											 window_archetype ==
													 WindowArchetype::
															 ArchedTraditional) &&
											window_col == 2))
								window = false;
							if (piano_nobile && role == FloorRole::Body &&
									h <= ground_top + floor_cycle && above_floor &&
									floor_row != 0 && window_col < 3)
								window = true;
							const bool accent =
									above_floor && floor_row == 0 &&
									(use_accent_lines ||
											(use_vertical_accent && window_col < 3));
							chosen = window	  ? window_block
									 : accent ? accent_block
											  : wall_block;
						}
					}
					if (has_base_course && h <= start_y_offset + base_course_rows)
						chosen = base_course_block;
					else if (rustication && role == FloorRole::Ground &&
							 (h - start_y_offset) % 2 == 0 && chosen != window_block &&
							 chosen != GLASS)
						chosen = base_course_block;
					chosen = apply_condition_variation(chosen, bx, h, bz, wall_block,
							window_block, has_windows, condition, category, era,
							visual_seed);
					editor->set_block_absolute(chosen, bx, h + abs_terrain_offset, bz);
					if (window_frame && outward != std::pair<int, int>{0, 0} &&
							!is_passage && !party_wall && depth_clear &&
							!facade_plan.is_door(bx, bz) &&
							condition == BuildingCondition::Normal &&
							!horizontal_windows && category != BuildingCategory::Tower) {
						const bool head_band =
								floor_row == 0 && window_col < 3 && above_floor;
						const bool flank = (window_col == 0 || window_col == 2) &&
										   floor_row > 0 && floor_row < floor_cycle - 1;
						if (head_band || flank)
							editor->set_block_absolute(
									window_frame_material(*window_frame),
									bx + outward.first, h + abs_terrain_offset,
									bz + outward.second, std::vector<Block>{AIR});
					}
					if (outward != std::pair<int, int>{0, 0} && !is_passage &&
							!party_wall && depth_clear &&
							condition == BuildingCondition::Normal) {
						const bool corner = (bx == prev.first && bz == prev.second) ||
											(bx == x && bz == z);
						bool relief = false;
						switch (wall_depth_style) {
						case WallDepthStyle::SubtlePilasters:
							relief = window_col == 3;
							break;
						case WallDepthStyle::ModernPillars:
							relief = window_col == 0 || window_col == 5 ||
									 (above_floor && floor_row == 0);
							break;
						case WallDepthStyle::InstitutionalBands:
							relief = window_col == 0 || (above_floor && floor_row == 0);
							break;
						case WallDepthStyle::IndustrialBeams:
							relief = corner;
							break;
						case WallDepthStyle::HistoricOrnate:
							relief = window_col == 3 ||
									 h == start_y_offset + building_height;
							break;
						case WallDepthStyle::ReligiousButtress:
							relief = window_col == 3 &&
									 h <= start_y_offset + building_height * 3 / 5;
							break;
						case WallDepthStyle::SkyscraperFins:
							relief = window_col == 3 || (above_floor && floor_row == 0);
							break;
						case WallDepthStyle::GlassCurtain:
							relief = corner;
							break;
						case WallDepthStyle::None:
							break;
						}
						if (relief) {
							const int ox = bx + outward.first, oz = bz + outward.second;
							editor->set_block_absolute(accent_block, ox,
									h + abs_terrain_offset, oz, std::vector<Block>{AIR});
							if (wall_depth_style == WallDepthStyle::ReligiousButtress &&
									h <= start_y_offset + 2)
								editor->set_block_absolute(accent_block,
										ox + outward.first, h + abs_terrain_offset,
										oz + outward.second, std::vector<Block>{AIR});
						}
					}
					if (outward != std::pair<int, int>{0, 0} && !is_passage &&
							!party_wall && depth_clear &&
							condition == BuildingCondition::Normal &&
							facade_plan.is_street(bx, bz)) {
						const int ox = bx + outward.first, oz = bz + outward.second;
						if (storefront && h == ground_top && window_col < 4 &&
								!facade_plan.is_door(bx, bz))
							editor->set_block_absolute(awning_block, ox,
									h + abs_terrain_offset, oz, std::vector<Block>{AIR});
						if (floor_row == 0 && window_col == 1 &&
								h >= start_y_offset + grammar_anchor + floor_cycle &&
								h <= start_y_offset + building_height - floor_cycle) {
							const auto hash = (std::uint64_t(std::uint32_t(bx)) *
													  0x9E3779B97F4A7C15ULL) ^
											  (std::uint64_t(std::uint32_t(bz)) *
													  0x517CC1B727220A95ULL) ^
											  visual_seed ^ std::uint64_t(h);
							const bool balcony =
									balcony_band == BalconyBand::EveryBay ||
									(balcony_band == BalconyBand::Alternating &&
											(((bx + bz + window_phase) / 6) & 1) == 0) ||
									(balcony_band == BalconyBand::Scattered &&
											hash % 100 >= 15 && hash % 100 < 23);
							if (balcony) {
								editor->set_block_absolute(OAK_SLAB, ox,
										h + abs_terrain_offset, oz,
										std::vector<Block>{AIR});
								editor->set_block_absolute(IRON_BARS, ox,
										h + abs_terrain_offset + 1, oz,
										std::vector<Block>{AIR});
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

	if (min_level_offset == 0 && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage) {
		bool mapped_entrance = false;
		std::unordered_set<std::pair<int, int>, PairHash> rendered_entrances;
		for (const auto &node : element.nodes)
			if ((!node.tags.get("entrance").empty() &&
						node.tags.get("entrance") != "no") ||
					(!node.tags.get("door").empty() && node.tags.get("door") != "no")) {
				if (const auto level = parse_i32_tag(node.tags, "level");
						level && *level > 0)
					continue;
				if (!rendered_entrances.emplace(node.x, node.z).second)
					continue;
				mapped_entrance = true;
				facade_plan.mark_door_column(node.x, node.z);
				editor->set_block_absolute(OAK_DOOR, node.x,
						start_y_offset + abs_terrain_offset + 1, node.z);
				editor->set_block_absolute(OAK_DOOR_UPPER, node.x,
						start_y_offset + abs_terrain_offset + 2, node.z);
			}
		if (!mapped_entrance) {
			std::optional<std::size_t> entrance_segment = facade_plan.front_segment;
			if (!entrance_segment)
				for (std::size_t i = 0; i < facade_plan.segments.size(); ++i)
					if (facade_plan.segments[i] &&
							(!entrance_segment ||
									facade_plan.segments[i]->len >
											facade_plan.segments[*entrance_segment]->len))
						entrance_segment = i;
			if (entrance_segment && *entrance_segment + 1 < element.nodes.size() &&
					facade_plan.segments[*entrance_segment]) {
				const auto &a = element.nodes[*entrance_segment];
				const auto &b = element.nodes[*entrance_segment + 1];
				const auto &segment = *facade_plan.segments[*entrance_segment];
				auto points = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
				if (points.size() >= 5) {
					auto position_rng =
							element_rng_salted(visual_seed, 0xD00E57E900000011ULL);
					const int jitter = int(position_rng.uniform(3)) - 1;
					const std::size_t center = std::clamp<std::size_t>(
							points.size() / 2 + jitter, 2, points.size() - 3);
					std::optional<std::pair<int, int>> door;
					for (const int offset : {0, 1, -1, 2, -2}) {
						const auto index = std::clamp<int>(
								int(center) + offset, 2, int(points.size()) - 3);
						const int dx = std::get<0>(points[index]),
								  dz = std::get<2>(points[index]);
						if (!passage_at(effective_passages, dx, dz) &&
								!facade_plan.is_party(dx, dz)) {
							door = {dx, dz};
							break;
						}
					}
					if (door) {
						const bool formal = category == BuildingCategory::Commercial ||
											category == BuildingCategory::Office ||
											category == BuildingCategory::Hotel ||
											category == BuildingCategory::School ||
											category == BuildingCategory::Hospital ||
											category == BuildingCategory::Historic ||
											category == BuildingCategory::Religious;
						const Block lower = formal ? DARK_OAK_DOOR_LOWER : OAK_DOOR;
						const Block upper = formal ? DARK_OAK_DOOR_UPPER : OAK_DOOR_UPPER;
						facade_plan.mark_door_column(door->first, door->second);
						editor->set_block_absolute(lower, door->first,
								start_y_offset + abs_terrain_offset + 1, door->second);
						editor->set_block_absolute(upper, door->first,
								start_y_offset + abs_terrain_offset + 2, door->second);
						if ((formal || storefront) &&
								element_rng_salted(visual_seed, 0xD00E57E900000013ULL)
										.random_bool(.6))
							editor->set_block_absolute(OAK_SLAB,
									door->first + segment.normal.first,
									start_y_offset + abs_terrain_offset + 3,
									door->second + segment.normal.second,
									std::vector<Block>{AIR});
					}
				}
			}
		}
	}

	if (editor->signage_enabled() && !element.tags.contains("building:part") &&
			facade_plan.front_segment &&
			*facade_plan.front_segment + 1 < element.nodes.size()) {
		const auto index = *facade_plan.front_segment;
		const auto &a = element.nodes[index], &b = element.nodes[index + 1];
		const auto &segment = *facade_plan.segments[index];
		const int anchor_x = (a.x + b.x) / 2, anchor_z = (a.z + b.z) / 2;
		const auto facing = WorldEditor::facing_for_normal(
				segment.normal.first, segment.normal.second);
		if (args.signage == SignageLevel::Full &&
				decals::pictograms::business_kind(element.tags)) {
			const auto name = element.tags.get("name");
			if (!name.empty() && decals::font::supports(name)) {
				const std::uint8_t cols = name.size() <= 26	  ? 2
										  : name.size() <= 44 ? 3
															  : 4;
				const decals::DecalKey key = decals::DecalKey::text(
						{decals::TextStyleKind::Fascia}, name, cols);
				const auto [left_x, left_z] =
						WorldEditor::panel_left_anchor(anchor_x, anchor_z, facing, cols);
				editor->place_decal_panel(left_x, start_y_offset + abs_terrain_offset + 3,
						left_z, facing, key, false, true);
			}
		}
		const auto number = element.tags.get("addr:housenumber");
		if (args.signage == SignageLevel::Full && !number.empty() && number.size() <= 8 &&
				decals::font::supports(number)) {
			const decals::DecalKey key = decals::DecalKey::text(
					{decals::TextStyleKind::HouseNumber}, number, 1);
			editor->place_decal(anchor_x, start_y_offset + abs_terrain_offset + 2,
					anchor_z, facing, key);
		}
	}

	if (std::get<2>(corner_addup) != 0) {
		const std::vector<std::pair<int, int>> &floor_area = cached_floor_area;

		std::vector<int> floor_levels;
		floor_levels.push_back(start_y_offset);
		if (building_height > floor_cycle + 2) {
			int num_upper_floors = std::max(1, building_height / floor_cycle);
			for (int floor = 1; floor < num_upper_floors; ++floor) {
				floor_levels.push_back(
						start_y_offset + grammar_anchor + (floor * floor_cycle));
			}
		}

		for (const auto &p : floor_area) {
			int x = p.first;
			int z = p.second;
			if (processed_points.insert(p).second) {
				const bool is_passage = passage_at(effective_passages, x, z);
				if (!is_passage) {
					editor->set_block_absolute(
							floor_block, x, start_y_offset + abs_terrain_offset, z);
				}

				if (building_height > 4) {
					const int passage_ceiling =
							start_y_offset +
							std::min(BUILDING_PASSAGE_HEIGHT, building_height);
					for (int h = start_y_offset + grammar_anchor + floor_cycle;
							h < start_y_offset + building_height; h += floor_cycle) {
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

			const bool is_abandoned_building =
					condition == BuildingCondition::Abandoned ||
					condition == BuildingCondition::Ruined;

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

	bool generated_sloped_roof = false;
	if (args.roof && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined) {
		auto it_shape = element.tags.find("roof:shape");
		if (it_shape != element.tags.end()) {
			RoofType roof_type = parse_roof_type(it_shape->second);
			generated_sloped_roof = roof_type != RoofType::Flat;

			auto roof_rng = element_rng_salted(visual_seed, 0xF00FC010BA5EF00DULL);
			const Block roof_floor_block =
					roof_block_from_tags(element, roof_rng).value_or(floor_block);

			generate_roof(*editor, element, start_y_offset, building_height,
					roof_floor_block, wall_block, accent_block, roof_type,
					cached_floor_area, abs_terrain_offset);
		} else if (!part_has_explicit_top) {
			std::string btype = "yes";
			auto it = element.tags.find("building");
			if (it != element.tags.end())
				btype = it->second;

			if (btype == "apartments" || btype == "residential" || btype == "house" ||
					btype == "yes") {
				std::size_t footprint_size = cached_footprint_size;
				const std::size_t max_footprint_for_gabled = 800;
				if (footprint_size <= max_footprint_for_gabled && rng.random_bool(0.9)) {
					generated_sloped_roof = true;
					generate_roof(*editor, element, start_y_offset, building_height,
							floor_block, wall_block, accent_block, RoofType::Gabled,
							cached_floor_area, abs_terrain_offset);
				}
			}
		}
	} else {
		// flat roof default - already applied
	}
	if (podium_tower) {
		const auto distances = roof_edge_distances(cached_floor_area);
		const std::vector<InsetTier> tower{{podium_tower->inset,
				podium_tower->full_height - podium_tower->podium_height, true}};
		generate_inset_tiers(*editor, cached_floor_area, distances,
				start_y_offset + building_height + 1, abs_terrain_offset, tower,
				floor_cycle, wall_block, window_block, floor_block, has_windows);
	}
	const bool has_crown =
			args.roof && !podium_tower && !generated_sloped_roof &&
			!element.tags.contains("building:part") &&
			generate_setback_crown(*editor, cached_floor_area, category, condition,
					is_tall_building, start_y_offset, building_height, abs_terrain_offset,
					floor_cycle, wall_block, window_block, floor_block, has_windows,
					visual_seed);
	const bool covered_by_sibling =
			element.tags.contains("building:part") && !sibling_cells.empty() &&
			std::all_of(cached_floor_area.begin(), cached_floor_area.end(),
					[&](const auto &cell) { return sibling_cells.contains(cell); });
	if (args.roof && !podium_tower && !has_crown)
		generate_rooftop_systems(*editor, element, cached_floor_area, category, condition,
				generated_sloped_roof, start_y_offset, building_height,
				abs_terrain_offset, floor_cycle, wall_block, visual_seed,
				covered_by_sibling);
}


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
	const auto roof_height_tag = parse_meter_tag(element.tags, "roof:height");
	const std::optional<int> tagged_roof_height =
			roof_height_tag
					? std::optional<int>(std::max(1, int(std::lround(*roof_height_tag))))
					: std::nullopt;

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
		if (tagged_roof_height)
			roof_height_boost = *tagged_roof_height;
		int32_t roof_peak_height = base_height + roof_height_boost;

		bool is_wider_than_long = width > length;
		int32_t max_distance = is_wider_than_long ? (length >> 1) : (width >> 1);

		Block roof_block = tagged_roof_block.value_or(
				roof_friendly_block(rng.random_bool() ? accent_block : wall_block));

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

		// Continue the sloped roof one block past each gable face. This gives
		// gabled buildings the same trimmed end profile as the Rust generator.
		std::unordered_set<std::pair<int32_t, int32_t>, PairHash> footprint(
				floor_area.begin(), floor_area.end());
		for (const auto &[cell, roof_height] : roof_heights) {
			const auto [rx, rz] = cell;
			for (const int sign : {-1, 1}) {
				const bool at_end = is_wider_than_long
											? (sign < 0 ? rx == min_x : rx == max_x)
											: (sign < 0 ? rz == min_z : rz == max_z);
				if (!at_end)
					continue;
				const int tx = rx + (is_wider_than_long ? sign : 0);
				const int tz = rz + (is_wider_than_long ? 0 : sign);
				if (footprint.contains({tx, tz}))
					continue;
				const StairFacing facing =
						is_wider_than_long
								? (rz < center_z ? StairFacing::South
												 : StairFacing::North)
								: (rx < center_x ? StairFacing::East : StairFacing::West);
				editor.set_block_with_properties_absolute(
						create_stair_with_properties(
								stair_block_material, facing, StairShape::Straight),
						tx, roof_height + abs_terrain_offset, tz, nullptr, nullptr);
			}
		}

		return;
	}

	if (roof_type == RoofType::Hipped || roof_type == RoofType::HalfHipped ||
			roof_type == RoofType::Gambrel || roof_type == RoofType::Mansard) {
		int32_t width = max_x - min_x;
		int32_t length = max_z - min_z;

		bool is_rectangular = ((static_cast<double>(width) / std::max(1, length) > 1.3) ||
							   (static_cast<double>(length) / std::max(1, width) > 1.3));
		bool long_axis_is_x = width > length;

		int32_t roof_peak_height =
				base_height + (tagged_roof_height
											  ? *tagged_roof_height
											  : ((std::max(width, length) > 20) ? 7 : 5));

		Block roof_block = tagged_roof_block.value_or(
				roof_friendly_block(rng.random_bool() ? accent_block : wall_block));

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

				const int edge_distance =
						std::max(0, max_distance_from_ridge - distance_to_ridge);
				const int max_rise = roof_peak_height - base_height;
				int rise = std::min(edge_distance, max_rise);
				if (roof_type == RoofType::Gambrel)
					rise = std::min(max_rise, 2 * std::min(edge_distance, 2) +
													  std::max(0, edge_distance - 2));
				else if (roof_type == RoofType::Mansard) {
					int steep_height = 4;
					if (auto levels = parse_meter_tag(element.tags, "roof:levels"))
						steep_height = std::clamp(int(std::lround(*levels * 3.0)), 3, 6);
					const int all_edge_distance =
							std::min({std::max(0, x - min_x), std::max(0, max_x - x),
									std::max(0, z - min_z), std::max(0, max_z - z)});
					rise = std::min(max_rise,
							(int(std::min(all_edge_distance, 2) * steep_height) + 1) / 2 +
									std::max(0, all_edge_distance - 2) / 2);
				}
				int32_t roof_height = base_height + rise;
				if (roof_type == RoofType::HalfHipped) {
					const int along_edge = long_axis_is_x
												   ? std::min(x - min_x, max_x - x)
												   : std::min(z - min_z, max_z - z);
					roof_height = std::min(roof_height,
							base_height + std::max(2, max_rise / 2) + along_edge);
				}
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

				int32_t roof_height;
				if (roof_type == RoofType::Mansard) {
					int steep_height = 4;
					if (auto levels = parse_meter_tag(element.tags, "roof:levels"))
						steep_height = std::clamp(int(std::lround(*levels * 3.0)), 3, 6);
					const int edge_distance =
							std::min({std::max(0, x - min_x), std::max(0, max_x - x),
									std::max(0, z - min_z), std::max(0, max_z - z)});
					const int rise = (std::min(edge_distance, 2) * steep_height + 1) / 2 +
									 std::max(0, edge_distance - 2) / 2;
					roof_height = std::min(roof_peak_height, base_height + rise);
				} else {
					const double distance_factor =
							max_distance > 0.0
									? std::min(1.0, distance_from_center / max_distance)
									: 0.0;
					roof_height = roof_peak_height -
								  static_cast<int32_t>(distance_factor *
													   (roof_peak_height - base_height));
				}
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
		int32_t max_roof_height =
				tagged_roof_height
						? std::clamp(*tagged_roof_height, 1, 12)
						: std::min(std::clamp(width / 3, 2, 10),
								  std::max(1, int(std::lround(building_height * .9))));

		Block roof_block = tagged_roof_block.value_or(
				roof_friendly_block(rng.random_bool() ? accent_block : wall_block));

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

		int32_t peak_height =
				base_height + (tagged_roof_height ? *tagged_roof_height
												  : std::clamp(building_size / 3, 3, 8));

		Block roof_block = tagged_roof_block.value_or(
				roof_friendly_block(rng.random_bool() ? accent_block : wall_block));

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

	if (roof_type == RoofType::Cone) {
		const double half_w = std::max(1.0, double(max_x - min_x) / 2.0);
		const double half_l = std::max(1.0, double(max_z - min_z) / 2.0);
		const int peak_height =
				tagged_roof_height
						? *tagged_roof_height
						: std::min(std::max(2, int(std::min(half_w, half_l) * 1.2)),
								  building_height * 2);
		const Block roof_block = tagged_roof_block.value_or(
				rng.random_bool() ? accent_block : roof_friendly_block(wall_block));
		for (const auto &[x, z] : floor_area) {
			const double nx = double(x - center_x) / half_w;
			const double nz = double(z - center_z) / half_l;
			const double normalized = std::min(1.0, std::sqrt(nx * nx + nz * nz));
			const int surface = base_height + int((1.0 - normalized) * peak_height);
			for (int y = base_height; y <= surface; ++y)
				editor.set_block_absolute(roof_block, x, y + abs_terrain_offset, z);
		}
		return;
	}

	if (roof_type == RoofType::Onion) {
		const double base_radius =
				std::max(1.0, double(std::min(max_x - min_x, max_z - min_z)) / 2.0);
		const int total_height = tagged_roof_height
										 ? *tagged_roof_height
										 : std::min(std::max(6, int(base_radius * 1.8)),
												   building_height * 2);
		const int max_search = int(std::ceil(base_radius * 1.25)) + 1;
		const Block roof_block = tagged_roof_block.value_or(
				rng.random_bool() ? accent_block : roof_friendly_block(wall_block));
		std::unordered_set<std::pair<int, int>, PairHash> footprint(
				floor_area.begin(), floor_area.end());
		auto radius_factor = [](double t) {
			if (t < .05)
				return 1.0;
			if (t < .15)
				return 1.0 - ((t - .05) / .10) * .45;
			if (t < .55)
				return .55 + std::sin(((t - .15) / .40) * std::numbers::pi) * .65;
			if (t < .78)
				return .55 - ((t - .55) / .23) * .37;
			return std::max(0.0, .18 * (1.0 - (t - .78) / .22));
		};
		for (int layer = 0; layer <= total_height; ++layer) {
			const double t = double(layer) / std::max(1, total_height);
			const double radius = base_radius * radius_factor(t);
			const int y = base_height + layer + abs_terrain_offset;
			if (radius < .6 && t > .85) {
				editor.set_block_absolute(roof_block, center_x, y, center_z);
				continue;
			}
			if (t < .05) {
				for (const auto &[x, z] : floor_area)
					editor.set_block_absolute(roof_block, x, y, z);
				continue;
			}
			const double radius_sq = radius * radius;
			for (int dz = -max_search; dz <= max_search; ++dz)
				for (int dx = -max_search; dx <= max_search; ++dx) {
					if (double(dx * dx + dz * dz) > radius_sq)
						continue;
					const int x = center_x + dx, z = center_z + dz;
					if (t < .15 && !footprint.contains({x, z}))
						continue;
					editor.set_block_absolute(roof_block, x, y, z);
				}
		}
		return;
	}

	if (roof_type == RoofType::Dome) {
		const double half_w = std::max(1.0, double(max_x - min_x) / 2.0);
		const double half_l = std::max(1.0, double(max_z - min_z) / 2.0);
		const double rise = tagged_roof_height
									? double(*tagged_roof_height)
									: std::max(1.0, std::min(half_w, half_l) * .8);

		Block roof_block = tagged_roof_block.value_or(
				roof_friendly_block(rng.random_bool() ? accent_block : wall_block));

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			const double nx = double(x - center_x) / half_w;
			const double nz = double(z - center_z) / half_l;
			double normalized_distance = std::min(1.0, std::sqrt(nx * nx + nz * nz));

			double height_factor = std::sqrt(
					std::max(0.0, 1.0 - normalized_distance * normalized_distance));
			int32_t surface_height =
					base_height + static_cast<int32_t>(height_factor * rise);

			for (int32_t y = base_height; y <= surface_height; ++y) {
				editor.set_block_absolute(
						roof_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
			}
		}

		return;
	}
}


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
