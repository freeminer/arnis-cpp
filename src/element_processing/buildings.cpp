#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <initializer_list>
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
#include "../clipping.h"
#include "../floodfill.h"
#include "../floodfill_cache.h"
#include "buildings.h"
#include "../mapillary/facades.h"
#include "building_facade.h"
#include "../building_facades/registry.h"
#include "signage.h"
#include "../decals/pictograms.h"
#include "../decals/font.h"
#include "historic.h"
#include "parking_garage.h"
#include "subprocessor/buildings_interior.h"
#include "subprocessor/interior/uses.h"
#include "subprocessor/interior/index.h"
#include "../structures/structures.h"
#include "../structures/starship.h"
#include "../deterministic_rng.h"
#include "../block_palette.h"
#include "../osm_parser.h"
#include "../strict_parse.h"
#include "way_segments.h"
namespace arnis
{

namespace
{
constexpr std::array<std::string_view, 21> INTERIOR_EXCLUSIONS = {"garage", "garages",
		"carport", "shed", "parking", "roof", "bridge", "greenhouse", "glasshouse",
		"silo", "storage_tank", "transformer_tower", "water_tower", "service", "toilets",
		"bunker", "container", "outbuilding", "allotment_house", "boathouse", "pavilion"};
}

namespace man_made
{
bool is_tank_structure(const ProcessedWay &way);
void generate_tank_structure(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
}

namespace buildings
{

Block rust_category_wall_block(
		BuildingCategory category, ArchEra era, biome::Climate climate, ChaCha8Rng &rng);

std::optional<double> parse_roof_direction_degrees(const std::string &value)
{
	std::string s = value;
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
		s.erase(s.begin());
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
		s.pop_back();
	std::transform(s.begin(), s.end(), s.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (s == "n" || s == "north")
		return 0.0;
	if (s == "ne")
		return 45.0;
	if (s == "e" || s == "east")
		return 90.0;
	if (s == "se")
		return 135.0;
	if (s == "s" || s == "south")
		return 180.0;
	if (s == "sw")
		return 225.0;
	if (s == "w" || s == "west")
		return 270.0;
	if (s == "nw")
		return 315.0;
	const auto parsed = strict_parse::f64(s);
	if (!parsed || !std::isfinite(*parsed))
		return std::nullopt;
	double degrees = std::fmod(*parsed, 360.0);
	if (degrees < 0.0)
		degrees += 360.0;
	return degrees == 0.0 ? 0.0 : degrees;
}

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
	Round,
	Flat
};

bool is_glass_wall_block(const Block &block)
{
	return block == GLASS || block == WHITE_STAINED_GLASS ||
		   block == GRAY_STAINED_GLASS || block == LIGHT_GRAY_STAINED_GLASS ||
		   block == BLACK_STAINED_GLASS || block == BROWN_STAINED_GLASS ||
		   block == CYAN_STAINED_GLASS || block == BLUE_STAINED_GLASS ||
		   block == LIGHT_BLUE_STAINED_GLASS;
}

constexpr int BUILDING_PASSAGE_HEIGHT = 4;
constexpr std::size_t THIN_RING_MAX_CELLS = 4;

int floor_div(int value, int divisor)
{
	const int quotient = value / divisor;
	return quotient - (value % divisor < 0 ? 1 : 0);
}

std::vector<std::pair<int, int>> thin_ring_cells(const ProcessedWay &way)
{
	if (way.nodes.size() < 4 || way.nodes.front().x != way.nodes.back().x ||
			way.nodes.front().z != way.nodes.back().z)
		return {};
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i)
		for (const auto &[x, unused_y, z] : bresenham_line(way.nodes[i - 1].x, 0,
					 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z))
			cells.emplace_back(x, z);
	std::sort(cells.begin(), cells.end());
	cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
	return cells;
}

// Rust's radial roof profile is only valid when the polygon is star-shaped
// around its area centroid. Return no profile for courtyards or concavities
// whose rays cross meaningful open gaps; callers then use their bbox fallback.
std::optional<std::vector<double>> footprint_radial_fractions(
		const ProcessedWay &element, const std::vector<std::pair<int, int>> &cells)
{
	std::vector<std::pair<double, double>> points;
	points.reserve(element.nodes.size());
	for (const auto &node : element.nodes) {
		const std::pair<double, double> point{double(node.x), double(node.z)};
		if (points.empty() || points.back() != point)
			points.push_back(point);
	}
	if (points.size() > 1 && points.front() == points.back())
		points.pop_back();
	if (points.size() < 3)
		return std::nullopt;
	const std::size_t count = points.size();
	double area2 = 0.0, centroid_x = 0.0, centroid_z = 0.0;
	for (std::size_t i = 0; i < count; ++i) {
		const auto [ax, az] = points[i];
		const auto [bx, bz] = points[(i + 1) % count];
		const double cross = ax * bz - bx * az;
		area2 += cross;
		centroid_x += (ax + bx) * cross;
		centroid_z += (az + bz) * cross;
	}
	if (std::abs(area2) < 1e-9)
		return std::nullopt;
	centroid_x /= 3.0 * area2;
	centroid_z /= 3.0 * area2;
	bool inside = false;
	for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
		const auto [ax, az] = points[i];
		const auto [bx, bz] = points[j];
		if ((az > centroid_z) != (bz > centroid_z)) {
			const double crossing = ax + (centroid_z - az) * (bx - ax) / (bz - az);
			if (crossing > centroid_x)
				inside = !inside;
		}
	}
	if (!inside)
		return std::nullopt;
	constexpr std::size_t BUCKET_COUNT = 512;
	constexpr double PI = std::numbers::pi;
	constexpr double TAU = 2.0 * PI;
	auto bucket_of = [](double angle) {
		return std::min(BUCKET_COUNT - 1,
				static_cast<std::size_t>((angle + PI) / TAU * BUCKET_COUNT));
	};
	std::array<std::vector<std::size_t>, BUCKET_COUNT> buckets;
	for (std::size_t i = 0; i < count; ++i) {
		const auto [ax, az] = points[i];
		const auto [bx, bz] = points[(i + 1) % count];
		const double a0 = std::atan2(az - centroid_z, ax - centroid_x);
		const double a1 = std::atan2(bz - centroid_z, bx - centroid_x);
		const double lo = std::min(a0, a1), hi = std::max(a0, a1);
		const std::size_t blo = bucket_of(lo), bhi = bucket_of(hi);
		if (hi - lo > PI) {
			for (std::size_t b = 0; b <= std::min(BUCKET_COUNT - 1, blo + 1); ++b)
				buckets[b].push_back(i);
			for (std::size_t b = bhi == 0 ? 0 : bhi - 1; b < BUCKET_COUNT; ++b)
				buckets[b].push_back(i);
		} else {
			const std::size_t from = blo == 0 ? 0 : blo - 1;
			const std::size_t to = std::min(BUCKET_COUNT - 1, bhi + 1);
			for (std::size_t b = from; b <= to; ++b)
				buckets[b].push_back(i);
		}
	}
	std::vector<double> fractions;
	fractions.reserve(cells.size());
	std::vector<double> before;
	for (const auto &[x, z] : cells) {
		const double px = double(x) - centroid_x, pz = double(z) - centroid_z;
		const double distance = std::hypot(px, pz);
		if (distance < 1e-9) {
			fractions.push_back(0.0);
			continue;
		}
		const double ux = px / distance, uz = pz / distance;
		const auto &edges = buckets[bucket_of(std::atan2(pz, px))];
		double hit = std::numeric_limits<double>::infinity(), far = 0.0;
		before.clear();
		for (const std::size_t i : edges) {
			const auto [ax, az] = points[i];
			const auto [bx, bz] = points[(i + 1) % count];
			const double ex = bx - ax, ez = bz - az;
			const double denom = ux * ez - uz * ex;
			if (std::abs(denom) < 1e-12)
				continue;
			const double wx = ax - centroid_x, wz = az - centroid_z;
			const double t = (wx * ez - wz * ex) / denom;
			const double s = (wx * uz - wz * ux) / denom;
			if (t <= 0.0 || s < -1e-9 || s > 1.0 + 1e-9)
				continue;
			far = std::max(far, t);
			if (t + 0.5 >= distance)
				hit = std::min(hit, t);
			else
				before.push_back(t);
		}
		std::sort(before.begin(), before.end());
		for (std::size_t i = 1; i < before.size(); i += 2)
			if (before[i] - before[i - 1] > 1.0)
				return std::nullopt;
		const double boundary = std::isfinite(hit) ? hit : far;
		fractions.push_back(boundary > 0.0 ? std::min(1.0, distance / boundary) : 1.0);
	}
	return fractions;
}

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, std::optional<Block> roof_block_override,
		std::optional<Block> curtain_glass, std::uint64_t style_seed, bool add_dormers,
		bool steep_gable, std::optional<bool> preferred_ridge_along_x, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset, double scale_factor);

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
	return strict_parse::i32(it->second);
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
	if (has_tag_value(tags, "location", "underground") ||
			has_tag_value(tags, "location", "subway"))
		return true;
	if (has_tag_value(tags, "location", "surface") ||
			has_tag_value(tags, "location", "overground") ||
			has_tag_value(tags, "location", "roof"))
		return false;
	if (auto layer = parse_i32_tag(tags, "layer"); layer.has_value() && *layer < 0)
		return true;
	if (auto level = parse_i32_tag(tags, "level"); level.has_value() && *level < 0)
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
		return *flood_fill_cache->get_or_compute(way, args.timeout);
	return flood_fill_area(way_polygon_coords(way), args.timeout_ref());
}

bool close_ring_if_near(std::vector<ProcessedNode> &ring)
{
	if (ring.size() < 3)
		return false;
	const auto first = ring.front();
	const auto last = ring.back();
	if (first.id != last.id && std::abs(first.x - last.x) <= 1 &&
			std::abs(first.z - last.z) <= 1)
		ring.push_back(first);
	const auto &closed_last = ring.back();
	return first.id == closed_last.id || (std::abs(first.x - closed_last.x) <= 1 &&
												 std::abs(first.z - closed_last.z) <= 1);
}

std::vector<std::vector<ProcessedNode>> collect_merged_rings(
		const ProcessedRelation &relation, ProcessedMemberRole role, const XZBBox &xzbbox)
{
	static constexpr std::array<std::uint64_t, 5> skipped_outer_ids = {
			5013364, 204068874, 32920861, 1352374225, 1486731987};
	std::vector<std::vector<ProcessedNode>> rings;
	for (const auto &member : relation.members) {
		const bool skip = role == ProcessedMemberRole::Outer &&
						  std::find(skipped_outer_ids.begin(), skipped_outer_ids.end(),
								  member.way.id) != skipped_outer_ids.end();
		if (member.role == role && !skip)
			rings.push_back(member.way.nodes);
	}
	merge_way_segments(rings);
	std::vector<std::vector<ProcessedNode>> out;
	for (auto &ring : rings) {
		ring = clipping::clip_way_to_bbox(ring, xzbbox);
		if (ring.size() < 4)
			continue;
		close_ring_if_near(ring);
		if (ring.size() >= 4 &&
				(ring.front().id == ring.back().id ||
						(std::abs(ring.front().x - ring.back().x) <= 1 &&
								std::abs(ring.front().z - ring.back().z) <= 1)))
			out.push_back(std::move(ring));
	}
	return out;
}

std::uint64_t relation_ring_id(std::uint64_t relation_id, std::size_t ring_index)
{
	return (std::uint64_t{1} << 63) | (relation_id << 16) |
		   (static_cast<std::uint64_t>(ring_index) & 0xffffULL);
}

bool passage_at(const CoordinateBitmap *building_passages, int x, int z)
{
	return building_passages && building_passages->contains(x, z);
}

std::optional<double> parse_meter_value(std::string value)
{
	const auto trim = [&value] {
		while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
			value.erase(value.begin());
		while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
			value.pop_back();
	};
	trim();
	while (!value.empty() && value.back() == 'm')
		value.pop_back();
	trim();
	return strict_parse::f64(value);
}

double parse_tag_meters(const std::string &value, double fallback = 0.0)
{
	return parse_meter_value(value).value_or(fallback);
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
		if (it == tags.end())
			return false;
		const auto &value = it->second;
		return value.size() != 2 ||
			   std::tolower(static_cast<unsigned char>(value[0])) != 'n' ||
			   std::tolower(static_cast<unsigned char>(value[1])) != 'o';
	};
	if (positive("historic") || positive("heritage") || tags.contains("ref:nrhp") ||
			positive("listed_status"))
		return ArchEra::HistoricOrnate;
	auto matches = [](std::string_view value,
						   std::initializer_list<std::string_view> choices) {
		return std::find(choices.begin(), choices.end(), value) != choices.end();
	};
	const auto architecture_value = [&]() -> std::string_view {
		for (const char *key : {"building:architecture", "architecture"})
			if (auto it = tags.find(key); it != tags.end())
				return it->second;
		return {};
	}();
	const auto architecture = normalized_material(architecture_value);
	if (matches(architecture,
				{"artdeco", "artnouveau", "gothic", "neogothic", "gothicrevival",
						"neoclassicism", "neoclassical", "classicism", "classicalrevival",
						"greekrevival", "baroque", "neobaroque", "rococo", "barocco",
						"historicism", "eclectic", "renaissance", "neorenaissance",
						"romanesque", "neoromanesque", "romanesquerevival", "victorian",
						"georgian", "federal", "italianate", "beauxarts",
						"wilhelminianstyle", "queenanne"}))
		return ArchEra::HistoricOrnate;
	if (matches(architecture, {"brutalist", "constructivism", "stalinistneoclassicism"}))
		return ArchEra::PostWarPanel;
	if (matches(architecture, {"modern", "contemporary", "modernism", "functionalism",
									  "newobjectivity", "postmodern", "bauhaus"}))
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
	const auto material_value = [&]() -> std::string_view {
		for (const char *key :
				{"building:material", "building:facade:material", "facade:material"})
			if (auto it = tags.find(key); it != tags.end())
				return it->second;
		return {};
	}();
	const auto material = normalized_material(material_value);
	if (matches(material, {"panel", "panels", "prefab", "prefabricated", "panelhouse"}))
		return ArchEra::PostWarPanel;
	if (matches(material,
				{"brick", "bricks", "redbrick", "silicatebrick", "stone", "naturalstone",
						"sandstone", "limestone", "masonry", "granite", "marble",
						"terracotta", "adobe", "stucco", "pebbledash"}))
		return ArchEra::TraditionalPreWar;
	if (matches(material, {"concrete", "reinforcedconcrete", "concretereinforced",
								  "concretemasonryunit", "glass", "mirror"}))
		return ArchEra::Contemporary;
	const auto cladding = normalized_material(tags.get("building:cladding"));
	if (matches(cladding, {"brick", "brickmonolith", "plaster", "rendered", "rendering",
								  "stone", "tiling"}))
		return ArchEra::TraditionalPreWar;
	return ArchEra::Unknown;
}

BuildingCategory building_category(const ProcessedWay &element, bool tall, int height,
		double scale, std::uint64_t seed)
{
	if (element.tags.get("man_made") == "tower")
		return BuildingCategory::Tower;
	if (tall) {
		const std::uint64_t clean_seed = osm_parser::seed_without_hint(seed);
		auto hint = osm_parser::style_hint_from_seed(seed);
		if (hint == osm_parser::StyleHint::None)
			hint = osm_parser::building_style_hint(element.tags);
		switch (hint) {
		case osm_parser::StyleHint::Glass:
			switch ((clean_seed ^ 0x6C07A55EULL) * 2654435761ULL % 3) {
			case 0:
				return BuildingCategory::GlassySkyscraper;
			case 1:
				return BuildingCategory::GridSkyscraper;
			default:
				return BuildingCategory::GlassCornerSkyscraper;
			}
		case osm_parser::StyleHint::Masonry:
			return BuildingCategory::MasonrySkyscraper;
		case osm_parser::StyleHint::Contemporary:
			return BuildingCategory::ContemporarySkyscraper;
		case osm_parser::StyleHint::None:
			break;
		}
		bool skyscraper = false;
		if (element.nodes.size() >= 3) {
			int min_x = INT_MAX, max_x = INT_MIN, min_z = INT_MAX, max_z = INT_MIN;
			for (const auto &node : element.nodes) {
				min_x = std::min(min_x, node.x);
				max_x = std::max(max_x, node.x);
				min_z = std::min(min_z, node.z);
				max_z = std::max(max_z, node.z);
			}
			const int longest_side = std::max(1, std::max(max_x - min_x, max_z - min_z));
			skyscraper = height >= int(120 * scale) &&
						 double(height) / double(longest_side) >= 2.0;
		}
		if (skyscraper) {
			const auto variant = (clean_seed * 2654435761ULL) % 100;
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
			type == "stable" || type == "cowshed" || type == "sty" || type == "sheepfold")
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

[[maybe_unused]] Block legacy_category_wall_block(
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
	// Rust uses climate-aware weighted façades for ordinary residential fabric
	// (and hot-climate commercial buildings).  Keep explicit material tags out
	// of this path; this only shapes otherwise inferred building appearance.
	const bool residential = category == BuildingCategory::House ||
							 category == BuildingCategory::Residential;
	const bool hot_commercial = (category == BuildingCategory::Commercial ||
										category == BuildingCategory::Office ||
										category == BuildingCategory::Hotel) &&
								(climate == biome::Climate::HotDesert ||
										climate == biome::Climate::HotSteppe ||
										climate == biome::Climate::TropicalSavanna);
	if (climate != biome::Climate::Temperate && (residential || hot_commercial)) {
		if (residential &&
				(climate == biome::Climate::Boreal || climate == biome::Climate::Tundra ||
						climate == biome::Climate::IceCap))
			palette.insert(palette.end(), {SPRUCE_PLANKS, OAK_PLANKS, DARK_OAK_PLANKS});
		auto weight = [&](Block b) -> int {
			auto residential_weight = [&](int value) {
				return category == BuildingCategory::Residential ? std::max(1, value / 2)
																 : value;
			};
			switch (climate) {
			case biome::Climate::HotDesert:
			case biome::Climate::HotSteppe:
				if (b == SANDSTONE || b == SMOOTH_SANDSTONE || b == MUD_BRICKS ||
						b == WHITE_TERRACOTTA)
					return 4;
				if (b == WHITE_CONCRETE || b == END_STONE_BRICKS || b == TERRACOTTA)
					return 3;
				if (b == LIGHT_GRAY_CONCRETE || b == QUARTZ_BRICKS || b == QUARTZ_BLOCK ||
						b == ORANGE_TERRACOTTA || b == BROWN_TERRACOTTA)
					return 2;
				if (b == RED_TERRACOTTA || b == SPRUCE_PLANKS || b == OAK_PLANKS ||
						b == DARK_OAK_PLANKS || b == DEEPSLATE_BRICKS ||
						b == POLISHED_DEEPSLATE || b == POLISHED_BLACKSTONE ||
						b == NETHER_BRICK)
					return 0;
				return 1;
			case biome::Climate::TropicalSavanna:
				if (b == WHITE_CONCRETE)
					return 4;
				if (b == WHITE_TERRACOTTA || b == LIGHT_GRAY_CONCRETE || b == BRICK)
					return 3;
				if (b == GRAY_CONCRETE || b == TERRACOTTA || b == ORANGE_TERRACOTTA ||
						b == QUARTZ_BRICKS || b == SMOOTH_SANDSTONE || b == MUD_BRICKS)
					return 2;
				if (b == SPRUCE_PLANKS || b == OAK_PLANKS || b == DARK_OAK_PLANKS ||
						b == DEEPSLATE_BRICKS || b == POLISHED_DEEPSLATE ||
						b == POLISHED_BLACKSTONE || b == NETHER_BRICK)
					return 0;
				return 1;
			case biome::Climate::Boreal:
				if (b == SPRUCE_PLANKS)
					return residential_weight(5);
				if (b == OAK_PLANKS)
					return residential_weight(3);
				if (b == DARK_OAK_PLANKS)
					return residential_weight(2);
				if (b == RED_TERRACOTTA)
					return 3;
				if (b == WHITE_CONCRETE || b == WHITE_TERRACOTTA || b == BRICK)
					return 2;
				return (b == SANDSTONE || b == SMOOTH_SANDSTONE || b == MUD_BRICKS) ? 0
																					: 1;
			case biome::Climate::Tundra:
			case biome::Climate::IceCap:
				if (b == SPRUCE_PLANKS)
					return residential_weight(3);
				if (b == OAK_PLANKS)
					return residential_weight(2);
				if (b == RED_TERRACOTTA || b == WHITE_CONCRETE || b == GRAY_CONCRETE ||
						b == LIGHT_GRAY_CONCRETE)
					return 2;
				return (b == SANDSTONE || b == SMOOTH_SANDSTONE || b == MUD_BRICKS ||
							   b == DARK_OAK_PLANKS)
							   ? 0
							   : 1;
			case biome::Climate::ColdSteppe:
			case biome::Climate::ColdDesert:
			case biome::Climate::DryContinental:
				if (b == BRICK)
					return 3;
				if (b == WHITE_TERRACOTTA || b == WHITE_CONCRETE ||
						b == LIGHT_GRAY_CONCRETE || b == GRAY_CONCRETE ||
						b == STONE_BRICKS || b == MUD_BRICKS)
					return 2;
				return (b == SPRUCE_PLANKS || b == OAK_PLANKS || b == DARK_OAK_PLANKS)
							   ? 0
							   : 1;
			case biome::Climate::Temperate:
				return 1;
			}
			return 1;
		};
		std::vector<Block> weighted;
		for (Block block : palette)
			for (int copies = weight(block); copies > 0; --copies)
				weighted.push_back(block);
		if (!weighted.empty())
			palette = std::move(weighted);
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

bool segment_axis_x(const ProcessedNode &a, const ProcessedNode &b)
{
	return std::abs(b.x - a.x) >= std::abs(b.z - a.z);
}

bool segment_start_axis_x(
		const std::vector<ProcessedNode> &nodes, std::size_t i, bool own_axis_x)
{
	const auto n = nodes.size();
	const bool closed = n >= 3 && nodes.front().x == nodes.back().x &&
						nodes.front().z == nodes.back().z;
	if (i == 0 || i > n)
		return own_axis_x;
	const auto &here = nodes[i - 1];
	std::size_t j = i - 1;
	for (std::size_t step = 0; step < n; ++step) {
		std::size_t prev_idx;
		if (j >= 1)
			prev_idx = j - 1;
		else if (closed)
			prev_idx = n - 2;
		else
			return own_axis_x;
		const auto &prev = nodes[prev_idx];
		if (prev.x != here.x || prev.z != here.z)
			return segment_axis_x(prev, here);
		j = prev_idx;
	}
	return own_axis_x;
}

int wall_ordinate(
		std::size_t point_index, bool start_axis_x, bool segment_is_x, int x, int z)
{
	return (point_index == 0 ? start_axis_x : segment_is_x) ? x : z;
}

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
	auto rng = element_rng(seed ^ 0xF7A3E00157BD2210ULL);
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
	const auto roll = element_rng(seed ^ 0xBA1C041700000009ULL).uniform(100);
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

WallDepthStyle wall_depth_style_for(BuildingCategory category)
{
	// Every non-default category has an explicit Rust BuildingStylePreset depth
	// value. Resolve those before any generic size/era fallback, as Rust's
	// preset.unwrap_or_else does.
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
	case BuildingCategory::ContemporarySkyscraper:
		return WallDepthStyle::ModernPillars;
	case BuildingCategory::Religious:
		return WallDepthStyle::ReligiousButtress;
	case BuildingCategory::TallBuilding:
	case BuildingCategory::ModernSkyscraper:
		return WallDepthStyle::SkyscraperFins;
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
		return WallDepthStyle::GlassCurtain;
	case BuildingCategory::GridSkyscraper:
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
	auto rng = element_rng(seed ^ 0x57A2C0DEA5C10007ULL);
	auto roll = rng.uniform(100);
	W picked = W::Standard3;
	for (const auto &[value, weight] : table) {
		if (roll < weight) {
			picked = value;
			break;
		}
		roll -= weight;
	}
	auto era_rng = element_rng(seed ^ 0x57A2C0DEA5C10008ULL);
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
	const Block *pool = nullptr;
	std::size_t count = 0;
	static const std::array residential = {GLASS, WHITE_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, BROWN_STAINED_GLASS, TINTED_GLASS,
			LIGHT_BLUE_STAINED_GLASS};
	static const std::array institutional = {GLASS, WHITE_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, LIGHT_BLUE_STAINED_GLASS};
	static const std::array hospitality = {GLASS, WHITE_STAINED_GLASS, GRAY_STAINED_GLASS,
			LIGHT_BLUE_STAINED_GLASS, BROWN_STAINED_GLASS};
	static const std::array industrial = {
			GLASS, GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, BROWN_STAINED_GLASS};
	static const std::array religious = {BLUE_STAINED_GLASS, CYAN_STAINED_GLASS,
			LIGHT_BLUE_STAINED_GLASS, BROWN_STAINED_GLASS, WHITE_STAINED_GLASS};
	static const std::array farm = {GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS};
	static const std::array historic = {
			GLASS, LIGHT_GRAY_STAINED_GLASS, WHITE_STAINED_GLASS};
	static const std::array variations = {GLASS, GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, BROWN_STAINED_GLASS,
			WHITE_STAINED_GLASS, TINTED_GLASS, LIGHT_BLUE_STAINED_GLASS,
			CYAN_STAINED_GLASS, BLACK_STAINED_GLASS, BROWN_STAINED_GLASS};
	const auto use = [&](const auto &options) {
		pool = options.data();
		count = options.size();
	};
	switch (category) {
	case BuildingCategory::House:
	case BuildingCategory::Residential:
		use(residential);
		break;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		use(institutional);
		break;
	case BuildingCategory::Hotel:
		use(hospitality);
		break;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		use(industrial);
		break;
	case BuildingCategory::Religious:
		use(religious);
		break;
	case BuildingCategory::Farm:
		use(farm);
		break;
	case BuildingCategory::Historic:
		use(historic);
		break;
	default:
		use(variations);
		break;
	}
	return pool[rng.uniform(static_cast<std::uint32_t>(count))];
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
	if (m == "wood" || m == "timber" || m == "timberframe" || m == "timberframing" ||
			m == "timberplanks" || m == "halftimber" || m == "halftimbered" ||
			m == "loghouse" || m == "logs" || m == "bamboo")
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
	if (normalized == "glass" || normalized == "glazing" ||
			normalized == "acrylicglass" || normalized == "mirror")
		return choose_block({GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS}, rng);
	if (normalized == "tile" || normalized == "tiles" || normalized == "rooftile" ||
			normalized == "rooftiles" || normalized == "ceramic" ||
			normalized == "ceramictiles" || normalized == "claytile" ||
			normalized == "claytiles" || normalized == "cementtile" ||
			normalized == "cementtiles" || normalized == "terracotta")
		return choose_block({BRICK, NETHER_BRICK, RED_NETHER_BRICKS, MUD_BRICKS}, rng);
	if (normalized == "slate" || normalized == "slates")
		return choose_block({POLISHED_BLACKSTONE, DEEPSLATE_BRICKS, BLACKSTONE}, rng);
	if (normalized == "metal" || normalized == "steel" || normalized == "aluminium" ||
			normalized == "aluminum" || normalized == "corrugatedsteel" ||
			normalized == "corrugatediron" || normalized == "corrugatedmetal" ||
			normalized == "corrugatedironsheets" || normalized == "corrugated" ||
			normalized == "cgi" || normalized == "tin" || normalized == "zinc" ||
			normalized == "zink" || normalized == "lead" || normalized == "sheetmetal" ||
			normalized == "metalsheet" || normalized == "metalplates")
		return choose_block({LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, IRON_BLOCK}, rng);
	if (normalized == "copper")
		return choose_block(
				{WAXED_OXIDIZED_COPPER, WAXED_EXPOSED_COPPER, WAXED_COPPER_BLOCK}, rng);
	if (normalized == "concrete" || normalized == "reinforcedconcrete" ||
			normalized == "rcc" || normalized == "concerte" || normalized == "cement" ||
			normalized == "concreteslab")
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
			normalized == "rolledroofing" || normalized == "asphaltshingle" ||
			normalized == "asphaltshingles" || normalized == "roofingfelt")
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
	// These blocks share one stair variant, keeping a pitched solar-panel roof
	// visually uniform rather than alternating incompatible block states.
	if (normalized == "solarpanels" || normalized == "photovoltaic")
		return choose_block({BLACK_CONCRETE, BLACKSTONE, POLISHED_BLACKSTONE}, rng);
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

std::vector<Block> roof_substitute_pool(Block block)
{
	if (block == STONE_BRICKS)
		return {COBBLESTONE, CRACKED_STONE_BRICKS, ANDESITE, CHISELED_STONE_BRICKS};
	if (block == COBBLESTONE)
		return {STONE_BRICKS, ANDESITE, STONE};
	if (block == STONE)
		return {COBBLESTONE, STONE_BRICKS};
	if (block == ANDESITE)
		return {POLISHED_ANDESITE, COBBLESTONE, STONE_BRICKS};
	if (block == POLISHED_ANDESITE)
		return {ANDESITE, STONE_BRICKS};
	if (block == CHISELED_STONE_BRICKS)
		return {STONE_BRICKS, CRACKED_STONE_BRICKS};
	if (block == CRACKED_STONE_BRICKS)
		return {STONE_BRICKS, MOSSY_STONE_BRICKS, CHISELED_STONE_BRICKS};
	if (block == TUFF)
		return {STONE_BRICKS, MOSSY_STONE_BRICKS};
	if (block == MOSSY_STONE_BRICKS)
		return {MOSSY_COBBLESTONE, TUFF};
	if (block == MOSSY_COBBLESTONE)
		return {MOSSY_STONE_BRICKS, TUFF};
	if (block == BRICK)
		return {POLISHED_GRANITE, GRANITE, TERRACOTTA};
	if (block == POLISHED_GRANITE)
		return {BRICK, GRANITE};
	if (block == GRANITE)
		return {POLISHED_GRANITE, BRICK};
	if (block == TERRACOTTA)
		return {BRICK, POLISHED_GRANITE};
	if (block == DEEPSLATE_BRICKS)
		return {POLISHED_DEEPSLATE, COBBLED_DEEPSLATE};
	if (block == POLISHED_DEEPSLATE)
		return {DEEPSLATE_BRICKS, COBBLED_DEEPSLATE};
	if (block == COBBLED_DEEPSLATE)
		return {DEEPSLATE_BRICKS, POLISHED_DEEPSLATE, DEEPSLATE};
	if (block == DEEPSLATE)
		return {COBBLED_DEEPSLATE, DEEPSLATE_BRICKS};
	if (block == POLISHED_BLACKSTONE_BRICKS)
		return {POLISHED_BLACKSTONE, BLACKSTONE};
	if (block == POLISHED_BLACKSTONE)
		return {POLISHED_BLACKSTONE_BRICKS, BLACKSTONE};
	if (block == BLACKSTONE)
		return {POLISHED_BLACKSTONE, POLISHED_BLACKSTONE_BRICKS};
	if (block == NETHER_BRICK)
		return {BLACK_TERRACOTTA};
	if (block == BLACK_TERRACOTTA)
		return {NETHER_BRICK};
	if (block == RED_NETHER_BRICKS)
		return {NETHER_BRICK};
	if (block == BROWN_TERRACOTTA)
		return {BROWN_CONCRETE_POWDER, BROWN_CONCRETE};
	if (block == BROWN_CONCRETE)
		return {BROWN_CONCRETE_POWDER, BROWN_TERRACOTTA};
	if (block == QUARTZ_BLOCK)
		return {QUARTZ_BRICKS, SMOOTH_QUARTZ};
	if (block == QUARTZ_BRICKS)
		return {QUARTZ_BLOCK, WHITE_CONCRETE, SMOOTH_QUARTZ};
	if (block == SMOOTH_QUARTZ)
		return {QUARTZ_BLOCK, QUARTZ_BRICKS};
	if (block == WHITE_CONCRETE)
		return {QUARTZ_BRICKS};
	if (block == SANDSTONE)
		return {SMOOTH_SANDSTONE};
	if (block == SMOOTH_SANDSTONE)
		return {SANDSTONE};
	if (block == SMOOTH_STONE)
		return {STONE_BRICKS, STONE};
	if (block == POLISHED_DIORITE)
		return {DIORITE};
	if (block == DIORITE)
		return {POLISHED_DIORITE};
	if (block == LIGHT_GRAY_TERRACOTTA)
		return {WAXED_EXPOSED_COPPER, WAXED_EXPOSED_CHISELED_COPPER,
				WAXED_EXPOSED_CUT_COPPER};
	if (block == WAXED_EXPOSED_COPPER)
		return {LIGHT_GRAY_TERRACOTTA, WAXED_EXPOSED_CHISELED_COPPER,
				WAXED_EXPOSED_CUT_COPPER};
	if (block == WAXED_EXPOSED_CHISELED_COPPER)
		return {LIGHT_GRAY_TERRACOTTA, WAXED_EXPOSED_COPPER, WAXED_EXPOSED_CUT_COPPER};
	if (block == WAXED_EXPOSED_CUT_COPPER)
		return {LIGHT_GRAY_TERRACOTTA, WAXED_EXPOSED_COPPER,
				WAXED_EXPOSED_CHISELED_COPPER};
	if (block == ORANGE_TERRACOTTA)
		return {WAXED_COPPER_BLOCK, TERRACOTTA};
	if (block == WAXED_COPPER_BLOCK)
		return {ORANGE_TERRACOTTA, TERRACOTTA};
	if (block == GRAY_CONCRETE)
		return {MUD};
	if (block == MUD)
		return {GRAY_CONCRETE};
	if (block == OAK_PLANKS)
		return {SPRUCE_PLANKS};
	if (block == SPRUCE_PLANKS)
		return {OAK_PLANKS, DARK_OAK_PLANKS};
	if (block == DARK_OAK_PLANKS)
		return {SPRUCE_PLANKS, SPRUCE_LOG};
	if (block == OAK_LOG)
		return {SPRUCE_LOG};
	return {};
}

Block default_roof_surface_block(Block wall_block, std::uint64_t style_seed)
{
	auto rng = element_rng(style_seed ^ 0xA11ED700F1A75EEDULL);
	auto stone_brick_rng = element_rng(style_seed ^ 0x574FE48B1C42E091ULL);
	const bool force_stone_bricks = stone_brick_rng.random_bool(.15);
	Block raw_roof = wall_block;
	if (force_stone_bricks) {
		raw_roof = STONE_BRICKS;
	} else if (rng.random_bool(.1)) {
		const auto pool = roof_substitute_pool(wall_block);
		if (!pool.empty())
			raw_roof = pool[rng.uniform(static_cast<std::uint32_t>(pool.size()))];
	}
	return roof_friendly_block(raw_roof);
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

// generate_roof's Rust counterpart intentionally considers only roof:* tags;
// building/material and building/colour are used by separate style selection.
std::optional<Block> roof_surface_block_from_tags(
		const ProcessedWay &element, ChaCha8Rng &rng)
{
	if (auto it = element.tags.find("roof:material"); it != element.tags.end()) {
		if (auto block = get_roof_block_for_material_cpp(it->second, rng))
			return block;
	}
	if (auto it = element.tags.find("roof:colour"); it != element.tags.end()) {
		if (auto rgb = color_text_to_rgb_tuple(it->second))
			return block_palette::roof_block_for_color(*rgb, rng);
	}
	return std::nullopt;
}

double compute_building_diagonality(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return 1.0;
	std::int64_t twice_area = 0;
	int min_x = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
	for (std::size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		twice_area += std::int64_t(a.x) * b.z - std::int64_t(b.x) * a.z;
		min_x = std::min(min_x, a.x);
		max_x = std::max(max_x, a.x);
		min_z = std::min(min_z, a.z);
		max_z = std::max(max_z, a.z);
	}
	const double bbox_area = double(max_x - min_x + 1) * (max_z - min_z + 1);
	if (bbox_area <= 0.0)
		return 1.0;
	return std::min(1.0, std::abs(double(twice_area)) / (2.0 * bbox_area));
}

double dominant_building_axis_angle(const std::vector<ProcessedNode> &nodes)
{
	double sum_cos = 0.0, sum_sin = 0.0;
	for (std::size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		const double dx = double(b.x) - a.x, dz = double(b.z) - a.z;
		const double length = std::hypot(dx, dz);
		if (length < 3.0)
			continue;
		const double angle = std::atan2(dz, dx);
		sum_cos += length * std::cos(4.0 * angle);
		sum_sin += length * std::sin(4.0 * angle);
	}
	if (sum_cos == 0.0 && sum_sin == 0.0)
		return 0.0;
	return std::atan2(sum_sin, sum_cos) / 4.0;
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
	if (roof_shape == "half-hipped" || roof_shape == "half_hipped" ||
			roof_shape == "side_half-hipped")
		return RoofType::HalfHipped;
	if (roof_shape == "round" || roof_shape == "barrel" || roof_shape == "vault" ||
			roof_shape == "vaulted")
		return RoofType::Round;
	if (roof_shape == "hipped" || roof_shape == "hip" || roof_shape == "side_hipped")
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

bool roof_dormers_enabled(
		BuildingCondition condition, BuildingCategory category, RoofType roof_type)
{
	if (condition != BuildingCondition::Normal)
		return false;
	if ((category == BuildingCategory::House ||
				category == BuildingCategory::Residential) &&
			(roof_type == RoofType::Gabled || roof_type == RoofType::Hipped ||
					roof_type == RoofType::Mansard))
		return true;
	return category == BuildingCategory::Historic && roof_type == RoofType::Mansard;
}

struct RoofTentFrame
{
	double angle;
	double u_min;
	double u_max;
	double v_min;
	double v_max;

	std::pair<double, double> uv(double x, double z) const
	{
		const double c = std::cos(angle), s = std::sin(angle);
		return {x * c + z * s, -x * s + z * c};
	}
	std::pair<double, double> u_axis() const
	{
		return {std::cos(angle), std::sin(angle)};
	}
	std::pair<double, double> v_axis() const
	{
		return {-std::sin(angle), std::cos(angle)};
	}
};

std::optional<RoofTentFrame> roof_tent_frame(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return std::nullopt;
	double sum_c = 0.0, sum_s = 0.0;
	std::int64_t twice_area = 0;
	for (std::size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		const double dx = double(b.x) - a.x, dz = double(b.z) - a.z;
		const double len = std::hypot(dx, dz);
		if (len >= 3.0) {
			const double edge_angle = std::atan2(dz, dx);
			sum_c += len * std::cos(4.0 * edge_angle);
			sum_s += len * std::sin(4.0 * edge_angle);
		}
		twice_area += std::int64_t(a.x) * b.z - std::int64_t(b.x) * a.z;
	}
	if (sum_c == 0.0 && sum_s == 0.0)
		return std::nullopt;
	const double angle = std::atan2(sum_s, sum_c) / 4.0;
	const double deviation = std::abs(angle * 180.0 / std::numbers::pi);
	if (deviation < 1.0 || deviation > 45.0)
		return std::nullopt;
	RoofTentFrame frame{angle, std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
			-std::numeric_limits<double>::max()};
	for (const auto &node : nodes) {
		const auto [u, v] = frame.uv(node.x, node.z);
		frame.u_min = std::min(frame.u_min, u);
		frame.u_max = std::max(frame.u_max, u);
		frame.v_min = std::min(frame.v_min, v);
		frame.v_max = std::max(frame.v_max, v);
	}
	const double rectangle_area =
			(frame.u_max - frame.u_min) * (frame.v_max - frame.v_min);
	const double polygon_area = std::abs(double(twice_area)) / 2.0;
	if (rectangle_area <= 0.0 || polygon_area / rectangle_area < .78)
		return std::nullopt;
	return frame;
}

std::optional<int> rust_roof_peak_cap(const tags_t &tags,
		const std::vector<ProcessedNode> &nodes, RoofType roof_type, int width,
		int length, int building_height, double scale_factor,
		std::optional<bool> preferred_ridge_along_x)
{
	auto tag_number = [&](const char *key, const char *suffix) -> std::optional<double> {
		auto it = tags.find(key);
		if (it == tags.end())
			return std::nullopt;
		std::string value = it->second;
		auto trim = [&]() {
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.front())))
				value.erase(value.begin());
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.back())))
				value.pop_back();
		};
		trim();
		const std::string suffix_str(suffix);
		if (!suffix_str.empty() && value.size() >= suffix_str.size() &&
				value.compare(value.size() - suffix_str.size(), suffix_str.size(),
						suffix_str) == 0)
			value.resize(value.size() - suffix_str.size());
		trim();
		const auto parsed = strict_parse::f64(value);
		return parsed && *parsed > 0.0 ? parsed : std::nullopt;
	};
	if (const auto height_m = tag_number("roof:height", "m")) {
		const int rounded = static_cast<int>(std::lround(*height_m));
		return std::max(1, static_cast<int>(std::floor(rounded * scale_factor)));
	}
	if (roof_type == RoofType::Flat || roof_type == RoofType::Dome ||
			roof_type == RoofType::Cone || roof_type == RoofType::Onion)
		return std::nullopt;
	const auto angle = tag_number("roof:angle", "\xC2\xB0");
	if (!angle || *angle >= 90.0)
		return std::nullopt;
	const double w = std::max(1, width + 1);
	const double l = std::max(1, length + 1);
	double run = 0.0;
	if (roof_type == RoofType::Skillion) {
		if (auto it = tags.find("roof:direction"); it != tags.end()) {
			if (const auto degrees = parse_roof_direction_degrees(it->second)) {
				const double radians = *degrees * std::numbers::pi / 180.0;
				run = std::abs(std::sin(radians)) * w + std::abs(std::cos(radians)) * l;
			}
		}
		if (run == 0.0)
			run = std::min(w, l);
	} else if (roof_type == RoofType::Hipped || roof_type == RoofType::Mansard ||
			   roof_type == RoofType::Pyramidal) {
		run = std::min(w, l) / 2.0;
	} else {
		const auto tent = roof_tent_frame(nodes);
		const auto u_axis = tent ? tent->u_axis() : std::pair{1.0, 0.0};
		const auto v_axis = tent ? tent->v_axis() : std::pair{0.0, 1.0};
		const double u_len = tent ? tent->u_max - tent->u_min : width;
		const double v_len = tent ? tent->v_max - tent->v_min : length;
		const bool u_is_longer = u_len >= v_len;
		bool ridge_along_x = preferred_ridge_along_x.value_or(u_is_longer);
		bool mapped_direction = false;
		if (auto direction = tags.find("roof:direction"); direction != tags.end()) {
			if (const auto degrees = parse_roof_direction_degrees(direction->second)) {
				const double radians = *degrees * std::numbers::pi / 180.0;
				const double dx = std::sin(radians), dz = -std::cos(radians);
				ridge_along_x = std::abs(dx * v_axis.first + dz * v_axis.second) >=
								std::abs(dx * u_axis.first + dz * u_axis.second);
				mapped_direction = true;
			}
		}
		if (!mapped_direction) {
			if (auto orientation = tags.find("roof:orientation");
					orientation != tags.end()) {
				std::string normalized = orientation->second;
				std::transform(normalized.begin(), normalized.end(), normalized.begin(),
						[](unsigned char c) {
							return static_cast<char>(std::tolower(c));
						});
				if (normalized == "along")
					ridge_along_x = u_is_longer;
				else if (normalized == "across")
					ridge_along_x = !u_is_longer;
			}
		}
		const double span = tent ? (ridge_along_x ? v_len + 1.0 : u_len + 1.0)
								 : (ridge_along_x ? l : w);
		run = span / 2.0;
	}
	const int rise = std::max(
			1, int(std::lround(std::tan(*angle * std::numbers::pi / 180.0) * run)));
	return std::min(rise, std::max(building_height, 3));
}

std::optional<double> parse_meter_tag(const tags_t &tags, const char *key);

std::optional<int32_t> roof_mansard_dormer_band(
		const ProcessedWay &element, int32_t base_height, RoofType roof_type)
{
	if (roof_type != RoofType::Mansard)
		return std::nullopt;
	int steep_height = 4;
	if (const auto levels = parse_meter_tag(element.tags, "roof:levels"))
		steep_height = std::clamp(int(std::lround(*levels * 3.0)), 3, 6);
	return base_height + (steep_height + 1) / 2 + 1;
}

void place_roof_dormer_windows(WorldEditor &editor,
		const std::vector<std::pair<int32_t, int32_t>> &floor_area,
		const std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash>
				&roof_heights,
		int32_t base_height, int32_t abs_terrain_offset, std::uint64_t style_seed,
		Block roof_block, bool enabled, std::pair<int, int> parallel_to_ridge,
		std::optional<int32_t> mansard_band = std::nullopt)
{
	if (!enabled || floor_area.empty())
		return;
	using Cell = std::pair<int32_t, int32_t>;
	std::unordered_set<Cell, pair_hash> footprint(floor_area.begin(), floor_area.end());
	const auto [px, pz] = parallel_to_ridge;
	const Cell perp_unit = px != 0 ? Cell{0, 1} : Cell{1, 0};
	auto scan = [&](int32_t x, int32_t z, int dx, int dz) {
		int distance = 0;
		while (footprint.contains({x + (distance + 1) * dx, z + (distance + 1) * dz}))
			++distance;
		return distance;
	};

	std::vector<std::tuple<int32_t, int32_t, int32_t>> candidates;
	for (const auto &[x, z] : floor_area) {
		const auto h_it = roof_heights.find({x, z});
		if (h_it == roof_heights.end())
			continue;
		const int h = h_it->second;
		if (h != base_height + 2 && h != base_height + 3 &&
				(!mansard_band || h != *mansard_band))
			continue;
		const int dm = scan(x, z, -perp_unit.first, -perp_unit.second);
		const int dp = scan(x, z, perp_unit.first, perp_unit.second);
		if (std::max(dm, dp) < 4)
			continue;
		const Cell left{x - px, z - pz}, right{x + px, z + pz};
		const auto left_it = roof_heights.find(left);
		const auto right_it = roof_heights.find(right);
		if (left_it == roof_heights.end() || right_it == roof_heights.end() ||
				left_it->second != h || right_it->second != h)
			continue;
		if (!footprint.contains({x - 2 * px, z - 2 * pz}) ||
				!footprint.contains({x + 2 * px, z + 2 * pz}))
			continue;
		const int inward = dm <= dp ? 1 : -1;
		const Cell above{x + pz * inward, z + px * inward};
		const auto above_it = roof_heights.find(above);
		if (above_it == roof_heights.end() || above_it->second <= h)
			continue;
		candidates.emplace_back(x, z, h);
	}
	if (candidates.empty())
		return;
	std::sort(candidates.begin(), candidates.end());
	const std::size_t target_count =
			std::clamp<std::size_t>(candidates.size() / 12, 1, 3);
	auto rng = element_rng(style_seed ^ 0x000D04E4DEC0ULL);
	std::vector<std::tuple<int32_t, int32_t, int32_t>> chosen;
	for (std::size_t attempts = 0;
			chosen.size() < target_count && attempts < candidates.size() * 4;
			++attempts) {
		const auto pos =
				candidates[rng.uniform(static_cast<std::uint32_t>(candidates.size()))];
		const auto [x, z, h] = pos;
		const bool too_close =
				std::any_of(chosen.begin(), chosen.end(), [&](const auto &other) {
					const auto [cx, cz, unused_h] = other;
					(void)unused_h;
					return std::abs((cx - x) * px + (cz - z) * pz) < 4;
				});
		if (!too_close)
			chosen.push_back(pos);
	}

	const StairFacing left_facing = px != 0 ? StairFacing::East : StairFacing::South;
	const StairFacing right_facing = px != 0 ? StairFacing::West : StairFacing::North;
	const Block stair_material = get_stair_block_for_material(roof_block);
	const auto replace_any = std::optional<std::vector<Block>>(std::vector<Block>{});
	auto put = [&](Block block, int x, int y, int z) {
		editor.set_block_absolute(block, x, y, z, std::nullopt, replace_any);
	};
	for (const auto &[x, z, h] : chosen) {
		const int lower_y = h - 1 + abs_terrain_offset;
		const int face_y = h + abs_terrain_offset;
		const int cap_y = h + 1 + abs_terrain_offset;
		const int dm = scan(x, z, -perp_unit.first, -perp_unit.second);
		const int dp = scan(x, z, perp_unit.first, perp_unit.second);
		const int ox = perp_unit.first * (dm <= dp ? -1 : 1);
		const int oz = perp_unit.second * (dm <= dp ? -1 : 1);
		const Cell extension{x + ox, z + oz};
		const bool extension_inside = footprint.contains(extension);
		for (int along : {-1, 0, 1}) {
			put(roof_block, x + along * px, lower_y, z + along * pz);
			put(roof_block, x + along * px, face_y, z + along * pz);
			if (extension_inside) {
				put(roof_block, extension.first + along * px, lower_y,
						extension.second + along * pz);
				put(roof_block, extension.first + along * px, face_y,
						extension.second + along * pz);
			}
		}
		put(LIGHT_GRAY_STAINED_GLASS, x, face_y, z);
		if (extension_inside)
			put(LIGHT_GRAY_STAINED_GLASS, extension.first, face_y, extension.second);
		for (int along : {-1, 1})
			editor.set_block_with_properties_absolute(
					create_stair_with_properties(stair_material,
							along < 0 ? left_facing : right_facing, StairShape::Straight),
					x + along * px, cap_y, z + along * pz, std::nullopt, replace_any);
		put(roof_block, x, cap_y, z);
		if (extension_inside) {
			for (int along : {-1, 1})
				editor.set_block_with_properties_absolute(
						create_stair_with_properties(stair_material,
								along < 0 ? left_facing : right_facing,
								StairShape::Straight),
						extension.first + along * px, cap_y,
						extension.second + along * pz, std::nullopt, replace_any);
			put(roof_block, extension.first, cap_y, extension.second);
		}
	}
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
	auto rng = element_rng(group_seed ^ 0x48E16F001EA50001ULL);
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
	return parse_meter_value(it->second);
}

std::optional<double> parse_building_height_tag(const tags_t &tags)
{
	const auto it = tags.find("height");
	if (it == tags.end())
		return std::nullopt;
	const auto height = parse_meter_value(it->second);
	if (!height || !std::isfinite(*height) || *height > 1000.0)
		return std::nullopt;
	return height;
}

std::pair<int, bool> calculate_building_height(const ProcessedWay &element,
		const std::string &building_type, int min_level, double scale,
		std::optional<int> relation_levels, int floor_cycle, std::size_t footprint,
		std::uint64_t group_seed)
{
	constexpr int ground_floor_bonus = 2;
	int height = std::max(3, int(10.0 * scale));
	bool tall = false, has_source = false, explicit_height = false;
	if (auto it = element.tags.find("building:levels"); it != element.tags.end()) {
		auto level_value = it->second;
		while (!level_value.empty() &&
				std::isspace(static_cast<unsigned char>(level_value.front())))
			level_value.erase(level_value.begin());
		while (!level_value.empty() &&
				std::isspace(static_cast<unsigned char>(level_value.back())))
			level_value.pop_back();
		if (const auto levels_value = strict_parse::f64(level_value)) {
			const double levels = *levels_value, lev = levels - min_level;
			if (lev >= 1) {
				height = std::max(
						3, int((lev * floor_cycle + (min_level > 0 ? 0 : 2)) * scale));
				has_source = true;
				tall = levels > 7;
			}
		}
	}
	if (const auto tagged = parse_building_height_tag(element.tags)) {
		explicit_height = has_source = true;
		double effective = *tagged;
		const bool is_part = element.tags.contains("building:part");
		bool elevated = false;
		if (element.tags.contains("min_height")) {
			// Rust treats a present but malformed min_height as 0.0; its
			// presence still takes precedence over building:min_level.
			const double min_height =
					parse_meter_tag(element.tags, "min_height").value_or(0.0);
			elevated = min_height > 0;
			effective = std::max(1.0, effective - min_height);
		} else if (min_level > 0) {
			elevated = true;
			effective = std::max(
					1.0, effective - (min_level * floor_cycle + ground_floor_bonus));
		}
		if (element.tags.get("roof:shape") != "flat")
			if (const auto roof_height = parse_meter_tag(element.tags, "roof:height"))
				effective = std::max(effective - std::max(0.0, *roof_height),
						std::min(is_part ? 1.0 : 3.0, effective));
		height = std::max((elevated || is_part) ? 1 : 3, int(effective * scale));
		// Rust only promotes tall buildings here; a prior building:levels value
		// above seven remains tall even when a shorter explicit height is given.
		if (*tagged > 28)
			tall = true;
	}
	if (!explicit_height && relation_levels) {
		height = std::max(
				3, int(((std::max(1, *relation_levels - min_level) * floor_cycle) +
							   (min_level > 0 ? 0 : ground_floor_bonus)) *
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
					3, int((levels * floor_cycle + (min_level > 0 ? 0 : 2)) * scale));
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

int terrain_base_level(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &footprint, int fallback)
{
	std::optional<int> highest;
	auto include = [&](int x, int z) {
		if (const auto level = editor.terrain_level(x, z))
			highest = highest ? std::max(*highest, *level) : *level;
	};
	for (const auto &node : element.nodes)
		include(node.x, node.z);
	// Outline samples alone can miss a local high point enclosed by the way.
	// Anchor the shell to the highest terrain it actually covers so its lower
	// storeys are not buried by the later ground pass.
	for (const auto &[x, z] : footprint)
		include(x, z);
	return highest.value_or(fallback);
}

void generate_roof_only_structure(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &cached_floor_area, const Args &args,
		std::uint64_t seed)
{
	const double scale_factor = args.scale;
	const bool terrain_enabled = args.terrain_enabled();
	const int abs_terrain_offset = !terrain_enabled ? args.ground_level : 0;
	const std::string building_type =
			element.tags.contains("building")
					? element.tags.get("building")
					: (element.tags.contains("building:part")
									  ? element.tags.get("building:part")
									  : "roof");
	const int floor_cycle = floor_cycle_for(building_type, element.tags);

	int min_level_offset = 0;
	if (auto it = element.tags.find("min_height"); it != element.tags.end()) {
		min_level_offset = static_cast<int>(parse_tag_meters(it->second) * scale_factor);
	} else if (auto it = element.tags.find("building:min_level");
			   it != element.tags.end()) {
		if (auto level = parse_i32_tag(element.tags, "building:min_level");
				level && *level > 0)
			min_level_offset = scaled_blocks(*level * floor_cycle + 2, scale_factor);
	} else if (auto it = element.tags.find("layer"); it != element.tags.end()) {
		if (auto layer = parse_i32_tag(element.tags, "layer"); layer && *layer > 0)
			min_level_offset = scaled_blocks(*layer * floor_cycle, scale_factor);
	}

	int start_y_offset = min_level_offset;
	if (terrain_enabled)
		start_y_offset = terrain_base_level(
								 editor, element, cached_floor_area, args.ground_level) +
						 min_level_offset;

	int roof_thickness = 5;
	if (const auto height = parse_building_height_tag(element.tags)) {
		const int total = static_cast<int>(*height * scale_factor);
		roof_thickness =
				element.tags.contains("min_height")
						? std::max(min_level_offset > 0 ? 1 : 3, total - min_level_offset)
						: std::max(3, total);
	} else if (auto levels = parse_i32_tag(element.tags, "building:levels")) {
		roof_thickness =
				std::max(3, scaled_blocks(*levels * floor_cycle + 2, scale_factor));
	}

	auto rng = element_rng(seed);
	const Block roof_block =
			roof_block_from_tags(element, rng).value_or(STONE_BRICK_SLAB);

	const RoofType roof_type = [&]() {
		if (auto it = element.tags.find("roof:shape"); it != element.tags.end())
			return parse_roof_type(it->second);
		return RoofType::Flat;
	}();

	if ((roof_type == RoofType::Dome || roof_type == RoofType::Hipped ||
				roof_type == RoofType::Pyramidal || roof_type == RoofType::Cone ||
				roof_type == RoofType::Onion || roof_type == RoofType::Round) &&
			!cached_floor_area.empty()) {
		const int springing = start_y_offset + roof_thickness;
		for (const auto &node : element.nodes) {
			const int pillar_base =
					terrain_enabled ? editor.get_ground_level(node.x, node.z) : 0;
			for (int y = pillar_base + 1; y < springing; ++y)
				editor.set_block_absolute(
						COBBLESTONE_WALL, node.x, y + abs_terrain_offset, node.z);
		}
		// Rust constructs this RoofConfig directly with peak_cap=None and the
		// material already selected above. Keep roof tags from being interpreted
		// a second time by the ordinary building-roof dispatcher.
		ProcessedWay shaped_roof = element;
		for (const char *key :
				{"roof:height", "roof:angle", "roof:material", "roof:colour"})
			shaped_roof.tags.erase(key);
		const int shaped_roof_height = std::max(roof_thickness, 3);
		generate_roof(editor, shaped_roof, springing - shaped_roof_height - 1,
				shaped_roof_height, roof_block, roof_block, roof_block,
				std::optional<Block>(roof_block), std::nullopt, element.id, false, false,
				std::nullopt, roof_type, cached_floor_area, abs_terrain_offset,
				args.scale);
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
				terrain_enabled ? editor.get_ground_level(node.x, node.z) : 0;
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
		const FloodFillCache &flood_fill_cache,
		const std::optional<std::chrono::milliseconds> &floodfill_timeout);

void generate_rooftop_systems(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &area, BuildingCategory category,
		BuildingCondition condition, RoofType roof_type, int start_y, int building_height,
		int abs_offset, int floor_cycle, Block wall_block, DetailTier detail,
		std::uint64_t seed, bool covered_by_sibling)
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
	const bool sloped = roof_type != RoofType::Flat;
	// Rust rooftop details use an empty blacklist: generated roof equipment is
	// allowed to replace the generated roof overlay beneath it.
	auto force_rooftop_block = [&](Block block, int x, int y, int z) {
		editor.set_block_absolute(block, x, y, z, std::nullopt,
				std::optional<std::vector<Block>>(std::vector<Block>{}));
	};
	if (sloped) {
		// Rust's separate antenna pass is intentionally rare and only applies
		// to normal detached houses on peaked roofs.  Chimneys are handled by
		// the dedicated chimney pass above.
		if (category != BuildingCategory::House ||
				condition != BuildingCondition::Normal || area.size() < 30 ||
				!(roof_type == RoofType::Gabled || roof_type == RoofType::Hipped ||
						roof_type == RoofType::Pyramidal) ||
				!element_rng(element.id ^ 0x04E7E44AULL).random_bool(.05))
			return;
		const std::unordered_set<std::pair<int, int>, PairHash> footprint(
				area.begin(), area.end());
		std::int64_t sum_x = 0, sum_z = 0;
		for (const auto &[x, z] : area) {
			sum_x += x;
			sum_z += z;
		}
		const auto center_x = sum_x / static_cast<std::int64_t>(area.size());
		const auto center_z = sum_z / static_cast<std::int64_t>(area.size());
		std::optional<std::pair<int, int>> best;
		std::int64_t best_distance = std::numeric_limits<std::int64_t>::max();
		for (const auto &p : area)
			if (footprint.contains({p.first - 1, p.second}) &&
					footprint.contains({p.first + 1, p.second}) &&
					footprint.contains({p.first, p.second - 1}) &&
					footprint.contains({p.first, p.second + 1})) {
				const auto dx = static_cast<std::int64_t>(p.first) - center_x;
				const auto dz = static_cast<std::int64_t>(p.second) - center_z;
				const auto distance = dx * dx + dz * dz;
				if (!best || distance < best_distance) {
					best = p;
					best_distance = distance;
				}
			}
		if (!best)
			return;
		const int wall_top = start_y + building_height + abs_offset;
		const auto ridge_y = editor.highest_block_between(
				best->first, best->second, wall_top, wall_top + 96);
		if (!ridge_y)
			return;
		editor.set_block_absolute(LIGHTNING_ROD, best->first, *ridge_y + 1, best->second);
		return;
	}
	if (covered_by_sibling)
		return;

	const bool terrace = element.tags.contains("building:part") && building_height >= 28;
	if (terrace) {
		for (const auto &[x, z] : area)
			if (!interior(x, z))
				force_rooftop_block(STONE_BRICKS, x, roof_y + 1, z);
		std::vector<std::pair<int, int>> terrace_interior;
		for (const auto &[x, z] : area)
			if (interior(x, z))
				terrace_interior.emplace_back(x, z);
		for (const auto &[x, z] : terrace_interior) {
			auto rng = coord_rng(x, z, element.id);
			const auto roll = rng.uniform(100);
			if (roll >= 15)
				continue;
			if (roll <= 2) {
				force_rooftop_block(IRON_BLOCK, x, roof_y + 1, z);
				force_rooftop_block(SMOOTH_STONE_SLAB, x, roof_y + 2, z);
			} else if (roll <= 5) {
				force_rooftop_block(CAULDRON, x, roof_y + 1, z);
				const auto leaf_roll = rng.uniform(3);
				const Block leaf = leaf_roll == 0	? OAK_LEAVES
								   : leaf_roll == 1 ? BIRCH_LEAVES
													: SPRUCE_LEAVES;
				force_rooftop_block(leaf, x, roof_y + 2, z);
			} else if (roll <= 8) {
				force_rooftop_block(OAK_FENCE, x, roof_y + 1, z);
				force_rooftop_block(OAK_SLAB, x, roof_y + 2, z);
			} else if (roll <= 10)
				force_rooftop_block(OAK_STAIRS, x, roof_y + 1, z);
			else if (roll <= 12)
				force_rooftop_block(LIGHTNING_ROD, x, roof_y + 1, z);
			else if (roll == 13)
				force_rooftop_block(CAULDRON, x, roof_y + 1, z);
			else if (roll == 14)
				force_rooftop_block(SEA_LANTERN, x, roof_y + 1, z);
		}
		if (!terrace_interior.empty()) {
			const auto best = std::min_element(terrace_interior.begin(),
					terrace_interior.end(), [&](const auto &a, const auto &b) {
						const auto a_dx = std::int64_t(a.first) - center_x;
						const auto a_dz = std::int64_t(a.second) - center_z;
						const auto b_dx = std::int64_t(b.first) - center_x;
						const auto b_dz = std::int64_t(b.second) - center_z;
						return a_dx * a_dx + a_dz * a_dz < b_dx * b_dx + b_dz * b_dz;
					});
			for (int dy = 0; dy < 6; ++dy)
				force_rooftop_block(
						IRON_BARS, best->first, roof_y + 1 + dy, best->second);
			force_rooftop_block(LIGHTNING_ROD, best->first, roof_y + 7, best->second);
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
	// Match Rust's generate_water_tower: one RNG stream seeded from the
	// element id chooses both whether a tower exists and its roof location.
	auto water_tower_rng = element_rng(element.id ^ 0x3A7E12F000000002ULL);
	if (water_tower_eligible && condition == BuildingCondition::Normal &&
			water_tower_rng.random_bool(.18)) {
		// Rust's roof-area input is sorted before binary-search membership and
		// random selection. Normalize here too so flood-fill traversal order
		// cannot change the chosen tower location.
		auto roof_area = area;
		std::sort(roof_area.begin(), roof_area.end());
		roof_area.erase(std::unique(roof_area.begin(), roof_area.end()), roof_area.end());
		std::vector<std::pair<int, int>> spots;
		for (const auto &[x, z] : roof_area) {
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
			water_tower = spots[water_tower_rng.uniform(
					static_cast<std::uint32_t>(spots.size()))];
			const auto [cx, cz] = *water_tower;
			const int base = roof_y + 1;
			for (const auto &[dx, dz] : {std::pair{-1, -1}, std::pair{1, -1},
						 std::pair{-1, 1}, std::pair{1, 1}})
				for (int h = 0; h < 2; ++h)
					force_rooftop_block(SPRUCE_FENCE, cx + dx, base + h, cz + dz);
			for (int dz = -1; dz <= 1; ++dz)
				for (int dx = -1; dx <= 1; ++dx) {
					for (int h = 2; h < 5; ++h)
						force_rooftop_block(SPRUCE_PLANKS, cx + dx, base + h, cz + dz);
					force_rooftop_block(dx == 0 && dz == 0 ? SPRUCE_PLANKS : SPRUCE_SLAB,
							cx + dx, base + 5, cz + dz);
				}
			force_rooftop_block(SPRUCE_SLAB, cx, base + 6, cz);
		}
	}

	const bool excluded_short_roof =
			category == BuildingCategory::House || category == BuildingCategory::Farm ||
			category == BuildingCategory::Garage || category == BuildingCategory::Shed ||
			category == BuildingCategory::Greenhouse ||
			category == BuildingCategory::Religious;
	const bool standard_equipment =
			!sloped && building_height >= 8 && !excluded_short_roof;
	const bool short_equipment =
			!standard_equipment && !sloped && building_height <= 8 &&
			!excluded_short_roof && condition == BuildingCondition::Normal &&
			detail != DetailTier::Minimal &&
			element_rng(element.id ^ 0xBCB15EED570477F0ULL).random_bool(.10);
	if (!standard_equipment && !short_equipment)
		return;
	std::unordered_set<std::pair<int, int>, PairHash> floor_set;
	floor_set.reserve(area.size());
	for (const auto &[x, z] : area)
		if (footprint.contains({x - 1, z}) && footprint.contains({x + 1, z}) &&
				footprint.contains({x, z - 1}) && footprint.contains({x, z + 1}))
			floor_set.insert({x, z});
	std::vector<std::pair<int, int>> equipment_cells;
	for (const auto &[x, z] : area) {
		const std::array<std::pair<int, int>, 8> neighbors = {
				{{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}, {x - 1, z - 1},
						{x + 1, z + 1}, {x - 1, z + 1}, {x + 1, z - 1}}};
		if (std::all_of(neighbors.begin(), neighbors.end(),
					[&](const auto &p) { return floor_set.contains(p); }))
			equipment_cells.emplace_back(x, z);
	}
	if (equipment_cells.empty())
		return;
	std::unordered_set<std::pair<int, int>, PairHash> used;
	if (water_tower)
		for (int dx = -3; dx <= 3; ++dx)
			for (int dz = -3; dz <= 3; ++dz)
				used.insert({water_tower->first + dx, water_tower->second + dz});
	const std::optional<std::vector<Block>> replace_any{std::vector<Block>{}};
	const int equip_y = roof_y + 1;
	for (const auto &[x, z] : equipment_cells) {
		if (used.contains({x, z}))
			continue;
		auto rng = coord_rng(x, z, element.id ^ 0xE90B375EED001001ULL);
		const auto roll = rng.uniform(1200);
		// Keep the rooftop equipment sparse, matching Rust's 12-in-1200 chance.
		if (roll >= 12)
			continue;
		if (roll <= 2) {
			editor.set_block_absolute(
					IRON_BLOCK, x, equip_y, z, std::nullopt, replace_any);
			editor.set_block_absolute(
					SMOOTH_STONE_SLAB, x, equip_y + 1, z, std::nullopt, replace_any);
			used.insert({x, z});
		} else if (roll <= 5) {
			const std::array<std::pair<int, int>, 4> offsets = {
					{{0, 0}, {6, 0}, {0, 5}, {6, 5}}};
			std::vector<std::pair<int, int>> panels, box;
			for (int dx = 0; dx < 11; ++dx)
				for (int dz = 0; dz < 9; ++dz)
					box.emplace_back(x + dx, z + dz);
			for (const auto &[ox, oz] : offsets)
				for (int dx = 0; dx < 5; ++dx)
					for (int dz = 0; dz < 4; ++dz)
						panels.emplace_back(x + ox + dx, z + oz + dz);
			const auto fits = [&](const auto &cells) {
				return std::all_of(cells.begin(), cells.end(), [&](const auto &p) {
					return floor_set.contains(p) && !used.contains(p);
				});
			};
			if (fits(box)) {
				for (const auto &[px, pz] : panels)
					editor.set_block_absolute(DAYLIGHT_DETECTOR, px, equip_y, pz,
							std::nullopt, replace_any);
				used.insert(box.begin(), box.end());
			} else {
				std::vector<std::pair<int, int>> single;
				for (int dx = 0; dx < 5; ++dx)
					for (int dz = 0; dz < 4; ++dz)
						single.emplace_back(x + dx, z + dz);
				if (fits(single)) {
					for (const auto &[px, pz] : single) {
						editor.set_block_absolute(DAYLIGHT_DETECTOR, px, equip_y, pz,
								std::nullopt, replace_any);
						used.insert({px, pz});
					}
				} else {
					editor.set_block_absolute(
							DAYLIGHT_DETECTOR, x, equip_y, z, std::nullopt, replace_any);
					used.insert({x, z});
				}
			}
		} else if (roll == 6) {
			editor.set_block_absolute(
					IRON_BARS, x, equip_y, z, std::nullopt, replace_any);
			editor.set_block_absolute(
					IRON_BARS, x, equip_y + 1, z, std::nullopt, replace_any);
			editor.set_block_absolute(
					LIGHTNING_ROD, x, equip_y + 2, z, std::nullopt, replace_any);
			used.insert({x, z});
		} else if (roll <= 8) {
			editor.set_block_absolute(BARREL, x, equip_y, z, std::nullopt, replace_any);
			editor.set_block_absolute(
					CAULDRON, x, equip_y + 1, z, std::nullopt, replace_any);
			used.insert({x, z});
		} else if (roll <= 10) {
			const int stack_height = 2 + static_cast<int>(rng.uniform(2));
			for (int dy = 0; dy < stack_height; ++dy)
				editor.set_block_absolute(
						COBBLESTONE_WALL, x, equip_y + dy, z, std::nullopt, replace_any);
			used.insert({x, z});
		} else {
			const std::array<std::pair<int, int>, 4> box = {
					{{x, z}, {x + 1, z}, {x, z + 1}, {x + 1, z + 1}}};
			const bool fits = std::all_of(box.begin(), box.end(), [&](const auto &p) {
				return floor_set.contains(p) && !used.contains(p);
			});
			if (fits)
				for (const auto &[px, pz] : box) {
					editor.set_block_absolute(
							STONE_BRICKS, px, equip_y, pz, std::nullopt, replace_any);
					editor.set_block_absolute(
							STONE_BRICKS, px, equip_y + 1, pz, std::nullopt, replace_any);
					editor.set_block_absolute(STONE_BRICK_SLAB, px, equip_y + 2, pz,
							std::nullopt, replace_any);
					used.insert({px, pz});
				}
		}
	}
}
// Rust adds a deterministic cap/edge treatment to plain flat roofs.
void generate_flat_roof_edge_variation(WorldEditor &editor, const ProcessedWay &element,
		int start_y, int building_height, int abs_offset, Block wall_block,
		Block accent_block)
{
	if (element.nodes.empty())
		return;
	auto rng = element_rng(element.id);
	if (!rng.random_bool(.55))
		return;
	const unsigned variation = rng.uniform(3);
	const int y = start_y + building_height + abs_offset + 2;
	std::optional<std::pair<int, int>> previous_node;
	for (const auto &node : element.nodes) {
		if (previous_node) {
			const auto [x1, z1] = *previous_node;
			const auto [x2, z2] = std::pair{node.x, node.z};
			for (const auto &[x, unused_y, z] :
					bresenham_line(x1, start_y, z1, x2, start_y, z2)) {
				const Block cap = variation == 0 ? wall_block
								  : variation == 1
										  ? get_slab_block_for_material(wall_block)
										  : accent_block;
				editor.set_block_absolute(cap, x, y, z, std::vector<Block>{AIR});
			}
		}
		previous_node = std::pair{node.x, node.z};
	}
}

bool short_flat_parapet_excluded(BuildingCategory category)
{
	return category == BuildingCategory::House || category == BuildingCategory::Farm ||
		   category == BuildingCategory::Garage || category == BuildingCategory::Shed ||
		   category == BuildingCategory::Greenhouse ||
		   category == BuildingCategory::Religious;
}

bool short_flat_parapet_for(std::uint64_t element_id, RoofType roof_type,
		int building_height, BuildingCategory category, BuildingCondition condition)
{
	if (roof_type != RoofType::Flat || building_height > 8 ||
			short_flat_parapet_excluded(category) ||
			condition != BuildingCondition::Normal)
		return false;
	return element_rng(element_id ^ 0x534FF1544AAEE0F0ULL).random_bool(.75);
}

bool category_has_parapet_preset(BuildingCategory category)
{
	switch (category) {
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
	case BuildingCategory::Hotel:
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
	case BuildingCategory::TallBuilding:
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
	case BuildingCategory::GridSkyscraper:
	case BuildingCategory::ContemporarySkyscraper:
	case BuildingCategory::ModernSkyscraper:
	case BuildingCategory::MasonrySkyscraper:
		return true;
	default:
		return false;
	}
}

void generate_parapet(WorldEditor &editor, const ProcessedWay &element, int start_y,
		int building_height, int abs_offset, Block wall_block, Block accent_block,
		BuildingCategory category)
{
	if (element.nodes.empty())
		return;
	const int parapet_y = start_y + building_height + abs_offset + 2;
	const Block wall_piece = get_wall_piece_for_material(wall_block);
	std::unordered_set<std::pair<int, int>, PairHash> placed;
	auto place_ring = [&](auto &&place) {
		for (std::size_t i = 1; i < element.nodes.size(); ++i) {
			const auto &a = element.nodes[i - 1];
			const auto &b = element.nodes[i];
			for (const auto &[x, unused_y, z] :
					bresenham_line(a.x, start_y, a.z, b.x, start_y, b.z))
				if (placed.insert({x, z}).second)
					place(x, z);
		}
	};
	place_ring([&](int x, int z) {
		const Block block = building_height >= 16 ? wall_block : wall_piece;
		editor.set_block_absolute(block, x, parapet_y, z, std::vector<Block>{AIR});
	});
	if (category != BuildingCategory::ModernSkyscraper)
		return;
	const BlockWithProperties cap{
			get_slab_block_for_material(accent_block), {{"type", "top"}}};
	placed.clear();
	place_ring([&](int x, int z) {
		editor.set_block_with_properties_absolute(
				cap, x, parapet_y + 1, z, std::vector<Block>{AIR}, std::nullopt);
	});
	std::unordered_set<std::pair<int, int>, PairHash> corners;
	for (const auto &node : element.nodes)
		corners.insert({node.x, node.z});
	for (const auto &[x, z] : corners)
		editor.set_block_absolute(
				accent_block, x, parapet_y + 1, z, std::nullopt, std::vector<Block>{});
}

// Rust's gabled profile extends only its two slope-side eaves, using stairs.
void place_gabled_eave_overhang(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &area, int base_y, int offset,
		Block roof_block, bool ridge_runs_along_x,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_cells = nullptr)
{
	std::unordered_set<std::pair<int, int>, PairHash> footprint(area.begin(), area.end());
	const int stair_y = base_y + offset;
	const int outer_y = stair_y - 1;
	const Block stair_material = get_stair_block_for_material(roof_block);
	const std::array<std::tuple<int, int, StairFacing>, 2> dirs =
			ridge_runs_along_x
					? std::array<std::tuple<int, int, StairFacing>,
							  2>{{{0, -1, StairFacing::South},
							  {0, 1, StairFacing::North}}}
					: std::array<std::tuple<int, int, StairFacing>, 2>{
							  {{-1, 0, StairFacing::East}, {1, 0, StairFacing::West}}};
	std::unordered_map<std::pair<int, int>, StairFacing, PairHash> inner, outer;
	for (const auto &[x, z] : area) {
		if (eave_cells && !eave_cells->contains({x, z}))
			continue;
		for (const auto &[dx, dz, facing] : dirs) {
			const std::pair<int, int> n1{x + dx, z + dz};
			const std::pair<int, int> n2{x + 2 * dx, z + 2 * dz};
			if (footprint.contains(n1))
				continue;
			inner.try_emplace(n1, facing);
			if (!footprint.contains(n2))
				outer.try_emplace(n2, facing);
		}
	}
	for (const auto &[cell, facing] : inner)
		editor.set_block_with_properties_absolute(
				create_stair_with_properties(
						stair_material, facing, StairShape::Straight),
				cell.first, stair_y, cell.second, nullptr, nullptr);
	for (const auto &[cell, facing] : outer)
		if (!inner.contains(cell))
			editor.set_block_with_properties_absolute(
					create_stair_with_properties(
							stair_material, facing, StairShape::Straight),
					cell.first, outer_y, cell.second, nullptr, nullptr);
}

void generate_gabled_roof_profile(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &area,
		const std::vector<ProcessedNode> &nodes, const tags_t &tags, int base_height,
		int abs_terrain_offset, int building_height, RoofType roof_type, bool steep_gable,
		std::optional<bool> preferred_ridge_along_x, std::optional<int> peak_cap,
		std::uint64_t style_seed, bool add_dormers, Block roof_block)
{
	if (area.empty())
		return;
	const bool gambrel = roof_type == RoofType::Gambrel;
	const bool half_hipped = roof_type == RoofType::HalfHipped;
	const bool round = roof_type == RoofType::Round;
	const auto tent = roof_tent_frame(nodes);
	const std::unordered_set<std::pair<int, int>, PairHash> footprint(
			area.begin(), area.end());
	auto scan_dir = [&](int x, int z, int dx, int dz) {
		int distance = 0;
		while (footprint.contains({x + (distance + 1) * dx, z + (distance + 1) * dz}))
			++distance;
		return distance;
	};
	int min_x = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int min_z = std::numeric_limits<int>::max();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &[x, z] : area) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const int width = max_x - min_x;
	const int length = max_z - min_z;
	const double u_len = tent ? tent->u_max - tent->u_min : width;
	const double v_len = tent ? tent->v_max - tent->v_min : length;
	const bool u_is_longer = u_len >= v_len;
	const auto u_axis = tent ? tent->u_axis() : std::pair{1.0, 0.0};
	const auto v_axis = tent ? tent->v_axis() : std::pair{0.0, 1.0};
	bool ridge_runs_along_x = preferred_ridge_along_x.value_or(u_is_longer);
	bool mapped_direction = false;
	if (auto direction = tags.find("roof:direction"); direction != tags.end()) {
		if (const auto degrees = parse_roof_direction_degrees(direction->second)) {
			const double radians = *degrees * std::numbers::pi / 180.0;
			const double dx = std::sin(radians), dz = -std::cos(radians);
			ridge_runs_along_x = std::abs(dx * v_axis.first + dz * v_axis.second) >=
								 std::abs(dx * u_axis.first + dz * u_axis.second);
			mapped_direction = true;
		}
	}
	if (!mapped_direction) {
		if (auto orientation = tags.find("roof:orientation"); orientation != tags.end()) {
			std::string normalized = orientation->second;
			std::transform(normalized.begin(), normalized.end(), normalized.begin(),
					[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			if (normalized == "along")
				ridge_runs_along_x = u_is_longer;
			else if (normalized == "across")
				ridge_runs_along_x = !u_is_longer;
		}
	}
	const double cap_factor = steep_gable && roof_type == RoofType::Gabled ? .9 : .6;
	const int wall_cap = peak_cap.value_or(
			std::max(1, int(std::lround(building_height * cap_factor))));
	struct CellData
	{
		int dist_to_edge;
		int local_half;
		int perp_reach;
		int dm_along;
		int dp_along;
		int scan_perp_min;
		int dm_perp;
		int dp_perp;
	};
	std::unordered_map<std::pair<int, int>, CellData, PairHash> cell_data;
	std::unordered_map<std::pair<int, int>, int, PairHash> roof_heights;
	cell_data.reserve(area.size() * 2);
	roof_heights.reserve(area.size() * 2);
	int max_perp_half = 0;
	for (const auto &[x, z] : area) {
		const int sm_z = scan_dir(x, z, 0, -1), sp_z = scan_dir(x, z, 0, 1);
		const int sm_x = scan_dir(x, z, -1, 0), sp_x = scan_dir(x, z, 1, 0);
		int dm_x = sm_x, dp_x = sp_x, dm_z = sm_z, dp_z = sp_z;
		if (tent) {
			const auto [u, v] = tent->uv(x, z);
			dm_x = int(std::floor(std::max(0.0, u - tent->u_min)));
			dp_x = int(std::floor(std::max(0.0, tent->u_max - u)));
			dm_z = int(std::floor(std::max(0.0, v - tent->v_min)));
			dp_z = int(std::floor(std::max(0.0, tent->v_max - v)));
		}
		const int dm_perp = ridge_runs_along_x ? dm_z : dm_x;
		const int dp_perp = ridge_runs_along_x ? dp_z : dp_x;
		const int dm_along = ridge_runs_along_x ? sm_x : sm_z;
		const int dp_along = ridge_runs_along_x ? sp_x : sp_z;
		const int scan_perp_min =
				tent ? 10000
					 : (ridge_runs_along_x ? std::min(sm_z, sp_z) : std::min(sm_x, sp_x));
		const int half_x = (dm_x + dp_x + 1) / 2;
		const int half_z = (dm_z + dp_z + 1) / 2;
		const int local_half = std::min(half_x, half_z);
		const int perp_half = (dm_perp + dp_perp + 1) / 2;
		max_perp_half = std::max(max_perp_half, perp_half);
		cell_data[{x, z}] = {std::min(dm_perp, dp_perp), local_half,
				(dm_perp + dp_perp) / 2, dm_along, dp_along, scan_perp_min, dm_perp,
				dp_perp};
	}
	const bool use_half_pitch =
			!peak_cap && !gambrel && !round && max_perp_half - wall_cap >= 4;
	const std::optional<int> stated_pitch = gambrel ? std::nullopt : peak_cap;
	for (const auto &[x, z] : area) {
		const auto &cell = cell_data.at({x, z});
		int boost;
		int capped_boost;
		int feather;
		if (round) {
			const int cap = peak_cap.value_or(std::min(
					wall_cap, std::max(1, int(std::lround(cell.local_half * .85)))));
			const double reach = std::max(1, cell.perp_reach);
			const double t = std::clamp(1.0 - cell.dist_to_edge / reach, 0.0, 1.0);
			boost = int(std::lround(cap * std::sqrt(1.0 - t * t)));
			capped_boost = std::max(1, cap);
			feather = std::numeric_limits<int>::max() / 2;
		} else if (stated_pitch) {
			const double pitch = double(*stated_pitch) / std::max(1, cell.perp_reach);
			boost = int(std::lround(cell.dist_to_edge * pitch));
			capped_boost = std::clamp(int(std::lround(cell.local_half * pitch)), 1,
					std::max(1, *stated_pitch));
			feather = int(std::lround(cell.scan_perp_min * pitch));
		} else {
			const int slope_dist =
					use_half_pitch ? (cell.dist_to_edge + 1) / 2 : cell.dist_to_edge;
			boost = gambrel ? 2 * std::min(slope_dist, 2) + std::max(0, slope_dist - 2)
							: slope_dist;
			capped_boost = std::min(
					std::max(1, int(std::lround(cell.local_half * .85))), wall_cap);
			feather = cell.scan_perp_min;
		}
		int height = std::min(base_height + boost, base_height + capped_boost) + 1;
		height = std::min(height, base_height + feather + 1);
		if (half_hipped) {
			const int hip_start = std::max(wall_cap / 2, 2);
			height = std::min(height,
					base_height + hip_start + std::min(cell.dm_along, cell.dp_along) + 1);
		}
		roof_heights[{x, z}] = height;
	}
	const std::pair<int, int> along =
			ridge_runs_along_x ? std::pair{1, 0} : std::pair{0, 1};
	const std::pair<int, int> across =
			ridge_runs_along_x ? std::pair{0, 1} : std::pair{1, 0};
	const bool smooth = !tent || std::abs(tent->angle * 180.0 / std::numbers::pi) <= 12.0;
	if (smooth) {
		auto smoothed = roof_heights;
		for (const auto &[cell, height] : roof_heights) {
			const auto [x, z] = cell;
			auto before = roof_heights.find({x - along.first, z - along.second});
			auto after = roof_heights.find({x + along.first, z + along.second});
			if (before != roof_heights.end() && after != roof_heights.end()) {
				std::array<int, 3> values{before->second, height, after->second};
				std::sort(values.begin(), values.end());
				smoothed[cell] = values[1];
			}
		}
		roof_heights.swap(smoothed);
	}
	const Block stair_material = get_stair_block_for_material(roof_block);
	const auto across_axis =
			tent ? (ridge_runs_along_x ? tent->v_axis() : tent->u_axis())
				 : (ridge_runs_along_x ? std::pair{0.0, 1.0} : std::pair{1.0, 0.0});
	auto cardinal_for_vector = [](double dx, double dz) {
		int direction =
				int(std::llround(std::atan2(dx, -dz) * 180.0 / std::numbers::pi / 90.0));
		direction = (direction % 4 + 4) % 4;
		switch (direction) {
		case 0:
			return StairFacing::North;
		case 1:
			return StairFacing::East;
		case 2:
			return StairFacing::South;
		default:
			return StairFacing::West;
		}
	};
	const StairFacing toward_plus =
			cardinal_for_vector(across_axis.first, across_axis.second);
	const StairFacing toward_minus =
			cardinal_for_vector(-across_axis.first, -across_axis.second);
	auto slope_stair = [&](int x, int z) {
		const auto &cell = cell_data.at({x, z});
		const bool nearer_minus = cell.dm_perp <= cell.dp_perp;
		const StairFacing facing = nearer_minus ? toward_plus : toward_minus;
		return create_stair_with_properties(stair_material, facing, StairShape::Straight);
	};
	auto eave_cell = [&](int x, int z) {
		if (tent)
			return cell_data.at({x, z}).dist_to_edge == 0 &&
				   (!footprint.contains({x - 1, z}) || !footprint.contains({x + 1, z}) ||
						   !footprint.contains({x, z - 1}) ||
						   !footprint.contains({x, z + 1}));
		return ridge_runs_along_x ? (!footprint.contains({x, z - 1}) ||
											!footprint.contains({x, z + 1}))
								  : (!footprint.contains({x - 1, z}) ||
											!footprint.contains({x + 1, z}));
	};
	for (const auto &[x, z] : area) {
		const int height = roof_heights.at({x, z});
		if (eave_cell(x, z)) {
			editor.set_block_absolute(
					roof_block, x, base_height + abs_terrain_offset, z, nullptr, nullptr);
			editor.set_block_with_properties_absolute(slope_stair(x, z), x,
					base_height + 1 + abs_terrain_offset, z, nullptr, nullptr);
			continue;
		}
		bool has_lower = false, steep_drop = false;
		for (const auto &[dx, dz] : {std::pair{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
			auto neighbor = roof_heights.find({x + dx, z + dz});
			if (neighbor == roof_heights.end())
				continue;
			has_lower |= neighbor->second < height;
			steep_drop |= height - neighbor->second > 2;
		}
		std::optional<BlockWithProperties> end_stair;
		if (half_hipped) {
			const auto lower = [&](int nx, int nz) {
				auto it = roof_heights.find({nx, nz});
				return it != roof_heights.end() && it->second < height;
			};
			const bool along_minus =
					ridge_runs_along_x ? lower(x - 1, z) : lower(x, z - 1);
			const bool along_plus =
					ridge_runs_along_x ? lower(x + 1, z) : lower(x, z + 1);
			const bool across_minus =
					ridge_runs_along_x ? lower(x, z - 1) : lower(x - 1, z);
			const bool across_plus =
					ridge_runs_along_x ? lower(x, z + 1) : lower(x + 1, z);
			if (!across_minus && !across_plus && along_minus != along_plus) {
				const StairFacing facing =
						ridge_runs_along_x
								? (along_minus ? StairFacing::East : StairFacing::West)
								: (along_minus ? StairFacing::South : StairFacing::North);
				end_stair = create_stair_with_properties(
						stair_material, facing, StairShape::Straight);
			}
		}
		for (int y = base_height; y <= height; ++y) {
			if (y == height && has_lower && !steep_drop)
				editor.set_block_with_properties_absolute(
						end_stair.value_or(slope_stair(x, z)), x, y + abs_terrain_offset,
						z, nullptr, nullptr);
			else
				editor.set_block_absolute(
						roof_block, x, y + abs_terrain_offset, z, nullptr, nullptr);
		}
	}
	if (!half_hipped) {
		for (const auto &[x, z] : area) {
			if (eave_cell(x, z))
				continue;
			const auto &cell = cell_data.at({x, z});
			const int height = roof_heights.at({x, z});
			const auto across_minus =
					roof_heights.find({x - across.first, z - across.second});
			const auto across_plus =
					roof_heights.find({x + across.first, z + across.second});
			const bool flat_here =
					(across_minus == roof_heights.end() ||
							across_minus->second == height) &&
					(across_plus == roof_heights.end() || across_plus->second == height);
			if (flat_here)
				continue;
			const std::array<std::pair<int, int>, 2> ends{
					{ridge_runs_along_x ? std::pair{x - 1, z} : std::pair{x, z - 1},
							ridge_runs_along_x ? std::pair{x + 1, z}
											   : std::pair{x, z + 1}}};
			const std::array<int, 2> along_dists{cell.dm_along, cell.dp_along};
			for (int end = 0; end < 2; ++end) {
				if (along_dists[end] != 0 || footprint.contains(ends[end]))
					continue;
				editor.set_block_with_properties_absolute(slope_stair(x, z),
						ends[end].first, height + abs_terrain_offset, ends[end].second,
						nullptr, nullptr);
			}
		}
	}
	std::unordered_set<std::pair<int, int>, PairHash> turned_eaves;
	if (tent)
		for (const auto &[x, z] : area)
			if (eave_cell(x, z))
				turned_eaves.insert({x, z});
	place_gabled_eave_overhang(editor, area, base_height, abs_terrain_offset, roof_block,
			ridge_runs_along_x, tent ? &turned_eaves : nullptr);
	const bool dormers_fit =
			!round && (!tent || std::abs(tent->angle * 180.0 / std::numbers::pi) <= 12.0);
	if (dormers_fit)
		place_roof_dormer_windows(editor, area, roof_heights, base_height,
				abs_terrain_offset, style_seed, roof_block, add_dormers, along);
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
		for (const auto &[dx, dz] :
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
		for (const auto &[dx, dz] :
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

// Sparse exterior downpipes on eligible convex building corners.
void generate_corner_downpipes(WorldEditor &editor, const ProcessedWay &element,
		const building_facade::FacadePlan &facade, BuildingCategory category,
		BuildingCondition condition, bool has_windows, bool is_ground_level, int start_y,
		int building_height, int abs_offset, Block wall_block,
		const CoordinateBitmap *passages, const building_facade::PointSet *preset_shell)
{
	if (element.nodes.size() < 3 || building_height < 10 ||
			condition != BuildingCondition::Normal || !has_windows ||
			!(category == BuildingCategory::House ||
					category == BuildingCategory::Residential ||
					category == BuildingCategory::Commercial ||
					category == BuildingCategory::Hotel ||
					category == BuildingCategory::Historic))
		return;
	auto rng = element_rng(element.id ^ 0xD0DA1290D0FFAA01ULL);
	if (!rng.random_bool(.35))
		return;
	std::int64_t area2 = 0;
	for (std::size_t i = 0; i < element.nodes.size(); ++i) {
		const auto &a = element.nodes[i];
		const auto &b = element.nodes[(i + 1) % element.nodes.size()];
		area2 += std::int64_t(a.x) * b.z - std::int64_t(b.x) * a.z;
	}
	const int outward = area2 > 0 ? -1 : 1;
	std::vector<std::pair<int, int>> vertices;
	vertices.reserve(element.nodes.size());
	for (const auto &node : element.nodes) {
		const std::pair point{node.x, node.z};
		if (vertices.empty() || vertices.back() != point)
			vertices.push_back(point);
	}
	if (vertices.size() > 1 && vertices.front() == vertices.back())
		vertices.pop_back();
	if (vertices.size() < 3)
		return;
	std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> corners;
	for (std::size_t i = 0; i < vertices.size(); ++i) {
		const auto p = vertices[(i + vertices.size() - 1) % vertices.size()];
		const auto c = vertices[i];
		const auto q = vertices[(i + 1) % vertices.size()];
		const std::int64_t cross =
				std::int64_t(c.first - p.first) * (q.second - c.second) -
				std::int64_t(c.second - p.second) * (q.first - c.first);
		if (!cross || (cross > 0 ? 1 : -1) != -outward)
			continue;
		const auto normal = [&](std::pair<int, int> a, std::pair<int, int> b) {
			const int dx = b.first - a.first, dz = b.second - a.second;
			const int nx = -dz * outward, nz = dx * outward;
			if (std::abs(nx) >= std::abs(nz))
				return std::pair{(nx > 0) - (nx < 0), 0};
			return std::pair{0, (nz > 0) - (nz < 0)};
		};
		const auto a = normal(p, c), b = normal(c, q);
		const std::pair diagonal{a.first + b.first, a.second + b.second};
		if (diagonal.first && diagonal.second)
			corners.emplace_back(c, diagonal);
	}
	if (corners.empty())
		return;
	const auto pipe = get_wall_piece_for_material(wall_block);
	const auto start = rng.uniform(static_cast<std::uint32_t>(corners.size()));
	for (std::size_t k = 0; k < 2; ++k) {
		const auto &[point, diagonal] =
				corners[(start + k * corners.size() / 2) % corners.size()];
		const auto [x, z] = point;
		if ((passages && passages->contains(x, z)) || facade.is_party(x, z) ||
				(preset_shell && preset_shell->contains({x, z})) ||
				mapillary::facades::photo_column(x, z, element.id))
			continue;
		const int ox = x + diagonal.first, oz = z + diagonal.second;
		int descent = 0;
		if (is_ground_level) {
			const auto wall_ground = editor.terrain_level(x, z);
			const auto pipe_ground = editor.terrain_level(ox, oz);
			if (wall_ground && pipe_ground)
				descent = std::max(0, start_y - std::min(*wall_ground, *pipe_ground));
			else if (wall_ground)
				descent = std::max(0, start_y - *wall_ground);
			else if (pipe_ground)
				descent = std::max(0, start_y - *pipe_ground);
		}
		for (int h = start_y + 1 - descent; h <= start_y + building_height; ++h)
			editor.set_block_absolute(pipe, ox, h + abs_offset, oz,
					std::optional<std::vector<Block>>{{AIR}});
	}
}

bool is_glass_surface_block(Block block)
{
	return block == GLASS || block == WHITE_STAINED_GLASS ||
		   block == GRAY_STAINED_GLASS || block == LIGHT_GRAY_STAINED_GLASS ||
		   block == BLACK_STAINED_GLASS || block == BROWN_STAINED_GLASS ||
		   block == CYAN_STAINED_GLASS || block == BLUE_STAINED_GLASS ||
		   block == LIGHT_BLUE_STAINED_GLASS;
}

void generate_chimney(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &floor_area, int min_x, int max_x,
		int min_z, int max_z, int roof_base, int abs_offset, std::uint64_t element_id)
{
	if (floor_area.empty())
		return;
	const std::unordered_set<std::pair<int, int>, PairHash> footprint(
			floor_area.begin(), floor_area.end());
	auto near_eave = [&](int x, int z) {
		for (const auto &[dx, dz] :
				{std::pair{-1, 0}, std::pair{1, 0}, std::pair{0, -1}, std::pair{0, 1}})
			if (!footprint.contains({x + dx, z + dz}))
				return false;
		for (const auto &[dx, dz] :
				{std::pair{-2, 0}, std::pair{2, 0}, std::pair{0, -2}, std::pair{0, 2}})
			if (!footprint.contains({x + dx, z + dz}))
				return true;
		return false;
	};
	auto rng = element_rng(element_id);
	const int center_x = (min_x + max_x) / 2;
	const int center_z = (min_z + max_z) / 2;
	const unsigned quadrant = rng.uniform(4);
	std::vector<std::pair<int, int>> candidates;
	for (const auto &[x, z] : floor_area) {
		const bool in_quadrant = quadrant == 0	 ? x < center_x && z < center_z
								 : quadrant == 1 ? x >= center_x && z < center_z
								 : quadrant == 2 ? x < center_x && z >= center_z
												 : x >= center_x && z >= center_z;
		if (in_quadrant && near_eave(x, z))
			candidates.emplace_back(x, z);
	}
	if (candidates.empty())
		for (const auto &[x, z] : floor_area)
			if (near_eave(x, z))
				candidates.emplace_back(x, z);
	std::erase_if(candidates, [&](const auto &p) {
		for (int y = roof_base + 1; y <= roof_base + 3; ++y)
			if (const auto block =
							editor.get_block_absolute(p.first, y + abs_offset, p.second);
					block && is_glass_surface_block(*block))
				return true;
		return false;
	});
	if (candidates.empty())
		return;
	const auto [x, z] =
			candidates[rng.uniform(static_cast<std::uint32_t>(candidates.size()))];
	const std::vector<Block> replace_any;
	for (int y = roof_base + 1; y < roof_base + 5; ++y)
		editor.set_block_absolute(BRICK, x, y + abs_offset, z, std::nullopt, replace_any);
	const Block cap = rng.random_bool(.4) ? EMPTY_FLOWER_POT : STONE_BRICK_SLAB;
	editor.set_block_absolute(
			cap, x, roof_base + 5 + abs_offset, z, std::nullopt, replace_any);
}

// Hospital wall marker corresponding to Rust's green-cross banner pass.
void generate_hospital_wall_cross(WorldEditor &editor, const ProcessedWay &element,
		BuildingCategory category, int start_y, int building_height, int abs_offset)
{
	if (category != BuildingCategory::Hospital || element.nodes.size() < 3)
		return;
	int min_x = element.nodes.front().x, max_x = min_x;
	int min_z = element.nodes.front().z, max_z = min_z;
	for (const auto &node : element.nodes) {
		min_x = std::min(min_x, node.x);
		max_x = std::max(max_x, node.x);
		min_z = std::min(min_z, node.z);
		max_z = std::max(max_z, node.z);
	}
	const int center_x = (min_x + max_x) / 2;
	const int center_z = (min_z + max_z) / 2;
	const int banner_y = start_y + std::max(building_height * 2 / 3, 2) + abs_offset;
	const std::vector<std::pair<std::string, std::string>> patterns = {
			{"green", "minecraft:straight_cross"}, {"white", "minecraft:stripe_top"},
			{"white", "minecraft:stripe_bottom"}, {"white", "minecraft:border"}};
	for (std::size_t i = 1; i < element.nodes.size(); ++i) {
		const auto &a = element.nodes[i - 1];
		const auto &b = element.nodes[i];
		if (std::max(std::abs(b.x - a.x), std::abs(b.z - a.z)) < 5)
			continue;
		const int mid_x = (a.x + b.x) / 2;
		const int mid_z = (a.z + b.z) / 2;
		const int dx = b.x - a.x, dz = b.z - a.z;
		const int normal_x = dz, normal_z = -dx;
		const int outward =
				(mid_x - center_x) * normal_x + (mid_z - center_z) * normal_z >= 0 ? 1
																				   : -1;
		const int nx = (normal_x * outward > 0) - (normal_x * outward < 0);
		const int nz = (normal_z * outward > 0) - (normal_z * outward < 0);
		const std::string facing = std::abs(nx) >= std::abs(nz)
										   ? (nx > 0 ? "east" : "west")
										   : (nz > 0 ? "south" : "north");
		editor.place_wall_banner(WHITE_WALL_BANNER, mid_x + nx, banner_y, mid_z + nz,
				facing, "white", patterns);
	}
}

// Rust's hospital helipad: a 7x7 yellow roof pad with a white 5x5 H, placed
// only when the complete marking fits on the generated roof footprint.
void generate_hospital_helipad(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &roof_area, int start_y,
		int building_height, int abs_offset)
{
	if (roof_area.empty() || element.nodes.empty())
		return;
	const std::unordered_set<std::pair<int, int>, PairHash> roof(
			roof_area.begin(), roof_area.end());
	int min_x = element.nodes.front().x, max_x = min_x;
	int min_z = element.nodes.front().z, max_z = min_z;
	for (const auto &node : element.nodes) {
		min_x = std::min(min_x, node.x);
		max_x = std::max(max_x, node.x);
		min_z = std::min(min_z, node.z);
		max_z = std::max(max_z, node.z);
	}
	const int center_x = (min_x + max_x) / 2;
	const int center_z = (min_z + max_z) / 2;
	constexpr int pad_half = 3;
	for (int dx = -pad_half; dx <= pad_half; ++dx)
		for (int dz = -pad_half; dz <= pad_half; ++dz)
			if (!roof.contains({center_x + dx, center_z + dz}))
				return;
	const int roof_y = start_y + building_height + abs_offset + 1;
	const std::optional<std::vector<Block>> replace_any{std::vector<Block>{}};
	for (int dx = -pad_half; dx <= pad_half; ++dx)
		for (int dz = -pad_half; dz <= pad_half; ++dz) {
			const bool border = std::abs(dx) == pad_half || std::abs(dz) == pad_half;
			const bool h = std::abs(dx) == 2 || (dz == 0 && std::abs(dx) <= 2);
			const Block block = !border && h ? WHITE_CONCRETE : YELLOW_CONCRETE;
			editor.set_block_absolute(block, center_x + dx, roof_y, center_z + dz,
					std::nullopt, replace_any);
		}
}

// Rust's corner-quoin pass: accent columns at true ring vertices, omitted on
// party walls and passage openings.  This uses the C++ facade plan directly so
// the result remains deterministic across streamed generation regions.
void generate_corner_quoins(WorldEditor &editor, const ProcessedWay &element,
		const building_facade::FacadePlan &facade, BuildingCategory category,
		BuildingCondition condition, Block wall_block, Block accent_block, int start_y,
		int building_height, int abs_offset, const CoordinateBitmap *passages,
		std::uint64_t seed, const building_facade::PointSet *preset_shell)
{
	if (wall_block == accent_block || condition == BuildingCondition::Construction ||
			condition == BuildingCondition::Ruined || element.nodes.size() < 3)
		return;
	int min_x = INT_MAX, max_x = INT_MIN, min_z = INT_MAX, max_z = INT_MIN;
	for (const auto &n : element.nodes) {
		min_x = std::min(min_x, n.x);
		max_x = std::max(max_x, n.x);
		min_z = std::min(min_z, n.z);
		max_z = std::max(max_z, n.z);
	}
	if (max_x - min_x < 4 || max_z - min_z < 4)
		return;
	const bool public_corner = category == BuildingCategory::Commercial ||
							   category == BuildingCategory::Hotel ||
							   category == BuildingCategory::Office ||
							   category == BuildingCategory::Historic;
	const double chance = category == BuildingCategory::Historic ? .9 : .6;
	const bool rolled = element_rng(seed + 3571).random_bool(chance);
	for (std::size_t i = 0; i < element.nodes.size(); ++i) {
		const auto &prev =
				element.nodes[(i + element.nodes.size() - 1) % element.nodes.size()];
		const auto &corner = element.nodes[i];
		const auto &next = element.nodes[(i + 1) % element.nodes.size()];
		const auto cross = std::int64_t(corner.x - prev.x) * (next.z - corner.z) -
						   std::int64_t(corner.z - prev.z) * (next.x - corner.x);
		if (cross == 0 || facade.is_party(corner.x, corner.z) ||
				(preset_shell && preset_shell->contains({corner.x, corner.z})) ||
				mapillary::facades::photo_column(corner.x, corner.z, element.id) ||
				(!rolled &&
						!(public_corner && facade.corner &&
								facade.corner->vertex == std::pair{corner.x, corner.z})))
			continue;
		const int first_y = start_y + 1 +
							((passages && passages->contains(corner.x, corner.z))
											? BUILDING_PASSAGE_HEIGHT
											: 0);
		for (int y = first_y; y <= start_y + building_height; ++y)
			editor.set_block_absolute(accent_block, corner.x, y + abs_offset, corner.z,
					std::vector<Block>{wall_block});
	}
}

void generate_facade_cornice(WorldEditor &editor, const ProcessedWay &element,
		const building_facade::FacadePlan &facade, BuildingCondition condition,
		Block accent_block, int start_y, int building_height, int abs_offset,
		int floor_cycle, const CoordinateBitmap *passages, bool sloped,
		const building_facade::PointSet *preset_shell)
{
	if (condition != BuildingCondition::Normal || sloped ||
			building_height < floor_cycle * 2 || accent_block == AIR ||
			element.nodes.size() < 3)
		return;
	if (element_rng(element.id ^ 0xC0A211CE000077ABULL).uniform(100) >= 55)
		return;
	const int top = start_y + building_height;
	for (std::size_t i = 0; i < element.nodes.size(); ++i) {
		const auto &a = element.nodes[i];
		const auto &b = element.nodes[(i + 1) % element.nodes.size()];
		const int dx = b.x - a.x, dz = b.z - a.z;
		const int nx = (dz > 0) - (dz < 0), nz = (dx < 0) - (dx > 0);
		if (nx == 0 && nz == 0)
			continue;
		const std::string facing = std::abs(nx) >= std::abs(nz)
										   ? (nx > 0 ? "east" : "west")
										   : (nz > 0 ? "south" : "north");
		const auto cornice =
				create_stair_with_properties(get_stair_block_for_material(accent_block),
						facing == "east"	? StairFacing::East
						: facing == "west"	? StairFacing::West
						: facing == "south" ? StairFacing::South
											: StairFacing::North,
						StairShape::Straight);
		for (const auto &[x, unused_y, z] :
				bresenham_line(a.x, top, a.z, b.x, top, b.z)) {
			(void)unused_y;
			if ((passages && passages->contains(x, z)) || facade.is_party(x, z) ||
					(preset_shell && preset_shell->contains({x, z})) ||
					mapillary::facades::photo_column(x, z, element.id))
				continue;
			const std::vector<Block> cornice_variants{AIR};
			editor.set_block_with_properties_absolute(cornice, x + nx, top + abs_offset,
					z + nz, &cornice_variants, nullptr);
		}
	}
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
	auto rng = element_rng(group_seed ^ 0x90D10A7000000001ULL);
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
	const std::vector<Block> replace_any;
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
			const bool wall = std::any_of(std::begin(offsets), std::end(offsets),
					[&, x = x, z = z](const auto &offset) {
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
							y + abs_terrain_offset, z, std::nullopt, replace_any);
				} else if (tier.with_ceilings && row == 0) {
					editor.set_block_absolute(
							(x % 3 == 0 && z % 3 == 0) ? GLOWSTONE : floor_block, x,
							y + abs_terrain_offset, z, std::nullopt, replace_any);
				}
			}
			editor.set_block_absolute(floor_block, x,
					current_base + tier.height + abs_terrain_offset, z, std::nullopt,
					replace_any);
		}
		current_base += tier.height;
	}
	return placed ? std::optional<int>(current_base) : std::nullopt;
}

bool generate_setback_crown(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &roof_area, BuildingCategory category,
		BuildingCondition condition, bool is_tall_building, int start_y,
		int building_height, int abs_terrain_offset, int floor_cycle, Block wall_block,
		Block window_block, Block floor_block, bool has_windows, std::uint64_t element_id)
{
	if (!is_tall_building || condition != BuildingCondition::Normal ||
			roof_area.size() < 200 ||
			!element_rng(element_id ^ 0x5E7BAC4C00000001ULL).random_bool(.45))
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
	std::optional<std::pair<int, int>> deepest;
	int deepest_distance = -1;
	for (const auto &cell : roof_area)
		if (const int distance = distances.at(cell); distance >= deepest_distance) {
			deepest = cell;
			deepest_distance = distance;
		}
	if (deepest && deepest_distance >= 6) {
		const int mast_y = *top + abs_terrain_offset + 1;
		const std::vector<Block> replace_any;
		for (int h = 0; h < 3; ++h)
			editor.set_block_absolute(IRON_BARS, deepest->first, mast_y + h,
					deepest->second, std::nullopt, replace_any);
		editor.set_block_absolute(LIGHTNING_ROD, deepest->first, mast_y + 3,
				deepest->second, std::nullopt, replace_any);
	}
	return true;
}

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, std::optional<Block> roof_block_override,
		std::optional<Block> curtain_glass, std::uint64_t style_seed, bool add_dormers,
		bool steep_gable, std::optional<bool> preferred_ridge_along_x, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset, double scale_factor);

std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels)
{
	FloodFillCache empty_cache;
	CoordinateBitmap empty_passages = CoordinateBitmap::new_empty();
	return generate_buildings(
			editor, element, args, relation_levels, empty_cache, empty_passages, nullptr);
}

std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels, const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons,
		std::optional<std::uint64_t> style_seed, const CoordinateBitmap *road_mask,
		const CoordinateBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members,
		const interior_uses::InteriorUseIndex *interior_index)
{
	const auto group_seed = style_seed.value_or(element.id);
	// Rust seeds procedural choices from the group seed directly. Do not use a
	// tag-packed visual seed here: the category resolver reads the tags itself,
	// while relation groups must retain their shared seed for all style choices.
	const auto visual_seed = group_seed;
	const auto clean_visual_seed = group_seed;
	if (should_skip_underground_tags(element.tags)) {
		return std::nullopt;
	}

	static const std::unordered_set<std::uint64_t> skip_way_ids = {
			5013364, 204068874, 32920861, 1352374225, 1486731987};
	if (skip_way_ids.contains(element.id))
		return std::nullopt;
	if (arnis::man_made::is_tank_structure(element)) {
		arnis::man_made::generate_tank_structure(
				*editor, ProcessedElement(element), args);
		return std::nullopt;
	}
	if (element.tags.get("tomb") == "pyramid") {
		historic::generate_pyramid(*editor, element, args, flood_fill_cache);
		return std::nullopt;
	}

	const int min_level =
			strict_parse::i32(element.tags.get("building:min_level")).value_or(0);

	const bool terrain_enabled = args.terrain_enabled();
	int abs_terrain_offset = (!terrain_enabled) ? args.ground_level : 0;
	double scale_factor = args.scale;
	const std::string early_building_type =
			!element.tags.get("building").empty()
					? element.tags.get("building")
					: (!element.tags.get("building:part").empty()
									  ? element.tags.get("building:part")
									  : "yes");
	int min_level_offset =
			min_level > 0
					? multiply_scale(min_level * floor_cycle_for(early_building_type,
														 element.tags) +
											 2,
							  scale_factor)
					: 0;
	if (auto it = element.tags.find("min_height"); it != element.tags.end()) {
		const auto min_height = parse_meter_value(it->second);
		min_level_offset = min_height ? static_cast<int>(*min_height * scale_factor) : 0;
	}
	const CoordinateBitmap *effective_passages =
			(min_level_offset == 0) ? &building_passages : nullptr;

	std::vector<std::pair<int, int>> cached_floor_area =
			compute_floor_area(&flood_fill_cache, element, args);
	// Thin rings (including ordinary buildings at reduced map scales) may have
	// no interior lattice cells. Rust keeps their outline unless the fill was
	// deliberately refused as oversized or the building has explicit holes.
	if (cached_floor_area.empty() && (!hole_polygons || hole_polygons->empty()) &&
			!is_oversized_ring(element))
		cached_floor_area = thin_ring_cells(element);
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
		return std::nullopt;
	int start_y_offset = 0;
	if (terrain_enabled) {
		start_y_offset = terrain_base_level(
								 *editor, element, cached_floor_area, args.ground_level) +
						 min_level_offset;
	} else {
		start_y_offset = min_level_offset;
	}

	// Match Rust's early special-structure dispatch order so these elements do
	// not pass through generic facade, wall, and roof decoration.
	if (element.tags.get("amenity") == "shelter") {
		for (const auto &node : element.nodes) {
			for (int y = 1; y <= multiply_scale(4, scale_factor); ++y)
				editor->set_block(OAK_FENCE, node.x, y, node.z);
			editor->set_block(STONE_BRICK_SLAB, node.x, 5, node.z);
		}
		for (const auto &[x, z] : cached_floor_area)
			editor->set_block(STONE_BRICK_SLAB, x, 5, z);
		return std::nullopt;
	}
	if (element.tags.get("building:part") == "roof") {
		generate_roof_only_structure(
				*editor, element, cached_floor_area, args, visual_seed);
		return std::nullopt;
	}
	const auto building_tag = element.tags.get("building");
	if (building_tag == "shed" && element.tags.contains("bicycle_parking")) {
		for (const auto &[x, z] : cached_floor_area)
			editor->set_block(OAK_PLANKS, x, 0, z);
		for (const auto &node : element.nodes) {
			for (int y = 1; y <= 4; ++y)
				editor->set_block(OAK_FENCE, node.x, y, node.z);
			editor->set_block(STONE_BLOCK_SLAB, node.x, 5, node.z);
		}
		for (const auto &[x, z] : cached_floor_area)
			editor->set_block(STONE_BLOCK_SLAB, x, 5, z);
		return std::nullopt;
	}
	auto generate_dedicated_parking = [&]() {
		const int parking_floor_cycle =
				floor_cycle_for(early_building_type, element.tags);
		const auto [parking_height, ignored_tall] = calculate_building_height(element,
				early_building_type, min_level, scale_factor, relation_levels,
				parking_floor_cycle, cached_footprint_size, group_seed);
		(void)ignored_tall;
		parking_garage::generate(*editor, cached_floor_area, parking_height,
				start_y_offset + abs_terrain_offset, visual_seed);
	};
	if (building_tag == "parking") {
		generate_dedicated_parking();
		return std::nullopt;
	}
	if (building_tag == "roof") {
		generate_roof_only_structure(
				*editor, element, cached_floor_area, args, visual_seed);
		return std::nullopt;
	}
	if (building_tag == "bridge" && !element.tags.contains("min_height") &&
			!element.tags.contains("building:min_level")) {
		generate_bridge(*editor, element, flood_fill_cache, args.timeout);
		return std::nullopt;
	}
	if (!building_tag.empty() && element.tags.get("parking") == "multi-storey") {
		generate_dedicated_parking();
		return std::nullopt;
	}

	building_facade::PointSet current_footprint(
			cached_floor_area.begin(), cached_floor_area.end());
	building_facade::FacadePlan facade_plan = building_facade::FacadePlan::empty();
	building_facade::PointSet sibling_cells;
	static const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
			empty_groups;
	const auto &groups = group_members ? *group_members : empty_groups;
	if (style_seed) {
		// Part groups are keyed by the seed with packed style-hint bits removed.
		// Different members may carry different hints, but still share a facade
		// and sibling footprint context.
		const auto group = groups.find(osm_parser::seed_without_hint(*style_seed));
		if (group != groups.end() && std::binary_search(group->second.begin(),
											 group->second.end(), element.id)) {
			for (const auto &sibling_id : group->second) {
				if (sibling_id == element.id)
					continue;
				if (const auto fill = flood_fill_cache.get_cached(sibling_id))
					sibling_cells.insert(fill->begin(), fill->end());
			}
		}
	}
	if (road_mask && building_footprints &&
			cached_footprint_size >= building_facade::MIN_FACADE_FOOTPRINT) {
		building_facade::PointSet own_cells = current_footprint;
		own_cells.insert(sibling_cells.begin(), sibling_cells.end());
		static const interior_uses::InteriorUseIndex empty_interior_uses;
		const auto &facade_interior_uses =
				interior_index ? *interior_index : empty_interior_uses;
		const building_facade::BuildingContext context{flood_fill_cache,
				building_passages, *road_mask, *building_footprints, groups,
				facade_interior_uses};
		facade_plan = building_facade::compute_facade_plan(
				element, context, args.scale, own_cells);
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
	// Rust resolves the building palette from the shared group-seeded stream.
	auto rng = element_rng(group_seed);

	Block wall_block = STONE;
	const auto resolve_tagged_wall = [&]() -> std::optional<Block> {
		auto it_hist = element.tags.find("historic");
		if (it_hist != element.tags.end() && it_hist->second == "castle")
			return get_castle_wall_block(rng);

		auto it_material = element.tags.find("building:material");
		if (it_material == element.tags.end())
			it_material = element.tags.find("building:facade:material");
		if (it_material == element.tags.end())
			it_material = element.tags.find("facade:material");
		if (it_material != element.tags.end()) {
			if (auto block = get_wall_block_for_material_cpp(it_material->second, rng))
				return block;
		}

		auto it_color = element.tags.find("building:colour");
		if (it_color == element.tags.end())
			it_color = element.tags.find("colour");
		if (it_color != element.tags.end())
			if (const auto rgb = color_text_to_rgb_tuple(it_color->second))
				return get_building_wall_block_for_color(*rgb, rng);

		return std::nullopt;
	};

	Block floor_block = STONE;
	Block window_block = GLASS;
	std::unordered_set<std::pair<int, int>, PairHash> processed_points;
	int floor_cycle = floor_cycle_for(building_type, element.tags);
	const int grammar_anchor = min_level_offset == 0 ? 2 : 0;
	auto [building_height, is_tall_building] =
			calculate_building_height(element, building_type, min_level, scale_factor,
					relation_levels, floor_cycle, cached_footprint_size, group_seed);
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
						relation_levels, floor_cycle, cached_footprint_size, group_seed);
	}
	const int style_building_height = building_height;
	// Rust classifies from the tagged/inferred full height, then applies
	// construction/ruin reductions to the realized shell.
	const BuildingCategory category = building_category(
			element, is_tall_building, style_building_height, scale_factor, visual_seed);
	if (condition == BuildingCondition::Construction)
		building_height = std::max(3, building_height / 2);
	else if (condition == BuildingCondition::Ruined)
		building_height = std::max(3, int(building_height * .6));
	// Rust records preset facade candidates after category/height selection.  The
	// backend-native registry performs the equivalent crop and submission here;
	// it is a no-op unless --building-facades was enabled and a backend sink is
	// installed.
	const auto preset_shell = building_facades::collect(*editor, element.nodes,
			element.id, visual_seed, category, facade_plan, building_footprints,
			current_footprint, min_level_offset == 0,
			start_y_offset + abs_terrain_offset + 1, building_height, scale_factor);
	const auto is_flat_column = [&](int x, int z) {
		return mapillary::facades::photo_column(x, z, element.id) ||
			   preset_shell.contains({x, z});
	};
	// Rust's glass-curtain presets use the facade itself as the window field;
	// ordinary procedural window slots would punch a mismatched rhythm into
	// the curtain wall. Keep the structural corner-pier logic separate.
	const bool curtain_glass_category =
			category == BuildingCategory::GlassySkyscraper ||
			category == BuildingCategory::GlassCornerSkyscraper;
	const bool has_windows =
			condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined && !curtain_glass_category &&
			category != BuildingCategory::Garage && category != BuildingCategory::Shed &&
			category != BuildingCategory::Greenhouse &&
			cached_footprint_size > THIN_RING_MAX_CELLS;
	ArchEra era = building_arch_era(element.tags);
	if (era == ArchEra::Unknown && element.tags.contains("building:part"))
		era = static_cast<ArchEra>(osm_parser::arch_era_from_hint(
				osm_parser::style_hint_from_seed(group_seed)));
	const DetailTier detail = compute_detail_tier(element, category,
			cached_footprint_size, style_building_height, facade_plan.has_any_street);
	const WallDepthStyle wall_depth_style = wall_depth_style_for(category);
	const bool wall_depth_span_ok = max_x - min_x >= 4 && max_z - min_z >= 4;
	const bool wall_depth_height_ok =
			building_height >= floor_cycle + 2 ||
			wall_depth_style == WallDepthStyle::HistoricOrnate ||
			wall_depth_style == WallDepthStyle::ReligiousButtress;
	const bool wall_depth_piers_active =
			wall_depth_span_ok && wall_depth_height_ok &&
			((wall_depth_style == WallDepthStyle::ModernPillars ||
					 wall_depth_style == WallDepthStyle::InstitutionalBands ||
					 wall_depth_style == WallDepthStyle::HistoricOrnate ||
					 wall_depth_style == WallDepthStyle::SkyscraperFins) ||
					(wall_depth_style == WallDepthStyle::SubtlePilasters &&
							element_rng(visual_seed + 7919).uniform(100) < 60));
	const auto climate = editor->get_ground() ? editor->get_ground()->climate()
											  : biome::Climate::Temperate;
	if (category == BuildingCategory::GlassySkyscraper) {
		if (element.tags.get("historic") == "castle")
			wall_block = get_castle_wall_block(rng);
		else if (condition != BuildingCondition::Construction)
			wall_block = rust_category_wall_block(category, era, climate, rng);
	} else if (explicit_wall) {
		if (const auto tagged_wall = resolve_tagged_wall())
			wall_block = *tagged_wall;
		else if (condition != BuildingCondition::Construction)
			wall_block = rust_category_wall_block(category, era, climate, rng);
	} else if (condition != BuildingCondition::Construction) {
		wall_block = rust_category_wall_block(category, era, climate, rng);
	}
	if (condition == BuildingCondition::Construction)
		wall_block = SCAFFOLDING;
	if (category == BuildingCategory::MasonrySkyscraper) {
		floor_block = SMOOTH_STONE;
	} else if (category == BuildingCategory::GlassySkyscraper ||
			   category == BuildingCategory::ModernSkyscraper) {
		const Block cap_options[] = {POLISHED_ANDESITE, BLACKSTONE, NETHER_BRICK};
		floor_block = cap_options[rng.uniform(3)];
	} else {
		floor_block = get_random_floor_block(rng);
	}
	if (category == BuildingCategory::GridSkyscraper ||
			category == BuildingCategory::ContemporarySkyscraper)
		window_block = LIGHT_BLUE_STAINED_GLASS;
	else if (category == BuildingCategory::MasonrySkyscraper)
		window_block = LIGHT_GRAY_STAINED_GLASS;
	else if (category == BuildingCategory::Tower)
		window_block = GLASS;
	else
		window_block = category_window_block(category, rng);
	const auto window_frame =
			pick_window_frame(category, era, detail, wall_block, clean_visual_seed);
	const WindowArchetype window_archetype =
			pick_window_archetype(category, era, clean_visual_seed);
	const int window_phase =
			element.tags.contains("building:part")
					? 0
					: int(element_rng(static_cast<std::uint64_t>(element.id) ^
									  0x77D0A3E19B1C5544ULL)
									  .uniform(6));
	const bool horizontal_windows = category == BuildingCategory::ModernSkyscraper;
	const bool storefront =
			(category == BuildingCategory::Commercial ||
					category == BuildingCategory::Hotel) &&
			has_windows && condition == BuildingCondition::Normal &&
			min_level_offset == 0 && facade_plan.front_segment.has_value() &&
			element_rng(group_seed ^ 0x5709EF9000000002ULL).random_bool(.6);
	const Block awning_options[] = {WARPED_TRAPDOOR, SPRUCE_TRAPDOOR, DARK_OAK_TRAPDOOR,
			JUNGLE_TRAPDOOR, ACACIA_TRAPDOOR};
	const Block awning_block =
			awning_options[element_rng(group_seed ^ 0x0A3B11B60000000BULL).uniform(5)];
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
					condition, !sloped_roof, args.roof, cached_floor_area, group_seed);
	if (podium_tower)
		building_height = podium_tower->podium_height;
	const bool top_treatment =
			has_windows && building_height >= 4 * floor_cycle && !horizontal_windows &&
			category != BuildingCategory::GlassySkyscraper &&
			category != BuildingCategory::GlassCornerSkyscraper &&
			category != BuildingCategory::GridSkyscraper &&
			category != BuildingCategory::Tower &&
			element_rng(group_seed ^ 0xF10A401E00000001ULL).random_bool(.45);
	const bool attic_style =
			has_windows && sloped_roof && building_height >= 3 * floor_cycle &&
			(category == BuildingCategory::House ||
					category == BuildingCategory::Residential ||
					category == BuildingCategory::Historic) &&
			element_rng(group_seed ^ 0xF10A401E00000002ULL).random_bool(.55);
	const bool piano_nobile =
			has_windows && building_height >= 3 * floor_cycle &&
			((category == BuildingCategory::Historic &&
					 element_rng(group_seed ^ 0xF10A401E00000003ULL).random_bool(.5)) ||
					(category == BuildingCategory::Hotel &&
							element_rng(group_seed ^ 0xF10A401E00000003ULL)
									.random_bool(.2)));
	const BalconyBand balcony_band = pick_balcony_band(category, building_height,
			floor_cycle, facade_plan.has_any_street, clean_visual_seed);
	const Block base_course_block = base_course_for_wall(wall_block);
	const bool has_base_course =
			min_level_offset == 0 && has_windows &&
			condition == BuildingCondition::Normal &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage &&
			category != BuildingCategory::GlassySkyscraper &&
			base_course_block != wall_block &&
			element_rng(group_seed ^ 0xBA5EC0A25E110001ULL).random_bool(.7);
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
							element_rng(group_seed ^ 0x0BA5E5A00000000AULL)
									.random_bool(.7)));

	Block accent_block;
	if (category == BuildingCategory::GlassCornerSkyscraper) {
		accent_block = LIGHT_GRAY_CONCRETE;
	} else if (category == BuildingCategory::MasonrySkyscraper) {
		accent_block = SMOOTH_STONE;
	} else if (category == BuildingCategory::GlassySkyscraper) {
		const Block options[] = {WHITE_STAINED_GLASS, BLACKSTONE};
		accent_block = options[rng.uniform(2)];
	} else if (category == BuildingCategory::ModernSkyscraper) {
		const Block options[] = {
				POLISHED_ANDESITE, SMOOTH_STONE, BLACKSTONE, NETHER_BRICK, STONE_BRICKS};
		accent_block = options[rng.uniform(5)];
	} else {
		const Block options[] = {POLISHED_ANDESITE, SMOOTH_STONE, STONE_BRICKS,
				MUD_BRICKS, ANDESITE, CHISELED_STONE_BRICKS};
		accent_block = options[rng.uniform(6)];
	}
	const auto preset_vertical_windows = [&]() -> std::optional<bool> {
		switch (category) {
		case BuildingCategory::House:
		case BuildingCategory::Residential:
		case BuildingCategory::Farm:
		case BuildingCategory::School:
		case BuildingCategory::Warehouse:
		case BuildingCategory::Commercial:
		case BuildingCategory::GridSkyscraper:
		case BuildingCategory::ContemporarySkyscraper:
		case BuildingCategory::MasonrySkyscraper:
			return false;
		case BuildingCategory::Office:
		case BuildingCategory::Hotel:
		case BuildingCategory::Hospital:
		case BuildingCategory::Religious:
		case BuildingCategory::TallBuilding:
			return true;
		default:
			return std::nullopt;
		}
	};
	const auto fixed_vertical_windows = preset_vertical_windows();
	bool use_vertical_windows =
			fixed_vertical_windows ? *fixed_vertical_windows : rng.random_bool(0.7);
	const auto preset_roof_line = [&]() -> std::optional<bool> {
		switch (category) {
		case BuildingCategory::House:
		case BuildingCategory::Office:
		case BuildingCategory::Hotel:
		case BuildingCategory::Hospital:
		case BuildingCategory::School:
		case BuildingCategory::Religious:
		case BuildingCategory::TallBuilding:
		case BuildingCategory::ModernSkyscraper:
		case BuildingCategory::GlassySkyscraper:
		case BuildingCategory::GlassCornerSkyscraper:
		case BuildingCategory::GridSkyscraper:
		case BuildingCategory::ContemporarySkyscraper:
		case BuildingCategory::MasonrySkyscraper:
		case BuildingCategory::Tower:
		case BuildingCategory::Garage:
		case BuildingCategory::Commercial:
			return true;
		case BuildingCategory::Warehouse:
		case BuildingCategory::Historic:
		case BuildingCategory::Shed:
		case BuildingCategory::Greenhouse:
			return false;
		default:
			return std::nullopt;
		}
	};
	const auto fixed_roof_line = preset_roof_line();
	bool use_accent_roof_line =
			fixed_roof_line ? *fixed_roof_line : rng.random_bool(0.25);
	bool has_multiple_floors = style_building_height > floor_cycle + 2;
	const auto preset_accent_lines = [&]() -> std::optional<bool> {
		switch (category) {
		case BuildingCategory::House:
		case BuildingCategory::Residential:
		case BuildingCategory::Farm:
		case BuildingCategory::Industrial:
		case BuildingCategory::Warehouse:
		case BuildingCategory::Religious:
		case BuildingCategory::Historic:
		case BuildingCategory::Garage:
		case BuildingCategory::Shed:
		case BuildingCategory::Greenhouse:
			return false;
		case BuildingCategory::Hotel:
		case BuildingCategory::ModernSkyscraper:
		case BuildingCategory::MasonrySkyscraper:
		case BuildingCategory::Tower:
			return true;
		default:
			return std::nullopt;
		}
	};
	const auto fixed_accent_lines = preset_accent_lines();
	bool use_accent_lines =
			fixed_accent_lines
					? *fixed_accent_lines
					: (category == BuildingCategory::GlassySkyscraper
									  ? rng.random_bool(0.6)
									  : (category == BuildingCategory::ModernSkyscraper ||
												(has_multiple_floors &&
														rng.random_bool(0.2))));
	const auto preset_vertical_accent = [&]() -> std::optional<bool> {
		switch (category) {
		case BuildingCategory::Farm:
		case BuildingCategory::Industrial:
		case BuildingCategory::Warehouse:
		case BuildingCategory::ModernSkyscraper:
		case BuildingCategory::GlassySkyscraper:
		case BuildingCategory::GlassCornerSkyscraper:
		case BuildingCategory::Historic:
		case BuildingCategory::Tower:
		case BuildingCategory::Garage:
		case BuildingCategory::Shed:
		case BuildingCategory::Greenhouse:
			return false;
		default:
			return std::nullopt;
		}
	};
	const auto fixed_vertical_accent = preset_vertical_accent();
	bool use_vertical_accent =
			fixed_vertical_accent
					? *fixed_vertical_accent
					: (has_multiple_floors && !use_accent_lines && rng.random_bool(0.1));
	const RoofType resolved_style_roof = [&]() {
		const auto roof_shape = element.tags.get("roof:shape");
		if (!roof_shape.empty())
			return parse_roof_type(roof_shape);
		const bool preset_flat_roof =
				category == BuildingCategory::TallBuilding ||
				category == BuildingCategory::ModernSkyscraper ||
				category == BuildingCategory::GlassySkyscraper ||
				category == BuildingCategory::GlassCornerSkyscraper ||
				category == BuildingCategory::GridSkyscraper ||
				category == BuildingCategory::ContemporarySkyscraper ||
				category == BuildingCategory::MasonrySkyscraper ||
				category == BuildingCategory::Historic ||
				category == BuildingCategory::Hospital ||
				category == BuildingCategory::Tower ||
				category == BuildingCategory::Garage ||
				category == BuildingCategory::Greenhouse;
		if (preset_flat_roof ||
				(element.tags.contains("building:part") &&
						(element.tags.contains("height") ||
								element.tags.contains("building:levels"))))
			return RoofType::Flat;
		const bool auto_gable_type =
				building_type == "apartments" || building_type == "residential" ||
				building_type == "house" || building_type == "yes" ||
				building_type == "detached" || building_type == "semidetached_house" ||
				building_type == "terrace" || building_type == "bungalow" ||
				building_type == "villa" || building_type == "cabin" ||
				building_type == "hut" || building_type == "farm" ||
				building_type == "farm_auxiliary" || building_type == "barn" ||
				building_type == "stable" || building_type == "cowshed" ||
				building_type == "sty" || building_type == "sheepfold";
		if (auto_gable_type) {
			constexpr std::size_t max_footprint_for_gabled = 800;
			auto roof_rng = element_rng(group_seed ^ 0x0F1E2D3C4B5A6907ULL);
			const bool big_block = cached_footprint_size > max_footprint_for_gabled ||
								   style_building_height >= 15;
			RoofType result = RoofType::Flat;
			if (building_type == "apartments" && big_block) {
				const bool prewar = era == ArchEra::HistoricOrnate ||
									era == ArchEra::TraditionalPreWar;
				const auto roll = roof_rng.uniform(100);
				const int hip_limit = prewar ? 65 : 80;
				if (roll >= 45)
					result = roll < hip_limit ? RoofType::Hipped : RoofType::Mansard;
			} else if ((era == ArchEra::HistoricOrnate ||
							   era == ArchEra::TraditionalPreWar) &&
					   cached_footprint_size >= 100 && cached_footprint_size <= 800 &&
					   roof_rng.random_bool(0.30)) {
				result = RoofType::Mansard;
			} else {
				const double gable_probability =
						(climate == biome::Climate::HotDesert ||
								climate == biome::Climate::HotSteppe)
								? 0.35
						: climate == biome::Climate::TropicalSavanna ? 0.60
						: (climate == biome::Climate::Boreal ||
								  climate == biome::Climate::Tundra ||
								  climate == biome::Climate::IceCap)
								? 0.95
								: 0.90;
				if (cached_footprint_size <= max_footprint_for_gabled &&
						roof_rng.random_bool(gable_probability))
					result = RoofType::Gabled;
			}
			if ((result == RoofType::Gabled || result == RoofType::Hipped ||
						result == RoofType::Mansard || result == RoofType::Gambrel) &&
					compute_building_diagonality(element.nodes) < 0.35 &&
					std::abs(dominant_building_axis_angle(element.nodes)) * 180.0 /
									std::numbers::pi >
							10.0)
				result = RoofType::Pyramidal;
			return result;
		}
		if ((building_type == "industrial" || building_type == "warehouse" ||
					building_type == "hangar") &&
				cached_footprint_size > 800) {
			auto roof_rng = element_rng(group_seed ^ 0x0F1E2D3C4B5A6907ULL);
			return roof_rng.random_bool(0.55) ? RoofType::Skillion : RoofType::Flat;
		}
		return RoofType::Flat;
	}();
	const bool chimney_roof_type = resolved_style_roof == RoofType::Gabled ||
								   resolved_style_roof == RoofType::Hipped ||
								   resolved_style_roof == RoofType::Mansard ||
								   resolved_style_roof == RoofType::Gambrel ||
								   resolved_style_roof == RoofType::HalfHipped;
	const bool residential_chimney_type =
			building_type == "house" || building_type == "residential" ||
			building_type == "detached" || building_type == "semidetached_house" ||
			building_type == "terrace" || building_type == "farm" ||
			building_type == "cabin" || building_type == "bungalow" ||
			building_type == "villa" || building_type == "yes";
	const bool preset_chimney = category == BuildingCategory::House;
	const bool infer_chimney = (category == BuildingCategory::Residential ||
									   category == BuildingCategory::Default) &&
							   residential_chimney_type && chimney_roof_type &&
							   cached_footprint_size >= 30 &&
							   cached_footprint_size <= 400 && rng.random_bool(0.40);
	const bool style_has_chimney = preset_chimney || infer_chimney;
	// Port fixed BuildingStylePreset options; unspecified fields retain the
	// deterministic category/default choices above.
	switch (category) {
	case BuildingCategory::House:
		use_vertical_windows = false;
		use_accent_lines = false;
		use_accent_roof_line = true;
		break;
	case BuildingCategory::Residential:
		use_vertical_windows = false;
		use_accent_lines = false;
		break;
	case BuildingCategory::Farm:
		use_vertical_windows = false;
		use_accent_lines = false;
		use_vertical_accent = false;
		break;
	case BuildingCategory::Office:
	case BuildingCategory::Hotel:
	case BuildingCategory::Hospital:
		use_vertical_windows = true;
		use_accent_roof_line = true;
		if (category == BuildingCategory::Hotel)
			use_accent_lines = true;
		break;
	case BuildingCategory::School:
		use_vertical_windows = false;
		use_accent_roof_line = true;
		break;
	case BuildingCategory::Religious:
		use_vertical_windows = true;
		use_accent_roof_line = true;
		use_accent_lines = false;
		break;
	case BuildingCategory::TallBuilding:
		use_vertical_windows = true;
		use_accent_roof_line = true;
		break;
	case BuildingCategory::ModernSkyscraper:
		use_accent_roof_line = true;
		use_vertical_accent = false;
		break;
	case BuildingCategory::GlassySkyscraper:
		use_accent_roof_line = true;
		use_vertical_accent = false;
		break;
	case BuildingCategory::GlassCornerSkyscraper:
		use_accent_roof_line = true;
		use_vertical_accent = false;
		accent_block = LIGHT_GRAY_CONCRETE;
		break;
	case BuildingCategory::GridSkyscraper:
		use_vertical_windows = false;
		use_accent_roof_line = true;
		window_block = LIGHT_BLUE_STAINED_GLASS;
		break;
	case BuildingCategory::ContemporarySkyscraper:
		use_vertical_windows = false;
		use_accent_roof_line = true;
		window_block = LIGHT_BLUE_STAINED_GLASS;
		break;
	case BuildingCategory::MasonrySkyscraper:
		use_accent_lines = true;
		use_accent_roof_line = true;
		accent_block = SMOOTH_STONE;
		floor_block = SMOOTH_STONE;
		window_block = LIGHT_GRAY_STAINED_GLASS;
		break;
	case BuildingCategory::Industrial:
		use_accent_lines = false;
		use_vertical_accent = false;
		break;
	case BuildingCategory::Warehouse:
		use_vertical_windows = false;
		use_accent_lines = false;
		use_vertical_accent = false;
		use_accent_roof_line = false;
		break;
	case BuildingCategory::Historic:
		use_accent_lines = false;
		use_vertical_accent = false;
		use_accent_roof_line = false;
		break;
	case BuildingCategory::Tower:
		use_accent_lines = true;
		use_vertical_accent = false;
		use_accent_roof_line = true;
		window_block = GLASS;
		break;
	case BuildingCategory::Garage:
		use_accent_lines = false;
		use_vertical_accent = false;
		use_accent_roof_line = true;
		break;
	case BuildingCategory::Shed:
	case BuildingCategory::Greenhouse:
		use_accent_lines = false;
		use_vertical_accent = false;
		use_accent_roof_line = false;
		break;
	case BuildingCategory::Commercial:
		use_vertical_windows = false;
		use_accent_roof_line = true;
		break;
	case BuildingCategory::Default:
		break;
	}
	if (category == BuildingCategory::ContemporarySkyscraper ||
			category == BuildingCategory::GridSkyscraper ||
			category == BuildingCategory::ModernSkyscraper) {
		const auto is_dark = [](Block block) {
			return block == BLACK_CONCRETE || block == GRAY_CONCRETE ||
				   block == BLACKSTONE || block == POLISHED_BLACKSTONE ||
				   block == DEEPSLATE_BRICKS || block == NETHER_BRICK ||
				   block == BLACK_TERRACOTTA || block == GRAY_TERRACOTTA;
		};
		if (is_dark(wall_block) || is_dark(accent_block))
			window_block = GRAY_STAINED_GLASS;
	}
	// Rust applies Mapillary's colour after the procedural choices, keeping this
	// building-level override from shifting the shared style RNG draw sequence.
	if (const auto facade_color = mapillary::facades::building_color(element.id)) {
		wall_block = get_building_wall_block_for_color(
				RGB{(*facade_color)[0], (*facade_color)[1], (*facade_color)[2]}, rng);
	}

	{
		auto it = element.tags.find("building");
		if (it != element.tags.end()) {
			const std::string &btype = it->second;
			if (btype == "garage") {
				building_height = std::max(3, static_cast<int>(2.0 * scale_factor));
			} else if (btype == "shed") {
				building_height = std::max(3, static_cast<int>(2.0 * scale_factor));
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
			const bool segment_is_x = segment_axis_x(
					element.nodes[node_index - 1], element.nodes[node_index]);
			const bool start_is_x =
					segment_start_axis_x(element.nodes, node_index, segment_is_x);
			auto bresenham_points = bresenham_line(
					prev.first, start_y_offset, prev.second, x, start_y_offset, z);
			for (std::size_t point_index = 0; point_index < bresenham_points.size();
					++point_index) {
				const auto &t = bresenham_points[point_index];
				int bx = std::get<0>(t);
				int bz = std::get<2>(t);
				const int wall_u =
						wall_ordinate(point_index, start_is_x, segment_is_x, bx, bz);
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

				if (terrain_enabled && min_level_offset == 0 && !is_passage) {
					const int local_ground_level =
							editor->terrain_level(bx, bz).value_or(args.ground_level);
					for (int y = local_ground_level; y <= start_y_offset; ++y) {
						editor->set_block_absolute(
								wall_block, bx, y + abs_terrain_offset, bz);
					}
				}

				const int passage_height =
						std::min(BUILDING_PASSAGE_HEIGHT, building_height);
				const int wall_start = is_passage ? start_y_offset + passage_height + 1
												  : start_y_offset + 1;
				int column_top = start_y_offset + building_height;
				if (condition == BuildingCondition::Construction) {
					// Rust derives an independent, repeatable half-built height for each
					// wall column from its world coordinate and the source element ID.
					auto column_rng =
							coord_rng(bx, bz, static_cast<std::uint64_t>(element.id));
					const double factor = .30 + column_rng.uniform_real(0.0, 1.0) * .55;
					const int local =
							start_y_offset +
							static_cast<int>(std::lround(
									static_cast<double>(building_height) * factor));
					column_top = std::max(start_y_offset + 1, local);
					column_top =
							std::min(start_y_offset + building_height - 1, column_top);
				}
				for (int h = wall_start; h <= column_top; ++h) {
					const bool above_floor = h > start_y_offset + 1;
					const int floor_row =
							((h - start_y_offset - grammar_anchor) % floor_cycle +
									floor_cycle) %
							floor_cycle;
					const int window_col = ((wall_u + window_phase) % 6 + 6) % 6;
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
											  ((wall_u + window_phase) % 4 + 4) % 4 == 1;
							chosen = slit ? window_block : wall_block;
						} else if (category == BuildingCategory::GridSkyscraper) {
							const bool mullion =
									!above_floor || floor_row == 0 ||
									((wall_u + window_phase) % 5 + 5) % 5 == 0;
							chosen = mullion ? wall_block : window_block;
						} else if (is_tall_building && use_vertical_windows) {
							chosen = above_floor && ((wall_u + window_phase) % 2 + 2) %
																	 2 ==
															 0
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
					// Rust's glass-corner skyscraper preset puts a solid pier at
					// every exterior corner, including corners on otherwise glass
					// window rows. Keep the corner structural instead of allowing a
					// continuous pane to wrap around it.
					if (category == BuildingCategory::GlassCornerSkyscraper &&
							!party_wall && outward != std::pair<int, int>{0, 0} &&
							((bx == prev.first && bz == prev.second) ||
									(bx == x && bz == z)) &&
							(chosen == window_block || chosen == GLASS))
						chosen = LIGHT_GRAY_CONCRETE;
					if (mapillary::facades::photo_column(bx, bz, element.id) ||
							preset_shell.contains({bx, bz}))
						chosen = wall_block;
					if (const auto facade = mapillary::facades::block_at(
								bx, h, bz, start_y_offset, element.id, window_block)) {
						chosen = *facade;
					} else if (chosen == wall_block &&
							   condition != BuildingCondition::Construction) {
						if (const auto band = mapillary::facades::band_block_at(
									bx, h, bz, start_y_offset, element.id))
							chosen = *band;
					}
					chosen = apply_condition_variation(chosen, bx, h, bz, wall_block,
							window_block, has_windows, condition, category, era,
							clean_visual_seed);
					editor->set_block_absolute(chosen, bx, h + abs_terrain_offset, bz);
					// Rust storefronts receive a shallow projecting awning above
					// street-facing ground-floor glass.  Slabs are used here as the
					// C++ backend's portable horizontal equivalent of trapdoors.
					if (chosen == GLASS && storefront && facade_plan.is_street(bx, bz) &&
							outward != std::pair<int, int>{0, 0} && h == ground_top &&
							!facade_plan.is_door(bx, bz))
						editor->set_block_absolute(OAK_SLAB, bx + outward.first,
								h + 1 + abs_terrain_offset, bz + outward.second,
								std::vector<Block>{AIR});
					if (chosen == window_block &&
							window_archetype == WindowArchetype::ArchedTraditional &&
							condition == BuildingCondition::Normal &&
							outward != std::pair<int, int>{0, 0} && above_floor &&
							floor_row == floor_cycle - 1 && window_col != 1 &&
							!facade_plan.is_door(bx, bz) && !party_wall)
						editor->set_block_absolute(STONE_BRICK_SLAB, bx + outward.first,
								h + abs_terrain_offset, bz + outward.second,
								std::vector<Block>{AIR});
					if (category == BuildingCategory::Historic &&
							condition == BuildingCondition::Normal &&
							wall_depth_style != WallDepthStyle::HistoricOrnate &&
							h == start_y_offset + building_height &&
							outward != std::pair<int, int>{0, 0} && !party_wall)
						editor->set_block_absolute(STONE_BRICK_SLAB, bx + outward.first,
								h + abs_terrain_offset + 1, bz + outward.second,
								std::vector<Block>{AIR});
					// Residential Rust facades add a shallow sill below ordinary
					// glazed columns. Keep it outside the wall and clear of doors.
					if (chosen == window_block && outward != std::pair<int, int>{0, 0} &&
							(category == BuildingCategory::House ||
									category == BuildingCategory::Residential) &&
							condition == BuildingCondition::Normal && h > wall_start &&
							(h - start_y_offset) % floor_cycle == 1 &&
							!facade_plan.is_door(bx, bz))
						editor->set_block_absolute(OAK_SLAB, bx + outward.first,
								h - 1 + abs_terrain_offset, bz + outward.second,
								std::vector<Block>{AIR});
					if (chosen == window_block && (window_col == 0 || window_col == 2) &&
							h > wall_start && (h - start_y_offset) % floor_cycle == 1 &&
							(category == BuildingCategory::House ||
									category == BuildingCategory::Residential) &&
							condition == BuildingCondition::Normal &&
							outward != std::pair<int, int>{0, 0} &&
							(!facade_plan.is_door(bx, bz) || h > start_y_offset + 3)) {
						const Block shutter =
								(clean_visual_seed & 1) ? OAK_TRAPDOOR : SPRUCE_TRAPDOOR;
						editor->set_block_absolute(shutter, bx + outward.first,
								h + abs_terrain_offset, bz + outward.second,
								std::vector<Block>{AIR});
					}
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
											((floor_div(wall_u + window_phase, 6) % 2 +
													 2) % 2 ==
													0)) ||
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

	// Rust builds the roof over both the filled footprint and the wall ring so
	// roof blocks reach the outside face of the generated walls. Keep the area
	// sorted to make every roof feature's iteration order deterministic.
	std::vector<std::pair<int, int>> roof_area = cached_floor_area;
	roof_area.insert(roof_area.end(), current_building.begin(), current_building.end());
	std::sort(roof_area.begin(), roof_area.end());
	roof_area.erase(std::unique(roof_area.begin(), roof_area.end()), roof_area.end());

	std::optional<building_facade::FacadeAnchor> signage_anchor;
	std::vector<interior_uses::Entry> interior_entries;
	// Rust keeps the selected door species and orientation as block-state
	// properties. Preserve those states in C++ too; the backend can then map
	// them to the target game's native door representation.
	const auto entrance_door_style = [&]() {
		if (category == BuildingCategory::Industrial ||
				category == BuildingCategory::Warehouse ||
				category == BuildingCategory::Commercial ||
				category == BuildingCategory::Office ||
				category == BuildingCategory::Hotel ||
				category == BuildingCategory::Historic ||
				category == BuildingCategory::Religious)
			return std::pair{DARK_OAK_DOOR_LOWER, DARK_OAK_TRAPDOOR};
		if (wall_block == OAK_PLANKS || wall_block == OAK_LOG)
			return std::pair{SPRUCE_DOOR_LOWER, SPRUCE_TRAPDOOR};
		if (wall_block == SPRUCE_PLANKS || wall_block == SPRUCE_LOG ||
				wall_block == DARK_OAK_PLANKS)
			return std::pair{OAK_DOOR, OAK_TRAPDOOR};
		if (wall_block == QUARTZ_BLOCK || wall_block == QUARTZ_BRICKS ||
				wall_block == WHITE_CONCRETE || wall_block == WHITE_TERRACOTTA)
			return std::pair{BIRCH_DOOR, BIRCH_TRAPDOOR};
		return element_rng(group_seed ^ 0xD00E57E900000012ULL).random_bool(.5)
					   ? std::pair{OAK_DOOR, OAK_TRAPDOOR}
					   : std::pair{DARK_OAK_DOOR_LOWER, DARK_OAK_TRAPDOOR};
	};
	const auto facing_name = [](int nx, int nz) -> const char * {
		if (std::abs(nx) >= std::abs(nz))
			return nx >= 0 ? "east" : "west";
		return nz >= 0 ? "south" : "north";
	};
	const auto put_entrance_door = [&](Block base, int x, int z, int nx, int nz,
										   const char *hinge) {
		const std::string facing = facing_name(nx, nz);
		const BlockWithProperties lower{
				base, {{"half", "lower"}, {"facing", facing}, {"hinge", hinge}}};
		const BlockWithProperties upper{
				base, {{"half", "upper"}, {"facing", facing}, {"hinge", hinge}}};
		editor->set_block_with_properties_absolute(lower, x,
				start_y_offset + abs_terrain_offset + 1, z, std::nullopt, std::nullopt);
		editor->set_block_with_properties_absolute(upper, x,
				start_y_offset + abs_terrain_offset + 2, z, std::nullopt, std::nullopt);
	};
	const auto put_doorstep = [&](int x, int z, int nx, int nz) {
		if (!terrain_enabled || min_level_offset != 0)
			return;
		constexpr int max_steps = 3;
		const auto ground = editor->terrain_level(x + nx, z + nz);
		if (!ground)
			return;
		const int drop = start_y_offset - *ground;
		if (drop < 1 || drop > max_steps)
			return;
		for (int step = 1; step <= drop; ++step) {
			const int sx = x + nx * step;
			const int sz = z + nz * step;
			const int sy = start_y_offset - step + 1;
			if (step > 1) {
				const auto cell_ground = editor->terrain_level(sx, sz);
				if (!cell_ground || *cell_ground < sy - 1)
					return;
			}
		}
		const int toward_door_x = -nx;
		const int toward_door_z = -nz;
		const StairFacing facing =
				std::abs(toward_door_x) >= std::abs(toward_door_z)
						? (toward_door_x >= 0 ? StairFacing::East : StairFacing::West)
						: (toward_door_z >= 0 ? StairFacing::South : StairFacing::North);
		const Block stair_base = get_stair_block_for_material(
				base_course_block != wall_block ? base_course_block : STONE_BRICKS);
		for (int step = 1; step <= drop; ++step) {
			const int sx = x + nx * step;
			const int sz = z + nz * step;
			const int sy = start_y_offset - step + 1 + abs_terrain_offset;
			const auto stair = create_stair_with_properties(
					stair_base, facing, StairShape::Straight);
			editor->set_block_with_properties_absolute(
					stair, sx, sy, sz, std::vector<Block>{AIR}, std::nullopt);
		}
	};
	const auto put_threshold = [&](int x, int z, int nx, int nz) {
		const Block threshold =
				base_course_block != wall_block ? base_course_block : STONE_BRICKS;
		editor->set_block_absolute(
				threshold, x + nx, start_y_offset + abs_terrain_offset, z + nz);
		put_doorstep(x, z, nx, nz);
	};
	const bool mapped_entrance = std::any_of(
			element.nodes.begin(), element.nodes.end(), [](const ProcessedNode &node) {
				return node.tags.contains("entrance") || node.tags.contains("door");
			});
	// Match BuildingStyle::resolve/configuration in Rust: preset utility doors
	// are suppressed for mapped entrances and thin-ring footprints, and an
	// abandoned building clears both special-door flags.
	const bool thin_ring_footprint = cached_footprint_size <= THIN_RING_MAX_CELLS;
	const bool allow_preset_utility_door = !mapped_entrance && !thin_ring_footprint &&
										   condition != BuildingCondition::Construction &&
										   condition != BuildingCondition::Ruined &&
										   condition != BuildingCondition::Abandoned;
	if (min_level_offset == 0 && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage) {
		std::unordered_set<std::pair<int, int>, PairHash> rendered_entrances;
		for (std::size_t node_index = 0; node_index < element.nodes.size();
				++node_index) {
			const auto &node = element.nodes[node_index];
			if ((!node.tags.get("entrance").empty() &&
						node.tags.get("entrance") != "no") ||
					(!node.tags.get("door").empty() && node.tags.get("door") != "no")) {
				if (const auto level_it = node.tags.find("level");
						level_it != node.tags.end()) {
					std::string_view level_text = level_it->second;
					while (!level_text.empty() &&
							std::isspace(static_cast<unsigned char>(level_text.front())))
						level_text.remove_prefix(1);
					while (!level_text.empty() &&
							std::isspace(static_cast<unsigned char>(level_text.back())))
						level_text.remove_suffix(1);
					if (const auto level = strict_parse::f64(level_text);
							level && *level != 0.0)
						continue;
				}
				if (!rendered_entrances.emplace(node.x, node.z).second)
					continue;
				// An outline node belongs to the segment before and/or after it.
				// Rust uses the longer side to decide the exterior normal; that
				// avoids putting thresholds into a short return wall at corners.
				const building_facade::SegmentFacade *before =
						node_index > 0 && node_index - 1 < facade_plan.segments.size() &&
										facade_plan.segments[node_index - 1]
								? &*facade_plan.segments[node_index - 1]
								: nullptr;
				const building_facade::SegmentFacade *after =
						node_index < facade_plan.segments.size() &&
										facade_plan.segments[node_index]
								? &*facade_plan.segments[node_index]
								: nullptr;
				const building_facade::SegmentFacade *entrance_façade =
						before && after ? (before->len >= after->len ? before : after)
										: (before ? before : after);
				facade_plan.mark_door_column(node.x, node.z);
				const auto [door_block, unused_trapdoor] = entrance_door_style();
				(void)unused_trapdoor;
				if (entrance_façade) {
					const auto inward = std::pair{-entrance_façade->normal.first,
							-entrance_façade->normal.second};
					const bool ends_here =
							before && (!after || before->len >= after->len);
					const std::pair<int, int> tangent =
							ends_here ? std::pair{-entrance_façade->tangent.first,
												-entrance_façade->tangent.second}
									  : entrance_façade->tangent;
					const bool double_door =
							node.tags.get("entrance") == "main" &&
							(category == BuildingCategory::Commercial ||
									category == BuildingCategory::Office ||
									category == BuildingCategory::Hotel) &&
							entrance_façade->len >= 8 &&
							(entrance_façade->tangent.first == 0 ||
									entrance_façade->tangent.second == 0);
					bool second_leaf = false;
					put_entrance_door(door_block, node.x, node.z,
							entrance_façade->normal.first, entrance_façade->normal.second,
							"left");
					if (double_door &&
							!passage_at(effective_passages, node.x + tangent.first,
									node.z + tangent.second) &&
							!facade_plan.is_party(
									node.x + tangent.first, node.z + tangent.second)) {
						put_entrance_door(door_block, node.x + tangent.first,
								node.z + tangent.second, entrance_façade->normal.first,
								entrance_façade->normal.second, "right");
						facade_plan.mark_door_column(
								node.x + tangent.first, node.z + tangent.second);
						second_leaf = true;
					}
					interior_entries.push_back(
							{{node.x + inward.first, node.z + inward.second}, inward});
					if (!signage_anchor)
						signage_anchor = building_facade::FacadeAnchor{node.x, node.z,
								entrance_façade->normal, 0, 0, std::pair{node.x, node.z}};
					put_threshold(node.x, node.z, entrance_façade->normal.first,
							entrance_façade->normal.second);
					if (second_leaf)
						put_threshold(node.x + tangent.first, node.z + tangent.second,
								entrance_façade->normal.first,
								entrance_façade->normal.second);
				} else {
					// Still provide the interior with a usable entry when no road-based
					// facade plan exists: infer the inward neighbour from the actual
					// rasterized footprint rather than assuming a cardinal wall normal.
					std::optional<std::pair<int, int>> inward;
					int best_score = std::numeric_limits<int>::min();
					for (const auto direction : {std::pair{0, -1}, std::pair{1, 0},
								 std::pair{0, 1}, std::pair{-1, 0}}) {
						if (!current_footprint.contains({node.x + direction.first,
									node.z + direction.second}))
							continue;
						const int score = direction.first * (min_x + max_x - 2 * node.x) +
										  direction.second * (min_z + max_z - 2 * node.z);
						if (!inward || score > best_score) {
							inward = direction;
							best_score = score;
						}
					}
					if (inward) {
						put_entrance_door(door_block, node.x, node.z, -inward->first,
								-inward->second, "left");
						interior_entries.push_back(
								{{node.x + inward->first, node.z + inward->second},
										*inward});
					}
				}
			}
		}
		if (!mapped_entrance) {
			const bool corner_entrance =
					facade_plan.corner && (category == BuildingCategory::Commercial ||
												  category == BuildingCategory::Hotel ||
												  category == BuildingCategory::Office);
			std::optional<std::size_t> entrance_segment;
			if (corner_entrance) {
				const auto &corner = *facade_plan.corner;
				const auto length = [&](std::size_t i) {
					return i < facade_plan.segments.size() && facade_plan.segments[i]
								   ? facade_plan.segments[i]->len
								   : 0;
				};
				entrance_segment = length(corner.seg_a) >= length(corner.seg_b)
										   ? corner.seg_a
										   : corner.seg_b;
			} else if (facade_plan.front_segment) {
				entrance_segment = facade_plan.front_segment;
			} else {
				for (std::size_t i = 0; i < facade_plan.segments.size(); ++i)
					if (facade_plan.segments[i] &&
							facade_plan.segments[i]->facade_class !=
									building_facade::FacadeClass::Party &&
							(!entrance_segment ||
									facade_plan.segments[i]->len >
											facade_plan.segments[*entrance_segment]->len))
						entrance_segment = i;
			}
			if (entrance_segment && *entrance_segment + 1 < element.nodes.size() &&
					facade_plan.segments[*entrance_segment]) {
				const auto &a = element.nodes[*entrance_segment];
				const auto &b = element.nodes[*entrance_segment + 1];
				const auto &segment = *facade_plan.segments[*entrance_segment];
				auto points = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
				if (points.size() >= 5) {
					auto position_rng = element_rng(group_seed ^ 0xD00E57E900000011ULL);
					std::size_t target = points.size() / 2;
					if (corner_entrance) {
						const int from_corner = 2 + int(position_rng.uniform(2));
						const bool corner_at_end =
								facade_plan.corner->seg_a == *entrance_segment;
						target = corner_at_end
										 ? points.size() - 1 -
												   static_cast<std::size_t>(from_corner)
										 : static_cast<std::size_t>(from_corner);
					} else {
						const double jitter = position_rng.uniform_real(-.15, .15);
						target = static_cast<std::size_t>(
								static_cast<double>(points.size()) * (.5 + jitter));
					}
					const std::size_t center =
							std::clamp<std::size_t>(target, 2, points.size() - 3);
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
						const auto inward =
								std::pair{-segment.normal.first, -segment.normal.second};
						interior_entries.push_back({{door->first + inward.first,
															door->second + inward.second},
								inward});
						if (!signage_anchor)
							signage_anchor = building_facade::FacadeAnchor{door->first,
									door->second, segment.normal, 0, 0, *door};
						const auto [door_block, trapdoor_block] = entrance_door_style();
						facade_plan.mark_door_column(door->first, door->second);
						put_entrance_door(door_block, door->first, door->second,
								segment.normal.first, segment.normal.second, "left");
						const bool double_door =
								(category == BuildingCategory::Commercial ||
										category == BuildingCategory::Office ||
										category == BuildingCategory::Hotel ||
										category == BuildingCategory::School ||
										category == BuildingCategory::Hospital) &&
								segment.len >= 8 &&
								(segment.tangent.first == 0 ||
										segment.tangent.second == 0);
						if (double_door &&
								!passage_at(effective_passages,
										door->first + segment.tangent.first,
										door->second + segment.tangent.second) &&
								!facade_plan.is_party(door->first + segment.tangent.first,
										door->second + segment.tangent.second)) {
							put_entrance_door(door_block,
									door->first + segment.tangent.first,
									door->second + segment.tangent.second,
									segment.normal.first, segment.normal.second, "right");
							facade_plan.mark_door_column(
									door->first + segment.tangent.first,
									door->second + segment.tangent.second);
							put_threshold(door->first + segment.tangent.first,
									door->second + segment.tangent.second,
									segment.normal.first, segment.normal.second);
						}
						const bool canopy =
								(double_door ||
										category == BuildingCategory::Commercial ||
										category == BuildingCategory::Hotel) &&
								element_rng(group_seed ^ 0xD00E57E900000013ULL)
										.random_bool(.6);
						const bool lantern =
								!canopy &&
								(category == BuildingCategory::House ||
										category == BuildingCategory::Residential ||
										category == BuildingCategory::Historic) &&
								element_rng(group_seed ^ 0xD00E57E900000013ULL)
										.random_bool(.35);
						const auto set_trapdoor = [&](int x, int y, int z) {
							editor->set_block_with_properties_absolute(
									BlockWithProperties{trapdoor_block,
											{{"half", "top"}, {"open", "false"},
													{"facing",
															facing_name(
																	segment.normal.first,
																	segment.normal
																			.second)}}},
									x, y, z, std::vector<Block>{AIR}, std::nullopt);
						};
						if (canopy) {
							for (int t = -1; t <= (double_door ? 2 : 1); ++t)
								set_trapdoor(door->first + segment.normal.first +
													 segment.tangent.first * t,
										start_y_offset + abs_terrain_offset + 3,
										door->second + segment.normal.second +
												segment.tangent.second * t);
						} else if (lantern) {
							const int lantern_x = door->first + segment.normal.first;
							const int lantern_z = door->second + segment.normal.second;
							set_trapdoor(lantern_x,
									start_y_offset + abs_terrain_offset + 4, lantern_z);
							const BlockWithProperties hanging_lantern{
									LANTERN, {{"hanging", "true"}}};
							editor->set_block_with_properties_absolute(hanging_lantern,
									lantern_x, start_y_offset + abs_terrain_offset + 3,
									lantern_z, std::vector<Block>{AIR}, std::nullopt);
						}
						put_threshold(door->first, door->second, segment.normal.first,
								segment.normal.second);
					}
				}
			}
		}
	}

	// Rust gives small utility buildings a dedicated entrance instead of the
	// normal façade-door planner (which deliberately excludes these categories).
	if (min_level_offset == 0 && allow_preset_utility_door &&
			(category == BuildingCategory::Garage || category == BuildingCategory::Shed ||
					category == BuildingCategory::Greenhouse)) {
		const int door_y = start_y_offset + abs_terrain_offset + 1;
		if (category == BuildingCategory::Garage) {
			for (std::size_t i = 0; i + 1 < element.nodes.size(); ++i) {
				const auto &a = element.nodes[i], &b = element.nodes[i + 1];
				const int dx = std::abs(b.x - a.x), dz = std::abs(b.z - a.z);
				if (std::max(dx, dz) < 2)
					continue;
				const int x1 = (a.x + b.x) / 2, z1 = (a.z + b.z) / 2;
				const int x2 = dx > dz ? x1 + 1 : x1;
				const int z2 = dx > dz ? z1 : z1 + 1;
				if (passage_at(effective_passages, x1, z1) ||
						passage_at(effective_passages, x2, z2) ||
						mapillary::facades::photo_column(x1, z1, element.id) ||
						mapillary::facades::photo_column(x2, z2, element.id))
					continue;
				for (const auto &[x, z] : {std::pair{x1, z1}, std::pair{x2, z2}}) {
					editor->set_block_absolute(SPRUCE_DOOR_LOWER, x, door_y, z,
							std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{}));
					editor->set_block_absolute(SPRUCE_DOOR_UPPER, x, door_y + 1, z,
							std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{}));
					facade_plan.mark_door_column(x, z);
				}
				break;
			}
		} else if (!current_building.empty()) {
			auto door_rng = element_rng(element.id);
			const auto &[x, z] = current_building[door_rng.uniform(
					static_cast<std::uint32_t>(current_building.size()))];
			if (!passage_at(effective_passages, x, z) &&
					!mapillary::facades::photo_column(x, z, element.id)) {
				editor->set_block_absolute(OAK_DOOR, x, door_y, z, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				editor->set_block_absolute(OAK_DOOR_UPPER, x, door_y + 1, z, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				facade_plan.mark_door_column(x, z);
			}
		}
	}

	if (!signage_anchor && facade_plan.front_segment &&
			*facade_plan.front_segment + 1 < element.nodes.size() &&
			facade_plan.segments[*facade_plan.front_segment]) {
		const auto index = *facade_plan.front_segment;
		const auto &a = element.nodes[index], &b = element.nodes[index + 1];
		const auto &segment = *facade_plan.segments[index];
		const auto cells = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
		if (!cells.empty()) {
			const auto &middle = cells[cells.size() / 2];
			signage_anchor = building_facade::FacadeAnchor{std::get<0>(middle),
					std::get<2>(middle), segment.normal, 0, 0, std::nullopt};
		}
	}
	if (signage_anchor) {
		const int door_y = start_y_offset + abs_terrain_offset + 1;
		signage_anchor->fascia_y = std::max(
				start_y_offset + grammar_anchor + floor_cycle + abs_terrain_offset,
				door_y + 3);
		signage_anchor->number_y = door_y + 1;
	}

	if (std::get<2>(corner_addup) != 0) {
		const std::vector<std::pair<int, int>> &floor_area = cached_floor_area;
		const bool interior_allowed =
				args.interior && floor_area.size() >= 24 &&
				std::find(INTERIOR_EXCLUSIONS.begin(), INTERIOR_EXCLUSIONS.end(),
						std::string_view(building_type)) == INTERIOR_EXCLUSIONS.end() &&
				condition != BuildingCondition::Construction &&
				condition != BuildingCondition::Ruined;

		std::vector<int> floor_levels;
		floor_levels.push_back(start_y_offset);
		if (building_height > floor_cycle + 2) {
			int num_upper_floors = std::max(1, building_height / floor_cycle);
			for (int floor = 1; floor < num_upper_floors; ++floor) {
				floor_levels.push_back(
						start_y_offset + grammar_anchor + (floor * floor_cycle));
			}
		}
		static const std::vector<interior_uses::Tenant> no_interior_tenants;
		const auto &interior_tenants = interior_index
											   ? interior_index->tenants(element.id)
											   : no_interior_tenants;
		const auto interior_area =
				interior_index ? interior_index->area(element.id) : std::nullopt;
		std::optional<interior_uses::InteriorPlan> interior_plan;
		if (interior_allowed) {
			const interior_uses::PlanInputs plan_inputs{element.tags, building_type,
					floor_levels.size(), min_level, min_level_offset > 0,
					floor_area.size(), {(min_x + max_x) / 2, (min_z + max_z) / 2},
					interior_tenants, interior_area};
			interior_plan = interior_uses::plan_interior(plan_inputs);
		}
		const bool open_hall = interior_plan && interior_plan->open_hall;
		// Rust gives each unit in a shared ground-floor tenancy its own street
		// entrance when the existing mapped/synthetic doors leave it unserved.
		// Keep the entrance selection tied to the same planned unit anchors and
		// facade-segment ranking so multi-shop buildings remain navigable.
		const bool preset_utility_door =
				allow_preset_utility_door &&
				(category == BuildingCategory::Garage ||
						category == BuildingCategory::Shed ||
						category == BuildingCategory::Greenhouse);
		if (min_level_offset == 0 && interior_plan && !interior_plan->floors.empty() &&
				interior_plan->floors.front().size() >= 2 &&
				condition != BuildingCondition::Construction &&
				condition != BuildingCondition::Ruined && !preset_utility_door &&
				!facade_plan.segments.empty()) {
			const auto &units = interior_plan->floors.front();
			const auto unit_of = [&](int x, int z) {
				std::size_t nearest = 0;
				std::int64_t nearest_distance = std::numeric_limits<std::int64_t>::max();
				for (std::size_t i = 0; i < units.size(); ++i) {
					const auto dx = std::int64_t(x) - units[i].anchor.first;
					const auto dz = std::int64_t(z) - units[i].anchor.second;
					const auto distance = dx * dx + dz * dz;
					if (distance < nearest_distance) {
						nearest = i;
						nearest_distance = distance;
					}
				}
				return nearest;
			};
			std::vector<std::pair<int, int>> doors;
			doors.reserve(interior_entries.size());
			std::vector<bool> served(units.size(), false);
			for (const auto &entry : interior_entries) {
				const auto door = std::pair{entry.cell.first - entry.inward.first,
						entry.cell.second - entry.inward.second};
				doors.push_back(door);
				served[unit_of(door.first + entry.inward.first,
						door.second + entry.inward.second)] = true;
			}
			const auto [door_block, unused_trapdoor] = entrance_door_style();
			(void)unused_trapdoor;
			for (std::size_t unit = 0; unit < units.size(); ++unit) {
				if (served[unit])
					continue;
				using Candidate = std::tuple<int, std::int64_t, int, int, std::size_t>;
				std::optional<Candidate> best;
				for (std::size_t i = 0; i < facade_plan.segments.size(); ++i) {
					if (!facade_plan.segments[i] || i + 1 >= element.nodes.size())
						continue;
					const auto &segment = *facade_plan.segments[i];
					int rank = 0;
					switch (segment.facade_class) {
					case building_facade::FacadeClass::Street:
						rank = 0;
						break;
					case building_facade::FacadeClass::Open:
						rank = 1;
						break;
					case building_facade::FacadeClass::Rear:
						rank = 2;
						break;
					case building_facade::FacadeClass::Party:
						continue;
					}
					const auto &a = element.nodes[i];
					const auto &b = element.nodes[i + 1];
					const auto points = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
					if (segment.len < 4 || points.size() < 5)
						continue;
					for (std::size_t p = 2; p + 2 < points.size(); ++p) {
						const int x = std::get<0>(points[p]);
						const int z = std::get<2>(points[p]);
						if (passage_at(effective_passages, x, z) ||
								facade_plan.is_party(x, z) ||
								unit_of(x - segment.normal.first,
										z - segment.normal.second) != unit)
							continue;
						if (std::any_of(
									doors.begin(), doors.end(), [&](const auto &door) {
										return std::abs(door.first - x) +
													   std::abs(door.second - z) <
											   3;
									}))
							continue;
						const auto dx = std::int64_t(x) - units[unit].anchor.first;
						const auto dz = std::int64_t(z) - units[unit].anchor.second;
						const Candidate candidate{rank, dx * dx + dz * dz, x, z, i};
						// Rust compares only (wall rank, distance) and keeps the
						// first candidate on ties. Do not let coordinates or segment
						// indices alter that stable iteration-order choice.
						if (!best ||
								std::pair{
										std::get<0>(candidate), std::get<1>(candidate)} <
										std::pair{std::get<0>(*best), std::get<1>(*best)})
							best = candidate;
					}
				}
				if (!best)
					continue;
				const auto [rank, distance, x, z, segment_index] = *best;
				(void)rank;
				(void)distance;
				const auto &segment = *facade_plan.segments[segment_index];
				facade_plan.mark_door_column(x, z);
				facade_plan.mark_door_column(
						x + segment.normal.first, z + segment.normal.second);
				put_entrance_door(door_block, x, z, segment.normal.first,
						segment.normal.second, "left");
				put_threshold(x, z, segment.normal.first, segment.normal.second);
				interior_entries.push_back(
						{{x - segment.normal.first, z - segment.normal.second},
								{-segment.normal.first, -segment.normal.second}});
				doors.emplace_back(x, z);
			}
		}
		if (open_hall && floor_levels.size() > 1)
			floor_levels.resize(1);

		for (const auto &p : floor_area) {
			int x = p.first;
			int z = p.second;
			if (processed_points.insert(p).second) {
				const bool is_passage = passage_at(effective_passages, x, z);
				const Block ceiling_light_block =
						condition == BuildingCondition::Abandoned ||
										condition == BuildingCondition::Ruined
								? COBWEB
								: GLOWSTONE;
				if (!is_passage) {
					editor->set_block_absolute(
							floor_block, x, start_y_offset + abs_terrain_offset, z);
				}
				const int top = start_y_offset + building_height;
				int top_floor = start_y_offset;
				if (building_height > floor_cycle && !open_hall) {
					for (int h = start_y_offset + grammar_anchor + floor_cycle; h < top;
							h += floor_cycle) {
						top_floor = h;
						if (is_passage &&
								h <= start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
															  building_height))
							continue;
						const Block block = (x % 3) == 0 && (z % 3) == 0
													? ceiling_light_block
													: floor_block;
						editor->set_block_absolute(block, x, h + abs_terrain_offset, z);
					}
				}
				const int top_headroom = top - top_floor;
				const bool under_passage =
						is_passage &&
						top <= start_y_offset + std::min(BUILDING_PASSAGE_HEIGHT,
														building_height);
				if ((x % 3) == 0 && (z % 3) == 0 && !under_passage &&
						(top_headroom >= 3 || top_floor == start_y_offset))
					editor->set_block_absolute(
							ceiling_light_block, x, top + abs_terrain_offset, z);

				if (is_passage && building_height > BUILDING_PASSAGE_HEIGHT)
					editor->set_block_absolute(floor_block, x,
							start_y_offset + BUILDING_PASSAGE_HEIGHT + abs_terrain_offset,
							z);
			}
		}

		if (!element.tags.contains("building:part"))
			generate_corner_downpipes(*editor, element, facade_plan, category, condition,
					has_windows, min_level_offset == 0, start_y_offset, building_height,
					abs_terrain_offset, wall_block, effective_passages, &preset_shell);
		if (!element.tags.contains("building:part"))
			generate_corner_quoins(*editor, element, facade_plan, category, condition,
					wall_block, accent_block, start_y_offset, building_height,
					abs_terrain_offset, effective_passages, element.id, &preset_shell);
		generate_hospital_wall_cross(*editor, element, category, start_y_offset,
				building_height, abs_terrain_offset);

		if (interior_allowed) {
			const bool is_abandoned_building =
					condition == BuildingCondition::Abandoned ||
					condition == BuildingCondition::Ruined;

			CoordinateBitmap no_passages = CoordinateBitmap::new_empty();
			const CoordinateBitmap &interior_passages =
					effective_passages ? *effective_passages : no_passages;
			static const std::vector<interior_uses::Claim> no_interior_claims;
			const auto &interior_claims = interior_index
												  ? interior_index->claims(element.id)
												  : no_interior_claims;
			const InteriorRequest interior_request{floor_area, floor_levels,
					start_y_offset, building_height, abs_terrain_offset, wall_block,
					interior_plan ? &*interior_plan : nullptr, is_abandoned_building,
					interior_passages, interior_entries, {{min_x, min_z}, {max_x, max_z}},
					interior_claims, args.scale, floor_block, clean_visual_seed};
			generate_building_interior(*editor, interior_request);
		}
	}

	const std::optional<Block> roof_block_override =
			category == BuildingCategory::Shed
					? std::optional<Block>(OAK_PLANKS)
					: (category == BuildingCategory::Greenhouse
									  ? std::optional<Block>(SMOOTH_STONE_SLAB)
									  : std::nullopt);
	bool generated_sloped_roof = false;
	RoofType generated_roof_type = RoofType::Flat;
	std::optional<bool> preferred_ridge_along_x;
	if (building_type == "terrace" || building_type == "semidetached_house") {
		if (facade_plan.front_segment &&
				*facade_plan.front_segment < facade_plan.segments.size() &&
				facade_plan.segments[*facade_plan.front_segment]) {
			const auto &front = *facade_plan.segments[*facade_plan.front_segment];
			preferred_ridge_along_x = front.normal.first == 0;
		}
	}
	if (args.roof && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined) {
		const bool preset_flat_roof =
				category == BuildingCategory::ModernSkyscraper ||
				category == BuildingCategory::GlassySkyscraper ||
				category == BuildingCategory::GlassCornerSkyscraper ||
				category == BuildingCategory::GridSkyscraper ||
				category == BuildingCategory::ContemporarySkyscraper ||
				category == BuildingCategory::MasonrySkyscraper ||
				category == BuildingCategory::Tower ||
				category == BuildingCategory::Garage ||
				category == BuildingCategory::Greenhouse;
		auto it_shape = element.tags.find("roof:shape");
		if (it_shape != element.tags.end()) {
			RoofType roof_type = parse_roof_type(it_shape->second);
			generated_roof_type = roof_type;
			generated_sloped_roof = roof_type != RoofType::Flat;

			const std::optional<Block> curtain_glass =
					is_glass_wall_block(wall_block) && is_glass_wall_block(window_block)
							? std::optional<Block>(window_block)
							: std::nullopt;
			generate_roof(*editor, element, start_y_offset, building_height, floor_block,
					wall_block, accent_block, roof_block_override, curtain_glass,
					group_seed, roof_dormers_enabled(condition, category, roof_type),
					category == BuildingCategory::Religious, preferred_ridge_along_x,
					roof_type, roof_area, abs_terrain_offset, args.scale);
		} else if (preset_flat_roof) {
			// These Rust BuildingStylePreset variants request a real flat cap even
			// without roof:shape (notably towers, garages, greenhouses and skyscrapers).
			const std::optional<Block> curtain_glass =
					is_glass_wall_block(wall_block) && is_glass_wall_block(window_block)
							? std::optional<Block>(window_block)
							: std::nullopt;
			generate_roof(*editor, element, start_y_offset, building_height, floor_block,
					wall_block, accent_block, roof_block_override, curtain_glass,
					group_seed, false, category == BuildingCategory::Religious,
					preferred_ridge_along_x, RoofType::Flat, roof_area,
					abs_terrain_offset, args.scale);
		} else if (!part_has_explicit_top) {
			std::string btype = "yes";
			auto it = element.tags.find("building");
			if (it != element.tags.end())
				btype = it->second;

			const bool auto_roof_type =
					category != BuildingCategory::TallBuilding &&
					category != BuildingCategory::Hospital &&
					category != BuildingCategory::Historic &&
					(btype == "apartments" || btype == "residential" ||
							btype == "house" || btype == "yes" || btype == "detached" ||
							btype == "semidetached_house" || btype == "terrace" ||
							btype == "bungalow" || btype == "villa" || btype == "cabin" ||
							btype == "hut" || btype == "farm" ||
							btype == "farm_auxiliary" || btype == "barn" ||
							btype == "stable" || btype == "cowshed" || btype == "sty" ||
							btype == "sheepfold");
			if (auto_roof_type) {
				std::size_t footprint_size = cached_footprint_size;
				const std::size_t max_footprint_for_gabled = 800;
				auto roof_rng = element_rng(group_seed ^ 0x0F1E2D3C4B5A6907ULL);
				const bool big_block = footprint_size > max_footprint_for_gabled ||
									   building_height >= 15;
				RoofType automatic_roof = RoofType::Flat;
				bool make_roof = false;
				if (btype == "apartments" && big_block) {
					const bool prewar = era == ArchEra::HistoricOrnate ||
										era == ArchEra::TraditionalPreWar;
					const auto roll = roof_rng.uniform(100);
					const int hip_limit = prewar ? 65 : 80;
					if (roll >= 45) {
						automatic_roof =
								roll < hip_limit ? RoofType::Hipped : RoofType::Mansard;
						make_roof = true;
					}
				} else if ((era == ArchEra::HistoricOrnate ||
								   era == ArchEra::TraditionalPreWar) &&
						   footprint_size >= 100 &&
						   footprint_size <= max_footprint_for_gabled &&
						   roof_rng.random_bool(.30)) {
					automatic_roof = RoofType::Mansard;
					make_roof = true;
				} else {
					const double gable_probability =
							(climate == biome::Climate::HotDesert ||
									climate == biome::Climate::HotSteppe)
									? .35
							: climate == biome::Climate::TropicalSavanna ? .60
							: (climate == biome::Climate::Boreal ||
									  climate == biome::Climate::Tundra ||
									  climate == biome::Climate::IceCap)
									? .95
									: .90;
					if (footprint_size <= max_footprint_for_gabled &&
							roof_rng.random_bool(gable_probability)) {
						automatic_roof = RoofType::Gabled;
						make_roof = true;
					}
				}
				if (make_roof &&
						(automatic_roof == RoofType::Gabled ||
								automatic_roof == RoofType::Hipped ||
								automatic_roof == RoofType::Mansard ||
								automatic_roof == RoofType::Gambrel) &&
						compute_building_diagonality(element.nodes) < 0.35 &&
						std::abs(dominant_building_axis_angle(element.nodes)) * 180.0 /
										std::numbers::pi >
								10.0)
					automatic_roof = RoofType::Pyramidal;
				if (make_roof) {
					generated_roof_type = automatic_roof;
					generated_sloped_roof = true;
					const std::optional<Block> curtain_glass =
							is_glass_wall_block(wall_block) &&
											is_glass_wall_block(window_block)
									? std::optional<Block>(window_block)
									: std::nullopt;
					generate_roof(*editor, element, start_y_offset, building_height,
							floor_block, wall_block, accent_block, roof_block_override,
							curtain_glass, group_seed,
							roof_dormers_enabled(condition, category, automatic_roof),
							category == BuildingCategory::Religious,
							preferred_ridge_along_x, automatic_roof, roof_area,
							abs_terrain_offset, args.scale);
				}
			} else if ((btype == "industrial" || btype == "warehouse" ||
							   btype == "hangar") &&
					   cached_footprint_size > 800) {
				auto roof_rng = element_rng(group_seed ^ 0x0F1E2D3C4B5A6907ULL);
				if (roof_rng.random_bool(.55)) {
					generated_roof_type = RoofType::Skillion;
					generated_sloped_roof = true;
					const std::optional<Block> curtain_glass =
							is_glass_wall_block(wall_block) &&
											is_glass_wall_block(window_block)
									? std::optional<Block>(window_block)
									: std::nullopt;
					generate_roof(*editor, element, start_y_offset, building_height,
							floor_block, wall_block, accent_block, roof_block_override,
							curtain_glass, group_seed,
							roof_dormers_enabled(condition, category, RoofType::Skillion),
							category == BuildingCategory::Religious,
							preferred_ridge_along_x, RoofType::Skillion, roof_area,
							abs_terrain_offset, args.scale);
				}
			}
		}
	} else {
		// flat roof default - already applied
	}
	// Apply the Rust wall-depth feature pass after roof selection so all styles
	// use the same roof-aware cutoff and wall-aligned facade coordinates.
	const bool glass_curtain_depth_active =
			wall_depth_style == WallDepthStyle::GlassCurtain &&
			element_rng(visual_seed + 7919).uniform(100) < 40;
	const bool roof_depth_style_active =
			wall_depth_span_ok && wall_depth_height_ok &&
			(wall_depth_style == WallDepthStyle::HistoricOrnate ||
					wall_depth_style == WallDepthStyle::ReligiousButtress ||
					wall_depth_style == WallDepthStyle::IndustrialBeams ||
					glass_curtain_depth_active || wall_depth_piers_active);
	if (roof_depth_style_active && condition == BuildingCondition::Normal &&
			!element.nodes.empty() && max_x - min_x >= 4 && max_z - min_z >= 4) {
		const int height_reduction = generated_sloped_roof ? 2 : 0;
		const int top_h = start_y_offset + building_height - height_reduction;
		const BlockWithProperties sill_block{
				get_slab_block_for_material(wall_block), {{"type", "top"}}};
		const auto stair_for_outward = [&](int nx, int nz) {
			const StairFacing facing = nx > 0	? StairFacing::East
									   : nx < 0 ? StairFacing::West
									   : nz > 0 ? StairFacing::South
												: StairFacing::North;
			return top_stair(
					create_stair_with_properties(get_stair_block_for_material(wall_block),
							facing, StairShape::Straight));
		};
		const auto floor_row_at = [&](int y) {
			return ((y - start_y_offset - grammar_anchor) % floor_cycle + floor_cycle) %
				   floor_cycle;
		};
		for (std::size_t node_index = 1; node_index < element.nodes.size();
				++node_index) {
			const auto &a = element.nodes[node_index - 1];
			const auto &b = element.nodes[node_index];
			if (a.x == b.x && a.z == b.z)
				continue;
			if (node_index - 1 >= facade_plan.segments.size() ||
					!facade_plan.segments[node_index - 1])
				continue;
			const auto outward = facade_plan.segments[node_index - 1]->normal;
			const int nx = outward.first, nz = outward.second;
			if (nx == 0 && nz == 0)
				continue;
			const bool segment_is_x = segment_axis_x(a, b);
			const bool start_is_x =
					segment_start_axis_x(element.nodes, node_index, segment_is_x);
			const auto points =
					bresenham_line(a.x, start_y_offset, a.z, b.x, start_y_offset, b.z);
			for (std::size_t point_index = 0; point_index < points.size();
					++point_index) {
				const int bx = std::get<0>(points[point_index]);
				const int bz = std::get<2>(points[point_index]);
				const int lx = bx + nx, lz = bz + nz;
				if (passage_at(effective_passages, bx, bz) ||
						facade_plan.is_party(bx, bz) || facade_plan.is_door(bx, bz) ||
						is_flat_column(bx, bz))
					continue;
				if (element.tags.contains("building:part")) {
					const auto blocked = [&](int x, int z) {
						return building_footprints &&
							   building_footprints->contains(x, z) &&
							   !current_footprint.contains({x, z});
					};
					if (sibling_cells.contains({lx, lz}) ||
							sibling_cells.contains({lx + nx, lz + nz}) ||
							blocked(lx, lz) || blocked(lx + nx, lz + nz))
						continue;
				}
				int descent = 0;
				if (min_level_offset == 0) {
					std::optional<int> ground;
					for (const auto &p : {std::pair{bx, bz}, std::pair{lx, lz}})
						if (const auto y = editor->terrain_level(p.first, p.second))
							ground = ground ? std::min(*ground, *y) : *y;
					if (ground)
						descent = std::max(0, start_y_offset - *ground);
				}
				const int wall_u =
						wall_ordinate(point_index, start_is_x, segment_is_x, bx, bz);
				const int mod6 = ((wall_u + window_phase) % 6 + 6) % 6;
				const auto place_absolute = [&](const Block &block, int y) {
					editor->set_block_absolute(block, lx, y + abs_terrain_offset, lz,
							std::vector<Block>{AIR});
				};
				const auto place_property = [&](const BlockWithProperties &block, int y) {
					editor->set_block_with_properties_absolute(block, lx,
							y + abs_terrain_offset, lz, std::vector<Block>{AIR},
							std::nullopt);
				};
				const auto place_vertical = [&](const Block &block) {
					for (int h = start_y_offset + 1 - descent; h <= top_h; ++h)
						place_absolute(block, h);
				};
				const auto place_foundation = [&]() {
					for (int h = start_y_offset + 1 - descent; h <= start_y_offset + 1;
							++h)
						place_absolute(accent_block, h);
				};
				const auto place_floor_slab_bands = [&]() {
					for (int h = start_y_offset + 2; h <= top_h; ++h)
						if (floor_row_at(h) == 0)
							place_property(sill_block, h);
				};
				const auto place_floor_stair_bands = [&]() {
					const auto stair = stair_for_outward(nx, nz);
					for (int h = start_y_offset + 2; h <= top_h; ++h)
						if (floor_row_at(h) == 0)
							place_property(stair, h);
				};
				switch (wall_depth_style) {
				case WallDepthStyle::SubtlePilasters:
					if (mod6 == 3) {
						for (int h = start_y_offset + 1 - descent; h <= top_h; ++h)
							place_absolute(
									h == start_y_offset + 1 ? accent_block : wall_block,
									h);
					}
					break;
				case WallDepthStyle::ModernPillars:
					if (mod6 == 3 || mod6 == 5) {
						place_vertical(accent_block);
					} else if (mod6 < 3) {
						place_foundation();
						place_floor_slab_bands();
					}
					break;
				case WallDepthStyle::InstitutionalBands:
					if (mod6 == 3) {
						place_vertical(accent_block);
					} else {
						place_foundation();
						if (mod6 < 3)
							place_floor_stair_bands();
					}
					break;
				case WallDepthStyle::IndustrialBeams:
					if (point_index < 2 ||
							point_index >= points.size() - std::min<std::size_t>(
																   2, points.size()))
						place_vertical(wall_block);
					break;
				case WallDepthStyle::HistoricOrnate:
					if (mod6 == 3) {
						place_vertical(wall_block);
						if (height_reduction == 0)
							place_property(stair_for_outward(nx, nz), top_h + 1);
					} else {
						place_foundation();
						if (mod6 == 0 || mod6 == 2) {
							const auto stair = stair_for_outward(nx, nz);
							for (int h = start_y_offset + 2; h <= top_h; ++h)
								if (floor_row_at(h) == floor_cycle - 1)
									place_property(stair, h);
						}
						if (height_reduction == 0)
							place_property(stair_for_outward(nx, nz), top_h + 1);
					}
					break;
				case WallDepthStyle::ReligiousButtress: {
					const int bay = floor_div(wall_u + window_phase, 6);
					if (mod6 == 0 && ((bay % 2 + 2) % 2) == 0) {
						place_vertical(wall_block);
						const int cutoff = start_y_offset + building_height * 3 / 5;
						// The outer buttress sits two cells from the wall; place it with
						// the same air-only replacement rule as the inner layer.
						const int outer_x = bx + nx * 2, outer_z = bz + nz * 2;
						for (int h = start_y_offset + 1 - descent; h <= cutoff; ++h)
							editor->set_block_absolute(wall_block, outer_x,
									h + abs_terrain_offset, outer_z,
									std::vector<Block>{AIR});
					} else if (height_reduction == 0) {
						place_property(stair_for_outward(nx, nz), top_h + 1);
					}
					break;
				}
				case WallDepthStyle::SkyscraperFins:
					place_foundation();
					if (mod6 == 3)
						place_vertical(accent_block);
					else
						place_floor_slab_bands();
					break;
				case WallDepthStyle::GlassCurtain:
					if (point_index == 0 || point_index + 1 == points.size())
						place_vertical(accent_block);
					break;
				case WallDepthStyle::None:
					break;
				}
			}
		}
	}
	// Resolve the roof first: automatic roofs are randomized by type, so an
	// earlier tag-only estimate can leave a flat ceiling beneath a pitched roof
	// or leave an open top when the automatic roof roll chose flat.
	const bool skip_top = condition == BuildingCondition::Construction ||
						  condition == BuildingCondition::Ruined;
	const bool covered_by_sibling =
			element.tags.contains("building:part") && !sibling_cells.empty() &&
			std::any_of(cached_floor_area.begin(), cached_floor_area.end(),
					[&](const auto &cell) { return sibling_cells.contains(cell); });
	const bool modeled_part_roof = element.tags.contains("building:part") &&
								   (element.tags.contains("roof:colour") ||
										   element.tags.contains("roof:material"));
	const bool has_parapet = category_has_parapet_preset(category) ||
							 short_flat_parapet_for(group_seed, generated_roof_type,
									 building_height, category, condition);
	if (!generated_sloped_roof && !skip_top) {
		for (const auto &[x, z] : cached_floor_area)
			editor->set_block_absolute(floor_block, x,
					start_y_offset + building_height + abs_terrain_offset + 1, z);
	}
	if (args.roof && !podium_tower && !skip_top &&
			generated_roof_type == RoofType::Flat && has_parapet && !covered_by_sibling)
		generate_parapet(*editor, element, start_y_offset, building_height,
				abs_terrain_offset, wall_block, accent_block, category);
	if (!generated_sloped_roof && args.roof && !podium_tower && !has_parapet &&
			!modeled_part_roof && !covered_by_sibling)
		generate_flat_roof_edge_variation(*editor, element, start_y_offset,
				building_height, abs_terrain_offset, wall_block, accent_block);
	if (!element.tags.contains("building:part"))
		generate_facade_cornice(*editor, element, facade_plan, condition, accent_block,
				start_y_offset, building_height, abs_terrain_offset, floor_cycle,
				effective_passages, generated_sloped_roof, &preset_shell);
	const bool generated_chimney_roof = generated_roof_type == RoofType::Gabled ||
										generated_roof_type == RoofType::Hipped ||
										generated_roof_type == RoofType::Mansard ||
										generated_roof_type == RoofType::Gambrel ||
										generated_roof_type == RoofType::HalfHipped;
	if (generated_sloped_roof && generated_chimney_roof && style_has_chimney)
		generate_chimney(*editor, roof_area, min_x, max_x, min_z, max_z,
				start_y_offset + building_height + 1, abs_terrain_offset, element.id);
	if (podium_tower) {
		const auto distances = roof_edge_distances(roof_area);
		const std::vector<InsetTier> tower{{podium_tower->inset,
				podium_tower->full_height - podium_tower->podium_height, true}};
		generate_inset_tiers(*editor, roof_area, distances,
				start_y_offset + building_height + 1, abs_terrain_offset, tower,
				floor_cycle, wall_block, window_block, floor_block, has_windows);
	}
	const bool has_crown = args.roof && !podium_tower && !generated_sloped_roof &&
						   !element.tags.contains("building:part") &&
						   generate_setback_crown(*editor, roof_area, category, condition,
								   is_tall_building, start_y_offset, building_height,
								   abs_terrain_offset, floor_cycle, wall_block,
								   window_block, floor_block, has_windows, element.id);
	const bool residential_antenna_roof =
			generated_sloped_roof && category == BuildingCategory::House &&
			condition == BuildingCondition::Normal &&
			(generated_roof_type == RoofType::Gabled ||
					generated_roof_type == RoofType::Hipped ||
					generated_roof_type == RoofType::Pyramidal);
	if (args.roof && !podium_tower && !has_crown &&
			(!modeled_part_roof || residential_antenna_roof))
		generate_rooftop_systems(*editor, element, roof_area, category, condition,
				generated_roof_type, start_y_offset, building_height, abs_terrain_offset,
				floor_cycle, wall_block, detail, clean_visual_seed, covered_by_sibling);
	if (category == BuildingCategory::Hospital && !generated_sloped_roof)
		generate_hospital_helipad(*editor, element, roof_area, start_y_offset,
				building_height, abs_terrain_offset);
	mapillary::facades::collect_displays(*editor, element.id, start_y_offset,
			abs_terrain_offset, building_height, args.facade_px);
	return signage_anchor;
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
		Block wall_block, Block accent_block, std::optional<Block> roof_block_override,
		std::optional<Block> curtain_glass, std::uint64_t style_seed, bool add_dormers,
		bool steep_gable, std::optional<bool> preferred_ridge_along_x, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset, double scale_factor)
{
	using std::int32_t;
	const auto &floor_area = cached_floor_area;
	auto fill_shell = [&](const std::unordered_map<std::pair<int32_t, int32_t>, int32_t,
								  pair_hash> &surface,
							  Block roof_block, int base_y) {
		for (const auto &[cell, top] : surface) {
			const auto [x, z] = cell;
			int bottom = top;
			for (const auto &[nx, nz] :
					{std::pair{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}}) {
				auto it = surface.find({nx, nz});
				if (it != surface.end())
					bottom = std::min(bottom, it->second - 1);
			}
			bottom = std::max(base_y, bottom);
			for (int y = bottom; y <= top; ++y)
				editor.set_block_absolute(roof_block, x, y + abs_terrain_offset, z,
						std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
		}
	};

	// Pre-calculate bounds
	int32_t min_x = std::numeric_limits<int32_t>::max();
	int32_t max_x = std::numeric_limits<int32_t>::min();
	int32_t min_z = std::numeric_limits<int32_t>::max();
	int32_t max_z = std::numeric_limits<int32_t>::min();

	for (const auto &[x, z] : floor_area) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}

	int32_t center_x = (min_x + max_x) >> 1;
	int32_t center_z = (min_z + max_z) >> 1;
	int32_t base_height = start_y_offset + building_height + 1;
	const std::optional<int> peak_cap = rust_roof_peak_cap(element.tags, element.nodes,
			roof_type, max_x - min_x, max_z - min_z, building_height, scale_factor,
			preferred_ridge_along_x);

	// Random generator
	auto rng = element_rng(style_seed ^ 0xF00FC010BA5EF00DULL);
	const std::optional<Block> tagged_roof_block =
			roof_surface_block_from_tags(element, rng);
	// Match Rust precedence: OSM tag, category preset, then the wall-derived roof.
	const auto selected_roof_block = [&]() {
		if (tagged_roof_block) {
			if (!is_glass_wall_block(*tagged_roof_block))
				return *tagged_roof_block;
			if (curtain_glass)
				return *curtain_glass;
			if (is_glass_wall_block(wall_block))
				return wall_block;
			return *tagged_roof_block;
		}
		if (roof_block_override)
			return *roof_block_override;
		return default_roof_surface_block(wall_block, style_seed);
	};

	if (roof_type == RoofType::Flat) {
		const Block flat_roof_block =
				tagged_roof_block.value_or(roof_block_override.value_or(floor_block));
		for (auto const &p : floor_area) {
			editor.set_block_absolute(flat_roof_block, p.first,
					base_height + abs_terrain_offset, p.second, nullptr, nullptr);
		}
		return;
	}

	if (roof_type == RoofType::Gabled || roof_type == RoofType::Gambrel ||
			roof_type == RoofType::HalfHipped || roof_type == RoofType::Round) {
		generate_gabled_roof_profile(editor, floor_area, element.nodes, element.tags,
				base_height, abs_terrain_offset, building_height, roof_type, steep_gable,
				preferred_ridge_along_x, peak_cap, style_seed, add_dormers,
				selected_roof_block());
		return;
	}

	if (roof_type == RoofType::Gabled) {
		int32_t width = max_x - min_x;
		int32_t length = max_z - min_z;
		int32_t building_size = std::max(width, length);
		bool is_wider_than_long = width > length;
		std::optional<double> mapped_slope_bearing;
		bool has_explicit_orientation = false;
		if (auto it = element.tags.find("roof:orientation"); it != element.tags.end()) {
			if (it->second == "along") {
				is_wider_than_long = width >= length;
				has_explicit_orientation = true;
			} else if (it->second == "across") {
				is_wider_than_long = width < length;
				has_explicit_orientation = true;
			}
		}
		if (!has_explicit_orientation && preferred_ridge_along_x)
			is_wider_than_long = *preferred_ridge_along_x;
		if (auto it = element.tags.find("roof:direction"); it != element.tags.end()) {
			if (auto degrees = parse_roof_direction_degrees(it->second)) {
				mapped_slope_bearing = *degrees;
				const double radians = *degrees * std::numbers::pi / 180.0;
				// OSM bearings use north=0 and east=90.  The ridge is
				// perpendicular to the requested downhill direction.
				is_wider_than_long =
						std::abs(std::cos(radians)) >= std::abs(std::sin(radians));
			}
		}
		// For rotated near-rectangular footprints, infer the downhill bearing
		// from the longest boundary edge when no explicit direction is mapped.
		if (!mapped_slope_bearing && !has_explicit_orientation &&
				!preferred_ridge_along_x && element.nodes.size() > 2) {
			double best_len = 0.0, best_angle = 0.0;
			for (std::size_t i = 0; i < element.nodes.size(); ++i) {
				const auto &a = element.nodes[i];
				const auto &b = element.nodes[(i + 1) % element.nodes.size()];
				const double dx = b.x - a.x, dz = b.z - a.z;
				const double len = dx * dx + dz * dz;
				if (len > best_len) {
					best_len = len;
					best_angle = std::atan2(dz, dx) * 180.0 / std::numbers::pi + 90.0;
				}
			}
			const double nearest_axis = std::fmod(std::abs(best_angle), 90.0);
			if (best_len > 16.0 && nearest_axis > 5.0 && nearest_axis < 85.0)
				mapped_slope_bearing = best_angle;
		}
		int32_t max_distance = is_wider_than_long ? (length >> 1) : (width >> 1);
		if (mapped_slope_bearing) {
			const double radians = *mapped_slope_bearing * std::numbers::pi / 180.0;
			max_distance = std::max(
					1, int(std::lround(std::abs(std::sin(radians)) * width / 2.0 +
									   std::abs(std::cos(radians)) * length / 2.0)));
		}

		int32_t roof_height_boost = static_cast<int32_t>(
				3.0 + std::max(1.0, std::log(std::max(1.0,
											static_cast<double>(building_size) * 0.15))));
		if (peak_cap)
			roof_height_boost = *peak_cap;
		else {
			const double cap_factor = steep_gable ? 0.9 : 0.6;
			const int wall_cap =
					std::max(1, int(std::lround(building_height * cap_factor)));
			roof_height_boost = std::min(roof_height_boost, wall_cap);
		}
		int32_t roof_peak_height = base_height + roof_height_boost;

		Block roof_block = selected_roof_block();

		std::vector<std::pair<std::pair<int32_t, int32_t>, int32_t>> roof_heights;
		roof_heights.reserve(floor_area.size());

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			int32_t distance_to_ridge;
			if (mapped_slope_bearing) {
				const double radians = *mapped_slope_bearing * std::numbers::pi / 180.0;
				distance_to_ridge =
						int(std::lround(std::abs((x - center_x) * std::sin(radians) +
												 (z - center_z) * std::cos(radians))));
			} else {
				distance_to_ridge = is_wider_than_long ? std::abs(z - center_z)
													   : std::abs(x - center_x);
			}

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
			bool steep_drop = false;
			std::pair<int32_t, int32_t> ncoords[4] = {
					{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
			for (auto const &nc : ncoords) {
				auto it = roof_map.find(nc);
				if (it != roof_map.end() && it->second < roof_height) {
					has_lower_neighbor = true;
					steep_drop |= roof_height - it->second > 2;
				}
			}

			for (int32_t y = base_height; y <= roof_height; ++y) {
				if (y == roof_height && has_lower_neighbor && !steep_drop) {
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
				const bool at_end = !mapped_slope_bearing && is_wider_than_long
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
		place_gabled_eave_overhang(editor, floor_area, base_height, abs_terrain_offset,
				roof_block, is_wider_than_long);
		place_roof_dormer_windows(editor, floor_area, roof_map, base_height,
				abs_terrain_offset, style_seed, roof_block, add_dormers,
				is_wider_than_long ? std::pair{1, 0} : std::pair{0, 1});

		return;
	}

	if (roof_type == RoofType::Round) {
		// Rust's round roof is a barrel vault: the cross-section is a
		// semicircle over the short axis, rather than a four-sided hip.
		const int width = std::max(1, max_x - min_x);
		const int length = std::max(1, max_z - min_z);
		bool axis_x = preferred_ridge_along_x.value_or(width >= length);
		std::optional<double> mapped_bearing;
		if (auto it = element.tags.find("roof:orientation"); it != element.tags.end()) {
			if (it->second == "along")
				axis_x = width >= length;
			else if (it->second == "across")
				axis_x = width < length;
		}
		if (auto it = element.tags.find("roof:direction"); it != element.tags.end())
			if (auto degrees = parse_roof_direction_degrees(it->second)) {
				mapped_bearing = *degrees;
				const double radians = *degrees * std::numbers::pi / 180.0;
				axis_x = std::abs(std::cos(radians)) >= std::abs(std::sin(radians));
			}
		int radius = std::max(1, (axis_x ? length : width) / 2);
		if (mapped_bearing) {
			const double radians = *mapped_bearing * std::numbers::pi / 180.0;
			radius = std::max(
					1, int(std::lround(std::abs(std::sin(radians)) * width / 2.0 +
									   std::abs(std::cos(radians)) * length / 2.0)));
		}
		int peak = peak_cap.value_or(std::max(2, radius));
		if (!peak_cap)
			if (auto it = element.tags.find("roof:angle"); it != element.tags.end()) {
				std::string angle = it->second;
				if (angle.size() >= 2 &&
						angle.compare(angle.size() - 2, 2, "\xC2\xB0") == 0)
					angle.resize(angle.size() - 2);
				try {
					const double degrees = std::stod(angle);
					if (degrees > 0.0 && degrees < 90.0)
						peak = std::max(
								1, int(std::lround(
										   std::tan(degrees * std::numbers::pi / 180.0) *
										   radius)));
				} catch (...) {
				}
			}
		const Block roof_block = selected_roof_block();
		for (const auto &p : floor_area) {
			int distance;
			if (mapped_bearing) {
				const double radians = *mapped_bearing * std::numbers::pi / 180.0;
				distance = int(
						std::lround(std::abs((p.first - center_x) * std::sin(radians) +
											 (p.second - center_z) * std::cos(radians))));
			} else {
				distance = axis_x ? std::abs(p.second - center_z)
								  : std::abs(p.first - center_x);
			}
			const double t = std::clamp(double(distance) / double(radius), 0.0, 1.0);
			const int rise = std::max(0, int(std::lround(peak * std::sqrt(1.0 - t * t))));
			editor.set_block_absolute(roof_block, p.first,
					base_height + abs_terrain_offset + rise, p.second, nullptr, nullptr);
		}
		return;
	}

	if (roof_type == RoofType::Hipped || roof_type == RoofType::HalfHipped ||
			roof_type == RoofType::Gambrel || roof_type == RoofType::Mansard) {
		int32_t width = max_x - min_x;
		int32_t length = max_z - min_z;
		if (roof_type == RoofType::Hipped || roof_type == RoofType::Mansard) {
			const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash> footprint(
					floor_area.begin(), floor_area.end());
			auto scan_dir = [&](int32_t x, int32_t z, int32_t dx, int32_t dz) {
				int distance = 0;
				while (footprint.contains(
						{x + (distance + 1) * dx, z + (distance + 1) * dz}))
					++distance;
				return distance;
			};
			struct HipCell
			{
				int dist_to_edge;
				int local_half;
				int local_reach;
				uint8_t closest_dir;
			};
			std::unordered_map<std::pair<int32_t, int32_t>, HipCell, pair_hash> cell_data;
			std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash>
					roof_heights;
			cell_data.reserve(floor_area.size() * 2);
			roof_heights.reserve(floor_area.size() * 2);
			int max_full_span = 0;
			for (const auto &[x, z] : floor_area) {
				const int dm_x = scan_dir(x, z, -1, 0);
				const int dp_x = scan_dir(x, z, 1, 0);
				const int dm_z = scan_dir(x, z, 0, -1);
				const int dp_z = scan_dir(x, z, 0, 1);
				const std::array<int, 4> distances{dm_x, dp_x, dm_z, dp_z};
				const int dist_to_edge =
						*std::min_element(distances.begin(), distances.end());
				uint8_t closest_dir = 0;
				if (dist_to_edge != dm_x && dist_to_edge == dp_x)
					closest_dir = 1;
				else if (dist_to_edge != dm_x && dist_to_edge != dp_x &&
						 dist_to_edge == dm_z)
					closest_dir = 2;
				else if (dist_to_edge != dm_x && dist_to_edge != dp_x &&
						 dist_to_edge != dm_z && dist_to_edge == dp_z)
					closest_dir = 3;
				const int half_x = (dm_x + dp_x + 1) / 2;
				const int half_z = (dm_z + dp_z + 1) / 2;
				const int local_half = std::min(half_x, half_z);
				const int local_reach = std::min((dm_x + dp_x) / 2, (dm_z + dp_z) / 2);
				max_full_span = std::max(max_full_span, std::max(half_x, half_z));
				cell_data[{x, z}] = {dist_to_edge, local_half, local_reach, closest_dir};
			}

			const int wall_cap = peak_cap.value_or(
					std::max(1, int(std::lround(building_height * .6))));
			const bool use_half_pitch = !peak_cap && max_full_span - wall_cap >= 4;
			int mansard_steep_height = 4;
			if (roof_type == RoofType::Mansard)
				if (const auto levels = parse_meter_tag(element.tags, "roof:levels"))
					mansard_steep_height =
							std::clamp(int(std::lround(*levels * 3.0)), 3, 6);
			constexpr int roof_lift = 1;
			for (const auto &[x, z] : floor_area) {
				const auto &cell = cell_data.at({x, z});
				int roof_y;
				if (roof_type == RoofType::Mansard) {
					const int cap = std::min(std::max(wall_cap, mansard_steep_height + 2),
							std::max(building_height, 3));
					const int local_cap =
							std::max(1, int(std::lround(cell.local_half * .9))) +
							mansard_steep_height;
					const int steep =
							(std::min(cell.dist_to_edge, 2) * mansard_steep_height + 1) /
							2;
					const int shallow = std::max(0, cell.dist_to_edge - 2) / 2;
					const int rise = std::min(cap, steep + shallow);
					roof_y = base_height + std::min(rise, std::min(cap, local_cap)) +
							 roof_lift;
				} else if (peak_cap) {
					const double pitch =
							double(*peak_cap) / std::max(1, cell.local_reach);
					int rise = std::clamp(int(std::lround(cell.dist_to_edge * pitch)), 0,
							std::max(1, *peak_cap));
					if (roof_type == RoofType::Gambrel) {
						const int slope_dist = use_half_pitch
													   ? (cell.dist_to_edge + 1) / 2
													   : cell.dist_to_edge;
						rise = std::min(*peak_cap, 2 * std::min(slope_dist, 2) +
														   std::max(0, slope_dist - 2));
					}
					roof_y = base_height + rise + roof_lift;
				} else {
					const int slope_dist = use_half_pitch ? (cell.dist_to_edge + 1) / 2
														  : cell.dist_to_edge;
					int rise = slope_dist;
					if (roof_type == RoofType::Gambrel)
						rise = 2 * std::min(slope_dist, 2) + std::max(0, slope_dist - 2);
					const int local_boost =
							std::max(1, int(std::lround(cell.local_half * .85)));
					roof_y = base_height +
							 std::min(rise, std::min(local_boost, wall_cap)) + roof_lift;
				}
				roof_heights[{x, z}] = roof_y;
			}

			const Block stair_material =
					get_stair_block_for_material(selected_roof_block());
			for (const auto &[x, z] : floor_area) {
				const int roof_y = roof_heights.at({x, z});
				const auto lower_at = [&](int nx, int nz) {
					auto it = roof_heights.find({nx, nz});
					return it != roof_heights.end() && it->second < roof_y;
				};
				const auto steep_at = [&](int nx, int nz) {
					auto it = roof_heights.find({nx, nz});
					return it != roof_heights.end() && roof_y - it->second > 2;
				};
				for (int y = base_height; y <= roof_y; ++y) {
					const bool lower_n = lower_at(x, z - 1), lower_s = lower_at(x, z + 1);
					const bool lower_w = lower_at(x - 1, z), lower_e = lower_at(x + 1, z);
					const int lower_count =
							int(lower_n) + int(lower_s) + int(lower_w) + int(lower_e);
					const bool polygon_edge = !footprint.contains({x - 1, z}) ||
											  !footprint.contains({x + 1, z}) ||
											  !footprint.contains({x, z - 1}) ||
											  !footprint.contains({x, z + 1});
					const bool steep_drop = steep_at(x, z - 1) || steep_at(x, z + 1) ||
											steep_at(x - 1, z) || steep_at(x + 1, z);
					if (y == roof_y && (polygon_edge || lower_count > 0) && !steep_drop) {
						StairFacing facing = StairFacing::East;
						StairShape shape = StairShape::Straight;
						if (lower_count == 2) {
							if (lower_n && lower_w) {
								facing = StairFacing::East;
								shape = StairShape::OuterRight;
							} else if (lower_n && lower_e) {
								facing = StairFacing::South;
								shape = StairShape::OuterRight;
							} else if (lower_s && lower_w) {
								facing = StairFacing::East;
								shape = StairShape::OuterLeft;
							} else if (lower_s && lower_e) {
								facing = StairFacing::North;
								shape = StairShape::OuterLeft;
							} else {
								switch (cell_data.at({x, z}).closest_dir) {
								case 0:
									facing = StairFacing::East;
									break;
								case 1:
									facing = StairFacing::West;
									break;
								case 2:
									facing = StairFacing::South;
									break;
								default:
									facing = StairFacing::North;
									break;
								}
							}
						} else if (lower_count == 1) {
							if (lower_n)
								facing = StairFacing::South;
							else if (lower_s)
								facing = StairFacing::North;
							else if (lower_w)
								facing = StairFacing::East;
							else
								facing = StairFacing::West;
						} else if (lower_count == 3) {
							if (!lower_n)
								facing = StairFacing::North;
							else if (!lower_s)
								facing = StairFacing::South;
							else if (!lower_w)
								facing = StairFacing::West;
							else
								facing = StairFacing::East;
						} else {
							switch (cell_data.at({x, z}).closest_dir) {
							case 0:
								facing = StairFacing::East;
								break;
							case 1:
								facing = StairFacing::West;
								break;
							case 2:
								facing = StairFacing::South;
								break;
							default:
								facing = StairFacing::North;
								break;
							}
						}
						editor.set_block_with_properties_absolute(
								create_stair_with_properties(
										stair_material, facing, shape),
								x, y + abs_terrain_offset, z, nullptr, nullptr);
					} else {
						editor.set_block_absolute(selected_roof_block(), x,
								y + abs_terrain_offset, z, nullptr, nullptr);
					}
				}
			}
			const int y_inner = base_height + abs_terrain_offset;
			const int y_outer = base_height - 1 + abs_terrain_offset;
			const std::array<std::tuple<int, int, StairFacing>, 4> overhang_dirs{
					{{-1, 0, StairFacing::East}, {1, 0, StairFacing::West},
							{0, -1, StairFacing::South}, {0, 1, StairFacing::North}}};
			std::unordered_map<std::pair<int, int>, StairFacing, pair_hash> inner_cells;
			std::unordered_map<std::pair<int, int>, StairFacing, pair_hash> outer_cells;
			for (const auto &[x, z] : floor_area) {
				for (const auto &[dx, dz, facing] : overhang_dirs) {
					const std::pair<int, int> inner{x + dx, z + dz};
					const std::pair<int, int> outer{x + 2 * dx, z + 2 * dz};
					if (footprint.contains(inner))
						continue;
					inner_cells.try_emplace(inner, facing);
					if (!footprint.contains(outer))
						outer_cells.try_emplace(outer, facing);
				}
			}
			using DiagonalStair = std::pair<StairFacing, StairShape>;
			std::unordered_map<std::pair<int, int>, DiagonalStair, pair_hash> diag_inner;
			std::unordered_map<std::pair<int, int>, DiagonalStair, pair_hash> diag_outer;
			std::unordered_map<std::pair<int, int>, StairFacing, pair_hash> diag_l_arm;
			const std::array<std::tuple<int, int, StairFacing, StairShape, StairFacing,
									 StairFacing>,
					4>
					corner_dirs{{{-1, -1, StairFacing::East, StairShape::OuterRight,
										 StairFacing::East, StairFacing::South},
							{1, -1, StairFacing::South, StairShape::OuterRight,
									StairFacing::West, StairFacing::South},
							{-1, 1, StairFacing::East, StairShape::OuterLeft,
									StairFacing::East, StairFacing::North},
							{1, 1, StairFacing::North, StairShape::OuterLeft,
									StairFacing::West, StairFacing::North}}};
			for (const auto &[x, z] : floor_area) {
				for (const auto &[dx, dz, facing, shape, x_arm, z_arm] : corner_dirs) {
					if (footprint.contains({x + dx, z}) ||
							footprint.contains({x, z + dz}))
						continue;
					diag_inner.try_emplace({x + dx, z + dz}, facing, shape);
					diag_outer.try_emplace({x + 2 * dx, z + 2 * dz}, facing, shape);
					diag_l_arm.try_emplace({x + 2 * dx, z + dz}, x_arm);
					diag_l_arm.try_emplace({x + dx, z + 2 * dz}, z_arm);
				}
			}
			auto place_overhang_stair = [&](const std::pair<int, int> &cell, int y,
												StairFacing facing, StairShape shape) {
				editor.set_block_with_properties_absolute(
						create_stair_with_properties(stair_material, facing, shape),
						cell.first, y, cell.second, nullptr, nullptr);
			};
			for (const auto &[cell, facing] : inner_cells)
				place_overhang_stair(cell, y_inner, facing, StairShape::Straight);
			for (const auto &[cell, stair] : diag_inner)
				if (!inner_cells.contains(cell))
					place_overhang_stair(cell, y_inner, stair.first, stair.second);
			for (const auto &[cell, facing] : outer_cells)
				if (!inner_cells.contains(cell) && !diag_inner.contains(cell))
					place_overhang_stair(cell, y_outer, facing, StairShape::Straight);
			for (const auto &[cell, stair] : diag_outer)
				if (!inner_cells.contains(cell) && !outer_cells.contains(cell) &&
						!diag_inner.contains(cell))
					place_overhang_stair(cell, y_outer, stair.first, stair.second);
			for (const auto &[cell, facing] : diag_l_arm)
				if (!inner_cells.contains(cell) && !outer_cells.contains(cell) &&
						!diag_inner.contains(cell) && !diag_outer.contains(cell))
					place_overhang_stair(cell, y_outer, facing, StairShape::Straight);
			const auto mansard_band =
					roof_mansard_dormer_band(element, base_height, roof_type);
			place_roof_dormer_windows(editor, floor_area, roof_heights, base_height,
					abs_terrain_offset, style_seed, selected_roof_block(), add_dormers,
					width >= length ? std::pair{1, 0} : std::pair{0, 1}, mansard_band);
			return;
		}

		bool is_rectangular = ((static_cast<double>(width) / std::max(1, length) > 1.3) ||
							   (static_cast<double>(length) / std::max(1, width) > 1.3));
		bool long_axis_is_x = preferred_ridge_along_x.value_or(width > length);
		if (auto orientation = element.tags.find("roof:orientation");
				orientation != element.tags.end()) {
			if (orientation->second == "along")
				long_axis_is_x = width >= length;
			else if (orientation->second == "across")
				long_axis_is_x = width < length;
		}
		if (auto direction = element.tags.find("roof:direction");
				direction != element.tags.end()) {
			if (auto degrees = parse_roof_direction_degrees(direction->second)) {
				const double radians = *degrees * std::numbers::pi / 180.0;
				long_axis_is_x =
						std::abs(std::cos(radians)) >= std::abs(std::sin(radians));
			}
		}

		int32_t roof_peak_height =
				base_height +
				(peak_cap ? *peak_cap : ((std::max(width, length) > 20) ? 7 : 5));
		if (!peak_cap)
			if (auto it = element.tags.find("roof:angle"); it != element.tags.end()) {
				std::string angle = it->second;
				if (angle.size() >= 2 &&
						angle.compare(angle.size() - 2, 2, "\xC2\xB0") == 0)
					angle.resize(angle.size() - 2);
				try {
					const double degrees = std::stod(angle);
					if (degrees > 0.0 && degrees < 90.0)
						roof_peak_height =
								base_height +
								std::max(1, int(std::lround(
													std::tan(degrees * std::numbers::pi /
															 180.0) *
													(std::min(width, length) / 2.0))));
				} catch (...) {
				}
			}

		Block roof_block = selected_roof_block();

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
			const auto mansard_band =
					roof_mansard_dormer_band(element, base_height, roof_type);
			place_roof_dormer_windows(editor, floor_area, roof_heights, base_height,
					abs_terrain_offset, style_seed, roof_block, add_dormers,
					long_axis_is_x ? std::pair{1, 0} : std::pair{0, 1}, mansard_band);
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
			const auto mansard_band =
					roof_mansard_dormer_band(element, base_height, roof_type);
			place_roof_dormer_windows(editor, floor_area, roof_heights, base_height,
					abs_terrain_offset, style_seed, roof_block, add_dormers,
					long_axis_is_x ? std::pair{1, 0} : std::pair{0, 1}, mansard_band);
		}
		if (roof_type == RoofType::Gambrel || roof_type == RoofType::HalfHipped)
			place_gabled_eave_overhang(editor, floor_area, base_height,
					abs_terrain_offset, roof_block, long_axis_is_x);

		return;
	}

	if (roof_type == RoofType::Skillion) {
		int32_t roof_min_x = std::numeric_limits<int32_t>::max();
		int32_t roof_max_x = std::numeric_limits<int32_t>::min();
		int32_t roof_min_z = std::numeric_limits<int32_t>::max();
		int32_t roof_max_z = std::numeric_limits<int32_t>::min();
		double min_projection = std::numeric_limits<double>::max();
		double max_projection = std::numeric_limits<double>::lowest();
		for (const auto &[x, z] : floor_area) {
			roof_min_x = std::min(roof_min_x, x);
			roof_max_x = std::max(roof_max_x, x);
			roof_min_z = std::min(roof_min_z, z);
			roof_max_z = std::max(roof_max_z, z);
		}
		const int32_t width = roof_max_x - roof_min_x;
		const int32_t length = roof_max_z - roof_min_z;
		auto roof_rng = element_rng(style_seed ^ 0x5C11'1104'D129'0000ULL);
		double downhill_degrees;
		const auto direction = element.tags.find("roof:direction");
		std::optional<double> parsed_direction;
		if (direction != element.tags.end())
			parsed_direction = parse_roof_direction_degrees(direction->second);
		if (parsed_direction) {
			downhill_degrees = *parsed_direction;
		} else {
			const bool flip = roof_rng.random_bool(0.5);
			if (width <= length)
				downhill_degrees = flip ? 270.0 : 90.0;
			else
				downhill_degrees = flip ? 0.0 : 180.0;
		}
		const double radians = downhill_degrees * std::numbers::pi / 180.0;
		const double downhill_x = std::sin(radians), downhill_z = -std::cos(radians);
		for (const auto &[x, z] : floor_area) {
			const double projection = x * downhill_x + z * downhill_z;
			min_projection = std::min(min_projection, projection);
			max_projection = std::max(max_projection, projection);
		}
		const double run = std::max(1.0, max_projection - min_projection);
		const int32_t max_roof_height =
				peak_cap ? std::max(1, *peak_cap)
						 : std::min(std::clamp(static_cast<int>(run) / 3, 2, 10),
								   static_cast<int>(std::round(
										   std::max(1.0, building_height * 0.9))));

		Block roof_block = selected_roof_block();

		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash> roof_heights;
		roof_heights.reserve(floor_area.size() * 2);
		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			double projection = x * downhill_x + z * downhill_z;
			double slope_progress = (max_projection - projection) /
									std::max(1.0, max_projection - min_projection);
			int32_t roof_height =
					base_height +
					static_cast<int32_t>(
							slope_progress * static_cast<double>(max_roof_height));
			roof_heights[{x, z}] = roof_height;
		}

		Block stair_block_material = get_stair_block_for_material(roof_block);
		const int cardinal =
				static_cast<int>(std::floor((downhill_degrees + 45.0) / 90.0)) & 3;
		const StairFacing stair_facing = cardinal == 0	 ? StairFacing::South
										 : cardinal == 1 ? StairFacing::West
										 : cardinal == 2 ? StairFacing::North
														 : StairFacing::East;
		const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash> footprint(
				floor_area.begin(), floor_area.end());

		for (auto const &p : floor_area) {
			int32_t x = p.first;
			int32_t z = p.second;
			int32_t roof_height = roof_heights[{x, z}];

			for (int32_t y = base_height; y <= roof_height; ++y) {
				if (y == roof_height) {
					bool has_lower_neighbor = false;
					bool steep_drop = false;
					std::pair<int32_t, int32_t> ncoords[4] = {
							{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}};
					bool on_polygon_edge = false;
					for (auto const &nc : ncoords) {
						on_polygon_edge |= !footprint.contains(nc);
						auto it = roof_heights.find(nc);
						if (it != roof_heights.end() && it->second < roof_height) {
							has_lower_neighbor = true;
							steep_drop |= roof_height - it->second > 2;
						}
					}
					has_lower_neighbor |= on_polygon_edge;

					if (has_lower_neighbor && !steep_drop) {
						BlockWithProperties stair_block_with_props =
								create_stair_with_properties(stair_block_material,
										stair_facing, StairShape::Straight);
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
		const int width = max_x - min_x, length = max_z - min_z;
		const double half_w = std::max(1, width / 2);
		const double half_l = std::max(1, length / 2);
		const int shorter_half = std::min(width, length) / 2;
		const int inferred_boost =
				std::min(std::max(1, static_cast<int>(std::round(shorter_half * 0.85))),
						std::max(1, static_cast<int>(std::round(building_height * 0.6))));
		const int peak_boost = peak_cap ? std::max(1, *peak_cap) : inferred_boost;
		const auto radial = footprint_radial_fractions(element, floor_area);

		Block roof_block = selected_roof_block();

		//std::unordered_map<std::pair<int32_t,int32_t>, int32_t, pair_hash> roof_heights;
		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, PairHash> roof_heights;
		roof_heights.reserve(floor_area.size() * 2);

		for (std::size_t i = 0; i < floor_area.size(); ++i) {
			const auto &p = floor_area[i];
			int32_t x = p.first;
			int32_t z = p.second;
			const double dx = std::abs(double(x - center_x));
			const double dz = std::abs(double(z - center_z));
			const double fraction =
					radial ? (*radial)[i]
						   : std::min(1.0, std::max(dx / half_w, dz / half_l));
			const int32_t roof_height =
					base_height + static_cast<int32_t>((1.0 - fraction) * peak_boost);
			roof_heights[{x, z}] = roof_height;
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
					const int lower_count = int(has_lower_north) + int(has_lower_south) +
											int(has_lower_west) + int(has_lower_east);
					if (lower_count == 2 && has_lower_north && has_lower_west) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::East, StairShape::OuterRight);
					} else if (lower_count == 2 && has_lower_north && has_lower_east) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::South, StairShape::OuterRight);
					} else if (lower_count == 2 && has_lower_south && has_lower_west) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::East, StairShape::OuterLeft);
					} else if (lower_count == 2 && has_lower_south && has_lower_east) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::North, StairShape::OuterLeft);
					} else if (lower_count == 0) {
						const StairFacing facing =
								std::abs(dx) >= std::abs(dz)
										? (dx > 0 ? StairFacing::West : StairFacing::East)
										: (dz > 0 ? StairFacing::North
												  : StairFacing::South);
						stair_block = create_stair_with_properties(
								stair_block_material, facing, StairShape::Straight);
					} else if (has_lower_north) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::South, StairShape::Straight);
					} else if (has_lower_south) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::North, StairShape::Straight);
					} else if (has_lower_west) {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::East, StairShape::Straight);
					} else {
						stair_block = create_stair_with_properties(stair_block_material,
								StairFacing::West, StairShape::Straight);
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
				peak_cap ? std::max(1, *peak_cap)
						 : std::min(std::max(2, int(std::min(half_w, half_l) * 1.2)),
								   building_height * 2);
		const Block roof_block = selected_roof_block();
		const auto radial = footprint_radial_fractions(element, floor_area);
		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash> surface;
		surface.reserve(floor_area.size() * 2);
		for (std::size_t i = 0; i < floor_area.size(); ++i) {
			const auto &[x, z] = floor_area[i];
			const double dx = double(x - center_x), dz = double(z - center_z);
			const double normalized =
					radial ? (*radial)[i]
						   : std::min(1.0, std::hypot(dx / half_w, dz / half_l));
			surface[{x, z}] = base_height + int((1.0 - normalized) * peak_height);
		}
		fill_shell(surface, roof_block, base_height);
		return;
	}

	if (roof_type == RoofType::Onion) {
		const double base_radius = double(std::min(max_x - min_x, max_z - min_z) / 2);
		const int total_height = peak_cap ? std::max(1, *peak_cap)
										  : std::min(std::max(6, int(base_radius * 1.8)),
													building_height * 2);
		const int max_search = int(std::ceil(base_radius * 1.25)) + 1;
		const Block roof_block = selected_roof_block();
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
				editor.set_block_absolute(roof_block, center_x, y, center_z, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				continue;
			}
			if (t < .05) {
				for (const auto &[x, z] : floor_area)
					editor.set_block_absolute(roof_block, x, y, z, std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{}));
				continue;
			}
			const double radius_sq = radius * radius;
			for (int dz = -max_search; dz <= max_search; ++dz)
				for (int dx = -max_search; dx <= max_search; ++dx) {
					if (double(dx * dx + dz * dz) > radius_sq)
						continue;
					const int x = center_x + dx, z = center_z + dz;
					// Rust clips only the drum to the footprint; the bulb above it
					// deliberately flares beyond the wall outline.
					if (t < .15 && !footprint.contains({x, z}))
						continue;
					editor.set_block_absolute(roof_block, x, y, z, std::nullopt,
							std::optional<std::vector<Block>>(std::vector<Block>{}));
				}
		}
		return;
	}

	if (roof_type == RoofType::Dome) {
		const double half_w = std::max(1.0, double(max_x - min_x) / 2.0);
		const double half_l = std::max(1.0, double(max_z - min_z) / 2.0);
		const double rise = peak_cap ? double(std::max(1, *peak_cap))
									 : std::max(1.0, std::min(half_w, half_l) * .8);

		Block roof_block = selected_roof_block();
		const auto radial = footprint_radial_fractions(element, floor_area);
		std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash> surface;
		surface.reserve(floor_area.size() * 2);
		for (std::size_t i = 0; i < floor_area.size(); ++i) {
			const auto &p = floor_area[i];
			int32_t x = p.first;
			int32_t z = p.second;
			const double dx = double(x - center_x), dz = double(z - center_z);
			const double normalized_distance =
					radial ? (*radial)[i]
						   : std::min(1.0, std::hypot(dx / half_w, dz / half_l));

			double height_factor = std::sqrt(
					std::max(0.0, 1.0 - normalized_distance * normalized_distance));
			surface[{x, z}] = base_height + static_cast<int32_t>(height_factor * rise);
		}
		fill_shell(surface, roof_block, base_height);

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
		const CoordinateBitmap &building_passages,
		const interior_uses::InteriorUseIndex *interior_index,
		const RoadMaskBitmap *road_mask,
		const BuildingFootprintBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members)
{
	if (should_skip_underground_tags(relation.tags)) {
		return;
	}
	for (const auto &member : relation.members)
		if (member.way.id == structures::starship::STARBASE_PAD2_INNER_RING_WAY &&
				member.role == ProcessedMemberRole::Inner)
			structures::place_starship(editor, member.way);

	std::optional<int> relation_levels;
	if (const auto levels_tag = relation.tags.find("building:levels");
			levels_tag != relation.tags.end()) {
		std::string value = levels_tag->second;
		const auto first = value.find_first_not_of(" \t\r\n");
		const auto last = value.find_last_not_of(" \t\r\n");
		if (first != std::string::npos) {
			value = value.substr(first, last - first + 1);
			try {
				std::size_t parsed = 0;
				const double levels = std::stod(value, &parsed);
				if (parsed == value.size() && std::isfinite(levels)) {
					const double rounded = std::round(levels);
					relation_levels = static_cast<int>(
							std::clamp(rounded, double(std::numeric_limits<int>::min()),
									double(std::numeric_limits<int>::max())));
				}
			} catch (const std::exception &) {
			}
		}
	}
	auto outer_ways = facade_outer_rings(relation, xzbbox);
	if (outer_ways.empty())
		return;
	auto inner_rings = collect_merged_rings(relation, ProcessedMemberRole::Inner, xzbbox);

	std::vector<HolePolygon> hole_polygons;
	hole_polygons.reserve(inner_rings.size());
	for (std::size_t i = 0; i < inner_rings.size(); ++i) {
		ProcessedWay way;
		way.id = relation_ring_id(relation.id, 0x8000ULL | (i & 0x7FFFULL));
		way.nodes = std::move(inner_rings[i]);
		hole_polygons.push_back(HolePolygon{std::move(way), true});
	}

	for (auto &merged_way : outer_ways) {
		generate_buildings(&editor, merged_way, args, relation_levels, flood_fill_cache,
				building_passages, hole_polygons.empty() ? nullptr : &hole_polygons,
				std::nullopt, road_mask, building_footprints, group_members,
				interior_index);
	}
}

std::vector<ProcessedWay> facade_outer_rings(
		const ProcessedRelation &relation, const XZBBox &xzbbox)
{
	if (should_skip_underground_tags(relation.tags))
		return {};

	// Match Rust relation_outer_rings(): only type=building relations treat
	// Part members as the rendered geometry. Multipolygon outer ways remain
	// eligible even when their relation carries building:part-like tags.
	if (relation.tags.get("type") == "building") {
		if (std::any_of(relation.members.begin(), relation.members.end(),
					[&](const auto &member) {
						return member.role == ProcessedMemberRole::Part;
					}))
			return {};
	}

	// Closed building:part outer rings render as their own ways for every
	// relation type. Match Rust's relation_outer_rings(): rendering the parent
	// relation as well would stack a second shell over those standalone parts.
	bool has_outer = false;
	bool all_outer_parts_are_standalone = true;
	for (const auto &member : relation.members) {
		if (member.role != ProcessedMemberRole::Outer)
			continue;
		has_outer = true;
		const auto part = member.way.tags.find("building:part");
		const bool explicitly_not_a_part =
				part != member.way.tags.end() && part->second.size() == 2 &&
				std::tolower(static_cast<unsigned char>(part->second[0])) == 'n' &&
				std::tolower(static_cast<unsigned char>(part->second[1])) == 'o';
		const bool is_part = part != member.way.tags.end() && !explicitly_not_a_part;
		const bool is_closed = member.way.nodes.size() >= 4 &&
							   member.way.nodes.front().id == member.way.nodes.back().id;
		all_outer_parts_are_standalone &= is_part && is_closed;
	}
	if (has_outer && all_outer_parts_are_standalone)
		return {};

	auto nodes = collect_merged_rings(relation, ProcessedMemberRole::Outer, xzbbox);
	std::vector<ProcessedWay> ways;
	ways.reserve(nodes.size());
	for (std::size_t i = 0; i < nodes.size(); ++i) {
		ProcessedWay way;
		way.id = relation_ring_id(relation.id, i);
		way.tags = relation.tags;
		way.nodes = std::move(nodes[i]);
		ways.push_back(std::move(way));
	}
	return ways;
}

void generate_bridge(WorldEditor &editor, const ProcessedWay &element,
		const FloodFillCache &flood_fill_cache,
		const std::optional<std::chrono::milliseconds> &floodfill_timeout)
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
			int start_y = editor.get_ground_level(start_node.x, start_node.z);
			int end_y = editor.get_ground_level(end_node.x, end_node.z);
			bridge_deck_ground_y = std::max(start_y, end_y);
		}
	}

	Block floor_block = STONE;
	Block railing_block = STONE_BRICKS;
	const int bridge_y_offset = [&]() {
		if (const auto level = strict_parse::i32(element.tags.get("level")))
			return (*level * 3) + 1;
		if (const auto layer = strict_parse::i32(element.tags.get("layer"));
				layer && *layer > 0)
			return (*layer * 4) + 2;
		return 1;
	}();

	std::optional<std::pair<int, int>> previous_node = std::nullopt;
	for (const auto &node : element.nodes) {
		int x = node.x;
		int z = node.z;

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

	const auto bridge_area = flood_fill_cache.get_or_compute(element, floodfill_timeout);

	// Use the same level bridge deck height for filled areas
	int floor_y = bridge_deck_ground_y + bridge_y_offset;

	for (const auto &p : *bridge_area) {
		int x = p.first;
		int z = p.second;
		editor.set_block(floor_block, x, floor_y, z, std::nullopt, std::nullopt);
	}
}

}
}
