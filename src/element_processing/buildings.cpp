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
#include <map>
#include <numbers>
#include <numeric>
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
#include "building_region.h"
#include "connected_blocks.h"
#include "../mapillary/facades.h"
#include "building_facade.h"
#include "rooftops.h"
#include "../building_facades/registry.h"
#include "signage.h"
#include "../decals/pictograms.h"
#include "../decals/font.h"
#include "historic.h"
#include "parking_garage.h"
#include "bridge_towers.h"
#include "subprocessor/buildings_interior.h"
#include "subprocessor/interior/uses.h"
#include "subprocessor/interior/index.h"
#include "../structures/structures.h"
#include "../structures/starship.h"
#include "../deterministic_rng.h"
#include "../block_palette.h"
#include "../data_processing.h"
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
using building_facade::BaySlot;

namespace
{
using PropertyPair = std::pair<std::string_view, std::string_view>;

// Match Rust's facade property cache: recurring states are resolved on the
// worker thread before allocating an unordered_map for BlockWithProperties.
BlockWithProperties cached_prop_block(
		Block base, std::initializer_list<PropertyPair> properties)
{
	if (properties.size() == 0)
		return BlockWithProperties{base};
	struct Entry
	{
		content_t block_id{};
		std::vector<std::pair<std::string, std::string>> properties;
		std::weak_ptr<const BlockWithProperties::Properties> state;
	};
	struct LocalCache
	{
		std::unordered_map<std::size_t, std::vector<Entry>> entries;
		std::size_t count = 0;
	};
	static thread_local LocalCache cache;

	std::size_t hash = std::hash<content_t>{}(base.id());
	for (const auto &[key, value] : properties) {
		for (const auto part : {key, value})
			for (const unsigned char byte : part)
				hash = (hash ^ byte) * 1099511628211ULL;
		hash = (hash ^ 0xff) * 1099511628211ULL;
	}
	if (const auto found = cache.entries.find(hash); found != cache.entries.end()) {
		auto &entries = found->second;
		for (auto it = entries.begin(); it != entries.end();) {
			if (auto state = it->state.lock()) {
				bool equal = it->block_id == base.id() &&
							 it->properties.size() == properties.size();
				auto requested = properties.begin();
				for (const auto &[key, value] : it->properties) {
					if (!equal)
						break;
					equal = key == requested->first && value == requested->second;
					++requested;
				}
				if (equal)
					return BlockWithProperties{base, std::move(state)};
				++it;
			} else {
				it = entries.erase(it);
				--cache.count;
			}
		}
		if (entries.empty())
			cache.entries.erase(found);
	}

	BlockWithProperties::Properties map;
	map.reserve(properties.size());
	Entry entry;
	entry.block_id = base.id();
	entry.properties.reserve(properties.size());
	for (const auto &[key, value] : properties) {
		entry.properties.emplace_back(key, value);
		map.emplace(entry.properties.back());
	}
	entry.state = BlockWithProperties::intern_properties(std::move(map));
	if (cache.count >= 512) {
		for (auto bucket = cache.entries.begin(); bucket != cache.entries.end();) {
			auto &entries = bucket->second;
			for (auto it = entries.begin(); it != entries.end();) {
				if (it->state.expired()) {
					it = entries.erase(it);
					--cache.count;
				} else {
					++it;
				}
			}
			if (entries.empty())
				bucket = cache.entries.erase(bucket);
			else
				++bucket;
		}
		if (cache.count >= 512) {
			auto bucket = cache.entries.begin();
			bucket->second.erase(bucket->second.begin());
			--cache.count;
			if (bucket->second.empty())
				cache.entries.erase(bucket);
		}
	}
	auto state = entry.state.lock();
	cache.entries[hash].push_back(std::move(entry));
	++cache.count;
	return BlockWithProperties{base, std::move(state)};
}
} // namespace

Block rust_category_wall_block(
		BuildingCategory category, ArchEra era, biome::Climate climate, ChaCha8Rng &rng);
std::optional<Block> rust_regional_wall_block(BuildingCategory category, ArchEra era,
		biome::Climate climate, building_style::MaterialRegion region, int height,
		double scale, bool attached, bool historic_fortification, std::uint64_t district,
		ChaCha8Rng &rng);

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

struct RoofStyleContext
{
	BuildingCategory category;
	ArchEra era;
	building_style::MaterialRegion materials;
	bool attached;
	std::uint64_t district;
	Block window_block;
	int window_phase;
	bool soffit{};
};

struct DormerStyle
{
	std::optional<int> window_phase;
	std::optional<Block> gable_block;
	std::optional<Block> window_block;
	bool copper_hood{false};
};

bool is_glass_wall_block(const Block &block)
{
	return block == GLASS || block == WHITE_STAINED_GLASS ||
		   block == GRAY_STAINED_GLASS || block == LIGHT_GRAY_STAINED_GLASS ||
		   block == BLACK_STAINED_GLASS || block == BROWN_STAINED_GLASS ||
		   block == CYAN_STAINED_GLASS || block == BLUE_STAINED_GLASS ||
		   block == LIGHT_BLUE_STAINED_GLASS || block == ORANGE_STAINED_GLASS ||
		   block == MAGENTA_STAINED_GLASS || block == YELLOW_STAINED_GLASS ||
		   block == PURPLE_STAINED_GLASS || block == RED_STAINED_GLASS;
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
	const auto edge_ranges = [&](std::size_t i) {
		using BucketRange = std::optional<std::pair<std::size_t, std::size_t>>;
		const auto [ax, az] = points[i];
		const auto [bx, bz] = points[(i + 1) % count];
		const double a0 = std::atan2(az - centroid_z, ax - centroid_x);
		const double a1 = std::atan2(bz - centroid_z, bx - centroid_x);
		const double lo = std::min(a0, a1), hi = std::max(a0, a1);
		const std::size_t blo = bucket_of(lo), bhi = bucket_of(hi);
		std::array<BucketRange, 2> ranges{};
		if (hi - lo > PI) {
			ranges[0] = std::pair<std::size_t, std::size_t>{0, blo + 1};
			ranges[1] = std::pair<std::size_t, std::size_t>{
					bhi == 0 ? 0 : bhi - 1, BUCKET_COUNT - 1};
		} else {
			ranges[0] =
					std::pair<std::size_t, std::size_t>{blo == 0 ? 0 : blo - 1, bhi + 1};
		}
		for (auto &range : ranges)
			if (range)
				range->second = std::min(range->second, BUCKET_COUNT - 1);
		return ranges;
	};
	// Store all angular bucket members in one contiguous array. This avoids
	// hundreds of small vector allocations on building footprints with many
	// outline nodes, matching Rust's flattened bucket layout.
	std::vector<std::size_t> bucket_start(BUCKET_COUNT + 1, 0);
	for (std::size_t i = 0; i < count; ++i)
		for (const auto &range : edge_ranges(i))
			if (range)
				for (std::size_t b = range->first; b <= range->second; ++b)
					++bucket_start[b + 1];
	for (std::size_t b = 0; b < BUCKET_COUNT; ++b)
		bucket_start[b + 1] += bucket_start[b];
	auto fill = bucket_start;
	std::vector<std::size_t> bucket_edges(bucket_start[BUCKET_COUNT]);
	for (std::size_t i = 0; i < count; ++i)
		for (const auto &range : edge_ranges(i))
			if (range)
				for (std::size_t b = range->first; b <= range->second; ++b)
					bucket_edges[fill[b]++] = i;
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
		const std::size_t bucket = bucket_of(std::atan2(pz, px));
		double hit = std::numeric_limits<double>::infinity(), far_ = 0.0;
		before.clear();
		for (std::size_t edge = bucket_start[bucket]; edge < bucket_start[bucket + 1];
				edge++) {
			const std::size_t i = bucket_edges[edge];
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
			far_ = std::max(far_, t);
			if (t + 0.5 >= distance)
				hit = std::min(hit, t);
			else
				before.push_back(t);
		}
		std::sort(before.begin(), before.end());
		for (std::size_t i = 1; i < before.size(); i += 2)
			if (before[i] - before[i - 1] > 1.0)
				return std::nullopt;
		const double boundary = std::isfinite(hit) ? hit : far_;
		fractions.push_back(boundary > 0.0 ? std::min(1.0, distance / boundary) : 1.0);
	}
	return fractions;
}

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
using RoofDistanceGrid = std::unordered_map<std::pair<int, int>, int, PairHash>;

// For each footprint cell, count cells along the contiguous row/column runs
// toward -x, +x, -z and +z. Sorting the sparse footprint avoids repeatedly
// walking long rows for every cell when roofs cover large buildings.
std::vector<std::array<int, 4>> axis_runs(const std::vector<std::pair<int, int>> &area)
{
	std::vector<std::array<int, 4>> runs(area.size(), {0, 0, 0, 0});
	std::vector<std::size_t> order(area.size());
	std::iota(order.begin(), order.end(), 0);
	for (const bool along_x : {true, false}) {
		auto key = [&](std::size_t i) {
			const auto [x, z] = area[i];
			return along_x ? std::pair{z, x} : std::pair{x, z};
		};
		std::sort(order.begin(), order.end(),
				[&](std::size_t a, std::size_t b) { return key(a) < key(b); });
		const std::size_t dir = along_x ? 0 : 2;
		for (std::size_t begin = 0; begin < order.size();) {
			std::size_t end = begin + 1;
			while (end < order.size()) {
				const auto previous = key(order[end - 1]);
				const auto next = key(order[end]);
				if (previous.first != next.first || next.second - previous.second > 1)
					break;
				++end;
			}
			const int first = key(order[begin]).second;
			const int last = key(order[end - 1]).second;
			for (std::size_t i = begin; i < end; ++i) {
				const int position = key(order[i]).second;
				runs[order[i]][dir] = position - first;
				runs[order[i]][dir + 1] = last - position;
			}
			begin = end;
		}
	}
	return runs;
}

inline void generate_roof(WorldEditor &editor, ProcessedWay const &element,
		int32_t start_y_offset, int32_t building_height, Block floor_block,
		Block wall_block, Block accent_block, std::optional<Block> roof_block_override,
		std::optional<Block> curtain_glass, std::uint64_t style_seed, bool add_dormers,
		bool steep_gable, std::optional<bool> preferred_ridge_along_x, RoofType roof_type,
		std::vector<std::pair<int32_t, int32_t>> const &cached_floor_area,
		int32_t abs_terrain_offset, double scale_factor, bool short_eave = false,
		std::optional<RoofStyleContext> style_context = std::nullopt,
		const std::vector<HolePolygon> *hole_polygons = nullptr,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_blocked = nullptr,
		bool tone_patches = false);

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
								  member.way->id) != skipped_outer_ids.end();
		if (member.role == role && !skip)
			rings.push_back(member.way->nodes);
	}
	merge_way_segments(rings);
	std::vector<std::vector<ProcessedNode>> out;
	for (auto &ring : rings) {
		ring = clipping::clip_way_to_bbox_owned(std::move(ring), xzbbox);
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
	const auto type = element.tags.contains("building")
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

Block rust_pitched_roof_block(BuildingCategory category, ArchEra era, RoofType roof_type,
		Block wall_block, building_style::MaterialRegion region, bool attached,
		std::uint64_t district, std::uint64_t style_seed)
{
	using R = building_style::MaterialRegion;
	using Entry = std::pair<Block, std::uint32_t>;
	using Pool = std::vector<Entry>;
	const Block tile_red = RED_TERRACOTTA, tile_brick = BRICK,
				tile_dark = MANGROVE_PLANKS, tile_brown = NETHER_BRICK,
				tile_orange = TERRACOTTA, slate = COBBLED_DEEPSLATE,
				slate_dark = DEEPSLATE_TILES, black = POLISHED_BLACKSTONE_BRICKS,
				zinc = POLISHED_ANDESITE, copper = WAXED_OXIDIZED_COPPER,
				metal_green = DARK_PRISMARINE, metal_red = RED_NETHER_BRICKS;
	const bool civic = category == BuildingCategory::Religious ||
					   category == BuildingCategory::Historic;
	const bool spire =
			roof_type == RoofType::Dome || roof_type == RoofType::Onion ||
			roof_type == RoofType::Cone ||
			(roof_type == RoofType::Pyramidal && category == BuildingCategory::Religious);
	const bool house = category == BuildingCategory::House ||
					   category == BuildingCategory::Farm ||
					   category == BuildingCategory::Garage ||
					   (!attached && (category == BuildingCategory::Default ||
											 category == BuildingCategory::Residential));
	Pool pool;
	if (spire) {
		pool = {{copper, 45}, {slate_dark, 40}, {black, 15}};
	} else if (roof_type == RoofType::Mansard) {
		pool = region == R::France ? Pool{{zinc, 50}, {slate_dark, 50}}
								   : Pool{{slate_dark, 50}, {zinc, 30}, {slate, 20}};
	} else if (category == BuildingCategory::Industrial ||
			   category == BuildingCategory::Warehouse) {
		pool = {{zinc, 40}, {tile_brick, 20}, {slate, 25}, {slate_dark, 15}};
	} else {
		switch (region) {
		case R::Central:
			pool = {{tile_red, 55}, {tile_brick, 10}, {tile_brown, 12}, {tile_orange, 5},
					{slate, 13}};
			break;
		case R::CentralEast:
			pool = {{tile_dark, 30}, {tile_red, 25}, {tile_brick, 15}, {tile_orange, 10},
					{slate, 15}};
			break;
		case R::BrickNorth:
			pool = {{tile_brick, 35}, {tile_red, 15}, {tile_orange, 15}, {slate_dark, 25},
					{black, 10}};
			break;
		case R::BritishIsles:
			pool = {{slate_dark, 65}, {tile_brick, 25}, {tile_orange, 10}};
			break;
		case R::France:
			pool = {{zinc, 50}, {slate, 20}, {slate_dark, 15}, {tile_red, 10},
					{tile_orange, 5}};
			break;
		case R::Mediterranean:
			pool = {{tile_orange, 45}, {POLISHED_GRANITE, 15}, {tile_brick, 25},
					{tile_red, 10}, {MUD_BRICKS, 5}};
			break;
		case R::Nordic:
			pool = {{tile_brick, 30}, {tile_red, 10}, {black, 25}, {slate_dark, 25},
					{tile_brown, 10}};
			break;
		case R::NorthAmerica:
			pool = house ? Pool{{slate, 28}, {slate_dark, 16}, {black, 12},
								   {DARK_OAK_PLANKS, 14}, {SPRUCE_PLANKS, 8},
								   {MUD_BRICKS, 6}, {tile_brick, 5}, {tile_orange, 4},
								   {metal_green, 3}}
						 : Pool{{slate_dark, 45}, {slate, 20}, {black, 20},
								   {tile_brick, 15}};
			break;
		case R::NorthAmericaSunbelt:
			pool = house ? Pool{{tile_orange, 22}, {tile_red, 8}, {tile_brick, 10},
								   {slate, 24}, {slate_dark, 12}, {MUD_BRICKS, 10},
								   {DARK_OAK_PLANKS, 7}, {SPRUCE_PLANKS, 7}}
						 : Pool{{slate_dark, 45}, {slate, 20}, {black, 20},
								   {tile_brick, 15}};
			break;
		case R::Canada:
			pool = house ? Pool{{slate, 28}, {slate_dark, 18}, {black, 14},
								   {DARK_OAK_PLANKS, 10}, {SPRUCE_PLANKS, 6}, {zinc, 10},
								   {MUD_BRICKS, 6}, {metal_green, 4}, {tile_brick, 4}}
						 : Pool{{slate_dark, 45}, {slate, 20}, {black, 20},
								   {tile_brick, 15}};
			break;
		case R::EastEurope:
		case R::CentralAsia:
			pool = {{zinc, 22}, {metal_green, 18}, {metal_red, 12}, {tile_dark, 8},
					{slate, 14}, {tile_red, 12}, {tile_brick, 8}, {tile_brown, 6}};
			break;
		case R::MiddleEast:
			pool = {{tile_orange, 35}, {tile_brick, 25}, {zinc, 20}, {tile_red, 10},
					{slate, 10}};
			break;
		case R::SubSaharanAfrica:
			pool = {{zinc, 30}, {tile_brick, 15}, {tile_red, 10}, {metal_green, 10},
					{slate, 12}, {tile_orange, 10}, {metal_red, 8}, {MUD_BRICKS, 5}};
			break;
		case R::SouthAsia:
			pool = {{tile_orange, 30}, {tile_brick, 20}, {zinc, 25}, {slate, 10},
					{tile_red, 10}, {metal_green, 5}};
			break;
		case R::SoutheastAsia:
			pool = {{tile_orange, 25}, {tile_brick, 15}, {zinc, 20}, {tile_red, 10},
					{slate, 10}, {metal_green, 8}, {metal_red, 6}, {MUD_BRICKS, 6}};
			break;
		case R::EastAsia:
			pool = {{slate, 30}, {slate_dark, 20}, {black, 10}, {tile_brick, 12},
					{tile_orange, 8}, {zinc, 10}, {metal_green, 6}, {tile_red, 4}};
			break;
		case R::Korea:
			pool = {{slate_dark, 26}, {slate, 16}, {metal_green, 18}, {tile_orange, 12},
					{tile_red, 10}, {metal_red, 8}, {zinc, 10}};
			break;
		case R::Japan:
			pool = {{slate_dark, 38}, {slate, 34}, {black, 10}, {zinc, 10},
					{tile_brown, 5}, {metal_green, 3}};
			break;
		case R::LatinAmerica:
			pool = {{tile_orange, 35}, {tile_brick, 25}, {tile_red, 10}, {zinc, 15},
					{slate, 8}, {MUD_BRICKS, 7}};
			break;
		case R::SouthernCone:
			pool = {{tile_brick, 30}, {tile_orange, 25}, {tile_red, 12}, {slate, 15},
					{zinc, 10}, {slate_dark, 8}};
			break;
		case R::Australasia:
			pool = {{tile_orange, 22}, {tile_brick, 14}, {slate, 22}, {zinc, 16},
					{tile_brown, 8}, {metal_red, 6}, {black, 6}, {metal_green, 4}};
			break;
		case R::Other:
			pool = {{tile_brick, 30}, {tile_orange, 15}, {tile_red, 5}, {slate, 20},
					{slate_dark, 15}, {zinc, 10}, {DARK_OAK_PLANKS, 5}};
			break;
		}
	}
	auto draw = [](const Pool &choices, ChaCha8Rng &rng) {
		std::uint32_t total = 0;
		for (const auto &[block, weight] : choices) {
			(void)block;
			total += weight;
		}
		std::uint32_t roll = rng.uniform(total);
		for (const auto &[block, weight] : choices) {
			if (roll < weight)
				return block;
			roll -= weight;
		}
		return choices.back().first;
	};
	const bool european = building_style::building_region(region) ==
								  building_style::BuildingRegion::Europe &&
						  region != R::EastEurope;
	const bool modern = era == ArchEra::PostWarPanel || era == ArchEra::Contemporary;
	auto rng = element_rng(style_seed ^ 0x2F0FC0DE00000004ULL);
	Block selected = pool.front().first;
	if (spire) {
		selected = draw(pool, rng);
	} else {
		auto district_rng = element_rng(district ^ 0x2F0FC0DED1570003ULL);
		Pool dominant_choices = pool;
		std::erase_if(dominant_choices, [&](const Entry &entry) {
			return entry.first == tile_brown ||
				   (region == R::France &&
						   (entry.first == tile_red || entry.first == tile_brick ||
								   entry.first == tile_dark ||
								   entry.first == tile_orange)) ||
				   (region != R::EastEurope &&
						   (entry.first == metal_green || entry.first == metal_red));
		});
		const Block dominant =
				draw(dominant_choices.empty() ? pool : dominant_choices, district_rng);
		Pool own = pool;
		if (civic) {
			own.emplace_back(slate_dark, 20);
			own.emplace_back(copper, 10);
		} else if (european && modern) {
			own.emplace_back(slate, 15);
			own.emplace_back(tile_brown, 10);
		}
		const double share = european ? .70 : .40;
		selected = (civic || modern || rng.random_bool(1.0 - share)) ? draw(own, rng)
																	 : dominant;
	}
	const bool same = selected == wall_block;
	const int warm_tone = selected == tile_brick						  ? 1
						  : selected == tile_red || selected == tile_dark ? 2
						  : selected == tile_orange						  ? 3
																		  : 0;
	const bool warm_wall =
			(warm_tone == 1 && (wall_block == BRICK || wall_block == POLISHED_GRANITE ||
									   wall_block == GRANITE)) ||
			(warm_tone == 2 &&
					(wall_block == RED_TERRACOTTA || wall_block == MANGROVE_PLANKS)) ||
			(warm_tone == 3 &&
					(wall_block == TERRACOTTA || wall_block == ORANGE_TERRACOTTA));
	const bool grey_roof = selected == POLISHED_ANDESITE || selected == ANDESITE ||
						   selected == STONE || selected == STONE_BRICKS ||
						   selected == SMOOTH_STONE || selected == LIGHT_GRAY_CONCRETE ||
						   selected == LIGHT_GRAY_CONCRETE_POWDER ||
						   selected == CYAN_TERRACOTTA || selected == TUFF;
	const bool grey_wall = wall_block == POLISHED_ANDESITE || wall_block == ANDESITE ||
						   wall_block == STONE || wall_block == STONE_BRICKS ||
						   wall_block == SMOOTH_STONE ||
						   wall_block == LIGHT_GRAY_CONCRETE ||
						   wall_block == LIGHT_GRAY_CONCRETE_POWDER ||
						   wall_block == CYAN_TERRACOTTA || wall_block == TUFF;
	if (same || warm_wall || (grey_roof && grey_wall)) {
		if (warm_tone != 0 && region != R::BritishIsles)
			return warm_tone == 2 ? tile_brick : tile_red;
		return selected == slate_dark || region == R::BrickNorth ? black : slate_dark;
	}
	return selected;
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
	if (element.tags.contains("heritage") && element.tags.get("heritage") != "no")
		score += 20;
	if (element.tags.contains("historic") && element.tags.get("historic") != "no")
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
	Sash,
	Portrait,
	Spaced,
	Wide,
	Ribbon,
	Hooded,
	Piers,
	Strips
};

struct BayRhythm
{
	int window;
	int pier;
	int corner;
};

struct BayLayout
{
	int len = 0;
	int lead = 0;
	int window = 0;
	int pier = 0;
	int count = 0;
	bool wide_window = false;
	bool wide_pier = false;

	static BayLayout plan(int len, BayRhythm rhythm)
	{
		const int window = std::max(1, rhythm.window);
		const int pier = std::max(1, rhythm.pier);
		const int corner = len < 3 * (window + pier) ? 1 : std::max(1, rhythm.corner);
		int count = len - 2 * corner >= window
							? (len - 2 * corner + pier) / (window + pier)
							: 0;
		for (;;) {
			BayLayout layout{len, len, window, pier, count, false, false};
			if (count == 0)
				return layout;
			const int spare = len - count * window - (count - 1) * pier;
			if (spare % 2 != 0) {
				if (count % 2 == 0) {
					layout.wide_pier = true;
				} else if (window == 1 && count >= 5) {
					--count;
					continue;
				} else {
					layout.wide_window = true;
				}
			}
			layout.lead = spare / 2;
			return layout;
		}
	}

	BaySlot slot(int i) const
	{
		if (count == 0 || i < lead || i >= len - lead)
			return {BaySlot::Kind::End};
		const int mid = count / 2;
		int offset = i - lead;
		for (int bay = 0; bay < count; ++bay) {
			const int width = window + (wide_window && bay == mid ? 1 : 0);
			if (offset < width)
				return {BaySlot::Kind::Window, offset, width, bay};
			offset -= width;
			const int pier_width = pier + (wide_pier && bay + 1 == mid ? 1 : 0);
			if (offset < pier_width)
				return {BaySlot::Kind::Pier};
			offset -= pier_width;
		}
		return {BaySlot::Kind::End};
	}
};

BayRhythm window_rhythm(WindowArchetype archetype)
{
	switch (archetype) {
	case WindowArchetype::Sash:
		return {1, 1, 1};
	case WindowArchetype::Portrait:
	case WindowArchetype::Hooded:
		return {2, 2, 2};
	case WindowArchetype::Spaced:
		return {1, 2, 2};
	case WindowArchetype::Wide:
		return {3, 2, 2};
	case WindowArchetype::Ribbon:
		return {4, 1, 1};
	case WindowArchetype::Piers:
		return {2, 1, 2};
	case WindowArchetype::Strips:
		return {1, 1, 1};
	}
	return {2, 2, 2};
}

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

BalconyBand pick_balcony_band(BuildingCategory category, ArchEra era,
		building_style::BuildingRegion region, int height, int floor_cycle,
		bool has_street, std::uint64_t seed)
{
	const bool old_europe = region == building_style::BuildingRegion::Europe &&
							era != ArchEra::PostWarPanel && era != ArchEra::Contemporary;
	if ((category != BuildingCategory::Residential &&
				category != BuildingCategory::House) ||
			height < 3 * floor_cycle || !has_street || old_europe)
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

Block trim_block_for_wall(Block wall, std::uint64_t seed)
{
	if (wall == BRICK || wall == TERRACOTTA || wall == RED_TERRACOTTA ||
			wall == ORANGE_TERRACOTTA || wall == GRANITE || wall == POLISHED_GRANITE ||
			wall == NETHER_BRICK || wall == RED_NETHER_BRICKS || wall == MUD_BRICKS ||
			wall == BROWN_TERRACOTTA || wall == BROWN_CONCRETE) {
		const Block light_stone[] = {SMOOTH_STONE, POLISHED_DIORITE, SMOOTH_SANDSTONE};
		return light_stone[element_rng(seed ^ 0x7121A57E00000001ULL).uniform(3)];
	}
	if (wall == SANDSTONE)
		return SMOOTH_SANDSTONE;
	if (wall == SMOOTH_SANDSTONE || wall == END_STONE_BRICKS || wall == YELLOW_TERRACOTTA)
		return SANDSTONE;
	if (wall == QUARTZ_BLOCK || wall == QUARTZ_BRICKS || wall == SMOOTH_QUARTZ ||
			wall == WHITE_CONCRETE)
		return STONE_BRICKS;
	if (wall == WHITE_TERRACOTTA)
		return QUARTZ_BLOCK;
	if (wall == SMOOTH_STONE || wall == POLISHED_DIORITE || wall == DIORITE)
		return QUARTZ_BLOCK;
	if (wall == OAK_PLANKS || wall == OAK_LOG || wall == SPRUCE_PLANKS ||
			wall == SPRUCE_LOG)
		return DARK_OAK_PLANKS;
	if (wall == DARK_OAK_PLANKS)
		return SPRUCE_PLANKS;
	return POLISHED_DIORITE;
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

Block weathered_variant(Block block, ChaCha8Rng &rng)
{
	if (block == STONE_BRICKS) {
		const std::array options{MOSSY_STONE_BRICKS, CRACKED_STONE_BRICKS, STONE_BRICKS};
		return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
	}
	if (block == COBBLESTONE)
		return MOSSY_COBBLESTONE;
	if (block == STONE) {
		const std::array options{ANDESITE, COBBLESTONE};
		return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
	}
	if (block == SMOOTH_STONE)
		return STONE;
	if (block == POLISHED_ANDESITE)
		return ANDESITE;
	if (block == OAK_PLANKS) {
		const std::array options{SPRUCE_PLANKS, DARK_OAK_PLANKS};
		return options[rng.uniform(static_cast<std::uint32_t>(options.size()))];
	}
	if (block == OAK_LOG)
		return SPRUCE_LOG;
	return block;
}

bool has_weathered_variant(Block block)
{
	return block == STONE_BRICKS || block == COBBLESTONE || block == STONE ||
		   block == SMOOTH_STONE || block == POLISHED_ANDESITE || block == OAK_PLANKS ||
		   block == OAK_LOG;
}

const std::vector<Block> &substitute_pool_only(Block block)
{
	static const std::vector<Block> none;
	static const std::vector<Block> stone_bricks{CRACKED_STONE_BRICKS, STONE};
	static const std::vector<Block> cobblestone{ANDESITE, STONE};
	static const std::vector<Block> stone{ANDESITE, COBBLESTONE};
	static const std::vector<Block> andesite{POLISHED_ANDESITE, STONE};
	static const std::vector<Block> polished_andesite{ANDESITE};
	static const std::vector<Block> chiseled_stone_bricks{STONE_BRICKS};
	static const std::vector<Block> mossy_stone_bricks{MOSSY_COBBLESTONE, STONE_BRICKS};
	static const std::vector<Block> mossy_cobblestone{MOSSY_STONE_BRICKS, COBBLESTONE};
	static const std::vector<Block> brick{POLISHED_GRANITE, POLISHED_GRANITE, GRANITE};
	static const std::vector<Block> polished_granite{BRICK, GRANITE};
	static const std::vector<Block> granite{POLISHED_GRANITE};
	static const std::vector<Block> terracotta{POLISHED_GRANITE};
	static const std::vector<Block> orange_terracotta{TERRACOTTA};
	static const std::vector<Block> deepslate_bricks{
			POLISHED_DEEPSLATE, COBBLED_DEEPSLATE};
	static const std::vector<Block> polished_deepslate{
			DEEPSLATE_BRICKS, COBBLED_DEEPSLATE};
	static const std::vector<Block> cobbled_deepslate{
			DEEPSLATE_BRICKS, POLISHED_DEEPSLATE, DEEPSLATE};
	static const std::vector<Block> deepslate{COBBLED_DEEPSLATE, DEEPSLATE_BRICKS};
	static const std::vector<Block> polished_blackstone_bricks{
			POLISHED_BLACKSTONE, BLACKSTONE};
	static const std::vector<Block> polished_blackstone{
			POLISHED_BLACKSTONE_BRICKS, BLACKSTONE};
	static const std::vector<Block> blackstone{
			POLISHED_BLACKSTONE, POLISHED_BLACKSTONE_BRICKS};
	static const std::vector<Block> brown_terracotta{BROWN_CONCRETE};
	static const std::vector<Block> brown_concrete{BROWN_TERRACOTTA};
	static const std::vector<Block> quartz_block{SMOOTH_QUARTZ};
	static const std::vector<Block> quartz_bricks{QUARTZ_BLOCK};
	static const std::vector<Block> white_concrete{CALCITE};
	static const std::vector<Block> sandstone{SMOOTH_SANDSTONE};
	static const std::vector<Block> smooth_sandstone{SANDSTONE};
	static const std::vector<Block> polished_diorite{DIORITE};
	static const std::vector<Block> diorite{POLISHED_DIORITE};
	static const std::vector<Block> gray_concrete{GRAY_CONCRETE_POWDER};
	static const std::vector<Block> light_gray_concrete{LIGHT_GRAY_CONCRETE_POWDER};
	static const std::vector<Block> light_gray_powder{LIGHT_GRAY_CONCRETE};
	static const std::vector<Block> exposed_copper{
			WAXED_EXPOSED_CHISELED_COPPER, WAXED_EXPOSED_CUT_COPPER};
	static const std::vector<Block> exposed_chiseled{
			WAXED_EXPOSED_COPPER, WAXED_EXPOSED_CUT_COPPER};
	static const std::vector<Block> exposed_cut{
			WAXED_EXPOSED_COPPER, WAXED_EXPOSED_CHISELED_COPPER};
	if (block == STONE_BRICKS)
		return stone_bricks;
	if (block == COBBLESTONE)
		return cobblestone;
	if (block == STONE)
		return stone;
	if (block == ANDESITE)
		return andesite;
	if (block == POLISHED_ANDESITE)
		return polished_andesite;
	if (block == CHISELED_STONE_BRICKS || block == CRACKED_STONE_BRICKS)
		return chiseled_stone_bricks;
	if (block == MOSSY_STONE_BRICKS)
		return mossy_stone_bricks;
	if (block == MOSSY_COBBLESTONE)
		return mossy_cobblestone;
	if (block == BRICK)
		return brick;
	if (block == POLISHED_GRANITE)
		return polished_granite;
	if (block == GRANITE)
		return granite;
	if (block == TERRACOTTA)
		return terracotta;
	if (block == ORANGE_TERRACOTTA)
		return orange_terracotta;
	if (block == DEEPSLATE_BRICKS)
		return deepslate_bricks;
	if (block == POLISHED_DEEPSLATE)
		return polished_deepslate;
	if (block == COBBLED_DEEPSLATE)
		return cobbled_deepslate;
	if (block == DEEPSLATE)
		return deepslate;
	if (block == POLISHED_BLACKSTONE_BRICKS)
		return polished_blackstone_bricks;
	if (block == POLISHED_BLACKSTONE)
		return polished_blackstone;
	if (block == BLACKSTONE)
		return blackstone;
	if (block == BROWN_TERRACOTTA)
		return brown_terracotta;
	if (block == BROWN_CONCRETE)
		return brown_concrete;
	if (block == QUARTZ_BLOCK)
		return quartz_block;
	if (block == QUARTZ_BRICKS || block == SMOOTH_QUARTZ)
		return quartz_bricks;
	if (block == WHITE_CONCRETE)
		return white_concrete;
	if (block == SANDSTONE)
		return sandstone;
	if (block == SMOOTH_SANDSTONE)
		return smooth_sandstone;
	if (block == POLISHED_DIORITE)
		return polished_diorite;
	if (block == DIORITE)
		return diorite;
	if (block == GRAY_CONCRETE)
		return gray_concrete;
	if (block == LIGHT_GRAY_CONCRETE)
		return light_gray_concrete;
	if (block == LIGHT_GRAY_CONCRETE_POWDER)
		return light_gray_powder;
	if (block == WAXED_EXPOSED_COPPER)
		return exposed_copper;
	if (block == WAXED_EXPOSED_CHISELED_COPPER)
		return exposed_chiseled;
	if (block == WAXED_EXPOSED_CUT_COPPER)
		return exposed_cut;
	return none;
}

int floor_divide(int value, int divisor)
{
	return static_cast<int>(std::floor(static_cast<double>(value) / divisor));
}

struct WallVariety
{
	struct Patch
	{
		int x;
		int z;
		int band;
		Block block;
	};
	bool flat_field;
	std::array<std::optional<Patch>, 64> patches{};

	explicit WallVariety(std::uint64_t element_id)
	{
		auto rng = element_rng(element_id ^ 0xBABEF1A122228888ULL);
		flat_field = rng.random_bool(.20);
	}
};

Block apply_block_variety(Block chosen, int x, int y, int z, Block wall_block,
		int start_y_offset, std::uint64_t element_id, Block window_block,
		BuildingCondition condition, WallVariety &variety)
{
	if (const auto facade = mapillary::facades::block_at(
				x, y, z, start_y_offset, element_id, window_block))
		return *facade;
	if (chosen != wall_block)
		return chosen;
	if (condition != BuildingCondition::Construction)
		if (const auto band = mapillary::facades::band_block_at(
					x, y, z, start_y_offset, element_id))
			return *band;
	const auto &pool = substitute_pool_only(chosen);
	if (pool.empty() || variety.flat_field)
		return chosen;
	const int band = floor_divide(y, 2);
	const int shift = band - floor_divide(band, 3) * 3;
	const int patch_x = floor_divide(x + shift, 3);
	const int patch_z = floor_divide(z + shift, 3);
	const int slot_index = band - floor_divide(band, 64) * 64;
	auto &slot = variety.patches[static_cast<std::size_t>(slot_index)];
	if (slot && slot->x == patch_x && slot->z == patch_z && slot->band == band)
		return slot->block;
	auto patch_rng = coord_rng(patch_x, patch_z,
			element_id ^ 0x50503030AAFFBBCCULL ^
					(std::uint64_t(static_cast<std::int64_t>(band)) << 20));
	const Block block =
			patch_rng.uniform(100) < 12
					? pool[patch_rng.uniform(static_cast<std::uint32_t>(pool.size()))]
					: chosen;
	slot = WallVariety::Patch{patch_x, patch_z, band, block};
	return block;
}

Block apply_condition_variation(Block chosen, int x, int y, int z, Block wall_block,
		Block window_block, bool has_windows, BuildingCondition condition,
		BuildingCategory category, ArchEra era, std::uint64_t seed)
{
	if (condition == BuildingCondition::Construction)
		return chosen;
	double normal_weather_rate = 0.0;
	if (condition == BuildingCondition::Normal) {
		const double category_rate =
				category == BuildingCategory::Historic ||
								category == BuildingCategory::Religious
						? .06
				: category == BuildingCategory::Farm ? .03
													 : 0.0;
		const double era_rate = era == ArchEra::HistoricOrnate		? .05
								: era == ArchEra::TraditionalPreWar ? .02
																	: 0.0;
		normal_weather_rate = std::max(category_rate, era_rate);
	}
	if (condition == BuildingCondition::Normal && normal_weather_rate == 0.0)
		return chosen;
	const auto make_rng = [&]() {
		return coord_rng(
				x, z, seed ^ (std::uint64_t(static_cast<std::int64_t>(y)) << 16));
	};
	const bool window = has_windows && chosen == window_block;
	const double board_rate = condition == BuildingCondition::Disused	  ? .30
							  : condition == BuildingCondition::Abandoned ? .50
																		  : 0.0;
	if (window && board_rate > 0.0) {
		auto rng = make_rng();
		if (rng.random_bool(board_rate))
			return wall_block;
	}
	const double weather_rate = condition == BuildingCondition::Abandoned ? .05
								: condition == BuildingCondition::Ruined  ? .50
								: condition == BuildingCondition::Normal
										? normal_weather_rate
										: 0.0;
	if (!window && weather_rate > 0.0 && has_weathered_variant(chosen)) {
		auto rng = make_rng();
		if (rng.random_bool(weather_rate))
			return weathered_variant(chosen, rng);
	}
	return chosen;
}

bool archetype_allows_window(
		WindowArchetype archetype, int column, int floor_row, int floor_cycle)
{
	if (column < 0)
		return false;
	switch (archetype) {
	case WindowArchetype::Ribbon:
		return floor_cycle >= 4 ? floor_row >= 2 : floor_row >= 1;
	case WindowArchetype::Sash:
	case WindowArchetype::Spaced:
		return floor_row >= 1 && floor_row <= 2;
	default:
		return floor_row >= 1;
	}
}

bool is_masonry_window_wall(Block block)
{
	return block == BRICK || block == STONE_BRICKS || block == SANDSTONE ||
		   block == SMOOTH_SANDSTONE || block == GRANITE || block == POLISHED_GRANITE ||
		   block == TERRACOTTA || block == RED_TERRACOTTA || block == ORANGE_TERRACOTTA ||
		   block == BROWN_TERRACOTTA || block == WHITE_TERRACOTTA ||
		   block == END_STONE_BRICKS || block == QUARTZ_BRICKS || block == MUD_BRICKS ||
		   block == NETHER_BRICK || block == RED_NETHER_BRICKS ||
		   block == POLISHED_DIORITE || block == COBBLESTONE;
}

WindowArchetype pick_window_archetype(BuildingCategory category, ArchEra era,
		building_style::BuildingRegion region, Block wall, std::uint64_t seed)
{
	using W = WindowArchetype;
	std::vector<std::pair<W, std::uint32_t>> table;
	const bool europe = region == building_style::BuildingRegion::Europe;
	const bool america = region == building_style::BuildingRegion::NorthAmerica;
	switch (category) {
	case BuildingCategory::House:
		table = {{W::Portrait, 45}, {W::Spaced, 30}, {W::Sash, 10}, {W::Wide, 15}};
		break;
	case BuildingCategory::Residential:
		if (america)
			table = {{W::Sash, 40}, {W::Portrait, 30}, {W::Spaced, 10}, {W::Ribbon, 10},
					{W::Wide, 10}};
		else if (europe)
			table = {{W::Spaced, 60}, {W::Portrait, 32}, {W::Sash, 4}, {W::Ribbon, 2},
					{W::Wide, 2}};
		else
			table = {{W::Portrait, 40}, {W::Spaced, 20}, {W::Sash, 15}, {W::Ribbon, 15},
					{W::Wide, 10}};
		break;
	case BuildingCategory::Default:
		if (america)
			table = {{W::Sash, 30}, {W::Portrait, 40}, {W::Ribbon, 15}, {W::Wide, 15}};
		else if (europe)
			table = {{W::Spaced, 55}, {W::Portrait, 33}, {W::Ribbon, 6}, {W::Wide, 6}};
		else
			table = {{W::Portrait, 45}, {W::Spaced, 20}, {W::Ribbon, 20}, {W::Wide, 15}};
		break;
	case BuildingCategory::Commercial:
	case BuildingCategory::Office:
		table = {{W::Portrait, 35}, {W::Ribbon, 35}, {W::Wide, 20}, {W::Spaced, 10}};
		break;
	case BuildingCategory::Hotel:
		table = {{W::Portrait, 45}, {W::Spaced, 25}, {W::Ribbon, 20}, {W::Wide, 10}};
		break;
	case BuildingCategory::School:
	case BuildingCategory::Hospital:
		table = {{W::Portrait, 30}, {W::Wide, 35}, {W::Ribbon, 35}};
		break;
	case BuildingCategory::Industrial:
	case BuildingCategory::Warehouse:
		table = {{W::Wide, 40}, {W::Ribbon, 50}, {W::Portrait, 10}};
		break;
	case BuildingCategory::Historic:
		table = {{W::Portrait, 50}, {W::Spaced, 30}, {W::Hooded, 20}};
		break;
	case BuildingCategory::TallBuilding:
		if (is_masonry_window_wall(wall) || era == ArchEra::TraditionalPreWar ||
				era == ArchEra::HistoricOrnate)
			table = {{W::Piers, 65}, {W::Strips, 35}};
		else
			table = {{W::Ribbon, 50}, {W::Piers, 25}, {W::Strips, 25}};
		break;
	case BuildingCategory::MasonrySkyscraper:
		return W::Piers;
	case BuildingCategory::ContemporarySkyscraper:
		return W::Wide;
	case BuildingCategory::Religious:
	case BuildingCategory::Farm:
		return W::Spaced;
	default:
		return W::Portrait;
	}
	auto rng = element_rng(seed ^ 0x57A2C0DEA5C10007ULL);
	auto roll = rng.uniform(100);
	W picked = table.front().first;
	for (const auto &[value, weight] : table) {
		if (roll < weight) {
			picked = value;
			break;
		}
		roll -= weight;
	}
	if (category == BuildingCategory::TallBuilding)
		return picked;
	auto era_rng = element_rng(seed ^ 0x57A2C0DEA5C10008ULL);
	if (era == ArchEra::HistoricOrnate && picked == W::Portrait &&
			era_rng.random_bool(.6))
		return W::Hooded;
	if ((era == ArchEra::HistoricOrnate || era == ArchEra::TraditionalPreWar) &&
			(picked == W::Ribbon || picked == W::Wide))
		return W::Portrait;
	if ((era == ArchEra::PostWarPanel || era == ArchEra::Contemporary) &&
			(picked == W::Sash || picked == W::Spaced || picked == W::Hooded))
		return era_rng.random_bool(.5) ? W::Ribbon : W::Wide;
	return picked;
}

Block category_window_block(BuildingCategory category, ChaCha8Rng &rng)
{
	const Block *pool = nullptr;
	std::size_t count = 0;
	static const std::array residential = {GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
			GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS};
	static const std::array institutional = {GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
			GRAY_STAINED_GLASS, BLACK_STAINED_GLASS};
	static const std::array hospitality = {GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
			GRAY_STAINED_GLASS, BLACK_STAINED_GLASS};
	static const std::array industrial = {
			GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS};
	static const std::array tower = {GRAY_STAINED_GLASS, GRAY_STAINED_GLASS,
			LIGHT_GRAY_STAINED_GLASS, BLACK_STAINED_GLASS, CYAN_STAINED_GLASS};
	static const std::array religious = {BLUE_STAINED_GLASS, CYAN_STAINED_GLASS,
			LIGHT_BLUE_STAINED_GLASS, BROWN_STAINED_GLASS, WHITE_STAINED_GLASS};
	static const std::array farm = {GLASS, WHITE_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS};
	static const std::array historic = {
			LIGHT_GRAY_STAINED_GLASS, GRAY_STAINED_GLASS, GLASS};
	static const std::array variations = {GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS,
			GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS};
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
	case BuildingCategory::TallBuilding:
		use(institutional);
		break;
	case BuildingCategory::ContemporarySkyscraper:
	case BuildingCategory::GridSkyscraper:
	case BuildingCategory::ModernSkyscraper:
	case BuildingCategory::GlassySkyscraper:
	case BuildingCategory::GlassCornerSkyscraper:
		use(tower);
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
		return choose_block({BRICK, BRICK, BRICK, POLISHED_GRANITE}, rng);
	if (m == "stone" || m == "naturalstone" || m == "hard")
		return choose_block(
				{STONE_BRICKS, SMOOTH_STONE, POLISHED_ANDESITE, ANDESITE}, rng);
	if (m == "limestone")
		return choose_block({MUSHROOM_STEM, SMOOTH_SANDSTONE, CALCITE}, rng);
	if (m == "sandstone")
		return choose_block({SMOOTH_SANDSTONE, SMOOTH_SANDSTONE, SANDSTONE}, rng);
	if (m == "marble")
		return choose_block({QUARTZ_BLOCK, CALCITE, POLISHED_DIORITE}, rng);
	if (m == "granite")
		return choose_block({POLISHED_ANDESITE, POLISHED_ANDESITE, LIGHT_GRAY_CONCRETE,
									POLISHED_GRANITE},
				rng);
	if (m == "slate")
		return choose_block(
				{DEEPSLATE_BRICKS, POLISHED_DEEPSLATE, POLISHED_BLACKSTONE}, rng);
	if (m == "concrete" || m == "reinforcedconcrete" || m == "cementblock" ||
			m == "cement" || m == "breezeblock" || m == "concreteblock" ||
			m == "concreteblocks" || m == "block" || m == "concretemasonryunit")
		return choose_block(
				{LIGHT_GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, SMOOTH_STONE, WHITE_CONCRETE},
				rng);
	if (m == "plaster" || m == "stucco" || m == "render" || m == "rendering" ||
			m == "limerender" || m == "plastered")
		return choose_block({WHITE_CONCRETE, WHITE_CONCRETE, WHITE_TERRACOTTA,
									SMOOTH_SANDSTONE, MUSHROOM_STEM},
				rng);
	if (m == "wood" || m == "timber" || m == "timberframe" || m == "timberframing" ||
			m == "timberplanks" || m == "halftimber" || m == "halftimbered" ||
			m == "loghouse" || m == "logs" || m == "bamboo")
		return choose_block({OAK_PLANKS, SPRUCE_PLANKS, DARK_OAK_PLANKS, OAK_LOG}, rng);
	if (m == "reed")
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
		return choose_block({GRAY_STAINED_GLASS, LIGHT_GRAY_STAINED_GLASS, GLASS,
									BLACK_STAINED_GLASS},
				rng);
	if (m == "mirror" || m == "solarpanels")
		return choose_block(
				{GRAY_STAINED_GLASS, CYAN_STAINED_GLASS, BLACK_STAINED_GLASS}, rng);
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
		return choose_block({ANDESITE, COBBLESTONE, STONE_BRICKS}, rng);
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
			normalized == "claytiles" || normalized == "terracotta")
		return choose_block({RED_TERRACOTTA, BRICK, BRICK, TERRACOTTA}, rng);
	if (normalized == "slate" || normalized == "slates")
		return choose_block({COBBLED_DEEPSLATE, DEEPSLATE_TILES}, rng);
	if (normalized == "metal" || normalized == "steel" || normalized == "aluminium" ||
			normalized == "aluminum" || normalized == "corrugatedsteel" ||
			normalized == "corrugatediron" || normalized == "corrugatedmetal" ||
			normalized == "corrugatedironsheets" || normalized == "corrugated" ||
			normalized == "cgi" || normalized == "tin" || normalized == "sheetmetal" ||
			normalized == "metalsheet" || normalized == "metalplates")
		return choose_block(
				{POLISHED_ANDESITE, SMOOTH_STONE, LIGHT_GRAY_CONCRETE, IRON_BLOCK}, rng);
	if (normalized == "zinc" || normalized == "zink" || normalized == "lead")
		return choose_block({POLISHED_ANDESITE, POLISHED_ANDESITE, STONE_BRICKS}, rng);
	if (normalized == "copper")
		return WAXED_OXIDIZED_COPPER;
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
				{COBBLED_DEEPSLATE, DEEPSLATE_TILES, POLISHED_BLACKSTONE_BRICKS}, rng);
	if (normalized == "stone")
		return choose_block({STONE_BRICKS, SMOOTH_STONE, ANDESITE}, rng);
	if (normalized == "gravel")
		return GRAVEL;
	if (normalized == "grass" || normalized == "green" || normalized == "vegetation" ||
			normalized == "greenroof" || normalized == "sod")
		return choose_block({GRASS_BLOCK, MOSS_BLOCK}, rng);
	if (normalized == "eternit" || normalized == "asbestos" ||
			normalized == "fibrecement" || normalized == "fibercement")
		return choose_block({POLISHED_ANDESITE, COBBLED_DEEPSLATE, DEEPSLATE_TILES}, rng);
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
	const auto roof_material = element.tags.find("roof:material");
	const auto material = roof_material != element.tags.end()
								  ? roof_material
								  : element.tags.find("material");
	if (material != element.tags.end())
		if (auto block = get_roof_block_for_material_cpp(material->second, rng))
			return block;

	const auto roof_colour = element.tags.find("roof:colour");
	const auto building_colour = element.tags.find("building:colour");
	const auto colour = roof_colour != element.tags.end()
								? roof_colour
								: (building_colour != element.tags.end()
												  ? building_colour
												  : element.tags.find("colour"));
	if (colour != element.tags.end())
		if (auto rgb = color_text_to_rgb_tuple(colour->second))
			return block_palette::roof_block_for_color(*rgb, rng);
	return std::nullopt;
}

const std::vector<Block> &tile_roof_family()
{
	static const std::vector<Block> family{
			RED_TERRACOTTA, BRICK, MANGROVE_PLANKS, NETHER_BRICK, TERRACOTTA};
	return family;
}

const std::vector<Block> *roof_material_family(const std::string &material)
{
	const auto normalized = normalized_material(material);
	if (normalized == "tile" || normalized == "tiles" || normalized == "rooftile" ||
			normalized == "rooftiles" || normalized == "claytile" ||
			normalized == "claytiles" || normalized == "ceramic" ||
			normalized == "ceramictiles" || normalized == "terracotta" ||
			normalized == "pantiles")
		return &tile_roof_family();
	static const std::vector<Block> concrete_tile{NETHER_BRICK, COBBLED_DEEPSLATE};
	static const std::vector<Block> slate{COBBLED_DEEPSLATE, DEEPSLATE_TILES};
	static const std::vector<Block> copper{WAXED_OXIDIZED_COPPER};
	if (normalized == "concretetile" || normalized == "concretetiles" ||
			normalized == "cementtile" || normalized == "cementtiles")
		return &concrete_tile;
	if (normalized == "slate" || normalized == "slates")
		return &slate;
	if (normalized == "copper")
		return &copper;
	return nullptr;
}

bool is_sheet_metal_roof(const std::string &material)
{
	const auto normalized = normalized_material(material);
	return normalized == "metal" || normalized == "steel" || normalized == "aluminium" ||
		   normalized == "aluminum" || normalized == "iron" || normalized == "tin" ||
		   normalized == "zinc" || normalized == "zink" || normalized == "lead" ||
		   normalized == "sheetmetal" || normalized == "metalsheet" ||
		   normalized == "metalplates" || normalized == "corrugated" ||
		   normalized == "corrugatediron" || normalized == "corrugatedsteel" ||
		   normalized == "corrugatedmetal";
}

std::pair<Block, float> nearest_roof_shade(const std::vector<Block> &family, RGBTuple rgb)
{
	static const std::array<std::pair<Block, RGBTuple>, 10> shades{
			{{RED_TERRACOTTA, {143, 61, 46}}, {BRICK, {150, 97, 83}},
					{MANGROVE_PLANKS, {117, 54, 48}}, {NETHER_BRICK, {44, 21, 26}},
					{TERRACOTTA, {152, 94, 67}}, {COBBLED_DEEPSLATE, {77, 77, 80}},
					{DEEPSLATE_TILES, {54, 54, 55}},
					{POLISHED_BLACKSTONE_BRICKS, {48, 43, 50}},
					{POLISHED_ANDESITE, {132, 135, 134}},
					{WAXED_OXIDIZED_COPPER, {82, 163, 133}}}};
	const auto target = oklab_components(rgb);
	Block best = family.front();
	float best_distance = std::numeric_limits<float>::infinity();
	for (const auto &[block, color] : shades) {
		if (std::find(family.begin(), family.end(), block) == family.end())
			continue;
		const auto lab = oklab_components(color);
		const float dl = .5f * (target[0] - lab[0]);
		const float da = target[1] - lab[1], db = target[2] - lab[2];
		const float distance = std::sqrt(dl * dl + da * da + db * db);
		if (distance < best_distance) {
			best = block;
			best_distance = distance;
		}
	}
	return {best, best_distance};
}

Block pitched_roof_family_for_color(RGBTuple rgb, ChaCha8Rng &rng)
{
	const auto lab = oklab_components(rgb);
	if (std::hypot(lab[1], lab[2]) < .035f) {
		if (lab[0] >= .55f)
			return POLISHED_ANDESITE;
		if (lab[0] >= .38f)
			return COBBLED_DEEPSLATE;
		if (lab[0] >= .25f)
			return DEEPSLATE_TILES;
		return POLISHED_BLACKSTONE_BRICKS;
	}
	const float r = std::get<0>(rgb), g = std::get<1>(rgb), b = std::get<2>(rgb);
	if (r > g && r > b) {
		const float hue = 60.f * (g - b) / (r - std::min(g, b));
		if (hue >= -20.f && hue <= 40.f)
			return nearest_roof_shade(tile_roof_family(), rgb).first;
	}
	const std::vector<Block> all{RED_TERRACOTTA, BRICK, MANGROVE_PLANKS, NETHER_BRICK,
			TERRACOTTA, COBBLED_DEEPSLATE, DEEPSLATE_TILES, POLISHED_BLACKSTONE_BRICKS,
			POLISHED_ANDESITE, WAXED_OXIDIZED_COPPER};
	const auto [block, distance] = nearest_roof_shade(all, rgb);
	return distance <= .17f ? block : block_palette::roof_block_for_color(rgb, rng);
}

std::optional<Block> rust_tagged_roof_block(const ProcessedWay &element, bool pitched,
		std::optional<Block> chosen, ChaCha8Rng &rng)
{
	const auto material_it = element.tags.find("roof:material");
	const auto colour_it = element.tags.find("roof:colour");
	const std::string *material =
			material_it == element.tags.end() ? nullptr : &material_it->second;
	const auto colour = colour_it == element.tags.end()
								? std::optional<RGBTuple>{}
								: color_text_to_rgb_tuple(colour_it->second);
	if (!pitched) {
		if (material)
			if (auto block = get_roof_block_for_material_cpp(*material, rng))
				return block;
		return colour ? std::optional<Block>(
								block_palette::roof_block_for_color(*colour, rng))
					  : std::nullopt;
	}
	if (material)
		if (const auto *family = roof_material_family(*material)) {
			if (colour)
				return nearest_roof_shade(*family, *colour).first;
			if (chosen &&
					std::find(family->begin(), family->end(), *chosen) != family->end())
				return chosen;
			return (*family)[rng.uniform(static_cast<std::uint32_t>(family->size()))];
		}
	if (material && colour && is_sheet_metal_roof(*material))
		return pitched_roof_family_for_color(*colour, rng);
	if (material)
		if (auto block = get_roof_block_for_material_cpp(*material, rng))
			return block;
	if (colour)
		return pitched_roof_family_for_color(*colour, rng);
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

bool roof_dormers_enabled(BuildingCondition condition, BuildingCategory category,
		RoofType roof_type, ArchEra era, building_style::BuildingRegion region,
		int building_height, int floor_cycle, std::uint64_t style_seed)
{
	if (condition != BuildingCondition::Normal ||
			(category != BuildingCategory::House &&
					category != BuildingCategory::Residential &&
					category != BuildingCategory::Commercial &&
					category != BuildingCategory::Office &&
					category != BuildingCategory::Hotel &&
					category != BuildingCategory::Historic) ||
			(roof_type != RoofType::Gabled && roof_type != RoofType::Hipped &&
					roof_type != RoofType::HalfHipped && roof_type != RoofType::Mansard))
		return false;
	const int storeys = building_height / std::max(1, floor_cycle);
	const bool town_block = category != BuildingCategory::House && storeys >= 3;
	const bool pre_war = era == ArchEra::Unknown || era == ArchEra::HistoricOrnate ||
						 era == ArchEra::TraditionalPreWar;
	double chance = .15;
	if (roof_type == RoofType::Mansard)
		chance = region == building_style::BuildingRegion::NorthAmerica ? .5 : .95;
	else if (region == building_style::BuildingRegion::Europe && !pre_war)
		chance = .15;
	else if (region == building_style::BuildingRegion::Europe && town_block)
		chance = .65;
	else if (region == building_style::BuildingRegion::Europe &&
			 category == BuildingCategory::House)
		chance = .3;
	else if (region == building_style::BuildingRegion::Europe)
		chance = .25;
	else if (region == building_style::BuildingRegion::NorthAmerica &&
			 category == BuildingCategory::House)
		chance = .12;
	else if (region == building_style::BuildingRegion::NorthAmerica)
		chance = .06;
	return element_rng(style_seed ^ 0xD0E400005EED0001ULL).random_bool(chance);
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

struct RoofWing
{
	std::pair<double, double> u;
	std::pair<double, double> v;
	bool along_u{};
	std::pair<bool, bool> clipped{};

	std::pair<double, double> along() const { return along_u ? u : v; }
	std::pair<double, double> across() const { return along_u ? v : u; }
	double length() const
	{
		const auto bounds = along();
		return bounds.second - bounds.first;
	}
	double area() const { return (u.second - u.first) * (v.second - v.first); }
	int reach() const
	{
		const auto bounds = across();
		return static_cast<int>(std::floor((bounds.second - bounds.first) / 2.0 + 1e-6));
	}
};

struct RoofTone
{
	Block full;
	Block stairs;
	std::optional<Block> slab;
};

std::optional<RoofTone> roof_tone_sibling(Block roof)
{
	if (roof == RED_TERRACOTTA || roof == BRICK)
		return RoofTone{roof, POLISHED_GRANITE_STAIRS, POLISHED_GRANITE_SLAB};
	if (roof == TERRACOTTA)
		return RoofTone{TERRACOTTA, POLISHED_GRANITE_STAIRS, POLISHED_GRANITE_SLAB};
	if (roof == COBBLED_DEEPSLATE)
		return RoofTone{
				POLISHED_DEEPSLATE, POLISHED_DEEPSLATE_STAIRS, POLISHED_DEEPSLATE_SLAB};
	if (roof == DEEPSLATE_TILES)
		return RoofTone{DEEPSLATE_BRICKS, DEEPSLATE_BRICK_STAIRS, DEEPSLATE_TILE_SLAB};
	if (roof == POLISHED_BLACKSTONE_BRICKS)
		return RoofTone{BLACKSTONE, BLACKSTONE_STAIRS, BLACKSTONE_SLAB};
	return std::nullopt;
}

using RoofPatchCache = std::unordered_map<std::pair<int, int>, bool, PairHash>;

std::pair<int, int> roof_tone_patch_of(int x, int z)
{
	const int row = floor_divide(z, 2);
	const int shift = (row % 2 + 2) % 2;
	return {floor_divide(x + shift, 2), row};
}

bool roof_tone_patch(std::pair<int, int> patch, std::uint64_t seed)
{
	return coord_rng(patch.first, patch.second, seed ^ 0x7111E5A100000005ULL)
				   .uniform(100) < 8;
}

bool cached_roof_tone_patch(int x, int z, std::uint64_t seed, RoofPatchCache &cache)
{
	const auto patch = roof_tone_patch_of(x, z);
	if (const auto it = cache.find(patch); it != cache.end())
		return it->second;
	const bool selected = roof_tone_patch(patch, seed);
	cache.emplace(patch, selected);
	return selected;
}

using RoofHeightMap = std::unordered_map<std::pair<int, int>, int, PairHash>;
using RoofCellSet = std::unordered_set<std::pair<int, int>, PairHash>;

class RoofLevelGrid
{
	static constexpr int EMPTY = std::numeric_limits<int>::min();
	const RoofHeightMap *sparse_ = nullptr;
	int64_t min_x_ = 0;
	int64_t min_z_ = 0;
	int64_t width_ = 0;
	int64_t height_ = 0;
	std::vector<int> dense_;

public:
	explicit RoofLevelGrid(const RoofHeightMap &levels) : sparse_(&levels)
	{
		if (levels.empty())
			return;
		auto it = levels.begin();
		int64_t min_x = it->first.first, max_x = min_x;
		int64_t min_z = it->first.second, max_z = min_z;
		for (const auto &[cell, level] : levels) {
			(void)level;
			min_x = std::min(min_x, int64_t(cell.first));
			max_x = std::max(max_x, int64_t(cell.first));
			min_z = std::min(min_z, int64_t(cell.second));
			max_z = std::max(max_z, int64_t(cell.second));
		}
		const int64_t width = max_x - min_x + 1;
		const int64_t height = max_z - min_z + 1;
		const auto max_size = std::numeric_limits<std::size_t>::max();
		const uint64_t dense_limit =
				levels.size() > (std::numeric_limits<uint64_t>::max() - 64) / 8
						? std::numeric_limits<uint64_t>::max()
						: 8 * uint64_t(levels.size()) + 64;
		if (width <= 0 || height <= 0 ||
				uint64_t(width) >
						std::numeric_limits<uint64_t>::max() / uint64_t(height) ||
				uint64_t(width) * uint64_t(height) > dense_limit ||
				uint64_t(width) > max_size / uint64_t(height))
			return;
		min_x_ = min_x;
		min_z_ = min_z;
		width_ = width;
		height_ = height;
		dense_.assign(static_cast<std::size_t>(width * height), EMPTY);
		for (const auto &[cell, level] : levels) {
			const auto x = int64_t(cell.first) - min_x_;
			const auto z = int64_t(cell.second) - min_z_;
			dense_[static_cast<std::size_t>(z * width_ + x)] = level;
		}
		sparse_ = nullptr;
	}

	std::optional<int> get(int x, int z) const
	{
		if (sparse_) {
			if (const auto it = sparse_->find({x, z}); it != sparse_->end())
				return it->second;
			return std::nullopt;
		}
		const auto local_x = int64_t(x) - min_x_;
		const auto local_z = int64_t(z) - min_z_;
		if (local_x < 0 || local_z < 0 || local_x >= width_ || local_z >= height_)
			return std::nullopt;
		const int value = dense_[static_cast<std::size_t>(local_z * width_ + local_x)];
		return value == EMPTY ? std::nullopt : std::optional<int>(value);
	}
};

RoofHeightMap hip_edge_distances(const std::vector<std::pair<int32_t, int32_t>> &area,
		const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash> &footprint)
{
	constexpr std::array<std::pair<int, int>, 4> sides{
			{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	constexpr std::array<std::pair<int, int>, 4> quarters{
			{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}}};
	RoofHeightMap distances;
	distances.reserve(area.size() * 2);
	std::deque<std::pair<int32_t, int32_t>> queue;
	const auto inside = [&](int32_t x, int32_t z) { return footprint.contains({x, z}); };
	for (const auto &[x, z] : area) {
		const bool edge = std::any_of(sides.begin(), sides.end(),
								  [&, x = x, z = z](const auto &d) {
									  return !inside(x + d.first, z + d.second);
								  }) ||
						  std::any_of(quarters.begin(), quarters.end(),
								  [&, x = x, z = z](const auto &d) {
									  return !inside(x + d.first, z + d.second) &&
											 !inside(x + 2 * d.first, z + d.second) &&
											 !inside(x + d.first, z + 2 * d.second) &&
											 inside(x + 2 * d.first, z) &&
											 inside(x, z + 2 * d.second);
								  });
		if (edge) {
			distances[{x, z}] = 0;
			queue.emplace_back(x, z);
		}
	}
	while (!queue.empty()) {
		const auto [x, z] = queue.front();
		queue.pop_front();
		const int distance = distances.at({x, z});
		const auto visit = [&, x = x, z = z](const auto &offsets) {
			for (const auto &[dx, dz] : offsets) {
				const std::pair<int32_t, int32_t> next{x + dx, z + dz};
				if (inside(next.first, next.second) && !distances.contains(next)) {
					distances[next] = distance + 1;
					queue.push_back(next);
				}
			}
		};
		visit(sides);
		visit(quarters);
	}
	return distances;
}

template <typename HeightMap>
RoofCellSet capped_flat_tops(const HeightMap &levels)
{
	constexpr std::size_t MIN_DECK = 30;
	constexpr std::array<std::pair<int, int>, 8> neighbors{
			{{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}};
	constexpr std::array<std::pair<int, int>, 4> sides{
			{{0, -1}, {-1, 0}, {1, 0}, {0, 1}}};
	RoofCellSet flat;
	for (const auto &[cell, height] : levels) {
		const auto [x, z] = cell;
		if (std::all_of(neighbors.begin(), neighbors.end(),
					[&, x = x, z = z, height = height](const auto &delta) {
						const auto it = levels.find({x + delta.first, z + delta.second});
						return it != levels.end() && it->second == height;
					}))
			flat.insert(cell);
	}
	RoofCellSet seen, decks;
	std::vector<std::pair<int, int>> starts(flat.begin(), flat.end());
	std::sort(starts.begin(), starts.end());
	for (const auto &start : starts) {
		if (!seen.insert(start).second)
			continue;
		std::vector<std::pair<int, int>> patch{start};
		for (std::size_t i = 0; i < patch.size(); ++i) {
			const auto [x, z] = patch[i];
			for (const auto &[dx, dz] : sides) {
				const std::pair<int, int> next{x + dx, z + dz};
				if (flat.contains(next) && seen.insert(next).second)
					patch.push_back(next);
			}
		}
		if (patch.size() >= MIN_DECK)
			decks.insert(patch.begin(), patch.end());
	}
	return decks;
}

Block roof_sheet_block(int x, int z, RoofPatchCache &cache)
{
	const auto patch = std::pair{floor_divide(x, 3), floor_divide(z, 3)};
	const auto it = cache.find(patch);
	const bool selected =
			it != cache.end() ? it->second
							  : cache.emplace(patch, coord_rng(patch.first, patch.second,
															 0x5EE70F7AULL)
															 .random_bool(.3))
										.first->second;
	return selected ? LIGHT_GRAY_CONCRETE : SMOOTH_STONE;
}

std::optional<Block> roof_slab_for_stair(Block stair)
{
	if (stair == STONE_BRICK_STAIRS)
		return STONE_BRICK_SLAB;
	if (stair == COBBLESTONE_STAIRS)
		return COBBLESTONE_SLAB;
	if (stair == MOSSY_COBBLESTONE_STAIRS)
		return MOSSY_COBBLESTONE_SLAB;
	if (stair == MOSSY_STONE_BRICK_STAIRS)
		return MOSSY_STONE_BRICK_SLAB;
	if (stair == STONE_STAIRS)
		return STONE_BLOCK_SLAB;
	if (stair == ANDESITE_STAIRS)
		return ANDESITE_SLAB;
	if (stair == POLISHED_ANDESITE_STAIRS)
		return POLISHED_ANDESITE_SLAB;
	if (stair == POLISHED_DIORITE_STAIRS || stair == DIORITE_STAIRS)
		return POLISHED_DIORITE_SLAB;
	if (stair == DEEPSLATE_BRICK_STAIRS || stair == DEEPSLATE_TILE_STAIRS)
		return DEEPSLATE_TILE_SLAB;
	if (stair == POLISHED_DEEPSLATE_STAIRS)
		return POLISHED_DEEPSLATE_SLAB;
	if (stair == COBBLED_DEEPSLATE_STAIRS)
		return COBBLED_DEEPSLATE_SLAB;
	if (stair == POLISHED_BLACKSTONE_BRICK_STAIRS)
		return POLISHED_BLACKSTONE_SLAB;
	if (stair == BLACKSTONE_STAIRS)
		return BLACKSTONE_SLAB;
	if (stair == BRICK_STAIRS)
		return BRICK_SLAB;
	if (stair == MUD_BRICK_STAIRS)
		return MUD_BRICK_SLAB;
	if (stair == END_STONE_BRICK_STAIRS)
		return END_STONE_BRICK_SLAB;
	if (stair == SMOOTH_SANDSTONE_STAIRS)
		return CUT_SANDSTONE_SLAB;
	if (stair == QUARTZ_STAIRS || stair == SMOOTH_QUARTZ_STAIRS)
		return SMOOTH_QUARTZ_SLAB;
	if (stair == RED_NETHER_BRICK_STAIRS)
		return RED_NETHER_BRICK_SLAB;
	if (stair == NETHER_BRICK_STAIRS)
		return NETHER_BRICK_SLAB;
	if (stair == GRANITE_STAIRS || stair == POLISHED_GRANITE_STAIRS)
		return POLISHED_GRANITE_SLAB;
	if (stair == WAXED_OXIDIZED_CUT_COPPER_STAIRS)
		return WAXED_OXIDIZED_CUT_COPPER_SLAB;
	if (stair == WAXED_EXPOSED_CUT_COPPER_STAIRS)
		return WAXED_EXPOSED_CUT_COPPER_SLAB;
	if (stair == OAK_STAIRS)
		return OAK_SLAB;
	if (stair == SPRUCE_STAIRS)
		return SPRUCE_SLAB;
	if (stair == DARK_OAK_STAIRS)
		return DARK_OAK_SLAB;
	if (stair == JUNGLE_STAIRS)
		return JUNGLE_SLAB;
	if (stair == CHERRY_STAIRS)
		return CHERRY_SLAB;
	if (stair == MANGROVE_STAIRS)
		return MANGROVE_SLAB;
	if (stair == CRIMSON_STAIRS)
		return CRIMSON_SLAB;
	if (stair == WARPED_STAIRS)
		return WARPED_SLAB;
	if (stair == BAMBOO_STAIRS)
		return BAMBOO_SLAB;
	if (stair == PURPUR_STAIRS)
		return PURPUR_SLAB;
	if (stair == DARK_PRISMARINE_STAIRS)
		return DARK_PRISMARINE_SLAB;
	return std::nullopt;
}

std::optional<std::pair<StairFacing, StairShape>> roof_stair_for_high_quarters(
		const std::array<bool, 4> &high)
{
	using F = StairFacing;
	using S = StairShape;
	if (high == std::array<bool, 4>{true, true, false, false})
		return std::pair{F::North, S::Straight};
	if (high == std::array<bool, 4>{false, false, true, true})
		return std::pair{F::South, S::Straight};
	if (high == std::array<bool, 4>{true, false, true, false})
		return std::pair{F::West, S::Straight};
	if (high == std::array<bool, 4>{false, true, false, true})
		return std::pair{F::East, S::Straight};
	if (high == std::array<bool, 4>{true, false, false, false})
		return std::pair{F::North, S::OuterLeft};
	if (high == std::array<bool, 4>{false, true, false, false})
		return std::pair{F::North, S::OuterRight};
	if (high == std::array<bool, 4>{false, false, true, false})
		return std::pair{F::South, S::OuterRight};
	if (high == std::array<bool, 4>{false, false, false, true})
		return std::pair{F::South, S::OuterLeft};
	if (high == std::array<bool, 4>{true, true, true, false})
		return std::pair{F::North, S::InnerLeft};
	if (high == std::array<bool, 4>{true, true, false, true})
		return std::pair{F::North, S::InnerRight};
	if (high == std::array<bool, 4>{true, false, true, true})
		return std::pair{F::South, S::InnerRight};
	if (high == std::array<bool, 4>{false, true, true, true})
		return std::pair{F::South, S::InnerLeft};
	return std::nullopt;
}

struct RoofWingFrame
{
	double angle{};
	std::vector<RoofWing> wings;

	std::pair<double, double> uv(int x, int z) const
	{
		const double c = std::cos(angle), s = std::sin(angle);
		return {x * c + z * s, -x * s + z * c};
	}
	std::size_t main_wing() const
	{
		std::size_t best = 0;
		for (std::size_t i = 1; i < wings.size(); ++i)
			if (wings[i].area() >= wings[best].area())
				best = i;
		return best;
	}
	static std::pair<int, int> ridge_step(const RoofWing &wing)
	{
		return wing.along_u ? std::pair{1, 0} : std::pair{0, 1};
	}
};

std::optional<RoofWingFrame> roof_wings(const std::vector<ProcessedNode> &nodes,
		std::size_t cells, std::optional<bool> main_along_x,
		const std::vector<HolePolygon> *holes)
{
	std::vector<std::pair<double, double>> ring;
	ring.reserve(nodes.size());
	for (const auto &node : nodes)
		ring.emplace_back(node.x, node.z);
	if (ring.size() > 1 && ring.front() == ring.back())
		ring.pop_back();
	if (ring.size() < 4 || (ring.size() < 6 && (!holes || holes->empty())))
		return std::nullopt;

	double sum_c = 0.0, sum_s = 0.0;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto [ax, az] = ring[i];
		const auto [bx, bz] = ring[(i + 1) % ring.size()];
		const double dx = bx - ax, dz = bz - az, len = std::hypot(dx, dz);
		if (len < 3.0)
			continue;
		const double edge_angle = std::atan2(dz, dx);
		sum_c += len * std::cos(4.0 * edge_angle);
		sum_s += len * std::sin(4.0 * edge_angle);
	}
	double angle = (sum_c == 0.0 && sum_s == 0.0) ? 0.0 : std::atan2(sum_s, sum_c) / 4.0;
	if (std::abs(angle * 180.0 / std::numbers::pi) < 1.0)
		angle = 0.0;
	const double c = std::cos(angle), s = std::sin(angle);
	for (auto &point : ring) {
		const auto [x, z] = point;
		point = {x * c + z * s, -x * s + z * c};
	}
	std::vector<std::vector<std::pair<double, double>>> rings{ring};
	if (holes) {
		for (const auto &hole : *holes) {
			std::vector<std::pair<double, double>> transformed;
			transformed.reserve(hole.way.nodes.size());
			for (const auto &node : hole.way.nodes)
				transformed.emplace_back(
						node.x * c + node.z * s, -node.x * s + node.z * c);
			if (transformed.size() > 1 && transformed.front() == transformed.back())
				transformed.pop_back();
			if (transformed.size() >= 3)
				rings.push_back(std::move(transformed));
		}
	}
	const auto shoelace = [](const auto &r) {
		double sum = 0.0;
		for (std::size_t i = 0; i < r.size(); ++i) {
			const auto &[ax, az] = r[i];
			const auto &[bx, bz] = r[(i + 1) % r.size()];
			sum += ax * bz - bx * az;
		}
		return std::abs(sum) * 0.5;
	};
	double polygon_area = shoelace(rings.front());
	for (std::size_t i = 1; i < rings.size(); ++i)
		polygon_area -= shoelace(rings[i]);
	if (polygon_area <= 0.0 || static_cast<double>(cells) < 0.9 * polygon_area)
		return std::nullopt;

	std::vector<std::pair<double, double>> u_lines, v_lines;
	double perimeter = 0.0, oblique = 0.0;
	const double skew = std::tan(15.0 * std::numbers::pi / 180.0);
	for (const auto &r : rings)
		for (std::size_t i = 0; i < r.size(); ++i) {
			const auto [ax, az] = r[i];
			const auto [bx, bz] = r[(i + 1) % r.size()];
			const double du = bx - ax, dv = bz - az, len = std::hypot(du, dv);
			if (len < 0.5)
				continue;
			perimeter += len;
			if (std::abs(dv) <= std::abs(du) * skew)
				v_lines.emplace_back((az + bz) * 0.5, len);
			else if (std::abs(du) <= std::abs(dv) * skew)
				u_lines.emplace_back((ax + bx) * 0.5, len);
			else
				oblique += len;
		}
	if (perimeter <= 0.0 || oblique > 0.15 * perimeter)
		return std::nullopt;
	const auto merge_lines = [](std::vector<std::pair<double, double>> lines)
			-> std::optional<std::vector<double>> {
		std::sort(lines.begin(), lines.end());
		std::vector<std::vector<std::pair<double, double>>> groups;
		for (const auto &line : lines) {
			if (!groups.empty() && line.first - groups.back().back().first <= 2.0 &&
					line.first - groups.back().front().first <= 3.0)
				groups.back().push_back(line);
			else
				groups.push_back({line});
		}
		if (groups.size() < 2 || groups.size() > 12)
			return std::nullopt;
		std::vector<double> cuts;
		cuts.reserve(groups.size());
		for (std::size_t i = 0; i < groups.size(); ++i) {
			double sum = 0.0, weight = 0.0;
			for (const auto &[position, len] : groups[i]) {
				sum += position * len;
				weight += len;
			}
			if (i == 0)
				cuts.push_back(
						std::min_element(groups[i].begin(), groups[i].end())->first);
			else if (i + 1 == groups.size())
				cuts.push_back(
						std::max_element(groups[i].begin(), groups[i].end())->first);
			else
				cuts.push_back(sum / std::max(weight, 1e-9));
		}
		return cuts;
	};
	auto us_opt = merge_lines(std::move(u_lines));
	auto vs_opt = merge_lines(std::move(v_lines));
	if (!us_opt || !vs_opt)
		return std::nullopt;
	const auto &us = *us_opt;
	const auto &vs = *vs_opt;
	const std::size_t nu = us.size() - 1, nv = vs.size() - 1;
	const auto inside = [](const auto &poly, double x, double z) {
		bool result = false;
		for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
			const auto &[xi, zi] = poly[i];
			const auto &[xj, zj] = poly[j];
			if (((zi > z) != (zj > z)) && x < (xj - xi) * (z - zi) / (zj - zi) + xi)
				result = !result;
		}
		return result;
	};
	std::vector<bool> occupied(nu * nv);
	for (std::size_t j = 0; j < nv; ++j)
		for (std::size_t i = 0; i < nu; ++i)
			occupied[j * nu + i] = [&] {
				const double u = (us[i] + us[i + 1]) * 0.5;
				const double v = (vs[j] + vs[j + 1]) * 0.5;
				if (!inside(rings.front(), u, v))
					return false;
				for (std::size_t k = 1; k < rings.size(); ++k)
					if (inside(rings[k], u, v))
						return false;
				return true;
			}();
	std::vector<int> prefix((nu + 1) * (nv + 1));
	for (std::size_t j = 0; j < nv; ++j)
		for (std::size_t i = 0; i < nu; ++i)
			prefix[(j + 1) * (nu + 1) + i + 1] =
					int(occupied[j * nu + i]) + prefix[j * (nu + 1) + i + 1] +
					prefix[(j + 1) * (nu + 1) + i] - prefix[j * (nu + 1) + i];
	const auto full = [&](std::size_t i0, std::size_t i1, std::size_t j0,
							  std::size_t j1) {
		const int sum = prefix[(j1 + 1) * (nu + 1) + i1 + 1] -
						prefix[j0 * (nu + 1) + i1 + 1] -
						prefix[(j1 + 1) * (nu + 1) + i0] + prefix[j0 * (nu + 1) + i0];
		return sum == static_cast<int>((i1 - i0 + 1) * (j1 - j0 + 1));
	};
	if (full(0, nu - 1, 0, nv - 1))
		return std::nullopt;
	using Rect = std::array<std::size_t, 4>;
	std::vector<Rect> rects;
	for (std::size_t i0 = 0; i0 < nu; ++i0)
		for (std::size_t i1 = i0; i1 < nu; ++i1)
			for (std::size_t j0 = 0; j0 < nv; ++j0)
				for (std::size_t j1 = j0; j1 < nv; ++j1) {
					if (!full(i0, i1, j0, j1))
						break;
					const bool grows = (i0 > 0 && full(i0 - 1, i1, j0, j1)) ||
									   (i1 + 1 < nu && full(i0, i1 + 1, j0, j1)) ||
									   (j0 > 0 && full(i0, i1, j0 - 1, j1)) ||
									   (j1 + 1 < nv && full(i0, i1, j0, j1 + 1));
					if (!grows)
						rects.push_back({i0, i1, j0, j1});
				}
	auto rect_gain = [&](const Rect &r, const std::vector<bool> &covered) {
		double gain = 0.0;
		for (std::size_t j = r[2]; j <= r[3]; ++j)
			for (std::size_t i = r[0]; i <= r[1]; ++i)
				if (!covered[j * nu + i])
					gain += (us[i + 1] - us[i]) * (vs[j + 1] - vs[j]);
		return gain;
	};
	std::vector<bool> covered(nu * nv);
	std::vector<Rect> chosen;
	for (;;) {
		const Rect *best = nullptr;
		double best_gain = -1.0;
		for (const auto &r : rects) {
			if (std::find(chosen.begin(), chosen.end(), r) != chosen.end())
				continue;
			const double gain = rect_gain(r, covered);
			if (gain >= best_gain) {
				best = &r;
				best_gain = gain;
			}
		}
		if (!best || best_gain <= 0.0)
			break;
		chosen.push_back(*best);
		if (chosen.size() > 8)
			return std::nullopt;
		for (std::size_t j = best->at(2); j <= best->at(3); ++j)
			for (std::size_t i = best->at(0); i <= best->at(1); ++i)
				covered[j * nu + i] = true;
	}
	for (std::size_t i = 0; i < occupied.size(); ++i)
		if (occupied[i] && !covered[i])
			return std::nullopt;
	for (std::size_t k = chosen.size(); k > 0; --k) {
		const std::size_t remove = k - 1;
		const auto &r = chosen[remove];
		bool redundant = true;
		for (std::size_t j = r[2]; j <= r[3] && redundant; ++j)
			for (std::size_t i = r[0]; i <= r[1] && redundant; ++i) {
				bool other = false;
				for (std::size_t m = 0; m < chosen.size(); ++m)
					if (m != remove && chosen[m][0] <= i && i <= chosen[m][1] &&
							chosen[m][2] <= j && j <= chosen[m][3]) {
						other = true;
						break;
					}
				redundant = other;
			}
		if (redundant)
			chosen.erase(chosen.begin() + static_cast<std::ptrdiff_t>(remove));
	}
	if (chosen.size() < 2)
		return std::nullopt;
	RoofWingFrame frame{angle, {}};
	for (const auto &r : chosen) {
		RoofWing wing{{us[r[0]], us[r[1] + 1]}, {vs[r[2]], vs[r[3] + 1]}};
		wing.along_u = wing.u.second - wing.u.first >= wing.v.second - wing.v.first;
		frame.wings.push_back(wing);
	}
	if (main_along_x)
		frame.wings[frame.main_wing()].along_u = *main_along_x;
	if (std::all_of(frame.wings.begin(), frame.wings.end(),
				[&](const auto &w) { return w.along_u == frame.wings.front().along_u; }))
		return std::nullopt;
	const auto snapshot = frame.wings;
	const auto dominates = [&](std::size_t a, std::size_t b) {
		if (std::abs(snapshot[a].length() - snapshot[b].length()) > 0.5)
			return snapshot[a].length() > snapshot[b].length();
		if (std::abs(snapshot[a].area() - snapshot[b].area()) > 0.5)
			return snapshot[a].area() > snapshot[b].area();
		return a < b;
	};
	for (std::size_t r = 0; r < snapshot.size(); ++r) {
		const auto ra = snapshot[r].along(), rb = snapshot[r].across();
		std::optional<double> lo, hi;
		for (std::size_t other = 0; other < snapshot.size(); ++other) {
			if (other == r || snapshot[other].along_u == snapshot[r].along_u ||
					!dominates(other, r))
				continue;
			const auto sa = snapshot[other].along();
			if (sa.first > rb.first + 0.5 || sa.second < rb.second - 0.5)
				continue;
			const auto sc = snapshot[other].across();
			const double ridge = (sc.first + sc.second) * 0.5;
			if (ra.second <= sc.second + 0.5 && ra.second > ridge && ra.first < ridge)
				hi = std::max(hi.value_or(ridge), ridge);
			if (ra.first >= sc.first - 0.5 && ra.first < ridge && ra.second > ridge)
				lo = std::min(lo.value_or(ridge), ridge);
		}
		const double a0 = lo.value_or(ra.first), a1 = hi.value_or(ra.second);
		if (a1 - a0 < 1.0)
			continue;
		auto &wing = frame.wings[r];
		if (wing.along_u)
			wing.u = {a0, a1};
		else
			wing.v = {a0, a1};
		wing.clipped = {lo.has_value(), hi.has_value()};
	}
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

int rust_max_inferred_roof_rise(bool steep, double scale_factor)
{
	return std::max(
			2, static_cast<int>(std::lround((steep ? 14.0 : 9.0) * scale_factor)));
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
		std::optional<int32_t> mansard_band = std::nullopt, DormerStyle style = {},
		const std::unordered_map<std::pair<int32_t, int32_t>, std::pair<int32_t, int32_t>,
				pair_hash> *edge_scans_override = nullptr,
		const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash>
				*footprint_override = nullptr)
{
	if (!enabled || floor_area.empty())
		return;
	using Cell = std::pair<int32_t, int32_t>;
	std::unordered_set<Cell, pair_hash> local_footprint(
			floor_area.begin(), floor_area.end());
	const auto &footprint = footprint_override ? *footprint_override : local_footprint;
	const auto [px, pz] = parallel_to_ridge;
	const Cell perp_unit = px != 0 ? Cell{0, 1} : Cell{1, 0};
	auto scan = [&](int32_t x, int32_t z, int dx, int dz) {
		int distance = 0;
		while (footprint.contains({x + (distance + 1) * dx, z + (distance + 1) * dz}))
			++distance;
		return distance;
	};

	std::vector<std::tuple<int32_t, int32_t, int32_t, int32_t>> candidates;
	for (const auto &[x, z] : floor_area) {
		const auto h_it = roof_heights.find({x, z});
		if (h_it == roof_heights.end())
			continue;
		const int h = h_it->second;
		if (h != base_height + 2 && h != base_height + 3 &&
				(!mansard_band || h != *mansard_band))
			continue;
		int dm, dp;
		if (edge_scans_override) {
			const auto scan_it = edge_scans_override->find({x, z});
			if (scan_it != edge_scans_override->end()) {
				dm = scan_it->second.first;
				dp = scan_it->second.second;
			} else {
				dm = scan(x, z, -perp_unit.first, -perp_unit.second);
				dp = scan(x, z, perp_unit.first, perp_unit.second);
			}
		} else {
			dm = scan(x, z, -perp_unit.first, -perp_unit.second);
			dp = scan(x, z, perp_unit.first, perp_unit.second);
		}
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
		candidates.emplace_back(x, z, h, -inward);
	}
	if (candidates.empty())
		return;
	std::sort(candidates.begin(), candidates.end());
	auto rng = element_rng(style_seed ^ 0x000D04E4DEC0ULL);
	std::vector<std::tuple<int32_t, int32_t, int32_t>> chosen;
	const auto along = [px = px](int x, int z) { return px != 0 ? x : z; };
	const auto on_axis = [&](int u) {
		return !style.window_phase || (u + *style.window_phase) % 6 == 1 ||
			   ((u + *style.window_phase) % 6 + 6) % 6 == 1;
	};
	std::array<std::vector<decltype(candidates)::value_type>, 2> sides;
	for (const auto &candidate : candidates) {
		const auto [x, z, h, side] = candidate;
		if (on_axis(along(x, z)))
			sides[side > 0 ? 1 : 0].push_back(candidate);
	}
	if (mansard_band) {
		// A mansard has a dormer row on both slopes.
	} else if (sides[0].size() != sides[1].size()) {
		sides[1 - (sides[0].size() > sides[1].size() ? 0 : 1)].clear();
	} else if (rng.random_bool(.5)) {
		sides[1].clear();
	} else {
		sides[0].clear();
	}
	for (auto &side_candidates : sides) {
		if (side_candidates.empty())
			continue;
		std::map<int, std::size_t> row_counts;
		for (const auto &[x, z, h, side] : side_candidates) {
			(void)x;
			(void)z;
			(void)side;
			++row_counts[h];
		}
		const int row = std::max_element(
				row_counts.begin(), row_counts.end(), [](const auto &a, const auto &b) {
					return a.second != b.second ? a.second < b.second : a.first > b.first;
				})->first;
		std::vector<std::tuple<int32_t, int32_t, int32_t, int32_t>> row_candidates;
		for (const auto &candidate : side_candidates)
			if (std::get<2>(candidate) == row)
				row_candidates.push_back(candidate);
		std::sort(row_candidates.begin(), row_candidates.end(),
				[&](const auto &a, const auto &b) {
					return along(std::get<0>(a), std::get<1>(a)) <
						   along(std::get<0>(b), std::get<1>(b));
				});
		if (!mansard_band) {
			const auto parity = [&](const auto &candidate) {
				const int phase = style.window_phase.value_or(0);
				return ((floor_div(along(std::get<0>(candidate), std::get<1>(candidate)) +
										   phase,
								 6) %
								2) +
							   2) %
					   2;
			};
			const auto even = static_cast<std::size_t>(std::count_if(
					row_candidates.begin(), row_candidates.end(),
					[&](const auto &candidate) { return parity(candidate) == 0; }));
			const auto odd = row_candidates.size() - even;
			const int keep =
					even == odd ? static_cast<int>(rng.uniform(2)) : (odd > even ? 1 : 0);
			std::erase_if(row_candidates,
					[&](const auto &candidate) { return parity(candidate) != keep; });
		}
		for (const auto &candidate : row_candidates) {
			const auto [x, z, h, side] = candidate;
			(void)side;
			const bool too_close = std::any_of(chosen.begin(), chosen.end(),
					[&, x = x, z = z, px = px, pz = pz](const auto &other) {
						return std::abs((std::get<0>(other) - x) * px +
										(std::get<1>(other) - z) * pz) < 4;
					});
			if (!too_close)
				chosen.emplace_back(x, z, h);
		}
	}

	const StairFacing left_facing = px != 0 ? StairFacing::East : StairFacing::South;
	const StairFacing right_facing = px != 0 ? StairFacing::West : StairFacing::North;
	const Block stair_material = style.copper_hood
										 ? WAXED_OXIDIZED_CUT_COPPER_STAIRS
										 : get_stair_block_for_material(roof_block);
	const Block flank = style.gable_block.value_or(roof_block);
	const Block glass = style.window_block.value_or(LIGHT_GRAY_STAINED_GLASS);
	const Block cap_centre = style.copper_hood ? WAXED_OXIDIZED_COPPER : roof_block;
	const auto replace_any = std::optional<std::vector<Block>>(std::vector<Block>{});
	auto put = [&](Block block, int x, int y, int z) {
		editor.set_block_absolute(block, x, y, z, std::nullopt, replace_any);
	};
	for (const auto &[x, z, h] : chosen) {
		const int lower_y = h - 1 + abs_terrain_offset;
		const int face_y = h + abs_terrain_offset;
		const int cap_y = h + 1 + abs_terrain_offset;
		int dm = scan(x, z, -perp_unit.first, -perp_unit.second);
		int dp = scan(x, z, perp_unit.first, perp_unit.second);
		if (edge_scans_override) {
			const auto scan_it = edge_scans_override->find({x, z});
			if (scan_it != edge_scans_override->end()) {
				dm = scan_it->second.first;
				dp = scan_it->second.second;
			}
		}
		const int ox = perp_unit.first * (dm <= dp ? -1 : 1);
		const int oz = perp_unit.second * (dm <= dp ? -1 : 1);
		const Cell extension{x + ox, z + oz};
		const bool extension_inside = footprint.contains(extension);
		for (int along : {-1, 0, 1}) {
			put(flank, x + along * px, lower_y, z + along * pz);
			put(flank, x + along * px, face_y, z + along * pz);
			if (extension_inside) {
				put(flank, extension.first + along * px, lower_y,
						extension.second + along * pz);
				put(flank, extension.first + along * px, face_y,
						extension.second + along * pz);
			}
		}
		put(glass, x, face_y, z);
		if (extension_inside)
			put(glass, extension.first, face_y, extension.second);
		for (int along : {-1, 1})
			editor.set_block_with_properties_absolute(
					create_stair_with_properties(stair_material,
							along < 0 ? left_facing : right_facing, StairShape::Straight),
					x + along * px, cap_y, z + along * pz, std::nullopt, replace_any);
		put(cap_centre, x, cap_y, z);
		if (extension_inside) {
			for (int along : {-1, 1})
				editor.set_block_with_properties_absolute(
						create_stair_with_properties(stair_material,
								along < 0 ? left_facing : right_facing,
								StairShape::Straight),
						extension.first + along * px, cap_y,
						extension.second + along * pz, std::nullopt, replace_any);
			put(cap_centre, extension.first, cap_y, extension.second);
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

bool pitched_roof(RoofType roof)
{
	return roof == RoofType::Gabled || roof == RoofType::Hipped ||
		   roof == RoofType::HalfHipped || roof == RoofType::Gambrel ||
		   roof == RoofType::Pyramidal || roof == RoofType::Mansard;
}

std::optional<int> mapped_height_roof_rise(const ProcessedWay &element, RoofType roof,
		BuildingCondition condition, int wall_height, std::size_t footprint_cells,
		bool steep, double scale_factor)
{
	if (!pitched_roof(roof) || !parse_building_height_tag(element.tags) ||
			element.tags.contains("roof:height") || element.tags.contains("roof:angle") ||
			(condition != BuildingCondition::Normal &&
					condition != BuildingCondition::Disused &&
					condition != BuildingCondition::Abandoned))
		return std::nullopt;

	double diameter = 1.0;
	for (std::size_t i = 0; i < element.nodes.size(); ++i)
		for (std::size_t j = i + 1; j < element.nodes.size(); ++j) {
			const double dx = static_cast<double>(element.nodes[i].x) -
							  static_cast<double>(element.nodes[j].x);
			const double dz = static_cast<double>(element.nodes[i].z) -
							  static_cast<double>(element.nodes[j].z);
			diameter = std::max(diameter, std::hypot(dx, dz));
		}
	const double short_half = static_cast<double>(footprint_cells) / diameter / 2.0;
	const double slope_rise =
			roof == RoofType::Mansard
					? 4.0 * scale_factor + std::max((short_half - 2.0) / 2.0, 0.0)
					: short_half * 0.85;
	const double wall_share = steep ? 0.47 : 0.375;
	const double rise = std::min({slope_rise,
			static_cast<double>(rust_max_inferred_roof_rise(steep, scale_factor)),
			static_cast<double>(wall_height) * wall_share});
	const int rounded_rise = static_cast<int>(std::round(rise));
	return rounded_rise >= 1 ? std::optional<int>(rounded_rise) : std::nullopt;
}

// For mapped-height buildings, resolve the same automatic roof choice before
// the shell is emitted. The later roof pass consumes this choice rather than
// rerolling after the wall height has been shortened.
std::optional<RoofType> mapped_height_auto_roof(const ProcessedWay &element,
		const std::string &building_type, BuildingCategory category, ArchEra era,
		biome::Climate climate, building_style::MaterialRegion materials,
		std::size_t footprint, int height, double scale, bool attached, bool enclosed,
		bool part_has_explicit_top, std::uint64_t seed)
{
	if (element.tags.contains("roof:shape") || part_has_explicit_top || enclosed)
		return std::nullopt;

	const bool preset_flat_roof = category == BuildingCategory::ModernSkyscraper ||
								  category == BuildingCategory::GlassySkyscraper ||
								  category == BuildingCategory::GlassCornerSkyscraper ||
								  category == BuildingCategory::GridSkyscraper ||
								  category == BuildingCategory::ContemporarySkyscraper ||
								  category == BuildingCategory::MasonrySkyscraper ||
								  category == BuildingCategory::Tower ||
								  category == BuildingCategory::Garage ||
								  category == BuildingCategory::Greenhouse;
	if (preset_flat_roof)
		return std::nullopt;

	const auto dominant_region = building_style::building_region(materials);
	const double scale_sq = scale * scale;
	const double footprint_m2 = static_cast<double>(footprint) / scale_sq;
	const double height_m = static_cast<double>(height) / scale;
	const bool qualifies = building_type == "apartments" ||
						   building_type == "residential" || building_type == "house" ||
						   building_type == "yes" || building_type == "detached" ||
						   building_type == "semidetached_house" ||
						   building_type == "terrace" || building_type == "bungalow" ||
						   building_type == "villa" || building_type == "cabin" ||
						   building_type == "hut" || building_type == "farm" ||
						   building_type == "farm_auxiliary" || building_type == "barn" ||
						   building_type == "stable" || building_type == "cowshed" ||
						   building_type == "sty" || building_type == "sheepfold";
	const bool house = building_type == "house" || building_type == "detached" ||
					   building_type == "semidetached_house" ||
					   building_type == "bungalow" || building_type == "villa" ||
					   building_type == "cabin" || building_type == "hut" ||
					   building_type == "farm" || building_type == "farm_auxiliary" ||
					   building_type == "barn" || building_type == "stable" ||
					   building_type == "cowshed" || building_type == "sty" ||
					   building_type == "sheepfold" || building_type == "terrace" ||
					   ((building_type == "yes" || building_type == "residential") &&
							   !attached && footprint_m2 <= 220.0 && height_m <= 11.0);

	if (qualifies && !element.tags.contains("building:part")) {
		if (const auto habit = building_style::roof_habit(materials)) {
			double chance = footprint_m2 > 1500.0 ? 0.0
							: house				  ? habit->house
							: height_m <= habit->block_max_m
									? ((era == ArchEra::PostWarPanel ||
											   era == ArchEra::Contemporary)
													  ? habit->block * .5
													  : habit->block)
									: 0.0;
			if (materials != building_style::MaterialRegion::Australasia) {
				if (climate == biome::Climate::HotDesert)
					chance *= .35;
				else if (climate == biome::Climate::HotSteppe)
					chance *= .7;
			}
			auto rng = element_rng(seed ^ 0x2E610A1F0F5E7001ULL);
			if (!rng.random_bool(chance))
				return RoofType::Flat;
			return attached || !rng.random_bool(habit->hipped) ? RoofType::Gabled
															   : RoofType::Hipped;
		}
	}

	if (dominant_region == building_style::BuildingRegion::Europe &&
			!building_style::roof_habit(materials) &&
			!element.tags.contains("building:part")) {
		const bool civic_or_commercial =
				building_type == "commercial" || building_type == "retail" ||
				building_type == "office" || building_type == "hotel" ||
				building_type == "public" || building_type == "civic" ||
				building_type == "government" || building_type == "school" ||
				building_type == "university" || building_type == "college" ||
				building_type == "kindergarten" || building_type == "townhall" ||
				building_type == "historic" || building_type == "palace" ||
				building_type == "castle" || building_type == "museum" ||
				building_type == "theatre";
		const bool urban_type = building_type == "apartments" ||
								building_type == "residential" ||
								building_type == "yes" || building_type == "terrace" ||
								civic_or_commercial;
		if ((urban_type || qualifies) && height_m <= 26.0 && footprint_m2 <= 3000.0) {
			double bbox_cells = 1.0;
			if (!element.nodes.empty()) {
				int min_x = element.nodes.front().x, max_x = min_x;
				int min_z = element.nodes.front().z, max_z = min_z;
				for (const auto &node : element.nodes) {
					min_x = std::min(min_x, node.x);
					max_x = std::max(max_x, node.x);
					min_z = std::min(min_z, node.z);
					max_z = std::max(max_z, node.z);
				}
				bbox_cells =
						std::max(1.0, static_cast<double>(max_x - min_x + 1) *
											  static_cast<double>(max_z - min_z + 1));
			}
			if (static_cast<double>(footprint) / bbox_cells >= .5) {
				const bool commercial =
						civic_or_commercial && building_type != "historic" &&
						building_type != "palace" && building_type != "castle" &&
						building_type != "townhall";
				double chance = 0.0;
				switch (era) {
				case ArchEra::HistoricOrnate:
					chance = .95;
					break;
				case ArchEra::TraditionalPreWar:
					chance = .88;
					break;
				case ArchEra::PostWarPanel:
					chance = commercial ? .10 : .40;
					break;
				case ArchEra::Contemporary:
					chance = commercial ? .05 : .25;
					break;
				case ArchEra::Unknown:
					chance = attached ? .80 : (footprint_m2 > 1500.0 ? .40 : .65);
					break;
				}
				auto rng = element_rng(seed ^ 0x5E0FEA570C1D70B5ULL);
				if (!rng.random_bool(chance))
					return RoofType::Flat;
				const bool tall = height >= 14.0 * scale;
				if (era == ArchEra::HistoricOrnate && footprint_m2 >= 400.0 && tall &&
						rng.random_bool(.15))
					return RoofType::Mansard;
				return attached ? RoofType::Gabled : RoofType::Hipped;
			}
		}
	}

	const bool auto_roof_type = category != BuildingCategory::TallBuilding &&
								category != BuildingCategory::Hospital &&
								category != BuildingCategory::Historic && qualifies;
	if (auto_roof_type) {
		constexpr std::size_t max_footprint_for_gabled = 800;
		auto rng = element_rng(seed ^ 0x0F1E2D3C4B5A6907ULL);
		const bool big_block = footprint > max_footprint_for_gabled || height >= 15;
		const bool north_american_flat =
				dominant_region == building_style::BuildingRegion::NorthAmerica &&
				(attached || height >= 13.0 * scale ||
						(!house && footprint > 300.0 * scale_sq));
		if (north_american_flat)
			return RoofType::Flat;
		RoofType automatic_roof = RoofType::Flat;
		bool make_roof = false;
		if (building_type == "apartments" && big_block) {
			const bool prewar =
					era == ArchEra::HistoricOrnate || era == ArchEra::TraditionalPreWar;
			const auto roll = rng.uniform(100);
			const std::uint32_t hip_limit = prewar ? 65u : 80u;
			if (roll >= 45) {
				automatic_roof = roll < hip_limit ? RoofType::Hipped : RoofType::Mansard;
				make_roof = true;
			}
		} else if ((era == ArchEra::HistoricOrnate ||
						   era == ArchEra::TraditionalPreWar) &&
				   footprint >= 100 && footprint <= max_footprint_for_gabled &&
				   rng.random_bool(.30)) {
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
			if (footprint <= max_footprint_for_gabled &&
					rng.random_bool(gable_probability)) {
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
		return make_roof ? automatic_roof : RoofType::Flat;
	}

	return std::nullopt;
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

int terrain_base_level(WorldEditor &editor, const ProcessedWay &element, int fallback,
		const std::vector<std::pair<int, int>> &footprint)
{
	std::optional<int> highest;
	const auto include = [&](int x, int z) {
		if (const auto level = editor.terrain_level(x, z))
			highest = highest ? std::max(*highest, *level) : *level;
	};
	for (const auto &node : element.nodes)
		include(node.x, node.z);
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
	if (terrain_enabled) {
		const int terrain_base =
				terrain_base_level(editor, element, args.ground_level, cached_floor_area);
		start_y_offset = terrain_base + min_level_offset;
	}

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
		int abs_offset, int floor_cycle, Block wall_block, DetailTier detail, ArchEra era,
		std::uint64_t seed, bool covered_by_sibling, double scale, bool green_roof)
{
	const bool sloped = roof_type != RoofType::Flat;
	if (area.empty() || (!sloped && area.size() <= THIN_RING_MAX_CELLS) ||
			condition == BuildingCondition::Construction ||
			condition == BuildingCondition::Ruined)
		return;
	std::unordered_set<std::pair<int, int>, PairHash> footprint(area.begin(), area.end());
	const int roof_y = start_y + building_height + abs_offset + 1;
	int min_x = INT_MAX, max_x = INT_MIN, min_z = INT_MAX, max_z = INT_MIN;
	for (const auto &[x, z] : area) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	const int center_x = (min_x + max_x) / 2, center_z = (min_z + max_z) / 2;
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

	const auto building_type = element.tags.get("building");
	const bool residential_type =
			building_type == "apartments" || building_type == "residential" ||
			building_type == "dormitory" || building_type == "house" ||
			building_type == "detached" || building_type == "semidetached_house" ||
			building_type == "terrace" || building_type == "bungalow" ||
			building_type == "villa";
	const bool residential = category == BuildingCategory::House ||
							 category == BuildingCategory::Residential ||
							 residential_type;
	const bool terrace = element.tags.contains("building:part") &&
						 building_height >= 28 && area.size() > THIN_RING_MAX_CELLS &&
						 residential;
	if (terrace) {
		const auto [latitude, longitude] = editor.geo_anchor();
		const auto materials = building_style::material_region_at(latitude, longitude);
		rooftops::RooftopBrief brief;
		brief.region = building_style::building_region(materials);
		brief.category = category;
		brief.era = era;
		brief.height = building_height;
		brief.floor_cycle = floor_cycle;
		brief.wall = wall_block;
		brief.scale = scale;
		brief.seed = element.id;
		brief.residential = residential;
		brief.whole_building = false;
		brief.sparse =
				detail == DetailTier::Minimal || condition != BuildingCondition::Normal;
		brief.green = green_roof;
		if (auto deck = rooftops::RoofDeck::make(area, roof_y))
			rooftops::fit_out_terrace(editor, *deck, brief);
		return;
	}

	// Keep the C++ roof fit-out on the same reusable planner as Rust. Flat roofs
	// on service buildings remain deliberately excluded; the former inline pass
	// below is retained temporarily as a reference during this port.
	const bool roof_equipment_excluded = category == BuildingCategory::Farm ||
										 category == BuildingCategory::Garage ||
										 category == BuildingCategory::Shed ||
										 category == BuildingCategory::Greenhouse ||
										 category == BuildingCategory::Religious;
	if (sloped || building_height < 4 || roof_equipment_excluded)
		return;
	const auto [latitude, longitude] = editor.geo_anchor();
	const auto materials = building_style::material_region_at(latitude, longitude);
	rooftops::RooftopBrief brief;
	brief.region = building_style::building_region(materials);
	brief.category = category;
	brief.era = era;
	brief.height = building_height;
	brief.floor_cycle = floor_cycle;
	brief.wall = wall_block;
	brief.scale = scale;
	brief.seed = element.id;
	brief.residential = residential;
	brief.whole_building = !element.tags.contains("building:part");
	brief.sparse =
			detail == DetailTier::Minimal || condition != BuildingCondition::Normal;
	brief.green = green_roof;
	if (auto deck = rooftops::RoofDeck::make(area, roof_y)) {
		if (category == BuildingCategory::Hospital)
			deck->reserve(center_x - 3, center_z - 3, 7, 7);
		rooftops::fit_out_roof(editor, *deck, brief);
	}
	return;

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

	// Plan roof plant against distance-to-edge rather than sampling independent
	// interior cells. This keeps rooftop equipment in usable rectangles and gives
	// the rows a stable orientation across footprint traversal order.
	std::vector<std::pair<int, int>> roof_cells = area;
	std::sort(roof_cells.begin(), roof_cells.end());
	roof_cells.erase(std::unique(roof_cells.begin(), roof_cells.end()), roof_cells.end());
	std::unordered_map<std::pair<int, int>, int, PairHash> edge_distance;
	std::deque<std::pair<int, int>> edge_queue;
	for (const auto &[x, z] : roof_cells) {
		for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
			if (!footprint.contains({x + dx, z + dz})) {
				edge_distance[{x, z}] = 0;
				edge_queue.emplace_back(x, z);
				break;
			}
		}
	}
	while (!edge_queue.empty()) {
		const auto [x, z] = edge_queue.front();
		edge_queue.pop_front();
		const int next = edge_distance.at({x, z}) + 1;
		for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
			const std::pair neighbor{x + dx, z + dz};
			if (footprint.contains(neighbor) && !edge_distance.contains(neighbor)) {
				edge_distance.emplace(neighbor, next);
				edge_queue.push_back(neighbor);
			}
		}
	}
	int deepest = -1;
	for (const auto &[cell, distance] : edge_distance) {
		(void)cell;
		deepest = std::max(deepest, distance);
	}
	std::vector<std::pair<int, int>> usable_cells;
	for (const auto &cell : roof_cells)
		if (edge_distance[cell] >= 2)
			usable_cells.push_back(cell);
	if (usable_cells.empty())
		return;
	std::pair<int, int> heart = {center_x, center_z};
	std::int64_t heart_distance = std::numeric_limits<std::int64_t>::max();
	for (const auto &[x, z] : roof_cells) {
		if (edge_distance[{x, z}] != deepest)
			continue;
		const std::int64_t dx = 2LL * x - min_x - max_x;
		const std::int64_t dz = 2LL * z - min_z - max_z;
		const auto distance = dx * dx + dz * dz;
		if (distance < heart_distance) {
			heart = {x, z};
			heart_distance = distance;
		}
	}
	const int grid_width = max_x - min_x + 1;
	const int grid_depth = max_z - min_z + 1;
	const bool along_x = grid_width >= grid_depth;
	const std::pair<int, int> axis = along_x ? std::pair{1, 0} : std::pair{0, 1};
	const std::pair<int, int> across{axis.second, axis.first};
	const int long_side = along_x ? grid_width : grid_depth;
	const int short_side = along_x ? grid_depth : grid_width;
	auto roof_rng = element_rng(element.id ^ 0x2F0F70F500000001ULL);
	const int flip = roof_rng.random_bool(.5) ? 1 : -1;
	const auto step = [](std::pair<int, int> p, std::pair<int, int> d, int n) {
		return std::pair{p.first + d.first * n, p.second + d.second * n};
	};
	const auto oriented = [&](int len, int width) {
		return along_x ? std::pair{len, width} : std::pair{width, len};
	};
	const int side_offset = std::max(short_side / 4, 2);
	const int reach = std::max(long_side, short_side) / 2;
	const auto find_roof_rect =
			[&](std::pair<int, int> target, int width, int depth, int min_edge,
					int search_reach) -> std::optional<std::pair<int, int>> {
		if (width > grid_width - 2 * min_edge || depth > grid_depth - 2 * min_edge ||
				deepest < min_edge + (std::min(width, depth) - 1) / 2)
			return std::nullopt;
		const int ox = target.first - width / 2;
		const int oz = target.second - depth / 2;
		const auto fits = [&](int x0, int z0) {
			for (int z = z0; z < z0 + depth; ++z)
				for (int x = x0; x < x0 + width; ++x)
					if (edge_distance[{x, z}] < min_edge)
						return false;
			for (int z = z0 - 1; z <= z0 + depth; ++z)
				for (int x = x0 - 1; x <= x0 + width; ++x)
					if (used.contains({x, z}))
						return false;
			return true;
		};
		for (int r = 0; r <= search_reach; ++r) {
			for (int d = -r; d <= r; ++d) {
				for (const int dz : {-r, r})
					if (fits(ox + d, oz + dz))
						return std::pair{ox + d, oz + dz};
			}
			for (int d = -r + 1; d < r; ++d)
				for (const int dx : {-r, r})
					if (fits(ox + dx, oz + d))
						return std::pair{ox + dx, oz + d};
		}
		return std::nullopt;
	};
	const auto reserve_rect = [&](int x0, int z0, int width, int depth) {
		for (int z = z0 - 1; z <= z0 + depth; ++z)
			for (int x = x0 - 1; x <= x0 + width; ++x)
				used.insert({x, z});
	};
	const auto end_a = step(heart, axis, -flip * long_side / 4);
	const auto end_b = step(heart, axis, flip * long_side / 4);
	const auto [lat, lon] = editor.geo_anchor();
	const auto material_region = building_style::material_region_at(lat, lon);
	const auto building_region = building_style::building_region(material_region);
	const bool north_america =
			building_region == building_style::BuildingRegion::NorthAmerica;
	const bool europe = building_region == building_style::BuildingRegion::Europe;
	double density = north_america ? 1.0 : (europe ? .5 : .8);
	if (detail == DetailTier::Minimal)
		density *= .5;
	if (green_roof)
		density *= .3;
	const auto count_roof_props = [&](double per_cells, int cap) {
		return std::min(cap,
				static_cast<int>(std::lround(
						static_cast<double>(usable_cells.size()) / per_cells * density)));
	};

	// Tall towers commonly have a smaller plant room on the roof. Rust's
	// RooftopBrief planner reserves the inset roof cells for this penthouse
	// before fitting bulkheads and mechanical plant; mirror that ordering here.
	const bool whole_building = !element.tags.contains("building:part");
	const bool roof_tower =
			whole_building &&
			building_height >= static_cast<int>(std::lround(36.0 * scale)) &&
			usable_cells.size() >= 80;
	const double penthouse_share = north_america ? .5 : (europe ? .3 : .4);
	bool penthouse = false;
	if (roof_tower && roof_rng.random_bool(penthouse_share)) {
		const auto count_at_least = [&](int edge) {
			return static_cast<std::size_t>(std::count_if(roof_cells.begin(),
					roof_cells.end(),
					[&](const auto &cell) { return edge_distance.at(cell) >= edge; }));
		};
		int inset = 3;
		while (inset < deepest && count_at_least(inset) * 20 > roof_cells.size() * 9)
			++inset;
		std::vector<std::pair<int, int>> penthouse_cells;
		for (const auto &cell : roof_cells)
			if (edge_distance.at(cell) >= inset)
				penthouse_cells.push_back(cell);
		bool fits = penthouse_cells.size() * 100 >= roof_cells.size() * 20 &&
					penthouse_cells.size() >= 36;
		// Keep a one-cell clearance from the penthouse to earlier reservations
		// (notably a helipad added by another building pass).
		for (const auto &[x, z] : penthouse_cells) {
			for (int dz = -1; dz <= 1 && fits; ++dz)
				for (int dx = -1; dx <= 1; ++dx)
					if (used.contains({x + dx, z + dz})) {
						fits = false;
						break;
					}
		}
		if (fits) {
			const auto penthouse_seed = element.id ^ 0x9E470A5E00000001ULL;
			auto penthouse_rng = element_rng(penthouse_seed);
			const int penthouse_storeys =
					building_height >= static_cast<int>(std::lround(120.0 * scale)) &&
									penthouse_cells.size() >= 300 &&
									penthouse_rng.random_bool(.5)
							? 2
							: 1;
			const int penthouse_height = std::max(3, floor_cycle) * penthouse_storeys;
			Block penthouse_wall = wall_block;
			if (is_glass_wall_block(wall_block) || penthouse_rng.random_bool(.5)) {
				const std::array<Block, 3> choices = {
						LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, SMOOTH_STONE};
				penthouse_wall = choices[penthouse_rng.uniform(3)];
			}
			const std::array<Block, 3> roof_choices = {
					GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE};
			const Block penthouse_roof = roof_choices[penthouse_rng.uniform(3)];
			const int penthouse_base = roof_y + 1;
			for (const auto &[x, z] : penthouse_cells) {
				std::optional<std::pair<int, int>> open;
				for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
					if (edge_distance[{x + dx, z + dz}] < inset) {
						open = std::pair{dx, dz};
						break;
					}
				if (open) {
					const auto [ox, oz] = *open;
					const int coordinate = ox != 0 ? z : x;
					const bool louvre_bay = ((coordinate % 6) + 6) % 6 >= 2 &&
											((coordinate % 6) + 6) % 6 <= 4;
					for (int k = 0; k < penthouse_height; ++k) {
						const int y = penthouse_base + k;
						if (louvre_bay && penthouse_height >= 4 &&
								k >= penthouse_height - 3 && k < penthouse_height - 1) {
							force_rooftop_block(IRON_BARS, x, y, z);
							force_rooftop_block(BLACK_CONCRETE, x - ox, y, z - oz);
						} else {
							force_rooftop_block(penthouse_wall, x, y, z);
						}
					}
					force_rooftop_block(SMOOTH_STONE_SLAB, x,
							penthouse_base + penthouse_height + 1, z);
				}
				force_rooftop_block(
						penthouse_roof, x, penthouse_base + penthouse_height, z);
				used.insert({x, z});
			}
			penthouse = true;
			if (penthouse_rng.random_bool(.6)) {
				const auto [x, z] = heart;
				const int mast_height =
						std::max(1, static_cast<int>(std::lround(6.0 * scale)));
				for (int dy = 0; dy < mast_height; ++dy)
					force_rooftop_block(
							IRON_BARS, x, penthouse_base + penthouse_height + 1 + dy, z);
				force_rooftop_block(LIGHTNING_ROD, x,
						penthouse_base + penthouse_height + 1 + mast_height, z);
			}
		}

		std::optional<std::tuple<int, int, int>> bulkhead_top;
		const int storeys = building_height / std::max(3, floor_cycle);
		if (!penthouse && whole_building && storeys >= 4 && usable_cells.size() >= 24 &&
				roof_rng.random_bool(north_america ? .9 : (europe ? .6 : .8))) {
			const bool big = usable_cells.size() >= 400;
			const auto [width, depth] =
					oriented(static_cast<int>(std::lround((big ? 5 : 4) * scale)),
							static_cast<int>(std::lround((big ? 4 : 3) * scale)));
			const int room_height = std::max(
					2, static_cast<int>(
							   std::lround((building_height >= 40 ? 4 : 3) * scale)));
			const bool pair = usable_cells.size() >= 500 && roof_rng.random_bool(.5);
			Block bulkhead_wall = wall_block;
			if (is_glass_wall_block(wall_block) || !roof_rng.random_bool(.55)) {
				const std::array<Block, 3> choices = {
						LIGHT_GRAY_CONCRETE, SMOOTH_STONE, WHITE_CONCRETE};
				bulkhead_wall = choices[roof_rng.uniform(3)];
			}
			const Block cap = roof_rng.random_bool(.6) ? POLISHED_BLACKSTONE_SLAB
													   : SMOOTH_STONE_SLAB;
			const auto cap_slab = cached_prop_block(cap, {{"type", "bottom"}});
			const bool iron_door = roof_rng.random_bool(.5);
			for (int target_index = 0; target_index < (pair ? 2 : 1); ++target_index) {
				const auto target = target_index == 0 ? end_a : end_b;
				const auto origin = find_roof_rect(target, width, depth, 2, reach);
				if (!origin)
					continue;
				const auto [x0, z0] = *origin;
				const int cx = x0 + width / 2;
				const int cz = z0 + depth / 2;
				const int door_x = heart.first >= cx ? x0 + width - 1 : x0;
				const int door_z = heart.second >= cz ? z0 + depth - 1 : z0;
				for (int y = 1; y <= room_height; ++y) {
					for (int x = x0; x < x0 + width; ++x) {
						if (!(!along_x && x == cx && door_z == z0 && y <= 2))
							force_rooftop_block(bulkhead_wall, x, roof_y + y, z0);
						if (!(!along_x && x == cx && door_z == z0 + depth - 1 && y <= 2))
							force_rooftop_block(
									bulkhead_wall, x, roof_y + y, z0 + depth - 1);
					}
					for (int z = z0; z < z0 + depth; ++z) {
						if (!(along_x && z == cz && door_x == x0 && y <= 2))
							force_rooftop_block(bulkhead_wall, x0, roof_y + y, z);
						if (!(along_x && z == cz && door_x == x0 + width - 1 && y <= 2))
							force_rooftop_block(
									bulkhead_wall, x0 + width - 1, roof_y + y, z);
					}
				}
				const int door_x_pos = along_x ? door_x : cx;
				const int door_z_pos = along_x ? cz : door_z;
				const int facing_x = along_x ? (door_x == x0 + width - 1 ? 1 : -1) : 0;
				const int facing_z = along_x ? 0 : (door_z == z0 + depth - 1 ? 1 : -1);
				const char *facing = facing_x > 0	? "east"
									 : facing_x < 0 ? "west"
									 : facing_z > 0 ? "south"
													: "north";
				const Block door_lower = iron_door ? IRON_DOOR : DARK_OAK_DOOR_LOWER;
				const Block door_upper = iron_door ? IRON_DOOR : DARK_OAK_DOOR_UPPER;
				editor.set_block_with_properties_absolute(
						cached_prop_block(
								door_lower, {{"half", "lower"}, {"facing", facing},
													{"hinge", "left"}}),
						door_x_pos, roof_y + 1, door_z_pos, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				editor.set_block_with_properties_absolute(
						cached_prop_block(
								door_upper, {{"half", "upper"}, {"facing", facing},
													{"hinge", "left"}}),
						door_x_pos, roof_y + 2, door_z_pos, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				if (room_height >= 3) {
					const int window_x =
							along_x ? (door_x == x0 ? x0 + width - 1 : x0) : cx;
					const int window_z =
							along_x ? cz : (door_z == z0 ? z0 + depth - 1 : z0);
					force_rooftop_block(
							GRAY_STAINED_GLASS, window_x, roof_y + 2, window_z);
				}
				for (int z = z0; z < z0 + depth; ++z)
					for (int x = x0; x < x0 + width; ++x)
						editor.set_block_with_properties_absolute(cap_slab, x,
								roof_y + room_height + 1, z, std::nullopt,
								std::optional<std::vector<Block>>(std::vector<Block>{}));
				if (!bulkhead_top)
					bulkhead_top = std::make_tuple(cx, cz, room_height + 1);
				reserve_rect(x0, z0, width, depth);
			}
		} else if (!penthouse && usable_cells.size() >= 6 && roof_rng.random_bool(.6)) {
			if (const auto hatch = find_roof_rect(end_a, 1, 1, 2, reach)) {
				editor.set_block_with_properties_absolute(
						cached_prop_block(
								IRON_TRAPDOOR, {{"facing", "north"}, {"half", "bottom"},
													   {"open", "false"}}),
						hatch->first, roof_y + 1, hatch->second, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
				reserve_rect(hatch->first, hatch->second, 1, 1);
			}
		}

		const double tank_share = era == ArchEra::HistoricOrnate ||
												  era == ArchEra::TraditionalPreWar ||
												  era == ArchEra::Unknown
										  ? .45
								  : era == ArchEra::PostWarPanel ? .25
																 : .06;
		const bool tank_building = category != BuildingCategory::GlassySkyscraper &&
								   category != BuildingCategory::GlassCornerSkyscraper &&
								   category != BuildingCategory::GridSkyscraper &&
								   category != BuildingCategory::ContemporarySkyscraper &&
								   category != BuildingCategory::ModernSkyscraper &&
								   category != BuildingCategory::Religious &&
								   category != BuildingCategory::Hospital &&
								   category != BuildingCategory::School &&
								   category != BuildingCategory::House &&
								   category != BuildingCategory::Greenhouse;
		if (north_america && whole_building && !penthouse && tank_building &&
				storeys >= 6 && building_height <= std::lround(160.0 * scale) &&
				usable_cells.size() >= 20 && roof_rng.random_bool(tank_share)) {
			const bool large = usable_cells.size() >= 90;
			const int radius = static_cast<int>(std::lround((large ? 2 : 1) * scale));
			const int size = 2 * radius + 1;
			const auto target =
					bulkhead_top ? end_b : step(heart, axis, flip * long_side / 6);
			if (const auto origin = find_roof_rect(target, size, size, 2, reach)) {
				const int cx = origin->first + radius;
				const int cz = origin->second + radius;
				const int frame_height =
						static_cast<int>(std::lround((large ? 3 : 2) * scale)) +
						static_cast<int>(roof_rng.uniform(2));
				const int barrel_height =
						static_cast<int>(std::lround((large ? 4 : 3) * scale));
				const int tank_base = roof_y + 1;
				const auto inside = [](int dx, int dz, int r) {
					return dx * dx + dz * dz <= r * r + r;
				};
				const auto rim = [&](int dx, int dz, int r) {
					return inside(dx, dz, r) &&
						   (!inside(dx + 1, dz, r) || !inside(dx - 1, dz, r) ||
								   !inside(dx, dz + 1, r) || !inside(dx, dz - 1, r));
				};
				std::vector<std::pair<int, int>> legs = {
						{radius, 0}, {-radius, 0}, {0, radius}, {0, -radius}};
				if (radius == 1)
					legs = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
				else if (radius >= 3) {
					const int k = static_cast<int>(std::lround(radius / std::sqrt(2.0)));
					legs.insert(legs.end(), {{k, k}, {-k, k}, {k, -k}, {-k, -k}});
				}
				for (const auto &[dx, dz] : legs)
					for (int y = tank_base; y < tank_base + frame_height; ++y)
						force_rooftop_block(DARK_OAK_FENCE, cx + dx, y, cz + dz);
				const int tub = tank_base + frame_height;
				for (int dz = -radius; dz <= radius; ++dz)
					for (int dx = -radius; dx <= radius; ++dx) {
						if (rim(dx, dz, radius)) {
							for (int y = tub; y < tub + barrel_height; ++y)
								editor.set_block_with_properties_absolute(
										cached_prop_block(BARREL,
												{{"facing", "up"}, {"open", "false"}}),
										cx + dx, y, cz + dz, std::nullopt,
										std::optional<std::vector<Block>>(
												std::vector<Block>{}));
						} else if (inside(dx, dz, radius)) {
							force_rooftop_block(SPRUCE_PLANKS, cx + dx, tub, cz + dz);
						}
					}
				const int cap_top = tub + barrel_height;
				for (int course = 0; course <= radius; ++course) {
					const int r = radius - course;
					for (int dz = -r; dz <= r; ++dz)
						for (int dx = -r; dx <= r; ++dx) {
							if (!inside(dx, dz, r))
								continue;
							const int x = cx + dx, y = cap_top + course, z = cz + dz;
							if (r == 0 ||
									(rim(dx, dz, r) && std::abs(dx) == std::abs(dz))) {
								editor.set_block_with_properties_absolute(
										cached_prop_block(
												SPRUCE_SLAB, {{"type", "bottom"}}),
										x, y, z, std::nullopt,
										std::optional<std::vector<Block>>(
												std::vector<Block>{}));
							} else if (!rim(dx, dz, r)) {
								force_rooftop_block(SPRUCE_PLANKS, x, y, z);
							} else {
								const char *facing =
										std::abs(dx) > std::abs(dz)
												? (dx > 0 ? "west" : "east")
												: (dz > 0 ? "north" : "south");
								editor.set_block_with_properties_absolute(
										cached_prop_block(
												SPRUCE_STAIRS, {{"facing", facing}}),
										x, y, z, std::nullopt,
										std::optional<std::vector<Block>>(
												std::vector<Block>{}));
							}
						}
				}
				reserve_rect(origin->first, origin->second, size, size);
			}
		}

		const bool industrial = category == BuildingCategory::Industrial ||
								category == BuildingCategory::Warehouse;
		const int skylights =
				industrial
						? count_roof_props(120.0, 8)
						: (usable_cells.size() >= 60 && roof_rng.random_bool(.4 * density)
										  ? (usable_cells.size() >= 200 &&
																	roof_rng.random_bool(
																			.5)
															? 2
															: 1)
										  : 0);
		const auto [skylight_w, skylight_d] =
				oriented(static_cast<int>(std::lround(4 * scale)),
						static_cast<int>(std::lround(3 * scale)));
		for (int i = 0; i < skylights; ++i) {
			const int along = (i % 2 == 0) ? (i / 2) * 6 : -(i / 2 + 1) * 6;
			const auto target = step(heart, axis, along);
			if (const auto origin =
							find_roof_rect(target, skylight_w, skylight_d, 2, 3)) {
				for (int z = origin->second; z < origin->second + skylight_d; ++z)
					for (int x = origin->first; x < origin->first + skylight_w; ++x)
						force_rooftop_block(GLASS, x, roof_y + 1, z);
				reserve_rect(origin->first, origin->second, skylight_w, skylight_d);
			}
		}

		const int hvac_count =
				count_roof_props(industrial ? 110.0 : 260.0, industrial ? 8 : 3);
		if (hvac_count > 0) {
			const Block body = roof_rng.uniform(3) == 0
									   ? IRON_BLOCK
									   : (roof_rng.uniform(2) == 0 ? LIGHT_GRAY_CONCRETE
																   : SMOOTH_STONE);
			const auto [width, depth] = oriented(static_cast<int>(std::lround(3 * scale)),
					static_cast<int>(std::lround(2 * scale)));
			const auto row_start = step(step(heart, across, side_offset), axis,
					-std::min(hvac_count, 4) * (width + 1) / 2);
			for (int i = 0; i < hvac_count; ++i) {
				const auto target = step(row_start, axis, i * (width + 1));
				if (const auto origin = find_roof_rect(target, width, depth, 2, 3)) {
					for (int z = origin->second; z < origin->second + depth; ++z)
						for (int x = origin->first; x < origin->first + width; ++x) {
							force_rooftop_block(body, x, roof_y + 1, z);
							if (roof_rng.random_bool(.25))
								force_rooftop_block(IRON_BARS, x, roof_y + 2, z);
						}
					reserve_rect(origin->first, origin->second, width, depth);
				}
			}
		}

		// Solar collectors follow the building's long axis, leaving the roof edge
		// and any already reserved plant clear.
		const double solar_share =
				europe ? (era == ArchEra::Contemporary
										 ? .45
										 : (era == ArchEra::PostWarPanel ||
																   era == ArchEra::Unknown
														   ? .18
														   : .05))
					   : (era == ArchEra::Contemporary && storeys <= 5 ? .2 : 0.0);
		if (usable_cells.size() >= 40 && !green_roof &&
				roof_rng.random_bool(solar_share)) {
			const int rows = std::clamp(static_cast<int>(usable_cells.size() / 60), 2, 6);
			const int len = std::clamp(long_side / 3, 3, 8);
			const auto [width, depth] = oriented(len, 1);
			for (int i = 0; i < rows; ++i) {
				const auto target =
						step(step(heart, across, -side_offset), across, -2 * i);
				if (const auto origin = find_roof_rect(target, width, depth, 2, 2)) {
					for (int z = origin->second; z < origin->second + depth; ++z)
						for (int x = origin->first; x < origin->first + width; ++x)
							force_rooftop_block(DAYLIGHT_DETECTOR, x, roof_y + 1, z);
					reserve_rect(origin->first, origin->second, width, depth);
				}
			}
		}

		// Small exhaust fans and packaged rooftop units form a separate row
		// from the larger condensers. Keep their block states explicit so the
		// library backend preserves the same facing and open/closed details.
		const int small_unit_count = count_roof_props(north_america ? 90.0 : 160.0, 6);
		if (small_unit_count > 0) {
			const auto row_start =
					step(step(heart, across, -side_offset), axis, small_unit_count);
			const std::array<const char *, 4> facings = {
					"north", "south", "east", "west"};
			for (int i = 0; i < small_unit_count; ++i) {
				const auto target = step(row_start, {-axis.first, -axis.second}, i * 2);
				const auto origin = find_roof_rect(target, 1, 1, 2, 3);
				if (!origin)
					continue;
				const int x = origin->first;
				const int z = origin->second;
				const auto place_props =
						[&](Block block, std::initializer_list<PropertyPair> props,
								int y) {
							editor.set_block_with_properties_absolute(
									cached_prop_block(block, props), x, y, z,
									std::nullopt,
									std::optional<std::vector<Block>>(
											std::vector<Block>{}));
						};
				const char *facing = facings[roof_rng.uniform(4)];
				switch (roof_rng.uniform(6)) {
				case 0:
					place_props(BLAST_FURNACE, {{"facing", facing}}, roof_y + 1);
					break;
				case 1:
					place_props(SMOKER, {{"facing", facing}}, roof_y + 1);
					break;
				case 2:
					place_props(GRINDSTONE, {{"face", "floor"}, {"facing", facing}},
							roof_y + 1);
					break;
				case 3:
					place_props(
							BARREL, {{"facing", "up"}, {"open", "false"}}, roof_y + 1);
					break;
				case 4:
					force_rooftop_block(IRON_BLOCK, x, roof_y + 1, z);
					place_props(IRON_TRAPDOOR,
							{{"facing", facing}, {"half", "bottom"}, {"open", "false"}},
							roof_y + 2);
					break;
				default:
					force_rooftop_block(LIGHT_GRAY_CONCRETE, x, roof_y + 1, z);
					force_rooftop_block(DAYLIGHT_DETECTOR, x, roof_y + 2, z);
					break;
				}
				reserve_rect(x, z, 1, 1);
			}
		}

		const int vent_count = std::max(count_roof_props(north_america ? 35.0 : 60.0, 12),
				static_cast<int>(usable_cells.size() >= 8));
		for (int i = 0; i < vent_count; ++i) {
			std::pair<int, int> target;
			if (i < (vent_count + 1) / 2) {
				target = step(step(heart, across, -side_offset - 2), axis,
						i * 3 - ((vent_count + 1) / 2) * 3 / 2);
			} else {
				const auto &cell = usable_cells[static_cast<std::size_t>(roof_rng.uniform(
						static_cast<std::uint32_t>(usable_cells.size())))];
				target = cell;
			}
			if (const auto vent = find_roof_rect(target, 1, 1, 2, 3)) {
				const int roll = static_cast<int>(roof_rng.uniform(europe ? 3 : 6));
				if (roll == 0) {
					const std::array<Block, 3> stacks = {
							ANDESITE_WALL, STONE_BRICK_WALL, COBBLESTONE_WALL};
					const Block stack = stacks[roof_rng.uniform(3)];
					const int height = 1 + static_cast<int>(roof_rng.uniform(2));
					for (int dy = 0; dy < height; ++dy)
						force_rooftop_block(
								stack, vent->first, roof_y + 1 + dy, vent->second);
				} else if (roll == 1) {
					force_rooftop_block(
							LIGHTNING_ROD, vent->first, roof_y + 1, vent->second);
				} else if (roll == 2) {
					force_rooftop_block(END_ROD, vent->first, roof_y + 1, vent->second);
				} else if (roll == 3) {
					force_rooftop_block(IRON_BARS, vent->first, roof_y + 1, vent->second);
					force_rooftop_block(
							SMOOTH_STONE_SLAB, vent->first, roof_y + 2, vent->second);
				} else if (roll == 4) {
					force_rooftop_block(CAULDRON, vent->first, roof_y + 1, vent->second);
				} else {
					force_rooftop_block(
							STONE_PRESSURE_PLATE, vent->first, roof_y + 1, vent->second);
				}
				reserve_rect(vent->first, vent->second, 1, 1);
			}
		}

		const bool residential = category == BuildingCategory::House ||
								 category == BuildingCategory::Residential;
		const bool mast = building_height >= static_cast<int>(std::lround(40 * scale))
								  ? roof_rng.random_bool(.4)
								  : (europe && residential && roof_rng.random_bool(.3));
		if (mast) {
			int x = heart.first, z = heart.second, y = roof_y + 1;
			if (bulkhead_top) {
				x = std::get<0>(*bulkhead_top);
				z = std::get<1>(*bulkhead_top);
				y = roof_y + std::get<2>(*bulkhead_top) + 1;
			} else if (const auto mast_cell = find_roof_rect(heart, 1, 1, 2, reach)) {
				x = mast_cell->first;
				z = mast_cell->second;
				reserve_rect(x, z, 1, 1);
			} else {
				x = std::numeric_limits<int>::min();
			}
			if (x != std::numeric_limits<int>::min()) {
				const int mast_height = std::max(
						1, static_cast<int>(std::lround(
								   (building_height >= static_cast<int>(40 * scale) ? 5
																					: 3) *
								   scale)));
				for (int dy = 0; dy < mast_height; ++dy)
					force_rooftop_block(IRON_BARS, x, y + dy, z);
				force_rooftop_block(LIGHTNING_ROD, x, y + mast_height, z);
			}
		}

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
						editor.set_block_absolute(DAYLIGHT_DETECTOR, x, equip_y, z,
								std::nullopt, replace_any);
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
				editor.set_block_absolute(
						BARREL, x, equip_y, z, std::nullopt, replace_any);
				editor.set_block_absolute(
						CAULDRON, x, equip_y + 1, z, std::nullopt, replace_any);
				used.insert({x, z});
			} else if (roll <= 10) {
				const int stack_height = 2 + static_cast<int>(rng.uniform(2));
				for (int dy = 0; dy < stack_height; ++dy)
					editor.set_block_absolute(COBBLESTONE_WALL, x, equip_y + dy, z,
							std::nullopt, replace_any);
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
						editor.set_block_absolute(STONE_BRICKS, px, equip_y + 1, pz,
								std::nullopt, replace_any);
						editor.set_block_absolute(STONE_BRICK_SLAB, px, equip_y + 2, pz,
								std::nullopt, replace_any);
						used.insert({px, pz});
					}
			}
		}
	}
}
Block parapet_coping(Block wall, Block accent, std::uint64_t seed);
RoofDistanceGrid roof_edge_distances(const std::vector<std::pair<int, int>> &roof_area);

// Rust adds a deterministic cap/edge treatment to plain flat roofs.
void generate_flat_roof_edge_variation(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &roof_area,
		const building_facade::FacadePlan &facade, int start_y, int building_height,
		int abs_offset, Block wall_block, Block accent_block, BuildingCategory category,
		std::uint64_t style_seed)
{
	if (roof_area.empty() || category == BuildingCategory::Shed ||
			category == BuildingCategory::Greenhouse)
		return;
	auto rng = element_rng(style_seed ^ 0xED6EC0B100000001ULL);
	if (!rng.random_bool(.6))
		return;
	const int curb_y = start_y + building_height + abs_offset + 2;
	const auto curb =
			rooftops::bottom_slab(parapet_coping(wall_block, accent_block, style_seed));
	const std::unordered_set<std::pair<int, int>, PairHash> footprint(
			roof_area.begin(), roof_area.end());
	for (const auto &[x, z] : roof_area) {
		const bool edge =
				!footprint.contains({x - 1, z}) || !footprint.contains({x + 1, z}) ||
				!footprint.contains({x, z - 1}) || !footprint.contains({x, z + 1});
		if (edge && !facade.is_party(x, z))
			editor.set_block_with_properties_absolute(
					curb, x, curb_y, z, std::vector<Block>{AIR}, std::nullopt);
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

std::uint64_t rooftop_hash2(std::uint64_t seed, int x, int z)
{
	std::uint64_t h = seed ^
					  (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) *
							  0x9E3779B97F4A7C15ULL) ^
					  (static_cast<std::uint64_t>(static_cast<std::uint32_t>(z)) *
							  0xC2B2AE3D27D4EB4FULL);
	h ^= h >> 30;
	h *= 0xBF58476D1CE4E5B9ULL;
	h ^= h >> 27;
	h *= 0x94D049BB133111EBULL;
	return h ^ (h >> 31);
}

Block parapet_coping(Block wall, Block accent, std::uint64_t seed)
{
	const auto pick = [seed](std::initializer_list<Block> blocks) {
		const auto index =
				rooftop_hash2(seed ^ 0xC0B1460000000001ULL, 0, 0) % blocks.size();
		return *(blocks.begin() + static_cast<std::ptrdiff_t>(index));
	};
	if (wall == BRICK || wall == MUD_BRICKS || wall == TERRACOTTA ||
			wall == RED_TERRACOTTA || wall == ORANGE_TERRACOTTA ||
			wall == BROWN_TERRACOTTA || wall == GRANITE || wall == POLISHED_GRANITE ||
			wall == BROWN_CONCRETE || wall == NETHER_BRICK || wall == RED_NETHER_BRICKS)
		return pick({SMOOTH_STONE_SLAB, SMOOTH_STONE_SLAB, STONE_BRICK_SLAB,
				QUARTZ_SLAB_TOP, CUT_SANDSTONE_SLAB});
	if (wall == QUARTZ_BLOCK || wall == QUARTZ_BRICKS || wall == SMOOTH_QUARTZ ||
			wall == WHITE_CONCRETE || wall == WHITE_TERRACOTTA || wall == CALCITE ||
			wall == DIORITE || wall == POLISHED_DIORITE)
		return pick({SMOOTH_QUARTZ_SLAB, POLISHED_DIORITE_SLAB, QUARTZ_SLAB_TOP});
	if (wall == SANDSTONE || wall == SMOOTH_SANDSTONE || wall == END_STONE_BRICKS ||
			wall == YELLOW_TERRACOTTA)
		return pick({CUT_SANDSTONE_SLAB, SMOOTH_STONE_SLAB});
	if (wall == STONE_BRICKS || wall == STONE || wall == ANDESITE ||
			wall == POLISHED_ANDESITE || wall == SMOOTH_STONE ||
			wall == LIGHT_GRAY_CONCRETE || wall == COBBLESTONE || wall == TUFF ||
			wall == LIGHT_GRAY_TERRACOTTA)
		return pick({SMOOTH_STONE_SLAB, STONE_BRICK_SLAB, POLISHED_ANDESITE_SLAB});
	if (wall == BLACKSTONE || wall == POLISHED_BLACKSTONE ||
			wall == POLISHED_BLACKSTONE_BRICKS || wall == DEEPSLATE ||
			wall == DEEPSLATE_BRICKS || wall == POLISHED_DEEPSLATE ||
			wall == GRAY_CONCRETE || wall == BLACK_CONCRETE || wall == GRAY_TERRACOTTA ||
			wall == CYAN_TERRACOTTA || wall == BLUE_TERRACOTTA)
		return pick({POLISHED_BLACKSTONE_SLAB, SMOOTH_STONE_SLAB, DEEPSLATE_TILE_SLAB});
	if (is_glass_wall_block(wall) && is_glass_wall_block(accent))
		return SMOOTH_STONE_SLAB;
	return is_glass_wall_block(wall) ? get_slab_block_for_material(accent)
									 : get_slab_block_for_material(wall);
}

bool neighbour_reaches_roof(
		WorldEditor &editor, const RoofDistanceGrid &dist, int x, int z, int surface_y)
{
	for (const auto &[dx, dz] :
			{std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}}) {
		if (dist.contains({x + dx, z + dz}))
			continue;
		for (int k = 1; k <= 2; ++k)
			if (editor.highest_block_between(
						x + dx * k, z + dz * k, surface_y, surface_y + 1))
				return true;
	}
	return false;
}

void generate_parapet(WorldEditor &editor, const ProcessedWay &element,
		const std::vector<std::pair<int, int>> &roof_area,
		const building_facade::FacadePlan &facade, int start_y, int building_height,
		int abs_offset, Block wall_block, Block accent_block, BuildingCategory category,
		bool railing = false, std::optional<Block> coping_override = std::nullopt)
{
	if (roof_area.empty())
		return;
	const RoofDistanceGrid dist = roof_edge_distances(roof_area);
	const int surface_y = start_y + building_height + abs_offset + 1;
	const int course_y = surface_y + 1;
	const bool modern = category == BuildingCategory::ModernSkyscraper;
	const Block coping = modern ? get_slab_block_for_material(accent_block)
								: coping_override.value_or(parapet_coping(
										  wall_block, accent_block, element.id));
	const BlockWithProperties cap = rooftops::bottom_slab(coping);
	for (const auto &[x, z] : roof_area) {
		auto edge = dist.find({x, z});
		if (edge == dist.end() || edge->second != 0)
			continue;
		const bool party = facade.is_party(x, z);
		if (party && neighbour_reaches_roof(editor, dist, x, z, surface_y))
			continue;
		editor.set_block_absolute(wall_block, x, course_y, z, std::vector<Block>{AIR});
		if (party)
			continue;
		if (railing)
			connected_blocks::place_connected(editor, IRON_BARS, x, course_y + 1, z);
		else
			editor.set_block_with_properties_absolute(
					cap, x, course_y + 1, z, std::vector<Block>{AIR}, std::nullopt);
	}
	if (category == BuildingCategory::ModernSkyscraper) {
		std::unordered_set<std::pair<int, int>, PairHash> corners;
		for (const auto &node : element.nodes)
			if (const auto it = dist.find({node.x, node.z});
					it != dist.end() && it->second == 0 &&
					!facade.is_party(node.x, node.z))
				corners.insert({node.x, node.z});
		for (const auto &[x, z] : corners)
			editor.set_block_absolute(
					accent_block, x, course_y + 1, z, std::nullopt, std::vector<Block>{});
	}
}

// Rust's gabled profile extends only its two slope-side eaves, using stairs.
void place_gabled_eave_overhang(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &area, int base_y, int offset,
		Block roof_block, bool ridge_runs_along_x,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_cells = nullptr,
		bool short_eave = false,
		const std::unordered_set<std::pair<int, int>, PairHash> *blocked = nullptr,
		bool soffit = false, bool slab_pitch = false)
{
	std::unordered_set<std::pair<int, int>, PairHash> footprint(area.begin(), area.end());
	auto open = [&](const std::pair<int, int> &cell) {
		return !footprint.contains(cell) && (!blocked || !blocked->contains(cell));
	};
	const int stair_y = base_y + offset;
	const Block stair_material = get_stair_block_for_material(roof_block);
	const auto slab = roof_slab_for_stair(stair_material);
	const int outer_y = stair_y - (slab_pitch && slab ? 0 : 1);
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
			if (!open(n1))
				continue;
			inner.try_emplace(n1, facing);
			if (!short_eave && open(n2))
				outer.try_emplace(n2, facing);
		}
	}
	for (const auto &[cell, facing] : inner) {
		if (slab_pitch && slab) {
			editor.set_block_with_properties_absolute(
					cached_prop_block(*slab, {{"type", "top"}}), cell.first, stair_y,
					cell.second, nullptr, nullptr);
			continue;
		}
		const auto stair = create_stair_with_properties(
				stair_material, facing, StairShape::Straight);
		if (soffit)
			editor.set_block_with_properties_absolute(top_stair(stair), cell.first,
					stair_y - 1, cell.second, nullptr, nullptr);
		editor.set_block_with_properties_absolute(
				stair, cell.first, stair_y, cell.second, nullptr, nullptr);
	}
	for (const auto &[cell, facing] : outer)
		if (!inner.contains(cell)) {
			if (slab_pitch && slab) {
				editor.set_block_with_properties_absolute(
						cached_prop_block(*slab, {{"type", "bottom"}}), cell.first,
						outer_y, cell.second, nullptr, nullptr);
				continue;
			}
			editor.set_block_with_properties_absolute(
					create_stair_with_properties(
							stair_material, facing, StairShape::Straight),
					cell.first, outer_y, cell.second, nullptr, nullptr);
		}
}

void place_hipped_eave_overhang(WorldEditor &editor,
		const std::vector<std::pair<int32_t, int32_t>> &area, int base_height,
		int abs_offset, Block stair_material, bool short_eave,
		const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash> *blocked =
				nullptr,
		bool soffit = false, bool slab_pitch = false)
{
	using Cell = std::pair<int32_t, int32_t>;
	std::unordered_set<Cell, pair_hash> footprint(area.begin(), area.end());
	auto open = [&](const Cell &cell) {
		return !footprint.contains(cell) && (!blocked || !blocked->contains(cell));
	};
	const int y_inner = base_height + abs_offset;
	const auto slab = roof_slab_for_stair(stair_material);
	const int y_outer = y_inner - (slab_pitch && slab ? 0 : 1);
	constexpr std::array<Cell, 4> sides{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
	constexpr std::array<Cell, 4> quarters{{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}}};
	std::unordered_map<Cell, std::uint8_t, pair_hash> inner, outer, arms;
	std::unordered_map<Cell, std::size_t, pair_hash> inner_corners, outer_corners;
	for (const auto &[x, z] : area) {
		std::uint8_t falling_sides = 0;
		for (std::size_t i = 0; i < sides.size(); ++i) {
			const auto [dx, dz] = sides[i];
			if (!footprint.contains({x + dx, z + dz}))
				falling_sides |= std::uint8_t(1U << i);
		}
		for (std::size_t i = 0; i < sides.size(); ++i) {
			if ((falling_sides & (1U << i)) == 0)
				continue;
			const auto [dx, dz] = sides[i];
			const Cell near_{x + dx, z + dz};
			const Cell far_{x + 2 * dx, z + 2 * dz};
			if (!open(near_))
				continue;
			inner[near_] |= std::uint8_t(1U << i);
			if (!short_eave && open(far_))
				outer[far_] |= std::uint8_t(1U << i);
		}
		constexpr std::array<std::pair<std::size_t, std::size_t>, 4> corner_sides{
				{{0, 2}, {1, 2}, {0, 3}, {1, 3}}};
		for (std::size_t q = 0; q < corner_sides.size(); ++q) {
			const auto [sx, sz] = corner_sides[q];
			if ((falling_sides & (1U << sx)) == 0 || (falling_sides & (1U << sz)) == 0)
				continue;
			const auto [dx, dz] = quarters[q];
			if (!open({x + dx, z}) || !open({x, z + dz}) || !open({x + dx, z + dz}))
				continue;
			inner_corners.try_emplace({x + dx, z + dz}, q);
			if (short_eave)
				continue;
			if (open({x + 2 * dx, z + 2 * dz}))
				outer_corners.try_emplace({x + 2 * dx, z + 2 * dz}, q);
			const auto [ix, iz] = corner_sides[q];
			if (open({x + 2 * dx, z + dz}))
				arms[{x + 2 * dx, z + dz}] |= std::uint8_t(1U << ix);
			if (open({x + dx, z + 2 * dz}))
				arms[{x + dx, z + 2 * dz}] |= std::uint8_t(1U << iz);
		}
	}
	auto eave_stair = [&](std::uint8_t bits) {
		std::array<bool, 4> high{};
		for (std::size_t q = 0; q < quarters.size(); ++q) {
			const auto [sx, sz] = quarters[q];
			for (std::size_t i = 0; i < sides.size(); ++i) {
				const auto [dx, dz] = sides[i];
				if ((bits & (1U << i)) != 0 &&
						((dx != 0 && sx == -dx) || (dz != 0 && sz == -dz))) {
					high[q] = true;
					break;
				}
			}
		}
		auto stair = roof_stair_for_high_quarters(high);
		if (!stair) {
			for (std::size_t i = 0; i < sides.size(); ++i) {
				if ((bits & (1U << i)) == 0)
					continue;
				high = {};
				for (std::size_t q = 0; q < quarters.size(); ++q) {
					const auto [sx, sz] = quarters[q];
					const auto [dx, dz] = sides[i];
					high[q] = (dx != 0 && sx == -dx) || (dz != 0 && sz == -dz);
				}
				stair = roof_stair_for_high_quarters(high);
				break;
			}
		}
		return create_stair_with_properties(stair_material,
				stair ? stair->first : StairFacing::North,
				stair ? stair->second : StairShape::Straight);
	};
	auto corner_stair = [&](std::size_t q) {
		std::array<bool, 4> high{};
		high[3 - q] = true;
		auto stair = roof_stair_for_high_quarters(high);
		return create_stair_with_properties(stair_material,
				stair ? stair->first : StairFacing::North,
				stair ? stair->second : StairShape::Straight);
	};
	auto put = [&](const Cell &cell, int y, StairFacing facing, StairShape shape,
					   bool upper) {
		if (slab_pitch && slab) {
			editor.set_block_with_properties_absolute(
					cached_prop_block(*slab, {{"type", upper ? "top" : "bottom"}}),
					cell.first, y, cell.second, nullptr, nullptr);
			return;
		}
		editor.set_block_with_properties_absolute(
				create_stair_with_properties(stair_material, facing, shape), cell.first,
				y, cell.second, nullptr, nullptr);
	};
	for (const auto &[cell, bits] : inner) {
		if (slab_pitch && slab) {
			put(cell, y_inner, StairFacing::North, StairShape::Straight, true);
			continue;
		}
		const auto stair = eave_stair(bits);
		if (soffit)
			editor.set_block_with_properties_absolute(top_stair(stair), cell.first,
					y_inner - 1, cell.second, nullptr, nullptr);
		editor.set_block_with_properties_absolute(
				stair, cell.first, y_inner, cell.second, nullptr, nullptr);
	}
	for (const auto &[cell, q] : inner_corners) {
		if (inner.contains(cell))
			continue;
		if (slab_pitch && slab) {
			put(cell, y_inner, StairFacing::North, StairShape::Straight, true);
			continue;
		}
		const auto stair = corner_stair(q);
		editor.set_block_with_properties_absolute(
				stair, cell.first, y_inner, cell.second, nullptr, nullptr);
	}
	for (const auto &[cell, bits] : outer)
		if (!inner.contains(cell) && !inner_corners.contains(cell)) {
			if (slab_pitch && slab) {
				put(cell, y_outer, StairFacing::North, StairShape::Straight, false);
				continue;
			}
			const auto stair = eave_stair(bits);
			editor.set_block_with_properties_absolute(
					stair, cell.first, y_outer, cell.second, nullptr, nullptr);
		}
	for (const auto &[cell, q] : outer_corners)
		if (!inner.contains(cell) && !outer.contains(cell) &&
				!inner_corners.contains(cell)) {
			if (slab_pitch && slab) {
				put(cell, y_outer, StairFacing::North, StairShape::Straight, false);
				continue;
			}
			const auto stair = corner_stair(q);
			editor.set_block_with_properties_absolute(
					stair, cell.first, y_outer, cell.second, nullptr, nullptr);
		}
	for (const auto &[cell, bits] : arms)
		if (!inner.contains(cell) && !outer.contains(cell) &&
				!inner_corners.contains(cell) && !outer_corners.contains(cell)) {
			if (slab_pitch && slab) {
				put(cell, y_outer, StairFacing::North, StairShape::Straight, false);
				continue;
			}
			const auto stair = eave_stair(bits);
			editor.set_block_with_properties_absolute(
					stair, cell.first, y_outer, cell.second, nullptr, nullptr);
		}
}

void generate_gabled_roof_profile(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &area,
		const std::vector<ProcessedNode> &nodes, const tags_t &tags, int base_height,
		int abs_terrain_offset, int building_height, RoofType roof_type, bool steep_gable,
		std::optional<bool> preferred_ridge_along_x, std::optional<int> peak_cap,
		std::uint64_t style_seed, bool add_dormers, Block roof_block, bool short_eave,
		double scale_factor, const std::vector<HolePolygon> *holes,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_blocked,
		bool steep_cap, bool tone_patches, std::uint64_t tone_seed, bool sheet_top,
		bool soffit, std::optional<Block> gable_block)
{
	if (area.empty())
		return;
	const bool gambrel = roof_type == RoofType::Gambrel;
	const bool half_hipped = roof_type == RoofType::HalfHipped;
	const bool round = roof_type == RoofType::Round;
	const auto tent = roof_tent_frame(nodes);
	const std::unordered_set<std::pair<int, int>, PairHash> footprint(
			area.begin(), area.end());
	const auto runs = axis_runs(area);
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
	const bool force_main_ridge =
			preferred_ridge_along_x.has_value() || mapped_direction ||
			(tags.contains("roof:orientation") &&
					(tags.at("roof:orientation") == "along" ||
							tags.at("roof:orientation") == "across"));
	std::optional<RoofWingFrame> wings;
	if (!tent && !round && roof_type != RoofType::Flat)
		wings = roof_wings(nodes, area.size(),
				force_main_ridge ? std::optional<bool>{ridge_runs_along_x} : std::nullopt,
				holes);
	const double cap_factor = steep_gable && roof_type == RoofType::Gabled ? .9 : .6;
	const int inferred_cap =
			std::min(std::max(1, int(std::lround(building_height * cap_factor))),
					rust_max_inferred_roof_rise(steep_gable, scale_factor));
	const int wall_cap = peak_cap.value_or(inferred_cap);
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
		bool wing_along_u{};
	};
	std::unordered_map<std::pair<int, int>, CellData, PairHash> cell_data;
	std::unordered_map<std::pair<int, int>, int, PairHash> roof_heights;
	cell_data.reserve(area.size() * 2);
	roof_heights.reserve(area.size() * 2);
	for (std::size_t area_i = 0; area_i < area.size(); ++area_i) {
		const auto [x, z] = area[area_i];
		const auto [sm_x, sp_x, sm_z, sp_z] = runs[area_i];
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
		cell_data[{x, z}] = {std::min(dm_perp, dp_perp), local_half,
				(dm_perp + dp_perp) / 2, dm_along, dp_along, scan_perp_min, dm_perp,
				dp_perp, false};
	}
	int max_reach = 0;
	for (const auto &[cell, data] : cell_data)
		max_reach = std::max(max_reach, std::min(data.perp_reach, data.local_half));
	if (wings)
		for (const auto &wing : wings->wings)
			max_reach = std::max(max_reach, wing.reach());
	const bool shallow_pitch =
			peak_cap ? static_cast<double>(*peak_cap) < 0.75 * std::max(1, max_reach)
					 : max_reach > wall_cap + 1 && !steep_cap;
	const auto profile_stair_material = get_stair_block_for_material(roof_block);
	const auto profile_slab = roof_slab_for_stair(profile_stair_material);
	const bool slab_pitch = profile_slab.has_value() && shallow_pitch && !gambrel &&
							!round && !half_hipped;
	const bool tread_pitch = !slab_pitch && !peak_cap && !gambrel && !round &&
							 !steep_cap && max_reach - wall_cap >= 4;
	RoofHeightMap roof_levels;
	const std::optional<int> stated_pitch = gambrel ? std::nullopt : peak_cap;
	const int eave_height = base_height + 1;
	for (const auto &[x, z] : area) {
		const auto &cell = cell_data.at({x, z});
		const int edge = std::min(cell.dist_to_edge, cell.scan_perp_min);
		int height = eave_height;
		if (round) {
			const int cap = peak_cap.value_or(std::min(
					wall_cap, std::max(1, int(std::lround(cell.local_half * .85)))));
			const double reach = std::max(1, cell.perp_reach);
			const double t = std::clamp(1.0 - cell.dist_to_edge / reach, 0.0, 1.0);
			height += int(std::lround(cap * std::sqrt(1.0 - t * t)));
		} else if (stated_pitch) {
			const double pitch = double(*stated_pitch) / std::max(1, cell.perp_reach);
			const int local = std::clamp(int(std::lround(cell.local_half * pitch)), 1,
					std::max(1, *stated_pitch));
			if (slab_pitch) {
				const int rise =
						std::min(int(std::lround(2.0 * edge * pitch)), 2 * local);
				height = eave_height + rise / 2;
			} else {
				const int rise =
						pitch < 1.0 ? 2 * std::min(edge, local)
									: 2 * std::min(int(std::lround(edge * pitch)), local);
				height += rise / 2;
			}
		} else if (slab_pitch) {
			const int rise = std::min({edge, std::max(1, cell.local_half), 2 * wall_cap});
			height = eave_height + rise / 2;
		} else {
			const int slope = tread_pitch ? (edge + 1) / 2 : edge;
			const int boost =
					gambrel ? 2 * std::min(slope, 2) + std::max(0, slope - 2) : slope;
			const int rise =
					2 * std::min({boost, std::max(1, cell.local_half), wall_cap});
			height += rise / 2;
		}
		if (half_hipped) {
			const int hip_start = std::max(wall_cap / 2, 2);
			height = std::min(height,
					base_height + hip_start + std::min(cell.dm_along, cell.dp_along) + 1);
		}
		roof_heights[{x, z}] = height;
	}
	std::unordered_map<std::pair<int, int>, std::size_t, PairHash> wing_owner;
	if (wings) {
		int max_wing_reach = 1;
		for (const auto &wing : wings->wings)
			max_wing_reach = std::max(max_wing_reach, wing.reach());
		for (const auto &[x, z] : area) {
			const auto [u, v] = wings->uv(x, z);
			int best_level = std::numeric_limits<int>::min();
			std::size_t best_index = 0;
			CellData best_cell{};
			for (std::size_t i = 0; i < wings->wings.size(); ++i) {
				const auto &wing = wings->wings[i];
				const double a = wing.along_u ? u : v;
				const double b = wing.along_u ? v : u;
				const auto [a0, a1] = wing.along();
				const auto [b0, b1] = wing.across();
				const double lo = wing.clipped.first ? a0 - 1e-6 : a0 - 1.0;
				const double hi = wing.clipped.second ? a1 + 1e-6 : a1 + 1.0;
				const double edge = std::min(b - b0, b1 - b);
				if (a < lo || a > hi || edge < -1.0)
					continue;
				const auto whole = [](double d) {
					return std::max(0, static_cast<int>(std::floor(d + 1e-6)));
				};
				const int reach = std::max(1, wing.reach());
				const int d_edge = whole(edge);
				const int d_along_minus = wing.clipped.first
												  ? std::numeric_limits<int>::max() / 4
												  : whole(a - a0);
				const int d_along_plus = wing.clipped.second
												 ? std::numeric_limits<int>::max() / 4
												 : whole(a1 - a);
				const int dm_perp = whole(b - b0);
				const int dp_perp = whole(b1 - b);
				const int e = std::min(d_edge, (dm_perp + dp_perp) / 2);
				int rise = 0;
				if (peak_cap && !gambrel) {
					const double pitch = static_cast<double>(*peak_cap) / max_wing_reach;
					const int local =
							std::clamp(static_cast<int>(std::lround(reach * pitch)), 1,
									std::max(1, *peak_cap));
					if (slab_pitch) {
						rise = static_cast<int>(std::lround(2.0 * e * pitch));
					} else if (pitch < 1.0) {
						rise = 2 * std::min(e, local);
					} else {
						rise = 2 * static_cast<int>(std::lround(e * pitch));
					}
					rise = std::min(rise, 2 * local);
				} else if (slab_pitch) {
					rise = std::min({e, reach, 2 * wall_cap});
				} else {
					const int slope_dist = tread_pitch ? (e + 1) / 2 : e;
					rise = gambrel ? 2 * std::min(slope_dist, 2) +
											 std::max(0, slope_dist - 2)
								   : slope_dist;
					rise = 2 * std::min(rise, std::min(reach, wall_cap));
				}
				if (half_hipped) {
					const int hip_start = std::max(wall_cap / 2, 2);
					rise = std::min(rise,
							2 * (hip_start + std::min(d_along_minus, d_along_plus)));
				}
				const int level = slab_pitch ? 2 * (base_height + 1) + rise
											 : 2 * (base_height + 1 + rise / 2) + 1;
				if (level < best_level || (level == best_level && i > best_index))
					continue;
				best_level = level;
				best_index = i;
				best_cell = {d_edge, reach, reach, d_along_minus, d_along_plus,
						std::numeric_limits<int>::max() / 4, dm_perp, dp_perp,
						wing.along_u};
			}
			// Rust lets a rasterized boundary sliver inherit the sample from its
			// nearest wing. Clamp the frame point to that wing, then evaluate the
			// same profile there instead of leaving this cell unowned.
			if (best_level == std::numeric_limits<int>::min() && !wings->wings.empty()) {
				auto gap_to_wing = [&, u = u, v = v](const RoofWing &wing) {
					const double du =
							std::max({wing.u.first - u, u - wing.u.second, 0.0});
					const double dv =
							std::max({wing.v.first - v, v - wing.v.second, 0.0});
					return std::max(du, dv);
				};
				for (std::size_t i = 1; i < wings->wings.size(); ++i)
					if (gap_to_wing(wings->wings[i]) <
							gap_to_wing(wings->wings[best_index]))
						best_index = i;
				const auto &wing = wings->wings[best_index];
				const double cu = std::clamp(u, wing.u.first, wing.u.second);
				const double cv = std::clamp(v, wing.v.first, wing.v.second);
				const double a = wing.along_u ? cu : cv;
				const double b = wing.along_u ? cv : cu;
				const auto [a0, a1] = wing.along();
				const auto [b0, b1] = wing.across();
				const auto whole = [](double d) {
					return std::max(0, static_cast<int>(std::floor(d + 1e-6)));
				};
				const int reach = std::max(1, wing.reach());
				const int d_edge = whole(std::min(b - b0, b1 - b));
				const int d_along_minus = wing.clipped.first
												  ? std::numeric_limits<int>::max() / 4
												  : whole(a - a0);
				const int d_along_plus = wing.clipped.second
												 ? std::numeric_limits<int>::max() / 4
												 : whole(a1 - a);
				const int dm_perp = whole(b - b0), dp_perp = whole(b1 - b);
				const int e = std::min(d_edge, (dm_perp + dp_perp) / 2);
				int rise;
				if (peak_cap && !gambrel) {
					const double pitch = static_cast<double>(*peak_cap) / max_wing_reach;
					const int local =
							std::clamp(static_cast<int>(std::lround(reach * pitch)), 1,
									std::max(1, *peak_cap));
					if (slab_pitch)
						rise = static_cast<int>(std::lround(2.0 * e * pitch));
					else if (pitch < 1.0)
						rise = 2 * std::min(e, local);
					else
						rise = 2 * static_cast<int>(std::lround(e * pitch));
					rise = std::min(rise, 2 * local);
				} else if (slab_pitch) {
					rise = std::min({e, reach, 2 * wall_cap});
				} else {
					const int slope_dist = tread_pitch ? (e + 1) / 2 : e;
					rise = gambrel ? 2 * std::min(slope_dist, 2) +
											 std::max(0, slope_dist - 2)
								   : slope_dist;
					rise = 2 * std::min(rise, std::min(reach, wall_cap));
				}
				if (half_hipped) {
					const int hip_start = std::max(wall_cap / 2, 2);
					rise = std::min(rise,
							2 * (hip_start + std::min(d_along_minus, d_along_plus)));
				}
				best_level = slab_pitch ? 2 * (base_height + 1) + rise
										: 2 * (base_height + 1 + rise / 2) + 1;
				best_cell = {d_edge, reach, reach, d_along_minus, d_along_plus,
						std::numeric_limits<int>::max() / 4, dm_perp, dp_perp,
						wing.along_u};
			}
			// Rust always emits a level for every footprint cell, even if a
			// degenerate wing set leaves no sample. Later slab-roof placement
			// iterates the full area and requires a level for every cell.
			const bool has_wing_sample = best_level != std::numeric_limits<int>::min();
			if (!has_wing_sample) {
				best_level = 2 * (base_height + 1) + (slab_pitch ? 0 : 1);
				best_index = 0;
			}
			roof_heights[{x, z}] = best_level / 2;
			if (slab_pitch)
				roof_levels[{x, z}] = best_level;
			if (has_wing_sample)
				cell_data[{x, z}] = best_cell;
			if (!wings->wings.empty())
				wing_owner[{x, z}] = best_index;
		}
	}
	if (slab_pitch && !wings) {
		roof_levels.reserve(area.size() * 2);
		const int eave = base_height + 1;
		for (const auto &[x, z] : area) {
			const auto &cell = cell_data.at({x, z});
			const int edge = std::min(cell.dist_to_edge, cell.scan_perp_min);
			int rise;
			if (peak_cap) {
				const double pitch =
						static_cast<double>(*peak_cap) / std::max(1, cell.perp_reach);
				const int local =
						std::clamp(static_cast<int>(std::lround(cell.local_half * pitch)),
								1, std::max(1, *peak_cap));
				rise = std::min(
						static_cast<int>(std::lround(2.0 * edge * pitch)), 2 * local);
			} else {
				rise = std::min({edge, std::max(1, cell.local_half), 2 * wall_cap});
			}
			roof_levels[{x, z}] = 2 * eave + rise;
			roof_heights[{x, z}] = roof_levels[{x, z}] / 2;
		}
	}
	const std::pair<int, int> along =
			ridge_runs_along_x ? std::pair{1, 0} : std::pair{0, 1};
	const std::pair<int, int> across =
			ridge_runs_along_x ? std::pair{0, 1} : std::pair{1, 0};
	const bool smooth = !tent || std::abs(tent->angle * 180.0 / std::numbers::pi) <= 12.0;
	if (smooth && !wings && slab_pitch) {
		auto smoothed = roof_levels;
		for (const auto &[cell, level] : roof_levels) {
			const auto [x, z] = cell;
			const auto before = roof_levels.find({x - along.first, z - along.second});
			const auto after = roof_levels.find({x + along.first, z + along.second});
			if (before != roof_levels.end() && after != roof_levels.end()) {
				std::array<int, 3> values{before->second, level, after->second};
				std::sort(values.begin(), values.end());
				smoothed[cell] = values[1];
			}
		}
		roof_levels.swap(smoothed);
		for (const auto &[cell, level] : roof_levels)
			roof_heights[cell] = level / 2;
	} else if (smooth && !wings) {
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
	const RoofCellSet sheet_cells =
			sheet_top ? (slab_pitch ? capped_flat_tops(roof_levels)
									: capped_flat_tops(roof_heights))
					  : RoofCellSet{};
	const Block stair_material = get_stair_block_for_material(roof_block);
	const auto base_slab = roof_slab_for_stair(stair_material);
	const auto tone = tone_patches ? roof_tone_sibling(roof_block) : std::nullopt;
	RoofPatchCache tone_patch_cache;
	RoofPatchCache sheet_patch_cache;
	const auto surface_block = [&](int x, int z) {
		return tone && cached_roof_tone_patch(x, z, tone_seed, tone_patch_cache)
					   ? tone->full
					   : roof_block;
	};
	const auto surface_stair_material = [&](int x, int z) {
		return tone && cached_roof_tone_patch(x, z, tone_seed, tone_patch_cache)
					   ? tone->stairs
					   : stair_material;
	};
	const auto surface_slab = [&](int x, int z) {
		return tone && cached_roof_tone_patch(x, z, tone_seed, tone_patch_cache)
					   ? tone->slab
					   : roof_slab_for_stair(stair_material);
	};
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
		StairFacing facing = nearer_minus ? toward_plus : toward_minus;
		if (wings) {
			const auto owner = wing_owner.find({x, z});
			if (owner != wing_owner.end()) {
				const auto &wing = wings->wings[owner->second];
				const double dx =
						wing.along_u ? -std::sin(wings->angle) : std::cos(wings->angle);
				const double dz =
						wing.along_u ? std::cos(wings->angle) : std::sin(wings->angle);
				facing = cardinal_for_vector(
						nearer_minus ? dx : -dx, nearer_minus ? dz : -dz);
			}
		}
		return create_stair_with_properties(
				surface_stair_material(x, z), facing, StairShape::Straight);
	};
	auto eave_cell = [&](int x, int z) {
		if (wings)
			return cell_data.at({x, z}).dist_to_edge == 0 &&
				   (!footprint.contains({x - 1, z}) || !footprint.contains({x + 1, z}) ||
						   !footprint.contains({x, z - 1}) ||
						   !footprint.contains({x, z + 1}));
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
	struct RakeCap
	{
		int y;
		BlockWithProperties block;
	};
	std::unordered_map<std::pair<int, int>, RakeCap, PairHash> rake_caps;
	const std::vector<Block> replace_any;
	const RoofLevelGrid surface_grid(slab_pitch ? roof_levels : roof_heights);
	const auto stored_level_at = [&](int nx, int nz) -> std::optional<int> {
		const auto stored = surface_grid.get(nx, nz);
		if (!stored)
			return std::nullopt;
		return slab_pitch ? *stored : 2 * *stored + 1;
	};
	for (const auto &[x, z] : area) {
		const int height = roof_heights.at({x, z});
		const auto &cell = cell_data.at({x, z});
		const bool edge =
				!footprint.contains({x - 1, z}) || !footprint.contains({x + 1, z}) ||
				!footprint.contains({x, z - 1}) || !footprint.contains({x, z + 1});
		const std::vector<Block> *replacements = edge ? &replace_any : nullptr;
		auto put_full = [&, x = x, z = z](const Block &block, int y) {
			editor.set_block_absolute(
					block, x, y + abs_terrain_offset, z, nullptr, replacements);
		};
		auto put_state = [&, x = x, z = z](const BlockWithProperties &block, int y) {
			editor.set_block_with_properties_absolute(
					block, x, y + abs_terrain_offset, z, nullptr, replacements);
		};
		const Block cell_stair_material = surface_stair_material(x, z);
		const Block cell_roof_block = surface_block(x, z);
		if (eave_cell(x, z) && !slab_pitch) {
			put_full(roof_block, base_height);
			const auto stair_cap = slope_stair(x, z);
			put_state(stair_cap, base_height + 1);
			rake_caps.insert_or_assign(
					std::pair{x, z}, RakeCap{base_height + 1, stair_cap});
			continue;
		}
		const int roof_level = slab_pitch ? roof_levels.at({x, z}) : 2 * height + 1;
		const auto level_at = [&, x = x, z = z](int nx, int nz) -> std::optional<int> {
			if (const auto stored = stored_level_at(nx, nz))
				return stored;
			// Continue the roof plane beyond a gable end, but not beyond an eave.
			if (!wings && !tent &&
					((ridge_runs_along_x && nz == z && nx != x) ||
							(!ridge_runs_along_x && nx == x && nz != z)))
				return roof_level;
			return std::nullopt;
		};
		constexpr std::array<std::pair<int, int>, 4> cap_sides{
				{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
		bool steep_drop = false;
		for (const auto &[dx, dz] : cap_sides) {
			const auto neighbor_level = stored_level_at(x + dx, z + dz);
			steep_drop |= neighbor_level && *neighbor_level < roof_level &&
						  roof_level - *neighbor_level > 4;
		}
		constexpr std::array<std::pair<int, int>, 4> cap_quarters{
				{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}}};
		std::array<bool, 4> high{};
		const int cap_floor = slab_pitch ? roof_level - 1 : roof_level;
		for (std::size_t i = 0; i < cap_quarters.size(); ++i) {
			const auto [dx, dz] = cap_quarters[i];
			const auto not_lower = [&](int nx, int nz) {
				const auto level = level_at(nx, nz);
				return level && *level >= cap_floor;
			};
			high[i] = not_lower(x + dx, z) && not_lower(x, z + dz) &&
					  not_lower(x + dx, z + dz);
		}
		const bool all_high =
				std::all_of(high.begin(), high.end(), [](bool v) { return v; });
		const bool all_low =
				std::none_of(high.begin(), high.end(), [](bool v) { return v; });
		const auto cap_stair = roof_stair_for_high_quarters(high);
		bool stepped = false;
		for (const auto &[dx, dz] : cap_sides) {
			const auto level = level_at(x + dx, z + dz);
			stepped |= level && *level < roof_level;
		}
		const auto slab = surface_slab(x, z);
		for (int y = base_height; y <= height; ++y) {
			const bool gable_fill =
					gable_block && (cell.dm_along == 0 || cell.dp_along == 0);
			if (y < height && gable_fill) {
				put_full(*gable_block, y);
				continue;
			}
			if (y == height) {
				if (roof_level % 2 == 0 && base_slab && slab) {
					const auto cap = cached_prop_block(*slab, {{"type", "bottom"}});
					put_state(cap, y);
					rake_caps.insert_or_assign({x, z}, RakeCap{y, cap});
				} else if (steep_drop) {
					put_full(cell_roof_block, y);
				} else if (all_high) {
					if (slab_pitch && stepped && base_slab && slab) {
						const auto cap = cached_prop_block(*slab, {{"type", "top"}});
						put_state(cap, y);
						rake_caps.insert_or_assign({x, z}, RakeCap{y, cap});
					} else {
						put_full(sheet_cells.contains({x, z})
										 ? roof_sheet_block(x, z, sheet_patch_cache)
										 : cell_roof_block,
								y);
					}
				} else if (cap_stair) {
					const auto surface_stair = create_stair_with_properties(
							cell_stair_material, cap_stair->first, cap_stair->second);
					const auto rake_stair = create_stair_with_properties(
							cell_stair_material, cap_stair->first, StairShape::Straight);
					put_state(surface_stair, y);
					rake_caps.insert_or_assign({x, z}, RakeCap{y, rake_stair});
				} else if (all_low && base_slab && slab) {
					const auto cap = cached_prop_block(*slab, {{"type", "bottom"}});
					put_state(cap, y);
					rake_caps.insert_or_assign({x, z}, RakeCap{y, cap});
				} else {
					put_full(cell_roof_block, y);
				}
			} else {
				put_full(gable_block && (cell.dm_along == 0 || cell.dp_along == 0)
								 ? *gable_block
								 : roof_block,
						y);
			}
		}
	}
	std::unordered_set<std::pair<int, int>, PairHash> rake_cells;
	std::unordered_set<std::pair<int, int>, PairHash> eave_blocked_with_rake =
			eave_blocked ? *eave_blocked
						 : std::unordered_set<std::pair<int, int>, PairHash>{};
	if (!half_hipped) {
		std::vector<Block> only_air{AIR};
		for (const auto &[cell_pos, cap] : rake_caps) {
			const auto [x, z] = cell_pos;
			const auto &cell = cell_data.at(cell_pos);
			std::pair<int, int> ridge_step =
					ridge_runs_along_x ? std::pair{1, 0} : std::pair{0, 1};
			if (wings) {
				auto owner = wing_owner.find(cell_pos);
				if (owner == wing_owner.end())
					continue;
				ridge_step = RoofWingFrame::ridge_step(wings->wings[owner->second]);
			}
			const std::array<std::pair<int, int>, 2> ends{
					{{-ridge_step.first, -ridge_step.second}, ridge_step}};
			const std::array<int, 2> along_distances{cell.dm_along, cell.dp_along};
			for (std::size_t end = 0; end < ends.size(); ++end) {
				if (along_distances[end] != 0)
					continue;
				const auto [dx, dz] = ends[end];
				const std::pair<int, int> outside{x + dx, z + dz};
				if (footprint.contains(outside) ||
						(eave_blocked && eave_blocked->contains(outside)))
					continue;
				if (!wings && !roof_heights.contains({x - dx, z - dz}))
					continue;
				if (!rake_cells.insert(outside).second)
					continue;
				editor.set_block_with_properties_absolute(cap.block, outside.first,
						cap.y + abs_terrain_offset, outside.second, &only_air, nullptr);
				eave_blocked_with_rake.insert(outside);
			}
		}
	}
	std::unordered_set<std::pair<int, int>, PairHash> turned_eaves;
	if (tent)
		for (const auto &[x, z] : area)
			if (eave_cell(x, z))
				turned_eaves.insert({x, z});
	if (wings) {
		for (std::size_t i = 0; i < wings->wings.size(); ++i) {
			std::unordered_set<std::pair<int, int>, PairHash> wing_eaves;
			for (const auto &[x, z] : area) {
				const auto owner = wing_owner.find({x, z});
				if (owner != wing_owner.end() && owner->second == i && eave_cell(x, z))
					wing_eaves.insert({x, z});
			}
			place_gabled_eave_overhang(editor, area, base_height, abs_terrain_offset,
					roof_block, wings->wings[i].along_u, &wing_eaves, short_eave,
					&eave_blocked_with_rake, soffit && !slab_pitch, slab_pitch);
		}
	} else {
		place_gabled_eave_overhang(editor, area, base_height, abs_terrain_offset,
				roof_block, ridge_runs_along_x, tent ? &turned_eaves : nullptr,
				short_eave, &eave_blocked_with_rake, soffit && !slab_pitch, slab_pitch);
	}
	const bool dormers_fit =
			!slab_pitch && !round &&
			(!tent || std::abs(tent->angle * 180.0 / std::numbers::pi) <= 12.0) &&
			(!wings || wings->angle == 0.0);
	if (dormers_fit && wings) {
		const std::size_t main = wings->main_wing();
		const auto &wing = wings->wings[main];
		std::vector<std::pair<int, int>> main_area;
		main_area.reserve(area.size());
		for (const auto &cell : area) {
			const auto owner = wing_owner.find(cell);
			if (owner != wing_owner.end() && owner->second == main)
				main_area.push_back(cell);
		}
		const auto [b0, b1] = wing.across();
		std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> scans;
		scans.reserve(main_area.size());
		const auto whole = [](double d) {
			return std::max(0, static_cast<int>(std::floor(d + 1e-6)));
		};
		for (const auto &[x, z] : main_area) {
			const auto [u, v] = wings->uv(x, z);
			const double b = wing.along_u ? v : u;
			scans.emplace(std::pair{x, z}, std::pair{whole(b - b0), whole(b1 - b)});
		}
		place_roof_dormer_windows(editor, main_area, roof_heights, base_height,
				abs_terrain_offset, style_seed, roof_block, add_dormers,
				RoofWingFrame::ridge_step(wing), std::nullopt, {}, &scans, &footprint);
	} else if (dormers_fit) {
		place_roof_dormer_windows(editor, area, roof_heights, base_height,
				abs_terrain_offset, style_seed, roof_block, add_dormers, along);
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

RoofDistanceGrid roof_edge_distances(const std::vector<std::pair<int, int>> &roof_area)
{
	if (roof_area.empty())
		return {};
	std::unordered_set<std::pair<int, int>, PairHash> cells(
			roof_area.begin(), roof_area.end());
	RoofDistanceGrid distances;
	distances.reserve(cells.size() * 2);
	for (const auto &cell : cells)
		distances.emplace(cell, std::numeric_limits<int>::max());
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
			const auto it = distances.find(neighbor);
			if (it != distances.end() && it->second == std::numeric_limits<int>::max()) {
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
		const CoordinateBitmap *passages, const building_facade::PointSet *preset_shell,
		const building_facade::PointSet *photo_shell)
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
				(photo_shell && photo_shell->contains({x, z})) ||
				(!photo_shell && mapillary::facades::photo_column(x, z, element.id)))
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

void generate_chimney(WorldEditor &editor, const RoofHeightMap &surface, int roof_base,
		int abs_offset, std::uint64_t element_id, std::optional<int> row_spacing,
		Block shaft)
{
	if (surface.size() < 9)
		return;
	using Cell = std::pair<int, int>;
	std::vector<std::pair<Cell, int>> tops;
	tops.reserve(surface.size());
	for (const auto &[cell, y] : surface) {
		const auto [x, z] = cell;
		bool margin = true;
		for (int d = -2; d <= 2 && margin; ++d)
			margin = surface.contains({x + d, z}) && surface.contains({x, z + d});
		if (margin)
			tops.emplace_back(cell, y);
	}
	if (tops.empty())
		return;
	std::sort(tops.begin(), tops.end());
	const int ridge =
			std::max_element(tops.begin(), tops.end(), [](const auto &a, const auto &b) {
				return a.second < b.second;
			})->second;
	std::vector<Cell> band;
	for (const auto &[cell, y] : tops)
		if (y >= ridge - 1)
			band.push_back(cell);
	std::vector<Cell> crown;
	for (const auto &cell : band)
		if (surface.at(cell) == ridge)
			crown.push_back(cell);
	if (crown.empty())
		crown = band;
	const auto spread = [&](bool x_axis) {
		auto [lo, hi] = std::minmax_element(
				crown.begin(), crown.end(), [&](const Cell &a, const Cell &b) {
					return (x_axis ? a.first : a.second) < (x_axis ? b.first : b.second);
				});
		return (x_axis ? hi->first - lo->first : hi->second - lo->second);
	};
	const bool along_x = spread(true) >= spread(false);
	const auto across = [&](const Cell &cell) {
		return along_x ? cell.second : cell.first;
	};
	std::vector<int> lines;
	lines.reserve(crown.size());
	for (const auto &cell : crown)
		lines.push_back(across(cell));
	std::sort(lines.begin(), lines.end());
	int line = lines.front();
	std::size_t best_count = 0;
	for (std::size_t i = 0; i < lines.size();) {
		std::size_t j = i + 1;
		while (j < lines.size() && lines[j] == lines[i])
			++j;
		const std::size_t count = j - i;
		if (count > best_count || (count == best_count && lines[i] < line)) {
			line = lines[i];
			best_count = count;
		}
		i = j;
	}
	std::vector<Cell> row;
	for (const auto &cell : band)
		if (across(cell) == line)
			row.push_back(cell);
	std::sort(row.begin(), row.end(), [&](const Cell &a, const Cell &b) {
		return along_x ? a.first < b.first : a.second < b.second;
	});
	const std::size_t n = row.size();
	if (!n)
		return;
	auto rng = element_rng(element_id ^ 0xC41E000000000003ULL);
	std::vector<std::size_t> picks;
	if (row_spacing) {
		const std::size_t count = std::clamp<std::size_t>(
				n / static_cast<std::size_t>(std::max(1, *row_spacing)), 1, 6);
		for (std::size_t k = 0; k < count; ++k)
			picks.push_back((2 * k + 1) * n / (2 * count));
	} else if (n >= 12 && rng.random_bool(.35)) {
		picks = {n / 5, n - 1 - n / 5};
	} else {
		const std::size_t lo = n / 5;
		const std::size_t hi = std::max(lo, n - 1 - n / 5);
		picks.push_back(lo + rng.uniform(static_cast<std::uint32_t>(hi - lo + 1)));
	}
	const std::vector<Block> replace_any;
	for (const std::size_t pick : picks) {
		const auto [x, z] = row[std::min(pick, n - 1)];
		const int top = ridge + abs_offset + 1 + rng.uniform(2);
		for (int y = roof_base + abs_offset + 1; y <= top; ++y)
			editor.set_block_absolute(shaft, x, y, z, std::nullopt, replace_any);
		const Block cap = rng.random_bool(.35) ? EMPTY_FLOWER_POT : STONE_BRICK_SLAB;
		editor.set_block_absolute(cap, x, top + 1, z, std::nullopt, replace_any);
	}
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
		const int nx = normal_x * outward, nz = normal_z * outward;
		const bool horizontal = std::abs(nx) >= std::abs(nz);
		const std::string facing =
				horizontal ? (nx > 0 ? "east" : "west") : (nz > 0 ? "south" : "north");
		const int bx = mid_x + (horizontal ? (nx > 0 ? 1 : -1) : 0);
		const int bz = mid_z + (horizontal ? 0 : (nz > 0 ? 1 : -1));
		editor.place_wall_banner(
				WHITE_WALL_BANNER, bx, banner_y, bz, facing, "white", patterns);
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
		std::uint64_t seed, const building_facade::PointSet *preset_shell,
		const building_facade::PointSet *photo_shell)
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
				(photo_shell && photo_shell->contains({corner.x, corner.z})) ||
				(!photo_shell && mapillary::facades::photo_column(
										 corner.x, corner.z, element.id)) ||
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

// Rust's masonry treatment adds a proper eave, dentils and ground-floor
// string course; the older C++ pass only placed an occasional roofline stair.
void generate_facade_cornice_details(WorldEditor &editor, const ProcessedWay &element,
		const building_facade::FacadePlan &facade, BuildingCategory category, ArchEra era,
		bool europe, BuildingCondition condition, WindowArchetype window_archetype,
		Block wall_block, Block trim_block, bool has_windows, bool ground_level,
		bool storefront, bool sloped, int start_y, int building_height, int abs_offset,
		int floor_cycle, int grammar_anchor, const CoordinateBitmap *passages,
		const building_facade::PointSet *preset_shell,
		const building_facade::PointSet *photo_shell,
		const building_facade::PointSet &own_footprint,
		const CoordinateBitmap *building_footprints)
{
	if (condition != BuildingCondition::Normal || !has_windows || floor_cycle <= 0 ||
			building_height < 2 * floor_cycle ||
			window_archetype == WindowArchetype::Ribbon ||
			category == BuildingCategory::Industrial ||
			category == BuildingCategory::Warehouse ||
			category == BuildingCategory::Farm ||
			category == BuildingCategory::Religious ||
			category == BuildingCategory::ContemporarySkyscraper)
		return;
	const bool masonry =
			era == ArchEra::TraditionalPreWar || era == ArchEra::HistoricOrnate ||
			(era == ArchEra::Unknown && (europe || is_masonry_window_wall(wall_block)));
	const bool ornate =
			era == ArchEra::HistoricOrnate || category == BuildingCategory::Historic;
	const int floors = building_height / floor_cycle;
	const bool string_course =
			ground_level && ((masonry && floors >= (europe ? 2 : 3)) || storefront);
	if (!masonry && !string_course)
		return;

	const int top = start_y + building_height;
	const int course_y = start_y + grammar_anchor + floor_cycle;
	const Block stair_block = get_stair_block_for_material(trim_block);
	const auto slab = get_slab_block_for_material(trim_block);
	const auto ledge = cached_prop_block(slab, {{"type", "top"}});
	const BlockWithProperties cap = rooftops::bottom_slab(slab);
	std::unordered_set<std::pair<int, int>, PairHash> placed;
	for (std::size_t i = 0; i + 1 < element.nodes.size(); ++i) {
		const auto &a = element.nodes[i];
		const auto &b = element.nodes[i + 1];
		const int dx = b.x - a.x, dz = b.z - a.z;
		const std::pair<int, int> normal =
				i < facade.segments.size() && facade.segments[i]
						? facade.segments[i]->normal
						: std::pair{(dz > 0) - (dz < 0), (dx < 0) - (dx > 0)};
		const auto [nx, nz] = normal;
		if (nx == 0 && nz == 0)
			continue;
		const auto points = bresenham_line(a.x, top, a.z, b.x, top, b.z);
		for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
			const auto &[x, unused_y, z] = points[point_index];
			(void)unused_y;
			if (!placed.insert({x, z}).second || facade.is_party(x, z) ||
					(passage_at(passages, x, z)) ||
					(preset_shell && preset_shell->contains({x, z})) ||
					(photo_shell && photo_shell->contains({x, z})) ||
					(!photo_shell && mapillary::facades::photo_column(x, z, element.id)))
				continue;
			const int lx = x + nx, lz = z + nz;
			if (building_footprints && building_footprints->contains(lx, lz) &&
					!own_footprint.contains({lx, lz}))
				continue;
			const auto face = editor.get_block_absolute(x, top - 1 + abs_offset, z);
			const auto inner =
					editor.get_block_absolute(x - nx, top - 1 + abs_offset, z - nz);
			const bool dentil = ornate && point_index % 2 == 0 &&
								!(face && is_glass_wall_block(*face)) &&
								!(inner && is_glass_wall_block(*inner));
			const auto facing =
					std::abs(nx) >= std::abs(nz)
							? (nx >= 0 ? StairFacing::East : StairFacing::West)
							: (nz >= 0 ? StairFacing::South : StairFacing::North);
			if (masonry) {
				const auto eave = top_stair(create_stair_with_properties(
						stair_block, facing, StairShape::Straight));
				editor.set_block_with_properties_absolute(eave, lx, top + abs_offset, lz,
						std::vector<Block>{AIR}, std::nullopt);
				if (!sloped)
					editor.set_block_with_properties_absolute(cap, lx,
							top + 1 + abs_offset, lz, std::vector<Block>{AIR},
							std::nullopt);
				if (dentil)
					editor.set_block_with_properties_absolute(ledge, lx,
							top - 1 + abs_offset, lz, std::vector<Block>{AIR},
							std::nullopt);
			}
			if (string_course && course_y < top - 1)
				editor.set_block_with_properties_absolute(ledge, lx,
						course_y + abs_offset, lz, std::vector<Block>{AIR}, std::nullopt);
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

using InsetWallSelector = std::function<Block(int, int, int, bool, bool)>;

std::optional<int> generate_inset_tiers(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &roof_area,
		const RoofDistanceGrid &distances, int base_y, int abs_terrain_offset,
		const std::vector<InsetTier> &tiers, int floor_cycle, int floor_row_origin,
		Block floor_block, const rooftops::RoofMembrane &cover,
		const InsetWallSelector &select_wall)
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
			const bool open_x = distances.find({x + 1, z}) == distances.end() ||
								distances.at({x + 1, z}) < tier.inset ||
								distances.find({x - 1, z}) == distances.end() ||
								distances.at({x - 1, z}) < tier.inset;
			const bool open_z = distances.find({x, z + 1}) == distances.end() ||
								distances.at({x, z + 1}) < tier.inset ||
								distances.find({x, z - 1}) == distances.end() ||
								distances.at({x, z - 1}) < tier.inset;
			for (int h = 0; h < tier.height; ++h) {
				const int y = current_base + h;
				const int row = ((y - floor_row_origin) % floor_cycle + floor_cycle) %
								floor_cycle;
				if (wall) {
					const Block selected = select_wall(x, y, z, open_x, open_x && open_z);
					editor.set_block_absolute(selected, x, y + abs_terrain_offset, z,
							std::nullopt, replace_any);
				} else if (tier.with_ceilings && row == 0) {
					editor.set_block_absolute(
							(x % 3 == 0 && z % 3 == 0) ? GLOWSTONE : floor_block, x,
							y + abs_terrain_offset, z, std::nullopt, replace_any);
				}
			}
			editor.set_block_absolute(cover.block_at(x, z), x,
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
		int building_height, int abs_terrain_offset, int floor_cycle,
		int floor_row_origin, Block floor_block, const rooftops::RoofMembrane &cover,
		std::uint64_t element_id, const InsetWallSelector &select_wall)
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
			floor_row_origin, floor_block, cover, select_wall);
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
		int32_t abs_terrain_offset, double scale_factor, bool short_eave,
		std::optional<RoofStyleContext> style_context,
		const std::vector<HolePolygon> *hole_polygons,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_blocked,
		bool tone_patches);

std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels)
{
	FloodFillCache empty_cache;
	CoordinateBitmap empty_passages = CoordinateBitmap::new_empty();
	return generate_buildings(
			editor, element, args, relation_levels, empty_cache, empty_passages, nullptr);
}

namespace
{
bool painted_metal(const std::string &material, const tags_t &tags)
{
	std::string normalized = material;
	const auto first = normalized.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return false;
	const auto last = normalized.find_last_not_of(" \t\r\n");
	normalized = normalized.substr(first, last - first + 1);
	std::transform(normalized.begin(), normalized.end(), normalized.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return (normalized == "steel" || normalized == "metal" || normalized == "iron") &&
		   (tags.contains("building:colour") || tags.contains("colour"));
}

Block wall_block_for_colour_tag(RGBTuple rgb, ChaCha8Rng &rng)
{
	static const std::array<std::pair<Block, RGBTuple>, 13> render_tones{{
			{WHITE_CONCRETE, {207, 213, 214}},
			{WHITE_TERRACOTTA, {210, 178, 161}},
			{CHERRY_PLANKS, {226, 178, 172}},
			{SMOOTH_SANDSTONE, {224, 214, 170}},
			{MUSHROOM_STEM, {203, 196, 185}},
			{BIRCH_PLANKS, {192, 175, 121}},
			{END_STONE_BRICKS, {218, 224, 162}},
			{YELLOW_TERRACOTTA, {186, 133, 35}},
			{LIGHT_GRAY_CONCRETE_POWDER, {154, 154, 148}},
			{SMOOTH_STONE, {159, 159, 159}},
			{RED_TERRACOTTA, {143, 61, 47}},
			{BRICK, {151, 98, 83}},
			{TERRACOTTA, {152, 94, 68}},
	}};
	const auto target = oklab_components(rgb);
	if (target[0] < .55f || std::hypot(target[1], target[2]) >= .14f)
		return block_palette::wall_block_for_color(rgb, rng);

	Block best = WHITE_CONCRETE;
	float best_distance = std::numeric_limits<float>::infinity();
	for (const auto &[block, color] : render_tones) {
		const auto lab = oklab_components(color);
		const float dl = .5f * (target[0] - lab[0]);
		const float da = target[1] - lab[1];
		const float db = target[2] - lab[2];
		const float distance = dl * dl + da * da + db * db;
		if (distance < best_distance) {
			best = block;
			best_distance = distance;
		}
	}
	return best;
}
} // namespace

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
			element.tags.contains("building")
					? element.tags.get("building")
					: (element.tags.contains("building:part")
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
	bool roof_enclosed = false;
	if (building_footprints && !cached_floor_area.empty()) {
		const std::unordered_set<std::pair<int, int>, PairHash> own_footprint(
				cached_floor_area.begin(), cached_floor_area.end());
		std::size_t exposed_ring = 0;
		std::size_t neighboring_walls = 0;
		for (const auto &[x, z] : cached_floor_area) {
			for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
				const int nx = x + dx, nz = z + dz;
				if (own_footprint.contains({nx, nz}))
					continue;
				++exposed_ring;
				if (building_footprints->contains(nx, nz))
					++neighboring_walls;
			}
		}
		roof_enclosed = exposed_ring > 0 && neighboring_walls * 10 >= exposed_ring * 9;
	}
	int start_y_offset = 0;
	if (terrain_enabled) {
		const int terrain_base = terrain_base_level(
				*editor, element, args.ground_level, cached_floor_area);
		start_y_offset = terrain_base + min_level_offset;
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
	if (bridge_towers::is_support(element.tags)) {
		const int support_floor_cycle =
				floor_cycle_for(early_building_type, element.tags);
		const auto [support_height, ignored_tall] = calculate_building_height(element,
				early_building_type, min_level, scale_factor, relation_levels,
				support_floor_cycle, cached_footprint_size, group_seed);
		(void)ignored_tall;
		bridge_towers::place_support(*editor, element, cached_floor_area,
				start_y_offset + abs_terrain_offset, support_height,
				min_level_offset == 0);
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
	if (min_level_offset == 0 && road_mask && building_footprints &&
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
	struct WindowLifeColumn
	{
		int x, z, nx, nz;
		int run, bay, k, width;
		bool street;
	};
	struct FacadeWindowColumn
	{
		int x, z, nx, nz, run, position;
		BaySlot slot, shop_slot;
	};
	struct SyntheticEntrancePlan
	{
		std::size_t segment;
		int x, z;
		bool double_door;
	};
	struct PlannedUnitEntrance
	{
		int x, z, nx, nz;
	};
	std::vector<WindowLifeColumn> window_life_columns;
	std::vector<FacadeWindowColumn> facade_window_columns;
	std::optional<SyntheticEntrancePlan> synthetic_entrance_plan;
	std::vector<PlannedUnitEntrance> planned_unit_entrances;
	std::vector<PlannedUnitEntrance> planned_entrances;

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
		if (it_material != element.tags.end() &&
				!painted_metal(it_material->second, element.tags)) {
			if (auto block = get_wall_block_for_material_cpp(it_material->second, rng))
				return block;
		}

		auto it_color = element.tags.find("building:colour");
		if (it_color == element.tags.end())
			it_color = element.tags.find("colour");
		if (it_color != element.tags.end())
			if (const auto rgb = color_text_to_rgb_tuple(it_color->second))
				return wall_block_for_colour_tag(*rgb, rng);

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
	ArchEra era = building_arch_era(element.tags);
	if (era == ArchEra::Unknown && element.tags.contains("building:part"))
		era = static_cast<ArchEra>(osm_parser::arch_era_from_hint(
				osm_parser::style_hint_from_seed(group_seed)));
	const auto climate = editor->get_ground() ? editor->get_ground()->climate()
											  : biome::Climate::Temperate;
	const auto [wall_lat, wall_lon] = editor->geo_anchor();
	const auto wall_region = building_style::material_region_at(wall_lat, wall_lon);
	std::size_t party_length = 0;
	std::size_t facade_length = 0;
	for (const auto &segment : facade_plan.segments) {
		if (!segment)
			continue;
		facade_length += segment->len;
		if (segment->facade_class == building_facade::FacadeClass::Party)
			party_length += segment->len;
	}
	const bool wall_attached =
			party_length >= 4 && party_length * 100 >= facade_length * 15;
	const bool part_has_explicit_top =
			element.tags.contains("building:part") &&
			(element.tags.contains("height") || element.tags.contains("building:levels"));
	if (condition == BuildingCondition::Construction)
		building_height = std::max(3, building_height / 2);
	else if (condition == BuildingCondition::Ruined)
		building_height = std::max(3, int(building_height * .6));
	std::optional<RoofType> mapped_auto_roof;
	if (args.roof && parse_building_height_tag(element.tags) &&
			!element.tags.contains("roof:height") &&
			!element.tags.contains("roof:angle")) {
		if (const auto shape = element.tags.find("roof:shape");
				shape != element.tags.end()) {
			if (const auto rise = mapped_height_roof_rise(element,
						parse_roof_type(shape->second), condition, building_height,
						cached_footprint_size, category == BuildingCategory::Religious,
						scale_factor))
				building_height = std::max(3, building_height - *rise);
		} else {
			std::string roof_building_type = "yes";
			if (const auto it = element.tags.find("building"); it != element.tags.end())
				roof_building_type = it->second;
			const bool roof_auto_attached =
					roof_building_type == "terrace" ||
					roof_building_type == "semidetached_house" ||
					std::any_of(cached_floor_area.begin(), cached_floor_area.end(),
							[&](const auto &cell) {
								return facade_plan.is_party(cell.first, cell.second);
							});
			mapped_auto_roof = mapped_height_auto_roof(element, roof_building_type,
					category, era, climate, wall_region, cached_footprint_size,
					building_height, args.scale, roof_auto_attached, roof_enclosed,
					part_has_explicit_top, group_seed);
			if (mapped_auto_roof)
				if (const auto rise = mapped_height_roof_rise(element, *mapped_auto_roof,
							condition, building_height, cached_footprint_size,
							category == BuildingCategory::Religious, scale_factor))
					building_height = std::max(3, building_height - *rise);
		}
	}
	// Rust records preset facade candidates after category/height selection.  The
	// backend-native registry performs the equivalent crop and submission here;
	// it is a no-op unless --building-facades was enabled and a backend sink is
	// installed.
	const auto preset_shell = building_facades::collect(*editor, element.nodes,
			element.id, visual_seed, category, facade_plan, building_footprints,
			current_footprint, min_level_offset == 0,
			start_y_offset + abs_terrain_offset + 1, building_height, scale_factor);
	// Match Rust's photo_shell_columns: image-covered columns and their adjacent
	// segment columns are kept as a flat wall, avoiding a lone procedural window
	// or ornament at the rounded edge of a photographed facade.
	building_facade::PointSet photo_shell;
	if (mapillary::facades::has_photo_shell(element.id)) {
		for (std::size_t segment = 1; segment < element.nodes.size(); ++segment) {
			const auto &a = element.nodes[segment - 1];
			const auto &b = element.nodes[segment];
			const auto points = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
			for (std::size_t i = 0; i < points.size(); ++i) {
				const auto &[x, unused_y, z] = points[i];
				(void)unused_y;
				if (!mapillary::facades::photo_column(x, z, element.id))
					continue;
				const std::size_t begin = i == 0 ? 0 : i - 1;
				const std::size_t end = std::min(points.size(), i + 2);
				for (std::size_t near_ = begin; near_ < end; ++near_)
					photo_shell.emplace(
							std::get<0>(points[near_]), std::get<2>(points[near_]));
			}
		}
	}
	const auto is_flat_column = [&](int x, int z) {
		return photo_shell.contains({x, z}) || preset_shell.contains({x, z});
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
					 (wall_depth_style == WallDepthStyle::SkyscraperFins &&
							 category == BuildingCategory::ModernSkyscraper)) ||
					(wall_depth_style == WallDepthStyle::SubtlePilasters &&
							element_rng(visual_seed + 7919).uniform(100) < 60));
	int district_min_x = min_x, district_max_x = max_x;
	int district_min_z = min_z, district_max_z = max_z;
	for (const auto &[x, z] : sibling_cells) {
		district_min_x = std::min(district_min_x, x);
		district_max_x = std::max(district_max_x, x);
		district_min_z = std::min(district_min_z, z);
		district_max_z = std::max(district_max_z, z);
	}
	const int district_cell =
			std::max(16, static_cast<int>(std::lround(128.0 * scale_factor)));
	const int district_x = floor_div(
			district_min_x + (district_max_x - district_min_x) / 2, district_cell);
	const int district_z = floor_div(
			district_min_z + (district_max_z - district_min_z) / 2, district_cell);
	auto district_rng = coord_rng(district_x, district_z, 0xD157C7A15EED0001ULL);
	const auto district_low = district_rng();
	const auto district_high = district_rng();
	const std::uint64_t wall_district =
			std::uint64_t(district_low) | (std::uint64_t(district_high) << 32);
	const auto tag_value = [&](std::string_view key) -> std::string_view {
		auto it = element.tags.find(std::string(key));
		return it == element.tags.end() ? std::string_view{} : it->second;
	};
	const auto historic_tag = tag_value("historic");
	const bool historic_fortification =
			historic_tag == "castle" || historic_tag == "fort" ||
			historic_tag == "fortress" || historic_tag == "fortification" ||
			historic_tag == "citywalls" || historic_tag == "city_wall" ||
			historic_tag == "city_gate" || historic_tag == "tower" ||
			historic_tag == "ruins" || historic_tag == "bunker" ||
			historic_tag == "archaeological_site" || building_type == "castle" ||
			building_type == "ruins" || building_type == "fort" ||
			building_type == "bunker";
	const auto pick_untagged_wall = [&]() {
		// Glass-family presets use the wall block itself as the curtain glazing
		// (their style disables the separate window pattern). Keep them on the
		// glass palette instead of passing them through the regional opaque-wall
		// typology, or these buildings become solid concrete/masonry shells.
		if (category == BuildingCategory::GlassySkyscraper ||
				category == BuildingCategory::GlassCornerSkyscraper)
			return rust_category_wall_block(category, era, climate, rng);
		if (auto regional = rust_regional_wall_block(category, era, climate, wall_region,
					style_building_height, scale_factor, wall_attached,
					historic_fortification, wall_district, rng))
			return *regional;
		return rust_category_wall_block(category, era, climate, rng);
	};
	if (category == BuildingCategory::GlassySkyscraper) {
		if (element.tags.get("historic") == "castle")
			wall_block = get_castle_wall_block(rng);
		else if (condition != BuildingCondition::Construction)
			wall_block = pick_untagged_wall();
	} else if (explicit_wall) {
		if (const auto tagged_wall = resolve_tagged_wall())
			wall_block = *tagged_wall;
		else if (condition != BuildingCondition::Construction)
			wall_block = pick_untagged_wall();
	} else if (condition != BuildingCondition::Construction) {
		wall_block = pick_untagged_wall();
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
	const WindowArchetype window_archetype = pick_window_archetype(category, era,
			building_style::building_region(wall_region), wall_block, clean_visual_seed);
	// Rust spaces European stucco and sandstone windows a column farther apart
	// for their flush Faschen surrounds; use the same bay rhythm on the facade
	// and on inset tower tiers. Brick and the other heavy masonry keep plain
	// openings.
	const bool window_surrounds =
			building_style::building_region(wall_region) ==
					building_style::BuildingRegion::Europe &&
			has_windows && wall_block != BRICK && wall_block != TERRACOTTA &&
			wall_block != RED_TERRACOTTA && wall_block != ORANGE_TERRACOTTA &&
			wall_block != BROWN_TERRACOTTA && wall_block != NETHER_BRICK &&
			wall_block != RED_NETHER_BRICKS && wall_block != MUD_BRICKS &&
			wall_block != GRANITE && wall_block != POLISHED_GRANITE &&
			wall_block != BROWN_CONCRETE && condition == BuildingCondition::Normal &&
			detail != DetailTier::Minimal;
	const bool frame_archetype = window_archetype != WindowArchetype::Ribbon &&
								 window_archetype != WindowArchetype::Piers &&
								 window_archetype != WindowArchetype::Strips;
	const auto window_frame = has_windows && condition == BuildingCondition::Normal &&
											  !is_tall_building && frame_archetype
									  ? pick_window_frame(category, era, detail,
												wall_block, clean_visual_seed)
									  : std::nullopt;
	static const std::array<Block, 4> shutter_options{
			OAK_TRAPDOOR_OPEN_NORTH, DARK_OAK_TRAPDOOR, SPRUCE_TRAPDOOR, BIRCH_TRAPDOOR};
	auto residential_detail_rng = element_rng(element.id);
	const Block shutter_base = shutter_options[residential_detail_rng.uniform(
			static_cast<std::uint32_t>(shutter_options.size()))];
	static const std::array<Block, 5> sill_options{
			QUARTZ_SLAB_TOP, STONE_BRICK_SLAB, MUD_BRICK_SLAB, OAK_SLAB, BRICK_SLAB};
	const Block sill_base = sill_options[residential_detail_rng.uniform(
			static_cast<std::uint32_t>(sill_options.size()))];
	static const std::array<Block, 4> potted_plant_options{
			FLOWER_POT, POTTED_RED_TULIP, POTTED_DANDELION, POTTED_BLUE_ORCHID};
	const int window_phase =
			element.tags.contains("building:part")
					? 0
					: int(element_rng(static_cast<std::uint64_t>(element.id) ^
									  0x77D0A3E19B1C5544ULL)
									  .uniform(6));
	const bool horizontal_windows = category == BuildingCategory::ModernSkyscraper;
	const bool european_punched_window_cap =
			building_style::building_region(wall_region) ==
					building_style::BuildingRegion::Europe &&
			window_archetype != WindowArchetype::Ribbon &&
			window_archetype != WindowArchetype::Wide &&
			window_archetype != WindowArchetype::Piers &&
			window_archetype != WindowArchetype::Strips;
	enum class GroundFloorKind
	{
		Plain,
		Shopfront,
		Lobby
	};
	GroundFloorKind ground_floor = GroundFloorKind::Plain;
	if (has_windows && condition == BuildingCondition::Normal && min_level_offset == 0) {
		const bool tower = category == BuildingCategory::TallBuilding ||
						   category == BuildingCategory::ModernSkyscraper ||
						   category == BuildingCategory::ContemporarySkyscraper ||
						   category == BuildingCategory::MasonrySkyscraper ||
						   category == BuildingCategory::GridSkyscraper;
		if (tower) {
			ground_floor = GroundFloorKind::Lobby;
		} else if (facade_plan.has_any_street) {
			const auto shop_tag = element.tags.get("shop");
			const auto amenity_tag = element.tags.get("amenity");
			const bool tagged_shop = (!shop_tag.empty() && shop_tag != "no") ||
									 amenity_tag == "restaurant" ||
									 amenity_tag == "cafe" || amenity_tag == "bar" ||
									 amenity_tag == "pub" || amenity_tag == "fast_food" ||
									 amenity_tag == "bank" || amenity_tag == "pharmacy";
			if (tagged_shop) {
				ground_floor = GroundFloorKind::Shopfront;
			} else {
				const bool corner = facade_plan.corner.has_value();
				double chance = 0.0;
				if (category == BuildingCategory::Commercial ||
						category == BuildingCategory::Hotel)
					chance = corner ? .85 : .65;
				else if (category == BuildingCategory::Office)
					chance = .40;
				else if ((category == BuildingCategory::Residential ||
								 category == BuildingCategory::Default) &&
						 building_height / std::max(1, floor_cycle) >= 3 &&
						 (wall_attached || corner)) {
					const auto region = building_style::building_region(wall_region);
					chance = region == building_style::BuildingRegion::NorthAmerica ? .60
							 : region == building_style::BuildingRegion::Europe		? .45
																					: .30;
					if (corner)
						chance += .15;
				}
				if (chance > 0.0 && element_rng(group_seed ^ 0x5709EF9000000002ULL)
											.random_bool(chance))
					ground_floor = GroundFloorKind::Shopfront;
			}
		}
	}
	const bool has_lobby_base = ground_floor == GroundFloorKind::Lobby;
	const bool glazed_ground_floor = ground_floor != GroundFloorKind::Plain;
	const bool storefront = ground_floor == GroundFloorKind::Shopfront;
	const auto shop_region = building_style::building_region(wall_region);
	const Block shop_frame_options[] = {BLACK_CONCRETE, POLISHED_DEEPSLATE,
			shop_region == building_style::BuildingRegion::Europe ? GRAY_CONCRETE
																  : POLISHED_BLACKSTONE,
			shop_region == building_style::BuildingRegion::Europe ? POLISHED_BLACKSTONE
																  : DARK_PRISMARINE};
	const Block shop_frame =
			shop_frame_options[element_rng(group_seed ^ 0x5409F2A300000003ULL)
									   .uniform(4)];
	const Block awning_options[] = {WARPED_TRAPDOOR, SPRUCE_TRAPDOOR, DARK_OAK_TRAPDOOR,
			JUNGLE_TRAPDOOR, ACACIA_TRAPDOOR};
	auto awning_rng = element_rng(group_seed ^ 0x0A3B11B60000000BULL);
	const bool awning_enabled = awning_rng.random_bool(.55);
	const Block awning_block = awning_options[awning_rng.uniform(5)];
	const auto roof_shape = element.tags.get("roof:shape");
	const bool sloped_roof =
			args.roof && roof_shape != "flat" &&
			(!roof_shape.empty() ||
					(mapped_auto_roof && pitched_roof(*mapped_auto_roof)) ||
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
	const BalconyBand balcony_band = pick_balcony_band(category, era,
			building_style::building_region(wall_region), building_height, floor_cycle,
			facade_plan.has_any_street, clean_visual_seed);
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
				const std::uint32_t hip_limit = prewar ? 65u : 80u;
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

	const bool mapped_entrance = std::any_of(
			element.nodes.begin(), element.nodes.end(), [](const ProcessedNode &node) {
				const auto entrance = node.tags.get("entrance");
				const auto door = node.tags.get("door");
				if ((!node.tags.contains("entrance") || entrance == "no") &&
						(!node.tags.contains("door") || door == "no"))
					return false;
				if (const auto level = node.tags.get("level"); !level.empty()) {
					if (const auto parsed = strict_parse::f64(level);
							parsed && *parsed != 0.0)
						return false;
				}
				return true;
			});
	const bool interior_allowed =
			args.interior && cached_floor_area.size() >= 24 &&
			std::find(INTERIOR_EXCLUSIONS.begin(), INTERIOR_EXCLUSIONS.end(),
					std::string_view(building_type)) == INTERIOR_EXCLUSIONS.end() &&
			condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined;
	std::vector<int> floor_levels;
	std::optional<interior_uses::InteriorPlan> interior_plan;
	bool open_hall = false;
	if (std::get<2>(corner_addup) != 0) {
		floor_levels.push_back(start_y_offset);
		if (building_height > floor_cycle + 2) {
			const int num_upper_floors = std::max(1, building_height / floor_cycle);
			for (int floor = 1; floor < num_upper_floors; ++floor)
				floor_levels.push_back(
						start_y_offset + grammar_anchor + floor * floor_cycle);
		}
		static const std::vector<interior_uses::Tenant> no_interior_tenants;
		const auto &interior_tenants = interior_index
											   ? interior_index->tenants(element.id)
											   : no_interior_tenants;
		const auto interior_area =
				interior_index ? interior_index->area(element.id) : std::nullopt;
		if (interior_allowed) {
			const interior_uses::PlanInputs plan_inputs{element.tags, building_type,
					floor_levels.size(), min_level, min_level_offset > 0,
					cached_floor_area.size(), {(min_x + max_x) / 2, (min_z + max_z) / 2},
					interior_tenants, interior_area};
			interior_plan = interior_uses::plan_interior(plan_inputs);
		}
		open_hall = interior_plan && interior_plan->open_hall;
	}
	const bool thin_ring_footprint = cached_footprint_size <= THIN_RING_MAX_CELLS;
	const bool allow_preset_utility_door = !mapped_entrance && !thin_ring_footprint &&
										   condition != BuildingCondition::Construction &&
										   condition != BuildingCondition::Ruined &&
										   condition != BuildingCondition::Abandoned;
	const bool preset_utility_door =
			allow_preset_utility_door &&
			(category == BuildingCategory::Garage || category == BuildingCategory::Shed ||
					category == BuildingCategory::Greenhouse);
	// Rust plans mapped entrances before drawing the facade so window bays,
	// sills, shutters, and other details leave both the door and its threshold
	// clear. C++ renders doors later in this monolithic pass, so reserve the
	// same columns here, using the segment choice used by the render pass.
	if (min_level_offset == 0 && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage) {
		std::unordered_set<std::pair<int, int>, PairHash> reserved_entrances;
		for (std::size_t i = 0; i < element.nodes.size(); ++i) {
			const auto &node = element.nodes[i];
			if (!((node.tags.contains("entrance") && node.tags.get("entrance") != "no") ||
						(node.tags.contains("door") && node.tags.get("door") != "no")))
				continue;
			if (const auto level_it = node.tags.find("level");
					level_it != node.tags.end()) {
				std::string_view level = level_it->second;
				while (!level.empty() &&
						std::isspace(static_cast<unsigned char>(level.front())))
					level.remove_prefix(1);
				while (!level.empty() &&
						std::isspace(static_cast<unsigned char>(level.back())))
					level.remove_suffix(1);
				if (const auto parsed = strict_parse::f64(level);
						parsed && *parsed != 0.0)
					continue;
			}
			if (!reserved_entrances.emplace(node.x, node.z).second)
				continue;
			const building_facade::SegmentFacade *before =
					i > 0 && i - 1 < facade_plan.segments.size() &&
									facade_plan.segments[i - 1]
							? &*facade_plan.segments[i - 1]
							: nullptr;
			const building_facade::SegmentFacade *after =
					i < facade_plan.segments.size() && facade_plan.segments[i]
							? &*facade_plan.segments[i]
							: nullptr;
			const auto *segment = before && after
										  ? (before->len >= after->len ? before : after)
										  : (before ? before : after);
			if (segment) {
				facade_plan.mark_door_column(node.x, node.z);
				facade_plan.mark_door_column(
						node.x + segment->normal.first, node.z + segment->normal.second);
				if (node.tags.get("entrance") == "main" &&
						(category == BuildingCategory::Commercial ||
								category == BuildingCategory::Office ||
								category == BuildingCategory::Hotel) &&
						segment->len >= 8 &&
						(segment->tangent.first == 0 || segment->tangent.second == 0)) {
					const bool ends_here =
							before && (!after || before->len >= after->len);
					const auto tangent = ends_here ? std::pair{-segment->tangent.first,
															 -segment->tangent.second}
												   : segment->tangent;
					const int x = node.x + tangent.first, z = node.z + tangent.second;
					if (!passage_at(effective_passages, x, z) &&
							!facade_plan.is_party(x, z)) {
						facade_plan.mark_door_column(x, z);
						facade_plan.mark_door_column(
								x + segment->normal.first, z + segment->normal.second);
					}
				}
				continue;
			}
			// If there is no road-derived facade segment, use the same footprint
			// inward-neighbour inference as the mapped-door renderer below.
			std::optional<std::pair<int, int>> inward;
			int best_score = std::numeric_limits<int>::min();
			for (const auto direction : {std::pair{0, -1}, std::pair{1, 0},
						 std::pair{0, 1}, std::pair{-1, 0}}) {
				if (!current_footprint.contains(
							{node.x + direction.first, node.z + direction.second}))
					continue;
				const int score = direction.first * (min_x + max_x - 2 * node.x) +
								  direction.second * (min_z + max_z - 2 * node.z);
				if (!inward || score > best_score) {
					inward = direction;
					best_score = score;
				}
			}
			if (inward) {
				facade_plan.mark_door_column(node.x, node.z);
				facade_plan.mark_door_column(
						node.x - inward->first, node.z - inward->second);
			}
		}
	}

	// The wall loop below also records these bay slots as it draws. Build that
	// compact plan first so a synthetic entrance can reserve its wall and sill
	// columns before any facade detail is written.
	for (std::size_t segment_index = 0; segment_index + 1 < element.nodes.size() &&
										segment_index < facade_plan.segments.size();
			++segment_index) {
		if (!facade_plan.segments[segment_index])
			continue;
		const auto &a = element.nodes[segment_index];
		const auto &b = element.nodes[segment_index + 1];
		auto points = bresenham_line(a.x, start_y_offset, a.z, b.x, start_y_offset, b.z);
		BayRhythm rhythm = window_rhythm(window_archetype);
		if (window_surrounds)
			++rhythm.pier;
		const auto bays = BayLayout::plan(static_cast<int>(points.size()), rhythm);
		const auto shop_bays = BayLayout::plan(static_cast<int>(points.size()),
				BayRhythm{3, 1, std::clamp(bays.lead, 1, 2)});
		const auto normal = facade_plan.segments[segment_index]->normal;
		if (normal == std::pair<int, int>{0, 0})
			continue;
		for (std::size_t position = 0; position < points.size(); ++position) {
			const auto &point = points[position];
			facade_window_columns.push_back({std::get<0>(point), std::get<2>(point),
					normal.first, normal.second, static_cast<int>(segment_index),
					static_cast<int>(position), bays.slot(static_cast<int>(position)),
					shop_bays.slot(static_cast<int>(position))});
		}
	}

	// Choose and reserve the same seeded, bay-aware synthetic entry used by the
	// rendering pass. In particular, its door and outward threshold stay clear
	// of wall/window details regardless of the later render order.
	if (!mapped_entrance && min_level_offset == 0 &&
			condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage) {
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
			const auto points = bresenham_line(a.x, 0, a.z, b.x, 0, b.z);
			if (points.size() >= 5) {
				auto position_rng = element_rng(group_seed ^ 0xD00E57E900000011ULL);
				std::size_t target = points.size() / 2;
				if (corner_entrance) {
					const int from_corner = 2 + int(position_rng.uniform(2));
					const bool corner_at_end =
							facade_plan.corner->seg_a == *entrance_segment;
					target = corner_at_end ? points.size() - 1 -
													 static_cast<std::size_t>(from_corner)
										   : static_cast<std::size_t>(from_corner);
				} else {
					const double jitter = position_rng.uniform_real(-.15, .15);
					target = static_cast<std::size_t>(
							static_cast<double>(points.size()) * (.5 + jitter));
				}
				const bool wants_double =
						(ground_floor == GroundFloorKind::Lobby ||
								category == BuildingCategory::Commercial ||
								category == BuildingCategory::Office ||
								category == BuildingCategory::Hotel ||
								category == BuildingCategory::School ||
								category == BuildingCategory::Hospital) &&
						segment.len >= 8 &&
						(segment.tangent.first == 0 || segment.tangent.second == 0);
				const auto usable = [&](std::size_t index) {
					if (index < 2 || index + 2 >= points.size())
						return false;
					const int x = std::get<0>(points[index]);
					const int z = std::get<2>(points[index]);
					return !passage_at(effective_passages, x, z) &&
						   !facade_plan.is_party(x, z);
				};
				const auto bay_accepts = [&](std::size_t index, int leaves) {
					const auto column = std::find_if(facade_window_columns.begin(),
							facade_window_columns.end(),
							[&](const FacadeWindowColumn &c) {
								return c.run == static_cast<int>(*entrance_segment) &&
									   c.position == static_cast<int>(index);
							});
					if (column == facade_window_columns.end())
						return false;
					const bool shopfront_column =
							(ground_floor != GroundFloorKind::Plain) &&
							facade_plan.is_street(column->x, column->z) &&
							!facade_plan.is_party(column->x, column->z) && has_windows;
					const BaySlot &slot =
							shopfront_column ? column->shop_slot : column->slot;
					return slot.kind == BaySlot::Kind::Window &&
						   slot.k + leaves <= slot.width;
				};
				const auto nearest = [&](auto accepts) -> std::optional<std::size_t> {
					const int reach = static_cast<int>(points.size() / 2);
					for (int distance = 0; distance <= reach; ++distance)
						for (const int sign : {1, -1}) {
							if (distance == 0 && sign < 0)
								continue;
							const int index = static_cast<int>(target) + sign * distance;
							if (index < 0 || index >= static_cast<int>(points.size()))
								continue;
							const auto candidate = static_cast<std::size_t>(index);
							if (usable(candidate) && accepts(candidate))
								return candidate;
						}
					return std::nullopt;
				};
				std::optional<std::size_t> door_index;
				bool double_door = false;
				if (wants_double)
					if (const auto index = nearest(
								[&](std::size_t i) { return bay_accepts(i, 2); })) {
						door_index = index;
						double_door = true;
					}
				if (!door_index)
					door_index =
							nearest([&](std::size_t i) { return bay_accepts(i, 1); });
				if (!door_index)
					door_index = nearest([](std::size_t) { return true; });
				if (door_index) {
					const int x = std::get<0>(points[*door_index]);
					const int z = std::get<2>(points[*door_index]);
					const int nx = segment.normal.first;
					const int nz = segment.normal.second;
					facade_plan.mark_door_column(x, z);
					facade_plan.mark_door_column(x + nx, z + nz);
					if (double_door) {
						const int second_x = x + segment.tangent.first;
						const int second_z = z + segment.tangent.second;
						if (!passage_at(effective_passages, second_x, second_z) &&
								!facade_plan.is_party(second_x, second_z)) {
							facade_plan.mark_door_column(second_x, second_z);
							facade_plan.mark_door_column(second_x + nx, second_z + nz);
						} else {
							double_door = false;
						}
					}
					synthetic_entrance_plan =
							SyntheticEntrancePlan{*entrance_segment, x, z, double_door};
				}
			}
		}
	}

	// Process nodes to create walls and corners
	WallVariety wall_variety(element.id);
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
			BayRhythm facade_rhythm = window_rhythm(window_archetype);
			if (window_surrounds)
				++facade_rhythm.pier;
			const BayLayout facade_bays = BayLayout::plan(
					static_cast<int>(bresenham_points.size()), facade_rhythm);
			const BayLayout shop_bays =
					BayLayout::plan(static_cast<int>(bresenham_points.size()),
							BayRhythm{3, 1, std::clamp(facade_bays.lead, 1, 2)});
			for (std::size_t point_index = 0; point_index < bresenham_points.size();
					++point_index) {
				const auto &t = bresenham_points[point_index];
				int bx = std::get<0>(t);
				int bz = std::get<2>(t);
				const int wall_u =
						wall_ordinate(point_index, start_is_x, segment_is_x, bx, bz);
				const bool wall_axis_x = point_index == 0 ? start_is_x : segment_is_x;
				const int wall_step =
						(wall_axis_x ? x - prev.first : z - prev.second) >= 0 ? 1 : -1;
				const BaySlot bay_slot = facade_bays.slot(static_cast<int>(point_index));
				const BaySlot shop_slot = shop_bays.slot(static_cast<int>(point_index));
				const bool facade_corner =
						facade_bays.len >= 5 &&
						(point_index == 0 || point_index + 1 == bresenham_points.size());
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
				const bool rear_facade =
						node_index - 1 < facade_plan.segments.size() &&
						facade_plan.segments[node_index - 1] &&
						facade_plan.segments[node_index - 1]->facade_class ==
								building_facade::FacadeClass::Rear;
				if (bay_slot.is_window() && !party_wall && !is_passage &&
						!facade_plan.is_door(bx, bz) && !is_flat_column(bx, bz) &&
						outward != std::pair<int, int>{0, 0} &&
						std::none_of(window_life_columns.begin(),
								window_life_columns.end(),
								[&](const WindowLifeColumn &column) {
									return column.x == bx && column.z == bz;
								}))
					window_life_columns.push_back({bx, bz, outward.first, outward.second,
							static_cast<int>(node_index - 1), bay_slot.bay, bay_slot.k,
							bay_slot.width, facade_plan.is_street(bx, bz)});
				if (terrain_enabled && min_level_offset == 0 && !is_passage) {
					const int local_ground_level =
							editor->terrain_level(bx, bz).value_or(args.ground_level);
					for (int y = local_ground_level; y <= start_y_offset; ++y) {
						const Block foundation = apply_block_variety(wall_block, bx, y,
								bz, wall_block, start_y_offset, element.id, window_block,
								condition, wall_variety);
						editor->set_block_absolute(
								foundation, bx, y + abs_terrain_offset, bz);
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
					const int window_col = bay_slot.is_window() ? bay_slot.k : 1000;
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
							chosen = !above_floor || (has_lobby_base && h <= ground_top &&
															 floor_row == 0)
											 ? wall_block
									 : floor_row == 0 ? accent_block
													  : window_block;
						else if (category == BuildingCategory::Tower) {
							const bool slit = above_floor &&
											  (floor_row == 1 || floor_row == 2) &&
											  ((wall_u + window_phase) % 4 + 4) % 4 == 1;
							chosen = slit ? window_block : wall_block;
						} else if (category == BuildingCategory::GridSkyscraper) {
							const bool mullion =
									!above_floor || floor_row == 0 || facade_corner ||
									((wall_u + window_phase) % 5 + 5) % 5 == 0;
							chosen = mullion ? wall_block : window_block;
						} else if (is_tall_building && use_vertical_windows) {
							chosen = above_floor && ((wall_u + window_phase) % 2 + 2) %
																	 2 ==
															 0
											 ? window_block
											 : wall_block;
						} else if (glazed_ground_floor && facade_plan.is_street(bx, bz) &&
								   above_floor && h <= ground_top && floor_row != 0 &&
								   window_col < 4) {
							chosen = GLASS;
						} else if (attic_style && role == FloorRole::Top) {
							chosen = above_floor && bay_slot.is_window() &&
													 floor_row == floor_cycle - 1
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
													: bay_slot.is_window() &&
															  archetype_allows_window(
																	  window_archetype,
																	  window_col,
																	  floor_row,
																	  floor_cycle));
							const bool nobile_storey = piano_nobile && h > ground_top &&
													   h <= ground_top + floor_cycle;
							const int european_row_cap = nobile_storey ? 3 : 2;
							if (european_punched_window_cap &&
									floor_row > european_row_cap)
								window = false;
							if (top_treatment && role == FloorRole::Top &&
									floor_cycle >= 3 && floor_row >= floor_cycle - 1)
								window = false;
							if (piano_nobile && role == FloorRole::Body &&
									h <= ground_top + floor_cycle && above_floor &&
									floor_row != 0 && bay_slot.is_window() &&
									window_col < 3)
								window = true;
							const bool ribbon_spandrel =
									above_floor &&
									window_archetype == WindowArchetype::Ribbon &&
									bay_slot.is_window() && floor_row < 2;
							const bool accent =
									ribbon_spandrel ||
									(above_floor && floor_row == 0 &&
											(use_accent_lines ||
													(use_vertical_accent &&
															window_col < 3) ||
													((window_archetype ==
																	 WindowArchetype::
																			 Piers ||
															 window_archetype ==
																	 WindowArchetype::
																			 Strips) &&
															bay_slot.is_window())));
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
					if (glazed_ground_floor && facade_plan.is_street(bx, bz) &&
							!party_wall && h <= ground_top) {
						const int pos = static_cast<int>(point_index);
						const bool end_pier = pos < shop_bays.lead ||
											  pos >= shop_bays.len - shop_bays.lead;
						if (end_pier) {
							chosen = has_base_course && h <= start_y_offset +
																		base_course_rows
											 ? base_course_block
											 : wall_block;
						} else if (h == ground_top || !shop_slot.is_window()) {
							chosen = shop_frame;
						} else {
							chosen = GLASS;
						}
					}
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
					if (photo_shell.contains({bx, bz}) || preset_shell.contains({bx, bz}))
						chosen = wall_block;
					// Rust resolves facade texture/color first, then same-tone patches,
					// followed by the separate building-condition weathering pass.
					chosen = apply_block_variety(chosen, bx, h, bz, wall_block,
							start_y_offset, element.id, window_block, condition,
							wall_variety);
					chosen = apply_condition_variation(chosen, bx, h, bz, wall_block,
							window_block, has_windows, condition, category, era,
							element.id);
					const int inset_x = bx - outward.first;
					const int inset_z = bz - outward.second;
					const bool recessed_spandrel =
							(window_archetype == WindowArchetype::Piers ||
									window_archetype == WindowArchetype::Strips) &&
							chosen == accent_block && h > ground_top &&
							h < start_y_offset + building_height && bay_slot.is_window();
					const bool recessed_window =
							has_windows && !args.interior &&
							condition == BuildingCondition::Normal && !is_tall_building &&
							!party_wall && !is_passage && !is_flat_column(bx, bz) &&
							(!facade_plan.is_door(bx, bz) || h > ground_top) &&
							outward != std::pair<int, int>{0, 0} &&
							(is_glass_wall_block(chosen) || recessed_spandrel) &&
							current_footprint.contains({inset_x, inset_z});
					if (recessed_window) {
						editor->set_block_absolute(
								chosen, inset_x, h + abs_terrain_offset, inset_z);
					} else {
						editor->set_block_absolute(
								chosen, bx, h + abs_terrain_offset, bz);
					}
					if (chosen == window_block &&
							window_archetype == WindowArchetype::Hooded &&
							condition == BuildingCondition::Normal &&
							outward != std::pair<int, int>{0, 0} && above_floor &&
							floor_row == floor_cycle - 1 && bay_slot.is_window() &&
							window_col != (bay_slot.width - 1) / 2 &&
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
					const bool residential = category == BuildingCategory::House ||
											 category == BuildingCategory::Residential;
					if (residential && !is_tall_building && has_windows &&
							condition == BuildingCondition::Normal && !is_passage &&
							!party_wall && !is_flat_column(bx, bz) &&
							outward != std::pair<int, int>{0, 0} && floor_row == 1 &&
							bay_slot.is_window() &&
							archetype_allows_window(
									window_archetype, window_col, 1, floor_cycle) &&
							!window_frame) {
						const int sill_y = h - 1;
						const int sill_max = start_y_offset + building_height -
											 (floor_cycle - 1) -
											 (attic_style ? floor_cycle : 0);
						if (sill_y <= sill_max) {
							const int centre_u =
									wall_u +
									((bay_slot.width - 1) / 2 - window_col) * wall_step;
							const int floor_idx = sill_y / floor_cycle;
							const int seg_cross = segment_is_x ? prev.second : prev.first;
							const auto decoration_roll =
									coord_rng(centre_u + floor_idx * 3,
											seg_cross + floor_idx * 5, element.id)
											.uniform(100);
							bool wants_balcony = false;
							if (bay_slot.is_window() &&
									window_col == (bay_slot.width - 1) / 2 &&
									(min_level_offset != 0 ||
											sill_y >= start_y_offset + 2 + floor_cycle)) {
								const int tangent_x = -outward.second;
								const int tangent_z = outward.first;
								bool balcony_fits = true;
								for (int offset = -2; offset <= 2; ++offset) {
									const int cx = bx + tangent_x * offset;
									const int cz = bz + tangent_z * offset;
									const bool on_segment =
											std::any_of(bresenham_points.begin(),
													bresenham_points.end(),
													[&](const auto &point) {
														return std::get<0>(point) == cx &&
															   std::get<2>(point) == cz;
													});
									balcony_fits =
											balcony_fits && on_segment &&
											!facade_plan.is_party(cx, cz) &&
											!passage_at(effective_passages, cx, cz);
								}
								wants_balcony =
										balcony_fits &&
										((balcony_band == BalconyBand::Scattered &&
												 decoration_roll >= 15 &&
												 decoration_roll <
														 (rear_facade ? 18u : 23u)) ||
												(balcony_band == BalconyBand::EveryBay &&
														facade_plan.is_street(bx, bz)) ||
												(balcony_band == BalconyBand::
																		 Alternating &&
														facade_plan.is_street(bx, bz) &&
														((floor_div(wall_u + window_phase,
																  6) % 2 +
																 2) % 2 ==
																0)));
							}
							const std::uint32_t sill_roll_max = rear_facade ? 8u : 15u;
							if (!wants_balcony && decoration_roll < sill_roll_max) {
								const auto sill =
										cached_prop_block(sill_base, {{"type", "top"}});
								editor->set_block_with_properties_absolute(sill,
										bx + outward.first, sill_y + abs_terrain_offset,
										bz + outward.second,
										std::optional<std::vector<Block>>{{AIR}},
										std::nullopt);
								auto pot_rng = coord_rng(bx, bz + floor_idx, element.id);
								const auto pot_chance =
										window_col == (bay_slot.width - 1) / 2
												? (rear_facade ? 35u : 70u)
												: (rear_facade ? 12u : 25u);
								if (pot_rng.uniform(100) < pot_chance) {
									const Block plant = potted_plant_options
											[pot_rng.uniform(static_cast<std::uint32_t>(
													potted_plant_options.size()))];
									editor->set_block_absolute(plant, bx + outward.first,
											sill_y + 1 + abs_terrain_offset,
											bz + outward.second, std::vector<Block>{AIR});
								}
							}
						}
					}
					const bool shutter_flank =
							bay_slot.is_window() &&
							(window_col == 0 || window_col == bay_slot.width - 1);
					const bool shutter_archetype =
							window_archetype == WindowArchetype::Sash ||
							window_archetype == WindowArchetype::Portrait ||
							window_archetype == WindowArchetype::Spaced ||
							window_archetype == WindowArchetype::Hooded;
					if (residential && !is_tall_building && has_windows &&
							shutter_flank && !window_frame && !wall_depth_piers_active &&
							shutter_archetype && h > start_y_offset + 1 &&
							floor_row != 0 &&
							(!facade_plan.is_door(bx, bz) || h > start_y_offset + 3) &&
							outward != std::pair<int, int>{0, 0} && !is_passage &&
							!party_wall && !is_flat_column(bx, bz)) {
						const int centre_u =
								wall_u +
								((bay_slot.width - 1) / 2 - window_col) * wall_step;
						const int seg_cross = segment_is_x ? prev.second : prev.first;
						const auto roll =
								coord_rng(centre_u, seg_cross, element.id).uniform(100);
						const std::uint32_t shutter_chance =
								detail == DetailTier::Minimal	 ? 0
								: detail == DetailTier::Standard ? 25
																 : 40;
						if (roll < shutter_chance) {
							const char *facing =
									std::abs(outward.first) >= std::abs(outward.second)
											? (outward.first >= 0 ? "east" : "west")
											: (outward.second >= 0 ? "south" : "north");
							const auto shutter = cached_prop_block(
									shutter_base, {{"facing", facing}, {"open", "true"},
														  {"half", "top"}});
							editor->set_block_with_properties_absolute(shutter,
									bx + outward.first, h + abs_terrain_offset,
									bz + outward.second,
									std::optional<std::vector<Block>>{{AIR}},
									std::nullopt);
						}
					}
					if (window_frame && outward != std::pair<int, int>{0, 0} &&
							!is_passage && !party_wall && depth_clear &&
							!facade_plan.is_door(bx, bz) &&
							condition == BuildingCondition::Normal &&
							!horizontal_windows && category != BuildingCategory::Tower) {
						const bool head_band =
								floor_row == 0 && bay_slot.is_window() && above_floor;
						const bool flank =
								bay_slot.is_window() &&
								(window_col == 0 || window_col == bay_slot.width - 1) &&
								floor_row > 0 && floor_row < floor_cycle - 1;
						if (head_band || flank)
							editor->set_block_absolute(
									window_frame_material(*window_frame),
									bx + outward.first, h + abs_terrain_offset,
									bz + outward.second, std::vector<Block>{AIR});
					}
					if (outward != std::pair<int, int>{0, 0} && !is_passage &&
							!party_wall && condition == BuildingCondition::Normal) {
						const int ox = bx + outward.first, oz = bz + outward.second;
						const bool street_facing = facade_plan.is_street(bx, bz);
						if (street_facing && storefront && awning_enabled &&
								h == ground_top && shop_slot.is_window() &&
								!is_flat_column(bx, bz)) {
							const char *facing =
									std::abs(outward.first) >= std::abs(outward.second)
											? (outward.first >= 0 ? "east" : "west")
											: (outward.second >= 0 ? "south" : "north");
							const auto awning = cached_prop_block(
									awning_block, {{"facing", facing}, {"open", "false"},
														  {"half", "top"}});
							editor->set_block_with_properties_absolute(awning, ox,
									h - 1 + abs_terrain_offset, oz,
									std::vector<Block>{AIR}, std::nullopt);
						}
						if (depth_clear && floor_row == 0 && bay_slot.is_window() &&
								window_col == (bay_slot.width - 1) / 2 &&
								h >= start_y_offset + 2 + floor_cycle &&
								h <= start_y_offset + building_height - floor_cycle) {
							const int floor_idx = h / floor_cycle;
							const int seg_cross = segment_is_x ? prev.second : prev.first;
							auto decoration_rng = coord_rng(wall_u + floor_idx * 3,
									seg_cross + floor_idx * 5, element.id);
							const auto decoration_roll = decoration_rng.uniform(100);
							bool balcony_fits = true;
							const int tangent_x = -outward.second;
							const int tangent_z = outward.first;
							for (int t = -2; t <= 2; ++t) {
								const int wx = bx + tangent_x * t;
								const int wz = bz + tangent_z * t;
								const bool on_segment = std::any_of(
										bresenham_points.begin(), bresenham_points.end(),
										[&](const auto &point) {
											return std::get<0>(point) == wx &&
												   std::get<2>(point) == wz;
										});
								balcony_fits = balcony_fits && on_segment &&
											   !facade_plan.is_party(wx, wz) &&
											   !passage_at(effective_passages, wx, wz);
							}
							const bool above_ground =
									min_level_offset != 0 ||
									h >= start_y_offset + 2 + floor_cycle;
							const bool residential =
									category == BuildingCategory::House ||
									category == BuildingCategory::Residential;
							const bool balcony =
									residential && has_windows && !is_tall_building &&
									above_ground && balcony_fits &&
									((balcony_band == BalconyBand::EveryBay &&
											 street_facing) ||
											(balcony_band == BalconyBand::Alternating &&
													street_facing &&
													((floor_div(wall_u + window_phase,
															  6) % 2 +
															 2) % 2 ==
															0)) ||
											(balcony_band == BalconyBand::Scattered &&
													decoration_roll >= 15 &&
													decoration_roll <
															(street_facing ? 23u : 18u)));
							if (balcony) {
								// Port Rust's full 3-wide by 2-deep balcony rather than a
								// lone projecting slab. The centre column is the bay anchor;
								// require it to stay on this segment and clear of party walls
								// and passages so the balcony cannot wrap a corner.
								const int balcony_y = h + abs_terrain_offset;
								const auto facing = [](int nx, int nz) {
									if (std::abs(nx) >= std::abs(nz))
										return nx >= 0 ? "east" : "west";
									return nz >= 0 ? "south" : "north";
								};
								const auto open_trapdoor = [&](Block block, int nx,
																   int nz) {
									return cached_prop_block(block,
											{{"facing", facing(nx, nz)}, {"open", "true"},
													{"half", "top"}});
								};
								static const std::array<Block, 4> trapdoor_options{
										OAK_TRAPDOOR, DARK_OAK_TRAPDOOR, SPRUCE_TRAPDOOR,
										BIRCH_TRAPDOOR};
								const Block trapdoor = trapdoor_options
										[element_rng(element.id)
														.uniform(static_cast<
																std::uint32_t>(
																trapdoor_options
																		.size()))];
								const auto balcony_floor = cached_prop_block(
										SMOOTH_STONE_SLAB, {{"type", "top"}});
								const auto front_fence = open_trapdoor(
										trapdoor, outward.first, outward.second);
								const auto left_fence =
										open_trapdoor(trapdoor, -tangent_x, -tangent_z);
								const auto right_fence =
										open_trapdoor(trapdoor, tangent_x, tangent_z);
								const std::optional<std::vector<Block>> air{
										std::vector<Block>{AIR}};
								for (int t = -1; t <= 1; ++t) {
									for (int depth = 1; depth <= 2; ++depth)
										editor->set_block_with_properties_absolute(
												balcony_floor,
												bx + tangent_x * t +
														outward.first * depth,
												balcony_y,
												bz + tangent_z * t +
														outward.second * depth,
												air, std::nullopt);
									const int fx = bx + tangent_x * t + outward.first * 3;
									const int fz =
											bz + tangent_z * t + outward.second * 3;
									editor->set_block_with_properties_absolute(
											front_fence, fx, balcony_y + 1, fz, air,
											std::nullopt);
								}
								for (int depth = 1; depth <= 2; ++depth) {
									const int lx =
											bx - tangent_x * 2 + outward.first * depth;
									const int lz =
											bz - tangent_z * 2 + outward.second * depth;
									const int rx =
											bx + tangent_x * 2 + outward.first * depth;
									const int rz =
											bz + tangent_z * 2 + outward.second * depth;
									editor->set_block_with_properties_absolute(left_fence,
											lx, balcony_y + 1, lz, air, std::nullopt);
									editor->set_block_with_properties_absolute(
											right_fence, rx, balcony_y + 1, rz, air,
											std::nullopt);
								}
								// Stable per-bay furnishing: a planter or an outward-facing
								// stair chair, matching Rust's two common balcony details.
								auto furniture_rng = coord_rng(bx + floor_idx * 11,
										bz + floor_idx * 17, element.id);
								const auto furniture_roll = furniture_rng.uniform(100);
								const int side = furniture_rng.random_bool(.5) ? -1 : 1;
								const int px = bx + tangent_x * side + outward.first;
								const int pz = bz + tangent_z * side + outward.second;
								if (furniture_roll < 30) {
									editor->set_block_absolute(CAULDRON, px,
											balcony_y + 1, pz, air, std::nullopt);
									editor->set_block_absolute(OAK_LEAVES, px,
											balcony_y + 2, pz, air, std::nullopt);
								} else if (furniture_roll < 55) {
									const auto chair_facing =
											facing(-outward.first, -outward.second);
									const auto stair_facing =
											std::string_view(chair_facing) == "north"
													? StairFacing::North
											: std::string_view(chair_facing) == "south"
													? StairFacing::South
											: std::string_view(chair_facing) == "east"
													? StairFacing::East
													: StairFacing::West;
									editor->set_block_with_properties_absolute(
											create_stair_with_properties(OAK_STAIRS,
													stair_facing, StairShape::Straight),
											px, balcony_y + 1, pz, air, std::nullopt);
								}
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

	// Rust's window-life pass decorates only recessed panes and keys each
	// decision to the complete window bay/storey, so all columns in one window
	// share the same curtain and air-conditioner choice.
	const bool window_life_dwelling =
			category == BuildingCategory::House ||
			category == BuildingCategory::Residential ||
			category == BuildingCategory::Hotel ||
			(category == BuildingCategory::Default &&
					(element.tags.get("building:use") == "residential" ||
							element.tags.get("building:use") == "apartments"));
	const auto window_region = building_style::building_region(wall_region);
	// Rust's window-life pass is limited to punched facades. Do not place
	// curtains, window boxes, or air conditioners across horizontal bands or
	// tower curtain grids even when their wall walk has a window bay lattice.
	const bool punched_windows = has_windows && !horizontal_windows &&
								 category != BuildingCategory::Tower &&
								 category != BuildingCategory::GridSkyscraper;
	const bool walkup_air_conditioners =
			punched_windows &&
			window_region == building_style::BuildingRegion::NorthAmerica &&
			!args.interior && detail != DetailTier::Minimal &&
			facade_plan.has_any_street &&
			(era == ArchEra::Unknown || era == ArchEra::TraditionalPreWar) &&
			((building_height - grammar_anchor) / std::max(1, floor_cycle) >= 4) &&
			((building_height - grammar_anchor) / std::max(1, floor_cycle) <= 8) &&
			(category == BuildingCategory::Residential ||
					category == BuildingCategory::Default) &&
			window_life_dwelling;
	const bool window_boxes =
			punched_windows && window_region == building_style::BuildingRegion::Europe &&
			detail != DetailTier::Minimal &&
			(era == ArchEra::TraditionalPreWar || era == ArchEra::HistoricOrnate) &&
			category != BuildingCategory::Hotel && window_life_dwelling;
	if (window_surrounds && !facade_window_columns.empty()) {
		std::unordered_map<std::pair<int, int>, const FacadeWindowColumn *, PairHash>
				columns_by_position;
		for (const auto &column : facade_window_columns)
			columns_by_position.emplace(std::pair{column.run, column.position}, &column);
		const auto glazed = [&](const FacadeWindowColumn &column, int h) {
			const auto pane = editor->get_block_absolute(
					column.x - column.nx, h + abs_terrain_offset, column.z - column.nz);
			const auto face = editor->get_block_absolute(
					column.x, h + abs_terrain_offset, column.z);
			return pane && (*pane == window_block || *pane == GLASS) &&
				   (!face || *face == AIR);
		};
		const int ground_top = start_y_offset + grammar_anchor + floor_cycle - 1;
		const int top_h = start_y_offset + building_height;
		for (const auto &column : facade_window_columns) {
			if (passage_at(effective_passages, column.x, column.z) ||
					facade_plan.is_party(column.x, column.z))
				continue;
			std::array<const FacadeWindowColumn *, 2> neighbours{};
			std::size_t neighbour_count = 0;
			if (column.slot.is_window()) {
				neighbours[neighbour_count++] = &column;
			} else {
				for (const int offset : {-1, 1}) {
					const auto it = columns_by_position.find(
							{column.run, column.position + offset});
					if (it != columns_by_position.end() && it->second->slot.is_window())
						neighbours[neighbour_count++] = it->second;
				}
			}
			for (int h = ground_top + 2; h < top_h; ++h) {
				bool framed = false;
				for (std::size_t i = 0; i < neighbour_count && !framed; ++i) {
					const auto &window = *neighbours[i];
					const bool head = glazed(window, h - 1) && !glazed(window, h);
					const bool jamb = !column.slot.is_window() &&
									  (glazed(window, h) ||
											  ((((h - start_y_offset - grammar_anchor) %
																floor_cycle +
														floor_cycle) %
													   floor_cycle) == 0 &&
													  glazed(window, h + 1)));
					framed = head || jamb;
				}
				if (framed)
					editor->set_block_absolute(
							trim_block_for_wall(wall_block, group_seed), column.x,
							h + abs_terrain_offset, column.z, std::nullopt,
							std::vector<Block>{window_block, GLASS});
			}
		}
	}
	// Rust only hangs curtains in rooms owned by this complete building. A
	// building:part can share its volume with later sibling walls, and a grouped
	// outline with sibling cells does not own the floor behind its facade.
	// Recessed panes are already required by glass_at() below.
	const bool window_curtains = !args.interior &&
								 !element.tags.contains("building:part") &&
								 sibling_cells.empty();
	if (condition == BuildingCondition::Normal && punched_windows && !is_tall_building &&
			window_life_dwelling && (!window_life_columns.empty())) {
		const int ground_top = start_y_offset + grammar_anchor + floor_cycle - 1;
		const int top_h = start_y_offset + building_height;
		const Block sill_block = sill_base;
		const std::unordered_set<std::pair<int, int>, PairHash> own_wall_ring(
				current_building.begin(), current_building.end());
		for (const auto &wc : window_life_columns) {
			if (facade_plan.is_door(wc.x, wc.z) ||
					passage_at(effective_passages, wc.x, wc.z))
				continue;
			const int lx = wc.x + wc.nx, lz = wc.z + wc.nz;
			const int fx = wc.x + 2 * wc.nx, fz = wc.z + 2 * wc.nz;
			const int ix = wc.x - wc.nx, iz = wc.z - wc.nz;
			const int bx = wc.x - 2 * wc.nx, bz = wc.z - 2 * wc.nz;
			const auto at = [&](int px, int y, int pz) {
				return editor->get_block_absolute(px, y + abs_terrain_offset, pz);
			};
			const auto clear = [&](int px, int y, int pz) {
				const auto block = at(px, y, pz);
				return !block || *block == AIR;
			};
			const auto glass_at = [&](int y) {
				const auto pane = at(ix, y, iz);
				// Match Rust's window-life pass: decorate the configured pane (or
				// ordinary clear glass), not every stained-glass block that happens
				// to be present behind this facade column.
				return pane && (*pane == window_block || *pane == GLASS) &&
					   clear(wc.x, y, wc.z);
			};
			const auto room_behind = [&]() {
				if (!current_footprint.contains({bx, bz}) ||
						own_wall_ring.contains({bx, bz}))
					return false;
				if (!building_footprints)
					return true;
				for (int dx = -2; dx <= 2; ++dx)
					for (int dz = -2; dz <= 2; ++dz) {
						const int px = bx + dx, pz = bz + dz;
						if (building_footprints->contains(px, pz) &&
								!current_footprint.contains({px, pz}) &&
								!own_wall_ring.contains({px, pz}))
							return false;
					}
				return true;
			};
			for (int h = start_y_offset + 1; h <= top_h; ++h) {
				if (h <= ground_top && glazed_ground_floor && wc.street)
					continue;
				if (!glass_at(h))
					continue;
				const int storey = floor_div(
						h - start_y_offset - grammar_anchor, std::max(1, floor_cycle));
				const bool lowest = !glass_at(h - 1);
				const bool top_row = !glass_at(h + 1);
				if (walkup_air_conditioners && wc.street && lowest && h > ground_top &&
						clear(lx, h, lz) && clear(fx, h - 1, fz)) {
					auto ac_rng = coord_rng(wc.run * 4096 + wc.bay, storey,
							element.id ^ 0xA1C00D1700000003ULL);
					const int unit =
							ac_rng.uniform(100) < 10
									? static_cast<int>(ac_rng.uniform(
											  static_cast<std::uint32_t>(wc.width)))
									: -1;
					if (unit == wc.k)
						editor->set_block_absolute(SMOOTH_STONE, wc.x,
								h + abs_terrain_offset, wc.z, std::vector<Block>{AIR},
								std::nullopt);
				}
				if (window_curtains && current_footprint.contains({bx, bz}) &&
						room_behind()) {
					auto curtain_rng = coord_rng(wc.run * 4096 + wc.bay, storey,
							element.id ^ 0xC0A7A1A500000001ULL);
					if (curtain_rng.uniform(100) < 32) {
						const Block colour = curtain_rng.random_bool(.65)
													 ? WHITE_WOOL
													 : LIGHT_GRAY_WOOL;
						const int draw = curtain_rng.uniform(3);
						const bool covers =
								draw == 0 ||
								(draw == 1 &&
										(wc.width >= 3			? (wc.k == 0 ||
																  wc.k == wc.width - 1)
												: wc.width == 2 ? wc.k == 0
																: top_row)) ||
								(draw == 2 && top_row);
						if (covers)
							editor->set_block_absolute(colour, bx, h + abs_terrain_offset,
									bz, std::vector<Block>{AIR}, std::nullopt);
					}
				}
			}
			if (window_boxes) {
				for (int h = ground_top + 1; h < top_h; ++h) {
					if (((h - start_y_offset - grammar_anchor) % floor_cycle +
								floor_cycle) %
											floor_cycle !=
									0 ||
							!glass_at(h + 1))
						continue;
					auto box_rng = coord_rng(wc.run * 4096 + wc.bay,
							floor_div(h - start_y_offset - grammar_anchor,
									std::max(1, floor_cycle)),
							element.id ^ 0xF10B0C5E00000002ULL);
					if (box_rng.uniform(100) >= 10 || at(lx, h, lz) != sill_block ||
							!clear(lx, h + 1, lz) || !clear(fx, h, fz) ||
							!clear(fx, h + 1, fz))
						continue;
					auto plant_rng =
							coord_rng(wc.x, wc.z + h * 13, element.id ^ 0xF10BULL);
					editor->set_block_absolute(plant_rng.random_bool(.6)
													   ? FLOWERING_AZALEA_LEAVES
													   : AZALEA_LEAVES,
							lx, h + 1 + abs_terrain_offset, lz, std::vector<Block>{AIR},
							std::nullopt);
				}
			}
		}
	}

	// Rust builds the roof over both the filled footprint and the wall ring so
	// roof blocks reach the outside face of the generated walls. Keep the area
	// sorted to make every roof feature's iteration order deterministic.
	std::vector<std::pair<int, int>> roof_area = cached_floor_area;
	roof_area.insert(roof_area.end(), current_building.begin(), current_building.end());
	std::sort(roof_area.begin(), roof_area.end());
	roof_area.erase(std::unique(roof_area.begin(), roof_area.end()), roof_area.end());
	// Keep pitched-roof eaves out of adjacent buildings and sibling parts, as in
	// Rust's RoofTrim::blocked set. Include the generated wall ring in the roof
	// footprint, but distinguish the filled building interior when collecting
	// neighboring occupancy.
	std::unordered_set<std::pair<int, int>, PairHash> eave_blocked;
	if (building_footprints && !roof_area.empty()) {
		const std::unordered_set<std::pair<int, int>, PairHash> own_roof(
				roof_area.begin(), roof_area.end());
		const std::unordered_set<std::pair<int, int>, PairHash> own_fill(
				cached_floor_area.begin(), cached_floor_area.end());
		for (const auto &[x, z] : roof_area) {
			const bool exposed =
					!own_roof.contains({x - 1, z}) || !own_roof.contains({x + 1, z}) ||
					!own_roof.contains({x, z - 1}) || !own_roof.contains({x, z + 1});
			if (!exposed)
				continue;
			for (int dx = -2; dx <= 2; ++dx)
				for (int dz = -2; dz <= 2; ++dz) {
					const std::pair<int, int> cell{x + dx, z + dz};
					if (own_roof.contains(cell) || own_fill.contains(cell))
						continue;
					if (building_footprints->contains(cell.first, cell.second) ||
							sibling_cells.contains(cell))
						eave_blocked.insert(cell);
				}
		}
	}
	const bool tone_patches_enabled = condition == BuildingCondition::Normal &&
									  !element.tags.contains("roof:colour") &&
									  !element.tags.contains("roof:material");

	std::optional<building_facade::FacadeAnchor> signage_anchor;
	std::vector<interior_uses::Entry> interior_entries;
	// Rust keeps the selected door species and orientation as block-state
	// properties. Preserve those states in C++ too; the backend can then map
	// them to the target game's native door representation.
	const auto entrance_door_style = [&](bool lobby_entrance) {
		if (lobby_entrance && ground_floor == GroundFloorKind::Lobby)
			return std::pair{IRON_DOOR, IRON_TRAPDOOR};
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
		const auto lower = cached_prop_block(
				base, {{"half", "lower"}, {"facing", facing}, {"hinge", hinge}});
		const auto upper = cached_prop_block(
				base, {{"half", "upper"}, {"facing", facing}, {"hinge", hinge}});
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
	if (min_level_offset == 0 && condition != BuildingCondition::Construction &&
			condition != BuildingCondition::Ruined &&
			category != BuildingCategory::Greenhouse &&
			category != BuildingCategory::Shed && category != BuildingCategory::Garage) {
		std::unordered_set<std::pair<int, int>, PairHash> rendered_entrances;
		bool mapped_entrance_rendered = false;
		for (std::size_t node_index = 0; node_index < element.nodes.size();
				++node_index) {
			const auto &node = element.nodes[node_index];
			if ((node.tags.contains("entrance") && node.tags.get("entrance") != "no") ||
					(node.tags.contains("door") && node.tags.get("door") != "no")) {
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
				const auto [door_block, unused_trapdoor] = entrance_door_style(false);
				(void)unused_trapdoor;
				if (entrance_façade) {
					const auto inward = std::pair{-entrance_façade->normal.first,
							-entrance_façade->normal.second};
					facade_plan.mark_door_column(node.x + entrance_façade->normal.first,
							node.z + entrance_façade->normal.second);
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
						facade_plan.mark_door_column(
								node.x + tangent.first + entrance_façade->normal.first,
								node.z + tangent.second + entrance_façade->normal.second);
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
					mapped_entrance_rendered = true;
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
						facade_plan.mark_door_column(
								node.x - inward->first, node.z - inward->second);
						put_entrance_door(door_block, node.x, node.z, -inward->first,
								-inward->second, "left");
						interior_entries.push_back(
								{{node.x + inward->first, node.z + inward->second},
										*inward});
						mapped_entrance_rendered = true;
					}
				}
			}
		}
		if (!mapped_entrance_rendered) {
			if (synthetic_entrance_plan) {
				const auto &plan = *synthetic_entrance_plan;
				const auto &segment = *facade_plan.segments[plan.segment];
				const std::optional<std::pair<int, int>> door = std::pair{plan.x, plan.z};
				const bool double_door = plan.double_door;
				if (door) {
					const auto inward =
							std::pair{-segment.normal.first, -segment.normal.second};
					interior_entries.push_back(
							{{door->first + inward.first, door->second + inward.second},
									inward});
					if (!signage_anchor)
						signage_anchor = building_facade::FacadeAnchor{
								door->first, door->second, segment.normal, 0, 0, *door};
					const auto [door_block, trapdoor_block] = entrance_door_style(true);
					facade_plan.mark_door_column(door->first, door->second);
					facade_plan.mark_door_column(door->first + segment.normal.first,
							door->second + segment.normal.second);
					put_entrance_door(door_block, door->first, door->second,
							segment.normal.first, segment.normal.second, "left");
					if (double_door &&
							!passage_at(effective_passages,
									door->first + segment.tangent.first,
									door->second + segment.tangent.second) &&
							!facade_plan.is_party(door->first + segment.tangent.first,
									door->second + segment.tangent.second)) {
						put_entrance_door(door_block, door->first + segment.tangent.first,
								door->second + segment.tangent.second,
								segment.normal.first, segment.normal.second, "right");
						facade_plan.mark_door_column(door->first + segment.tangent.first,
								door->second + segment.tangent.second);
						facade_plan.mark_door_column(door->first + segment.tangent.first +
															 segment.normal.first,
								door->second + segment.tangent.second +
										segment.normal.second);
						put_threshold(door->first + segment.tangent.first,
								door->second + segment.tangent.second,
								segment.normal.first, segment.normal.second);
					}
					auto dress_rng = element_rng(group_seed ^ 0xD00E57E900000013ULL);
					const bool canopy =
							(double_door || category == BuildingCategory::Commercial ||
									category == BuildingCategory::Hotel) &&
							ground_floor != GroundFloorKind::Shopfront &&
							dress_rng.random_bool(.6);
					const bool lantern =
							!canopy &&
							(category == BuildingCategory::House ||
									category == BuildingCategory::Residential ||
									category == BuildingCategory::Historic) &&
							dress_rng.random_bool(.35);
					const auto set_trapdoor = [&, trapdoor_block = trapdoor_block](
													  int x, int y, int z) {
						editor->set_block_with_properties_absolute(
								cached_prop_block(trapdoor_block,
										{{"half", "top"}, {"open", "false"},
												{"facing",
														facing_name(segment.normal.first,
																segment.normal.second)}}),
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
						set_trapdoor(lantern_x, start_y_offset + abs_terrain_offset + 4,
								lantern_z);
						const auto hanging_lantern =
								cached_prop_block(LANTERN, {{"hanging", "true"}});
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
						photo_shell.contains({x1, z1}) || photo_shell.contains({x2, z2}))
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
			if (!passage_at(effective_passages, x, z) && !photo_shell.contains({x, z})) {
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
			const auto [door_block, unused_trapdoor] = entrance_door_style(false);
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
				const bool hidden_lights =
						!interior_plan && condition != BuildingCondition::Abandoned &&
						condition != BuildingCondition::Ruined &&
						editor->get_generation_format() !=
								static_cast<int>(WorldFormat::LuantiWorld);
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
						const bool lamp = (x % 3) == 0 && (z % 3) == 0;
						const Block block = lamp && !hidden_lights ? ceiling_light_block
																   : floor_block;
						editor->set_block_absolute(block, x, h + abs_terrain_offset, z);
						if (lamp && hidden_lights)
							editor->set_block_absolute(
									LIGHT, x, h - 1 + abs_terrain_offset, z);
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
							hidden_lights ? LIGHT : ceiling_light_block, x,
							top + abs_terrain_offset, z);

				if (is_passage && building_height > BUILDING_PASSAGE_HEIGHT)
					editor->set_block_absolute(floor_block, x,
							start_y_offset + BUILDING_PASSAGE_HEIGHT + abs_terrain_offset,
							z);
			}
		}

		if (!element.tags.contains("building:part"))
			generate_corner_downpipes(*editor, element, facade_plan, category, condition,
					has_windows, min_level_offset == 0, start_y_offset, building_height,
					abs_terrain_offset, wall_block, effective_passages, &preset_shell,
					&photo_shell);
		if (!element.tags.contains("building:part"))
			generate_corner_quoins(*editor, element, facade_plan, category, condition,
					wall_block, accent_block, start_y_offset, building_height,
					abs_terrain_offset, effective_passages, element.id, &preset_shell,
					&photo_shell);
		if (interior_plan) {
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
					*interior_plan, is_abandoned_building, interior_passages,
					interior_entries, {{min_x, min_z}, {max_x, max_z}}, interior_claims,
					args.scale, floor_block, clean_visual_seed};
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
	bool roof_generated = false;
	RoofType generated_roof_type = RoofType::Flat;
	const bool short_roof_eave = building_style::building_region(wall_region) ==
										 building_style::BuildingRegion::Europe &&
								 category != BuildingCategory::House &&
								 building_height / std::max(1, floor_cycle) >= 3;
	const bool roof_soffit =
			condition == BuildingCondition::Normal &&
			(detail >= DetailTier::Enhanced ||
					(detail >= DetailTier::Standard &&
							building_style::building_region(wall_region) !=
									building_style::BuildingRegion::Europe));
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
					group_seed,
					roof_dormers_enabled(condition, category, roof_type, era,
							building_style::building_region(wall_region), building_height,
							floor_cycle, group_seed),
					category == BuildingCategory::Religious, preferred_ridge_along_x,
					roof_type, roof_area, abs_terrain_offset, args.scale, short_roof_eave,
					RoofStyleContext{category, era, wall_region, wall_attached,
							wall_district, window_block, window_phase, roof_soffit},
					hole_polygons, &eave_blocked, tone_patches_enabled);
			roof_generated = true;
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
					abs_terrain_offset, args.scale, short_roof_eave,
					RoofStyleContext{category, era, wall_region, wall_attached,
							wall_district, window_block, window_phase, roof_soffit},
					hole_polygons, &eave_blocked, tone_patches_enabled);
			roof_generated = true;
		} else if (roof_enclosed) {
			// Rust's RoofContext treats a footprint surrounded by neighboring
			// building cells as covered infill, not an exposed roof surface.
			generated_roof_type = RoofType::Flat;
			generated_sloped_roof = false;
		} else if (mapped_auto_roof) {
			if (pitched_roof(*mapped_auto_roof)) {
				const std::optional<Block> curtain_glass =
						is_glass_wall_block(wall_block) &&
										is_glass_wall_block(window_block)
								? std::optional<Block>(window_block)
								: std::nullopt;
				generated_roof_type = *mapped_auto_roof;
				generated_sloped_roof = true;
				generate_roof(*editor, element, start_y_offset, building_height,
						floor_block, wall_block, accent_block, roof_block_override,
						curtain_glass, group_seed,
						roof_dormers_enabled(condition, category, *mapped_auto_roof, era,
								building_style::building_region(wall_region),
								building_height, floor_cycle, group_seed),
						category == BuildingCategory::Religious, preferred_ridge_along_x,
						*mapped_auto_roof, roof_area, abs_terrain_offset, args.scale,
						short_roof_eave,
						RoofStyleContext{category, era, wall_region, wall_attached,
								wall_district, window_block, window_phase, roof_soffit},
						hole_polygons, &eave_blocked, tone_patches_enabled);
				roof_generated = true;
			}
		} else if (!part_has_explicit_top) {
			std::string btype = "yes";
			auto it = element.tags.find("building");
			if (it != element.tags.end())
				btype = it->second;

			const bool qualifies_for_auto_roof =
					btype == "apartments" || btype == "residential" || btype == "house" ||
					btype == "yes" || btype == "detached" ||
					btype == "semidetached_house" || btype == "terrace" ||
					btype == "bungalow" || btype == "villa" || btype == "cabin" ||
					btype == "hut" || btype == "farm" || btype == "farm_auxiliary" ||
					btype == "barn" || btype == "stable" || btype == "cowshed" ||
					btype == "sty" || btype == "sheepfold";
			const bool auto_roof_type = category != BuildingCategory::TallBuilding &&
										category != BuildingCategory::Hospital &&
										category != BuildingCategory::Historic &&
										qualifies_for_auto_roof;
			const auto [lat, lon] = editor->geo_anchor();
			const auto materials = building_style::material_region_at(lat, lon);
			const auto dominant_region = building_style::building_region(materials);
			const double footprint_m2 = static_cast<double>(cached_footprint_size) /
										(args.scale * args.scale);
			const double height_m = building_height / args.scale;
			const bool attached =
					btype == "terrace" || btype == "semidetached_house" ||
					std::any_of(cached_floor_area.begin(), cached_floor_area.end(),
							[&](const auto &cell) {
								return facade_plan.is_party(cell.first, cell.second);
							});
			const bool house = btype == "house" || btype == "detached" ||
							   btype == "semidetached_house" || btype == "bungalow" ||
							   btype == "villa" || btype == "cabin" || btype == "hut" ||
							   btype == "farm" || btype == "farm_auxiliary" ||
							   btype == "barn" || btype == "stable" ||
							   btype == "cowshed" || btype == "sty" ||
							   btype == "sheepfold" || btype == "terrace" ||
							   ((btype == "yes" || btype == "residential") && !attached &&
									   footprint_m2 <= 220.0 && height_m <= 11.0);
			bool regional_roof_handled = false;
			if (!element.tags.contains("building:part") && qualifies_for_auto_roof) {
				const auto habit = building_style::roof_habit(materials);
				if (habit) {
					double chance = footprint_m2 > 1500.0 ? 0.0
									: house				  ? habit->house
									: height_m <= habit->block_max_m
											? ((era == ArchEra::PostWarPanel ||
													   era == ArchEra::Contemporary)
															  ? habit->block * .5
															  : habit->block)
											: 0.0;
					if (materials != building_style::MaterialRegion::Australasia) {
						if (climate == biome::Climate::HotDesert)
							chance *= .35;
						else if (climate == biome::Climate::HotSteppe)
							chance *= .7;
					}
					auto regional_rng = element_rng(group_seed ^ 0x2E610A1F0F5E7001ULL);
					regional_roof_handled = true;
					if (regional_rng.random_bool(chance)) {
						const RoofType regional_roof =
								attached || !regional_rng.random_bool(habit->hipped)
										? RoofType::Gabled
										: RoofType::Hipped;
						generated_roof_type = regional_roof;
						generated_sloped_roof = true;
						const std::optional<Block> curtain_glass =
								is_glass_wall_block(wall_block) &&
												is_glass_wall_block(window_block)
										? std::optional<Block>(window_block)
										: std::nullopt;
						generate_roof(*editor, element, start_y_offset, building_height,
								floor_block, wall_block, accent_block,
								roof_block_override, curtain_glass, group_seed,
								roof_dormers_enabled(condition, category, regional_roof,
										era, building_style::building_region(wall_region),
										building_height, floor_cycle, group_seed),
								category == BuildingCategory::Religious,
								preferred_ridge_along_x, regional_roof, roof_area,
								abs_terrain_offset, args.scale, short_roof_eave,
								RoofStyleContext{category, era, wall_region,
										wall_attached, wall_district, window_block,
										window_phase, roof_soffit},
								hole_polygons, &eave_blocked, tone_patches_enabled);
						roof_generated = true;
					}
				}
			}
			if (!regional_roof_handled &&
					dominant_region == building_style::BuildingRegion::Europe &&
					!building_style::roof_habit(materials) &&
					!element.tags.contains("building:part")) {
				const bool civic_or_commercial =
						btype == "commercial" || btype == "retail" || btype == "office" ||
						btype == "hotel" || btype == "public" || btype == "civic" ||
						btype == "government" || btype == "school" ||
						btype == "university" || btype == "college" ||
						btype == "kindergarten" || btype == "townhall" ||
						btype == "historic" || btype == "palace" || btype == "castle" ||
						btype == "museum" || btype == "theatre";
				const bool urban_type = btype == "apartments" || btype == "residential" ||
										btype == "yes" || btype == "terrace" ||
										civic_or_commercial;
				const bool eligible_type = urban_type || qualifies_for_auto_roof;
				// Rust leaves unknown-era detached houses to the general roof
				// rules instead of consuming them in the European fabric rule.
				if (eligible_type && (era != ArchEra::Unknown || urban_type) &&
						height_m <= 26.0 && footprint_m2 <= 3000.0) {
					double bbox_cells = 1.0;
					if (!element.nodes.empty()) {
						int min_x = element.nodes.front().x;
						int max_x = min_x;
						int min_z = element.nodes.front().z;
						int max_z = min_z;
						for (const auto &node : element.nodes) {
							min_x = std::min(min_x, node.x);
							max_x = std::max(max_x, node.x);
							min_z = std::min(min_z, node.z);
							max_z = std::max(max_z, node.z);
						}
						bbox_cells = std::max(
								1.0, static_cast<double>(max_x - min_x + 1) *
											 (static_cast<double>(max_z - min_z + 1)));
					}
					if (static_cast<double>(cached_footprint_size) / bbox_cells >= .5) {
						const bool commercial = civic_or_commercial &&
												btype != "historic" &&
												btype != "palace" && btype != "castle" &&
												btype != "townhall";
						double chance = 0.0;
						switch (era) {
						case ArchEra::HistoricOrnate:
							chance = .95;
							break;
						case ArchEra::TraditionalPreWar:
							chance = .88;
							break;
						case ArchEra::PostWarPanel:
							chance = commercial ? .10 : .40;
							break;
						case ArchEra::Contemporary:
							chance = commercial ? .05 : .25;
							break;
						case ArchEra::Unknown:
							chance = attached ? .80 : (footprint_m2 > 1500.0 ? .40 : .65);
							break;
						}
						auto european_rng =
								element_rng(group_seed ^ 0x5E0FEA570C1D70B5ULL);
						regional_roof_handled = true;
						if (european_rng.random_bool(chance)) {
							const bool tall = building_height >= 14.0 * args.scale;
							const RoofType european_roof =
									era == ArchEra::HistoricOrnate &&
													footprint_m2 >= 400.0 && tall &&
													european_rng.random_bool(.15)
											? RoofType::Mansard
									: attached ? RoofType::Gabled
											   : RoofType::Hipped;
							generated_roof_type = european_roof;
							generated_sloped_roof = true;
							const std::optional<Block> curtain_glass =
									is_glass_wall_block(wall_block) &&
													is_glass_wall_block(window_block)
											? std::optional<Block>(window_block)
											: std::nullopt;
							generate_roof(*editor, element, start_y_offset,
									building_height, floor_block, wall_block,
									accent_block, roof_block_override, curtain_glass,
									group_seed,
									roof_dormers_enabled(condition, category,
											european_roof, era,
											building_style::building_region(wall_region),
											building_height, floor_cycle, group_seed),
									category == BuildingCategory::Religious,
									preferred_ridge_along_x, european_roof, roof_area,
									abs_terrain_offset, args.scale, short_roof_eave,
									RoofStyleContext{category, era, wall_region,
											wall_attached, wall_district, window_block,
											window_phase, roof_soffit},
									hole_polygons, &eave_blocked, tone_patches_enabled);
							roof_generated = true;
						}
					}
				}
			}
			if (auto_roof_type && !regional_roof_handled) {
				std::size_t footprint_size = cached_footprint_size;
				const std::size_t max_footprint_for_gabled = 800;
				auto roof_rng = element_rng(group_seed ^ 0x0F1E2D3C4B5A6907ULL);
				const bool big_block = footprint_size > max_footprint_for_gabled ||
									   building_height >= 15;
				RoofType automatic_roof = RoofType::Flat;
				bool make_roof = false;
				const bool north_american_flat =
						dominant_region == building_style::BuildingRegion::NorthAmerica &&
						(attached || building_height >= 13.0 * args.scale ||
								(!house && footprint_size >
												   300.0 * args.scale * args.scale));
				if (north_american_flat) {
					// Preserve the flat roof choice for attached and mid-rise stock.
				} else if (btype == "apartments" && big_block) {
					const bool prewar = era == ArchEra::HistoricOrnate ||
										era == ArchEra::TraditionalPreWar;
					const auto roll = roof_rng.uniform(100);
					const std::uint32_t hip_limit = prewar ? 65u : 80u;
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
							roof_dormers_enabled(condition, category, automatic_roof, era,
									building_style::building_region(wall_region),
									building_height, floor_cycle, group_seed),
							category == BuildingCategory::Religious,
							preferred_ridge_along_x, automatic_roof, roof_area,
							abs_terrain_offset, args.scale, short_roof_eave,
							RoofStyleContext{category, era, wall_region, wall_attached,
									wall_district, window_block, window_phase,
									roof_soffit},
							hole_polygons, &eave_blocked, tone_patches_enabled);
					roof_generated = true;
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
							roof_dormers_enabled(condition, category, RoofType::Skillion,
									era, building_style::building_region(wall_region),
									building_height, floor_cycle, group_seed),
							category == BuildingCategory::Religious,
							preferred_ridge_along_x, RoofType::Skillion, roof_area,
							abs_terrain_offset, args.scale, short_roof_eave,
							RoofStyleContext{category, era, wall_region, wall_attached,
									wall_district, window_block, window_phase,
									roof_soffit},
							hole_polygons, &eave_blocked, tone_patches_enabled);
					roof_generated = true;
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
		const auto sill_block = cached_prop_block(
				get_slab_block_for_material(wall_block), {{"type", "top"}});
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
					if (horizontal_windows) {
						const auto ledge = cached_prop_block(
								get_slab_block_for_material(accent_block),
								{{"type", "top"}});
						for (int h = start_y_offset + 2; h <= top_h; ++h)
							if (floor_row_at(h) == 0)
								place_property(ledge, h);
					}
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
	// Flat roofs are emitted by generate_roof with their resolved material.
	// The fallback ceiling belongs only to styles that declined roof
	// generation; rewriting it after a flat roof would erase roof:material and
	// category/preset roof blocks.
	if (!roof_generated && !skip_top && !roof_enclosed) {
		for (const auto &[x, z] : cached_floor_area)
			editor->set_block_absolute(floor_block, x,
					start_y_offset + building_height + abs_terrain_offset + 1, z);
	}
	const auto building_tag_for_terrace = element.tags.get("building");
	const bool residential_building_for_terrace =
			category == BuildingCategory::House ||
			category == BuildingCategory::Residential ||
			building_tag_for_terrace == "apartments" ||
			building_tag_for_terrace == "residential" ||
			building_tag_for_terrace == "dormitory" ||
			building_tag_for_terrace == "house" ||
			building_tag_for_terrace == "detached" ||
			building_tag_for_terrace == "semidetached_house" ||
			building_tag_for_terrace == "terrace" ||
			building_tag_for_terrace == "bungalow" || building_tag_for_terrace == "villa";
	const bool residential_tower_terrace =
			element.tags.contains("building:part") &&
			generated_roof_type == RoofType::Flat && building_height >= 28 &&
			roof_area.size() > THIN_RING_MAX_CELLS && residential_building_for_terrace &&
			!modeled_part_roof && !covered_by_sibling;
	if (args.roof && roof_generated && !skip_top &&
			roof_area.size() > THIN_RING_MAX_CELLS &&
			generated_roof_type == RoofType::Flat && has_parapet && !covered_by_sibling)
		generate_parapet(*editor, element, roof_area, facade_plan, start_y_offset,
				building_height, abs_terrain_offset, wall_block, accent_block, category,
				residential_tower_terrace ||
						(podium_tower && residential_building_for_terrace));
	if (roof_generated && !generated_sloped_roof && args.roof &&
			roof_area.size() > THIN_RING_MAX_CELLS && !has_parapet &&
			!modeled_part_roof && !covered_by_sibling)
		generate_flat_roof_edge_variation(*editor, roof_area, facade_plan, start_y_offset,
				building_height, abs_terrain_offset, wall_block, accent_block, category,
				group_seed);
	generate_facade_cornice_details(*editor, element, facade_plan, category, era,
			building_style::building_region(wall_region) ==
					building_style::BuildingRegion::Europe,
			condition, window_archetype, wall_block, sill_base, has_windows,
			min_level_offset == 0, storefront, generated_sloped_roof, start_y_offset,
			building_height, abs_terrain_offset, floor_cycle, grammar_anchor,
			effective_passages, &preset_shell, &photo_shell, current_footprint,
			building_footprints);
	const bool generated_chimney_roof = generated_roof_type == RoofType::Gabled ||
										generated_roof_type == RoofType::Hipped ||
										generated_roof_type == RoofType::Mansard ||
										generated_roof_type == RoofType::Gambrel ||
										generated_roof_type == RoofType::HalfHipped;
	const auto chimney_region = building_style::building_region(wall_region);
	const int chimney_storeys = building_height / std::max(1, floor_cycle);
	const bool chimney_town_block =
			category != BuildingCategory::House && chimney_storeys >= 3;
	const bool chimney_pre_war = era == ArchEra::Unknown ||
								 era == ArchEra::HistoricOrnate ||
								 era == ArchEra::TraditionalPreWar;
	const bool town_flues =
			chimney_region == building_style::BuildingRegion::Europe &&
			chimney_town_block && chimney_pre_war && !covered_by_sibling &&
			(category == BuildingCategory::Residential ||
					category == BuildingCategory::Historic ||
					category == BuildingCategory::Hotel) &&
			element_rng(group_seed ^ 0xC41E00005EED0004ULL).random_bool(.7);
	if (generated_sloped_roof && generated_chimney_roof &&
			(style_has_chimney || town_flues)) {
		const int roof_base = start_y_offset + building_height + 1;
		int scan_height = std::max(64, building_height * 4 + 16);
		if (const auto mapped_height = parse_meter_tag(element.tags, "roof:height"))
			scan_height = std::max(scan_height,
					static_cast<int>(std::ceil(*mapped_height * args.scale)) + 16);
		scan_height = std::min(scan_height, 2048);
		RoofHeightMap roof_surface;
		roof_surface.reserve(roof_area.size() * 2);
		for (const auto &[x, z] : roof_area) {
			bool found = false;
			int top = roof_base;
			for (int y = roof_base; y <= roof_base + scan_height; ++y) {
				const auto block =
						editor->get_block_absolute(x, y + abs_terrain_offset, z);
				if (block && *block != AIR) {
					found = true;
					top = y;
				} else if (found) {
					break;
				}
			}
			if (found)
				roof_surface[{x, z}] = top;
		}
		const std::optional<int> row_spacing =
				chimney_region == building_style::BuildingRegion::Europe &&
								chimney_town_block
						? std::optional<int>(std::max(4, multiply_scale(9, args.scale)))
						: std::nullopt;
		const bool stucco =
				row_spacing && !is_glass_wall_block(wall_block) &&
				element_rng(group_seed ^ 0xC41E00005EED0002ULL).random_bool(.5);
		generate_chimney(*editor, roof_surface, roof_base, abs_terrain_offset, element.id,
				row_spacing, stucco ? wall_block : BRICK);
	}
	// Rust sends crown and podium walls through its normal facade selector.
	// Preserve the same bay phase and floor grammar here instead of synthesizing
	// windows from the inset cells' world-coordinate checkerboard.
	const int inset_style_height =
			podium_tower ? podium_tower->full_height : building_height;
	const Block inset_trim_block = trim_block_for_wall(wall_block, group_seed);
	auto roof_cover_rng = element_rng(group_seed ^ 0xF00FC010BA5EF00DULL);
	const auto mapped_cover =
			rust_tagged_roof_block(element, false, std::nullopt, roof_cover_rng);
	const rooftops::RoofMembrane inset_roof_cover =
			mapped_cover ? rooftops::RoofMembrane::uniform(*mapped_cover)
			: roof_block_override
					? rooftops::RoofMembrane::uniform(*roof_block_override)
					: rooftops::pick_membrane(
							  building_style::building_region(wall_region), wall_region,
							  category, era, inset_style_height, group_seed);
	WallVariety inset_wall_variety(element.id);
	const InsetWallSelector select_inset_wall = [&](int x, int h, int z, bool open_x,
														bool corner) {
		const bool above_floor = h > start_y_offset + 1;
		const int floor_row =
				((h - start_y_offset - grammar_anchor) % floor_cycle + floor_cycle) %
				floor_cycle;
		const int ground_top = start_y_offset + grammar_anchor - 1 + floor_cycle;
		const bool ground_role = h <= ground_top;
		const bool top_role = h > start_y_offset + inset_style_height - floor_cycle;
		const int wall_u = open_x ? z : x;
		auto rhythm = window_rhythm(window_archetype);
		if (window_surrounds)
			++rhythm.pier;
		const int period = std::max(1, rhythm.window + rhythm.pier);
		const int phase = wall_u + window_phase;
		const int slot_index = (phase % period + period) % period;
		const bool slot_window = slot_index < rhythm.window;
		const bool window_cut = above_floor && has_windows && slot_window &&
								category != BuildingCategory::Tower &&
								category != BuildingCategory::GridSkyscraper &&
								!horizontal_windows;
		Block chosen = wall_block;
		if (!has_windows) {
			const bool glass_tower = category == BuildingCategory::GlassySkyscraper ||
									 category == BuildingCategory::GlassCornerSkyscraper;
			if (corner && glass_tower)
				chosen = is_glass_wall_block(accent_block) ? LIGHT_GRAY_CONCRETE
														   : accent_block;
			else if (use_accent_lines && above_floor && floor_row == 0)
				chosen = accent_block;
		} else if (horizontal_windows) {
			chosen = corner || !above_floor ? wall_block
					 : floor_row == 0		? accent_block
											: window_block;
		} else if (category == BuildingCategory::Tower) {
			const bool slit = above_floor && (floor_row == 1 || floor_row == 2) &&
							  ((phase % 4 + 4) % 4) == 1;
			chosen = slit ? window_block
						  : (use_accent_lines && above_floor && floor_row == 0
											? accent_block
											: wall_block);
		} else if (category == BuildingCategory::GridSkyscraper) {
			const bool mullion = !above_floor || floor_row == 0 || corner ||
								 ((phase % 5 + 5) % 5) == 0;
			chosen = mullion ? wall_block : window_block;
		} else {
			if (has_base_course && h <= start_y_offset + base_course_rows && !window_cut)
				chosen = base_course_block;
			else if (ground_role) {
				if (above_floor && slot_window)
					chosen = window_block;
				else if (rustication && h > start_y_offset + base_course_rows &&
						 (h - start_y_offset) % 2 == 0)
					chosen = has_base_course ? base_course_block : accent_block;
			} else if (floor_row == 0) {
				const bool top_band =
						top_treatment &&
						h == start_y_offset + inset_style_height - floor_cycle;
				if (top_band || use_accent_lines)
					chosen = accent_block;
				else if (slot_window &&
						 (window_archetype == WindowArchetype::Piers ||
								 window_archetype == WindowArchetype::Strips ||
								 use_vertical_accent))
					chosen = accent_block;
				else if (slot_window)
					chosen = inset_trim_block;
			} else {
				bool rows = false;
				if (attic_style && top_role)
					rows = floor_row == floor_cycle - 1;
				else if (top_treatment && top_role && floor_cycle >= 3)
					rows = floor_row < floor_cycle - 1;
				else {
					rows = archetype_allows_window(
							window_archetype, slot_index, floor_row, floor_cycle);
					const bool nobile = piano_nobile && h > ground_top &&
										h <= ground_top + floor_cycle;
					if (european_punched_window_cap && floor_row > (nobile ? 3 : 2))
						rows = false;
				}
				if (slot_window && rows)
					chosen = window_block;
				else if (window_archetype == WindowArchetype::Ribbon && !rows)
					chosen = accent_block;
			}
		}
		if (photo_shell.contains({x, z}) || preset_shell.contains({x, z}))
			chosen = wall_block;
		chosen = apply_block_variety(chosen, x, h, z, wall_block, start_y_offset,
				element.id, window_block, condition, inset_wall_variety);
		return apply_condition_variation(chosen, x, h, z, wall_block, window_block,
				has_windows, condition, category, era, element.id);
	};
	if (podium_tower) {
		const auto distances = roof_edge_distances(roof_area);
		const std::vector<InsetTier> tower{{podium_tower->inset,
				podium_tower->full_height - podium_tower->podium_height, true}};
		generate_inset_tiers(*editor, roof_area, distances,
				start_y_offset + building_height + 1, abs_terrain_offset, tower,
				floor_cycle, start_y_offset + grammar_anchor, floor_block,
				inset_roof_cover, select_inset_wall);
		std::vector<std::pair<int, int>> tower_cells;
		for (const auto &cell : roof_area)
			if (distances.at(cell) >= podium_tower->inset)
				tower_cells.push_back(cell);
		const int tower_height = podium_tower->full_height;
		if (has_parapet && !tower_cells.empty()) {
			const auto empty_facade = building_facade::FacadePlan::empty();
			generate_parapet(*editor, element, tower_cells, empty_facade, start_y_offset,
					tower_height, abs_terrain_offset, wall_block, accent_block, category,
					false,
					rooftops::coping_for(*editor, wall_block, accent_block, group_seed));
		}
		// Rust fits out the inset tower's own roof after its podium/crown pass.
		// The ordinary rooftop pass is skipped for podium towers, so build the
		// same deck explicitly here at the full-height surface.
		if (tower_height >= 4 && condition != BuildingCondition::Construction &&
				condition != BuildingCondition::Ruined &&
				category != BuildingCategory::Farm &&
				category != BuildingCategory::Garage &&
				category != BuildingCategory::Shed &&
				category != BuildingCategory::Greenhouse &&
				category != BuildingCategory::Religious) {
			rooftops::RooftopBrief brief;
			brief.region = building_style::building_region(wall_region);
			brief.category = category;
			brief.era = era;
			brief.height = tower_height;
			brief.floor_cycle = floor_cycle;
			brief.wall = wall_block;
			brief.scale = args.scale;
			brief.seed = element.id;
			brief.residential = residential_building_for_terrace;
			brief.whole_building = !element.tags.contains("building:part");
			brief.sparse = detail == DetailTier::Minimal ||
						   condition != BuildingCondition::Normal;
			brief.green = inset_roof_cover.is_green();
			const int surface_y = start_y_offset + tower_height + abs_terrain_offset + 1;
			if (auto deck = rooftops::RoofDeck::make(tower_cells, surface_y)) {
				if (category == BuildingCategory::Hospital) {
					int min_x = std::numeric_limits<int>::max();
					int max_x = std::numeric_limits<int>::min();
					int min_z = std::numeric_limits<int>::max();
					int max_z = std::numeric_limits<int>::min();
					for (const auto &node : element.nodes) {
						min_x = std::min(min_x, node.x);
						max_x = std::max(max_x, node.x);
						min_z = std::min(min_z, node.z);
						max_z = std::max(max_z, node.z);
					}
					deck->reserve((min_x + max_x) / 2 - 3, (max_z + min_z) / 2 - 3, 7, 7);
				}
				rooftops::fit_out_roof(*editor, *deck, brief);
			}
		}
	}
	const bool has_crown =
			roof_generated && args.roof && !podium_tower && !generated_sloped_roof &&
			!element.tags.contains("building:part") &&
			generate_setback_crown(*editor, roof_area, category, condition,
					is_tall_building, start_y_offset, building_height, abs_terrain_offset,
					floor_cycle, start_y_offset + grammar_anchor, floor_block,
					inset_roof_cover, element.id, select_inset_wall);
	const bool residential_antenna_roof =
			generated_sloped_roof && category == BuildingCategory::House &&
			condition == BuildingCondition::Normal &&
			(generated_roof_type == RoofType::Gabled ||
					generated_roof_type == RoofType::Hipped ||
					generated_roof_type == RoofType::Pyramidal);
	if (roof_generated && args.roof && !podium_tower && !has_crown &&
			(!modeled_part_roof || residential_antenna_roof)) {
		bool green_roof = false;
		if (generated_roof_type == RoofType::Flat && !roof_block_override) {
			auto membrane_rng = element_rng(group_seed ^ 0xF00FC010BA5EF00DULL);
			if (!rust_tagged_roof_block(element, false, std::nullopt, membrane_rng)) {
				const auto [lat, lon] = editor->geo_anchor();
				const auto materials = building_style::material_region_at(lat, lon);
				green_roof = rooftops::pick_membrane(
						building_style::building_region(materials), materials, category,
						era, building_height, group_seed)
									 .is_green();
			}
		}
		generate_rooftop_systems(*editor, element, roof_area, category, condition,
				generated_roof_type, start_y_offset, building_height, abs_terrain_offset,
				floor_cycle, wall_block, detail, era, clean_visual_seed,
				covered_by_sibling, args.scale, green_roof);
	}
	if (roof_generated && category == BuildingCategory::Hospital &&
			generated_roof_type == RoofType::Flat)
		generate_hospital_helipad(*editor, element, roof_area, start_y_offset,
				building_height, abs_terrain_offset);
	// Rust only paints the hospital wall-cross pass for a mapped roof:shape
	// (the hospital preset itself does not request an explicit roof pass).
	const bool explicit_roof_pass = element.tags.contains("roof:shape");
	if (roof_generated && explicit_roof_pass && category == BuildingCategory::Hospital)
		generate_hospital_wall_cross(*editor, element, category, start_y_offset,
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
		int32_t abs_terrain_offset, double scale_factor, bool short_eave,
		std::optional<RoofStyleContext> style_context,
		const std::vector<HolePolygon> *hole_polygons,
		const std::unordered_set<std::pair<int, int>, PairHash> *eave_blocked,
		bool tone_patches)
{
	using std::int32_t;
	const auto &floor_area = cached_floor_area;
	const bool soffit = style_context && style_context->soffit;
	auto fill_shell = [&](const std::unordered_map<std::pair<int32_t, int32_t>, int32_t,
								  pair_hash> &surface,
							  Block roof_block, int base_y) {
		for (const auto &[cell, top] : surface) {
			const auto [x, z] = cell;
			int lowest_neighbour = std::numeric_limits<int>::max();
			for (const auto &[nx, nz] :
					{std::pair{x - 1, z}, {x + 1, z}, {x, z - 1}, {x, z + 1}}) {
				auto it = surface.find({nx, nz});
				lowest_neighbour = std::min(
						lowest_neighbour, it == surface.end() ? base_y : it->second);
			}
			const int bottom = std::max(base_y, lowest_neighbour - 1);
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
	const bool steep_cap =
			style_context && building_style::building_region(style_context->materials) ==
									 building_style::BuildingRegion::Europe;
	const auto roof_category =
			style_context
					? style_context->category
					: building_category(element, building_height >= 30.0 * scale_factor,
							  building_height, scale_factor, style_seed);
	const std::string building_type = element.tags.contains("building")
											  ? element.tags.get("building")
											  : element.tags.get("building:part");
	const int storeys =
			building_height / std::max(1, floor_cycle_for(building_type, element.tags));
	const bool sheet_top_candidate =
			roof_category != BuildingCategory::House && storeys >= 3;
	const std::optional<int> peak_cap = rust_roof_peak_cap(element.tags, element.nodes,
			roof_type, max_x - min_x, max_z - min_z, building_height, scale_factor,
			preferred_ridge_along_x);
	DormerStyle dormer_style;
	if (style_context) {
		dormer_style.window_phase = style_context->window_phase;
		dormer_style.gable_block = is_glass_wall_block(wall_block)
										   ? std::nullopt
										   : std::optional<Block>(wall_block);
		dormer_style.window_block = style_context->window_block;
		dormer_style.copper_hood =
				style_context->era == ArchEra::HistoricOrnate &&
				element_rng(style_seed ^ 0xC0FFE40000000001ULL).random_bool(.6);
	} else {
		dormer_style.gable_block = is_glass_wall_block(wall_block)
										   ? std::nullopt
										   : std::optional<Block>(wall_block);
		dormer_style.window_block = curtain_glass;
	}

	// Random generator
	auto rng = element_rng(style_seed ^ 0xF00FC010BA5EF00DULL);
	std::optional<Block> regional_pitched_roof;
	if (roof_type != RoofType::Flat && !roof_block_override) {
		const auto [lat, lon] = editor.geo_anchor();
		const auto materials = style_context
									   ? style_context->materials
									   : building_style::material_region_at(lat, lon);
		const auto category =
				style_context ? style_context->category
							  : building_category(element,
										building_height >= 30.0 * scale_factor,
										building_height, scale_factor, style_seed);
		std::uint64_t district = style_context ? style_context->district : 0;
		if (!style_context) {
			int min_x = std::numeric_limits<int>::max();
			int max_x = std::numeric_limits<int>::min();
			int min_z = std::numeric_limits<int>::max();
			int max_z = std::numeric_limits<int>::min();
			for (const auto &[x, z] : floor_area) {
				min_x = std::min(min_x, x);
				max_x = std::max(max_x, x);
				min_z = std::min(min_z, z);
				max_z = std::max(max_z, z);
			}
			const int cell =
					std::max(16, static_cast<int>(std::lround(128.0 * scale_factor)));
			const int cx = floor_div(min_x + (max_x - min_x) / 2, cell);
			const int cz = floor_div(min_z + (max_z - min_z) / 2, cell);
			auto district_rng = coord_rng(cx, cz, 0xD157C7A15EED0001ULL);
			district =
					std::uint64_t(district_rng()) | (std::uint64_t(district_rng()) << 32);
		}
		const auto type = element.tags.contains("building")
								  ? element.tags.get("building")
								  : element.tags.get("building:part");
		const bool attached = style_context
									  ? style_context->attached
									  : type == "terrace" || type == "semidetached_house";
		regional_pitched_roof = rust_pitched_roof_block(category,
				style_context ? style_context->era : building_arch_era(element.tags),
				roof_type, wall_block, materials, attached, district, style_seed);
	}
	const std::optional<Block> chosen_roof_style =
			roof_block_override ? roof_block_override : regional_pitched_roof;
	const std::optional<Block> tagged_roof_block = rust_tagged_roof_block(
			element, roof_type != RoofType::Flat, chosen_roof_style, rng);
	// Match Rust: an explicit OSM roof material/colour owns the whole surface,
	// so a generated sheet-metal cap must not replace its mapped covering.
	const bool sheet_top_enabled = sheet_top_candidate && !tagged_roof_block;
	// Match Rust precedence: OSM tag, category preset, regional pitched style.
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
		if (regional_pitched_roof)
			return *regional_pitched_roof;
		return default_roof_surface_block(wall_block, style_seed);
	};
	RoofPatchCache tone_patch_cache;
	RoofPatchCache sheet_patch_cache;
	const auto roof_tone_at = [&](int x, int z) -> std::optional<RoofTone> {
		if (!tone_patches || tagged_roof_block ||
				!cached_roof_tone_patch(x, z, element.id, tone_patch_cache))
			return std::nullopt;
		return roof_tone_sibling(selected_roof_block());
	};
	const auto roof_surface_block = [&](int x, int z) {
		if (const auto tone = roof_tone_at(x, z))
			return tone->full;
		return selected_roof_block();
	};
	const auto roof_surface_stair = [&](int x, int z) {
		if (const auto tone = roof_tone_at(x, z))
			return tone->stairs;
		return get_stair_block_for_material(selected_roof_block());
	};

	if (roof_type == RoofType::Flat) {
		const auto distances = roof_edge_distances(floor_area);
		const std::vector<Block> replace_any;
		if (tagged_roof_block || roof_block_override) {
			const Block flat_roof_block =
					tagged_roof_block ? *tagged_roof_block : *roof_block_override;
			for (const auto &[x, z] : floor_area) {
				const auto it = distances.find({x, z});
				const int edge = it == distances.end() ? 0 : it->second;
				editor.set_block_absolute(flat_roof_block, x,
						base_height + abs_terrain_offset, z, nullptr,
						edge > 0 ? &replace_any : nullptr);
			}
		} else {
			const auto [lat, lon] = editor.geo_anchor();
			const auto materials = style_context
										   ? style_context->materials
										   : building_style::material_region_at(lat, lon);
			const auto region = building_style::building_region(materials);
			const auto category =
					style_context ? style_context->category
								  : building_category(element,
											building_height >= 30.0 * scale_factor,
											building_height, scale_factor, style_seed);
			auto era =
					style_context ? style_context->era : building_arch_era(element.tags);
			if (!style_context && era == ArchEra::Unknown &&
					element.tags.contains("building:part"))
				era = static_cast<ArchEra>(osm_parser::arch_era_from_hint(
						osm_parser::style_hint_from_seed(style_seed)));
			const auto membrane = rooftops::pick_membrane(
					region, materials, category, era, building_height, style_seed);
			for (const auto &[x, z] : floor_area) {
				const auto it = distances.find({x, z});
				const int edge = it == distances.end() ? 0 : it->second;
				editor.set_block_absolute(membrane.block_at_edge(x, z, edge), x,
						base_height + abs_terrain_offset, z, nullptr,
						edge > 0 ? &replace_any : nullptr);
			}
		}
		return;
	}

	if (roof_type == RoofType::Gabled || roof_type == RoofType::Gambrel ||
			roof_type == RoofType::HalfHipped || roof_type == RoofType::Round) {
		const std::optional<Block> gable_block =
				is_glass_wall_block(wall_block) ? std::nullopt
												: std::optional<Block>(wall_block);
		generate_gabled_roof_profile(editor, floor_area, element.nodes, element.tags,
				base_height, abs_terrain_offset, building_height, roof_type, steep_gable,
				preferred_ridge_along_x, peak_cap, style_seed, add_dormers,
				selected_roof_block(), short_eave, scale_factor, hole_polygons,
				eave_blocked, steep_cap, tone_patches && !tagged_roof_block.has_value(),
				element.id, sheet_top_enabled, soffit, gable_block);
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
				roof_block, is_wider_than_long, nullptr, short_eave, eave_blocked,
				soffit);
		place_roof_dormer_windows(editor, floor_area, roof_map, base_height,
				abs_terrain_offset, style_seed, roof_block, add_dormers,
				is_wider_than_long ? std::pair{1, 0} : std::pair{0, 1}, std::nullopt,
				dormer_style);

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
			editor.set_block_absolute(roof_surface_block(p.first, p.second), p.first,
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
			const auto edge_distances = hip_edge_distances(floor_area, footprint);
			const auto runs = axis_runs(floor_area);
			struct HipCell
			{
				int dist_to_edge;
				int local_half;
			};
			std::unordered_map<std::pair<int32_t, int32_t>, HipCell, pair_hash> cell_data;
			std::unordered_map<std::pair<int32_t, int32_t>, int32_t, pair_hash>
					roof_heights;
			std::unordered_map<std::pair<int32_t, int32_t>, std::pair<int32_t, int32_t>,
					pair_hash>
					dormer_edge_scans;
			cell_data.reserve(floor_area.size() * 2);
			roof_heights.reserve(floor_area.size() * 2);
			dormer_edge_scans.reserve(floor_area.size() * 2);
			const bool ridge_runs_along_x = width >= length;
			int max_dist = 0;
			for (std::size_t area_i = 0; area_i < floor_area.size(); ++area_i) {
				const auto [x, z] = floor_area[area_i];
				const auto [dm_x, dp_x, dm_z, dp_z] = runs[area_i];
				// Rust treats cells absent from the distance field as eaves. Keep
				// malformed/disconnected raster footprints from throwing here too.
				const auto edge_it = edge_distances.find({x, z});
				const int dist_to_edge =
						edge_it == edge_distances.end() ? 0 : edge_it->second;
				max_dist = std::max(max_dist, dist_to_edge);
				const int half_x = (dm_x + dp_x + 1) / 2;
				const int half_z = (dm_z + dp_z + 1) / 2;
				const int local_half = std::min(half_x, half_z);
				cell_data[{x, z}] = {dist_to_edge, local_half};
				dormer_edge_scans[{x, z}] = ridge_runs_along_x ? std::pair{dm_z, dp_z}
															   : std::pair{dm_x, dp_x};
			}

			const int wall_cap = peak_cap.value_or(
					std::min(std::max(1, int(std::lround(building_height * .6))),
							rust_max_inferred_roof_rise(steep_gable, scale_factor)));
			const Block stair_material =
					get_stair_block_for_material(selected_roof_block());
			const auto roof_slab = roof_slab_for_stair(stair_material);
			const bool slab_pitch =
					roof_type != RoofType::Mansard && roof_slab &&
					((peak_cap && *peak_cap < .75 * std::max(1, max_dist)) ||
							(!peak_cap && max_dist > wall_cap + 1 && !steep_cap));
			const bool tread_pitch = !slab_pitch && roof_type != RoofType::Mansard &&
									 !peak_cap && !steep_cap && max_dist - wall_cap >= 4;
			int mansard_steep_height = 4;
			if (roof_type == RoofType::Mansard)
				if (const auto levels = parse_meter_tag(element.tags, "roof:levels"))
					mansard_steep_height =
							std::clamp(int(std::lround(*levels * 3.0)), 3, 6);
			constexpr int roof_lift = 1;
			const int eave = base_height + roof_lift;
			RoofHeightMap roof_levels;
			roof_levels.reserve(floor_area.size() * 2);
			for (const auto &[x, z] : floor_area) {
				const auto &cell = cell_data.at({x, z});
				int roof_level;
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
					const int roof_y = eave + std::min(rise, std::min(cap, local_cap));
					roof_level = 2 * roof_y + 1;
				} else if (peak_cap) {
					const double pitch = double(*peak_cap) / std::max(1, max_dist);
					if (slab_pitch) {
						const int rise = std::clamp(
								int(std::lround(2.0 * cell.dist_to_edge * pitch)), 0,
								2 * std::max(1, *peak_cap));
						roof_level = 2 * eave + rise;
					} else {
						int rise;
						if (pitch < 1.0) {
							rise = 2 *
								   std::min(cell.dist_to_edge, std::max(1, *peak_cap));
						} else {
							rise = 2 *
								   std::clamp(int(std::lround(cell.dist_to_edge * pitch)),
										   0, std::max(1, *peak_cap));
						}
						roof_level = 2 * (eave + rise / 2) + 1;
					}
				} else {
					if (slab_pitch) {
						const int rise = std::min(cell.dist_to_edge, 2 * wall_cap);
						roof_level = 2 * eave + rise;
					} else {
						const int slope = tread_pitch ? (cell.dist_to_edge + 1) / 2
													  : cell.dist_to_edge;
						const int rise = 2 * std::min(slope, wall_cap);
						roof_level = 2 * (eave + rise / 2) + 1;
					}
				}
				roof_levels[{x, z}] = roof_level;
				roof_heights[{x, z}] = roof_level / 2;
			}

			const RoofCellSet sheet_cells =
					sheet_top_enabled ? (slab_pitch ? capped_flat_tops(roof_levels)
													: capped_flat_tops(roof_heights))
									  : RoofCellSet{};
			const std::vector<Block> replace_any;
			constexpr std::array<std::pair<int, int>, 4> roof_sides{
					{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};
			constexpr std::array<std::pair<int, int>, 4> roof_quarters{
					{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}}};
			const RoofLevelGrid roof_level_grid(roof_levels);
			for (const auto &[x, z] : floor_area) {
				const int roof_y = roof_heights.at({x, z});
				const int level = roof_levels.at({x, z});
				const Block cell_stair_material = roof_surface_stair(x, z);
				const Block cell_roof_block = roof_surface_block(x, z);
				const auto cell_slab = roof_slab_for_stair(cell_stair_material);
				for (int y = base_height; y < roof_y; ++y)
					editor.set_block_absolute(cell_roof_block, x, y + abs_terrain_offset,
							z, nullptr, &replace_any);

				bool sheet = false;
				std::optional<BlockWithProperties> cap;
				if (level % 2 == 0) {
					if (roof_slab && cell_slab)
						cap = cached_prop_block(*cell_slab, {{"type", "bottom"}});
				} else {
					const int floor = slab_pitch ? level - 1 : level;
					const auto not_lower = [&](int nx, int nz) {
						const auto neighbor = roof_level_grid.get(nx, nz);
						return neighbor && *neighbor >= floor;
					};
					std::array<bool, 4> high{};
					for (std::size_t i = 0; i < roof_quarters.size(); ++i) {
						const auto [dx, dz] = roof_quarters[i];
						high[i] = not_lower(x + dx, z) && not_lower(x, z + dz) &&
								  not_lower(x + dx, z + dz);
					}
					const bool steep_drop = std::any_of(roof_sides.begin(),
							roof_sides.end(), [&, x = x, z = z](const auto &d) {
								const auto neighbor =
										roof_level_grid.get(x + d.first, z + d.second);
								return neighbor && level - *neighbor > 4;
							});
					const bool all_high = std::all_of(
							high.begin(), high.end(), [](bool value) { return value; });
					const bool all_low = std::none_of(
							high.begin(), high.end(), [](bool value) { return value; });
					if (steep_drop) {
						cap = BlockWithProperties::simple(cell_roof_block);
					} else if (all_high) {
						const bool stepped = std::any_of(roof_sides.begin(),
								roof_sides.end(), [&, x = x, z = z](const auto &d) {
									const auto neighbor = roof_level_grid.get(
											x + d.first, z + d.second);
									return neighbor && *neighbor < level;
								});
						sheet = sheet_cells.contains({x, z});
						if (slab_pitch && stepped && roof_slab && cell_slab)
							cap = cached_prop_block(*cell_slab, {{"type", "top"}});
						else
							cap = BlockWithProperties::simple(
									sheet ? roof_sheet_block(x, z, sheet_patch_cache)
										  : cell_roof_block);
					} else if (const auto stair = roof_stair_for_high_quarters(high)) {
						cap = create_stair_with_properties(
								cell_stair_material, stair->first, stair->second);
					} else if (all_low && roof_slab && cell_slab) {
						cap = cached_prop_block(*cell_slab, {{"type", "bottom"}});
					} else {
						cap = BlockWithProperties::simple(cell_roof_block);
					}
				}
				if (!cap)
					cap = BlockWithProperties::simple(cell_roof_block);
				editor.set_block_with_properties_absolute(
						*cap, x, roof_y + abs_terrain_offset, z, nullptr, &replace_any);
			}
			place_hipped_eave_overhang(editor, floor_area, base_height,
					abs_terrain_offset, stair_material, short_eave, eave_blocked,
					soffit && !slab_pitch, slab_pitch);
			const auto mansard_band =
					roof_mansard_dormer_band(element, base_height, roof_type);
			place_roof_dormer_windows(editor, floor_area, roof_heights, base_height,
					abs_terrain_offset, style_seed, selected_roof_block(), add_dormers,
					width >= length ? std::pair{1, 0} : std::pair{0, 1}, mansard_band,
					dormer_style, &dormer_edge_scans, &footprint);
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
			const RoofCellSet sheet_cells =
					sheet_top_enabled ? capped_flat_tops(roof_heights) : RoofCellSet{};

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
							editor.set_block_absolute(
									sheet_cells.contains({x, z})
											? roof_sheet_block(x, z, sheet_patch_cache)
											: roof_block,
									x, y + abs_terrain_offset, z, nullptr, nullptr);
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
					long_axis_is_x ? std::pair{1, 0} : std::pair{0, 1}, mansard_band,
					dormer_style);
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
					long_axis_is_x ? std::pair{1, 0} : std::pair{0, 1}, mansard_band,
					dormer_style);
		}
		if (roof_type == RoofType::Gambrel || roof_type == RoofType::HalfHipped)
			place_gabled_eave_overhang(editor, floor_area, base_height,
					abs_terrain_offset, roof_block, long_axis_is_x, nullptr, short_eave,
					eave_blocked, soffit);

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
		const std::vector<Block> replace_any;

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
								x, y + abs_terrain_offset, z, nullptr, &replace_any);
					} else {
						editor.set_block_absolute(roof_block, x, y + abs_terrain_offset,
								z, nullptr, &replace_any);
					}
				} else {
					editor.set_block_absolute(roof_block, x, y + abs_terrain_offset, z,
							nullptr, &replace_any);
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
		const int inferred_boost = std::min(
				std::max(1, static_cast<int>(std::round(shorter_half * 0.85))),
				std::min(std::max(1, static_cast<int>(std::round(building_height * 0.6))),
						rust_max_inferred_roof_rise(steep_gable, scale_factor)));
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
		const std::vector<Block> replace_any;
		const std::unordered_set<std::pair<int32_t, int32_t>, pair_hash> footprint(
				floor_area.begin(), floor_area.end());

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
					const bool on_polygon_edge = !footprint.contains({x - 1, z}) ||
												 !footprint.contains({x + 1, z}) ||
												 !footprint.contains({x, z - 1}) ||
												 !footprint.contains({x, z + 1});
					const bool steep_drop =
							(has_lower_north && roof_height - north_height > 2) ||
							(has_lower_south && roof_height - south_height > 2) ||
							(has_lower_west && roof_height - west_height > 2) ||
							(has_lower_east && roof_height - east_height > 2);

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

					if ((lower_count > 0 || on_polygon_edge) && !steep_drop)
						editor.set_block_with_properties_absolute(stair_block, x,
								y + abs_terrain_offset, z, nullptr, &replace_any);
					else
						editor.set_block_absolute(roof_block, x, y + abs_terrain_offset,
								z, nullptr, &replace_any);
				} else {
					editor.set_block_absolute(roof_block, x, y + abs_terrain_offset, z,
							nullptr, &replace_any);
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
		if (member.way->id == structures::starship::STARBASE_PAD2_INNER_RING_WAY &&
				member.role == ProcessedMemberRole::Inner)
			structures::place_starship(editor, *member.way);

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
		const auto part = member.way->tags.find("building:part");
		const bool explicitly_not_a_part =
				part != member.way->tags.end() && part->second.size() == 2 &&
				std::tolower(static_cast<unsigned char>(part->second[0])) == 'n' &&
				std::tolower(static_cast<unsigned char>(part->second[1])) == 'o';
		const bool is_part = part != member.way->tags.end() && !explicitly_not_a_part;
		const bool is_closed =
				member.way->nodes.size() >= 4 &&
				member.way->nodes.front().id == member.way->nodes.back().id;
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
