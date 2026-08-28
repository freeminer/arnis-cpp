#include "data_processing.h"
#include "element_processing/signage.h"
#include "element_processing/advtrains.h"
#include <sys/types.h>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <cstdlib>
#include <fstream>

#include "../../arnis_adapter.h"
#include "bresenham.h"
#include "element_processing/historic.h"
#include "element_processing/power.h"
#include "element_processing/emergency.h"
#include "element_processing/advertising.h"
#include "element_processing/bridges.h"
#include "element_processing/highway_tunnels.h"
#include "element_processing/buildings.h"
#include "floodfill_cache.h"
#include "ground_generation.h"
#include "land_cover/land_cover.h"
#include "ore_generation.h"
#include "water_depth.h"
#include "world_editor/floor_state.h"
#include "clipping.h"
#include "structures/boat.h"
#include "structures/starship.h"
#include "structures/helicopter.h"
#include "models_3d/wikidata/osm_models.h"
#include "models_3d/wikidata/remote_provider.h"
#include "models_3d/pipeline.h"
#include "models_3d/placement_executor.h"
#include "models_3d/custom/client.h"
#include "models_3d/three_dmr/client.h"
#include "landmarks.h"
#include "trees/engine.h"
#include "decals/render.h"
#include "map_item_palette.h"
#include "util/base64.h"
#include "util/png.h"

namespace arnis
{

// Helper functions for ground fill area detection
namespace
{

std::optional<std::string> decal_texture(
		const decals::DecalRegistry &registry, int map_id)
{
	const auto tile = registry.tile(map_id);
	if (!tile)
		return std::nullopt;
	const auto &[key, tile_x, tile_y] = *tile;
	const auto canvas = decals::render(key);
	constexpr std::uint32_t size = decals::TILE;
	std::vector<std::uint8_t> rgba(std::size_t(size) * size * 4);
	for (std::uint32_t y = 0; y < size; ++y)
		for (std::uint32_t x = 0; x < size; ++x) {
			const auto color = canvas.get(int(tile_x * size + x), int(tile_y * size + y));
			const auto [r, g, b] = map_palette::map_color_rgb(color);
			const auto offset = (std::size_t(y) * size + x) * 4;
			rgba[offset] = r;
			rgba[offset + 1] = g;
			rgba[offset + 2] = b;
			rgba[offset + 3] = color == TRANSPARENT ? 0 : 255;
		}
	const auto png = encodePNG(rgba.data(), size, size, 6);
	return "[png:" + base64_encode(png);
}

bool water_tag_is(const tags_t &tags, const std::string &key, const std::string &value)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

bool is_water_area_way(const ProcessedWay &way)
{
	return water_tag_is(way.tags, "waterway", "dock") ||
		   water_tag_is(way.tags, "waterway", "riverbank");
}

bool is_still_water_relation(const ProcessedRelation &rel)
{
	return rel.tags.contains("water") || water_tag_is(rel.tags, "natural", "water") ||
		   water_tag_is(rel.tags, "natural", "bay") ||
		   water_tag_is(rel.tags, "waterway", "riverbank") ||
		   water_tag_is(rel.tags, "landuse", "reservoir");
}

/// Compute still water surface level for a water polygon
int compute_still_surface_level(const Ground *ground,
		const std::vector<ProcessedNode> &outer_ring, const XZBBox &xzbbox)
{
	if (!ground || !ground->has_land_cover())
		return -1000;

	if (outer_ring.empty())
		return -1000;

	// Sample points from the ring and compute average ground level
	std::int64_t sum_level = 0;
	int count = 0;
	for (const auto &node : outer_ring) {
		if (node.x >= xzbbox.min_x() && node.x <= xzbbox.max_x() &&
				node.z >= xzbbox.min_z() && node.z <= xzbbox.max_z()) {
			sum_level += ground->level({node.x, node.z});
			count++;
		}
	}

	if (count == 0)
		return -1000;

	return static_cast<int>(sum_level / count);
}

} // anonymous namespace

StillWaterSurfaces prescan_still_surfaces(const std::vector<ProcessedElement> &elements,
		const Ground *ground, const XZBBox &xzbbox)
{
	StillWaterSurfaces result;

	if (!ground || !ground->has_land_cover())
		return result;

	for (const auto &element : elements) {
		std::pair<std::string, std::uint64_t> key;
		std::vector<ProcessedNode> outer_ring;

		if (element.is_way()) {
			const auto &way = element.as_way();
			if (!is_water_area_way(way))
				continue;
			key = {"way", way.id};
			outer_ring = way.nodes;
		} else if (element.is_relation()) {
			const auto &rel = element.as_relation();
			if (!is_still_water_relation(rel))
				continue;
			key = {"relation", rel.id};
			// Collect outer ring from relation members
			for (const auto &member : rel.members) {
				if (member.role == ProcessedMemberRole::Outer) {
					outer_ring.insert(outer_ring.end(), member.way.nodes.begin(),
							member.way.nodes.end());
				}
			}
		} else {
			continue;
		}

		if (!outer_ring.empty()) {
			int level = compute_still_surface_level(ground, outer_ring, xzbbox);
			if (level > -1000) {
				result.surfaces[key] = level;
			}
		}
	}

	return result;
}

bool should_stream_to_disk(std::size_t tile_count)
{
	if (const char *v = std::getenv("ARNIS_STREAM_TO_DISK")) {
		if (std::string(v) == "1")
			return true;
		if (std::string(v) == "0")
			return false;
	}
	constexpr std::uint64_t base_mb = 500, per_region_mb = 26;
	std::uint64_t available_mb = 0;
	std::ifstream mem("/proc/meminfo");
	std::string key;
	std::uint64_t kb;
	while (mem >> key >> kb) {
		if (key == "MemAvailable:") {
			available_mb = kb / 1024;
			break;
		}
		std::string unit;
		std::getline(mem, unit);
	}
	return available_mb > 0 &&
		   (base_mb + per_region_mb * tile_count) * 100 > available_mb * 55;
}

bool should_use_parallel_tiles(std::size_t tile_count, bool java_format)
{
	// Rust only parallelizes Java region work once there are enough tiles to
	// amortize thread/editor setup; Bedrock and Luanti retain ordered writes.
	return java_format && tile_count >= 3;
}

GenerationFeatureFlags generation_features(
		bool java_format, bool luanti_format, bool map_item, bool map_preview)
{
	GenerationFeatureFlags f;
	f.map_item = map_item && java_format;
	f.map_preview = map_preview && !luanti_format;
	f.branding = java_format;
	f.map_decals = java_format;
	return f;
}

GenerationTilePolicy generation_tile_policy(std::size_t tile_count, bool java_format)
{
	return {should_use_parallel_tiles(tile_count, java_format),
			should_stream_to_disk(tile_count)};
}

void release_finished_fills(FloodFillCache &cache,
		const std::unordered_map<std::uint64_t, std::size_t> &last_use,
		const ProcessedElement &element, std::size_t index)
{
	if (element.is_way()) {
		const auto &way = element.as_way();
		auto it = last_use.find(way.id);
		if (it != last_use.end() && it->second == index)
			cache.remove_way(way.id);
	} else if (element.is_relation()) {
		for (const auto &member : element.as_relation().members) {
			auto it = last_use.find(member.way.id);
			if (it != last_use.end() && it->second == index)
				cache.remove_way(member.way.id);
		}
	}
}

std::unordered_map<std::uint64_t, std::size_t> compute_last_fill_use(
		const std::vector<ProcessedElement> &elements)
{
	std::unordered_map<std::uint64_t, std::size_t> result;
	for (std::size_t i = 0; i < elements.size(); ++i) {
		const auto &element = elements[i];
		if (element.is_way()) {
			result[element.as_way().id] = i;
		} else if (element.is_relation()) {
			for (const auto &member : element.as_relation().members)
				result[member.way.id] = i;
		}
	}
	return result;
}

namespace
{

bool is_closed_ring(const std::vector<ProcessedNode> &nodes)
{
	return nodes.size() >= 4 && nodes.front().x == nodes.back().x &&
		   nodes.front().z == nodes.back().z;
}

bool same_point(const ProcessedNode &a, const ProcessedNode &b)
{
	return a.x == b.x && a.z == b.z;
}

void stitch_way_segments(std::vector<std::vector<ProcessedNode>> &rings)
{
	bool changed = true;
	while (changed) {
		changed = false;
		for (std::size_t i = 0; i < rings.size() && !changed; ++i) {
			if (rings[i].empty() || is_closed_ring(rings[i]))
				continue;
			for (std::size_t j = i + 1; j < rings.size(); ++j) {
				if (rings[j].empty() || is_closed_ring(rings[j]))
					continue;
				if (same_point(rings[i].back(), rings[j].front())) {
					rings[i].insert(rings[i].end(), rings[j].begin() + 1, rings[j].end());
				} else if (same_point(rings[i].front(), rings[j].back())) {
					rings[j].insert(rings[j].end(), rings[i].begin() + 1, rings[i].end());
					rings[i] = std::move(rings[j]);
				} else if (same_point(rings[i].front(), rings[j].front())) {
					std::reverse(rings[j].begin(), rings[j].end());
					rings[j].insert(rings[j].end(), rings[i].begin() + 1, rings[i].end());
					rings[i] = std::move(rings[j]);
				} else if (same_point(rings[i].back(), rings[j].back())) {
					std::reverse(rings[j].begin(), rings[j].end());
					rings[i].insert(rings[i].end(), rings[j].begin() + 1, rings[j].end());
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

bool tag_is(const tags_t &tags, const std::string &key, const std::string &value)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

double ring_area(const std::vector<ProcessedNode> &nodes)
{
	if (nodes.size() < 3)
		return 0.0;
	double twice_area = 0.0;
	for (size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		twice_area += static_cast<double>(a.x) * b.z - static_cast<double>(b.x) * a.z;
	}
	return std::abs(twice_area * 0.5);
}

bool landuse_paints_ground(const tags_t &tags)
{
	const auto it = tags.find("landuse");
	return it == tags.end() ||
		   (it->second != "residential" && it->second != "commercial");
}

std::optional<double> ground_fill_area(const ProcessedElement &element)
{
	double area = 0.0;
	if (element.is_way()) {
		const auto &way = element.as_way();
		const auto &tags = way.tags;
		if (tags.contains("building") || tags.contains("building:part") ||
				tags.contains("highway"))
			return std::nullopt;
		bool fills = false;
		if (tags.contains("landuse"))
			fills = landuse_paints_ground(tags);
		else if (tags.contains("natural"))
			fills = !tag_is(tags, "amenity", "fountain");
		else if (!tags.contains("amenity"))
			fills = tags.contains("leisure");
		if (!fills)
			return std::nullopt;
		area = ring_area(way.nodes);
	} else if (element.is_relation()) {
		const auto &rel = element.as_relation();
		const auto &tags = rel.tags;
		if (tags.contains("building") || tags.contains("building:part") ||
				tag_is(tags, "type", "building") || tags.contains("water") ||
				tag_is(tags, "natural", "water") || tag_is(tags, "natural", "bay"))
			return std::nullopt;
		const bool fills = tags.contains("natural") ||
						   (tags.contains("landuse") && landuse_paints_ground(tags)) ||
						   tag_is(tags, "leisure", "park");
		if (!fills)
			return std::nullopt;
		for (const auto &member : rel.members) {
			if (member.role == ProcessedMemberRole::Outer)
				area += ring_area(member.way.nodes);
		}
	} else {
		return std::nullopt;
	}
	return area > 0.0 ? std::optional<double>(area) : std::nullopt;
}

void sort_ground_fill_areas(std::vector<ProcessedElement> &elements)
{
	std::vector<size_t> slots;
	std::vector<ProcessedElement> areas;
	for (size_t i = 0; i < elements.size(); ++i) {
		if (ground_fill_area(elements[i])) {
			slots.push_back(i);
			areas.push_back(elements[i]);
		}
	}
	std::stable_sort(areas.begin(), areas.end(), [](const auto &a, const auto &b) {
		return ground_fill_area(a).value_or(0.0) > ground_fill_area(b).value_or(0.0);
	});
	for (size_t i = 0; i < slots.size(); ++i)
		elements[slots[i]] = std::move(areas[i]);
}

bool is_water_polygon_way(const ProcessedWay &way)
{
	return tag_is(way.tags, "natural", "water") || tag_is(way.tags, "natural", "bay") ||
		   tag_is(way.tags, "waterway", "riverbank") ||
		   tag_is(way.tags, "landuse", "reservoir") || way.tags.contains("water");
}

bool is_water_relation(const ProcessedRelation &rel)
{
	return rel.tags.contains("water") || tag_is(rel.tags, "natural", "water") ||
		   tag_is(rel.tags, "natural", "bay") ||
		   tag_is(rel.tags, "waterway", "riverbank") ||
		   tag_is(rel.tags, "landuse", "reservoir");
}

int waterway_width(const ProcessedWay &way)
{
	const auto it_width = way.tags.find("width");
	if (it_width != way.tags.end()) {
		try {
			return std::max(1, static_cast<int>(std::round(std::stod(it_width->second))));
		} catch (...) {
		}
	}
	const auto it = way.tags.find("waterway");
	if (it == way.tags.end())
		return 1;
	if (it->second == "river")
		return 8;
	if (it->second == "canal")
		return 5;
	if (it->second == "stream" || it->second == "drain")
		return 2;
	return 3;
}

std::pair<std::size_t, std::size_t> grid_index_for(
		int x, int z, const XZBBox &xzbbox, std::size_t width, std::size_t height)
{
	const double xr = std::clamp(static_cast<double>(x - xzbbox.min_x()) /
										 static_cast<double>(std::max<int>(
												 1, xzbbox.max_x() - xzbbox.min_x())),
			0.0, 1.0);
	const double zr = std::clamp(static_cast<double>(z - xzbbox.min_z()) /
										 static_cast<double>(std::max<int>(
												 1, xzbbox.max_z() - xzbbox.min_z())),
			0.0, 1.0);
	const auto gx = std::min<std::size_t>(
			static_cast<std::size_t>(std::llround(xr * static_cast<double>(width - 1))),
			width - 1);
	const auto gz = std::min<std::size_t>(
			static_cast<std::size_t>(std::llround(zr * static_cast<double>(height - 1))),
			height - 1);
	return {gx, gz};
}

bool point_in_ring(double px, double pz, const std::vector<ProcessedNode> &ring)
{
	bool inside = false;
	for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
		const double xi = ring[i].x;
		const double zi = ring[i].z;
		const double xj = ring[j].x;
		const double zj = ring[j].z;
		if (((zi > pz) != (zj > pz)) && (px < (xj - xi) * (pz - zi) / (zj - zi) + xi))
			inside = !inside;
	}
	return inside;
}

void mark_grid_radius(
		land_cover::LandCoverData &lc, const XZBBox &xzbbox, int x, int z, int radius)
{
	if (lc.width == 0 || lc.height == 0)
		return;
	for (int dz = -radius; dz <= radius; ++dz) {
		for (int dx = -radius; dx <= radius; ++dx) {
			if (dx * dx + dz * dz > radius * radius)
				continue;
			const auto [gx, gz] =
					grid_index_for(x + dx, z + dz, xzbbox, lc.width, lc.height);
			lc.grid[gz][gx] = land_cover::LC_WATER;
		}
	}
}

void mark_polygon(land_cover::LandCoverData &lc, const XZBBox &xzbbox,
		const std::vector<ProcessedNode> &outer,
		const std::vector<std::vector<ProcessedNode>> &inners)
{
	if (!is_closed_ring(outer))
		return;
	int min_x = xzbbox.max_x();
	int max_x = xzbbox.min_x();
	int min_z = xzbbox.max_z();
	int max_z = xzbbox.min_z();
	for (const auto &n : outer) {
		min_x = std::min(min_x, n.x);
		max_x = std::max(max_x, n.x);
		min_z = std::min(min_z, n.z);
		max_z = std::max(max_z, n.z);
	}
	min_x = std::max(min_x, xzbbox.min_x());
	max_x = std::min(max_x, xzbbox.max_x());
	min_z = std::max(min_z, xzbbox.min_z());
	max_z = std::min(max_z, xzbbox.max_z());
	if (min_x > max_x || min_z > max_z)
		return;

	for (int z = min_z; z <= max_z; ++z) {
		for (int x = min_x; x <= max_x; ++x) {
			if (!point_in_ring(static_cast<double>(x) + 0.5, static_cast<double>(z) + 0.5,
						outer))
				continue;
			bool in_hole = false;
			for (const auto &inner : inners) {
				if (is_closed_ring(inner) &&
						point_in_ring(static_cast<double>(x) + 0.5,
								static_cast<double>(z) + 0.5, inner)) {
					in_hole = true;
					break;
				}
			}
			if (in_hole)
				continue;
			const auto [gx, gz] = grid_index_for(x, z, xzbbox, lc.width, lc.height);
			lc.grid[gz][gx] = land_cover::LC_WATER;
		}
	}
}

land_cover::LandCoverData build_osm_water_land_cover(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox)
{
	// Source selection is isolated here; ESAWorldCover can replace this provider
	// without changing bridge repair or ground-generation consumers.
	const auto width = static_cast<std::size_t>(xzbbox.max_x() - xzbbox.min_x() + 1);
	const auto height = static_cast<std::size_t>(xzbbox.max_z() - xzbbox.min_z() + 1);
	land_cover::LandCoverData lc;
	if (width == 0 || height == 0)
		return lc;
	lc.width = width;
	lc.height = height;
	lc.grid.assign(height, std::vector<uint8_t>(width, 0));

	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			const auto waterway = way.tags.find("waterway");
			if (is_water_polygon_way(way) && is_closed_ring(way.nodes)) {
				if (auto clipped = clipping::clip_water_ring_to_bbox(way.nodes, xzbbox))
					mark_polygon(lc, xzbbox, *clipped, {});
			} else if (waterway != way.tags.end() && !way.nodes.empty()) {
				const int radius = std::max(1, waterway_width(way) / 2);
				for (std::size_t i = 1; i < way.nodes.size(); ++i) {
					const auto points = bresenham_line(way.nodes[i - 1].x, 0,
							way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z);
					for (const auto &[x, y, z] : points) {
						(void)y;
						mark_grid_radius(lc, xzbbox, x, z, radius);
					}
				}
			}
		} else if (element.is_relation()) {
			const auto &rel = element.as_relation();
			if (!is_water_relation(rel))
				continue;
			std::vector<std::vector<ProcessedNode>> outers;
			std::vector<std::vector<ProcessedNode>> inners;
			for (const auto &member : rel.members) {
				if (member.way.nodes.empty())
					continue;
				if (member.role == ProcessedMemberRole::Inner)
					inners.push_back(member.way.nodes);
				else if (member.role == ProcessedMemberRole::Outer)
					outers.push_back(member.way.nodes);
			}
			stitch_way_segments(outers);
			stitch_way_segments(inners);
			for (const auto &outer : outers) {
				auto clipped = clipping::clip_water_ring_to_bbox(outer, xzbbox);
				if (clipped)
					mark_polygon(lc, xzbbox, *clipped, inners);
			}
		}
	}

	bool any = false;
	for (const auto &row : lc.grid) {
		if (std::find(row.begin(), row.end(), land_cover::LC_WATER) != row.end()) {
			any = true;
			break;
		}
	}
	if (!any) {
		lc.grid.clear();
		lc.width = 0;
		lc.height = 0;
	}
	return lc;
}

}

// Forward declarations for all element processing functions
namespace buildings
{
void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args);
void generate_building_from_relation(WorldEditor &editor,
		const ProcessedRelation &relation, const Args &args,
		const FloodFillCache &flood_fill_cache, const XZBBox &xzbbox,
		const CoordinateBitmap &building_passages);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels,
		const FloodFillCache &flood_fill_cache, const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons,
		std::optional<std::uint64_t> style_seed, const CoordinateBitmap *road_mask,
		const CoordinateBitmap *building_footprints,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members);
}

namespace highways
{
void generate_highways(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const std::vector<ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout);
void generate_highways(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const std::vector<ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const RoadMaskBitmap &road_mask,
		const bridges::BridgeStructureMap &bridge_structures,
		const bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints);
void generate_helipad_node(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints);
void generate_siding(WorldEditor &editor, const ProcessedWay &way);
void generate_siding(WorldEditor &editor, const ProcessedWay &way,
		const bridges::BridgeSurfaceMap &bridge_surface);
CoordinateBitmap collect_road_surface_coords(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor,
		const ::XZBBox &xzbbox, double scale);
CoordinateBitmap collect_building_passage_coords(
		const std::vector<ProcessedElement> &elements, const ::XZBBox &xzbbox,
		double scale);
}

namespace landuse
{
void generate_landuse(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface);
void generate_landuse_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const RoadMaskBitmap &road_mask, const bridges::BridgeSurfaceMap &bridge_surface);
void generate_place(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache);
}

namespace natural
{
void generate_natural(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_natural_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace amenities
{
void generate_amenities(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
void generate_amenities(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const FloodFillCache &flood_fill_cache,
		const RoadMaskBitmap &road_mask);
}

namespace leisure
{
void generate_leisure(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_leisure_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, FloodFillCache const &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace barriers
{
void generate_barriers(WorldEditor &editor, const ProcessedElement &element);
void generate_barriers(WorldEditor &editor, const ProcessedElement &element,
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_barrier_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_barrier_nodes(WorldEditor &editor, const ProcessedNode &node,
		const bridges::BridgeSurfaceMap &bridge_surface);
}

namespace waterways
{
void generate_waterways(WorldEditor &editor, const ProcessedWay &way);
}

namespace water_areas
{
void generate_water_areas_from_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const XZBBox &xzbbox,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint, std::optional<int> precomputed_surface);
void generate_water_area_from_way(WorldEditor &editor, const ProcessedWay &way,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint, std::optional<int> precomputed_surface);
}

namespace railways
{
using RailBridgeInternalEndpoints = std::vector<std::pair<int, int>>;
void generate_roller_coaster(WorldEditor &editor, const ProcessedWay &way);
void generate_railways(WorldEditor &editor, const ProcessedWay &element);
void generate_railways(WorldEditor &editor, const ProcessedWay &element,
		std::vector<std::pair<int, int>> &subway_points,
		const RailBridgeInternalEndpoints &rail_bridge_internal_endpoints,
		const bridge_styles::BridgeOutlineIndex &bridge_outlines,
		const CoordinateBitmap &road_mask, const CoordinateBitmap &building_footprints,
		const CoordinateBitmap &rail_mask);
RailBridgeInternalEndpoints collect_rail_bridge_internal_endpoints(
		const std::vector<ProcessedElement> &elements);
CoordinateBitmap collect_at_grade_rail_mask(
		const std::vector<ProcessedElement> &elements, const XZBBox &xzbbox);
void add_tunnel_footprint(const std::vector<ProcessedElement> &elements,
		const XZBBox &xzbbox, CoordinateBitmap &footprint);
void carve_subway_interior(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &subway_points);
}

namespace tourisms
{
void generate_tourisms(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask);
}

namespace man_made
{
void generate_man_made(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_man_made_nodes(
		WorldEditor &editor, const ProcessedNode &node, const Args &args);
}

namespace doors
{
void generate_doors(WorldEditor &editor, const ProcessedNode &rel);
}

namespace historic
{
void generate_pyramid(WorldEditor &editor, const ProcessedWay &way, const Args &args);
void generate_historic(WorldEditor &editor, const ProcessedNode &node);
}

namespace power
{
void generate_power(WorldEditor &editor, const ProcessedElement &element);
void generate_power_nodes(WorldEditor &editor, const ProcessedNode &node);
}

namespace emergency
{
void generate_emergency(WorldEditor &editor, const ProcessedNode &node);
}

namespace advertising
{
void generate_advertising(WorldEditor &editor, const ProcessedNode &node);
}

// Main generate_world function
bool generate_world(WorldEditor &editor,
		const std::vector<ProcessedElement> &input_elements, const Args &args_,
		FloodFillCache &flood_fill_cache,
		BuildingFootprintBitmap const &building_footprints)
{
	if (!valid_scale(args_.scale))
		return false;
	editor.reserve_ground_level_cache();
	world_editor::set_world_bounds(
			args_.disable_height_limit && !args_.bedrock ? -2032 : -64,
			args_.disable_height_limit && !args_.bedrock ? 2031 : 319);
	const int base_level = editor.ground ? editor.ground->base_level(args_.ground_level)
										 : args_.ground_level;
	world_editor::set_terrain_floor_y(base_level);
	world_editor::set_base_chunk_y(base_level);
	static const std::vector<ProcessedElement> no_elements;
	const auto &elements = args_.skip_objects() ? no_elements : input_elements;
	const auto geo = editor.geographic_bounds();
	const double centre_lat = (geo[0] + geo[1]) * .5, centre_lon = (geo[2] + geo[3]) * .5;
	if (editor.map_decals && !editor.decal_registry)
		editor.set_decal_registry(signage::build_registry(
				elements, args_.signage, decals::detect_region(centre_lat, centre_lon),
				args_.scale));
	if (editor.decal_registry && !editor.decal_frame_sink) {
		editor.set_decal_frame_sink([&editor](const WorldEditor::DecalFrame &frame) {
			const auto texture = decal_texture(*editor.decal_registry, frame.map_id);
			if (!texture)
				return false;
			Block node = block_definitions::DECAL_FRAME;
			editor.set_block_absolute(node, frame.x, frame.y, frame.z,
					std::nullopt, std::nullopt);
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++11-narrowing"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnarrowing"
#endif
			return editor.mg && editor.mg->queueGeneratedDecal(
					{frame.x, frame.y, frame.z},
					*texture, frame.map_id, frame.facing, frame.rotation, frame.glow);
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
		});
	}
	if (auto selector = trees::RegionSelector::load_for_location(centre_lat, centre_lon,
				std::filesystem::path("assets/trees"), args_.scale, base_level)) {
		auto shared_selector =
				std::make_shared<trees::RegionSelector>(std::move(*selector));
		editor.set_tree_slot_spacing(shared_selector->base_spacing());
		editor.set_regional_tree_placer(
				[&editor, shared_selector](int x, int y, int z, std::uint8_t cover) {
					return trees::place_selected_region_tree_for_cover(
							editor, *shared_selector, x, z, cover, y);
				});
	}
	models_3d::RemoteModelProvider wikidata_provider(
			std::filesystem::path(".arnis_wikidata_cache"));
	models_3d::three_dmr::Client three_dmr_provider(
			std::filesystem::path(".arnis_3dmr_cache"));
	models_3d::custom::Client custom_model_provider(
			std::filesystem::path("assets/models"));
	// Landmarks use their matched OSM feature's projected centre as the world
	// anchor.  This keeps the C++ mapgen host independent of Rust's geographic
	// projection plumbing while preserving the same suppression/late-placement
	// ordering.
	std::vector<landmarks::WorldAnchor> landmark_anchors;
	for (const auto &landmark : landmarks::catalogue()) {
		for (const auto &element : elements) {
			auto wikidata = element.tags().find("wikidata");
			const auto key = std::pair<std::string, std::uint64_t>{
					std::string(element.kind()), element.id()};
			const bool named_match =
					wikidata != element.tags().end() && wikidata->second == landmark.qid;
			const bool osm_match =
					std::find(landmark.osm_ids.begin(), landmark.osm_ids.end(), key) !=
					landmark.osm_ids.end();
			if (!named_match && !osm_match)
				continue;
			std::vector<std::pair<int, int>> points;
			if (element.is_node())
				points.emplace_back(element.as_node().x, element.as_node().z);
			else if (element.is_way())
				for (const auto &n : element.as_way().nodes)
					points.emplace_back(n.x, n.z);
			else
				for (const auto &m : element.as_relation().members)
					for (const auto &n : m.way.nodes)
						points.emplace_back(n.x, n.z);
			if (points.empty())
				continue;
			std::int64_t sx = 0, sz = 0;
			for (const auto &[x, z] : points) {
				sx += x;
				sz += z;
			}
			landmark_anchors.push_back(
					{landmark.qid, static_cast<int>(sx / points.size()),
							static_cast<int>(sz / points.size())});
			break;
		}
	}
	auto landmark_plan = landmarks::prescan(elements, landmark_anchors, args_.scale);
	const auto model_pipeline =
			args_.use_3d
					? std::optional<models_3d::Models3dPipeline>(
							  models_3d::Models3dPipeline::prescan_fetchable_models(
									  elements, args_.scale,
									  [&](const std::string &key) {
										  return wikidata_provider.fetch(key).has_value();
									  },
									  [&]() {
										  return custom_model_provider
												  .fetch("stadium.glb")
												  .has_value();
									  },
									  0.0, landmark_plan.suppressed))
					: std::nullopt;
	// Match Rust's specificity ordering: broad ground-cover polygons render
	// first, allowing smaller nested areas to overwrite them later.
	auto ordered_elements = elements;
	sort_ground_fill_areas(ordered_elements);
	const auto &render_elements = ordered_elements;
	// A fill may be shared by its standalone way and a later relation.  Keep it
	// only through its last reader, then release it just as the Rust sequential
	// path does; this prevents large worlds retaining every polygon fill.
	const auto last_fill_use = compute_last_fill_use(render_elements);
	auto [min_x, min_z] = editor.get_min_coords();
	auto [max_x, max_z] = editor.get_max_coords();
	::XZBBox xzbbox(min_x, min_z, max_x, max_z);
	if (editor.ground && !editor.ground->has_land_cover()) {
		auto land_cover = build_osm_water_land_cover(elements, xzbbox);
		if (land_cover.width > 0 && land_cover.height > 0) {
			editor.ground->set_land_cover_data(std::move(land_cover),
					static_cast<std::size_t>(max_x - min_x + 1),
					static_cast<std::size_t>(max_z - min_z + 1));
		}
	}
	if (editor.ground && editor.ground->has_land_cover()) {
		auto &land_cover = *editor.ground->land_cover;
		const auto [world_width, world_height] = editor.ground->world_dims();
		std::vector<std::vector<float>> cover_heights(
				land_cover.height, std::vector<float>(land_cover.width));
		for (std::size_t z = 0; z < land_cover.height; ++z)
			for (std::size_t x = 0; x < land_cover.width; ++x) {
				const int world_x =
						min_x + static_cast<int>(std::llround(
										(double(x) / std::max<std::size_t>(
															 1, land_cover.width - 1)) *
										(world_width - 1)));
				const int world_z =
						min_z + static_cast<int>(std::llround(
										(double(z) / std::max<std::size_t>(
															 1, land_cover.height - 1)) *
										(world_height - 1)));
				cover_heights[z][x] = editor.ground->level({world_x, world_z});
			}
		land_cover::apply_osm_water_override(
				land_cover, cover_heights, world_width, world_height, elements, xzbbox);
		land_cover::apply_osm_land_override(
				land_cover, world_width, world_height, elements, xzbbox, args_.scale);
		land_cover::apply_bridge_land_cover_repair(land_cover, cover_heights, world_width,
				world_height, elements, xzbbox, args_.scale);
	}
	auto road_mask = highways::collect_road_surface_coords(elements, editor, xzbbox,
			args_.scale);
	auto big_water_field = water_depth::compute_big_water_field(editor, xzbbox);
	// Pre-scan still water surfaces for consistent water levels across tiles
	auto still_surfaces =
			editor.ground ? prescan_still_surfaces(elements, editor.ground, xzbbox)
						  : StillWaterSurfaces{};
	auto bridge_outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	auto bridge_structures =
			bridges::BridgeStructureMap::build(elements, editor, bridge_outlines);
	auto bridge_surface =
			bridges::BridgeSurfaceMap::build(elements, bridge_structures, args_.scale);
	auto building_passages =
			highways::collect_building_passage_coords(elements, xzbbox, args_.scale);
	std::vector<std::pair<int, int>> subway_points;
	std::vector<highways::HighwayTunnelCell> highway_tunnel_cells;
	auto tunnel_internal_endpoints =
			highways::collect_tunnel_internal_endpoints(elements, xzbbox);
	auto tunnel_portals = highways::collect_tunnel_portals(
			elements, editor, bridge_structures, tunnel_internal_endpoints, args_.scale);
	auto tunnel_footprint = highways::collect_tunnel_footprint(
			elements, editor, tunnel_internal_endpoints, xzbbox, args_.scale);
	railways::add_tunnel_footprint(elements, xzbbox, tunnel_footprint);
	auto rail_bridge_internal_endpoints =
			railways::collect_rail_bridge_internal_endpoints(elements);
	railways::advtrains::prepare_network(elements, editor);
	// Centreline bitmap prevents catenary masts from being stamped into a
	// neighbouring parallel track, matching the Rust railway pass.
	auto rail_mask = railways::collect_at_grade_rail_mask(elements, xzbbox);

	// Pre-scan: detect building relation outlines that should be suppressed.
	// Only applies to type=building relations (NOT type=multipolygon).
	// When a type=building relation has "part" members, the outline way should not
	// render as a standalone building, the individual parts render instead.
	std::unordered_set<uint64_t> suppressed_building_outlines;
	std::unordered_map<uint64_t, uint64_t> building_part_groups;
	std::unordered_map<uint64_t, std::vector<uint64_t>> building_group_members;
	for (const auto &element : elements) {
		if (element.is_relation()) {
			const auto &rel = element.as_relation();
			auto it_type = rel.tags.find("type");
			bool is_building_type =
					(it_type != rel.tags.end() && it_type->second == "building");

			if (is_building_type) {
				for (const auto &member : rel.members)
					if (member.role == ProcessedMemberRole::Part) {
						building_part_groups.emplace(member.way.id, rel.id);
						building_group_members[rel.id].push_back(member.way.id);
					}
				bool has_parts = false;
				for (const auto &member : rel.members) {
					if (member.role == ProcessedMemberRole::Part) {
						has_parts = true;
						break;
					}
				}

				if (has_parts) {
					for (const auto &member : rel.members) {
						if (member.role == ProcessedMemberRole::Outer) {
							suppressed_building_outlines.insert(member.way.id);
						}
					}
				}
			}
		}
	}

	for (std::size_t element_index = 0; element_index < render_elements.size();
			++element_index) {
		auto const &element = render_elements[element_index];
		auto args = args_;
		if (model_pipeline &&
				std::find(model_pipeline->suppressed().begin(),
						model_pipeline->suppressed().end(),
						std::pair<std::string, std::uint64_t>{std::string(element.kind()),
								element.id()}) !=
						model_pipeline->suppressed().end()) {
			release_finished_fills(
					flood_fill_cache, last_fill_use, element, element_index);
			continue;
		}

		if (element.is_way()) {
			auto const &way = element.as_way();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{
								"way", way.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(
						flood_fill_cache, last_fill_use, element, element_index);
				continue;
			}
			// A successfully placed 3D model replaces the source OSM feature;
			// rendering both was a C++-only duplication absent from Rust.
			if (way.id == 1486752423ULL)
				structures::place_starship(editor, way);

			if (editor.ground && !way.nodes.empty()) {
				args.ground_level = editor.ground->level(way.nodes.begin()->xz());
			}

			// Solar farms are commonly mapped as barrier=fence plus
			// power=generator.  The barrier handler would otherwise shadow the
			// generator, unlike the Rust dispatcher.
			if (way.tags.contains("barrier") && !way.tags.contains("building") &&
					way.tags.get("power") == std::optional<std::string>("generator")) {
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);
			}

			if (way.tags.contains("building") || way.tags.contains("building:part")) {
				// Skip building outlines that are suppressed by building relations with parts.
				// The individual building:part ways will render instead.
				if (suppressed_building_outlines.find(way.id) ==
						suppressed_building_outlines.end()) {
					auto group = building_part_groups.find(way.id);
					buildings::generate_buildings(&editor, way, args,
							std::optional<int>{}, flood_fill_cache, building_passages,
							nullptr,
							group == building_part_groups.end()
									? std::nullopt
									: std::optional<std::uint64_t>(group->second),
							&road_mask, &building_footprints, &building_group_members);
				}
			} else if (way.tags.contains("highway")) {
				const bool tunnel_rendered =
						highways::renders_as_highway_tunnel(way) &&
						highways::generate_highway_tunnel_shell(editor, way, args,
								tunnel_internal_endpoints, tunnel_portals,
								highway_tunnel_cells);
				if (editor.signage_enabled())
					signage::generate_highway_way_signage(
							editor, way, building_footprints, road_mask);
				if (!tunnel_rendered)
					highways::generate_highways(editor, element, args, elements, {},
							road_mask, bridge_structures, bridge_surface, tunnel_portals);
			} else if (way.tags.contains("landuse")) {
				landuse::generate_landuse(editor, way, args, flood_fill_cache,
						building_footprints, road_mask, bridge_surface);
			} else if (way.tags.contains("natural") &&
					   way.tags.get("amenity") !=
							   std::optional<std::string>(std::string("fountain"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("amenity")) {
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
				if (editor.signage_enabled() && way.tags.get("amenity") == "parking") {
					signage::generate_parking_signage(editor, way, road_mask);
				}
			} else if (way.tags.contains("leisure")) {
				leisure::generate_leisure(editor, way, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (way.tags.contains("barrier")) {
				barriers::generate_barriers(editor, element, bridge_surface);
			} else if (way.tags.contains("waterway")) {
				auto it_val = way.tags.find("waterway");
				if (it_val != way.tags.end() && it_val->second == "dock") {
					// docks count as water areas
					std::optional<int> surface_level;
					if (still_surfaces.has("way", way.id))
						surface_level = still_surfaces.get("way", way.id);
					water_areas::generate_water_area_from_way(editor, way,
							big_water_field, road_mask,
							tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
							surface_level);
				} else {
					waterways::generate_waterways(editor, way);
				}
			} else if (way.tags.contains("bridge")) {
				// Bridge members are rendered by the highway/rail passes, which also
				// apply relation-aware deck heights and schematic modules.
			} else if (way.tags.contains("railway")) {
				railways::generate_railways(editor, way, subway_points,
						rail_bridge_internal_endpoints, bridge_outlines, road_mask,
						building_footprints, rail_mask);
			} else if (way.tags.contains("roller_coaster")) {
				railways::generate_roller_coaster(editor, way);
			} else if (way.tags.contains("aeroway") ||
					   way.tags.contains("area:aeroway")) {
				highways::generate_aeroway(editor, way, args, building_footprints);
			} else if (way.tags.get("service") ==
					   std::optional<std::string>(std::string("siding"))) {
				highways::generate_siding(editor, way, bridge_surface);
			} else if (way.tags.get("tomb") ==
					   std::optional<std::string>(std::string("pyramid"))) {
				historic::generate_pyramid(editor, way, args);
			} else if (way.tags.contains("man_made")) {
				man_made::generate_man_made(editor, element, args);
			} else if (way.tags.contains("power")) {
				power::generate_power(editor, element, building_footprints,
						flood_fill_cache, args.timeout);
			} else if (way.tags.contains("place")) {
				if (editor.signage_enabled()) {
					signage::generate_power_signage(editor, way, road_mask);
				}
				landuse::generate_place(editor, way, args, flood_fill_cache);
			}
			// Highway-specific signage was already placed above with roadside
			// positioning. The generic pass would duplicate it at the first node.
			if (!way.tags.contains("highway"))
				signage::place_way_signage(
						editor, way, decals::detect_region(centre_lat, centre_lon));
		} else if (element.is_node()) {
			auto const &node = element.as_node();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{
								"node", node.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(
						flood_fill_cache, last_fill_use, element, element_index);
				continue;
			}
			if (node.tags.get("aeroway") == std::optional<std::string>("helipad"))
				structures::maybe_place_helicopter(editor, node.x, node.z);

			if (editor.ground)
				args.ground_level = editor.ground->level(node.xz());

			if (node.tags.contains("door") || node.tags.contains("entrance")) {
				doors::generate_doors(editor, node);
			} else if (node.tags.contains("natural") &&
					   node.tags.get("natural") ==
							   std::optional<std::string>(std::string("tree"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache,
						building_footprints, bridge_surface);
			} else if (node.tags.contains("amenity")) {
				amenities::generate_amenities(
						editor, element, args, flood_fill_cache, road_mask);
			} else if (node.tags.contains("barrier")) {
				barriers::generate_barrier_nodes(editor, node, bridge_surface);
			} else if (node.tags.contains("highway")) {
				highways::generate_highways(editor, element, args, elements, {},
						road_mask, bridge_structures, bridge_surface, tunnel_portals);
			} else if (node.tags.get("aeroway") ==
					   std::optional<std::string>(std::string("helipad"))) {
				highways::generate_helipad_node(editor, node, args, building_footprints);
			} else if (node.tags.contains("tourism")) {
				tourisms::generate_tourisms(editor, node, road_mask);
			} else if (node.tags.contains("man_made")) {
				man_made::generate_man_made_nodes(editor, node, args);
			} else if (node.tags.contains("power")) {
				power::generate_power_nodes(editor, node);
			} else if (node.tags.contains("historic")) {
				historic::generate_historic(editor, node);
			} else if (node.tags.contains("emergency")) {
				emergency::generate_emergency(editor, node);
			} else if (node.tags.contains("advertising")) {
				advertising::generate_advertising(editor, node, road_mask);
			}
			if (editor.signage_enabled()) {
				signage::generate_node_signage(
						editor, node, building_footprints, road_mask);
			}
		} else if (element.is_relation()) {
			auto const &rel = element.as_relation();
			if (std::find(landmark_plan.suppressed.begin(),
						landmark_plan.suppressed.end(),
						std::pair<std::string, std::uint64_t>{
								"relation", rel.id}) !=
					landmark_plan.suppressed.end()) {
				release_finished_fills(
						flood_fill_cache, last_fill_use, element, element_index);
				continue;
			}

			if (editor.ground && !rel.members.empty() &&
					!rel.members.begin()->way.nodes.empty()) {
				args.ground_level = editor.ground->level(
						rel.members.begin()->way.nodes.begin()->xz());
			}

			bool is_building_relation =
					rel.tags.contains("building") || rel.tags.contains("building:part") ||
					(rel.tags.get("type") ==
							std::optional<std::string>(std::string("building")));

			if (is_building_relation) {
				buildings::generate_building_from_relation(
						editor, rel, args, flood_fill_cache, xzbbox, building_passages);
			} else if (rel.tags.contains("water") ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("water")) ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("bay"))) {
				std::optional<int> surface_level;
				if (still_surfaces.has("relation", rel.id))
					surface_level = still_surfaces.get("relation", rel.id);
				water_areas::generate_water_areas_from_relation(editor, rel,
						xzbbox,
						big_water_field, road_mask,
						tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint,
						surface_level);
			} else if (rel.tags.contains("natural")) {
				natural::generate_natural_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, bridge_surface);
			} else if (rel.tags.contains("landuse")) {
				landuse::generate_landuse_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, road_mask, bridge_surface);
			} else if (rel.tags.get("leisure") ==
					   std::optional<std::string>(std::string("park"))) {
				leisure::generate_leisure_from_relation(editor, rel, args,
						flood_fill_cache, building_footprints, bridge_surface);
			} else if (rel.tags.contains("man_made")) {
				man_made::generate_man_made(editor, ProcessedElement(rel), args);
			}
		}

		// Do this after processing: a relation may consume fills belonging to
		// member ways, so eviction before dispatch would cause a refill.
		release_finished_fills(flood_fill_cache, last_fill_use, element, element_index);
	}

	// Rust ordering: ground_generation runs before water_depth::carve_lc_water_pass.
	ground_generation::generate_ground_layer(editor, args_, xzbbox, building_footprints,
			tunnel_footprint.is_empty() ? nullptr : &tunnel_footprint);
	if (args_.fillground)
		ore_generation::generate_ores(
				editor, xzbbox.min_x(), xzbbox.max_x(), xzbbox.min_z(), xzbbox.max_z());

	water_depth::carve_lc_water_pass(
			editor, big_water_field, road_mask, tunnel_footprint);
	if (model_pipeline) {
		models_3d::place_three_dmr_prescan(
				three_dmr_provider, editor, model_pipeline->three_dmr(), args_.scale);
		models_3d::place_wikidata_prescan(
				wikidata_provider, editor, model_pipeline->wikidata(), args_.scale);
		models_3d::place_custom_models(
				*model_pipeline, custom_model_provider, editor, args_.scale);
	}
	landmarks::place_all(editor, landmark_plan, args_.scale);
	structures::scatter_boats(editor, min_x, min_z, max_x, max_z);

	if (!subway_points.empty()) {
		railways::carve_subway_interior(editor, subway_points);
	}
	if (!highway_tunnel_cells.empty())
		highways::carve_highway_tunnel_interior(editor, highway_tunnel_cells);
	// Mark the completed generation for the format-specific persistence layer.
	// Java/Bedrock/Luanti writers consume these lifecycle requests when wired by
	// their respective WorldEditor backends.
	editor.request_flush();
	editor.request_save();
	if (!editor.finalize_persistence())
		return false;

	return true;
}

}
