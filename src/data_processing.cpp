#include "data_processing.h"
#include <sys/types.h>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "../../arnis_adapter.h"
#include "bresenham.h"
#include "element_processing/historic.h"
#include "element_processing/power.h"
#include "element_processing/emergency.h"
#include "element_processing/advertising.h"
#include "element_processing/bridges.h"
#include "element_processing/buildings.h"
#include "floodfill_cache.h"
#include "ground_generation.h"
#include "land_cover.h"
#include "water_depth.h"

namespace arnis
{

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

bool is_water_polygon_way(const ProcessedWay &way)
{
	return tag_is(way.tags, "natural", "water") ||
		   tag_is(way.tags, "natural", "bay") ||
		   tag_is(way.tags, "waterway", "riverbank") ||
		   tag_is(way.tags, "landuse", "reservoir") ||
		   way.tags.contains("water");
}

bool is_water_relation(const ProcessedRelation &rel)
{
	return rel.tags.contains("water") ||
		   tag_is(rel.tags, "natural", "water") ||
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
					static_cast<double>(std::max<int>(1, xzbbox.max_x() - xzbbox.min_x())),
			0.0, 1.0);
	const double zr = std::clamp(static_cast<double>(z - xzbbox.min_z()) /
					static_cast<double>(std::max<int>(1, xzbbox.max_z() - xzbbox.min_z())),
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
		if (((zi > pz) != (zj > pz)) &&
				(px < (xj - xi) * (pz - zi) / (zj - zi) + xi))
			inside = !inside;
	}
	return inside;
}

void mark_grid_radius(land_cover::LandCoverData &lc, const XZBBox &xzbbox,
		int x, int z, int radius)
{
	if (lc.width == 0 || lc.height == 0)
		return;
	for (int dz = -radius; dz <= radius; ++dz) {
		for (int dx = -radius; dx <= radius; ++dx) {
			if (dx * dx + dz * dz > radius * radius)
				continue;
			const auto [gx, gz] = grid_index_for(x + dx, z + dz, xzbbox, lc.width, lc.height);
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
			if (!point_in_ring(static_cast<double>(x) + 0.5,
						static_cast<double>(z) + 0.5, outer))
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
	// Rust parity gap: src/land_cover/mod.rs ESA WorldCover COG fetch is not ported.
	// This minimal grid mirrors src/land_cover/osm_water_override.rs for OSM water.
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
				mark_polygon(lc, xzbbox, way.nodes, {});
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
			for (const auto &outer : outers)
				mark_polygon(lc, xzbbox, outer, inners);
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
void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args,
		const FloodFillCache &flood_fill_cache, const XZBBox &xzbbox,
		const CoordinateBitmap &building_passages);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels,
		const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons);
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
		const bridges::BridgeSurfaceMap &bridge_surface);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args);
void generate_siding(WorldEditor &editor, const ProcessedWay &way);
void generate_siding(WorldEditor &editor, const ProcessedWay &way,
		const bridges::BridgeSurfaceMap &bridge_surface);
CoordinateBitmap collect_road_surface_coords(
		const std::vector<ProcessedElement> &elements, const ::XZBBox &xzbbox, double scale);
CoordinateBitmap collect_building_passage_coords(
		const std::vector<ProcessedElement> &elements, const ::XZBBox &xzbbox, double scale);
}

namespace landuse
{
void generate_landuse(WorldEditor &editor, const ProcessedWay &way, const Args &args, 
                     FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
void generate_landuse_from_relation(
		WorldEditor &editor, const ProcessedRelation &rel, const Args &args, 
        FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
void generate_place(WorldEditor &editor, const ProcessedWay &way, const Args &args, 
                   FloodFillCache const & flood_fill_cache);
}

namespace natural
{
void generate_natural(
		WorldEditor &editor, const ProcessedElement &element, const Args &args, 
        FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
void generate_natural_from_relation(
		WorldEditor &editor, const ProcessedRelation &rel, const Args &args, 
        FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
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
                     FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
void generate_leisure_from_relation(
		WorldEditor &editor, const ProcessedRelation &rel, const Args &args, 
        FloodFillCache const & flood_fill_cache, BuildingFootprintBitmap const & building_footprints);
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
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask);
void generate_water_area_from_way(WorldEditor &editor, const ProcessedWay &way,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask);
}

namespace railways
{
using RailBridgeInternalEndpoints = std::vector<std::pair<int, int>>;
void generate_roller_coaster(WorldEditor &editor, const ProcessedWay &way);
void generate_railways(WorldEditor &editor, const ProcessedWay &element);
void generate_railways(WorldEditor &editor, const ProcessedWay &element,
		std::vector<std::pair<int, int>> &subway_points,
		const RailBridgeInternalEndpoints &rail_bridge_internal_endpoints,
		const bridge_styles::BridgeOutlineIndex &bridge_outlines);
RailBridgeInternalEndpoints collect_rail_bridge_internal_endpoints(
		const std::vector<ProcessedElement> &elements);
void carve_subway_interior(WorldEditor &editor,
		const std::vector<std::pair<int, int>> &subway_points);
}

namespace tourisms
{
void generate_tourisms(WorldEditor &editor, const ProcessedNode &node);
}

namespace man_made
{
void generate_man_made(
		WorldEditor &editor, const ProcessedElement &element, const Args &args);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node);
void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node,
		const Args &args);
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
bool generate_world(WorldEditor &editor, const std::vector<ProcessedElement> &elements,
		const Args &args_, FloodFillCache const & flood_fill_cache, 
        BuildingFootprintBitmap const & building_footprints)
{
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
	auto road_mask = highways::collect_road_surface_coords(elements, xzbbox, args_.scale);
	auto big_water_field = water_depth::compute_big_water_field(editor, xzbbox);
	auto bridge_outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	auto bridge_structures = bridges::BridgeStructureMap::build(elements, editor, bridge_outlines);
	auto bridge_surface = bridges::BridgeSurfaceMap::build(elements, bridge_structures, args_.scale);
	auto building_passages = highways::collect_building_passage_coords(elements, xzbbox, args_.scale);
	std::vector<std::pair<int, int>> subway_points;
	auto rail_bridge_internal_endpoints =
			railways::collect_rail_bridge_internal_endpoints(elements);

	// Pre-scan: detect building relation outlines that should be suppressed.
	// Only applies to type=building relations (NOT type=multipolygon).
	// When a type=building relation has "part" members, the outline way should not
	// render as a standalone building, the individual parts render instead.
	std::unordered_set<uint64_t> suppressed_building_outlines;
	for (const auto &element : elements) {
		if (element.is_relation()) {
			const auto &rel = element.as_relation();
			auto it_type = rel.tags.find("type");
			bool is_building_type = (it_type != rel.tags.end() && it_type->second == "building");
			
			if (is_building_type) {
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

	for (auto const &element : elements) {
		auto args = args_;

		if (element.is_way()) {
			auto const &way = element.as_way();

			if (!way.nodes.empty()) {
				args.ground_level = editor.ground->level(way.nodes.begin()->xz());
			}

			if (way.tags.contains("building") || way.tags.contains("building:part")) {
				// Skip building outlines that are suppressed by building relations with parts.
				// The individual building:part ways will render instead.
				if (suppressed_building_outlines.find(way.id) == suppressed_building_outlines.end()) {
					buildings::generate_buildings(&editor, way, args, std::optional<int>{},
							flood_fill_cache, building_passages, nullptr);
				}
			} else if (way.tags.contains("highway")) {
				highways::generate_highways(editor, element, args, elements, {},
						road_mask, bridge_structures, bridge_surface);
			} else if (way.tags.contains("landuse")) {
				landuse::generate_landuse(editor, way, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("natural") &&
					   way.tags.get("amenity") !=
							   std::optional<std::string>(std::string("fountain"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("amenity")) {
				amenities::generate_amenities(editor, element, args, flood_fill_cache, road_mask);
			} else if (way.tags.contains("leisure")) {
				leisure::generate_leisure(editor, way, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("barrier")) {
				barriers::generate_barriers(editor, element, bridge_surface);
			} else if (way.tags.contains("waterway")) {
				auto it_val = way.tags.find("waterway");
				if (it_val != way.tags.end() && it_val->second == "dock") {
					// docks count as water areas
					water_areas::generate_water_area_from_way(
							editor, way, big_water_field, road_mask);
				} else {
					waterways::generate_waterways(editor, way);
				}
			} else if (way.tags.contains("bridge")) {
				// bridges::generate_bridges(editor, way, ground_level); // TODO FIX
			} else if (way.tags.contains("railway")) {
				railways::generate_railways(
						editor, way, subway_points, rail_bridge_internal_endpoints, bridge_outlines);
			} else if (way.tags.contains("roller_coaster")) {
				railways::generate_roller_coaster(editor, way);
			} else if (way.tags.contains("aeroway") ||
					   way.tags.contains("area:aeroway")) {
				highways::generate_aeroway(editor, way, args);
			} else if (way.tags.get("service") ==
					   std::optional<std::string>(std::string("siding"))) {
				highways::generate_siding(editor, way, bridge_surface);
			} else if (way.tags.get("tomb") ==
					   std::optional<std::string>(std::string("pyramid"))) {
				historic::generate_pyramid(editor, way, args);
			} else if (way.tags.contains("man_made")) {
				man_made::generate_man_made(editor, element, args);
			} else if (way.tags.contains("power")) {
				power::generate_power(editor, element);
			} else if (way.tags.contains("place")) {
				landuse::generate_place (editor, way, args, flood_fill_cache);
			}
		} else if (element.is_node()) {
			auto const &node = element.as_node();

			args.ground_level = editor.ground->level(node.xz());

			if (node.tags.contains("door") || node.tags.contains("entrance")) {
				doors::generate_doors(editor, node);
			} else if (node.tags.contains("natural") &&
					   node.tags.get("natural") ==
							   std::optional<std::string>(std::string("tree"))) {
				natural::generate_natural(editor, element, args, flood_fill_cache, building_footprints);
			} else if (node.tags.contains("amenity")) {
				amenities::generate_amenities(editor, element, args, flood_fill_cache, road_mask);
			} else if (node.tags.contains("barrier")) {
				barriers::generate_barrier_nodes(editor, node, bridge_surface);
			} else if (node.tags.contains("highway")) {
				highways::generate_highways(editor, element, args, elements, {},
						road_mask, bridge_structures, bridge_surface);
			} else if (node.tags.contains("tourism")) {
				tourisms::generate_tourisms(editor, node);
			} else if (node.tags.contains("man_made")) {
				man_made::generate_man_made_nodes(editor, node, args);
			} else if (node.tags.contains("power")) {
				power::generate_power_nodes(editor, node);
			} else if (node.tags.contains("historic")) {
				historic::generate_historic(editor, node);
			} else if (node.tags.contains("emergency")) {
				emergency::generate_emergency(editor, node);
			} else if (node.tags.contains("advertising")) {
				advertising::generate_advertising(editor, node);
			}
		} else if (element.is_relation()) {
			auto const &rel = element.as_relation();

			if (!rel.members.empty() && !rel.members.begin()->way.nodes.empty()) {
				args.ground_level = editor.ground->level(
						rel.members.begin()->way.nodes.begin()->xz());
			}

			bool is_building_relation = rel.tags.contains("building") ||
									   rel.tags.contains("building:part") ||
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
				water_areas::generate_water_areas_from_relation(
						editor, rel, big_water_field, road_mask);
			} else if (rel.tags.contains("natural")) {
				natural::generate_natural_from_relation(editor, rel, args, flood_fill_cache, building_footprints);
			} else if (rel.tags.contains("landuse")) {
				landuse::generate_landuse_from_relation(editor, rel, args, flood_fill_cache, building_footprints);
			} else if (rel.tags.get("leisure") ==
					   std::optional<std::string>(std::string("park"))) {
				leisure::generate_leisure_from_relation(editor, rel, args, flood_fill_cache, building_footprints);
			} else if (rel.tags.contains("man_made")) {
				man_made::generate_man_made(editor, ProcessedElement(rel), args);
			}
		}
	}

	// Rust ordering: ground_generation runs before water_depth::carve_lc_water_pass.
	ground_generation::generate_ground_layer(editor, args_, xzbbox, building_footprints);

	water_depth::carve_lc_water_pass(editor, big_water_field, road_mask);

	if (!subway_points.empty()) {
		railways::carve_subway_interior(editor, subway_points);
	}

	return true;
}

}
