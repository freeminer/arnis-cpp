#include "data_processing.h"
#include <sys/types.h>
#include <unordered_set>
#include <memory>

#include "../../arnis_adapter.h"
#include "element_processing/historic.h"
#include "element_processing/power.h"
#include "element_processing/emergency.h"
#include "element_processing/advertising.h"
#include "floodfill_cache.h"

namespace arnis
{

// Forward declarations for all element processing functions
namespace buildings
{
void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels);
}

namespace highways
{
void generate_highways(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const std::vector<ProcessedElement> &all_elements, 
		const std::optional<std::chrono::duration<double>> &floodfill_timeout);
void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args);
void generate_siding(WorldEditor &editor, const ProcessedWay &way);
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
void generate_barrier_nodes(WorldEditor &editor, const ProcessedNode &node);
}

namespace waterways
{
void generate_waterways(WorldEditor &editor, const ProcessedWay &way);
}

namespace water_areas
{
void generate_water_areas_from_relation(WorldEditor &editor, const ProcessedRelation &rel);
void generate_water_area_from_way(WorldEditor &editor, const ProcessedWay &way);
}

namespace railways
{
void generate_roller_coaster(WorldEditor &editor, const ProcessedWay &way);
void generate_railways(WorldEditor &editor, const ProcessedWay &element);
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
					buildings::generate_buildings(&editor, way, args, std::optional<int>{});
				}
			} else if (way.tags.contains("highway")) {
				highways::generate_highways(editor, element, args, elements, {});
			} else if (way.tags.contains("landuse")) {
				landuse::generate_landuse(editor, way, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("natural")) {
				natural::generate_natural(editor, element, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("amenity")) {
				amenities::generate_amenities(editor, element, args);
			} else if (way.tags.contains("leisure")) {
				leisure::generate_leisure(editor, way, args, flood_fill_cache, building_footprints);
			} else if (way.tags.contains("barrier")) {
				barriers::generate_barriers(editor, element);
			} else if (way.tags.contains("waterway")) {
				auto it_val = way.tags.find("waterway");
				if (it_val != way.tags.end() && it_val->second == "dock") {
					// docks count as water areas
					water_areas::generate_water_area_from_way(editor, way);
				} else {
					waterways::generate_waterways(editor, way);
				}
			} else if (way.tags.contains("bridge")) {
				// bridges::generate_bridges(editor, way, ground_level); // TODO FIX
			} else if (way.tags.contains("railway")) {
				railways::generate_railways(editor, way);
			} else if (way.tags.contains("roller_coaster")) {
				railways::generate_roller_coaster(editor, way);
			} else if (way.tags.contains("aeroway") ||
					   way.tags.contains("area:aeroway")) {
				highways::generate_aeroway(editor, way, args);
			} else if (way.tags.get("service") ==
					   std::optional<std::string>(std::string("siding"))) {
				highways::generate_siding(editor, way);
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
				amenities::generate_amenities(editor, element, args);
			} else if (node.tags.contains("barrier")) {
				barriers::generate_barrier_nodes(editor, node);
			} else if (node.tags.contains("highway")) {
				highways::generate_highways(editor, element, args, elements, {});
			} else if (node.tags.contains("tourism")) {
				tourisms::generate_tourisms(editor, node);
			} else if (node.tags.contains("man_made")) {
				man_made::generate_man_made_nodes(editor, node);
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
				buildings::generate_building_from_relation(editor, rel, args);
			} else if (rel.tags.contains("water") ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("water")) ||
					   rel.tags.get("natural") ==
							   std::optional<std::string>(std::string("bay"))) {
				water_areas::generate_water_areas_from_relation(editor, rel);
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

	return true;
}

}