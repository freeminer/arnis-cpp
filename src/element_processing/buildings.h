#pragma once
#include <optional>
#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis
{

namespace buildings
{

struct HolePolygon {
	ProcessedWay way;
	bool add_walls{true};
};

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
		const std::vector<HolePolygon> *hole_polygons = nullptr);
}
}
