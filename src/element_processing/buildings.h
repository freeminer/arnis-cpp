#pragma once
#include <optional>
#include <unordered_map>
#include <vector>
#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis
{

namespace buildings
{

enum class BuildingCondition
{
	Normal,
	Construction,
	Disused,
	Abandoned,
	Ruined
};

enum class WallDepthStyle
{
	None,
	SubtlePilasters,
	ModernPillars,
	InstitutionalBands,
	IndustrialBeams,
	HistoricOrnate,
	ReligiousButtress,
	SkyscraperFins,
	GlassCurtain
};

enum class DetailTier
{
	Minimal,
	Standard,
	Enhanced,
	Landmark
};

enum class BuildingCategory
{
	Residential,
	House,
	Farm,
	Commercial,
	Office,
	Hotel,
	Industrial,
	Warehouse,
	School,
	Hospital,
	Religious,
	TallBuilding,
	GlassySkyscraper,
	GlassCornerSkyscraper,
	GridSkyscraper,
	ContemporarySkyscraper,
	ModernSkyscraper,
	MasonrySkyscraper,
	Historic,
	Tower,
	Garage,
	Shed,
	Greenhouse,
	Default
};

enum class ArchEra
{
	Unknown,
	HistoricOrnate,
	TraditionalPreWar,
	PostWarPanel,
	Contemporary
};

struct HolePolygon
{
	ProcessedWay way;
	bool add_walls{true};
};

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
		const std::vector<HolePolygon> *hole_polygons = nullptr,
		std::optional<std::uint64_t> style_seed = std::nullopt,
		const CoordinateBitmap *road_mask = nullptr,
		const CoordinateBitmap *building_footprints = nullptr,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members = nullptr);
}
}
