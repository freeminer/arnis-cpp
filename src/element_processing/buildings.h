#pragma once
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "building_facade.h"

namespace arnis
{

namespace interior_uses
{
class InteriorUseIndex;
}

namespace buildings
{

// Shared underground predicate used by building generation and blueprint
// outlines, matching Rust's is_underground_building helper.
bool is_underground_building(const tags_t &tags);

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

struct HeightHint
{
	std::optional<double> height_m;
	std::optional<double> levels;
};

// Highest wall/roof point this generator can produce above the footprint.
// Used by elevation fitting for Overture hints before block placement.
std::optional<double> mapped_top_m(
		const tags_t &tags, HeightHint hint, double width_m, double length_m);

// Return the clipped, merged outer rings with the same synthetic IDs used by
// relation building generation. Consumers that attach per-wall metadata must
// project against these exact generated ways.
std::vector<ProcessedWay> facade_outer_rings(
		const ProcessedRelation &relation, const XZBBox &xzbbox);

void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args);
void generate_building_from_relation(WorldEditor &editor,
		const ProcessedRelation &relation, const Args &args,
		const FloodFillCache &flood_fill_cache, const XZBBox &xzbbox,
		const CoordinateBitmap &building_passages,
		const interior_uses::InteriorUseIndex *interior_index = nullptr,
		const RoadMaskBitmap *road_mask = nullptr,
		const BuildingFootprintBitmap *building_footprints = nullptr,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members = nullptr,
		const std::unordered_set<std::uint64_t> *housing_relation_parts = nullptr);
std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels);
std::optional<building_facade::FacadeAnchor> generate_buildings(WorldEditor *editor,
		const ProcessedWay &element, const Args &args,
		const std::optional<int> &relation_levels, const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_passages,
		const std::vector<HolePolygon> *hole_polygons = nullptr,
		std::optional<std::uint64_t> style_seed = std::nullopt,
		const CoordinateBitmap *road_mask = nullptr,
		const CoordinateBitmap *building_footprints = nullptr,
		const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>>
				*group_members = nullptr,
		const interior_uses::InteriorUseIndex *interior_index = nullptr);
}
}
