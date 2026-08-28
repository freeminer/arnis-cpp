#pragma once
#include "../args.h"
#include "../decals/registry.h"
#include "../floodfill_cache.h"
#include "../osm_parser.h"
#include "building_facade.h"
#include "world_editor.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
namespace arnis::signage
{
struct StreetNameBlade
{
	std::string name;
	std::pair<double, double> direction{1.0, 0.0};
	int half_width{1};
};
struct IntersectionPost
{
	std::uint64_t owner_way{0};
	std::vector<StreetNameBlade> blades;
};
using IntersectionIndex = std::unordered_map<std::uint64_t, IntersectionPost>;

struct SignageContext
{
	std::shared_ptr<const decals::DecalRegistry> registry;
	SignageLevel level{SignageLevel::None};
	decals::SignRegion region{decals::SignRegion::Europe};
	IntersectionIndex intersections;
	double scale{1.0};
	RoadMaskBitmap carriageway;

	bool has(const decals::DecalKey &key) const
	{
		return registry && registry->contains(key);
	}
};

struct WaySigns
{
	std::optional<decals::DecalKey> speed, shield, no_entry, cycleway;
};
WaySigns highway_way_signs(const tags_t &tags, decals::SignRegion region);
std::optional<decals::DecalKey> highway_node_sign(const tags_t &tags, SignageLevel level);
std::optional<decals::DecalKey> power_sign(const tags_t &tags);
std::vector<decals::DecalKey> advertising_keys(const tags_t &tags, std::uint64_t id);
std::optional<decals::DecalKey> information_key(const ProcessedNode &node);
std::optional<decals::DecalKey> furniture_pictogram(const tags_t &tags);
std::shared_ptr<const decals::DecalRegistry> build_registry(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region, double scale);
std::shared_ptr<const SignageContext> build_context(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region, double scale, const RoadMaskBitmap &carriageway);
IntersectionIndex build_intersection_index(
		const std::vector<ProcessedElement> &elements, double scale);
void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region);
void place_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		decals::SignRegion region);

// Building and POI signage
struct NameSign
{
	std::string text;
	std::optional<decals::DecalKey> key;
};
std::optional<NameSign> poi_name(const tags_t &tags, SignageLevel level);
std::optional<decals::DecalKey> house_number(const tags_t &tags, SignageLevel level);
void generate_building_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const std::optional<building_facade::FacadeAnchor> &anchor);
void generate_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask);
void generate_node_facade_signage(world_editor::WorldEditor &editor,
		const ProcessedNode &node, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &road_mask);
void generate_power_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask);
void generate_highway_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const BuildingFootprintBitmap &footprints);
void generate_parking_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask);

// Billboard and advertising support
bool generate_billboard(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);
bool generate_column(WorldEditor &editor, const ProcessedNode &node);
bool generate_poster_box_posters(WorldEditor &editor, const ProcessedNode &node);
bool generate_information_board(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);
std::optional<decals::DecalKey> plaque_key(const tags_t &tags);
bool generate_plaque(WorldEditor &editor, const ProcessedNode &node);
void place_bus_stop_signs(WorldEditor &editor, const tags_t &tags,
		int x, int sign_y, int z);

}
