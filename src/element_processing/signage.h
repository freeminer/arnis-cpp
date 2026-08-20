#pragma once
#include "../args.h"
#include "../coordinate_system/cartesian.h"
#include "../decals/registry.h"
#include "../osm_parser.h"
#include "../world_editor/world_editor.h"
#include <memory>
#include <unordered_set>
namespace arnis::signage
{
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
		decals::SignRegion region);
void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region);
void place_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		decals::SignRegion region);

// Building and POI signage
struct NameSign
{
	std::string text;
	decals::DecalKey key;
};
NameSign poi_name(const tags_t &tags, SignageLevel level);
NameSign house_number(const tags_t &tags);
void generate_building_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		std::optional<std::pair<int, int>> anchor, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &road_mask);
void generate_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask);
void generate_power_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask);
void generate_highway_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask);
void generate_parking_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask);

// Helper functions for decals and orientation
std::int8_t facing_for_dir(double dx, double dz);
std::int8_t opposite(std::int8_t f);
std::pair<int, int> right_dir(std::int8_t facing);
std::optional<std::pair<int, int>> get_nearest_road_block(int x, int z, int radius,
		const RoadMaskBitmap &road_mask);

// Billboard and advertising support
bool generate_billboard(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);
bool generate_column(WorldEditor &editor, const ProcessedNode &node);
bool generate_poster_box_posters(WorldEditor &editor, const ProcessedNode &node);
bool generate_information_board(WorldEditor &editor, const ProcessedNode &node,
		const RoadMaskBitmap &road_mask);

}
