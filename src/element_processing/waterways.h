#pragma once

#include "../../../arnis_types.h"
#include "../args.h"
#include "../floodfill_cache.h"

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::water_depth
{
class BigWaterField;
}

namespace arnis::bridges
{
class BridgeSurfaceMap;
}

namespace arnis::waterways
{
int get_waterway_width(const std::string &waterway_type);
bool is_channel_waterway(const std::string &waterway_type);
bool is_underground_waterway(const tags_t &tags);
constexpr int MAX_WATERWAY_WIDTH = 128;
int waterway_width(const std::string &waterway_type, const tags_t &tags);
int capped_waterway_width(
		const std::string &waterway_type, const tags_t &tags, std::optional<int> cap);
struct ChannelBed
{
	const water_depth::BigWaterField &field;
	const RoadMaskBitmap &road_mask;
	const RoadMaskBitmap &tunnel_footprint;
	const bridges::BridgeSurfaceMap *piped_under_roads = nullptr;
};
void generate_waterways(world_editor::WorldEditor &, const ProcessedWay &);
void generate_waterways(
		world_editor::WorldEditor &, const ProcessedWay &, WaterDetail, double scale);
void generate_waterways(world_editor::WorldEditor &, const ProcessedWay &, WaterDetail,
		double scale, const ChannelBed &bed);
}
