#pragma once

#include "../../../arnis_types.h"
#include "../args.h"
#include "../floodfill_cache.h"

#include <functional>

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
struct ChannelCentre
{
	int x{}, z{};
	std::size_t segment{}, step{}, steps{};
	bool operator==(const ChannelCentre &) const = default;
};

int get_waterway_width(const std::string &waterway_type);
bool is_channel_waterway(const std::string &waterway_type);
bool is_underground_waterway(const tags_t &tags);
constexpr int MAX_WATERWAY_WIDTH = 128;
int waterway_width(
		const std::string &waterway_type, const tags_t &tags, double scale = 1.0);
int capped_waterway_width(const std::string &waterway_type, const tags_t &tags,
		double scale, std::optional<int> cap);
inline int capped_waterway_width(
		const std::string &waterway_type, const tags_t &tags, std::optional<int> cap)
{
	return capped_waterway_width(waterway_type, tags, 1.0, cap);
}
std::pair<int, int> channel_reach(int width);
void channel_walk(const std::vector<ProcessedNode> &nodes, int width,
		const CoordinateBitmap *roads_at_grade, const XZBBox &area,
		const std::function<void(const ChannelCentre &, int, int)> &cell);
inline void channel_cells(const std::vector<ProcessedNode> &nodes, int width,
		const CoordinateBitmap *roads_at_grade, const XZBBox &area,
		const std::function<void(int, int)> &cell)
{
	channel_walk(nodes, width, roads_at_grade, area,
			[&](const ChannelCentre &, int x, int z) { cell(x, z); });
}
struct ChannelBed
{
	const water_depth::BigWaterField &field;
	const RoadMaskBitmap &road_mask;
	const RoadMaskBitmap &tunnel_footprint;
	const bridges::BridgeSurfaceMap *piped_under_roads = nullptr;
	const CoordinateBitmap *roads_at_grade = nullptr;
};
void generate_waterways(world_editor::WorldEditor &, const ProcessedWay &);
void generate_waterways(
		world_editor::WorldEditor &, const ProcessedWay &, WaterDetail, double scale);
void generate_waterways(world_editor::WorldEditor &, const ProcessedWay &, WaterDetail,
		double scale, const ChannelBed &bed);
}
