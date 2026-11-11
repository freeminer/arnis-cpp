#pragma once
#include <array>
#include <cstdint>
#include <utility>
#include <vector>
#include <tuple>
#include "../../arnis_adapter.h"
#include "floodfill_cache.h"
namespace arnis::clipping
{
inline constexpr std::uint64_t INVENTED_NODE_BASE = std::uint64_t{1} << 62;
bool is_invented_node_id(std::uint64_t id);
// Re-label clipped open-way endpoints moved away from their original OSM ends.
// The line clipper deliberately retains a nearby source endpoint ID; aerialway
// planning needs the cut to remain distinguishable from a real cable terminal.
void mark_cut_ends(std::vector<ProcessedNode> &nodes,
		const std::array<std::pair<int, int>, 2> &ends, std::uint64_t way_id);
// General Rust clip_way_to_bbox equivalent.  Closed ways remain explicitly
// closed after clipping; open ways are clipped segment-by-segment and never
// acquire an artificial flood-fillable ring.
std::vector<ProcessedNode> clip_way_to_bbox(
		const std::vector<ProcessedNode> &, const XZBBox &);
std::vector<ProcessedNode> clip_way_to_bbox_owned(
		std::vector<ProcessedNode>, const XZBBox &);
std::optional<std::vector<ProcessedNode>> clip_water_ring_to_bbox(
		const std::vector<ProcessedNode> &, const XZBBox &);
using OutlinePoint = std::tuple<std::uint64_t, double, double>;
using OutlineRun = std::vector<OutlinePoint>;
std::optional<std::vector<OutlineRun>> cut_line_to_outline(std::uint64_t way_id,
		const std::vector<OutlinePoint> &points, const XZBBox &world,
		std::uint64_t &cuts);
std::optional<std::vector<ProcessedWay>> cut_way_to_outline(
		const ProcessedWay &way, const XZBBox &world);
std::vector<ProcessedElement> cut_elements_to_outline(
		const std::vector<ProcessedElement> &elements, const XZBBox &world);
}
