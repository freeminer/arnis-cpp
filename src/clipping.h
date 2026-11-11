#pragma once
#include <array>
#include <cstdint>
#include <optional>
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

// C++ equivalent of Rust's Cow-backed cut_ways_to_outline: ways which need no
// outline cut stay borrowed, while clipped pieces own their generated nodes.
// Keep the returned vector alive while using way() references.
template <typename T>
struct OutlineWay
{
	const ProcessedWay *borrowed{};
	std::optional<ProcessedWay> owned;
	T value;

	const ProcessedWay &way() const { return owned ? *owned : *borrowed; }
};

template <typename T>
std::vector<OutlineWay<T>> cut_ways_to_outline(
		const std::vector<std::pair<const ProcessedWay *, T>> &ways, const XZBBox &world)
{
	std::vector<OutlineWay<T>> result;
	result.reserve(ways.size());
	// Match Rust's borrowed fast path. Most generated areas are rectangles, so
	// avoid building per-way coordinate vectors just to discover there is no mask.
	if (!world.outline()) {
		for (const auto &[way, value] : ways)
			result.push_back({way, std::nullopt, value});
		return result;
	}
	for (const auto &[way, value] : ways) {
		if (auto pieces = cut_way_to_outline(*way, world)) {
			for (auto &piece : *pieces)
				result.push_back({nullptr, std::move(piece), value});
		} else {
			result.push_back({way, std::nullopt, value});
		}
	}
	return result;
}

std::vector<ProcessedElement> cut_elements_to_outline(
		const std::vector<ProcessedElement> &elements, const XZBBox &world);
}
