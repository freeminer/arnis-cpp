#pragma once
#include <vector>
#include "../../arnis_adapter.h"
#include "floodfill_cache.h"
namespace arnis::clipping
{
// General Rust clip_way_to_bbox equivalent.  Closed ways remain explicitly
// closed after clipping; open ways are clipped segment-by-segment and never
// acquire an artificial flood-fillable ring.
std::vector<ProcessedNode> clip_way_to_bbox(
		const std::vector<ProcessedNode> &, const XZBBox &);
std::optional<std::vector<ProcessedNode>> clip_water_ring_to_bbox(
		const std::vector<ProcessedNode> &, const XZBBox &);
}
