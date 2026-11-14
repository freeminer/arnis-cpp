#pragma once

#include "../decals/region.h"
#include "../args.h"
#include "bridges.h"
#include "../floodfill_cache.h"

#include <cstdint>

namespace arnis::cemetery
{

bool claims(const tags_t &tags);
Block flower(std::uint64_t hash);
bool is_mapped_grave(const tags_t &tags);
void prepare(std::vector<ProcessedElement> &elements, decals::SignRegion region);
void generate_cemetery(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const bridges::BridgeSurfaceMap &bridges);
void cemetery_area(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const bridges::BridgeSurfaceMap &bridges,
		const Holes *holes);
void generate_mapped_grave(WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints);

} // namespace arnis::cemetery
