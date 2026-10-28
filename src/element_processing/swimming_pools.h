#pragma once

#include "../floodfill_cache.h"
#include "../block_definitions.h"
#include "bridges.h"
#include "../args.h"

namespace arnis::swimming_pools
{
extern const Block COPING;
extern const Block LINING;

bool is_swimming_pool(const tags_t &tags);
void generate(WorldEditor &, const ProcessedWay &, const Args &, const FloodFillCache &,
		const BuildingFootprintBitmap &, const RoadMaskBitmap &);
void generate_relation(WorldEditor &, const ProcessedRelation &, const Args &,
		const FloodFillCache &, const BuildingFootprintBitmap &, const RoadMaskBitmap &);
} // namespace arnis::swimming_pools
