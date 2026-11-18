#pragma once

#include "../floodfill_cache.h"
#include "../osm_parser.h"

namespace arnis::element_processing::parking_lots
{

bool is_surface_lot(const tags_t &tags);
void generate_parking_lot(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const Holes *holes = nullptr);

} // namespace arnis::element_processing::parking_lots
