#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

#include <chrono>
#include <optional>

namespace arnis::attractions
{
bool claims(const tags_t &tags);
bool replaces_parts(const tags_t &tags);
bool generate_ride(WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout);
} // namespace arnis::attractions
