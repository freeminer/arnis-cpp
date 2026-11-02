#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

namespace arnis::attractions
{
bool claims(const tags_t &tags);
bool replaces_parts(const tags_t &tags);
std::unordered_set<std::uint64_t> building_parts_to_suppress(
		const std::vector<ProcessedElement> &elements);
bool generate_ride(WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout);
} // namespace arnis::attractions
