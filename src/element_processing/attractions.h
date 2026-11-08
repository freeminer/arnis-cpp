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
// Exposed as a small policy seam so the Rust/C++ height threshold stays
// regression-tested independently of a concrete world-editor backend.
bool is_observation_wheel(
		const std::optional<double> &height, double footprint_length, double scale);
bool generate_ride(WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, double scale,
		const std::optional<std::chrono::milliseconds> &timeout);
} // namespace arnis::attractions
