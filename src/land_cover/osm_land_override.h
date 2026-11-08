#pragma once

#include "../../../arnis_adapter.h"
#include "land_cover.h"

namespace arnis::land_cover
{
inline constexpr double SHORE_BAND_M = 15.0;

// Shared OSM deck classification used when deciding whether an extracted
// building footprint over ESA water is actually supported by a mapped pier.
bool is_over_water_structure(const tags_t &tags);
std::optional<int> over_water_half_width(const ProcessedWay &way, double scale);

void apply_osm_land_override(LandCoverData &land_cover, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale);
}
