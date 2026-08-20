#pragma once

#include "land_cover.h"

namespace arnis::land_cover
{
void apply_osm_land_override(LandCoverData &land_cover, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale);
}
