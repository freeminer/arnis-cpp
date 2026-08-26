#pragma once

#include "land_cover.h"

namespace arnis::land_cover
{

// Rust bridge_repair.rs equivalent.  The implementation remains in
// land_cover.cpp for now because it shares the raster helpers with the other
// land-cover passes; this header keeps the pass independently addressable in
// the C++ module layout.
void apply_bridge_land_cover_repair(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale);

}
