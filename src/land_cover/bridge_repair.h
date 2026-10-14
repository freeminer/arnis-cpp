#pragma once

#include "land_cover.h"

namespace arnis::land_cover
{

// Rust bridge_repair.rs equivalent.
void apply_bridge_land_cover_repair(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox, double scale);

}
