#pragma once

#include "land_cover.h"
#include "../biome.h"
#include <vector>

namespace arnis::land_cover
{
// Restore natural cover in terrain-only/blueprint worlds. Heights, when present,
// are aligned with the land-cover grid and are lowered for reclaimed water.
void naturalize(LandCoverData &data, std::vector<std::vector<float>> *heights,
		biome::Climate climate);
}
