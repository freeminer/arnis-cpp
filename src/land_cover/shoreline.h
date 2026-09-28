#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arnis::land_cover
{
struct LandCoverData;
bool reconstruct_water_shoreline(LandCoverData &data);
}
