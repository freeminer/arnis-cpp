#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arnis::land_cover
{
struct LandCoverData;
struct EsaRasterTile;
bool reconstruct_water_shoreline(
		LandCoverData &data, const std::vector<EsaRasterTile> &source_tiles);
}
