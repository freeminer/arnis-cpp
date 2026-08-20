#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arnis::land_cover
{
bool reconstruct_water_shoreline(std::vector<std::vector<std::uint8_t>> &grid,
		std::size_t width, std::size_t height, double cells_per_meter);
}
