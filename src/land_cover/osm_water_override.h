#pragma once

#include "land_cover.h"

#include <cstdint>

namespace arnis::land_cover
{
struct WaterOverrideGuard
{
	std::vector<std::uint64_t> protected_mask;
	std::vector<float> nearest_water_y;
	std::size_t width{0}, height{0};
	bool has_elevation{false};
	bool allows(const std::vector<std::vector<float>> &heights, std::size_t x,
			std::size_t z, float tolerance = 1.5f) const;
};

WaterOverrideGuard build_water_override_guard(
		const std::vector<std::vector<std::uint8_t>> &grid,
		const std::vector<std::vector<float>> &heights, double cells_per_m2);

void apply_osm_water_override(LandCoverData &data,
		std::vector<std::vector<float>> &heights, std::size_t world_width,
		std::size_t world_height, const std::vector<ProcessedElement> &elements,
		const XZBBox &bbox);
}
