#pragma once

#include "../floodfill_cache.h"

#include <array>
#include <optional>
#include <utility>

namespace arnis::element_processing
{

// Mirrors Rust's shared OSM road-mask search: test cardinal cells, then
// diagonals, at even distances. The stable order matters when several roads
// (or non-road cells) are equally near.
template <typename Predicate>
inline std::optional<std::pair<int, int>> nearest_block_matching(int x, int z,
		int max_radius, const RoadMaskBitmap &road_mask, Predicate &&predicate)
{
	for (int distance = 2; distance <= max_radius; distance += 2) {
		const std::array<std::pair<int, int>, 8> candidates{{{x, z - distance},
				{x, z + distance}, {x - distance, z}, {x + distance, z},
				{x - distance, z - distance}, {x + distance, z + distance},
				{x - distance, z + distance}, {x + distance, z - distance}}};
		for (const auto &[candidate_x, candidate_z] : candidates)
			if (predicate(road_mask.contains(candidate_x, candidate_z)))
				return {{candidate_x, candidate_z}};
	}
	return std::nullopt;
}

inline std::optional<std::pair<int, int>> get_nearest_road_block(
		int x, int z, int max_radius, const RoadMaskBitmap &road_mask)
{
	return nearest_block_matching(
			x, z, max_radius, road_mask, [](bool on_road) { return on_road; });
}

inline std::optional<std::pair<int, int>> get_nearest_non_road_block(
		int x, int z, int max_radius, const RoadMaskBitmap &road_mask)
{
	return nearest_block_matching(
			x, z, max_radius, road_mask, [](bool on_road) { return !on_road; });
}

} // namespace arnis::element_processing
