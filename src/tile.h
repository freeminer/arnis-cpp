#pragma once

#include "floodfill_cache.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace arnis
{

/// Bounds of a tile. Maximum edges are exclusive.
struct TileBounds
{
	int32_t min_x;
	int32_t min_z;
	int32_t max_x;
	int32_t max_z;

	bool contains(int32_t x, int32_t z) const noexcept
	{
		return x >= min_x && x < max_x && z >= min_z && z < max_z;
	}
	TileBounds expanded(int32_t halo) const noexcept
	{
		return {min_x - halo, min_z - halo, max_x + halo, max_z + halo};
	}
};

inline constexpr int32_t DEFAULT_TILE_SIZE = 512;
inline constexpr int32_t TILE_EDITOR_HALO = 64;

namespace tiles
{
inline int region_floor(int32_t coordinate) noexcept
{
	return coordinate >= 0 ? coordinate / DEFAULT_TILE_SIZE
						   : -static_cast<int>(
									 (-std::int64_t(coordinate) + DEFAULT_TILE_SIZE - 1) /
									 DEFAULT_TILE_SIZE);
}
} // namespace tiles

/// Build region-aligned tiles overlapping the supplied world bounds.
std::vector<TileBounds> create_tiles(
		const ::XZBBox &bbox, int32_t tile_size = DEFAULT_TILE_SIZE);

/// Return the source-element indices owned by each tile using Rust's tile halo
/// and element-width rules. Per-tile index order matches the input order.
std::vector<std::vector<std::size_t>> assign_elements_to_tiles(
		const std::vector<ProcessedElement> &elements,
		const std::vector<TileBounds> &tiles, double scale);

} // namespace arnis
