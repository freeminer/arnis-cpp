#pragma once

// Compatibility facade for the Rust fixed_tile module.  The implementation
// lives in ../providers.cpp so all elevation providers use one Mercator
// projection and one cache-key representation.
#include "../providers.h"

namespace arnis::elevation::fixed_tile
{
inline constexpr std::size_t TILE_PIXELS = 512;
using Resolution = providers::UsgsResolution;
using TileKey = providers::UsgsFixedTileKey;

inline double meters_per_pixel(Resolution level)
{
	return providers::meters_per_pixel(level);
}
inline const char *level_id(Resolution level)
{
	return providers::resolution_id(level);
}
inline TileKey for_mercator(Resolution level, double mx, double my)
{
	return providers::usgs_tile_for_mercator(level, mx, my);
}
inline std::vector<TileKey> covering_tiles(const providers::GeoBBox &bbox, Resolution level)
{
	return providers::covering_usgs_tiles(bbox, level);
}
inline Resolution select_level_for_cell_size(const providers::GeoBBox &bbox, std::size_t width,
		std::size_t height)
{
	return providers::choose_usgs_resolution(bbox, width, height);
}
inline std::optional<double> sample_tile_bilinear(const std::vector<std::vector<double>> &tile,
		const TileKey &key, double mx, double my)
{
	return providers::sample_usgs_tile_bilinear(tile, key, mx, my);
}
inline std::string tile_url(const TileKey &key)
{
	return providers::usgs_3dep_tile_url(key);
}
}
