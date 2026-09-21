#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>
namespace arnis::overture::pmtiles
{
// PMTiles type 0 means the archive carries a private payload rather than MVT.
// Arnis OSM archives use this for AOT1 tiles.
inline constexpr std::uint8_t TILE_TYPE_UNKNOWN = 0;
struct Header
{
	std::uint64_t root_offset = 0, root_length = 0, leaf_offset = 0, tile_data_offset = 0;
	std::uint8_t internal_compression = 0, tile_compression = 0, min_zoom = 0,
				 max_zoom = 0;
};
struct TileLocation
{
	std::uint64_t offset = 0;
	std::uint32_t length = 0;
};
struct Archive
{
	Header header;
	// Stored exactly as it appears in the archive. `read_tile` applies the
	// header's internal-compression setting before parsing it.
	std::vector<std::uint8_t> root_directory;
};
// Reads exactly one byte range from an archive.  Keeping transport here makes
// the PMTiles reader usable both by the standalone library and host-provided
// HTTP/cache implementations.
using RangeReader = std::function<std::optional<std::vector<std::uint8_t>>(
		std::uint64_t offset, std::uint64_t length)>;
std::optional<Header> parse_header(const std::vector<std::uint8_t> &);
std::optional<Archive> open_archive(const RangeReader &);
std::optional<std::uint64_t> zxy_to_tile_id(
		std::uint8_t zoom, std::uint32_t x, std::uint32_t y);
std::pair<std::uint32_t, std::uint32_t> lonlat_to_tile(
		double longitude, double latitude, std::uint8_t zoom);
double tile_x_to_longitude(std::uint8_t zoom, std::uint32_t tile_x, double fraction);
double tile_y_to_latitude(std::uint8_t zoom, std::uint32_t tile_y, double fraction);
std::optional<TileLocation> find_tile(const Header &,
		const std::vector<std::uint8_t> &directory, std::uint64_t tile_id);
// Locate a tile through root and leaf directories.  Directory and tile
// compression are decoded according to the archive header (none or gzip).
std::optional<std::vector<std::uint8_t>> read_tile(const Header &,
		const std::vector<std::uint8_t> &root_directory, std::uint8_t zoom,
		std::uint32_t x, std::uint32_t y, const RangeReader &);
} // namespace arnis::overture::pmtiles
