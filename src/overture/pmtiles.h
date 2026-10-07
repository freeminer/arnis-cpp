#pragma once
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
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
enum class TileReadStatus
{
	Found,
	Missing,
	Error
};
struct TileReadResult
{
	TileReadStatus status = TileReadStatus::Error;
	std::vector<std::uint8_t> bytes;
};
struct TileLocateResult
{
	TileReadStatus status = TileReadStatus::Error;
	TileLocation location;
};
// Decompressed leaf directories reused while serially locating one bbox's tiles.
using LeafDirectoryCache = std::unordered_map<std::uint64_t, std::vector<std::uint8_t>>;
struct Archive
{
	Header header;
	// Stored exactly as it appears in the archive. `read_tile` applies the
	// header's internal-compression setting before parsing it.
	std::vector<std::uint8_t> root_directory;
};
// Reads exactly one byte range from an archive.  Keeping transport here makes
// the PMTiles reader usable both by the standalone library and host-provided
// HTTP/cache implementations. Sources used by pmtiles_building_source must be
// safe for concurrent calls; tile bodies are fetched by a bounded worker set.
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
// Distinguishes a valid empty-directory lookup from transport/corruption errors.
TileReadResult read_tile_checked(const Header &,
		const std::vector<std::uint8_t> &root_directory, std::uint8_t zoom,
		std::uint32_t x, std::uint32_t y, const RangeReader &);
TileLocateResult locate_tile_checked(const Header &,
		const std::vector<std::uint8_t> &root_directory, std::uint8_t zoom,
		std::uint32_t x, std::uint32_t y, const RangeReader &,
		LeafDirectoryCache *leaf_cache = nullptr);
// Locate a raw directory id, including non-geographic records stored outside
// the advertised tile zoom range (e.g. AOT2 whole-relation records).
TileLocateResult locate_id_checked(const Header &,
		const std::vector<std::uint8_t> &root_directory, std::uint64_t tile_id,
		const RangeReader &, LeafDirectoryCache *leaf_cache = nullptr);
TileReadResult read_tile_payload_checked(
		const Header &, const TileLocation &, const RangeReader &);
} // namespace arnis::overture::pmtiles
