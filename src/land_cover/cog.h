#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace arnis::land_cover
{
// Minimal Cloud-Optimized GeoTIFF reader for the single-band ESA WorldCover
// products. Network/cache ownership remains with the embedding application.
struct CogInfo
{
	std::uint64_t image_width{0}, image_height{0};
	std::uint64_t tile_width{0}, tile_height{0};
	std::vector<std::uint64_t> tile_offsets;
	std::vector<std::uint64_t> tile_byte_counts;
	std::uint16_t compression{1};
	bool valid() const;
};

using CogRangeFetcher = std::function<std::vector<std::uint8_t>(
		const std::string &url, std::uint64_t offset, std::uint64_t length)>;

// Parses a TIFF or BigTIFF IFD. The callback is used only when an IFD/value
// lies outside `header`; callers normally pass a 64 KiB range as `header`.
bool read_cog_info(const std::string &url, const std::vector<std::uint8_t> &header,
		const CogRangeFetcher &fetch_range, CogInfo &out);

// Decodes the TIFF compression modes used by ESA WorldCover: none, deflate,
// and LZW. Invalid streams return an empty vector.
std::vector<std::uint8_t> decompress_cog_tile(const std::vector<std::uint8_t> &data,
		std::size_t expected_pixels, std::uint16_t compression);

std::vector<std::uint8_t> lzw_decompress_tiff(
		const std::vector<std::uint8_t> &data, std::size_t expected_pixels);

// Samples one ESA 3x3-degree COG into an already allocated [z][x] grid.  The
// function only downloads/decompresses internal TIFF tiles touched by bbox.
bool read_esa_cog_into_grid(const std::string &url, int tile_south_lat, int tile_west_lng,
		double min_lat, double min_lng, double max_lat, double max_lng,
		std::vector<std::vector<std::uint8_t>> &grid, const CogRangeFetcher &fetch_range);
}
