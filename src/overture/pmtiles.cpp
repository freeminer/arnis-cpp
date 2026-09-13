#include "pmtiles.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <zlib.h>
#include <zstd.h>
namespace arnis::overture::pmtiles
{
namespace
{
constexpr std::size_t HEADER_LEN = 127, MAX_DIRECTORY_BYTES = 64 * 1024 * 1024,
					  MAX_ENTRIES = 8 * 1024 * 1024;
constexpr std::uint64_t MAX_TILE_BYTES = 64 * 1024 * 1024;
constexpr std::uint8_t COMPRESSION_NONE = 1, COMPRESSION_GZIP = 2, COMPRESSION_BROTLI = 3,
					   COMPRESSION_ZSTD = 4;
constexpr double PI = 3.141592653589793238462643383279502884;
std::uint64_t le64(const std::vector<std::uint8_t> &b, std::size_t at)
{
	if (at + 8 > b.size())
		return 0;
	std::uint64_t v = 0;
	for (unsigned i = 0; i < 8; ++i)
		v |= std::uint64_t(b[at + i]) << (i * 8);
	return v;
}
struct Reader
{
	const std::vector<std::uint8_t> &b;
	std::size_t p = 0;
	bool next(std::uint64_t &v)
	{
		v = 0;
		for (unsigned s = 0; s < 64; s += 7) {
			if (p == b.size())
				return false;
			auto c = b[p++];
			v |= std::uint64_t(c & 127) << s;
			if (!(c & 128))
				return true;
		}
		return false;
	}
};
struct Entry
{
	std::uint64_t id = 0, offset = 0;
	std::uint32_t run = 0, length = 0;
};
std::optional<std::vector<Entry>> decode_directory(const std::vector<std::uint8_t> &bytes)
{
	if (bytes.size() > MAX_DIRECTORY_BYTES)
		return {};
	Reader r{bytes};
	std::uint64_t count;
	if (!r.next(count) || count > MAX_ENTRIES || count > (bytes.size() - r.p) / 4)
		return {};
	std::vector<Entry> e;
	e.reserve(std::size_t(count));
	std::uint64_t id = 0, v;
	for (std::uint64_t i = 0; i < count; ++i) {
		if (!r.next(v) || v > UINT64_MAX - id)
			return {};
		id += v;
		e.push_back({id});
	}
	for (auto &x : e) {
		if (!r.next(v) || v > UINT32_MAX)
			return {};
		x.run = std::uint32_t(v);
	}
	for (auto &x : e) {
		if (!r.next(v) || v > UINT32_MAX)
			return {};
		x.length = std::uint32_t(v);
	}
	for (std::size_t i = 0; i < e.size(); ++i) {
		if (!r.next(v))
			return {};
		if (!v) {
			if (!i || UINT64_MAX - e[i - 1].offset < e[i - 1].length)
				return {};
			e[i].offset = e[i - 1].offset + e[i - 1].length;
		} else
			e[i].offset = v - 1;
	}
	return e;
}

std::optional<std::vector<std::uint8_t>> decompress_gzip(
		const std::vector<std::uint8_t> &compressed, std::uint64_t limit)
{
	z_stream stream{};
	if (inflateInit2(&stream, 15 + 16) != Z_OK)
		return {};
	stream.next_in =
			const_cast<Bytef *>(reinterpret_cast<const Bytef *>(compressed.data()));
	stream.avail_in =
			static_cast<uInt>(std::min<std::size_t>(compressed.size(), UINT_MAX));
	std::vector<std::uint8_t> output;
	output.reserve(std::min<std::uint64_t>(limit, compressed.size() * 4 + 1024));
	int result = Z_OK;
	while (result == Z_OK) {
		if (output.size() >= limit) {
			inflateEnd(&stream);
			return {};
		}
		const auto old_size = output.size();
		const auto chunk = static_cast<std::size_t>(
				std::min<std::uint64_t>(64 * 1024, limit - old_size));
		output.resize(old_size + chunk);
		stream.next_out = reinterpret_cast<Bytef *>(output.data() + old_size);
		stream.avail_out = static_cast<uInt>(chunk);
		result = inflate(&stream, Z_NO_FLUSH);
		output.resize(old_size + chunk - stream.avail_out);
	}
	inflateEnd(&stream);
	return result == Z_STREAM_END
				   ? std::optional<std::vector<std::uint8_t>>(std::move(output))
				   : std::nullopt;
}

std::optional<std::vector<std::uint8_t>> decompress_zstd(
		const std::vector<std::uint8_t> &compressed, std::uint64_t limit)
{
	ZSTD_DStream *stream = ZSTD_createDStream();
	if (!stream || ZSTD_isError(ZSTD_initDStream(stream))) {
		ZSTD_freeDStream(stream);
		return {};
	}
	ZSTD_inBuffer input{compressed.data(), compressed.size(), 0};
	std::vector<std::uint8_t> output;
	output.reserve(std::min<std::uint64_t>(limit, compressed.size() * 4 + 1024));
	std::array<std::uint8_t, 64 * 1024> buffer{};
	std::size_t result = 1;
	while (input.pos < input.size || result != 0) {
		ZSTD_outBuffer chunk{buffer.data(), buffer.size(), 0};
		result = ZSTD_decompressStream(stream, &chunk, &input);
		if (ZSTD_isError(result) || output.size() > limit - chunk.pos) {
			ZSTD_freeDStream(stream);
			return {};
		}
		output.insert(output.end(), buffer.begin(), buffer.begin() + chunk.pos);
		if (chunk.pos == 0 && input.pos == input.size && result != 0) {
			ZSTD_freeDStream(stream);
			return {};
		}
	}
	ZSTD_freeDStream(stream);
	return output;
}

std::optional<std::vector<std::uint8_t>> decompress(
		const std::vector<std::uint8_t> &input, std::uint8_t compression,
		std::uint64_t limit)
{
	if (input.size() > limit)
		return {};
	if (compression == COMPRESSION_NONE)
		return input;
	if (compression == COMPRESSION_GZIP)
		return decompress_gzip(input, limit);
	if (compression == COMPRESSION_ZSTD)
		return decompress_zstd(input, limit);
	// PMTiles defines Brotli too, but Freeminer does not link a Brotli decoder.
	if (compression == COMPRESSION_BROTLI)
		return {};
	return {};
}

std::optional<Entry> entry_for(const std::vector<Entry> &entries, std::uint64_t tile_id)
{
	const auto it = std::upper_bound(entries.begin(), entries.end(), tile_id,
			[](std::uint64_t id, const Entry &entry) { return id < entry.id; });
	if (it == entries.begin())
		return {};
	const auto entry = *std::prev(it);
	if (entry.run == 0)
		return entry;
	return tile_id - entry.id < entry.run ? std::optional<Entry>(entry) : std::nullopt;
}
}
}
namespace arnis::overture::pmtiles
{
std::optional<Header> parse_header(const std::vector<std::uint8_t> &b)
{
	if (b.size() < HEADER_LEN || !std::equal(b.begin(), b.begin() + 7, "PMTiles") ||
			b[7] != 3)
		return {};
	Header h{le64(b, 8), le64(b, 16), le64(b, 40), le64(b, 56), b[97], b[98], b[100],
			b[101]};
	if (!h.root_length || h.root_length > MAX_DIRECTORY_BYTES || b[99] != 1)
		return {};
	return h;
}

std::optional<Archive> open_archive(const RangeReader &read_range)
{
	if (!read_range)
		return {};
	const auto bytes = read_range(0, HEADER_LEN);
	if (!bytes || bytes->size() != HEADER_LEN)
		return {};
	const auto header = parse_header(*bytes);
	if (!header)
		return {};
	const auto root = read_range(header->root_offset, header->root_length);
	if (!root || root->size() != header->root_length)
		return {};
	return Archive{*header, *root};
}
std::optional<TileLocation> find_tile(const Header &h,
		const std::vector<std::uint8_t> &directory, std::uint64_t tile_id)
{
	auto entries = decode_directory(directory);
	if (!entries)
		return {};
	const auto entry = entry_for(*entries, tile_id);
	if (!entry || entry->run == 0 || entry->length == 0)
		return {};
	if (UINT64_MAX - h.tile_data_offset < entry->offset)
		return {};
	return TileLocation{h.tile_data_offset + entry->offset, entry->length};
}

std::optional<std::uint64_t> zxy_to_tile_id(
		std::uint8_t zoom, std::uint32_t x, std::uint32_t y)
{
	if (zoom > 31)
		return {};
	const std::uint64_t side = std::uint64_t{1} << zoom;
	if (x >= side || y >= side)
		return {};
	std::uint64_t id = ((std::uint64_t{1} << (2 * std::uint32_t(zoom))) - 1) / 3;
	std::uint64_t tx = x, ty = y;
	for (std::uint64_t half = side >> 1; half; half >>= 1) {
		const std::uint64_t rx = (tx & half) != 0, ry = (ty & half) != 0;
		id += half * half * ((3 * rx) ^ ry);
		if (!ry) {
			if (rx) {
				tx = half - 1 - tx;
				ty = half - 1 - ty;
			}
			std::swap(tx, ty);
		}
	}
	return id;
}

std::pair<std::uint32_t, std::uint32_t> lonlat_to_tile(
		double longitude, double latitude, std::uint8_t zoom)
{
	const auto side = double(std::uint64_t{1} << std::min<unsigned>(zoom, 31));
	const auto latitude_rad =
			std::clamp(latitude, -85.05112878, 85.05112878) * PI / 180.0;
	const auto max_tile = static_cast<std::uint32_t>(side - 1);
	const auto x = std::clamp((longitude + 180.0) / 360.0 * side, 0.0, side - 1.0);
	const auto y =
			std::clamp((1.0 - std::asinh(std::tan(latitude_rad)) / PI) * side / 2.0, 0.0,
					side - 1.0);
	return {std::min<std::uint32_t>(static_cast<std::uint32_t>(x), max_tile),
			std::min<std::uint32_t>(static_cast<std::uint32_t>(y), max_tile)};
}

double tile_x_to_longitude(std::uint8_t zoom, std::uint32_t tile_x, double fraction)
{
	return (double(tile_x) + fraction) / double(std::uint64_t{1} << zoom) * 360.0 - 180.0;
}

double tile_y_to_latitude(std::uint8_t zoom, std::uint32_t tile_y, double fraction)
{
	const auto t = PI * (1.0 - 2.0 * (double(tile_y) + fraction) /
										double(std::uint64_t{1} << zoom));
	return std::atan(std::sinh(t)) * 180.0 / PI;
}

std::optional<std::vector<std::uint8_t>> read_tile(const Header &header,
		const std::vector<std::uint8_t> &root_directory, std::uint8_t zoom,
		std::uint32_t x, std::uint32_t y, const RangeReader &read_range)
{
	if (!read_range)
		return {};
	const auto tile_id = zxy_to_tile_id(zoom, x, y);
	if (!tile_id || zoom < header.min_zoom || zoom > header.max_zoom)
		return {};
	auto directory =
			decompress(root_directory, header.internal_compression, MAX_DIRECTORY_BYTES);
	if (!directory)
		return {};
	for (unsigned depth = 0; depth < 4; ++depth) {
		const auto entries = decode_directory(*directory);
		if (!entries)
			return {};
		const auto entry = entry_for(*entries, *tile_id);
		if (!entry)
			return {};
		if (entry->run != 0) {
			if (!entry->length || entry->length > MAX_TILE_BYTES ||
					UINT64_MAX - header.tile_data_offset < entry->offset)
				return {};
			auto raw = read_range(header.tile_data_offset + entry->offset, entry->length);
			if (!raw || raw->size() != entry->length)
				return {};
			return decompress(*raw, header.tile_compression, MAX_TILE_BYTES);
		}
		if (!entry->length || entry->length > MAX_DIRECTORY_BYTES ||
				UINT64_MAX - header.leaf_offset < entry->offset)
			return {};
		auto raw = read_range(header.leaf_offset + entry->offset, entry->length);
		if (!raw || raw->size() != entry->length)
			return {};
		directory = decompress(*raw, header.internal_compression, MAX_DIRECTORY_BYTES);
		if (!directory)
			return {};
	}
	return {};
}
} // namespace arnis::overture::pmtiles
