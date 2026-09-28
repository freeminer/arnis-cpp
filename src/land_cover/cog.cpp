#include "cog.h"
#include "land_cover.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <new>
#include <unordered_map>
#include <zlib.h>

namespace arnis::land_cover
{
namespace
{
std::uint16_t u16(const std::vector<std::uint8_t> &v, std::size_t p, bool be)
{
	if (p + 2 > v.size())
		return 0;
	return be ? (std::uint16_t(v[p]) << 8 | v[p + 1])
			  : (std::uint16_t(v[p + 1]) << 8 | v[p]);
}
std::uint32_t u32(const std::vector<std::uint8_t> &v, std::size_t p, bool be)
{
	if (p + 4 > v.size())
		return 0;
	std::uint32_t r = 0;
	for (int i = 0; i < 4; ++i)
		r = be ? (r << 8) | v[p + i] : (r | (std::uint32_t(v[p + i]) << (8 * i)));
	return r;
}
std::uint64_t u64(const std::vector<std::uint8_t> &v, std::size_t p, bool be)
{
	if (p + 8 > v.size())
		return 0;
	std::uint64_t r = 0;
	for (int i = 0; i < 8; ++i)
		r = be ? (r << 8) | v[p + i] : (r | (std::uint64_t(v[p + i]) << (8 * i)));
	return r;
}
std::size_t type_size(std::uint16_t type)
{
	switch (type) {
	case 1:
	case 2:
	case 6:
	case 7:
		return 1;
	case 3:
	case 8:
		return 2;
	case 4:
	case 9:
	case 11:
		return 4;
	case 5:
	case 10:
	case 12:
	case 16:
	case 17:
	case 18:
		return 8;
	}
	return 0;
}
std::vector<std::uint8_t> value_bytes(const std::string &url,
		const std::vector<std::uint8_t> &source, std::size_t entry, std::uint16_t type,
		std::uint64_t count, bool big, bool be, const CogRangeFetcher &fetch)
{
	const auto n = type_size(type) * count;
	const auto inline_size = big ? 8U : 4U;
	const auto field = entry + (big ? 12U : 8U);
	if (n <= inline_size && field + n <= source.size())
		return {source.begin() + field, source.begin() + field + n};
	const auto offset = big ? u64(source, field, be) : u32(source, field, be);
	if (n == 0 || n > 64U * 1024U * 1024U)
		return {};
	return fetch(url, offset, n);
}
std::uint64_t scalar(
		const std::vector<std::uint8_t> &v, std::uint16_t type, bool be, std::size_t i)
{
	const auto n = type_size(type), p = i * n;
	if (n == 1)
		return p < v.size() ? v[p] : 0;
	if (n == 2)
		return u16(v, p, be);
	if (n == 4)
		return u32(v, p, be);
	if (n == 8)
		return u64(v, p, be);
	return 0;
}
}

bool CogInfo::valid() const
{
	return image_width && image_height && tile_width && tile_height &&
		   !tile_offsets.empty() && tile_offsets.size() == tile_byte_counts.size();
}

bool read_cog_info(const std::string &url, const std::vector<std::uint8_t> &header,
		const CogRangeFetcher &fetch, CogInfo &out)
{
	out = {};
	if (header.size() < 8 || !fetch)
		return false;
	const bool be = header[0] == 'M' && header[1] == 'M';
	if (!be && !(header[0] == 'I' && header[1] == 'I'))
		return false;
	const auto magic = u16(header, 2, be);
	const bool big = magic == 43;
	if (magic != 42 && !big)
		return false;
	const auto ifd_offset = big ? u64(header, 8, be) : u32(header, 4, be);
	std::vector<std::uint8_t> owned;
	const std::vector<std::uint8_t> *bytes = &header;
	std::size_t base = 0;
	if (ifd_offset >= header.size()) {
		owned = fetch(url, ifd_offset, 65536);
		if (owned.empty())
			return false;
		bytes = &owned;
		base = std::size_t(ifd_offset);
	}
	const auto &v = *bytes;
	const auto start = ifd_offset - base;
	const auto count_size = big ? 8U : 2U;
	if (start + count_size > v.size())
		return false;
	const auto count = big ? u64(v, start, be) : u16(v, start, be);
	const auto entry_size = big ? 20U : 12U;
	if (count > 4096 || start + count_size + count * entry_size > v.size())
		return false;
	for (std::uint64_t i = 0; i < count; ++i) {
		const auto e = start + count_size + std::size_t(i) * entry_size;
		const auto tag = u16(v, e, be), type = u16(v, e + 2, be);
		const auto values = big ? u64(v, e + 4, be) : u32(v, e + 4, be);
		auto data = value_bytes(url, v, e, type, values, big, be, fetch);
		if (data.empty())
			continue;
		auto first = scalar(data, type, be, 0);
		switch (tag) {
		case 256:
			out.image_width = first;
			break;
		case 257:
			out.image_height = first;
			break;
		case 259:
			out.compression = std::uint16_t(first);
			break;
		case 322:
			out.tile_width = first;
			break;
		case 323:
			out.tile_height = first;
			break;
		case 324:
			out.tile_offsets.clear();
			for (std::uint64_t n = 0; n < values; ++n)
				out.tile_offsets.push_back(scalar(data, type, be, n));
			break;
		case 325:
			out.tile_byte_counts.clear();
			for (std::uint64_t n = 0; n < values; ++n)
				out.tile_byte_counts.push_back(scalar(data, type, be, n));
			break;
		}
	}
	return out.valid();
}

std::vector<std::uint8_t> lzw_decompress_tiff(
		const std::vector<std::uint8_t> &data, std::size_t expected)
{
	std::size_t bit = 0;
	auto code_at = [&](int bits, std::uint16_t &out) {
		if (bit + std::size_t(bits) > data.size() * 8)
			return false;
		out = 0;
		for (int i = 0; i < bits; ++i)
			out = std::uint16_t(
					(out << 1) | ((data[(bit + i) / 8] >> (7 - ((bit + i) & 7))) & 1));
		bit += bits;
		return true;
	};
	std::vector<std::vector<std::uint8_t>> dict(4096);
	auto reset = [&] {
		for (int i = 0; i < 256; ++i)
			dict[i] = {std::uint8_t(i)};
		for (int i = 256; i < 4096; ++i)
			dict[i].clear();
	};
	reset();
	int bits = 9, next = 258, previous = -1;
	std::vector<std::uint8_t> out;
	out.reserve(expected);
	std::uint16_t code;
	while (code_at(bits, code)) {
		if (code == 256) {
			reset();
			bits = 9;
			next = 258;
			previous = -1;
			continue;
		}
		if (code == 257)
			break;
		std::vector<std::uint8_t> entry;
		if (code < next && !dict[code].empty())
			entry = dict[code];
		else if (code == next && previous >= 0) {
			entry = dict[previous];
			entry.push_back(entry.front());
		} else
			return {};
		out.insert(out.end(), entry.begin(), entry.end());
		if (previous >= 0 && next < 4096) {
			dict[next] = dict[previous];
			dict[next].push_back(entry.front());
			++next;
			if (next == (1 << bits) - 1 && bits < 12)
				++bits;
		}
		previous = code;
		if (expected && out.size() >= expected) {
			out.resize(expected);
			return out;
		}
	}
	return expected && out.size() != expected ? std::vector<std::uint8_t>{} : out;
}

std::vector<std::uint8_t> decompress_cog_tile(const std::vector<std::uint8_t> &data,
		std::size_t expected, std::uint16_t compression)
{
	if (compression == 1)
		return data;
	if (compression == 5)
		return lzw_decompress_tiff(data, expected);
	if ((compression != 8 && compression != 32946) || expected == 0)
		return {};
	std::vector<std::uint8_t> out(expected);
	uLongf size = expected;
	if (uncompress(out.data(), &size, data.data(), data.size()) == Z_OK) {
		out.resize(size);
		return out;
	}
	z_stream stream{};
	stream.next_in = const_cast<Bytef *>(reinterpret_cast<const Bytef *>(data.data()));
	stream.avail_in = uInt(data.size());
	stream.next_out = out.data();
	stream.avail_out = uInt(out.size());
	if (inflateInit2(&stream, -MAX_WBITS) != Z_OK)
		return {};
	const auto rc = inflate(&stream, Z_FINISH);
	inflateEnd(&stream);
	if (rc != Z_STREAM_END)
		return {};
	out.resize(stream.total_out);
	return out;
}

bool read_esa_cog_into_raster(const std::string &url, int south, int west, double min_lat,
		double min_lng, double max_lat, double max_lng, EsaRasterTile &raster,
		const CogRangeFetcher &fetch)
{
	if (!fetch || !std::isfinite(min_lat) || !std::isfinite(max_lat) ||
			!std::isfinite(min_lng) || !std::isfinite(max_lng) || min_lat > max_lat ||
			min_lng > max_lng)
		return false;
	CogInfo info;
	if (!read_cog_info(url, fetch(url, 0, 65536), fetch, info))
		return false;
	const double ppd = double(info.image_width) / 3.0;
	if (!(ppd > 0.0) || std::abs(double(info.image_height) / 3.0 - ppd) > 1e-9)
		return false;
	// Match Rust's global pixel convention before clipping to this COG.
	const double tile_x = std::round((west + 180.0) * ppd);
	const double tile_y = std::round((90.0 - (south + 3.0)) * ppd);
	const auto x0 = std::uint64_t(std::clamp(
			std::floor((min_lng + 180.0) * ppd) - tile_x, 0.0, double(info.image_width)));
	const auto x1 =
			std::uint64_t(std::clamp(std::floor((max_lng + 180.0) * ppd) + 1.0 - tile_x,
					0.0, double(info.image_width)));
	const auto y0 = std::uint64_t(std::clamp(
			std::floor((90.0 - max_lat) * ppd) - tile_y, 0.0, double(info.image_height)));
	const auto y1 =
			std::uint64_t(std::clamp(std::floor((90.0 - min_lat) * ppd) + 1.0 - tile_y,
					0.0, double(info.image_height)));
	if (x0 >= x1 || y0 >= y1)
		return false;
	constexpr std::uint64_t MAX_RASTER_PIXELS = std::uint64_t{1} << 31;
	if (x1 - x0 > MAX_RASTER_PIXELS / (y1 - y0) ||
			info.tile_width > std::numeric_limits<std::size_t>::max() / info.tile_height)
		return false;
	EsaRasterTile out;
	out.south_lat = south;
	out.west_lng = west;
	out.width = x1 - x0;
	out.height = y1 - y0;
	out.min_lng = (tile_x + x0) / ppd - 180.0;
	out.max_lng = (tile_x + x1) / ppd - 180.0;
	out.max_lat = 90.0 - (tile_y + y0) / ppd;
	out.min_lat = 90.0 - (tile_y + y1) / ppd;
	try {
		out.pixels.resize(out.width * out.height);
	} catch (const std::bad_alloc &) {
		return false;
	}
	const auto across = (info.image_width + info.tile_width - 1) / info.tile_width;
	bool wrote = false;
	for (auto ty = y0 / info.tile_height; ty <= (y1 - 1) / info.tile_height; ++ty)
		for (auto tx = x0 / info.tile_width; tx <= (x1 - 1) / info.tile_width; ++tx) {
			const auto index = ty * across + tx;
			if (index >= info.tile_offsets.size() ||
					index >= info.tile_byte_counts.size() || !info.tile_offsets[index] ||
					!info.tile_byte_counts[index])
				continue;
			const auto pixels = decompress_cog_tile(
					fetch(url, info.tile_offsets[index], info.tile_byte_counts[index]),
					std::size_t(info.tile_width * info.tile_height), info.compression);
			const auto tile_px = tx * info.tile_width, tile_py = ty * info.tile_height;
			for (auto y = std::max(y0, tile_py);
					y < std::min(y1, tile_py + info.tile_height); ++y)
				for (auto x = std::max(x0, tile_px);
						x < std::min(x1, tile_px + info.tile_width); ++x) {
					const auto src = (y - tile_py) * info.tile_width + x - tile_px;
					if (src < pixels.size()) {
						out.pixels[(y - y0) * out.width + x - x0] = pixels[src];
						wrote |= pixels[src] != 0;
					}
				}
		}
	if (wrote)
		raster = std::move(out);
	return wrote;
}

bool read_esa_cog_into_grid(const std::string &url, int south, int west, double min_lat,
		double min_lng, double max_lat, double max_lng,
		std::vector<std::vector<std::uint8_t>> &grid, const CogRangeFetcher &fetch)
{
	if (grid.empty() || grid.front().empty() || !fetch)
		return false;
	auto header = fetch(url, 0, 65536);
	CogInfo info;
	if (!read_cog_info(url, header, fetch, info))
		return false;
	const auto height = grid.size(), width = grid.front().size();
	const auto across = (info.image_width + info.tile_width - 1) / info.tile_width;
	std::unordered_map<std::uint64_t, std::vector<std::uint8_t>> decoded;
	bool wrote = false;
	for (std::size_t z = 0; z < height; ++z)
		for (std::size_t x = 0; x < width; ++x) {
			const double lng = min_lng + (width == 1 ? 0.0
													 : double(x) * (max_lng - min_lng) /
																 double(width - 1));
			const double lat = max_lat - (height == 1 ? 0.0
													  : double(z) * (max_lat - min_lat) /
																 double(height - 1));
			if (lat < south || lat > south + 3 || lng < west || lng > west + 3)
				continue;
			const auto px = std::min(info.image_width - 1,
					std::uint64_t(std::floor((lng - west) / 3.0 * info.image_width)));
			const auto py = std::min(info.image_height - 1,
					std::uint64_t(
							std::floor((south + 3 - lat) / 3.0 * info.image_height)));
			const auto tx = px / info.tile_width;
			const auto ty = py / info.tile_height;
			const auto tile_index = ty * across + tx;
			if (tile_index >= info.tile_offsets.size() ||
					!info.tile_offsets[tile_index] || !info.tile_byte_counts[tile_index])
				continue;
			auto it = decoded.find(tile_index);
			if (it == decoded.end()) {
				auto bytes = fetch(url, info.tile_offsets[tile_index],
						info.tile_byte_counts[tile_index]);
				it = decoded.emplace(tile_index, decompress_cog_tile(bytes,
														 std::size_t(info.tile_width *
																	 info.tile_height),
														 info.compression))
							 .first;
			}
			const auto local =
					(py % info.tile_height) * info.tile_width + (px % info.tile_width);
			if (local < it->second.size() && it->second[local]) {
				grid[z][x] = it->second[local];
				wrote = true;
			}
		}
	return wrote;
}
}
