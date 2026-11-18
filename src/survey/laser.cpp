#include "laser.h"

#include "laser_cache.h"
#include "../net.h"
#include "photo.h"

#include <zstd.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <atomic>
#include <cctype>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <numeric>
#include <semaphore>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <zlib.h>

namespace arnis::survey::laser
{
namespace
{
constexpr std::size_t CELLS = TILE * TILE;
constexpr std::size_t BLOCK_CELLS = BLOCK * BLOCK;
constexpr std::array<std::uint8_t, 8> MAGIC{'A', 'R', 'N', 'S', 'L', 'A', 'S', '4'};
constexpr std::uint8_t TARGET_DENSITY_TENTHS = 70;
constexpr std::uint8_t BUILDING = 1;
constexpr std::uint8_t PLANT = 2;
constexpr std::uint8_t UNSORTED = 4;
constexpr float ROOF_MIN_M = 2.5f;
constexpr std::size_t CHUNK_SIZE = 1U << 13;
std::atomic<std::uint64_t> download_sequence{0};
std::array<std::mutex, 64> tile_fetch_locks;
std::counting_semaphore<2> whole_file_downloads{2};

struct WholeDownloadPermit
{
	explicit WholeDownloadPermit(std::counting_semaphore<2> &slots) : slots(slots)
	{
		slots.acquire();
	}
	~WholeDownloadPermit() { slots.release(); }
	std::counting_semaphore<2> &slots;
};

void append_u16(std::vector<std::uint8_t> &out, std::uint16_t value)
{
	out.push_back(static_cast<std::uint8_t>(value));
	out.push_back(static_cast<std::uint8_t>(value >> 8));
}

std::uint16_t read_u16(const std::uint8_t *p)
{
	return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1]) << 8);
}

std::uint32_t read_u32(const std::uint8_t *p)
{
	return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
		   (static_cast<std::uint32_t>(p[2]) << 16) |
		   (static_cast<std::uint32_t>(p[3]) << 24);
}

template <typename T>
void append_layer(std::vector<std::uint8_t> &out, const std::vector<T> &layer)
{
	for (const auto value : layer) {
		if constexpr (sizeof(T) == 1) {
			out.push_back(static_cast<std::uint8_t>(value));
		} else {
			append_u16(out, static_cast<std::uint16_t>(value));
		}
	}
}

template <typename T>
bool read_layer(const std::vector<std::uint8_t> &raw, std::size_t &offset,
		std::vector<T> &layer, std::size_t count)
{
	if (count > (raw.size() - std::min(offset, raw.size())) / sizeof(T))
		return false;
	layer.resize(count);
	if constexpr (sizeof(T) == 1) {
		std::copy_n(raw.data() + offset, count, layer.data());
	} else {
		for (std::size_t i = 0; i < count; ++i)
			layer[i] = static_cast<T>(read_u16(raw.data() + offset + i * 2));
	}
	offset += count * sizeof(T);
	return true;
}

std::optional<std::vector<std::uint8_t>> decompress_file(
		const std::filesystem::path &path)
{
	std::ifstream file(path, std::ios::binary);
	if (!file)
		return std::nullopt;
	const std::vector<std::uint8_t> compressed(
			(std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	if (compressed.empty())
		return std::nullopt;
	constexpr std::size_t MAX_ORIGIN = std::numeric_limits<std::uint16_t>::max();
	constexpr std::size_t MAX_RAW =
			8 + 1 + BLOCKS * BLOCKS + 2 + MAX_ORIGIN + 7 * CELLS + 2 * CELLS;
	const auto frame_size =
			ZSTD_getFrameContentSize(compressed.data(), compressed.size());
	if (frame_size == ZSTD_CONTENTSIZE_ERROR ||
			(frame_size != ZSTD_CONTENTSIZE_UNKNOWN && frame_size > MAX_RAW))
		return std::nullopt;
	std::vector<std::uint8_t> raw(frame_size == ZSTD_CONTENTSIZE_UNKNOWN
										  ? MAX_RAW
										  : static_cast<std::size_t>(frame_size));
	const auto size =
			ZSTD_decompress(raw.data(), raw.size(), compressed.data(), compressed.size());
	if (ZSTD_isError(size) || size > raw.size())
		return std::nullopt;
	raw.resize(size);
	return raw;
}

template <typename T>
class Chunked
{
	std::vector<std::vector<T>> chunks_;
	std::size_t size_ = 0;

public:
	std::size_t size() const { return size_; }
	void push(T value)
	{
		if (chunks_.empty() || chunks_.back().size() == CHUNK_SIZE) {
			chunks_.emplace_back();
			chunks_.back().reserve(CHUNK_SIZE);
		}
		chunks_.back().push_back(std::move(value));
		++size_;
	}
	const T &at(std::size_t index) const
	{
		return chunks_[index / CHUNK_SIZE][index % CHUNK_SIZE];
	}
	void clear()
	{
		chunks_.clear();
		size_ = 0;
	}
	template <typename F>
	void each(F &&fn) const
	{
		for (const auto &chunk : chunks_)
			for (const auto &value : chunk)
				fn(value);
	}
};

struct BlockPoints
{
	Chunked<std::uint32_t> ground;
	Chunked<std::uint16_t> above_at;
	Chunked<std::uint8_t> above_flags;
	Chunked<float> above_z;
};

std::size_t tile_cell(std::size_t block, std::size_t local)
{
	const auto bx = block % BLOCKS, by = block / BLOCKS;
	return (by * BLOCK + local / BLOCK) * TILE + bx * BLOCK + local % BLOCK;
}

std::vector<bool> grow_by_one(const std::vector<bool> &mask)
{
	auto grown = mask;
	for (std::size_t row = 0; row < TILE; ++row) {
		for (std::size_t col = 0; col < TILE; ++col) {
			if (!mask[row * TILE + col])
				continue;
			const auto r0 = row == 0 ? 0 : row - 1;
			const auto c0 = col == 0 ? 0 : col - 1;
			for (std::size_t r = r0; r <= std::min(row + 1, TILE - 1); ++r)
				for (std::size_t c = c0; c <= std::min(col + 1, TILE - 1); ++c)
					grown[r * TILE + c] = true;
		}
	}
	return grown;
}

bool roofs_unsorted(const std::vector<BlockPoints> &blocks,
		const std::function<std::optional<std::pair<std::uint8_t, float>>(
				std::size_t, std::uint8_t, float)> &settle)
{
	bool unsorted = false;
	for (std::size_t b = 0; b < blocks.size(); ++b) {
		const auto &block = blocks[b];
		for (std::size_t i = 0; i < block.above_at.size(); ++i) {
			const auto flags = block.above_flags.at(i);
			const auto kept = settle(
					tile_cell(b, block.above_at.at(i)), flags, block.above_z.at(i));
			if (!kept)
				continue;
			if (kept->first & BUILDING)
				return false;
			unsorted |= (kept->first & UNSORTED) != 0;
		}
	}
	return unsorted;
}

void find_roofs(std::vector<std::tuple<std::uint32_t, float, std::uint8_t>> &points)
{
	std::unordered_map<std::uint32_t, std::pair<std::uint32_t, std::uint32_t>> tall;
	for (const auto &[cell, height, flags] : points) {
		if ((flags & UNSORTED) && height >= ROOF_MIN_M) {
			auto &entry = tall[cell];
			++entry.first;
			entry.second += (flags & PLANT) != 0;
		}
	}
	for (auto &[cell, height, flags] : points) {
		if ((flags & UNSORTED) && !(flags & PLANT) && height >= ROOF_MIN_M) {
			const auto found = tall.find(cell);
			if (found != tall.end() && found->second.first >= 2 &&
					4 * found->second.second < found->second.first)
				flags = BUILDING;
		}
	}
}

std::vector<float> fill_ground(
		const std::vector<double> &sum, const std::vector<std::uint32_t> &count)
{
	std::vector<float> ground(CELLS, std::numeric_limits<float>::quiet_NaN());
	std::vector<float> known;
	known.reserve(CELLS);
	for (std::size_t i = 0; i < CELLS; ++i) {
		if (count[i]) {
			ground[i] = static_cast<float>(sum[i] / count[i]);
			known.push_back(ground[i]);
		}
	}
	if (known.empty())
		return std::vector<float>(CELLS, 0.0f);
	std::vector<std::size_t> holes;
	for (std::size_t i = 0; i < CELLS; ++i)
		if (std::isnan(ground[i]))
			holes.push_back(i);
	std::vector<std::pair<std::size_t, float>> found;
	for (int ring = 0; ring < 60 && !holes.empty(); ++ring) {
		found.clear();
		auto end = std::remove_if(holes.begin(), holes.end(), [&](std::size_t cell) {
			const auto row = static_cast<std::int64_t>(cell / TILE);
			const auto col = static_cast<std::int64_t>(cell % TILE);
			float total = 0.0f;
			std::uint32_t adjacent = 0;
			for (const auto [dr, dc] : {std::pair{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
				const auto r = row + dr, c = col + dc;
				if (r < 0 || c < 0 || r >= static_cast<std::int64_t>(TILE) ||
						c >= static_cast<std::int64_t>(TILE))
					continue;
				const auto value = ground[static_cast<std::size_t>(r) * TILE +
										  static_cast<std::size_t>(c)];
				if (!std::isnan(value)) {
					total += value;
					++adjacent;
				}
			}
			if (adjacent) {
				found.emplace_back(cell, total / adjacent);
				return true;
			}
			return false;
		});
		holes.erase(end, holes.end());
		if (found.empty())
			break;
		for (const auto &[cell, value] : found)
			ground[cell] = value;
	}
	std::sort(known.begin(), known.end());
	const float median = known[known.size() / 2];
	for (auto &value : ground)
		if (std::isnan(value))
			value = median;
	return ground;
}

void write_unique(
		const std::filesystem::path &path, const std::vector<std::uint8_t> &packed)
{
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (ec)
		throw std::filesystem::filesystem_error(
				"create cache directory", path.parent_path(), ec);
	auto temporary = path;
	temporary += ".tmp-" +
				 std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
	{
		std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
		if (!out || !out.write(reinterpret_cast<const char *>(packed.data()),
							static_cast<std::streamsize>(packed.size()))) {
			std::filesystem::remove(temporary, ec);
			throw std::runtime_error("write survey laser cache failed");
		}
	}
	std::filesystem::rename(temporary, path, ec);
	if (ec) {
		std::error_code ignored;
		std::filesystem::remove(temporary, ignored);
		throw std::filesystem::filesystem_error("rename survey laser cache", path, ec);
	}
}

std::string url_path_lower(std::string_view url)
{
	const auto end = url.find_first_of("?#");
	url = url.substr(0, end);
	std::string path(url);
	std::transform(path.begin(), path.end(), path.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return path;
}

fetch::Result<bool> extract_plain_las_from_zip(
		const std::filesystem::path &archive, const std::filesystem::path &destination)
{
	const auto unusable = [](std::string message) -> fetch::Failure {
		return {fetch::Failure::Kind::Unusable, std::move(message)};
	};
	std::ifstream input(archive, std::ios::binary);
	if (!input)
		return fetch::Failure{
				fetch::Failure::Kind::Other, "cannot open " + archive.string()};
	std::error_code ec;
	const auto file_size = std::filesystem::file_size(archive, ec);
	if (ec)
		return fetch::Failure{fetch::Failure::Kind::Other, ec.message()};
	if (file_size < 22)
		return unusable("short ZIP archive");
	const auto tail_size =
			static_cast<std::size_t>(std::min<std::uint64_t>(file_size, 22 + 65'535));
	std::vector<std::uint8_t> tail(tail_size);
	input.seekg(static_cast<std::streamoff>(file_size - tail_size));
	input.read(reinterpret_cast<char *>(tail.data()),
			static_cast<std::streamsize>(tail.size()));
	if (static_cast<std::size_t>(input.gcount()) != tail.size())
		return unusable("short ZIP end record");
	std::optional<std::size_t> end_record;
	for (std::size_t pos = tail.size() - 22 + 1; pos-- > 0;) {
		if (read_u32(tail.data() + pos) == 0x06054b50U) {
			const auto comment_length = read_u16(tail.data() + pos + 20);
			if (pos + 22 + comment_length == tail.size()) {
				end_record = pos;
				break;
			}
		}
	}
	if (!end_record)
		return unusable("ZIP end record not found");
	const auto *end = tail.data() + *end_record;
	const auto disk = read_u16(end + 4);
	const auto directory_disk = read_u16(end + 6);
	const auto entries_disk = read_u16(end + 8);
	const auto entries = read_u16(end + 10);
	const auto directory_size = read_u32(end + 12);
	const auto directory_offset = read_u32(end + 16);
	if (disk != 0 || directory_disk != 0 || entries_disk != entries ||
			entries == 0xffff || directory_size == 0xffffffffU ||
			directory_offset == 0xffffffffU)
		return unusable("multi-disk and ZIP64 archives are unsupported");
	if (directory_offset > file_size || directory_size > file_size - directory_offset)
		return unusable("ZIP central directory is outside the archive");
	const auto directory_end =
			static_cast<std::uint64_t>(directory_offset) + directory_size;

	std::optional<std::uint32_t> local_offset;
	std::uint16_t method = 0, flags = 0;
	std::uint32_t crc_expected = 0, compressed_size = 0, uncompressed_size = 0;
	std::uint64_t cursor = directory_offset;
	for (std::uint32_t index = 0; index < entries; ++index) {
		std::array<std::uint8_t, 46> header{};
		input.clear();
		input.seekg(static_cast<std::streamoff>(cursor));
		input.read(reinterpret_cast<char *>(header.data()), header.size());
		if (!input || read_u32(header.data()) != 0x02014b50U)
			return unusable("invalid ZIP central directory entry");
		const auto name_size = read_u16(header.data() + 28);
		const auto extra_size = read_u16(header.data() + 30);
		const auto comment_size = read_u16(header.data() + 32);
		const auto variable_size =
				static_cast<std::uint64_t>(name_size) + extra_size + comment_size;
		if (cursor > directory_end || header.size() > directory_end - cursor ||
				variable_size > directory_end - cursor - header.size())
			return unusable("short ZIP central directory entry");
		std::string name(name_size, '\0');
		input.read(name.data(), name.size());
		if (!input)
			return unusable("short ZIP filename");
		std::transform(name.begin(), name.end(), name.begin(),
				[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		const bool is_las = name.size() >= 4 && name.ends_with(".las");
		if (is_las && !local_offset) {
			flags = read_u16(header.data() + 8);
			method = read_u16(header.data() + 10);
			crc_expected = read_u32(header.data() + 16);
			compressed_size = read_u32(header.data() + 20);
			uncompressed_size = read_u32(header.data() + 24);
			local_offset = read_u32(header.data() + 42);
		}
		cursor += header.size() + variable_size;
	}
	if (cursor > directory_end)
		return unusable("ZIP central directory entries exceed declared size");
	if (!local_offset)
		return unusable(
				"ZIP contains no uncompressed LAS member (LAZ requires a decoder)");
	if ((flags & 1U) != 0 || (method != 0 && method != 8))
		return unusable("encrypted or unsupported ZIP compression method");
	if (uncompressed_size > (4ULL << 30))
		return unusable("unpacked LAS exceeds 4 GiB limit");
	if (method == 0 && compressed_size != uncompressed_size)
		return unusable("stored ZIP member has inconsistent sizes");

	std::array<std::uint8_t, 30> local{};
	input.clear();
	input.seekg(static_cast<std::streamoff>(*local_offset));
	input.read(reinterpret_cast<char *>(local.data()), local.size());
	if (!input || read_u32(local.data()) != 0x04034b50U)
		return unusable("invalid ZIP local member header");
	const auto local_name_size = read_u16(local.data() + 26);
	const auto local_extra_size = read_u16(local.data() + 28);
	if (read_u16(local.data() + 6) != flags || read_u16(local.data() + 8) != method)
		return unusable("ZIP local and central member headers disagree");
	const auto data_offset = static_cast<std::uint64_t>(*local_offset) + local.size() +
							 local_name_size + local_extra_size;
	if (data_offset > file_size || compressed_size > file_size - data_offset)
		return unusable("ZIP member data is outside the archive");
	input.clear();
	input.seekg(static_cast<std::streamoff>(data_offset));
	if (!input)
		return fetch::Failure{fetch::Failure::Kind::Other, "cannot seek to ZIP member"};
	std::ofstream output(destination, std::ios::binary | std::ios::trunc);
	if (!output)
		return fetch::Failure{
				fetch::Failure::Kind::Other, "cannot create " + destination.string()};
	std::array<std::uint8_t, 64 * 1024> in_buffer{}, out_buffer{};
	std::uint64_t remaining = compressed_size, produced = 0;
	uLong crc = crc32(0L, Z_NULL, 0);
	if (method == 0) {
		while (remaining > 0) {
			const auto chunk = static_cast<std::size_t>(
					std::min<std::uint64_t>(remaining, in_buffer.size()));
			input.read(reinterpret_cast<char *>(in_buffer.data()), chunk);
			if (static_cast<std::size_t>(input.gcount()) != chunk)
				return unusable("short stored ZIP member");
			output.write(reinterpret_cast<const char *>(in_buffer.data()), chunk);
			if (!output)
				return fetch::Failure{
						fetch::Failure::Kind::Other, "failed writing LAS member"};
			crc = crc32(crc, in_buffer.data(), static_cast<uInt>(chunk));
			remaining -= chunk;
			produced += chunk;
		}
	} else {
		z_stream stream{};
		if (inflateInit2(&stream, -MAX_WBITS) != Z_OK)
			return unusable("cannot initialize ZIP deflate decoder");
		int result = Z_OK;
		std::uint64_t loaded = 0;
		while (result != Z_STREAM_END) {
			if (stream.avail_in == 0 && loaded < compressed_size) {
				const auto chunk = static_cast<std::size_t>(std::min<std::uint64_t>(
						compressed_size - loaded, in_buffer.size()));
				input.read(reinterpret_cast<char *>(in_buffer.data()), chunk);
				if (static_cast<std::size_t>(input.gcount()) != chunk) {
					inflateEnd(&stream);
					return unusable("short deflated ZIP member");
				}
				loaded += chunk;
				stream.next_in = in_buffer.data();
				stream.avail_in = static_cast<uInt>(chunk);
			}
			stream.next_out = out_buffer.data();
			stream.avail_out = static_cast<uInt>(out_buffer.size());
			result = inflate(&stream, Z_NO_FLUSH);
			const auto chunk_out = out_buffer.size() - stream.avail_out;
			if (chunk_out) {
				if (chunk_out > uncompressed_size - std::min<std::uint64_t>(produced,
															uncompressed_size)) {
					inflateEnd(&stream);
					return unusable("deflated ZIP member exceeds its declared size");
				}
				output.write(
						reinterpret_cast<const char *>(out_buffer.data()), chunk_out);
				if (!output) {
					inflateEnd(&stream);
					return fetch::Failure{
							fetch::Failure::Kind::Other, "failed writing LAS member"};
				}
				crc = crc32(crc, out_buffer.data(), static_cast<uInt>(chunk_out));
				produced += chunk_out;
			}
			if (result != Z_OK && result != Z_STREAM_END) {
				inflateEnd(&stream);
				return unusable("invalid deflate stream in ZIP member");
			}
			if (result != Z_STREAM_END && stream.avail_in == 0 &&
					loaded == compressed_size && chunk_out == 0) {
				inflateEnd(&stream);
				return unusable("truncated deflate stream in ZIP member");
			}
		}
		inflateEnd(&stream);
		if (stream.total_in != compressed_size)
			return unusable("ZIP member has trailing compressed data");
	}
	output.flush();
	if (!output)
		return fetch::Failure{fetch::Failure::Kind::Other, "failed flushing LAS member"};
	if (produced != uncompressed_size || crc != crc_expected)
		return unusable("ZIP member size or CRC does not match");
	return true;
}
} // namespace

Tile Tile::empty(const std::string &source_url)
{
	Tile tile;
	tile.n_all.resize(CELLS);
	tile.n_gnd.resize(CELLS);
	tile.n_obj.resize(CELLS);
	tile.n_bld.resize(CELLS);
	tile.n_veg.resize(CELLS);
	tile.h_obj.resize(CELLS);
	tile.h_bld.resize(CELLS);
	tile.i_gnd.resize(CELLS);
	tile.blocks.resize(BLOCKS * BLOCKS);
	tile.origin = source_url;
	return tile;
}

bool Tile::valid() const
{
	return n_all.size() == CELLS && n_gnd.size() == CELLS && n_obj.size() == CELLS &&
		   n_bld.size() == CELLS && n_veg.size() == CELLS && h_obj.size() == CELLS &&
		   h_bld.size() == CELLS && i_gnd.size() == CELLS &&
		   blocks.size() == BLOCKS * BLOCKS;
}

bool Tile::has(const std::vector<bool> &wanted) const
{
	for (std::size_t i = 0; i < wanted.size(); ++i)
		if (wanted[i] && (i >= blocks.size() || !blocks[i]))
			return false;
	return true;
}

bool Tile::write(const std::filesystem::path &path) const
{
	if (origin.size() > std::numeric_limits<std::uint16_t>::max() || !valid())
		return false;
	std::vector<std::uint8_t> raw;
	raw.reserve(8 + 1 + BLOCKS * BLOCKS + 2 + origin.size() + 9 * CELLS);
	raw.insert(raw.end(), MAGIC.begin(), MAGIC.end());
	raw.push_back(TARGET_DENSITY_TENTHS);
	for (const bool block : blocks)
		raw.push_back(block ? 1 : 0);
	append_u16(raw, static_cast<std::uint16_t>(origin.size()));
	raw.insert(raw.end(), origin.begin(), origin.end());
	append_layer(raw, n_all);
	append_layer(raw, n_gnd);
	append_layer(raw, n_obj);
	append_layer(raw, n_bld);
	append_layer(raw, n_veg);
	append_layer(raw, h_obj);
	append_layer(raw, h_bld);
	append_layer(raw, i_gnd);
	std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw.size()));
	const auto size = ZSTD_compress(
			compressed.data(), compressed.size(), raw.data(), raw.size(), 9);
	if (ZSTD_isError(size))
		return false;
	compressed.resize(size);
	try {
		write_unique(path, compressed);
	} catch (...) {
		return false;
	}
	return true;
}

std::optional<Tile> Tile::read(const std::filesystem::path &path)
{
	auto raw_value = decompress_file(path);
	if (!raw_value)
		return std::nullopt;
	auto &raw = *raw_value;
	const auto fixed = 8 + 1 + BLOCKS * BLOCKS + 2;
	if (raw.size() < fixed || !std::equal(MAGIC.begin(), MAGIC.end(), raw.begin()) ||
			raw[8] != TARGET_DENSITY_TENTHS)
		return std::nullopt;
	Tile tile;
	tile.blocks.reserve(BLOCKS * BLOCKS);
	for (std::size_t i = 0; i < BLOCKS * BLOCKS; ++i)
		tile.blocks.push_back(raw[9 + i] != 0);
	std::size_t offset = 9 + BLOCKS * BLOCKS;
	const auto origin_size = read_u16(raw.data() + offset);
	offset += 2;
	if (origin_size > raw.size() - std::min(offset, raw.size()))
		return std::nullopt;
	tile.origin.assign(reinterpret_cast<const char *>(raw.data() + offset), origin_size);
	offset += origin_size;
	if (!read_layer(raw, offset, tile.n_all, CELLS) ||
			!read_layer(raw, offset, tile.n_gnd, CELLS) ||
			!read_layer(raw, offset, tile.n_obj, CELLS) ||
			!read_layer(raw, offset, tile.n_bld, CELLS) ||
			!read_layer(raw, offset, tile.n_veg, CELLS) ||
			!read_layer(raw, offset, tile.h_obj, CELLS) ||
			!read_layer(raw, offset, tile.h_bld, CELLS) ||
			!read_layer(raw, offset, tile.i_gnd, CELLS) || offset != raw.size())
		return std::nullopt;
	return tile;
}

void Tile::take_blocks(const Tile &other, const std::vector<bool> &fill)
{
	if (!valid() || !other.valid())
		return;
	for (std::size_t block = 0; block < std::min(fill.size(), blocks.size()); ++block) {
		if (!fill[block])
			continue;
		const auto bx = block % BLOCKS, by = block / BLOCKS;
		for (std::size_t row = by * BLOCK; row < (by + 1) * BLOCK; ++row) {
			const auto begin = row * TILE + bx * BLOCK;
			const auto end = begin + BLOCK;
			auto copy = [&](auto &to, const auto &from) {
				std::copy(from.begin() + begin, from.begin() + end, to.begin() + begin);
			};
			copy(n_all, other.n_all);
			copy(n_gnd, other.n_gnd);
			copy(n_obj, other.n_obj);
			copy(n_bld, other.n_bld);
			copy(n_veg, other.n_veg);
			copy(h_obj, other.h_obj);
			copy(h_bld, other.h_bld);
			copy(i_gnd, other.i_gnd);
		}
		blocks[block] = true;
	}
}

struct Measure::Impl
{
	const source::Sheet &source;
	double east0;
	double north1;
	std::array<std::size_t, 4> window;
	std::vector<std::uint32_t> n_all = std::vector<std::uint32_t>(CELLS);
	std::vector<bool> footprint;
	std::vector<double> ground_z_sum = std::vector<double>(CELLS);
	std::vector<BlockPoints> blocks = std::vector<BlockPoints>(BLOCKS * BLOCKS);
	std::optional<crs::Affine> to_grid;

	Impl(const source::Sheet &sheet, std::int32_t east, std::int32_t north,
			std::array<std::size_t, 4> cells) :
			source(sheet), east0(static_cast<double>(east) * TILE),
			north1(static_cast<double>(north + 1) * TILE), window(cells)
	{
		if (!source.source->classes.footprint.empty())
			footprint.resize(CELLS);
	}

	void add(const copc::PointLayout &layout, std::span<const std::uint8_t> rec)
	{
		if (rec.size() < 17 || layout.record_len < 17)
			return;
		const auto xi = std::bit_cast<std::int32_t>(read_u32(rec.data()));
		const auto yi = std::bit_cast<std::int32_t>(read_u32(rec.data() + 4));
		double x = static_cast<double>(xi) * layout.scale[0] + layout.offset[0];
		double y = static_cast<double>(yi) * layout.scale[1] + layout.offset[1];
		if (to_grid) {
			const auto transformed = to_grid->apply(x, y);
			x = transformed.first;
			y = transformed.second;
		}
		const auto column = std::floor(x - east0);
		const auto row = std::floor(north1 - y);
		const auto [c0, r0, c1, r1] = window;
		if (column < static_cast<double>(c0) || column >= static_cast<double>(c1) ||
				row < static_cast<double>(r0) || row >= static_cast<double>(r1) ||
				column < 0.0 || row < 0.0 || column >= TILE || row >= TILE)
			return;
		const auto col = static_cast<std::size_t>(column);
		const auto r = static_cast<std::size_t>(row);
		const auto cell = r * TILE + col;
		const auto format = layout.format;
		const auto classification = format >= 6 ? rec[16] : rec[15] & 0x1f;
		const auto ret = format >= 6 ? rec[14] & 0x0f : rec[14] & 0x07;
		const auto nret = format >= 6 ? rec[14] >> 4 : (rec[14] >> 3) & 0x07;
		const auto &classes = source.source->classes;
		if (std::find(classes.footprint.begin(), classes.footprint.end(),
					classification) != classes.footprint.end()) {
			footprint[cell] = true;
			return;
		}
		++n_all[cell];
		const auto zi = std::bit_cast<std::int32_t>(read_u32(rec.data() + 8));
		const auto z = static_cast<double>(zi) * layout.scale[2] + layout.offset[2];
		auto &block = blocks[(r / BLOCK) * BLOCKS + col / BLOCK];
		const auto local = (r % BLOCK) * BLOCK + col % BLOCK;
		const auto has_class = [classification](const std::vector<std::uint8_t> &list) {
			return std::find(list.begin(), list.end(), classification) != list.end();
		};
		if (has_class(classes.ground)) {
			const auto intensity =
					has_class(classes.vegetation) ? 0 : read_u16(rec.data() + 12);
			block.ground.push((static_cast<std::uint32_t>(local) << 16) | intensity);
			ground_z_sum[cell] += z;
			return;
		}
		std::uint8_t flags = 0;
		if (has_class(classes.building)) {
			flags = BUILDING;
		} else if (has_class(classes.object)) {
			const bool passes = nret > 1 && ret < nret;
			const bool plant = classes.vegetation.empty()
									   ? passes
									   : has_class(classes.vegetation) ||
												 (passes && has_class(classes.unsorted));
			flags = (has_class(classes.unsorted) ? UNSORTED : 0) | (plant ? PLANT : 0);
		} else {
			return;
		}
		block.above_at.push(static_cast<std::uint16_t>(local));
		block.above_flags.push(flags);
		block.above_z.push(static_cast<float>(z));
	}

	Tile finish(const std::string &origin)
	{
		Tile result = Tile::empty(origin);
		for (std::size_t i = 0; i < CELLS; ++i)
			result.n_all[i] =
					static_cast<std::uint8_t>(std::min<std::uint32_t>(n_all[i], 255));
		std::vector<std::uint32_t> start(BLOCK_CELLS + 1), fill(BLOCK_CELLS);
		std::vector<std::uint32_t> n_gnd(CELLS);
		std::vector<std::uint16_t> values;
		for (std::size_t b = 0; b < blocks.size(); ++b) {
			const auto &ground = blocks[b].ground;
			if (!ground.size())
				continue;
			std::fill(start.begin(), start.end(), 0);
			ground.each([&](std::uint32_t packed) { ++start[(packed >> 16) + 1]; });
			for (std::size_t i = 0; i < BLOCK_CELLS; ++i)
				start[i + 1] += start[i];
			std::copy_n(start.begin(), BLOCK_CELLS, fill.begin());
			values.resize(ground.size());
			ground.each([&](std::uint32_t packed) {
				const auto local = packed >> 16;
				values[fill[local]++] = static_cast<std::uint16_t>(packed);
			});
			for (std::size_t local = 0; local < BLOCK_CELLS; ++local) {
				auto first = values.begin() + start[local];
				auto last = values.begin() + start[local + 1];
				if (first == last)
					continue;
				std::sort(first, last);
				const auto cell = tile_cell(b, local);
				const auto count = static_cast<std::uint32_t>(last - first);
				n_gnd[cell] = count;
				result.n_gnd[cell] =
						static_cast<std::uint8_t>(std::min<std::uint32_t>(count, 255));
				const auto median = static_cast<std::size_t>(
						std::floor(static_cast<double>(last - first - 1) * 0.5));
				result.i_gnd[cell] = *(first + median);
			}
		}
		auto ground_z = fill_ground(ground_z_sum, n_gnd);
		std::vector<bool> roof;
		if (!footprint.empty())
			roof = grow_by_one(footprint);
		const auto settle =
				[&](std::size_t cell, std::uint8_t flags,
						float z) -> std::optional<std::pair<std::uint8_t, float>> {
			if (!(flags & PLANT) && !roof.empty() && roof[cell])
				flags = BUILDING;
			const auto height = z - ground_z[cell];
			if ((flags & BUILDING) || height > 0.25f)
				return std::pair{flags, height};
			return std::nullopt;
		};
		const bool guess_roofs = !source.source->classes.unsorted.empty() &&
								 roofs_unsorted(blocks, settle);
		std::vector<std::uint32_t> n_obj(BLOCK_CELLS), n_bld(BLOCK_CELLS),
				n_veg(BLOCK_CELLS), fill_bld(BLOCK_CELLS);
		std::vector<std::tuple<std::uint32_t, float, std::uint8_t>> kept;
		std::vector<float> heights;
		for (std::size_t b = 0; b < blocks.size(); ++b) {
			auto &block = blocks[b];
			if (!block.above_at.size())
				continue;
			kept.clear();
			kept.reserve(block.above_at.size());
			for (std::size_t i = 0; i < block.above_at.size(); ++i) {
				const auto local = block.above_at.at(i);
				if (auto settled = settle(tile_cell(b, local), block.above_flags.at(i),
							block.above_z.at(i)))
					kept.emplace_back(local, settled->second, settled->first);
			}
			block = BlockPoints{};
			if (guess_roofs)
				find_roofs(kept);
			std::fill(n_obj.begin(), n_obj.end(), 0);
			std::fill(n_bld.begin(), n_bld.end(), 0);
			std::fill(n_veg.begin(), n_veg.end(), 0);
			for (const auto &[local, height, flags] : kept) {
				(void)height;
				if (flags & BUILDING)
					++n_bld[local];
				else {
					++n_obj[local];
					if (flags & PLANT)
						++n_veg[local];
				}
			}
			start[0] = 0;
			for (std::size_t local = 0; local < BLOCK_CELLS; ++local) {
				start[local + 1] = start[local] + n_obj[local] + n_bld[local];
				fill[local] = start[local];
				fill_bld[local] = start[local] + n_obj[local];
			}
			heights.assign(start[BLOCK_CELLS], 0.0f);
			for (const auto &[local, height, flags] : kept) {
				auto &at = (flags & BUILDING) ? fill_bld[local] : fill[local];
				heights[at++] = height;
			}
			for (std::size_t local = 0; local < BLOCK_CELLS; ++local) {
				const auto cell = tile_cell(b, local);
				result.n_obj[cell] = static_cast<std::uint8_t>(
						std::min<std::uint32_t>(n_obj[local], 255));
				result.n_bld[cell] = static_cast<std::uint8_t>(
						std::min<std::uint32_t>(n_bld[local], 255));
				result.n_veg[cell] = static_cast<std::uint8_t>(
						std::min<std::uint32_t>(n_veg[local], 255));
				const auto begin = static_cast<std::size_t>(start[local]);
				const auto middle = begin + n_obj[local];
				const auto end = static_cast<std::size_t>(start[local + 1]);
				auto object = heights.begin() + begin;
				auto building = heights.begin() + middle;
				auto after = heights.begin() + end;
				if (object != building) {
					std::sort(object, building);
					const auto index = static_cast<std::size_t>(std::floor(
							static_cast<double>(building - object - 1) * 0.95));
					result.h_obj[cell] = static_cast<std::uint8_t>(
							std::clamp(std::lround(object[index] * 5.0f), 0L, 255L));
				}
				if (building != after) {
					std::sort(building, after);
					const auto index = static_cast<std::size_t>(
							std::floor(static_cast<double>(after - building - 1) * 0.95));
					result.h_bld[cell] = static_cast<std::uint8_t>(
							std::clamp(std::lround(building[index] * 5.0f), 0L, 255L));
				}
			}
		}
		return result;
	}
};

Measure::Measure(const source::Sheet &source, std::int32_t tile_east,
		std::int32_t tile_north, std::array<std::size_t, 4> window) :
		impl_(std::make_unique<Impl>(source, tile_east, tile_north, window))
{
}

Measure::~Measure() = default;
Measure::Measure(Measure &&) noexcept = default;
Measure &Measure::operator=(Measure &&) noexcept = default;

void Measure::set_to_grid(std::optional<crs::Affine> transform)
{
	impl_->to_grid = std::move(transform);
}

void Measure::add(const copc::PointLayout &layout, std::span<const std::uint8_t> record)
{
	impl_->add(layout, record);
}

Tile Measure::finish(const std::string &origin)
{
	return impl_->finish(origin);
}

ept::ReadResult read_ept(const source::Sheet &sheet, std::int32_t tile_east,
		std::int32_t tile_north, const std::vector<bool> &wanted, const std::string &urls,
		Tile &target)
{
	if (!target.valid() || target.origin != urls)
		target = Tile::empty(urls);
	if (!sheet.source || sheet.source->laser.kind != source::Laser::Kind::Ept)
		return fetch::Failure{fetch::Failure::Kind::Unusable,
				"EPT reader used for a non-EPT survey source"};
	std::vector<bool> fill(BLOCKS * BLOCKS, false);
	std::size_t bx0 = BLOCKS, by0 = BLOCKS, bx1 = 0, by1 = 0;
	for (std::size_t b = 0; b < fill.size() && b < wanted.size(); ++b) {
		fill[b] = wanted[b] && (b >= target.blocks.size() || !target.blocks[b]);
		if (!fill[b])
			continue;
		bx0 = std::min(bx0, b % BLOCKS);
		bx1 = std::max(bx1, b % BLOCKS + 1);
		by0 = std::min(by0, b / BLOCKS);
		by1 = std::max(by1, b / BLOCKS + 1);
	}
	if (bx1 == 0)
		return copc::ReadStats{};
	const auto pad = static_cast<std::size_t>(PAD_M);
	const std::array<std::size_t, 4> cells{bx0 * BLOCK > pad ? bx0 * BLOCK - pad : 0,
			by0 * BLOCK > pad ? by0 * BLOCK - pad : 0, std::min(bx1 * BLOCK + pad, TILE),
			std::min(by1 * BLOCK + pad, TILE)};
	const double east0 = static_cast<double>(tile_east) * TILE;
	const double north1 = static_cast<double>(tile_north + 1) * TILE;
	const source::Rect window{
			east0 + cells[0], north1 - cells[3], east0 + cells[2], north1 - cells[1]};
	const double area = (window[2] - window[0]) * (window[3] - window[1]);
	const auto centre_x = (window[0] + window[2]) * 0.5;
	const auto centre_y = (window[1] + window[3]) * 0.5;
	Measure measure(sheet, tile_east, tile_north, cells);
	std::mutex measure_mutex;
	copc::ReadStats total{};
	bool found = false;
	std::optional<fetch::Failure> unusable;
	std::size_t start = 0;
	while (start <= urls.size()) {
		const auto end = urls.find('\n', start);
		const auto url = urls.substr(
				start, (end == std::string::npos ? urls.size() : end) - start);
		if (!url.empty()) {
			const auto transform = crs::Affine::fit({crs::Kind::WebMercator},
					sheet.projection, centre_x, centre_y, 250.0);
			measure.set_to_grid(transform);
			const auto source_window =
					transform.inverse().rect(window[0], window[1], window[2], window[3]);
			const ept::Visit visit = [&](const copc::PointLayout &layout,
											 std::span<const std::uint8_t> records) {
				if (layout.record_len == 0)
					return;
				std::lock_guard lock(measure_mutex);
				for (std::size_t offset = 0; offset + layout.record_len <= records.size();
						offset += layout.record_len)
					measure.add(layout, records.subspan(offset, layout.record_len));
			};
			auto result = ept::read(url, source_window, TARGET_DENSITY, 8, visit);
			if (auto *failure = std::get_if<fetch::Failure>(&result)) {
				if (failure->kind == fetch::Failure::Kind::Unusable) {
					unusable = *failure;
				} else if (failure->kind != fetch::Failure::Kind::Missing) {
					return *failure;
				}
			} else {
				const auto &stats = std::get<copc::ReadStats>(result);
				found = true;
				total.level = std::max(total.level, stats.level);
				total.nodes += stats.nodes;
				total.requests += stats.requests;
				total.bytes += stats.bytes;
				total.points += stats.points;
			}
		}
		if (end == std::string::npos)
			break;
		start = end + 1;
	}
	if (!found) {
		if (unusable)
			return *unusable;
		return fetch::Failure{fetch::Failure::Kind::Missing, "no EPT source file found"};
	}
	fetch::count_laser_area(area);
	const auto measured = measure.finish(urls);
	target.take_blocks(measured, fill);
	return total;
}

fetch::Result<Tile> measure_las_file(const std::filesystem::path &path,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north,
		const std::string &origin)
{
	auto unusable = [](std::string message) -> fetch::Failure {
		return {fetch::Failure::Kind::Unusable, std::move(message)};
	};
	std::ifstream file(path, std::ios::binary);
	if (!file)
		return fetch::Failure{
				fetch::Failure::Kind::Other, "cannot open " + path.string()};
	std::vector<std::uint8_t> header(375);
	file.seekg(0);
	file.read(reinterpret_cast<char *>(header.data()),
			static_cast<std::streamsize>(header.size()));
	const auto header_bytes = static_cast<std::size_t>(file.gcount());
	if (header_bytes < 227 || std::memcmp(header.data(), "LASF", 4) != 0)
		return unusable("not a LAS/LAZ file");
	const auto u16 = [&](std::size_t offset) { return read_u16(header.data() + offset); };
	const auto u32 = [&](std::size_t offset) { return read_u32(header.data() + offset); };
	const auto u64 = [&](std::size_t offset) {
		std::uint64_t value = 0;
		for (unsigned i = 0; i < 8; ++i)
			value |= static_cast<std::uint64_t>(header[offset + i]) << (8 * i);
		return value;
	};
	const auto f64 = [&](std::size_t offset) {
		const auto bits = u64(offset);
		double value = 0.0;
		std::memcpy(&value, &bits, sizeof(value));
		return value;
	};
	const auto header_size = static_cast<std::uint64_t>(u16(94));
	const auto point_offset = static_cast<std::uint64_t>(u32(96));
	const auto vlr_count = u32(100);
	const auto format_byte = header[104];
	const bool compressed = (format_byte & 0x80) != 0;
	copc::PointLayout layout;
	layout.format = static_cast<std::uint8_t>(format_byte & 0x3f);
	layout.record_len = u16(105);
	std::uint64_t point_count = u32(107);
	if (point_count == 0 && header_bytes >= 255 && header[25] >= 4)
		point_count = u64(247);
	for (std::size_t axis = 0; axis < 3; ++axis) {
		layout.scale[axis] = f64(131 + axis * 8);
		layout.offset[axis] = f64(155 + axis * 8);
	}
	if (const auto error = layout.check())
		return unusable(*error);
	if (header_size < 227 || point_offset < header_size)
		return unusable("invalid LAS header or point offset");
	std::error_code ec;
	const auto file_bytes = std::filesystem::file_size(path, ec);
	if (ec)
		return fetch::Failure{fetch::Failure::Kind::Other, ec.message()};
	if (header_size > file_bytes || point_offset > file_bytes)
		return unusable("LAS point data starts past end of file");
	std::uint64_t position = header_size;
	bool has_laz_vlr = false;
	for (std::uint32_t i = 0; i < vlr_count; ++i) {
		if (position > file_bytes || 54 > file_bytes - position)
			return unusable("short LAS variable-length record header");
		std::array<std::uint8_t, 54> vlr{};
		file.clear();
		file.seekg(static_cast<std::streamoff>(position));
		file.read(reinterpret_cast<char *>(vlr.data()),
				static_cast<std::streamsize>(vlr.size()));
		if (!file)
			return unusable("short LAS variable-length record header");
		std::string user(vlr.begin() + 2, vlr.begin() + 18);
		while (!user.empty() && user.back() == '\0')
			user.pop_back();
		const auto record_id = read_u16(vlr.data() + 18);
		const auto record_size = read_u16(vlr.data() + 20);
		position += 54;
		if (record_size > file_bytes - position)
			return unusable("short LAS variable-length record payload");
		if (user == "laszip encoded" && record_id == 22204)
			has_laz_vlr = true;
		position += record_size;
	}
	if (compressed || has_laz_vlr)
		return unusable("LAZ point decompressor is unavailable");
	if (point_count > (file_bytes - point_offset) / layout.record_len)
		return unusable("short LAS point data");
	file.clear();
	file.seekg(static_cast<std::streamoff>(point_offset));
	if (!file)
		return fetch::Failure{
				fetch::Failure::Kind::Other, "cannot seek to LAS point data"};
	Measure measure(sheet, tile_east, tile_north);
	const auto batch_points = std::max<std::size_t>(
			1, std::min<std::size_t>(50'000, (64U << 20) / layout.record_len));
	std::vector<std::uint8_t> records(batch_points * layout.record_len);
	while (point_count > 0) {
		const auto batch = static_cast<std::size_t>(
				std::min<std::uint64_t>(point_count, batch_points));
		const auto bytes = batch * layout.record_len;
		file.read(reinterpret_cast<char *>(records.data()),
				static_cast<std::streamsize>(bytes));
		if (static_cast<std::size_t>(file.gcount()) != bytes)
			return unusable("short LAS point data");
		for (std::size_t offset = 0; offset < bytes; offset += layout.record_len)
			measure.add(layout, std::span(records).subspan(offset, layout.record_len));
		point_count -= batch;
	}
	auto result = measure.finish(origin);
	std::fill(result.blocks.begin(), result.blocks.end(), true);
	fetch::count_laser_area(static_cast<double>(TILE * TILE));
	return result;
}

fetch::Result<std::optional<Tile>> fetch_tile(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north,
		const std::vector<bool> &wanted, const std::vector<std::string> &urls)
{
	auto lock_key = std::hash<std::string>{}(root.string());
	lock_key ^= std::hash<std::string>{}(sheet.cache_dir()) + 0x9e3779b9 +
				(lock_key << 6) + (lock_key >> 2);
	lock_key ^= std::hash<std::int32_t>{}(tile_east) + 0x9e3779b9 + (lock_key << 6) +
				(lock_key >> 2);
	lock_key ^= std::hash<std::int32_t>{}(tile_north) + 0x9e3779b9 + (lock_key << 6) +
				(lock_key >> 2);
	std::lock_guard tile_lock(tile_fetch_locks[lock_key % tile_fetch_locks.size()]);
	const auto cache = laser_cache::cache_path(root, sheet, tile_east, tile_north);
	auto have = Tile::read(cache);
	if (have && have->valid() && have->has(wanted))
		return std::optional<Tile>(std::move(*have));
	const auto needed =
			laser_cache::lacking_blocks(root, sheet, tile_east, tile_north, wanted);
	if (std::none_of(needed.begin(), needed.end(), [](bool b) { return b; }))
		return std::optional<Tile>{};
	if (urls.empty()) {
		laser_cache::mark_absent(root, sheet, tile_east, tile_north, needed);
		return std::optional<Tile>{};
	}
	std::error_code ec;
	std::filesystem::create_directories(cache.parent_path(), ec);
	if (ec)
		return fetch::Failure{fetch::Failure::Kind::Other, ec.message()};
	if (sheet.source->laser.kind == source::Laser::Kind::Ept) {
		std::string joined;
		for (const auto &url : urls) {
			if (!joined.empty())
				joined.push_back('\n');
			joined += url;
		}
		Tile merged = have && have->valid() ? std::move(*have) : Tile::empty(joined);
		auto result = read_ept(sheet, tile_east, tile_north, wanted, joined, merged);
		if (const auto *failure = std::get_if<fetch::Failure>(&result)) {
			if (failure->kind == fetch::Failure::Kind::Missing ||
					failure->kind == fetch::Failure::Kind::Unusable) {
				if (failure->kind == fetch::Failure::Kind::Unusable)
					std::cerr << "Warning: laser scan " << tile_east << '_' << tile_north
							  << " passed over: " << failure->message << '\n';
				laser_cache::mark_absent(root, sheet, tile_east, tile_north, needed);
				return std::optional<Tile>{};
			}
			return *failure;
		}
		if (fetch::past_laser_limit())
			return fetch::Failure{fetch::Failure::Kind::Other, fetch::PAST_LIMIT};
		if (!merged.write(cache))
			net::note_uncached("laser scans");
		return std::optional<Tile>(std::move(merged));
	}

	bool found = false;
	std::optional<fetch::Failure> unusable;
	for (const auto &url : urls) {
		if (fetch::remaining_laser_bytes() == 0)
			return fetch::Failure{fetch::Failure::Kind::Other, fetch::PAST_LIMIT};
		const auto path = url_path_lower(url);
		const bool zip_las = path.ends_with(".las.zip");
		const bool plain_las = path.ends_with(".las");
		if ((!plain_las && !zip_las) ||
				(zip_las && sheet.source->laser.kind != source::Laser::Kind::Whole &&
						sheet.source->laser.kind != source::Laser::Kind::Copc) ||
				(plain_las && sheet.source->laser.kind != source::Laser::Kind::Whole)) {
			unusable = fetch::Failure{fetch::Failure::Kind::Unusable,
					"this source file needs a LAZ/COPC decoder or has an unsupported format"};
			continue;
		}
		const auto sequence = download_sequence.fetch_add(1, std::memory_order_relaxed);
		const auto temporary = cache.string() + ".download." + std::to_string(sequence);
		struct RemoveFile
		{
			std::filesystem::path path;
			~RemoveFile()
			{
				std::error_code ignored;
				std::filesystem::remove(path, ignored);
			}
		} remove{temporary};
		const auto extracted = temporary + ".las";
		RemoveFile remove_extracted{extracted};
		if (const auto size = fetch::content_length(url); size && *size > 0) {
			laser_cache::note_size(root, sheet, tile_east, tile_north, *size);
			if (*size > fetch::remaining_laser_bytes())
				return fetch::Failure{fetch::Failure::Kind::Other, fetch::PAST_LIMIT};
		}
		[[maybe_unused]] WholeDownloadPermit download_permit(whole_file_downloads);
		auto downloaded = fetch::download_file(url, temporary);
		if (const auto *failure = std::get_if<fetch::Failure>(&downloaded)) {
			if (failure->kind == fetch::Failure::Kind::Missing)
				continue;
			if (failure->kind == fetch::Failure::Kind::Unusable) {
				unusable = *failure;
				continue;
			}
			return *failure;
		}
		std::filesystem::path las_path = temporary;
		if (zip_las) {
			auto extraction = extract_plain_las_from_zip(temporary, extracted);
			if (const auto *failure = std::get_if<fetch::Failure>(&extraction)) {
				if (failure->kind == fetch::Failure::Kind::Unusable) {
					unusable = *failure;
					continue;
				}
				return *failure;
			}
			las_path = extracted;
		}
		auto measured = measure_las_file(las_path, sheet, tile_east, tile_north, url);
		if (auto *failure = std::get_if<fetch::Failure>(&measured)) {
			if (failure->kind == fetch::Failure::Kind::Unusable) {
				unusable = *failure;
				continue;
			}
			return *failure;
		}
		Tile merged = have && have->valid() ? std::move(*have) : Tile::empty(url);
		merged.take_blocks(std::get<Tile>(measured), needed);
		found = true;
		if (!merged.write(cache))
			net::note_uncached("laser scans");
		have = std::move(merged);
	}
	if (found && have)
		return std::optional<Tile>(std::move(*have));
	if (unusable)
		std::cerr << "Warning: laser scan " << tile_east << '_' << tile_north
				  << " passed over: " << unusable->message << '\n';
	laser_cache::mark_absent(root, sheet, tile_east, tile_north, needed);
	return std::optional<Tile>{};
}
} // namespace arnis::survey::laser
