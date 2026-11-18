#include "laser_cache.h"

#include <zstd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>

namespace arnis::survey::laser_cache
{
namespace
{
constexpr std::array<std::uint8_t, 8> MAGIC{'A', 'R', 'N', 'S', 'L', 'A', 'S', '4'};
constexpr std::uint8_t TARGET_DENSITY_TENTHS = 70;
constexpr std::size_t FIXED_HEAD = 8 + 1 + BLOCK_COUNT + 2;

std::filesystem::path marker_path(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t east, std::int32_t north,
		const char *suffix)
{
	return root / sheet.cache_dir() /
		   ("tile_" + std::to_string(east) + "_" + std::to_string(north) + suffix);
}

bool fresh(const std::filesystem::path &path, std::chrono::hours max_age)
{
	std::error_code ec;
	const auto modified = std::filesystem::last_write_time(path, ec);
	if (ec)
		return false;
	const auto age = std::filesystem::file_time_type::clock::now() - modified;
	return age >= std::filesystem::file_time_type::duration::zero() && age < max_age;
}

std::optional<std::vector<std::uint8_t>> decompress_head(
		const std::filesystem::path &path)
{
	std::ifstream file(path, std::ios::binary);
	if (!file)
		return std::nullopt;

	using StreamPtr = std::unique_ptr<ZSTD_DStream, decltype(&ZSTD_freeDStream)>;
	StreamPtr stream(ZSTD_createDStream(), &ZSTD_freeDStream);
	if (!stream)
		return std::nullopt;
	const auto init = ZSTD_initDStream(stream.get());
	if (ZSTD_isError(init)) {
		return std::nullopt;
	}
	std::array<std::uint8_t, 16384> compressed{};
	std::array<std::uint8_t, 32768> decoded{};
	std::vector<std::uint8_t> raw;
	std::size_t required = FIXED_HEAD;
	std::size_t remaining = 1;
	while (raw.size() < required) {
		file.read(reinterpret_cast<char *>(compressed.data()), compressed.size());
		const auto read = static_cast<std::size_t>(file.gcount());
		if (!read)
			return std::nullopt;
		ZSTD_inBuffer input{compressed.data(), read, 0};
		while (input.pos < input.size && raw.size() < required) {
			ZSTD_outBuffer output{decoded.data(), decoded.size(), 0};
			const auto input_before = input.pos;
			remaining = ZSTD_decompressStream(stream.get(), &output, &input);
			if (ZSTD_isError(remaining))
				return std::nullopt;
			raw.insert(raw.end(), decoded.begin(), decoded.begin() + output.pos);
			if (raw.size() >= FIXED_HEAD && required == FIXED_HEAD) {
				const auto offset = 8 + 1 + BLOCK_COUNT;
				const auto origin_size = std::uint16_t(raw[offset]) |
										 (std::uint16_t(raw[offset + 1]) << 8);
				required += origin_size;
			}
			if (remaining == 0 && raw.size() < required)
				return std::nullopt;
			if (input.pos == input_before && output.pos == 0)
				return std::nullopt;
		}
	}
	if (raw.size() < required || raw.size() < FIXED_HEAD ||
			!std::equal(MAGIC.begin(), MAGIC.end(), raw.begin()) ||
			raw[8] != TARGET_DENSITY_TENTHS)
		return std::nullopt;
	return raw;
}

std::optional<BlockMask> absent_blocks(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t east, std::int32_t north)
{
	const auto path = marker_path(root, sheet, east, north, ".absent");
	const auto max_age = sheet.source->laser.locate.kind == source::Locate::Kind::Urls
								 ? std::chrono::hours(24 * 30)
								 : std::chrono::hours(24 * 7);
	if (!fresh(path, max_age))
		return std::nullopt;

	std::ifstream file(path, std::ios::binary);
	if (!file)
		return std::nullopt;
	if (sheet.source->laser.locate.kind == source::Locate::Kind::Urls)
		return BlockMask(BLOCK_COUNT, true);
	std::vector<std::uint8_t> bytes(
			(std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	if (bytes.size() != BLOCK_COUNT)
		return std::nullopt;
	BlockMask mask;
	mask.reserve(BLOCK_COUNT);
	for (auto byte : bytes)
		mask.push_back(byte != 0);
	return mask;
}
} // namespace

BlockMask blocks_for(
		std::int32_t tile_east, std::int32_t tile_north, const source::Rect &rect)
{
	const double west = static_cast<double>(tile_east) * TILE_METRES;
	const double north = static_cast<double>(tile_north + 1) * TILE_METRES;
	BlockMask result(BLOCK_COUNT, false);
	for (std::size_t by = 0; by < BLOCKS_PER_TILE; ++by) {
		const double block_north = north - by * BLOCK_METRES;
		const double block_south = north - (by + 1) * BLOCK_METRES;
		for (std::size_t bx = 0; bx < BLOCKS_PER_TILE; ++bx) {
			const double block_west = west + bx * BLOCK_METRES;
			const double block_east = west + (bx + 1) * BLOCK_METRES;
			result[by * BLOCKS_PER_TILE + bx] =
					rect[0] < block_east && rect[2] > block_west &&
					rect[1] < block_north && rect[3] > block_south;
		}
	}
	return result;
}

std::filesystem::path cache_path(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north)
{
	return root / sheet.cache_dir() /
		   ("tile_" + std::to_string(tile_east) + "_" + std::to_string(tile_north) +
				   ".bin");
}

std::optional<BlockMask> cached_blocks(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north)
{
	const auto raw = decompress_head(cache_path(root, sheet, tile_east, tile_north));
	if (!raw)
		return std::nullopt;
	BlockMask blocks;
	blocks.reserve(BLOCK_COUNT);
	for (std::size_t i = 0; i < BLOCK_COUNT; ++i)
		blocks.push_back((*raw)[9 + i] != 0);
	return blocks;
}

BlockMask lacking_blocks(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted)
{
	const auto have = cached_blocks(root, sheet, tile_east, tile_north);
	const auto absent = absent_blocks(root, sheet, tile_east, tile_north);
	BlockMask missing(std::min(wanted.size(), BLOCK_COUNT), false);
	for (std::size_t i = 0; i < missing.size(); ++i)
		missing[i] = wanted[i] && (!have || i >= have->size() || !(*have)[i]) &&
					 (!absent || i >= absent->size() || !(*absent)[i]);
	return missing;
}

double missing_km2(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted)
{
	const auto missing = lacking_blocks(root, sheet, tile_east, tile_north, wanted);
	return static_cast<double>(std::count(missing.begin(), missing.end(), true)) *
		   (BLOCK_METRES * BLOCK_METRES) / 1'000'000.0;
}

bool is_cached(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted)
{
	const auto missing = lacking_blocks(root, sheet, tile_east, tile_north, wanted);
	return std::none_of(missing.begin(), missing.end(), [](bool b) { return b; });
}

bool known_absent(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted)
{
	const auto absent = absent_blocks(root, sheet, tile_east, tile_north);
	if (!absent)
		return false;
	for (std::size_t i = 0; i < std::min(wanted.size(), absent->size()); ++i)
		if (wanted[i] && !(*absent)[i])
			return false;
	return true;
}

void mark_absent(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &blocks)
{
	if (std::none_of(blocks.begin(), blocks.end(), [](bool b) { return b; }))
		return;
	const auto directory = root / sheet.cache_dir();
	std::error_code ec;
	std::filesystem::create_directories(directory, ec);
	if (ec)
		return;
	const auto path = marker_path(root, sheet, tile_east, tile_north, ".absent");
	if (sheet.source->laser.locate.kind == source::Locate::Kind::Urls) {
		std::ofstream(path, std::ios::binary | std::ios::trunc);
		return;
	}
	auto mask = absent_blocks(root, sheet, tile_east, tile_north)
						.value_or(BlockMask(BLOCK_COUNT, false));
	for (std::size_t i = 0; i < std::min(mask.size(), blocks.size()); ++i)
		mask[i] = mask[i] || blocks[i];
	std::ofstream file(path, std::ios::binary | std::ios::trunc);
	for (bool bit : mask)
		file.put(bit ? 1 : 0);
}

bool noted_whole(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north)
{
	std::error_code ec;
	const bool exists = std::filesystem::exists(
			marker_path(root, sheet, tile_east, tile_north, ".whole"), ec);
	return exists && !ec;
}

void note_whole(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, bool whole)
{
	const auto path = marker_path(root, sheet, tile_east, tile_north, ".whole");
	std::error_code ec;
	if (!whole) {
		std::filesystem::remove(path, ec);
		return;
	}
	std::filesystem::create_directories(path.parent_path(), ec);
	if (!ec)
		std::ofstream(path, std::ios::binary | std::ios::trunc);
}

std::optional<std::uint64_t> noted_size(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north)
{
	std::ifstream file(marker_path(root, sheet, tile_east, tile_north, ".size"));
	std::uint64_t bytes = 0;
	if (!(file >> bytes))
		return std::nullopt;
	file >> std::ws;
	return file.eof() ? std::optional<std::uint64_t>(bytes) : std::nullopt;
}

void note_size(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, std::uint64_t bytes)
{
	const auto path = marker_path(root, sheet, tile_east, tile_north, ".size");
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	if (!ec)
		std::ofstream(path, std::ios::trunc) << bytes;
}

bool due_recheck(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north)
{
	const auto path = cache_path(root, sheet, tile_east, tile_north);
	const auto raw = decompress_head(path);
	if (!raw || raw->size() < FIXED_HEAD)
		return false;
	const auto offset = 8 + 1 + BLOCK_COUNT;
	const auto origin_size =
			std::uint16_t((*raw)[offset]) | (std::uint16_t((*raw)[offset + 1]) << 8);
	if (raw->size() < FIXED_HEAD + origin_size)
		return false;
	const std::string origin(
			raw->begin() + FIXED_HEAD, raw->begin() + FIXED_HEAD + origin_size);
	return is_zip(origin) && !fresh(path, std::chrono::hours(24 * 30));
}

bool is_zip(const std::string &url)
{
	return url.ends_with(".zip");
}
} // namespace arnis::survey::laser_cache
