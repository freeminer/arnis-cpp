#pragma once

#include "source.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace arnis::survey::laser_cache
{
inline constexpr std::size_t TILE_METRES = 1000;
inline constexpr std::size_t BLOCK_METRES = 100;
inline constexpr std::size_t BLOCKS_PER_TILE = TILE_METRES / BLOCK_METRES;
inline constexpr std::size_t BLOCK_COUNT = BLOCKS_PER_TILE * BLOCKS_PER_TILE;

using BlockMask = std::vector<bool>;

// The 100 m cells in a 1 km tile touched by the requested projected rectangle.
BlockMask blocks_for(
		std::int32_t tile_east, std::int32_t tile_north, const source::Rect &rect);
std::filesystem::path cache_path(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north);

// Returns the cache's measured-block bitmap only when its magic and target
// density match the Rust laser cache format.
std::optional<BlockMask> cached_blocks(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north);
BlockMask lacking_blocks(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted);
double missing_km2(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted);
bool is_cached(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted);
bool known_absent(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &wanted);
void mark_absent(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, const BlockMask &blocks);
bool noted_whole(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north);
void note_whole(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, bool whole);
std::optional<std::uint64_t> noted_size(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t tile_east, std::int32_t tile_north);
void note_size(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north, std::uint64_t bytes);
bool due_recheck(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t tile_east, std::int32_t tile_north);
bool is_zip(const std::string &url);
} // namespace arnis::survey::laser_cache
