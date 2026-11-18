#pragma once

#include "classify.h"
#include "source.h"

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <filesystem>
#include <optional>
#include <vector>

namespace arnis::survey::photo
{
inline constexpr std::size_t TILE = 1000;
inline constexpr std::size_t PIXELS_PER_METRE = 5;
inline constexpr std::size_t PIXELS_PER_BLOCK = 256;
inline constexpr std::size_t BLOCKS_ACROSS =
		(TILE * PIXELS_PER_METRE + PIXELS_PER_BLOCK - 1) / PIXELS_PER_BLOCK;

using PhotoCell = classify::PhotoCell;

struct Tile
{
	std::vector<PhotoCell> cells;
	// Which source-image blocks are available, row-major.
	std::vector<bool> blocks_in;
	std::size_t blocks_across = 0;

	Tile() = default;
	explicit Tile(std::size_t across);
	bool valid() const;
};

std::filesystem::path cache_path(const std::filesystem::path &root,
		const source::Sheet &sheet, std::int32_t east, std::int32_t north);
std::optional<Tile> read_cache(const std::filesystem::path &path);
bool write_cache(const std::filesystem::path &path, const Tile &tile);
std::vector<std::size_t> needed_blocks(
		const Tile &tile, std::size_t c0, std::size_t r0, std::size_t c1, std::size_t r1);

// Fold a decoded RGB source block into the 1 m photo cells represented by it.
// `absent` is optional per-pixel metadata used by multi-sheet rendering.
void fill_cells(Tile &tile, std::size_t block_index, const std::uint8_t *pixels,
		std::size_t pixel_stride, const std::vector<bool> *absent = nullptr);
std::optional<Tile> tile(const std::filesystem::path &root, const source::Sheet &sheet,
		std::int32_t east, std::int32_t north, std::size_t c0, std::size_t r0,
		std::size_t c1, std::size_t r1, std::atomic_bool &down);
} // namespace arnis::survey::photo
