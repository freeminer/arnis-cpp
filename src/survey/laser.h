#pragma once

#include "copc.h"
#include "crs.h"
#include "ept.h"
#include "source.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace arnis::survey::laser
{
inline constexpr std::size_t TILE = 1000;
inline constexpr std::size_t BLOCK = 100;
inline constexpr std::size_t BLOCKS = TILE / BLOCK;
inline constexpr double PAD_M = 30.0;
inline constexpr double TARGET_DENSITY = 7.0;

struct Tile
{
	std::vector<std::uint8_t> n_all, n_gnd, n_obj, n_bld, n_veg, h_obj, h_bld;
	std::vector<std::uint16_t> i_gnd;
	std::vector<bool> blocks;
	std::string origin;

	static Tile empty(const std::string &origin = {});
	bool valid() const;
	bool has(const std::vector<bool> &wanted) const;
	bool write(const std::filesystem::path &path) const;
	static std::optional<Tile> read(const std::filesystem::path &path);
	void take_blocks(const Tile &other, const std::vector<bool> &fill);
};

// Reduce LAS point records into one-metre survey statistics for a tile window.
// Call add() once per complete point record, then finish() after all source
// files contributing to the window have been visited.
class Measure
{
	struct Impl;
	std::unique_ptr<Impl> impl_;

public:
	Measure(const source::Sheet &source, std::int32_t tile_east, std::int32_t tile_north,
			std::array<std::size_t, 4> window = {0, 0, TILE, TILE});
	~Measure();
	Measure(Measure &&) noexcept;
	Measure &operator=(Measure &&) noexcept;
	Measure(const Measure &) = delete;
	Measure &operator=(const Measure &) = delete;

	void set_to_grid(std::optional<crs::Affine> transform);
	void add(const copc::PointLayout &layout, std::span<const std::uint8_t> record);
	Tile finish(const std::string &origin);
};

// Read an EPT URL (or newline-separated set of EPT URLs), measure points in
// the requested 100 m blocks plus the ground-interpolation margin, and merge
// those blocks into `target`.
ept::ReadResult read_ept(const source::Sheet &source, std::int32_t tile_east,
		std::int32_t tile_north, const std::vector<bool> &wanted, const std::string &urls,
		Tile &target);

// Read an uncompressed LAS file in bounded record batches. LAZ records return
// an Unusable failure until a native LASzip-compatible decoder is available.
fetch::Result<Tile> measure_las_file(const std::filesystem::path &path,
		const source::Sheet &source, std::int32_t tile_east, std::int32_t tile_north,
		const std::string &origin);

// Return cached measurements or acquire and measure a source tile. Whole LAS,
// deflate/stored ZIPs containing ordinary LAS, and EPT are supported here;
// compressed COPC/LAZ still needs a LASzip-compatible point decoder.
fetch::Result<std::optional<Tile>> fetch_tile(const std::filesystem::path &root,
		const source::Sheet &source, std::int32_t tile_east, std::int32_t tile_north,
		const std::vector<bool> &wanted, const std::vector<std::string> &urls);
} // namespace arnis::survey::laser
