#include "geo_grid.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <zstd.h>

namespace arnis::geo_grid
{
namespace
{
constexpr std::size_t HEADER = 24;
}
std::uint32_t TiledGrid::u32(std::size_t p) const
{
	return std::uint32_t((*data_)[p]) | (std::uint32_t((*data_)[p + 1]) << 8) |
		   (std::uint32_t((*data_)[p + 2]) << 16) |
		   (std::uint32_t((*data_)[p + 3]) << 24);
}
std::size_t TiledGrid::offset(std::size_t i) const
{
	return u32(HEADER + 4 * i);
}
std::optional<TiledGrid> TiledGrid::parse(const std::vector<std::uint8_t> &data)
{
	if (data.size() < HEADER || std::memcmp(data.data(), "AGRD", 4) != 0 || data[4] != 1)
		return {};
	const auto u16 = [&](std::size_t p) {
		return std::size_t(data[p]) | (std::size_t(data[p + 1]) << 8);
	};
	const auto u32 = [&](std::size_t p) {
		return std::uint32_t(data[p]) | (std::uint32_t(data[p + 1]) << 8) |
			   (std::uint32_t(data[p + 2]) << 16) | (std::uint32_t(data[p + 3]) << 24);
	};
	const auto cell_bytes = std::size_t(data[5]), tile = u16(6),
			   cols = std::size_t(u32(8)), rows = std::size_t(u32(12));
	double cell_deg = 0;
	std::memcpy(&cell_deg, data.data() + 16, sizeof(cell_deg));
	if ((cell_bytes != 1 && cell_bytes != 2) || !tile || cols % tile || rows % tile ||
			!(cell_deg > 0))
		return {};
	TiledGrid g;
	g.data_ = &data;
	g.cell_bytes_ = cell_bytes;
	g.tile_ = tile;
	g.cols_ = cols;
	g.rows_ = rows;
	g.cell_deg_ = cell_deg;
	g.frames_at_ = HEADER + 4 * (g.tile_count() + 1);
	if (g.frames_at_ > data.size() ||
			g.frames_at_ + g.offset(g.tile_count()) != data.size())
		return {};
	return g;
}
std::size_t TiledGrid::tile_count() const
{
	return (cols_ / tile_) * (rows_ / tile_);
}
std::pair<double, double> TiledGrid::position(double lat, double lon) const
{
	return {(lon + 180.0) / cell_deg_, (90.0 - lat) / cell_deg_};
}
std::pair<std::size_t, std::size_t> TiledGrid::cell(double col, double row) const
{
	auto c =
			static_cast<std::int64_t>(std::floor(col)) % static_cast<std::int64_t>(cols_);
	if (c < 0)
		c += static_cast<std::int64_t>(cols_);
	auto r = std::clamp(static_cast<std::int64_t>(std::floor(row)), std::int64_t(0),
			static_cast<std::int64_t>(rows_ - 1));
	return {static_cast<std::size_t>(c), static_cast<std::size_t>(r)};
}
std::size_t TiledGrid::tile_of(std::pair<std::size_t, std::size_t> p) const
{
	return (p.second / tile_) * (cols_ / tile_) + p.first / tile_;
}
std::size_t TiledGrid::index_in_tile(std::pair<std::size_t, std::size_t> p) const
{
	return (p.second % tile_) * tile_ + p.first % tile_;
}
std::optional<std::vector<std::uint8_t>> TiledGrid::decode(std::size_t tile) const
{
	if (tile >= tile_count())
		return {};
	const auto start = offset(tile), end = offset(tile + 1);
	if (start > end || frames_at_ + end > data_->size() || start == end)
		return {};
	std::vector<std::uint8_t> out(tile_ * tile_ * cell_bytes_);
	const auto n = ZSTD_decompress(
			out.data(), out.size(), data_->data() + frames_at_ + start, end - start);
	if (ZSTD_isError(n) || n != out.size())
		return {};
	return out;
}
std::uint16_t TiledGrid::value(
		const std::vector<std::uint8_t> &tile, std::size_t index) const
{
	if (cell_bytes_ == 1)
		return tile[index];
	return std::uint16_t(tile[2 * index]) | (std::uint16_t(tile[2 * index + 1]) << 8);
}
}
