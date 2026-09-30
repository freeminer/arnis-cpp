#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace arnis::geo_grid
{
class TiledGrid
{
	const std::vector<std::uint8_t> *data_ = nullptr;
	std::size_t cell_bytes_ = 0, tile_ = 0, cols_ = 0, rows_ = 0, frames_at_ = 0;
	double cell_deg_ = 0;
	std::uint32_t u32(std::size_t) const;
	std::size_t offset(std::size_t) const;
	TiledGrid() = default;

public:
	static std::optional<TiledGrid> parse(const std::vector<std::uint8_t> &data);
	std::size_t tile_count() const;
	std::size_t columns() const { return cols_; }
	std::size_t rows() const { return rows_; }
	std::size_t cells_per_tile() const { return tile_ * tile_; }
	double cells_per_degree() const { return 1.0 / cell_deg_; }
	std::pair<double, double> position(double lat, double lon) const;
	std::pair<std::size_t, std::size_t> cell(double col, double row) const;
	std::size_t tile_of(std::pair<std::size_t, std::size_t>) const;
	std::size_t index_in_tile(std::pair<std::size_t, std::size_t>) const;
	std::optional<std::vector<std::uint8_t>> decode(std::size_t tile) const;
	std::uint16_t value(const std::vector<std::uint8_t> &, std::size_t index) const;
};
}
