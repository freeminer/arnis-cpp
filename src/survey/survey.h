#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "../grid_ops.h"
#include "photo.h"

namespace arnis::survey
{

enum class Kind : std::uint8_t
{
	None = 0,
	Lawn = 1,
	Paved = 2,
	Hedge = 3,
	Bush = 4,
	Tree = 5,
	Shed = 6,
	Pool = 7,
	Building = 8,
	Bed = 9,
	Fence = 10,
	Bare = 11,
};

struct Features
{
	bool garden = false;
	bool trees = false;
	bool heights = false;
	bool roofs = false;
};

struct Cell
{
	Kind kind = Kind::None;
	float height = 0.0f;
	std::array<std::uint8_t, 3> color{};
};

// Quantized survey data sampled onto the same world grid as canopy data.
// Each cell stores class, object height in fifths of a metre, and RGB photo color.
class SurveyData
{
	std::vector<std::array<std::uint8_t, 5>> cells_;

public:
	std::size_t width = 0;
	std::size_t height = 0;
	Features features;

	SurveyData() = default;
	SurveyData(std::vector<std::array<std::uint8_t, 5>> cells, std::size_t w,
			std::size_t h, Features enabled = {}) :
			cells_(std::move(cells)), width(w), height(h), features(enabled)
	{
		if (w == 0 || h == 0 || cells_.size() != w * h) {
			cells_.clear();
			width = height = 0;
		}
	}

	static SurveyData from_cells(std::vector<std::array<std::uint8_t, 5>> cells,
			std::size_t w, std::size_t h, Features enabled = {})
	{
		return SurveyData(std::move(cells), w, h, enabled);
	}

	bool empty() const { return cells_.empty() || width == 0 || height == 0; }

	std::array<std::uint8_t, 5> raw_at(std::size_t x, std::size_t z) const
	{
		if (empty())
			return {};
		return cells_[std::min(z, height - 1) * width + std::min(x, width - 1)];
	}

	void remap_rows_to_mercator(double latitude_top, double latitude_bottom)
	{
		const auto rows = height;
		grid_ops::remap_flat_rows_nearest(cells_, width, [=](std::size_t z) {
			return grid_ops::mercator_source_row(latitude_top, latitude_bottom, rows, z);
		});
	}

	void crop(
			std::size_t x0, std::size_t z0, std::size_t new_width, std::size_t new_height)
	{
		grid_ops::crop_flat(cells_, width, x0, z0, new_width, new_height);
		const auto rows = new_width ? cells_.size() / new_width : 0;
		width = rows ? std::min(new_width, width > x0 ? width - x0 : 0) : 0;
		height = rows;
	}

	std::array<std::size_t, 12> counts() const
	{
		std::array<std::size_t, 12> result{};
		for (const auto &cell : cells_)
			++result[std::min<std::size_t>(cell[0], result.size() - 1)];
		return result;
	}

	std::optional<Cell> at_any(std::size_t x, std::size_t z) const
	{
		if (empty() || x >= width || z >= height)
			return std::nullopt;
		const auto &raw = cells_[z * width + x];
		if (raw[0] < static_cast<std::uint8_t>(Kind::Lawn) ||
				raw[0] > static_cast<std::uint8_t>(Kind::Bare))
			return std::nullopt;
		return Cell{static_cast<Kind>(raw[0]), static_cast<float>(raw[1]) / 5.0f,
				{raw[2], raw[3], raw[4]}};
	}

	void regrid(
			std::vector<std::array<std::uint8_t, 5>> cells, std::size_t w, std::size_t h)
	{
		if (w == 0 || h == 0 || cells.size() != w * h) {
			cells_.clear();
			width = height = 0;
			return;
		}
		cells_ = std::move(cells);
		width = w;
		height = h;
	}
};

} // namespace arnis::survey

#include "plan.h"
#include "copc.h"
#include "ept.h"
