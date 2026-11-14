#pragma once

#include "../floodfill_cache.h"

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace arnis::element_processing::area_layout
{

struct Frame
{
	std::pair<double, double> along{};
	std::pair<double, double> across{};
	std::pair<double, double> along_range{};
	std::pair<double, double> across_range{};

	static std::optional<Frame> of(const std::vector<ProcessedNode> &nodes);
	double length() const;
	double width() const;
	std::pair<double, double> local(int32_t x, int32_t z) const;
	std::pair<int32_t, int32_t> column(double a, double c) const;
	std::array<std::pair<double, double>, 2> local_span(
			const std::array<int32_t, 4> &bounds) const;
	std::pair<int32_t, int32_t> along_axis() const;
	std::pair<int32_t, int32_t> across_axis() const;
};

class AreaMask
{
	int32_t min_x_{};
	int32_t min_z_{};
	std::size_t width_{};
	std::size_t height_{};
	std::vector<std::uint64_t> bits_;

public:
	AreaMask() = default;
	AreaMask(const std::vector<std::pair<int32_t, int32_t>> &cells,
			const std::function<bool(int32_t, int32_t)> &keep);
	bool contains(int32_t x, int32_t z) const;
};

using Bounds = std::array<double, 4>; // min x, min z, max x, max z
std::optional<Bounds> bounds(const std::vector<ProcessedNode> &nodes);
bool inside(const Bounds &bounds, const std::pair<double, double> &point);
std::pair<double, double> centroid(const std::vector<ProcessedNode> &nodes);
bool point_in_rings(const std::vector<const std::vector<ProcessedNode> *> &rings,
		const std::pair<double, double> &point);
std::optional<std::pair<double, double>> interior_point(
		const std::vector<ProcessedNode> &nodes);
std::uint64_t mix(std::uint64_t value);
std::uint64_t cell_hash(std::uint64_t id, std::int64_t i, std::int64_t j);

} // namespace arnis::element_processing::area_layout
