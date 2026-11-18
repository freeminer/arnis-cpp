#pragma once

#include "../floodfill_cache.h"
#include "../osm_parser.h"

#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arnis::building_inset
{

inline constexpr int MIN_INSET_SPAN = 5;
inline constexpr int MIN_SHRUNK_RINGS = 5;
inline constexpr double MIN_INSET_SCALE = 0.75;

struct CellHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		const auto key = (std::uint64_t(static_cast<std::uint32_t>(p.first)) << 32) |
						 std::uint32_t(p.second);
		return std::hash<std::uint64_t>{}(key);
	}
};

using CellSet = std::unordered_set<std::pair<int, int>, CellHash>;

struct OutlineInset
{
	std::vector<ProcessedNode> nodes;
	std::vector<std::pair<int, int>> fill;
	CellSet strip;
};

std::vector<std::pair<int, int>> ring_cells(const std::vector<ProcessedNode> &nodes);
std::vector<bool> free_edges(const std::vector<ProcessedNode> &nodes, const CellSet &own,
		const CoordinateBitmap &footprints);
std::optional<OutlineInset> inset_outline(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const CoordinateBitmap &footprints);
std::optional<std::vector<bool>> step_plan(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const CoordinateBitmap &footprints,
		const std::vector<bool> &off);
std::optional<OutlineInset> inset_planned(const std::vector<ProcessedNode> &nodes,
		const std::vector<std::pair<int, int>> &fill, const std::vector<bool> &step);

} // namespace arnis::building_inset
