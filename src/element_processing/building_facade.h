#pragma once

#include "../floodfill_cache.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arnis::building_facade
{
inline constexpr std::size_t MIN_FACADE_FOOTPRINT = 12;

struct PointHash
{
	std::size_t operator()(const std::pair<int, int> &point) const noexcept;
};
using PointSet = std::unordered_set<std::pair<int, int>, PointHash>;

struct ColumnFacade
{
	bool party{false};
	bool street{true};
};

struct BuildingContext
{
	const FloodFillCache &flood_fill_cache;
	const CoordinateBitmap &building_passages;
	const CoordinateBitmap &road_mask;
	const CoordinateBitmap &building_footprints;
	const std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> &group_members;
};

enum class FacadeClass
{
	Street,
	Rear,
	Party,
	Open
};

struct SegmentFacade
{
	FacadeClass facade_class{FacadeClass::Open};
	std::optional<int> road_dist;
	std::pair<int, int> normal;
	std::pair<int, int> tangent;
	int len{0};
};

struct CornerPlan
{
	std::pair<int, int> vertex;
	std::size_t seg_a{0};
	std::size_t seg_b{0};
};

struct FacadeAnchor
{
	int x{0};
	int z{0};
	std::pair<int, int> normal;
	int fascia_y{0};
	int number_y{0};
	std::optional<std::pair<int, int>> door;
};

class FacadePlan
{
	PointSet party_columns_;
	PointSet street_columns_;

public:
	std::vector<std::optional<SegmentFacade>> segments;
	PointSet door_columns;
	std::optional<std::size_t> front_segment;
	std::optional<CornerPlan> corner;
	bool has_any_street{false};

	static FacadePlan empty();
	bool is_party(int x, int z) const;
	bool is_street(int x, int z) const;
	bool is_door(int x, int z) const;
	void mark_door_column(int x, int z);
	void add_party_column(int x, int z);
	void add_street_column(int x, int z);
};

FacadePlan compute_facade_plan(const ProcessedWay &element,
		const BuildingContext &context, double scale, const PointSet &own_cells);
}
