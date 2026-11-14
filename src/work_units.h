#pragma once

#include "coordinate_system/cartesian/xzbbox/rectangle.h"
#include "coordinate_system/geographic/llbbox.h"
#include "fm_ll_shapes.h"
#include "fm_xz_bbox_shape.h"
#include "projection/web_mercator.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::work_units
{

inline constexpr std::int32_t REGION_BLOCKS = 512;
inline constexpr std::int32_t SLIVER_BLOCKS = REGION_BLOCKS / 4;

struct WorkUnit
{
	WorkUnit(std::uint32_t index, std::int32_t gx, std::int32_t gz, std::int32_t size,
			cartesian::XZBBoxRect rect, geographic::LLBBox bbox, std::uint64_t regions);

	// Position in the deterministic centre-out build order.
	std::uint32_t index = 0;
	// Lattice cell and cell size, measured in map regions.
	std::int32_t gx = 0;
	std::int32_t gz = 0;
	std::int32_t size = 1;
	cartesian::XZBBoxRect rect;
	geographic::LLBBox bbox;
	std::uint64_t regions = 0;

	std::uint64_t chunks() const;
	std::string region_key() const;
};

// Cut a chunk-aligned world into fixed region-lattice cells, ordered centre first.
// With merge_slivers enabled, edge strips narrower than 128 blocks join their neighbour.
bool plan_units(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &world, std::int32_t regions_per_side,
		std::vector<WorkUnit> &units, std::string *error = nullptr,
		bool merge_slivers = false);

class Mask
{
	std::shared_ptr<const XZBBoxShape> shape_;
	std::uint64_t hash_ = 0;
	Mask(std::shared_ptr<const XZBBoxShape> shape, std::uint64_t hash) :
			shape_(std::move(shape)), hash_(hash)
	{
	}

public:
	static std::optional<Mask> load(const std::filesystem::path &path,
			const projection::WebMercatorProjection &projection,
			const cartesian::XZBBoxRect &rect, std::string *error = nullptr);
	bool touches(const cartesian::XZBBoxRect &rect) const;
	bool covers(const cartesian::XZBBoxRect &rect) const;
	std::uint64_t hash() const { return hash_; }
};

enum class SnapRegions
{
	None,
	Cover,
	Fit,
	Merge,
};

enum class PieceOrderMode
{
	Centre,
	West,
	East,
	North,
	South,
	List,
};

struct PieceOrder
{
	PieceOrderMode mode = PieceOrderMode::Centre;
	// One-based piece numbers, matching the Rust --piece-order argument.
	std::vector<std::uint32_t> list;
};

struct PlannerOptions
{
	// Set for fixed-sized units; absent chooses an adaptive cut using auto_max_regions.
	std::optional<std::int32_t> fixed_regions;
	std::int32_t auto_max_regions = 8;
	// Rust scale::MAX_PIECES, using FIRST_MAP_ID = 1,000,000 and 65,536 ids per piece.
	std::size_t max_pieces =
			(static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()) -
					1'000'000) /
			(1U << 16);
	std::int32_t margin_regions = 0;
	SnapRegions snap_regions = SnapRegions::None;
	std::optional<std::filesystem::path> mask_path;
};

struct Plan
{
	Plan(cartesian::XZBBoxRect requested, cartesian::XZBBoxRect rect,
			std::vector<WorkUnit> units, std::string name) :
			requested(std::move(requested)), rect(std::move(rect)),
			units(std::move(units)), name(std::move(name))
	{
	}
	cartesian::XZBBoxRect requested;
	cartesian::XZBBoxRect rect;
	std::vector<WorkUnit> units;
	std::string name;
};

bool build_order(const std::vector<WorkUnit> &units, const PieceOrder &order,
		std::vector<std::size_t> &indices, std::string *error = nullptr);

std::optional<Plan> plan_selection(const projection::WebMercatorProjection &projection,
		const geographic::LLBBox &selection, const PlannerOptions &options,
		std::string *error = nullptr);

// Recursively split partially covered max-size lattice cells; cells wholly inside the
// selection and mask stay whole, while clipped/masked cells split down to min_regions.
bool plan_auto_units(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &world, std::int32_t max_regions,
		std::int32_t min_regions, const Mask *mask, std::vector<WorkUnit> &units,
		std::string *error = nullptr);

// Whole lattice cells just outside the current rectangle, filtered by projection stretch.
std::vector<geographic::LLBBox> room(const projection::WebMercatorProjection &projection,
		const cartesian::XZBBoxRect &rect, std::int32_t regions_per_side,
		double max_stretch, std::size_t cap);

} // namespace arnis::work_units
