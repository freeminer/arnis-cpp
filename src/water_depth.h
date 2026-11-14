#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <utility>

#include "../../arnis_adapter.h"
#include "floodfill_cache.h"
#include "args.h"
#include "river_bed.h"

namespace arnis::bridges
{
class BridgeSurfaceMap;
}

namespace arnis::water_detail
{
struct CarveDepth;
}

namespace arnis::water_depth
{

// The elevation fetch computes bore reserves before the world editor exists.
// Publish the current run's carve policy so bore crossings reserve enough room
// for the water and river-bed passes that will run later.
void set_crossing_carve(water_detail::CarveDepth carve);

inline constexpr int BORE_ROOM = 8;
inline constexpr int MAX_WATER_DEPTH = 6;
// Keep every block that can obstruct an underwater bore in one shared set,
// matching Rust's water_depth::BORE_SHELL. Water carving uses this to leave
// the channel bed on top of the shell when there is not enough room below it.
const std::array<Block, 4> &bore_shell();
struct BoreCrossing
{
	int depth{};
	double lowest_m{};
};

// In-place 3-4 chamfer distance transform, saturating at u8::MAX. Shared with
// elevation postprocessing for the bank-distance raster, matching Rust's
// crate-visible helper.
void chamfer_3_4_dt(
		std::vector<std::uint8_t> &distances, std::size_t width, std::size_t height);

// Convert an inclusive grid-cell interval into a conservative inclusive block
// interval. This mirrors Rust's mapping helper and is used when carving only
// the water sub-region instead of scanning the complete world rectangle.
std::pair<int, int> grid_span_to_block_span(std::size_t grid_lo, std::size_t grid_hi,
		std::size_t world_dim, std::size_t grid_dim);

// Clear a connected tree while restricting traversal to selected map columns.
// The predicate is evaluated before reading or mutating the block.
void clear_tree_within(WorldEditor &editor, int x, int y, int z,
		const std::function<bool(int, int)> &keep_column);
void clear_tree_from(WorldEditor &editor, int x, int y, int z);

class BigWaterField
{
public:
	BigWaterField() = default;

	bool empty() const
	{
		return (depth_.empty() || width_ == 0 || height_ == 0) && river_.empty();
	}
	bool reaches(int x, int z, int radius) const;
	int depth_at(int x, int z) const;
	int body_max_7x7(int x, int z) const;
	bool deeper_than_crust() const { return deeper_than_crust_; }

	int min_x() const
	{
		return river_.has_bounds() && (depth_.empty() || width_ == 0) ? river_.min_x()
			   : river_.has_bounds() ? std::min(min_x_, river_.min_x())
									 : min_x_;
	}
	int min_z() const
	{
		return river_.has_bounds() && (depth_.empty() || height_ == 0) ? river_.min_z()
			   : river_.has_bounds() ? std::min(min_z_, river_.min_z())
									 : min_z_;
	}
	int max_x() const
	{
		const int legacy = min_x_ + static_cast<int>(width_) - 1;
		return river_.has_bounds() && (depth_.empty() || width_ == 0) ? river_.max_x()
			   : river_.has_bounds() ? std::max(legacy, river_.max_x())
									 : legacy;
	}
	int max_z() const
	{
		const int legacy = min_z_ + static_cast<int>(height_) - 1;
		return river_.has_bounds() && (depth_.empty() || height_ == 0) ? river_.max_z()
			   : river_.has_bounds() ? std::max(legacy, river_.max_z())
									 : legacy;
	}
	int legacy_depth_at(int x, int z) const;
	bool is_river(int x, int z) const { return river_.depth_override(x, z).has_value(); }
	void set_river_bed(river_bed::RiverBedField field)
	{
		deeper_than_crust_ = deeper_than_crust_ || !field.empty();
		river_ = std::move(field);
	}

private:
	friend BigWaterField compute_big_water_field(
			WorldEditor &editor, const XZBBox &xzbbox);
	friend BigWaterField compute_big_water_field(
			WorldEditor &editor, const XZBBox &xzbbox, WaterDetail detail, double scale);

	std::vector<std::uint8_t> depth_;
	std::size_t width_ = 0;
	std::size_t height_ = 0;
	int min_x_ = 0;
	int min_z_ = 0;
	river_bed::RiverBedField river_;
	bool deeper_than_crust_ = false;
};

BigWaterField compute_big_water_field(WorldEditor &editor, const XZBBox &xzbbox);
BigWaterField compute_big_water_field(
		WorldEditor &editor, const XZBBox &xzbbox, WaterDetail detail, double scale);
std::optional<int> lc_carved_bed(
		const WorldEditor &editor, const BigWaterField &field, int x, int z);
std::pair<Block, Block> shallow_bed(const WorldEditor &editor, int x, int z);
int estimate_max_carve_depth(
		const std::vector<std::vector<std::uint8_t>> &land_cover_grid,
		std::size_t world_width, std::size_t world_height);
std::vector<BoreCrossing> bore_crossings(
		const std::vector<std::vector<std::uint8_t>> &land_cover_grid,
		std::size_t world_width, std::size_t world_height,
		const std::vector<std::pair<std::size_t, std::size_t>> &cells,
		const std::function<double(std::size_t, std::size_t)> &height_m);

void carve_water_column(WorldEditor &editor, int x, int z, int water_y, int depth,
		const RoadMaskBitmap &road_mask, const BigWaterField &bwf);
bool at_bore(const BigWaterField &field, const RoadMaskBitmap &footprint, int x, int z);

void carve_lc_water_pass(
		WorldEditor &editor, const BigWaterField &bwf, const RoadMaskBitmap &road_mask);
void carve_lc_water_pass(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint);
void carve_lc_water_pass(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint,
		const bridges::BridgeSurfaceMap *bridge_surface);
void carve_lc_water_region(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint,
		int min_x, int max_x, int min_z, int max_z);
void carve_lc_water_region(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap &tunnel_footprint,
		int min_x, int max_x, int min_z, int max_z,
		const bridges::BridgeSurfaceMap *bridge_surface);

// Shared model-placement helpers: remove a connected tree and classify tree
// blocks without exposing the water-carving implementation details.
bool is_tree_block(const std::optional<std::string> &name);
void clear_tree(WorldEditor &editor, int x, int y, int z);

}
