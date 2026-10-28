#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>
#include <utility>

#include "../../arnis_adapter.h"
#include "floodfill_cache.h"

namespace arnis::bridges
{
class BridgeSurfaceMap;
}

namespace arnis::water_depth
{

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

	bool empty() const { return depth_.empty() || width_ == 0 || height_ == 0; }
	int depth_at(int x, int z) const;
	int body_max_7x7(int x, int z) const;

	int min_x() const { return min_x_; }
	int min_z() const { return min_z_; }
	int max_x() const { return min_x_ + static_cast<int>(width_) - 1; }
	int max_z() const { return min_z_ + static_cast<int>(height_) - 1; }

private:
	friend BigWaterField compute_big_water_field(
			WorldEditor &editor, const XZBBox &xzbbox);

	std::vector<std::uint8_t> depth_;
	std::size_t width_ = 0;
	std::size_t height_ = 0;
	int min_x_ = 0;
	int min_z_ = 0;
};

BigWaterField compute_big_water_field(WorldEditor &editor, const XZBBox &xzbbox);
std::pair<Block, Block> shallow_bed(const WorldEditor &editor, int x, int z);
int estimate_max_carve_depth(
		const std::vector<std::vector<std::uint8_t>> &land_cover_grid,
		std::size_t world_width, std::size_t world_height);

void carve_water_column(WorldEditor &editor, int x, int z, int water_y, int depth,
		const RoadMaskBitmap &road_mask, const BigWaterField &bwf);

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
