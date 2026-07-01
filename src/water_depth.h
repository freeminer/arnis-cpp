#pragma once

#include <cstdint>
#include <vector>

#include "../../arnis_adapter.h"
#include "floodfill_cache.h"

namespace arnis::water_depth
{

class BigWaterField {
public:
	BigWaterField() = default;

	bool empty() const { return depth_.empty() || width_ == 0 || height_ == 0; }
	int depth_at(int x, int z) const;

	int min_x() const { return min_x_; }
	int min_z() const { return min_z_; }
	int max_x() const { return min_x_ + static_cast<int>(width_) - 1; }
	int max_z() const { return min_z_ + static_cast<int>(height_) - 1; }

private:
	friend BigWaterField compute_big_water_field(WorldEditor &editor, const XZBBox &xzbbox);

	std::vector<std::uint8_t> depth_;
	std::size_t width_ = 0;
	std::size_t height_ = 0;
	int min_x_ = 0;
	int min_z_ = 0;
};

BigWaterField compute_big_water_field(WorldEditor &editor, const XZBBox &xzbbox);

void carve_water_column(WorldEditor &editor, int x, int z, int water_y, int depth,
		const RoadMaskBitmap &road_mask);

void carve_lc_water_pass(WorldEditor &editor, const BigWaterField &bwf,
		const RoadMaskBitmap &road_mask);

}
