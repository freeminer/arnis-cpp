#pragma once

#include "args.h"
#include "../../arnis_block.h"
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <utility>
#include <unordered_map>
#include <vector>

namespace arnis
{
struct Ground;
class ProcessedElement;
namespace water_depth
{
class BigWaterField;
}
namespace river_bed
{
using RiverBed = ::arnis::RiverBed;

inline constexpr int TILE_SHIFT = 9;
inline constexpr int TILE_SIDE = 1 << TILE_SHIFT;
inline constexpr std::uint8_t NO_RIVER = 255;

class RiverBedField
{
	std::unordered_map<std::int64_t, std::vector<std::uint8_t>> tiles_;
	bool has_bounds_ = false;
	int min_x_ = 0, min_z_ = 0, max_x_ = -1, max_z_ = -1;

public:
	bool empty() const { return tiles_.empty(); }
	bool has_bounds() const { return has_bounds_; }
	int min_x() const { return min_x_; }
	int min_z() const { return min_z_; }
	int max_x() const { return max_x_; }
	int max_z() const { return max_z_; }
	std::optional<int> depth_override(int x, int z) const;
	void set(int x, int z, std::uint8_t depth);
};

RiverBedField compute_river_bed_field(RiverBed mode,
		const std::vector<ProcessedElement> &elements, const Ground &ground,
		const water_depth::BigWaterField &legacy, int world_origin_x, int world_origin_z,
		int min_x, int max_x, int min_z, int max_z, double scale,
		std::optional<int> channel_width_cap = std::nullopt);

std::pair<Block, Block> bed_blocks(int x, int z, int depth, bool sandy);
}
}
