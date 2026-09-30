#pragma once

#include "block_definitions.h"
#include "land_cover/land_cover.h"
#include "biome.h"
#include "../../arnis_world_editor.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace arnis::terrain_surface
{

struct Strata
{
	int x{0};
	int z{0};
	double warp{0};
	static Strata at(int x, int z);
	Block block(int y) const;
};

void fill_strata(::arnis::WorldEditor &editor, int x, int z, int y_min, int y_max,
		bool streaks = false);
std::pair<Block, Block> steep_palette(
		int x, int z, int ground_y, int slope, uint8_t cover);
std::pair<Block, Block> bare_rock_palette(int x, int z);

enum class Snow
{
	None,
	Layer,
	Block
};
// Convert partial snow cover to the Java snow-layer depth (one through eight
// eighths).  The C++ selector currently stores the cover class, so keep a
// deterministic one-layer dusting while exposing the Rust-compatible helper
// for property-aware backends.
inline unsigned snow_eighths(Snow snow, double terrain_rise = 0.0)
{
	if (snow == Snow::None)
		return 0;
	if (snow == Snow::Block)
		return 8;
	return static_cast<unsigned>(std::clamp(1.0 + terrain_rise * 2.0, 1.0, 7.0));
}
struct SnowLine
{
	int threshold_y{std::numeric_limits<int>::max()};
	double band_blocks{4};
	SnowLine() = default;
	SnowLine(int threshold, double blocks_per_meter);
	double depth(int x, int z, int y) const;
};
bool is_plausible_ice(double depth, biome::Climate climate);
Snow snow_cover(double depth, double slope, double convexity, int x, int z);
double glacier_depth(double depth);
inline const std::pair<Block, Block> GLACIER_ICE{PACKED_ICE, PACKED_ICE};
void place_snow_layer(
		::arnis::WorldEditor &editor, int x, int ground_y, int z, unsigned eighths = 1);
inline bool is_glacier_cover(uint8_t cover)
{
	return cover == land_cover::LC_SNOW_ICE;
}
inline bool is_ice(const Block &block)
{
	return block == ICE || block == PACKED_ICE;
}

}
