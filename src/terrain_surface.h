#pragma once

#include "block_definitions.h"
#include "land_cover/land_cover.h"
#include "biome.h"
#include "../../arnis_world_editor.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

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

struct TalusField
{
	static constexpr int STEP = 4;
	static constexpr std::size_t SIDE = 15;
	int x0{0}, z0{0};
	std::array<double, SIDE * SIDE> heights{};
	double highest{0}, correction{1};
	static std::optional<TalusField> build(const ::arnis::Ground &, int chunk_x,
			int chunk_z, int origin_x, int origin_z);
	double nearr(int x, int z, double here) const;
};

const std::vector<Block> &talus_buries();
bool takes_talus(std::uint8_t cover);
std::optional<std::pair<Block, Block>> talus_palette(
		int x, int z, double nearr, std::uint8_t cover);
std::pair<Block, Block> steep_palette(
		int x, int z, int ground_y, int slope, uint8_t cover);
std::pair<Block, Block> bare_rock_palette(int x, int z);

enum class Snow
{
	None,
	Layer,
	Layer2,
	Layer3,
	Layer4,
	Layer5,
	Layer6,
	Layer7,
	Block
};
// Convert partial snow cover to its Java snow-layer depth (one through seven
// eighths), following the terrain's unrounded height within the top block.
inline unsigned snow_eighths(Snow snow, double terrain_rise = 0.0)
{
	if (snow == Snow::None)
		return 0;
	const unsigned smooth =
			terrain_rise >= 0.0 && terrain_rise < 1.0
					? static_cast<unsigned>(std::min(std::round(terrain_rise * 8.0), 7.0))
					: 0;
	if (snow == Snow::Block)
		return smooth;
	const unsigned max_layers =
			static_cast<unsigned>(snow) - static_cast<unsigned>(Snow::Layer) + 1;
	return std::clamp(smooth, 1u, std::max(max_layers, 1u));
}
struct SnowLine
{
	int threshold_y{std::numeric_limits<int>::max()};
	double band_blocks{4};
	double pole_x{0}, pole_z{0};
	SnowLine() = default;
	SnowLine(int threshold, double blocks_per_meter, double latitude = 0,
			double rotation = 0);
	double depth(int x, int z, int y) const;
	double shade(double gradient_x, double gradient_z, double slope) const;
};
bool is_plausible_ice(double depth, biome::Climate climate);
Snow snow_cover(double depth, double slope, const std::function<double()> &convexity,
		double shade, int x, int z);
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
	return block == ICE || block == PACKED_ICE || block == BLUE_ICE;
}

}
