#include "terrain_surface.h"

#include "climate.h"
#include "ground_generation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis::terrain_surface
{
namespace
{
constexpr uint32_t STRATA_WARP = 0x5157A7A1, LEDGE = 0x1ED6E5A1, SCREE = 0x5C4EE0B2,
				   SCREE_EDGE = 0x5C4ED66E, WORN = 0x0B0A4E11, BARE_ROCK = 0xBA4E40C3,
				   SNOW_LINE = 0x5A0E11A4, SNOW_DRIFT = 0xD41F7B05,
				   SNOW_FIELD = 0xF1E1D5A0, STRATA_WAVE = 0x5157A7B2,
				   BED_LENS = 0xB3D1E5A7;
double noise(int x, int z, int scale, uint32_t salt)
{
	return ground_generation::patch_noise(x, z, scale, salt);
}
bool vegetated(uint8_t c)
{
	return c == 0 || c == land_cover::LC_TREE_COVER || c == land_cover::LC_SHRUBLAND ||
		   c == land_cover::LC_GRASSLAND || c == land_cover::LC_CROPLAND ||
		   c == land_cover::LC_MOSS;
}
}

Strata Strata::at(int x, int z)
{
	return {x, z,
			(noise(x, z, 48, STRATA_WARP) - .5) * 7.0 +
					(noise(x, z, 13, STRATA_WAVE) - .5) * 2.5};
}
Block Strata::block(int y) const
{
	const int layer = int(std::floor((double(y) + warp) / 4.0));
	const auto h = land_cover::coord_hash(layer, 0x57A7) % 100;
	const Block kind = h < 70 ? STONE : h < 90 ? ANDESITE : TUFF;
	// Rust's beds contain long horizontal lenses rather than a single material
	// selected independently at every column.  Keep the layer choice stable and
	// use a low-frequency patch field to break up the non-stone beds.
	if (kind != STONE && noise(x + layer * 97, z - layer * 61, 24, BED_LENS) >= .7)
		return STONE;
	return kind;
}
void fill_strata(WorldEditor &editor, int x, int z, int lo, int hi, bool streaks)
{
	if (lo > hi)
		return;
	const auto s = Strata::at(x, z);
	int start = lo;
	Block old = s.block(lo);
	for (int y = lo + 1; y <= hi; ++y) {
		Block b = s.block(y);
		if (streaks) {
			const auto streak = noise(x * 31 ^ z, y, 10, 0x57EA0A01);
			if (streak < .08)
				b = DEEPSLATE;
			else if (streak < .22)
				b = TUFF;
		}
		if (b != old) {
			editor.fill_column_absolute(old, x, z, start, y - 1, true);
			start = y;
			old = b;
		}
	}
	editor.fill_column_absolute(old, x, z, start, hi, true);
}
std::pair<Block, Block> bare_rock_palette(int x, int z)
{
	const auto n = noise(x, z, 9, BARE_ROCK);
	return n < .15	 ? std::pair{GRAVEL, STONE}
		   : n < .35 ? std::pair{ANDESITE, STONE}
					 : std::pair{STONE, STONE};
}
std::pair<Block, Block> steep_palette(
		int x, int z, int ground_y, int slope, uint8_t cover)
{
	if (slope > 6)
		return slope <= 8 && noise(x, z, 10, LEDGE) < .1
					   ? std::pair{GRAVEL, STONE}
					   : std::pair{Strata::at(x, z).block(ground_y), STONE};
	const double rock = .7 * noise(x, z, 14, SCREE) + .3 * noise(x, z, 5, SCREE_EDGE);
	if (vegetated(cover) && rock < .53)
		return noise(x, z, 6, WORN) < .08 ? std::pair{COARSE_DIRT, DIRT}
										  : std::pair{GRASS_BLOCK, DIRT};
	if (!vegetated(cover) && rock < .33)
		return {GRAVEL, STONE};
	return bare_rock_palette(x, z);
}
SnowLine::SnowLine(int threshold, double bpm) :
		threshold_y(threshold), band_blocks(std::max(200.0 * bpm, 4.0))
{
}
double SnowLine::depth(int x, int z, int y) const
{
	if (threshold_y == std::numeric_limits<int>::max())
		return -std::numeric_limits<double>::infinity();
	if (threshold_y == std::numeric_limits<int>::min())
		return 2.5;
	return std::min((double(y - threshold_y) / band_blocks) +
							(noise(x, z, 32, SNOW_LINE) - .5) * .6,
			2.5);
}
bool is_plausible_ice(double d, biome::Climate c)
{
	return d > -3.0 || c == biome::Climate::Tundra || c == biome::Climate::IceCap ||
		   c == biome::Climate::Boreal;
}
Snow snow_cover(double d, double slope, double convexity, int x, int z)
{
	if (d < -1.0)
		return Snow::None;
	const std::array<std::pair<double, double>, 5> slope_terms{
			{{2.0, .1}, {4.0, 0.0}, {6.0, -.7}, {8.0, -1.5}, {10.0, -2.8}}};
	double st = slope_terms.back().second;
	if (slope <= slope_terms.front().first)
		st = slope_terms.front().second;
	else
		for (std::size_t i = 1; i < slope_terms.size(); ++i)
			if (slope <= slope_terms[i].first) {
				const auto [x0, y0] = slope_terms[i - 1];
				const auto [x1, y1] = slope_terms[i];
				const double t = (slope - x0) / (x1 - x0);
				st = y0 + t * (y1 - y0);
				break;
			}
	const double drift = (noise(x, z, 40, SNOW_FIELD) - .5) * .5 +
						 (noise(x, z, 9, SNOW_DRIFT) - .5) * .25;
	const double score = d + st + .25 * std::clamp(convexity, -2.0, 2.0) + drift;
	return score >= .6 ? Snow::Block : score >= 0 ? Snow::Layer : Snow::None;
}
double glacier_depth(double d)
{
	return std::max(d, -.2);
}
void place_snow_layer(WorldEditor &e, int x, int y, int z, unsigned eighths)
{
	if (e.block_exists_absolute(x, y + 1, z))
		return;
	const auto layer = snow_layer_with_depth(eighths);
	e.set_block_with_properties_absolute(layer, x, y + 1, z, std::nullopt, std::nullopt);
	if (e.check_for_block_absolute(
				x, y, z, std::optional<std::vector<Block>>{{GRASS_BLOCK}}))
		e.set_block_absolute(
				GRASS_BLOCK, x, y, z, std::optional<std::vector<Block>>{{GRASS_BLOCK}});
	else if (e.check_for_block_absolute(
					 x, y, z, std::optional<std::vector<Block>>{{PODZOL}}))
		e.set_block_absolute(
				PODZOL, x, y, z, std::optional<std::vector<Block>>{{PODZOL}});
}
}
