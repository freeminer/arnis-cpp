#include "terrain_surface.h"

#include "climate.h"
#include "ground_generation.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace arnis::terrain_surface
{
namespace
{
constexpr uint32_t STRATA_WARP = 0x5157A7A1, LEDGE = 0x1ED6E5A1, SCREE = 0x5C4EE0B2,
				   WORN = 0x0B0A4E11, BARE_ROCK = 0xBA4E40C3, SNOW_LINE = 0x5A0E11A4,
				   SNOW_DRIFT = 0xD41F7B05, SNOW_FIELD = 0xF1E1D5A0,
				   STRATA_WAVE = 0x5157A7A2, BED_EDGE = 0x5157A7A4, BED_LENS = 0x5157A7A5,
				   SCREE_EDGE_SALT = 0x5C4EED6E, STREAK = 0x57EAC0DE,
				   STREAK_RUN = 0x57EA0F11, SHADE_X = 0x05ADE0A1, SHADE_Z = 0x05ADE0B2;
constexpr int BED_FLOOR = -2200, BED_CEIL = 4200;
constexpr std::array<int, 10> BED_THICKNESS{2, 3, 3, 4, 4, 5, 5, 6, 7, 9};
constexpr int TOP_RAMP_MAX = 3;
constexpr int TONE_STEP = 4;

const std::array<Block, 6> &face_ramp()
{
	static const std::array<Block, 6> blocks{
			ANDESITE, STONE, COBBLESTONE, TUFF, COBBLED_DEEPSLATE, DEEPSLATE};
	return blocks;
}

double value_noise_salted(int x, int z, int scale, uint32_t salt)
{
	constexpr double COS = 0.891006524188368, SIN = 0.453990499739547;
	const double s = std::max(scale, 1);
	const double fx = x, fz = z;
	const double u = (fx * COS - fz * SIN) / s;
	const double v = (fx * SIN + fz * COS) / s;
	const double u0 = std::floor(u), v0 = std::floor(v);
	const double tu = u - u0, tv = v - v0;
	const double su = tu * tu * (3.0 - 2.0 * tu);
	const double sv = tv * tv * (3.0 - 2.0 * tv);
	const auto cu = static_cast<int>(u0), cv = static_cast<int>(v0);
	const auto salt_x = static_cast<int32_t>(salt);
	const auto salt_z = static_cast<int32_t>((salt << 16) | (salt >> 16));
	const auto sample = [&](int cx, int cz) {
		return double(land_cover::coord_hash(cx ^ salt_x, cz ^ salt_z) % 1000) / 1000.0;
	};
	const double a = sample(cu, cv) * (1.0 - su) + sample(cu + 1, cv) * su;
	const double b = sample(cu, cv + 1) * (1.0 - su) + sample(cu + 1, cv + 1) * su;
	return a * (1.0 - sv) + b * sv;
}

const std::vector<int> &bed_starts()
{
	static const std::vector<int> starts = [] {
		std::vector<int> out;
		int y = BED_FLOOR;
		while (y < BED_CEIL) {
			out.push_back(y);
			const auto k = static_cast<int>(out.size());
			y += BED_THICKNESS[land_cover::coord_hash(k, 0x0BED) % BED_THICKNESS.size()];
		}
		return out;
	}();
	return starts;
}

int stratum_layer(int x, int z, double warp, int y, std::size_t &cursor)
{
	const auto &starts = bed_starts();
	const double w = y + warp;
	if (cursor == 0) {
		const auto it = std::upper_bound(starts.begin(), starts.end(), w,
				[](double value, int start) { return value < start; });
		cursor = std::max<std::size_t>(1, std::distance(starts.begin(), it));
	}
	while (cursor < starts.size() && starts[cursor] <= w)
		++cursor;
	const auto i = std::min(cursor, starts.size() - 1);
	const double start = starts[i - 1];
	const double end =
			i < starts.size() ? starts[i] : std::numeric_limits<double>::infinity();
	const int bed = static_cast<int>(i) - 1;
	if (w - start >= .5 && end - w > .5)
		return bed;
	const double edge = value_noise_salted(2 * x + 3 * z, y, 4, BED_EDGE) - .5;
	const double jittered = w + edge;
	if (jittered < start)
		return std::max(0, bed - 1);
	if (jittered >= end)
		return bed + 1;
	return bed;
}

double noise(int x, int z, int scale, uint32_t salt);

Block bed_kind(int x, int z, int layer)
{
	const auto h = land_cover::coord_hash(layer, 0x57A7) % 100;
	if (h < 70)
		return STONE;
	const Block kind = h < 90 ? ANDESITE : TUFF;
	return noise(x + layer * 97, z - layer * 61, 24, BED_LENS) < .7 ? kind : STONE;
}

int ramp_step(Block kind, double tone, int depth)
{
	const int base = kind == ANDESITE ? 0 : kind == TUFF ? 3 : 1;
	tone += .09 * std::min(depth / 32.0, 1.0);
	const int shift = tone < .328 ? -1 : tone < .682 ? 0 : tone < .763 ? 1 : 2;
	return std::clamp(base + shift, 0, static_cast<int>(face_ramp().size()) - 1);
}
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
			(value_noise_salted(x, z, 48, STRATA_WARP) - .5) * 7.0 +
					(value_noise_salted(x, z, 13, STRATA_WAVE) - .5) * 2.5};
}
Block Strata::block(int y) const
{
	std::size_t cursor = 0;
	return bed_kind(x, z, stratum_layer(x, z, warp, y, cursor));
}
void fill_strata(WorldEditor &editor, int x, int z, int lo, int hi, bool streaks)
{
	if (lo > hi)
		return;
	const auto s = Strata::at(x, z);
	const auto streak_noise = streaks ? noise(x, z, 3, STREAK) : 1.0;
	const Block streak_block = streak_noise < .08	? DEEPSLATE
							   : streak_noise < .22 ? TUFF
													: Block{};
	const bool has_streak = streaks && streak_noise < .22;
	int start = lo;
	std::size_t cursor = 0;
	int layer = stratum_layer(x, z, s.warp, lo, cursor);
	Block kind = bed_kind(x, z, layer);
	auto sample_tone = [&](int y) {
		return value_noise_salted(2 * x, y, 12, SHADE_X) * .5 +
			   value_noise_salted(2 * z, y, 12, SHADE_Z) * .5;
	};
	int tone_y = lo;
	double tone_a = sample_tone(tone_y);
	double tone_b = sample_tone(tone_y + TONE_STEP);
	auto block_at = [&](int y) {
		while (y >= tone_y + TONE_STEP) {
			tone_y += TONE_STEP;
			tone_a = tone_b;
			tone_b = sample_tone(tone_y + TONE_STEP);
		}
		const int current_layer = stratum_layer(x, z, s.warp, y, cursor);
		if (current_layer != layer) {
			layer = current_layer;
			kind = bed_kind(x, z, layer);
		}
		const auto streak_x = std::bit_cast<std::int32_t>(
				static_cast<std::uint32_t>(x) * 31u ^ static_cast<std::uint32_t>(z));
		const bool streak_run =
				has_streak && value_noise_salted(streak_x, y, 10, STREAK_RUN) < .7;
		if (streak_run)
			return streak_block;
		const double tone = tone_a + (tone_b - tone_a) * double(y - tone_y) / TONE_STEP;
		return face_ramp()[ramp_step(kind, tone, hi + 1 - y)];
	};
	Block old = block_at(lo);
	for (int y = lo + 1; y <= hi; ++y) {
		const Block b = block_at(y);
		if (b != old) {
			editor.fill_column_absolute(old, x, z, start, y - 1, true);
			start = y;
			old = b;
		}
	}
	editor.fill_column_absolute(old, x, z, start, hi, true);
}

std::optional<TalusField> TalusField::build(
		const Ground &ground, int chunk_x, int chunk_z, int origin_x, int origin_z)
{
	const auto at = [&](int x, int z) {
		return ground.level_exact({x - origin_x, z - origin_z});
	};
	const double correction = ground.elevation_slope_correction;
	double lo = std::numeric_limits<double>::max();
	double hi = std::numeric_limits<double>::lowest();
	for (int i = 0; i < 5; ++i)
		for (int j = 0; j < 5; ++j) {
			const double h =
					at((chunk_x << 4) - 16 + i * 12, (chunk_z << 4) - 16 + j * 12);
			lo = std::min(lo, h);
			hi = std::max(hi, h);
		}
	if ((hi - lo) * correction < 6.0)
		return std::nullopt;
	TalusField field;
	field.x0 = (chunk_x << 4) - 20;
	field.z0 = (chunk_z << 4) - 20;
	field.correction = correction;
	field.highest = std::numeric_limits<double>::lowest();
	for (std::size_t k = 0; k < field.heights.size(); ++k) {
		const int i = static_cast<int>(k % SIDE);
		const int j = static_cast<int>(k / SIDE);
		auto &height = field.heights[k];
		height = at(field.x0 + i * STEP, field.z0 + j * STEP);
		field.highest = std::max(field.highest, height);
	}
	return field;
}

double TalusField::near(int x, int z, double here) const
{
	constexpr std::array<std::pair<int, double>, 3> reaches{
			{{4, 1.0}, {8, .6}, {16, .3}}};
	constexpr double cliff_tan = 1.2;
	if ((highest - here) * correction < cliff_tan * reaches.front().first)
		return 0.0;
	const auto cell = [](int value, int origin) {
		return static_cast<std::size_t>(std::clamp(
				static_cast<int>(std::floor(double(value - origin + STEP / 2) / STEP)), 0,
				static_cast<int>(SIDE) - 1));
	};
	for (const auto [reach, score] : reaches)
		for (const auto [dx, dz] :
				std::array<std::pair<int, int>, 4>{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
			const double wall =
					heights[cell(z + dz * reach, z0) * SIDE + cell(x + dx * reach, x0)];
			if ((wall - here) * correction >= cliff_tan * reach)
				return score;
		}
	return 0.0;
}

const std::vector<Block> &talus_buries()
{
	static const std::vector<Block> blocks{
			GRASS_BLOCK, DIRT, COARSE_DIRT, PODZOL, MOSS_BLOCK};
	return blocks;
}

bool takes_talus(std::uint8_t cover)
{
	return cover == 0 || cover == land_cover::LC_TREE_COVER ||
		   cover == land_cover::LC_SHRUBLAND || cover == land_cover::LC_GRASSLAND ||
		   cover == land_cover::LC_MOSS || cover == land_cover::LC_BARE;
}

std::optional<std::pair<Block, Block>> talus_palette(
		int x, int z, double near, std::uint8_t cover)
{
	if (near <= 0.0 || !takes_talus(cover) || noise(x, z, 7, 0x07A105CE) >= .8 * near)
		return std::nullopt;
	const double rock = noise(x, z, 4, 0x07A10B0D);
	if (rock < .55)
		return std::pair{GRAVEL, STONE};
	if (rock < .75)
		return std::pair{ANDESITE, STONE};
	if (rock < .9)
		return std::pair{COBBLESTONE, STONE};
	return std::pair{STONE, STONE};
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
					   : std::pair{
								 face_ramp()[std::min(TOP_RAMP_MAX,
										 ramp_step(Strata::at(x, z).block(ground_y),
												 value_noise_salted(
														 2 * x, ground_y, 12, SHADE_X) *
																 .5 +
														 value_noise_salted(2 * z,
																 ground_y, 12, SHADE_Z) *
																 .5,
												 0))],
								 STONE};
	const double rock =
			.7 * noise(x, z, 14, SCREE) + .3 * noise(x, z, 5, SCREE_EDGE_SALT);
	if (vegetated(cover) && rock < .53)
		return noise(x, z, 6, WORN) < .08 ? std::pair{COARSE_DIRT, DIRT}
										  : std::pair{GRASS_BLOCK, DIRT};
	if (!vegetated(cover) && rock < .33)
		return {GRAVEL, STONE};
	return bare_rock_palette(x, z);
}
SnowLine::SnowLine(int threshold, double bpm, double latitude, double rotation) :
		threshold_y(threshold), band_blocks(std::max(200.0 * bpm, 4.0))
{
	const double radians = rotation * (std::acos(-1.0) / 180.0);
	const double reach = std::clamp((std::abs(latitude) - 10.0) / 15.0, 0.0, 1.0) *
						 (latitude < 0			? -1.0
								 : latitude > 0 ? 1.0
												: 0.0);
	pole_x = std::sin(radians) * reach;
	pole_z = -std::cos(radians) * reach;
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
double SnowLine::shade(double gradient_x, double gradient_z, double slope) const
{
	if ((pole_x == 0.0 && pole_z == 0.0) || slope <= 0.0)
		return 0.0;
	const double rise = std::hypot(gradient_x, gradient_z);
	if (rise < 1e-9)
		return 0.0;
	const double weight =
			std::min(slope / 4.0, 1.0) - std::clamp((slope - 8.0) / 4.0, 0.0, 0.75);
	return -(gradient_x * pole_x + gradient_z * pole_z) / rise * weight;
}
bool is_plausible_ice(double d, biome::Climate c)
{
	return d > -3.0 || c == biome::Climate::Tundra || c == biome::Climate::IceCap ||
		   c == biome::Climate::Boreal;
}
Snow snow_cover(double d, double slope, const std::function<double()> &convexity,
		double shade, int x, int z)
{
	if (d < -1.5)
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
	const double shade_term = .5 * std::clamp(shade, -1.0, 1.0);
	const double base = d + st + shade_term + drift;
	constexpr double full_cover = .6, hollow_max = .5;
	const double score = base + hollow_max < 0.0 || base - hollow_max >= full_cover
								 ? base
								 : d + st + .25 * std::clamp(convexity(), -2.0, 2.0) +
										   shade_term + drift;
	if (score >= full_cover)
		return Snow::Block;
	if (score < 0.0)
		return Snow::None;
	const unsigned layers = 1 + static_cast<unsigned>(score / full_cover * 7.0);
	return static_cast<Snow>(std::clamp(layers, 1u, 7u));
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
