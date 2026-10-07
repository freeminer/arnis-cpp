#include "caves_density.h"
#include "caves_shape.h"
#include "block_definitions.h"
#include "world_editor/floor_state.h"
#include "../../arnis_world_editor.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace arnis::caves
{
namespace
{
constexpr int VANILLA_FLOOR = -64;
constexpr int CELL_WIDTH = 4;
constexpr int CELL_HEIGHT = 8;
constexpr int TOP_GATE = 6;
constexpr double LAYER_SQUEEZE = 7.0;
constexpr double ENTRANCE_SHRINK = .32;
constexpr double TUBE_SHRINK = .05;
constexpr double CHEESE_SHRINK = .38;

struct NoiseSpec
{
	const char *id;
	int first_octave;
	std::vector<double> amplitudes;
};

const std::array<NoiseSpec, 20> NOISES{{
		{"cave_cheese", -8, {.5, 1, 2, 1, 2, 1, 0, 2, 0}},
		{"cave_layer", -8, {1}},
		{"cave_entrance", -7, {.4, .5, 1}},
		{"spaghetti_2d", -7, {1}},
		{"spaghetti_2d_elevation", -8, {1}},
		{"spaghetti_2d_modulator", -11, {1}},
		{"spaghetti_2d_thickness", -11, {1}},
		{"spaghetti_3d_1", -7, {1}},
		{"spaghetti_3d_2", -7, {1}},
		{"spaghetti_3d_rarity", -11, {1}},
		{"spaghetti_3d_thickness", -8, {1}},
		{"spaghetti_roughness", -5, {1}},
		{"spaghetti_roughness_modulator", -8, {1}},
		{"noodle", -8, {1}},
		{"noodle_thickness", -8, {1}},
		{"noodle_ridge_a", -7, {1}},
		{"noodle_ridge_b", -7, {1}},
		{"pillar", -7, {1, 1}},
		{"pillar_rareness", -8, {1}},
		{"pillar_thickness", -8, {1}},
}};

enum NoiseIndex : std::size_t
{
	CAVE_CHEESE,
	CAVE_LAYER,
	CAVE_ENTRANCE,
	SPAGHETTI_2D,
	SPAGHETTI_2D_ELEVATION,
	SPAGHETTI_2D_MODULATOR,
	SPAGHETTI_2D_THICKNESS,
	SPAGHETTI_3D_1,
	SPAGHETTI_3D_2,
	SPAGHETTI_3D_RARITY,
	SPAGHETTI_3D_THICKNESS,
	SPAGHETTI_ROUGHNESS,
	SPAGHETTI_ROUGHNESS_MODULATOR,
	NOODLE,
	NOODLE_THICKNESS,
	NOODLE_RIDGE_A,
	NOODLE_RIDGE_B,
	PILLAR,
	PILLAR_RARENESS,
	PILLAR_THICKNESS,
};

double clamp(double value, double lo, double hi)
{
	return std::clamp(value, lo, hi);
}

double y_clamped_gradient(
		double y, int from_y, int to_y, double from_value, double to_value)
{
	if (y <= from_y)
		return from_value;
	if (y >= to_y)
		return to_value;
	return from_value + (to_value - from_value) * ((y - from_y) / (to_y - from_y));
}

double rarity_2d(double value)
{
	if (value < -.75)
		return .5;
	if (value < -.5)
		return .75;
	if (value < .5)
		return 1.0;
	if (value < .75)
		return 2.0;
	return 3.0;
}

double rarity_3d(double value)
{
	if (value < -.5)
		return .75;
	if (value < 0.0)
		return 1.0;
	if (value < .5)
		return 1.5;
	return 2.0;
}

int floor_div(int value, int divisor)
{
	const int quotient = value / divisor;
	const int remainder = value % divisor;
	return remainder < 0 ? quotient - 1 : quotient;
}

double lerp(double delta, double start, double end)
{
	return start + delta * (end - start);
}
}

CaveGen::CaveGen(std::int64_t seed, int floor_y, int world_max_y) :
		y_shift_(floor_y - VANILLA_FLOOR), world_max_y_(world_max_y)
{
	XoroRandom root = XoroRandom::from_seed(seed);
	const auto factory = root.fork_positional();
	noises_.reserve(NOISES.size());
	for (const auto &spec : NOISES) {
		const std::string name = std::string("minecraft:") + spec.id;
		XoroRandom random = XoroRandom::from_hash_of(factory.first, factory.second, name);
		noises_.push_back(
				NormalNoise::create(random, spec.first_octave, spec.amplitudes));
	}
}

double CaveGen::s2d_thickness_modulator(double x, double y, double z) const
{
	return -.95 - .35000000000000003 *
						  noises_[SPAGHETTI_2D_THICKNESS].get_value(x * 2.0, y, z * 2.0);
}

double CaveGen::spaghetti_roughness(double x, double y, double z) const
{
	const double a =
			-.05 - .05 * noises_[SPAGHETTI_ROUGHNESS_MODULATOR].get_value(x, y, z);
	const double b = -.4 + std::abs(noises_[SPAGHETTI_ROUGHNESS].get_value(x, y, z));
	return a * b;
}

double CaveGen::spaghetti_2d(double x, double y, double z) const
{
	const double mod = noises_[SPAGHETTI_2D_MODULATOR].get_value(x * 2.0, y, z * 2.0);
	const double scale = rarity_2d(mod);
	const double spaghetti = scale * std::abs(noises_[SPAGHETTI_2D].get_value(
											 x / scale, y / scale, z / scale));
	const double thickness = s2d_thickness_modulator(x, y, z);
	const double arg1 = spaghetti + .083 * thickness;
	const double elevation = 8.0 * noises_[SPAGHETTI_2D_ELEVATION].get_value(x, 0.0, z);
	const double y_gradient = y_clamped_gradient(y, -64, 320, 8.0, -40.0);
	const double cube_arg = std::abs(elevation + y_gradient) + thickness;
	const double arg2 = cube_arg * cube_arg * cube_arg;
	return clamp(std::max(arg1, arg2), -1.0, 1.0);
}

double CaveGen::entrances(double x, double y, double z) const
{
	const double depth_bias = y_clamped_gradient(y, -40, 30, 0.0, ENTRANCE_SHRINK);
	const double arg_a = .37 +
						 noises_[CAVE_ENTRANCE].get_value(x * .75, y * 1.1, z * .75) +
						 y_clamped_gradient(y, -10, 30, .3, 0.0) + depth_bias;
	const double rarity = noises_[SPAGHETTI_3D_RARITY].get_value(x * 2.0, y, z * 2.0);
	const double scale = rarity_3d(rarity);
	const double s1 = scale * std::abs(noises_[SPAGHETTI_3D_1].get_value(
									  x / scale, y / scale, z / scale));
	const double s2 = scale * std::abs(noises_[SPAGHETTI_3D_2].get_value(
									  x / scale, y / scale, z / scale));
	const double tube_shrink = y_clamped_gradient(y, -40, 30, 0.0, TUBE_SHRINK);
	const double s3d_thickness =
			-.0765 -
			.011499999999999996 * noises_[SPAGHETTI_3D_THICKNESS].get_value(x, y, z) +
			tube_shrink;
	const double clamped = clamp(std::max(s1, s2) + s3d_thickness, -1.0, 1.0);
	const double arg_b = spaghetti_roughness(x, y, z) + clamped;
	return std::min(arg_a, arg_b);
}

double CaveGen::pillars(double x, double y, double z) const
{
	const double a = 2.0 * noises_[PILLAR].get_value(x * 25.0, y * .3, z * 25.0) - 1.0 -
					 noises_[PILLAR_RARENESS].get_value(x, y, z);
	const double thickness = .55 + .55 * noises_[PILLAR_THICKNESS].get_value(x, y, z);
	return a * thickness * thickness * thickness;
}

double CaveGen::noodle(double x, double y, double z) const
{
	const bool in_range = y >= -60.0 && y < 321.0;
	const double toggle = in_range ? noises_[NOODLE].get_value(x, y, z) : -1.0;
	if (toggle < 0.0)
		return 64.0;
	const double thickness =
			in_range ? -.07500000000000001 -
							   .025 * noises_[NOODLE_THICKNESS].get_value(x, y, z)
					 : 0.0;
	constexpr double scale = 2.6666666666666665;
	const double ridge_a =
			in_range ? noises_[NOODLE_RIDGE_A].get_value(x * scale, y * scale, z * scale)
					 : 0.0;
	const double ridge_b =
			in_range ? noises_[NOODLE_RIDGE_B].get_value(x * scale, y * scale, z * scale)
					 : 0.0;
	return thickness + 1.5 * std::max(std::abs(ridge_a), std::abs(ridge_b));
}

double CaveGen::combined_density(int x, int y, int z) const
{
	const double xf = x, yf = y - y_shift_, zf = z;
	const double entrance = entrances(xf, yf, zf);
	const double layer = noises_[CAVE_LAYER].get_value(xf, yf * 8.0, zf);
	const double cheese_noise =
			clamp(.27 + noises_[CAVE_CHEESE].get_value(xf, yf * .6666666666666666, zf),
					-1.0, 1.0);
	const double depth_k = y_clamped_gradient(yf, -40, 30, 5.0, .5);
	const double cheese_bias = clamp(1.5 - .64 * depth_k, 0.0, .5);
	const double cheese =
			LAYER_SQUEEZE * layer * layer + cheese_noise + cheese_bias + CHEESE_SHRINK;
	const double spaghetti = spaghetti_2d(xf, yf, zf) + spaghetti_roughness(xf, yf, zf);
	const double t2 = std::min(std::min(cheese, entrance), spaghetti);
	const double pillar = pillars(xf, yf, zf);
	const double cave = std::max(t2, pillar < .03 ? -1.0e6 : pillar);
	const double yg1 =
			y_clamped_gradient(yf, VANILLA_FLOOR, VANILLA_FLOOR + 24, 0.0, 1.0);
	const int top_exclusive = world_max_y_ + 1 - y_shift_;
	const double yg2 =
			y_clamped_gradient(yf, top_exclusive, top_exclusive + 16, 1.0, 0.0);
	const double inner =
			.1171875 + yg1 * (-.1171875 + (-.078125 + yg2 * (.078125 + cave)));
	const double value = clamp(.64 * inner, -1.0, 1.0);
	return value / 2.0 - value * value * value / 24.0;
}

double CaveGen::noodle_density(int x, int y, int z) const
{
	return noodle(x, y - y_shift_, z);
}

void carve_density_region(const CaveGen &gen, world_editor::WorldEditor &editor,
		int min_x, int max_x, int min_z, int max_z, int floor_y,
		std::unordered_set<std::int64_t> *carved)
{
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRAVEL, DIRT,
			ANDESITE, GRANITE, DIORITE};
	const std::optional<std::vector<Block>> host_options{hosts};
	const int z_span = max_z - min_z + 1;
	std::vector<int> surface(std::size_t(max_x - min_x + 1) * z_span);
	int max_surface = floor_y;
	for (int x = min_x; x <= max_x; ++x)
		for (int z = min_z; z <= max_z; ++z) {
			const int height = editor.get_ground_level(x, z);
			surface[std::size_t(x - min_x) * z_span + (z - min_z)] = height;
			max_surface = std::max(max_surface, height);
		}
	const int cx0 = floor_div(min_x, CELL_WIDTH), cx1 = floor_div(max_x, CELL_WIDTH);
	const int cz0 = floor_div(min_z, CELL_WIDTH), cz1 = floor_div(max_z, CELL_WIDTH);
	const int cy0 = floor_div(floor_y + 1, CELL_HEIGHT);
	const int cy1 = floor_div(max_surface - TOP_GATE, CELL_HEIGHT);
	for (int cx = cx0; cx <= cx1; ++cx)
		for (int cz = cz0; cz <= cz1; ++cz) {
			const int wx0 = cx * CELL_WIDTH, wx1 = wx0 + CELL_WIDTH;
			const int wz0 = cz * CELL_WIDTH, wz1 = wz0 + CELL_WIDTH;
			double b00 = gen.combined_density(wx0, cy0 * CELL_HEIGHT, wz0);
			double b10 = gen.combined_density(wx1, cy0 * CELL_HEIGHT, wz0);
			double b01 = gen.combined_density(wx0, cy0 * CELL_HEIGHT, wz1);
			double b11 = gen.combined_density(wx1, cy0 * CELL_HEIGHT, wz1);
			for (int cy = cy0; cy <= cy1; ++cy) {
				const int wy0 = cy * CELL_HEIGHT, wy1 = wy0 + CELL_HEIGHT;
				const double n000 = b00, n100 = b10, n001 = b01, n101 = b11;
				const double n010 = gen.combined_density(wx0, wy1, wz0);
				const double n110 = gen.combined_density(wx1, wy1, wz0);
				const double n011 = gen.combined_density(wx0, wy1, wz1);
				const double n111 = gen.combined_density(wx1, wy1, wz1);
				b00 = n010;
				b10 = n110;
				b01 = n011;
				b11 = n111;
				const int by_lo = std::max({wy0, floor_y + 1, write_min_y});
				const int by_hi =
						std::min({wy1 - 1, max_surface - TOP_GATE, write_max_y});
				for (int by = by_lo; by <= by_hi; ++by) {
					const double fy = double(by - wy0) / CELL_HEIGHT;
					const double xz00 = lerp(fy, n000, n010);
					const double xz10 = lerp(fy, n100, n110);
					const double xz01 = lerp(fy, n001, n011);
					const double xz11 = lerp(fy, n101, n111);
					for (int bx = std::max(wx0, min_x); bx <= std::min(wx1 - 1, max_x);
							++bx) {
						const double fx = double(bx - wx0) / CELL_WIDTH;
						const double z0 = lerp(fx, xz00, xz10);
						const double z1 = lerp(fx, xz01, xz11);
						const std::size_t column = std::size_t(bx - min_x) * z_span;
						for (int bz = std::max(wz0, min_z);
								bz <= std::min(wz1 - 1, max_z); ++bz) {
							const int top = surface[column + (bz - min_z)] - TOP_GATE;
							if (by > top)
								continue;
							const double fz = double(bz - wz0) / CELL_WIDTH;
							const double combined = lerp(fz, z0, z1);
							if (combined <= 0.0 ||
									gen.noodle_density(bx, by, bz) <= 0.0) {
								editor.set_block_absolute(
										AIR, bx, by, bz, host_options, std::nullopt);
								if (carved)
									carved->insert(pack_cave_pos(bx, by, bz));
							}
						}
					}
				}
			}
		}
}
}
