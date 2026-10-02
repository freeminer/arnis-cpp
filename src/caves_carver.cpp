#include "caves_carver.h"
#include "caves_biomes.h"
#include "caves_rng.h"
#include "args.h"
#include "block_definitions.h"
#include "world_editor/floor_state.h"
#include "../../arnis_world_editor.h"
#include <array>
#include <cmath>
#include <optional>
#include <tuple>
#include <vector>

namespace arnis::caves
{
namespace
{
constexpr int CAVE_REACH_CHUNKS = 8;

std::uint64_t hash3(std::int64_t seed, int x, int y, int z)
{
	std::uint64_t h = std::uint64_t(seed) ^ 0x9e3779b97f4a7c15ULL;
	for (std::int64_t v : {std::int64_t(x), std::int64_t(y), std::int64_t(z)}) {
		h ^= std::uint64_t(v) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
		h *= 0xbf58476d1ce4e5b9ULL;
		h ^= h >> 27;
	}
	return h;
}
double noise(std::int64_t seed, double x, double y, double z)
{
	const int ix = int(std::floor(x)), iy = int(std::floor(y)), iz = int(std::floor(z));
	const double fx = x - ix, fy = y - iy, fz = z - iz;
	auto sample = [&](int dx, int dy, int dz) {
		return double(hash3(seed, ix + dx, iy + dy, iz + dz) >> 11) / double(1ULL << 53) *
					   2.0 -
			   1.0;
	};
	double out = 0;
	for (int dz = 0; dz <= 1; ++dz)
		for (int dy = 0; dy <= 1; ++dy)
			for (int dx = 0; dx <= 1; ++dx) {
				const double wx = dx ? fx : 1.0 - fx, wy = dy ? fy : 1.0 - fy,
							 wz = dz ? fz : 1.0 - fz;
				out += wx * wy * wz * sample(dx, dy, dz);
			}
	return out;
}

void carve_ellipsoid(world_editor::WorldEditor &editor, const CaveRect &region, double ox,
		double oy, double oz, double radius, double vertical, double floor_level,
		int floor_y)
{
	const int min_x = std::max(region.min_x, int(std::floor(ox - radius)));
	const int max_x = std::min(region.max_x, int(std::floor(ox + radius)));
	const int min_z = std::max(region.min_z, int(std::floor(oz - radius)));
	const int max_z = std::min(region.max_z, int(std::floor(oz + radius)));
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_y =
			std::max({int(std::floor(oy - vertical)) - 1, floor_y + 1, write_min_y});
	const int max_y = std::min(int(std::floor(oy + vertical)) + 1, write_max_y);
	if (min_y > max_y || min_x > max_x || min_z > max_z)
		return;
	const std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, DIRT, GRAVEL};
	const auto host_options = std::optional<std::vector<Block>>(hosts);
	for (int x = min_x; x <= max_x; ++x) {
		const double dx = (double(x) + .5 - ox) / radius;
		if (dx * dx >= 1.0)
			continue;
		for (int z = min_z; z <= max_z; ++z) {
			const double dz = (double(z) + .5 - oz) / radius;
			if (dx * dx + dz * dz >= 1.0)
				continue;
			const int roof = editor.get_ground_level(x, z) - 6;
			for (int y = min_y; y <= max_y; ++y) {
				if (y > roof)
					continue;
				const double dy = (double(y) - .5 - oy) / vertical;
				if (dy > floor_level && dx * dx + dy * dy + dz * dz < 1.0 &&
						editor.check_for_block_absolute(x, y, z, host_options))
					editor.set_block_absolute(AIR, x, y, z, host_options, std::nullopt);
			}
		}
	}
}

constexpr int BRANCH_BUDGET = 112;
constexpr int VANILLA_FLOOR = -64;

bool can_reach(
		int cx, int cz, double x, double z, int step, int branch_count, double width)
{
	const double dx = x - (cx * 16 + 8), dz = z - (cz * 16 + 8);
	const double remaining = branch_count - step;
	const double reach = width + 18.0;
	return dx * dx + dz * dz - remaining * remaining <= reach * reach;
}

double get_thickness(XoroRandom &r)
{
	double f = double(r.next_float()) * 2.0 + double(r.next_float());
	if (r.next_int(10) == 0)
		f *= double(r.next_float()) * double(r.next_float()) * 3.0 + 1.0;
	return std::min(f, 3.0);
}

void create_tunnel(std::int64_t tunnel_seed, double x, double y, double z, double h_mult,
		double v_mult, double thickness, double yaw, double pitch, int branch_index,
		int branch_count, double horizontal_vertical_ratio, double floor_level, int cx,
		int cz, int floor_y, world_editor::WorldEditor &editor, const CaveRect &region)
{
	XoroRandom r = XoroRandom::from_seed(tunnel_seed);
	const int branch_point = r.next_int(branch_count / 2) + branch_count / 4;
	const bool steep = r.next_int(6) == 0;
	double yaw_delta = 0.0, pitch_delta = 0.0;
	for (int step = branch_index; step < branch_count; ++step) {
		const double d = 1.5 + std::sin(M_PI * step / branch_count) * thickness;
		const double d1 = d * horizontal_vertical_ratio;
		const double cos_pitch = std::cos(pitch);
		x += std::cos(yaw) * cos_pitch;
		y += std::sin(pitch);
		z += std::sin(yaw) * cos_pitch;
		pitch *= steep ? .92 : .7;
		pitch += pitch_delta * .1;
		yaw += yaw_delta * .1;
		pitch_delta *= .9;
		yaw_delta *= .75;
		pitch_delta += (double(r.next_float()) - double(r.next_float())) *
					   double(r.next_float()) * 2.0;
		yaw_delta += (double(r.next_float()) - double(r.next_float())) *
					 double(r.next_float()) * 4.0;
		if (step == branch_point && thickness > 1.0) {
			const double t1 = double(r.next_float()) * .5 + .5;
			const auto s1 = r.next_long();
			const double t2 = double(r.next_float()) * .5 + .5;
			const auto s2 = r.next_long();
			create_tunnel(s1, x, y, z, h_mult, v_mult, t1, yaw - M_PI / 2.0, pitch / 3.0,
					step, branch_count, 1.0, floor_level, cx, cz, floor_y, editor,
					region);
			create_tunnel(s2, x, y, z, h_mult, v_mult, t2, yaw + M_PI / 2.0, pitch / 3.0,
					step, branch_count, 1.0, floor_level, cx, cz, floor_y, editor,
					region);
			return;
		}
		if (r.next_int(4) != 0) {
			if (!can_reach(cx, cz, x, z, step, branch_count, thickness))
				return;
			carve_ellipsoid(editor, region, x, y, z, d * h_mult, d1 * v_mult, floor_level,
					floor_y);
		}
	}
}

void carve_random_walk_carvers(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y)
{
	const auto chunk_floor = [](int value) {
		return value >= 0 ? value / 16 : -(((-value) + 15) / 16);
	};
	const int shift = floor_y - VANILLA_FLOOR;
	for (int cx = chunk_floor(region.min_x) - CAVE_REACH_CHUNKS;
			cx <= chunk_floor(region.max_x) + CAVE_REACH_CHUNKS; ++cx) {
		for (int cz = chunk_floor(region.min_z) - CAVE_REACH_CHUNKS;
				cz <= chunk_floor(region.max_z) + CAVE_REACH_CHUNKS; ++cz) {
			for (const auto &cfg : {std::tuple<std::int64_t, float, int, int>{
											0x0CA50CA5LL, .075f, -56, 180},
						 {0xE547E547LL, .025f, -56, 47}}) {
				const auto [salt, probability, y_min, y_max] = cfg;
				XoroRandom r =
						XoroRandom::from_seed(seed ^ std::int64_t(cx) * 341873128712LL ^
											  std::int64_t(cz) * 132897987541LL ^ salt);
				if (r.next_float() > probability)
					continue;
				const int a = r.next_int(15) + 1;
				const int b = r.next_int(a) + 1;
				const int count = r.next_int(b);
				for (int i = 0; i < count; ++i) {
					const int ox = cx * 16 + r.next_int(16);
					const int oy = y_min + r.next_int(y_max - y_min + 1);
					const int oz = cz * 16 + r.next_int(16);
					const double h_mult = .7 + double(r.next_float()) * .7;
					const double v_mult = .8 + double(r.next_float()) * .5;
					const double floor_level = -1.0 + double(r.next_float()) * .6;
					int tunnels = 1;
					if (r.next_int(4) == 0) {
						const double y_scale = .1 + double(r.next_float()) * .8;
						const double room_radius = 1.0 + double(r.next_float()) * 2.0;
						const double d = 1.5 + room_radius;
						carve_ellipsoid(editor, region, ox + 1.0, oy + shift, oz, d,
								d * y_scale, floor_level, floor_y);
						tunnels += r.next_int(4);
					}
					for (int tunnel = 0; tunnel < tunnels; ++tunnel) {
						const double yaw = double(r.next_float()) * 2.0 * M_PI;
						const double pitch = (double(r.next_float()) - .5) / 4.0;
						const double thickness = get_thickness(r);
						const int branch_count =
								BRANCH_BUDGET - r.next_int(BRANCH_BUDGET / 4);
						const auto tunnel_seed = r.next_long();
						create_tunnel(tunnel_seed, ox, oy + shift, oz, h_mult, v_mult,
								thickness, yaw, pitch, 0, branch_count, 1.0, floor_level,
								cx, cz, floor_y, editor, region);
					}
				}
			}

			// Vanilla's canyon carver uses a separate origin salt and walk shape.
			XoroRandom r = XoroRandom::from_seed(
					seed ^ std::int64_t(cx) * 341873128712LL ^
					std::int64_t(cz) * 132897987541LL ^ 0x4A1E4A1ELL);
			if (r.next_float() <= .008f) {
				double x = cx * 16 + r.next_int(16);
				double y = -54 + r.next_int(64) + shift;
				double z = cz * 16 + r.next_int(16);
				const double h_mult = .7 + double(r.next_float()) * .7;
				const double v_mult = .8 + double(r.next_float()) * .5;
				const double thickness =
						std::min(2.0 * (double(r.next_float()) * 2.0 +
											   double(r.next_float()) + 1.0),
								5.0);
				double yaw = double(r.next_float()) * 2.0 * M_PI;
				double pitch = (double(r.next_float()) - .5) / 8.0;
				const int branch_count = BRANCH_BUDGET - r.next_int(BRANCH_BUDGET / 4);
				XoroRandom turn = XoroRandom::from_seed(r.next_long());
				std::array<double, BRANCH_BUDGET> widths{};
				double width = 1.0;
				for (int i = 0; i < branch_count; ++i) {
					if (i == 0 || turn.next_int(3) == 0)
						width = 1.0 +
								double(turn.next_float()) * double(turn.next_float());
					widths[i] = std::min(width * width, 1.8);
				}
				const double floor_level = -1.0 + double(r.next_float()) * .6;
				double yaw_delta = 0.0, pitch_delta = 0.0;
				for (int step = 0; step < branch_count; ++step) {
					const double half =
							1.5 + std::sin(M_PI * step / branch_count) * thickness;
					const double cos_pitch = std::cos(pitch);
					x += std::cos(yaw) * cos_pitch;
					y += std::sin(pitch);
					z += std::sin(yaw) * cos_pitch;
					pitch = pitch * .7 + pitch_delta * .05;
					yaw += yaw_delta * .05;
					pitch_delta *= .8;
					yaw_delta *= .5;
					pitch_delta +=
							(double(turn.next_float()) - double(turn.next_float())) *
							double(turn.next_float()) * 2.0;
					yaw_delta += (double(turn.next_float()) - double(turn.next_float())) *
								 double(turn.next_float()) * 4.0;
					if (turn.next_int(4) != 0) {
						if (!can_reach(cx, cz, x, z, step, branch_count, thickness))
							break;
						carve_ellipsoid(editor, region, x, y, z,
								half * h_mult * std::sqrt(widths[step]),
								half * v_mult * 2.2, floor_level, floor_y);
					}
				}
			}
		}
	}
}
}
void carve_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y)
{
	// Keep this in sync with the Rust cave host predicate: all natural
	// underground rock variants are eligible, while authored blocks remain
	// protected by the whitelist.
	const std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, DIRT, COARSE_DIRT, GRAVEL};
	const std::optional<std::vector<Block>> host_options{hosts};
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	for (int z = region.min_z; z <= region.max_z; ++z)
		for (int x = region.min_x; x <= region.max_x; ++x) {
			const int surface = editor.get_ground_level(x, z);
			const int top = surface - 8;
			if (top <= floor_y + 12)
				continue;
			for (int y = std::max(floor_y + 8, write_min_y);
					y <= std::min(top, write_max_y); ++y) {
				const double n = noise(seed, x * .055, y * .075, z * .055);
				const double band =
						std::abs(noise(seed ^ 0x5deece66dLL, x * .11, y * .025, z * .11));
				// The two fields form mostly horizontal, connected tunnels;
				// the depth taper keeps the surface and bedrock intact.
				const double depth = double(y - floor_y) / std::max(1, top - floor_y);
				if (n < -.72 && band < .58 && depth > .08 && depth < .94 &&
						editor.check_for_block_absolute(x, y, z, host_options))
					editor.set_block_absolute(AIR, x, y, z, std::nullopt, std::nullopt);
			}
		}
	// Random-walk carvers are origin-chunk features; use Rust's branching cave,
	// extra-underground cave, and canyon walks with the same deterministic salts.
	carve_random_walk_carvers(editor, region, seed, floor_y);
}

void decorate_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args)
{
	BiomeAmounts amounts = BiomeAmounts::defaults();
	if (args.cave_biomes)
		amounts = BiomeAmounts::parse(*args.cave_biomes);
	const std::vector<Block> rocks{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, GRAVEL, DIRT};
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const std::optional<std::vector<Block>> rock_options{rocks};
	// Candidates can write one node above or below their own height.
	// Keep decoration independent of traversal order: each candidate is keyed
	// by its absolute column and height, matching the Rust cave decoration pass.
	for (int z = region.min_z; z <= region.max_z; ++z)
		for (int x = region.min_x; x <= region.max_x; ++x) {
			const int top = editor.get_ground_level(x, z) - 8;
			for (int y = std::max(floor_y + 9, write_min_y - 1);
					y < std::min(top, write_max_y + 2); ++y) {
				if (!editor.check_for_block_type_absolute(x, y, z, AIR))
					continue;
				const auto h = hash3(seed ^ 0x6a09e667f3bcc909LL, x, y, z);
				const bool adjacent_rock =
						editor.check_for_block_absolute(x + 1, y, z, rock_options) ||
						editor.check_for_block_absolute(x - 1, y, z, rock_options) ||
						editor.check_for_block_absolute(x, y, z + 1, rock_options) ||
						editor.check_for_block_absolute(x, y, z - 1, rock_options);
				if (!adjacent_rock)
					continue;
				const bool floor_rock =
						editor.check_for_block_absolute(x, y - 1, z, rock_options);
				const auto theme = h % 1000;
				if (theme < 7 && amounts.lush > 0 && floor_rock)
					editor.set_block_absolute(MOSS_BLOCK, x, y - 1, z,
							std::optional<std::vector<Block>>(
									std::vector<Block>{STONE, DEEPSLATE, TUFF, DIRT}),
							std::nullopt);
				else if (theme >= 40 && theme < 44 && amounts.volcanic > 0 && floor_rock)
					editor.set_block_absolute(
							MAGMA_BLOCK, x, y - 1, z, rock_options, std::nullopt);
				else if (theme >= 44 && theme < 48 && amounts.deepdark > 0 && floor_rock)
					editor.set_block_absolute(
							BLACKSTONE, x, y - 1, z, rock_options, std::nullopt);
				else if (theme >= 48 && theme < 51 && amounts.ice > 0 && floor_rock &&
						 y > top - 20)
					editor.set_block_absolute(
							ICE, x, y - 1, z, rock_options, std::nullopt);
				else if (theme == 777 && amounts.mushroom > 0 && y + 1 < top)
					editor.set_block_absolute(BROWN_MUSHROOM, x, y + 1, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
			}
		}
}
}
