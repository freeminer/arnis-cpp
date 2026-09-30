#include "caves_carver.h"
#include "caves_biomes.h"
#include "caves_rng.h"
#include "args.h"
#include "block_definitions.h"
#include "world_editor/floor_state.h"
#include "../../arnis_world_editor.h"
#include <cmath>
#include <optional>
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

void carve_ellipsoid(world_editor::WorldEditor &editor, const CaveRect &region, int ox,
		int oy, int oz, double radius, double vertical, double floor_level)
{
	const int min_x = std::max(region.min_x, int(std::floor(ox - radius)));
	const int max_x = std::min(region.max_x, int(std::floor(ox + radius)));
	const int min_z = std::max(region.min_z, int(std::floor(oz - radius)));
	const int max_z = std::min(region.max_z, int(std::floor(oz + radius)));
	const int min_y = std::max(
			oy - int(std::ceil(vertical)) - 1, world_editor::terrain_floor_y() + 1);
	const int max_y = oy + int(std::ceil(vertical)) + 1;
	const std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, DIRT, COARSE_DIRT, GRAVEL};
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
						editor.check_for_block_absolute(x, y, z, hosts))
					editor.set_block_absolute(AIR, x, y, z, host_options, std::nullopt);
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
	for (int z = region.min_z; z <= region.max_z; ++z)
		for (int x = region.min_x; x <= region.max_x; ++x) {
			const int surface = editor.get_ground_level(x, z);
			const int top = surface - 8;
			if (top <= floor_y + 12)
				continue;
			for (int y = floor_y + 8; y <= top; ++y) {
				const double n = noise(seed, x * .055, y * .075, z * .055);
				const double band =
						std::abs(noise(seed ^ 0x5deece66dLL, x * .11, y * .025, z * .11));
				// The two fields form mostly horizontal, connected tunnels;
				// the depth taper keeps the surface and bedrock intact.
				const double depth = double(y - floor_y) / std::max(1, top - floor_y);
				if (n < -.72 && band < .58 && depth > .08 && depth < .94 &&
						editor.check_for_block_absolute(x, y, z, hosts))
					editor.set_block_absolute(AIR, x, y, z, std::nullopt, std::nullopt);
			}
		}
	// Rust's random-walk carvers are chunk-origin features.  Evaluate every
	// origin that can reach this region so streamed tiles agree at their edges.
	const auto chunk_floor = [](int value) {
		return value >= 0 ? value / 16 : -(((-value) + 15) / 16);
	};
	for (int cx = chunk_floor(region.min_x) - CAVE_REACH_CHUNKS;
			cx <= chunk_floor(region.max_x) + CAVE_REACH_CHUNKS; ++cx)
		for (int cz = chunk_floor(region.min_z) - CAVE_REACH_CHUNKS;
				cz <= chunk_floor(region.max_z) + CAVE_REACH_CHUNKS; ++cz) {
			XoroRandom r = XoroRandom::from_seed(
					seed ^ std::int64_t(cx) * 341873128712LL ^
					std::int64_t(cz) * 132897987541LL ^ 0x0CA50CA5LL);
			if (r.next_float() > .075f)
				continue;
			const int a = r.next_int(15) + 1;
			const int b = r.next_int(a) + 1;
			const int count = r.next_int(b);
			for (int i = 0; i < count; ++i) {
				const int ox = cx * 16 + r.next_int(16);
				const int oy = -56 + r.next_int(237);
				const int oz = cz * 16 + r.next_int(16);
				carve_ellipsoid(editor, region, ox, oy, oz, 2.0 + r.next_float() * 5.0,
						1.5 + r.next_float() * 4.0, -.8 + r.next_float() * .4);
			}
		}
	// Rust's cave_extra_underground carver is an independent, lower band.  It
	// must use its own salt and probability so it does not correlate with the
	// primary cave origins.
	for (int cx = chunk_floor(region.min_x) - CAVE_REACH_CHUNKS;
			cx <= chunk_floor(region.max_x) + CAVE_REACH_CHUNKS; ++cx)
		for (int cz = chunk_floor(region.min_z) - CAVE_REACH_CHUNKS;
				cz <= chunk_floor(region.max_z) + CAVE_REACH_CHUNKS; ++cz) {
			XoroRandom r = XoroRandom::from_seed(
					seed ^ std::int64_t(cx) * 341873128712LL ^
					std::int64_t(cz) * 132897987541LL ^ 0xE547E547LL);
			if (r.next_float() > .025f)
				continue;
			const int a = r.next_int(15) + 1;
			const int b = r.next_int(a) + 1;
			const int count = r.next_int(b);
			for (int i = 0; i < count; ++i) {
				const int ox = cx * 16 + r.next_int(16);
				const int oy = -56 + r.next_int(104);
				const int oz = cz * 16 + r.next_int(16);
				carve_ellipsoid(editor, region, ox, oy, oz, 2.0 + r.next_float() * 5.0,
						1.5 + r.next_float() * 4.0, -.8 + r.next_float() * .4);
			}
		}
	// Rare ravines use a long, gently turning branch of vertically stretched
	// ellipsoids.  Keeping the origin/chunk salt stable makes the feature seam
	// safe when neighboring regions are generated independently.
	for (int cx = chunk_floor(region.min_x) - CAVE_REACH_CHUNKS;
			cx <= chunk_floor(region.max_x) + CAVE_REACH_CHUNKS; ++cx)
		for (int cz = chunk_floor(region.min_z) - CAVE_REACH_CHUNKS;
				cz <= chunk_floor(region.max_z) + CAVE_REACH_CHUNKS; ++cz) {
			XoroRandom r = XoroRandom::from_seed(
					seed ^ std::int64_t(cx) * 341873128712LL ^
					std::int64_t(cz) * 132897987541LL ^ 0x4A1E4A1ELL);
			if (r.next_float() > .008f)
				continue;
			double x = cx * 16 + r.next_int(16), y = -54 + r.next_int(64),
				   z = cz * 16 + r.next_int(16);
			const double h_mult = .7 + r.next_float() * .7;
			const double v_mult = .8 + r.next_float() * .5;
			const double thickness =
					std::min(2.0 * (r.next_float() * 2.0 + r.next_float() + 1.0), 5.0);
			double yaw = r.next_float() * 2.0 * M_PI;
			double pitch = (r.next_float() - .5) / 8.0;
			double yaw_delta = 0, pitch_delta = 0;
			const int steps = 112 - r.next_int(28);
			XoroRandom turn = XoroRandom::from_seed(r.next_long());
			const double floor_level = -1.0 + r.next_float() * .6;
			for (int step = 0; step < steps; ++step) {
				const double half = 1.5 + std::sin(M_PI * step / steps) * thickness;
				const double cp = std::cos(pitch);
				x += std::cos(yaw) * cp;
				y += std::sin(pitch);
				z += std::sin(yaw) * cp;
				pitch = pitch * .7 + pitch_delta * .05;
				yaw += yaw_delta * .05;
				pitch_delta *= .8;
				yaw_delta *= .5;
				pitch_delta +=
						(turn.next_float() - turn.next_float()) * turn.next_float() * 2.0;
				yaw_delta +=
						(turn.next_float() - turn.next_float()) * turn.next_float() * 4.0;
				if (turn.next_int(4) != 0)
					carve_ellipsoid(editor, region, int(std::lround(x)),
							int(std::lround(y)), int(std::lround(z)), half * h_mult,
							half * v_mult * 2.2, floor_level);
			}
		}
}

void decorate_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args)
{
	BiomeAmounts amounts = BiomeAmounts::defaults();
	if (args.cave_biomes)
		amounts = BiomeAmounts::parse(*args.cave_biomes);
	const std::vector<Block> rocks{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, GRAVEL, DIRT};
	// Keep decoration independent of traversal order: each candidate is keyed
	// by its absolute column and height, matching the Rust cave decoration pass.
	for (int z = region.min_z; z <= region.max_z; ++z)
		for (int x = region.min_x; x <= region.max_x; ++x) {
			const int top = editor.get_ground_level(x, z) - 8;
			for (int y = floor_y + 9; y < top; ++y) {
				if (!editor.check_for_block_absolute(x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR})))
					continue;
				const auto h = hash3(seed ^ 0x6a09e667f3bcc909LL, x, y, z);
				const bool adjacent_rock =
						editor.check_for_block_absolute(x + 1, y, z, rocks) ||
						editor.check_for_block_absolute(x - 1, y, z, rocks) ||
						editor.check_for_block_absolute(x, y, z + 1, rocks) ||
						editor.check_for_block_absolute(x, y, z - 1, rocks);
				if (!adjacent_rock)
					continue;
				const bool floor_rock =
						editor.check_for_block_absolute(x, y - 1, z, rocks);
				const auto theme = h % 1000;
				if (theme < 7 && amounts.lush > 0 && floor_rock)
					editor.set_block_absolute(MOSS_BLOCK, x, y - 1, z,
							std::optional<std::vector<Block>>(
									std::vector<Block>{STONE, DEEPSLATE, TUFF, DIRT}),
							std::nullopt);
				else if (theme >= 40 && theme < 44 && amounts.volcanic > 0 && floor_rock)
					editor.set_block_absolute(MAGMA_BLOCK, x, y - 1, z,
							std::optional<std::vector<Block>>(rocks), std::nullopt);
				else if (theme >= 44 && theme < 48 && amounts.deepdark > 0 && floor_rock)
					editor.set_block_absolute(BLACKSTONE, x, y - 1, z,
							std::optional<std::vector<Block>>(rocks), std::nullopt);
				else if (theme >= 48 && theme < 51 && amounts.ice > 0 && floor_rock &&
						 y > top - 20)
					editor.set_block_absolute(ICE, x, y - 1, z,
							std::optional<std::vector<Block>>(rocks), std::nullopt);
				else if (theme == 777 && amounts.mushroom > 0 && y + 1 < top)
					editor.set_block_absolute(BROWN_MUSHROOM, x, y + 1, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
			}
		}
}
}
