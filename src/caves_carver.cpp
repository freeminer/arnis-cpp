#include "caves_carver.h"
#include "caves_biomes.h"
#include "args.h"
#include "block_definitions.h"
#include "../../arnis_world_editor.h"
#include <cmath>
#include <optional>
#include <vector>

namespace arnis::caves
{
namespace
{
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
}
void carve_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y)
{
	const std::vector<Block> hosts{STONE, DEEPSLATE, DIRT, COARSE_DIRT, GRAVEL};
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
				const auto theme = h % 1000;
				if (theme < 7 && amounts.lush > 0)
					editor.set_block_absolute(MOSS_BLOCK, x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
				else if (theme >= 40 && theme < 44 && amounts.volcanic > 0)
					editor.set_block_absolute(MAGMA_BLOCK, x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
				else if (theme >= 44 && theme < 48 && amounts.deepdark > 0)
					editor.set_block_absolute(BLACKSTONE, x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
				else if (theme >= 48 && theme < 51 && amounts.ice > 0 && y > top - 20)
					editor.set_block_absolute(ICE, x, y, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
				else if (theme == 777 && amounts.mushroom > 0 && y + 1 < top)
					editor.set_block_absolute(BROWN_MUSHROOM, x, y + 1, z,
							std::optional<std::vector<Block>>(std::vector<Block>{AIR}),
							std::nullopt);
			}
		}
}
}
