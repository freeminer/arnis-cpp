#include "ore_generation.h"
#include "block_definitions.h"
#include "../../arnis_adapter.h"
#include "deterministic_rng.h"
#include "world_editor/floor_state.h"
#include <algorithm>
#include <limits>
#include <optional>
namespace arnis::ore_generation
{
const std::array<OreRule, 6> &rules()
{
	static const std::array<OreRule, 6> r{{{block_definitions::COAL_ORE, 3, 45, 8, 17, 8},
			{block_definitions::IRON_ORE, 3, 60, 5, 9, 6},
			{block_definitions::LAPIS_ORE, 25, 55, 4, 7, 2},
			{block_definitions::GOLD_ORE, 40, 60, 5, 9, 3},
			{block_definitions::REDSTONE_ORE, 45, 65, 5, 10, 4},
			{block_definitions::DIAMOND_ORE, 50, 65, 4, 7, 1}}};
	return r;
}
void generate_ores(
		world_editor::WorldEditor &e, int min_x, int max_x, int min_z, int max_z)
{
	const int min_y = world_editor::terrain_floor_y() + 1;
	for (int cx = min_x >> 4; cx <= (max_x >> 4); ++cx)
		for (int cz = min_z >> 4; cz <= (max_z >> 4); ++cz) {
			ChaCha8Rng rng((std::uint64_t(std::uint32_t(cx)) << 32) ^ std::uint32_t(cz) ^
						   0xC0DE);
			const int ground = e.get_ground_level((cx << 4) + 8, (cz << 4) + 8);
			for (const auto &r : rules()) {
				const int y_max = std::max(min_y, ground - r.depth_min);
				if (y_max < min_y)
					continue;
				const unsigned span = unsigned(y_max - min_y + 1),
							   original = unsigned(r.depth_max - r.depth_min + 1);
				const unsigned max_veins =
						original ? std::min<unsigned>(
										   std::numeric_limits<unsigned>::max() / 2,
										   r.avg_veins_per_chunk * span * 2 / original)
								 : 0;
				const unsigned n = rng.uniform(max_veins + 1);
				for (unsigned i = 0; i < n; ++i) {
					int x = (cx << 4) + int(rng.uniform(16)),
						z = (cz << 4) + int(rng.uniform(16)),
						y = min_y + int(rng.uniform(span));
					const unsigned length =
							r.vein_min + rng.uniform(r.vein_max - r.vein_min + 1);
					for (unsigned j = 0; j < length; ++j) {
						// Rust continues the random walk past a tile edge; the editor's
						// region/stone checks own clipping. Breaking here changed the RNG
						// trajectory and made the same chunk produce different veins when
						// generated through a larger world region.
						if (e.check_for_block_absolute(x, y, z,
									std::optional<std::vector<Block>>(
											{block_definitions::STONE})))
							e.set_block_absolute(r.block, x, y, z,
									std::optional<const std::vector<Block>>(
											std::vector<Block>{
													block_definitions::STONE}));
						switch (rng.uniform(6)) {
						case 0:
							++x;
							break;
						case 1:
							--x;
							break;
						case 2:
							++y;
							break;
						case 3:
							--y;
							break;
						case 4:
							++z;
							break;
						default:
							--z;
							break;
						}
					}
				}
			}
		}
}
}
