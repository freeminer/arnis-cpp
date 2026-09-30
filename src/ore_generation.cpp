#include "ore_generation.h"
#include "block_definitions.h"
#include "deterministic_rng.h"
#include "../../arnis_adapter.h"
#include "world_editor/floor_state.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

namespace arnis::ore_generation
{
namespace
{
struct Ore
{
	Block stone, deep;
	int size;
};
int chunk_floor(int v)
{
	// Rust's `>> 4` is an arithmetic shift, therefore negative coordinates
	// belong to the chunk below them rather than truncating toward zero.
	return v >= 0 ? v / 16 : -(((-v) + 15) / 16);
}
void blob(
		world_editor::WorldEditor &e, int cx, int cy, int cz, const Ore &o, ChaCha8Rng &r)
{
	for (int i = 0; i < o.size; ++i) {
		if (e.check_for_block_absolute(cx, cy, cz,
					std::optional<std::vector<Block>>(std::vector<Block>{STONE})))
			e.set_block_absolute(o.stone, cx, cy, cz,
					std::optional<std::vector<Block>>(std::vector<Block>{STONE}),
					std::nullopt);
		switch (r.uniform(6)) {
		case 0:
			++cx;
			break;
		case 1:
			--cx;
			break;
		case 2:
			++cy;
			break;
		case 3:
			--cy;
			break;
		case 4:
			++cz;
			break;
		default:
			--cz;
			break;
		}
	}
}
}
const std::array<OreRule, 6> &rules()
{
	static const std::array<OreRule, 6> r{{{COAL_ORE, COAL_ORE, 3, 45, 8, 17, 8},
			{IRON_ORE, IRON_ORE, 3, 60, 5, 9, 6}, {LAPIS_ORE, LAPIS_ORE, 25, 55, 4, 7, 2},
			{GOLD_ORE, GOLD_ORE, 40, 60, 5, 9, 3},
			{REDSTONE_ORE, REDSTONE_ORE, 45, 65, 5, 10, 4},
			{DIAMOND_ORE, DIAMOND_ORE, 50, 65, 4, 7, 1}}};
	return r;
}
void generate_ores(world_editor::WorldEditor &e, int min_x, int max_x, int min_z,
		int max_z, bool show_progress)
{
	if (show_progress)
		std::cout << "[6b/7] Sprinkling ore veins...\n";
	int floor = world_editor::terrain_floor_y();
	for (int cx = chunk_floor(min_x); cx <= chunk_floor(max_x); ++cx)
		for (int cz = chunk_floor(min_z); cz <= chunk_floor(max_z); ++cz) {
			ChaCha8Rng r = coord_rng(cx, cz, 0xC0DE);
			int ground = e.get_ground_level(cx * 16 + 8, cz * 16 + 8);
			for (const auto &rule : rules()) {
				int lo = std::max(floor + 1, ground - MAX_ORE_DEPTH),
					hi = std::max(lo, ground - rule.depth_min),
					span = std::max(0, hi - lo + 1),
					orig_span = rule.depth_max - rule.depth_min + 1,
					max_veins =
							orig_span > 0
									? int(rule.avg_veins_per_chunk * span * 2 / orig_span)
									: 0,
					n = static_cast<int>(
							r.uniform(static_cast<std::uint32_t>(max_veins + 1)));
				for (int i = 0; i < n; ++i) {
					Ore o{rule.block, rule.deep_block,
							static_cast<int>(
									rule.vein_min +
									r.uniform(rule.vein_max - rule.vein_min + 1))};
					blob(e, cx * 16 + r.uniform(16),
							lo + r.uniform(static_cast<std::uint32_t>(
										 std::max(0, hi - lo + 1))),
							cz * 16 + r.uniform(16), o, r);
				}
			}
		}
}
}
