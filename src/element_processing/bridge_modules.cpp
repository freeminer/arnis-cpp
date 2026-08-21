#include "bridge_modules.h"
#include "../block_definitions.h"
#include <cmath>
#include <unordered_map>

namespace arnis::bridge_modules
{

const int PILLAR_FOOT_MIN_DEPTH = 3;
const int PILLAR_GROUND_FILL_LIMIT = 48;

static std::vector<BridgeModule> loaded_modules;
static bool modules_loaded = false;

static void ensure_modules_loaded()
{
	if (modules_loaded)
		return;

	loaded_modules.clear();

	// Try to load bridge segments
	// These would need to be actual .schem files in the project
	// For now, create placeholder modules
	// TODO: Load actual schematic files when available
	modules_loaded = true;
}

std::optional<size_t> pick_module_index(int block_range, size_t bridge_len)
{
	ensure_modules_loaded();
	if (bridge_len < MIN_MODULE_BRIDGE_LEN || loaded_modules.size() < 4) {
		return std::nullopt;
	}

	if (block_range >= 6)
		return 0;
	if (block_range == 5) {
		return bridge_len >= 45 ? 1 : 2;
	}
	return 3;
}

const BridgeModule *module_at(size_t idx)
{
	ensure_modules_loaded();
	if (idx >= loaded_modules.size())
		return nullptr;
	return &loaded_modules[idx];
}

int module_half_width(size_t idx)
{
	ensure_modules_loaded();
	if (idx >= loaded_modules.size())
		return 0;
	return loaded_modules[idx].half_width;
}

static uint8_t direction_quarter_turns(float px, float pz)
{
	float ux = pz;
	float uz = -px;

	if (std::abs(ux) >= std::abs(uz)) {
		return ux >= 0.0f ? 0 : 2;
	} else {
		return uz >= 0.0f ? 1 : 3;
	}
}

Block rotated_block(const Block &block, uint8_t k)
{
	if (k == 0)
		return block;
	// TODO: Implement proper rotation based on block properties
	return block;
}

void sweep_module(world_editor::WorldEditor *editor,
		const std::vector<BridgePathSample> &path, const BridgeModule &module)
{

	for (size_t i = 0; i < path.size(); ++i) {
		const auto &sample = path[i];
		int x = sample.x;
		int deck_y = sample.deck_y;
		int z = sample.z;
		float px = sample.px;
		float pz = sample.pz;

		uint8_t k = direction_quarter_turns(px, pz);
		const auto &slice = module.slices[i % module.length];

		for (const auto &voxel : slice) {
			int bx = static_cast<int>(std::round(x + px * voxel.w));
			int bz = static_cast<int>(std::round(z + pz * voxel.w));
			int by = deck_y + voxel.dy;

			Block rotated = rotated_block(voxel.block, k);
			editor->set_block_absolute(rotated, bx, by, bz, {}, {});
		}

		// Place pillar feet
		if (module.has_pillars) {
			const auto &feet_slice = module.feet[i % module.length];
			for (const auto &foot : feet_slice) {
				int bx = static_cast<int>(std::round(x + px * foot.w));
				int bz = static_cast<int>(std::round(z + pz * foot.w));
				int bottom = deck_y + foot.dy;
				int ground = editor->get_ground_level(bx, bz);

				if (bottom > ground) {
					int limit = std::min(bottom, ground + PILLAR_GROUND_FILL_LIMIT);
					for (int y = ground; y < limit; ++y) {
						Block rotated = rotated_block(foot.block, k);
						editor->set_block_absolute(rotated, bx, y, bz, {}, {});
					}
				}
			}
		}
	}
}

} // namespace bridge_modules
