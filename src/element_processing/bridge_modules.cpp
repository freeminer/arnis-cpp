#include "bridge_modules.h"
#include "../block_definitions.h"
#include "../structures/schem_decoder.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <unordered_map>

namespace arnis::bridge_modules
{

const int PILLAR_FOOT_MIN_DEPTH = 3;
const int PILLAR_GROUND_FILL_LIMIT = 48;

static std::vector<BridgeModule> loaded_modules;
static bool modules_loaded = false;

static bool is_pillar_material(const Block &block)
{
	using namespace block_definitions;
	return block.id() == SANDSTONE.id() || block.id() == SMOOTH_SANDSTONE.id() ||
			block.id() == STONE.id() || block.id() == STONE_BRICKS.id() ||
			block.id() == ANDESITE.id() || block.id() == ANDESITE_WALL.id() ||
			block.id() == COBBLESTONE.id() || block.id() == SMOOTH_STONE.id() ||
			block.id() == structures::resolve_schem_block("minecraft:sandstone_wall").id();
}

static void ensure_modules_loaded()
{
	if (modules_loaded)
		return;

	loaded_modules.clear();

	const auto asset_dir = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
			"assets/structures";
	const auto build_module = [&](const char *name, int street_y, bool pillars) {
		std::ifstream file(asset_dir / (std::string(name) + ".schem"), std::ios::binary);
		if (!file)
			return;
		std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), {});
		try {
			auto schem = structures::decode_sponge_schem(bytes);
			const int length = std::max(1, schem.width);
			const int center_w = schem.length / 2;
			BridgeModule module{static_cast<size_t>(length), center_w, pillars,
				std::vector<std::vector<BridgeModuleSlice>>(length),
				std::vector<std::vector<BridgeModuleSlice>>(length)};
			for (const auto &voxel : schem.voxels) {
				if (voxel.x < 0 || voxel.x >= length ||
						(voxel.block == "minecraft:stone_button" && voxel.y < street_y))
					continue;
				module.slices[voxel.x].push_back({voxel.z - center_w, voxel.y - street_y,
						BlockWithProperties{structures::resolve_schem_block(voxel.block), voxel.properties}});
			}
			if (pillars) {
				for (int l = 0; l < length; ++l) {
					std::unordered_map<int, BridgeModuleSlice> lowest;
					for (const auto &voxel : module.slices[l]) {
						const Block &b = voxel.block.block;
						if (!is_pillar_material(b))
							continue;
						auto it = lowest.find(voxel.w);
						if (it == lowest.end() || voxel.dy < it->second.dy)
							lowest[voxel.w] = voxel;
					}
					for (const auto &[w, voxel] : lowest)
						if (voxel.dy <= -PILLAR_FOOT_MIN_DEPTH)
							module.feet[l].push_back(voxel);
				}
			}
			loaded_modules.push_back(std::move(module));
		} catch (...) {
			// A missing or invalid optional segment simply disables modular bridges.
		}
	};
	build_module("bridge_segment_1", 8, true);
	build_module("bridge_segment_2", 16, true);
	build_module("bridge_segment_3", 2, false);
	build_module("bridge_segment_4", 3, false);
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

BlockWithProperties rotated_block(const BlockWithProperties &block, uint8_t k)
{
	if (k == 0)
		return block;
	return BlockWithProperties{block.block,
			structures::rotate_schem_properties(block.properties, k)};
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

			editor->set_block_with_properties_absolute(rotated_block(voxel.block, k), bx, by, bz,
				nullptr, nullptr);
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
					editor->set_block_with_properties_absolute(rotated_block(foot.block, k), bx, y, bz,
						nullptr, nullptr);
					}
				}
			}
		}
	}
}

} // namespace bridge_modules
