#include "bridge_modules.h"
#include "../assets_root.h"
#include "../block_definitions.h"
#include "bridges.h"
#include "../structures/schem_decoder.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <string_view>
#include <unordered_map>

namespace arnis::bridge_modules
{

const int PILLAR_FOOT_MIN_DEPTH = 3;
const int PILLAR_GROUND_FILL_LIMIT = 512;

// Keep one immutable module set per asset root. Generation workers may still
// be sweeping a module while another world/editor initializes a different
// asset root; replacing a process-global vector would invalidate those refs.
static std::map<std::filesystem::path, std::vector<BridgeModule>> loaded_modules;
static std::mutex modules_mutex;

static bool is_pillar_material(const Block &block)
{
	using namespace block_definitions;
	return block.id() == SANDSTONE.id() || block.id() == SMOOTH_SANDSTONE.id() ||
		   block.id() == STONE.id() || block.id() == STONE_BRICKS.id() ||
		   block.id() == ANDESITE.id() || block.id() == ANDESITE_WALL.id() ||
		   block.id() == COBBLESTONE.id() || block.id() == SMOOTH_STONE.id() ||
		   block.id() == structures::resolve_schem_block("minecraft:sandstone_wall").id();
}

static bool is_street_block(const Block &block)
{
	using namespace block_definitions;
	return block == CYAN_TERRACOTTA || block == WHITE_CONCRETE ||
		   block == YELLOW_CONCRETE;
}

static bool name_ends_with(std::string_view name, std::string_view suffix)
{
	return name.size() >= suffix.size() &&
		   name.substr(name.size() - suffix.size()) == suffix;
}

static bool is_railing_part(std::string_view name)
{
	for (const auto suffix : {"_wall", "_fence", "_fence_gate", "_bars", "_sign",
				 "_button", "_trapdoor", "_pane", "lantern"})
		if (name_ends_with(name, suffix))
			return true;
	return name.find("chain") != std::string_view::npos;
}

static bool side_faces_open(const bridges::BridgeSurfaceMap &surface,
		std::pair<int, int> cell, std::pair<float, float> outward, int deck_y)
{
	for (int k = 0; k <= 2; ++k) {
		const int x = static_cast<int>(std::lround(cell.first + outward.first * k));
		const int z = static_cast<int>(std::lround(cell.second + outward.second * k));
		if (surface.deck_near(x, z, deck_y, 2))
			return false;
	}
	return true;
}

static const std::vector<BridgeModule> &modules_for_root(
		const std::filesystem::path &requested_root = {})
{
	const auto asset_dir =
			requested_root.empty() ? assets::path("structures") : requested_root;
	const std::lock_guard<std::mutex> lock(modules_mutex);
	if (const auto found = loaded_modules.find(asset_dir); found != loaded_modules.end())
		return found->second;

	std::vector<BridgeModule> modules;

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
						BlockWithProperties{structures::resolve_schem_block(voxel.block),
								voxel.properties},
						voxel.block});
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
			modules.push_back(std::move(module));
		} catch (...) {
			// A missing or invalid optional segment simply disables modular bridges.
		}
	};
	build_module("bridge_segment_1", 8, true);
	build_module("bridge_segment_2", 16, true);
	build_module("bridge_segment_3", 2, false);
	build_module("bridge_segment_4", 3, false);
	return loaded_modules.emplace(asset_dir, std::move(modules)).first->second;
}

std::optional<size_t> pick_module_index(
		int block_range, size_t bridge_len, const std::filesystem::path &asset_root)
{
	const auto &modules = modules_for_root(asset_root);
	if (bridge_len < MIN_MODULE_BRIDGE_LEN || modules.size() < 4) {
		return std::nullopt;
	}

	if (block_range >= 6)
		return 0;
	if (block_range == 5) {
		return bridge_len >= 45 ? 1 : 2;
	}
	return 3;
}

const BridgeModule *module_at(size_t idx, const std::filesystem::path &asset_root)
{
	const auto &modules = modules_for_root(asset_root);
	if (idx >= modules.size())
		return nullptr;
	return &modules[idx];
}

int module_half_width(size_t idx, const std::filesystem::path &asset_root)
{
	const auto &modules = modules_for_root(asset_root);
	if (idx >= modules.size())
		return 0;
	return modules[idx].half_width;
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
	return BlockWithProperties{
			block.block, structures::rotate_schem_properties(block.properties, k)};
}

void sweep_module(world_editor::WorldEditor *editor,
		const bridges::BridgeSurfaceMap &surface,
		const std::vector<BridgePathSample> &path, const BridgeModule &module)
{
	if (!editor || path.empty() || module.length == 0)
		return;
	std::size_t open_left_count = 0, open_right_count = 0;
	for (const auto &sample : path) {
		const float edge =
				module.half_width * (std::abs(sample.px) + std::abs(sample.pz)) + 1.0f;
		for (const auto &[sign, count] : std::array<std::pair<float, std::size_t *>, 2>{
					 {{1.0f, &open_left_count}, {-1.0f, &open_right_count}}}) {
			const std::pair<int, int> cell{
					static_cast<int>(std::lround(sample.x + sample.px * sign * edge)),
					static_cast<int>(std::lround(sample.z + sample.pz * sign * edge))};
			*count += side_faces_open(
					surface, cell, {sample.px * sign, sample.pz * sign}, sample.deck_y);
		}
	}
	const bool open_left = open_left_count * 2 >= path.size();
	const bool open_right = open_right_count * 2 >= path.size();
	const auto for_cells = [](const BridgePathSample &sample, int w, auto &&fn) {
		const auto at = [&](int offset) {
			return std::pair{static_cast<int>(std::lround(sample.x + sample.px * offset)),
					static_cast<int>(std::lround(sample.z + sample.pz * offset))};
		};
		const auto [x, z] = at(w);
		const int step = (w > 0) - (w < 0);
		const auto [inner_x, inner_z] = at(w - step);
		fn(x, z);
		// A diagonal cross-section skips one cell on its inner edge. Fill that
		// corner so the swept deck is continuous on both diagonal orientations.
		if (w != 0 && inner_x != x && inner_z != z)
			fn(x, inner_z);
	};
	for (size_t i = 0; i < path.size(); ++i) {
		const auto &sample = path[i];
		const int deck_y = sample.deck_y;
		const std::uint8_t k = direction_quarter_turns(sample.px, sample.pz);
		const auto &slice = module.slices[i % module.length];
		for (const auto &voxel : slice) {
			const bool open = voxel.w > 0 ? open_left : open_right;
			if (!open && voxel.dy > 0 && std::abs(voxel.w) * 2 >= module.half_width)
				continue;
			if (!open && voxel.dy == 0 && is_railing_part(voxel.source_name))
				continue;
			const bool kerb = !open && voxel.dy == 0 &&
							  std::abs(voxel.w) >= module.half_width - 2 &&
							  !is_street_block(voxel.block.block);
			for_cells(sample, voxel.w, [&](int bx, int bz) {
				if (kerb) {
					editor->set_block_absolute(block_definitions::CYAN_TERRACOTTA, bx,
							deck_y, bz, nullptr, nullptr);
					return;
				}
				if (voxel.dy <= -PILLAR_FOOT_MIN_DEPTH &&
						surface.support_blocked(bx, bz, deck_y))
					return;
				editor->set_block_with_properties_absolute(rotated_block(voxel.block, k),
						bx, deck_y + voxel.dy, bz, std::nullopt,
						std::optional<std::vector<Block>>{std::vector<Block>{}});
			});
		}

		if (module.has_pillars) {
			const auto &feet_slice = module.feet[i % module.length];
			for (const auto &foot : feet_slice) {
				for_cells(sample, foot.w, [&](int bx, int bz) {
					if (surface.support_blocked(bx, bz, deck_y))
						return;
					const int bottom = deck_y + foot.dy;
					const int ground = editor->get_ground_level(bx, bz);
					for (int offset = 1; offset <= PILLAR_GROUND_FILL_LIMIT &&
										 offset <= bottom - ground;
							++offset) {
						const int y = bottom - offset;
						editor->set_block_with_properties_absolute(
								rotated_block(foot.block, k), bx, y, bz, std::nullopt,
								std::optional<std::vector<Block>>{std::vector<Block>{}});
					}
					editor->register_support_column(bx, bz, foot.block.block);
				});
			}
		}
	}
}

} // namespace bridge_modules
