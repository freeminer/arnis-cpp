#pragma once

#include "../../../arnis_adapter.h"
#include "../structures/schem_decoder.h"
#include <vector>
#include <optional>
#include <cstddef>
#include <filesystem>
#include <string>

namespace arnis::bridges
{
class BridgeSurfaceMap;
}

namespace arnis::bridge_modules
{

struct BridgePathSample
{
	int x, deck_y, z;
	float px, pz;
};

struct BridgeModuleSlice
{
	int w, dy;
	BlockWithProperties block;
	// Keep the source name: Luanti's Block has no portable name accessor, but
	// bridge edge treatment follows the schematic's Java block-name suffixes.
	std::string source_name;
};

struct BridgeModule
{
	size_t length;
	int half_width;
	bool has_pillars;
	std::vector<std::vector<BridgeModuleSlice>> slices;
	std::vector<std::vector<BridgeModuleSlice>> feet;
};

constexpr int MIN_MODULE_BRIDGE_LEN = 12;

std::optional<std::size_t> pick_module_index(int block_range, std::size_t bridge_len,
		const std::filesystem::path &asset_root = {});
const BridgeModule *module_at(
		std::size_t idx, const std::filesystem::path &asset_root = {});
int module_half_width(std::size_t idx, const std::filesystem::path &asset_root = {});

void sweep_module(world_editor::WorldEditor *editor,
		const bridges::BridgeSurfaceMap &surface,
		const std::vector<BridgePathSample> &path, const BridgeModule &module);

} // namespace arnis::bridge_modules
