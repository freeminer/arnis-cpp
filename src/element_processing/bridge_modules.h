#pragma once

#include "../../../arnis_adapter.h"
#include "../structures/schem_decoder.h"
#include <vector>
#include <optional>
#include <cstddef>

namespace arnis::bridge_modules {

struct BridgePathSample {
	int x, deck_y, z;
	float px, pz;
};

struct BridgeModuleSlice {
	int w, dy;
	BlockWithProperties block;
};

struct BridgeModule {
	size_t length;
	int half_width;
	bool has_pillars;
	std::vector<std::vector<BridgeModuleSlice>> slices;
	std::vector<std::vector<BridgeModuleSlice>> feet;
};

constexpr int MIN_MODULE_BRIDGE_LEN = 12;

std::optional<std::size_t> pick_module_index(int block_range, std::size_t bridge_len);
const BridgeModule* module_at(std::size_t idx);
int module_half_width(std::size_t idx);

void sweep_module(world_editor::WorldEditor* editor,
                   const std::vector<BridgePathSample>& path,
                   const BridgeModule& module);

} // namespace arnis::bridge_modules
