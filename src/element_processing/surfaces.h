#pragma once

#include <vector>
#include <string>
#include <optional>

#include "../block_definitions.h"
#include "../../../arnis_adapter.h"

namespace arnis::surfaces
{

const std::vector<Block> *get_blocks_for_surface(const std::string &surface_type);
const std::vector<Block> *get_blocks_for_surface_way_view(
		const ProcessedWay &way, const std::vector<Block> &default_blocks);
std::vector<Block> get_blocks_for_surface_way(
		const ProcessedWay &way, const std::vector<Block> &default_blocks);
const std::vector<Block> *cycleway_palette_view(const tags_t &tags);
const std::vector<Block> *cycleway_palette_view(const ProcessedWay &way);
std::optional<std::vector<Block>> cycleway_palette(const tags_t &tags);
std::optional<std::vector<Block>> cycleway_palette(const ProcessedWay &way);
Block semirandom_surface(int x, int z, const std::vector<Block> &block_types);

}
