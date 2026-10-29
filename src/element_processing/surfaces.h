#pragma once

#include <span>
#include <string>

#include "../block_definitions.h"
#include "../../../arnis_adapter.h"

namespace arnis::surfaces
{

std::span<const Block> get_blocks_for_surface(const std::string &surface_type);
std::span<const Block> get_blocks_for_surface_way(
		const ProcessedWay &way, std::span<const Block> default_blocks);
std::span<const Block> cycleway_palette(const tags_t &tags);
std::span<const Block> cycleway_palette(const ProcessedWay &way);
Block semirandom_surface(int x, int z, std::span<const Block> block_types);

}
