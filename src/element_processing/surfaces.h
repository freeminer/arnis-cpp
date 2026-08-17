#pragma once

#include <vector>
#include <string>

#include "../block_definitions.h"
#include "../../../arnis_adapter.h"

namespace arnis::surfaces
{

const std::vector<Block> *get_blocks_for_surface(const std::string &surface_type);
std::vector<Block> get_blocks_for_surface_way(
		const ProcessedWay &way, const std::vector<Block> &default_blocks);
Block semirandom_surface(int x, int z, const std::vector<Block> &block_types);

}
