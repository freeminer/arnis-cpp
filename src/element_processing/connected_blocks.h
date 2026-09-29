#pragma once

#include "../../arnis_block.h"
#include "../../arnis_world_editor.h"
#include <utility>
#include <vector>

namespace arnis::connected_blocks
{
bool is_connectable(const Block &block);
void place_connected(WorldEditor &editor, const Block &block, int x, int y, int z);
BlockWithProperties connected_fence(
		const Block &block, bool north, bool south, bool east, bool west);
BlockWithProperties connected_wall(
		const Block &block, bool north, bool south, bool east, bool west);
std::vector<std::pair<int, int>> stair_steps(
		std::pair<int, int> prev, std::pair<int, int> curr);
std::vector<std::pair<int, int>> four_connected_line(
		const std::vector<std::pair<int, int>> &points);
std::vector<std::pair<int, int>> cross_cells(
		int x, int z, std::pair<float, float> perp, int from, int to);
}
