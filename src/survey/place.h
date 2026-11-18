#pragma once

#include "../../../arnis_block.h"
#include "survey.h"

#include <optional>
#include <utility>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::survey
{

bool claims_cover(std::uint8_t cover);
std::optional<std::pair<Block, Block>> ground_blocks(
		Cell cell, int x, int z, std::uint8_t cover);
bool lawn_tuft(int x, int z);
void place_objects(
		world_editor::WorldEditor &editor, Cell cell, int x, int ground_y, int z);

} // namespace arnis::survey
