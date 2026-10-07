#pragma once

#include <array>

#include "../../../arnis_adapter.h"

namespace arnis::bush
{
enum class Kind
{
	Wild,
	Garden,
	Low
};

const std::array<Block, 8> &leaf_blocks();

void place(WorldEditor &editor, int x, int z, Kind kind);
Block shrubbery_species(const WorldEditor &editor, int x, int z);
Block shrubbery_leaf(Block species, int x, int y, int z);
} // namespace arnis::bush
