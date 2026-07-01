#pragma once

#include <cstddef>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"

namespace arnis::structures
{

namespace fountain
{
void place(WorldEditor &editor, int x, int z, std::size_t area_cells);
}

namespace playground
{
void scatter_playgrounds(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace lighthouse
{
void place(WorldEditor &editor, int x, int z);
}

namespace crane
{
void maybe_place_crane(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace excavator
{
void scatter_excavators(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace tractor
{
void maybe_place_tractor(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace boat
{
void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z);
}

}
