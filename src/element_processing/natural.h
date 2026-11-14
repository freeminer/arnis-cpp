#pragma once

#include "../args.h"

namespace arnis::natural
{
// In a split One World job, draw area-level decoration as usual but let only
// the unit owning a cell root trees and bushes there.
bool piece_plants_at(const Args &args, int x, int z);

// Forest tree placement uses coordinate-seeded draws in split jobs so both
// neighbouring pieces select the same trees in their overlap.
bool woods_by_cell(const Args &args);
} // namespace arnis::natural
