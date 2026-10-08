#pragma once

#include "../../../arnis_adapter.h"

#include <vector>

namespace arnis
{
// Merge clipped OSM way fragments that share endpoints.
void merge_way_segments(std::vector<std::vector<ProcessedNode>> &rings);
} // namespace arnis
