#pragma once

#include "../../arnis_adapter.h"

namespace arnis
{
struct PreparedBuildingData;

// Prepare grouping/suppression while original geographic coordinates are still
// available, matching osm_parser.rs before map projection and clipping.
PreparedBuildingData prepare_building_data(const std::vector<ProcessedElement> &elements);
} // namespace arnis
