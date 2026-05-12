#pragma once

#include <vector>

#include "../../arnis_adapter.h"

namespace arnis::overture
{

inline constexpr uint64_t OVERTURE_ID_HIGH_BIT = 0x8000000000000000ULL;

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
        double max_lat, double max_lng, double scale, bool debug);

std::vector<ProcessedElement> deduplicate_against_osm(
        std::vector<ProcessedElement> overture_elements,
        const std::vector<ProcessedElement> &osm_elements);

}
