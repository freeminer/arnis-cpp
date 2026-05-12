#pragma once

#include <utility>
#include <vector>

#include "../../floodfill_cache.h"

namespace arnis::map_transformation::rotate
{

std::pair<int, int> rotate_xz_point(int x, int z, double angle_degrees,
        const XZBBox &xzbbox);

void rotate_world(double angle_degrees, std::vector<ProcessedElement> &elements,
        XZBBox &xzbbox);

}
