#pragma once

#include "elevation.h"

#include <optional>
#include <utility>
#include <vector>

namespace arnis::geographic
{
class LLBBox;
}
namespace arnis::land_cover
{
struct LandCoverData;
}

namespace arnis::elevation::detail
{
std::size_t add_steep_detail(std::vector<std::vector<double>> &heights,
		const geographic::LLBBox &bbox, const SourcePixels &source, double scale,
		std::optional<std::pair<std::int64_t, std::int64_t>> one_world_origin,
		const land_cover::LandCoverData *land_cover,
		const std::vector<std::vector<std::pair<double, double>>> &graded);
}
