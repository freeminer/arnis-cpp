#pragma once

#include "../osm_parser.h"

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

struct XZBBox;

namespace arnis
{
struct Ground;
}

namespace arnis::survey::elements
{

std::vector<ProcessedElement> candidates(const Ground &ground, const ::XZBBox &bbox);
std::pair<std::vector<ProcessedElement>, std::size_t> merge(
		std::vector<ProcessedElement> candidates,
		const std::vector<ProcessedElement> &existing, double scale,
		const std::function<bool(int, int)> &is_water);

} // namespace arnis::survey::elements
