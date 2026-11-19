#pragma once

#include "../osm_parser.h"

#include <cstddef>
#include <vector>

struct XZBBox;

namespace arnis
{
struct Ground;
namespace survey::roofs
{
inline constexpr char SHAPE_HINT[] = "arnis:roof:shape";
inline constexpr char DIRECTION_HINT[] = "arnis:roof:direction";

// Annotate OSM and Overture building outlines with the shape visible in the
// laser survey. Existing mapped roof tags always take precedence.
std::size_t apply(std::vector<ProcessedElement> &elements, const Ground &ground,
		const ::XZBBox &bbox, double scale);
} // namespace survey::roofs
} // namespace arnis
