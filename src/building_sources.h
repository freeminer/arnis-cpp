#pragma once

#include "osm_parser.h"

#include <cstddef>
#include <functional>
#include <vector>

struct XZBBox;

namespace arnis
{
struct Ground;
namespace overture
{
class OvertureHints;
}

namespace building_sources
{

struct HeightMergeStats
{
	std::size_t measured = 0;
	std::size_t overture = 0;
	std::size_t deferred = 0;
	std::size_t roof_shapes = 0;
};

struct GardenMergeStats
{
	std::vector<ProcessedElement> elements;
	std::size_t on_paved = 0;
};

// Match Rust source precedence: laser measurements first, Overture hints
// second, and saturated laser measurements only where Overture had no height.
HeightMergeStats merge_heights(std::vector<ProcessedElement> &elements,
		const Ground *ground, const ::XZBBox &bbox, double scale,
		const overture::OvertureHints *hints = nullptr);

// Add laser-measured sheds, garages and pools where mapped buildings and water
// do not already claim the cells.
GardenMergeStats merge_gardens(const Ground &ground, const ::XZBBox &bbox,
		const std::vector<ProcessedElement> &existing, double scale,
		const std::function<bool(int, int)> &is_water);

} // namespace building_sources
} // namespace arnis
