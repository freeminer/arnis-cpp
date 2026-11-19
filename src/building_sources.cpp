#include "building_sources.h"

#include "../../arnis_ground.h"
#include "floodfill_cache.h"
#include "overture.h"
#include "survey/elements.h"
#include "survey/heights.h"
#include "survey/roofs.h"

#include <utility>

namespace arnis::building_sources
{

HeightMergeStats merge_heights(std::vector<ProcessedElement> &elements,
		const Ground *ground, const ::XZBBox &bbox, double scale,
		const overture::OvertureHints *hints)
{
	HeightMergeStats stats;
	survey::heights::ApplyResult measured;
	if (ground) {
		measured = survey::heights::apply(elements, *ground, bbox, scale);
		stats.measured = measured.filled;
	}
	if (hints)
		stats.overture = hints->apply(elements);
	if (ground)
		stats.deferred = survey::heights::apply_deferred(elements, measured.deferred);
	if (ground)
		stats.roof_shapes = survey::roofs::apply(elements, *ground, bbox, scale);
	return stats;
}

GardenMergeStats merge_gardens(const Ground &ground, const ::XZBBox &bbox,
		const std::vector<ProcessedElement> &existing, double scale,
		const std::function<bool(int, int)> &is_water)
{
	GardenMergeStats stats;
	if (!ground.has_survey())
		return stats;
	auto measured = survey::elements::candidates(ground, bbox);
	auto result = survey::elements::merge(std::move(measured), existing, scale, is_water);
	stats.elements = std::move(result.first);
	stats.on_paved = result.second;
	return stats;
}

} // namespace arnis::building_sources
