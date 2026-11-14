#pragma once

#include "../floodfill_cache.h"

namespace arnis::camp_sites
{

bool claims(const tags_t &tags);
bool is_pitch(const tags_t &tags);
void prepare(std::vector<ProcessedElement> &elements);
void generate_camp_site(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &flood_fill_cache,
		const BuildingFootprintBitmap &building_footprints,
		const RoadMaskBitmap &road_mask,
		const std::optional<std::chrono::milliseconds> &timeout);
void camp_area(WorldEditor &editor, const ProcessedWay &way,
		const FloodFillCache &flood_fill_cache,
		const BuildingFootprintBitmap &building_footprints,
		const RoadMaskBitmap &road_mask,
		const std::optional<std::chrono::milliseconds> &timeout, const Holes *holes);
void generate_pitch_node(WorldEditor &editor, const ProcessedNode &node);

} // namespace arnis::camp_sites
