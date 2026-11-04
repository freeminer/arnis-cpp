#pragma once

#include <utility>
#include <vector>

#include "../block_definitions.h"
#include "../osm_parser.h"
#include "../../../arnis_adapter.h"

namespace arnis::sport_pitches
{
std::vector<std::pair<double, double>> hull_ccw(
		const std::vector<std::pair<int, int>> &points);
std::optional<Block> default_surface(const tags_t &tags);
std::optional<std::pair<double, double>> longest_edge_dir(const ProcessedWay &way);
void draw_pitch_markings(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &filled_area, Block surface);
}
