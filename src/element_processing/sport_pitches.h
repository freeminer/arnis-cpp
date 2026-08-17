#pragma once

#include <utility>
#include <vector>

#include "../block_definitions.h"
#include "../osm_parser.h"
#include "../../../arnis_adapter.h"

namespace arnis::sport_pitches
{
void draw_pitch_markings(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &filled_area, Block surface);
}
