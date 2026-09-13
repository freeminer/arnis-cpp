#pragma once

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis::structures::jetbridge
{
bool claims(const ProcessedWay &way);
void generate_jet_bridge(
		WorldEditor &, const ProcessedWay &, const BuildingFootprintBitmap &);
}
