#pragma once

#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"

namespace arnis::building_construction
{
struct Progress
{
	int storey = 3;
	int planned = 1;
	int built = 1;
	static Progress of(std::uint64_t id, int height, int storey);
	bool topped_out() const { return built >= planned; }
};

void generate(WorldEditor &, const ProcessedWay &,
		const std::vector<std::pair<int, int>> &, int base_y, Progress, bool terrain,
		double scale, const CoordinateBitmap &footprints,
		const CoordinateBitmap &passages, int passage_height);
} // namespace arnis::building_construction
