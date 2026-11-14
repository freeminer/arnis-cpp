#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "args.h"
#include "../../arnis_types.h"

namespace arnis::water_detail
{

inline constexpr int BOWL_MAX_DEPTH = 5;

bool rounds_depth(WaterDetail detail);
bool uses_bowl(WaterDetail detail, double scale);
int channel_width_cap(WaterDetail detail, double scale);
bool drowns_crossing(WaterDetail detail, double scale, const tags_t &tags);
int max_carve_depth(WaterDetail detail, int estimate, double scale);
int bowl_depth(int x, int z, std::uint16_t dt_units, std::uint16_t component_max_units,
		double scale);
std::vector<bool> drowned_points(const std::vector<std::pair<int, int>> &points,
		int half_width, const std::function<bool(int, int)> &wet);

struct CarveDepth
{
	int floor{};
	WaterDetail detail{WaterDetail::Default};
	double scale{1.0};
	int crossing(int estimate) const;
	int bound(int estimate) const;
};

} // namespace arnis::water_detail
