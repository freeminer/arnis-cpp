#pragma once

#include <nlohmann/json_fwd.hpp>

#include <string>
#include <utility>
#include <vector>

#include "celestial.h"

namespace arnis::datapack::disasters
{
// Rust Plan::without_maps counterpart. The generated functions deliberately
// omit floods, tsunamis and wildfires when no recorded surface/wood map exists.
class Plan
{
public:
	Plan(CelestialBody body, int floor, int world_max_y);

	std::string coming() const;
	std::vector<std::pair<std::string, std::string>> files(const std::string &area) const;
	nlohmann::json dialog(const std::string &area, bool has_pause_menu) const;

private:
	CelestialBody body_;
	int floor_;
	int snow_top_;
	std::vector<int> kinds_;
};
} // namespace arnis::datapack::disasters
