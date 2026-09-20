#pragma once
#include "../element_processing/buildings.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>
namespace arnis::building_facades
{
struct Entry
{
	std::string file;
	std::vector<buildings::BuildingCategory> categories;
	double metres_wide = 1, metres_tall = 1;
	unsigned storeys = 1;
	bool tiles_horizontally = false, has_ground_floor = false;
	double storey_m() const { return metres_tall / std::max(1u, storeys); }
	double ground_m() const { return has_ground_floor ? storey_m() : 0.0; }
	double upper_m() const
	{
		const auto floor_count = has_ground_floor ? (storeys > 1 ? storeys - 1 : 1)
												  : std::max(1u, storeys);
		return static_cast<double>(floor_count) * storey_m();
	}
	std::pair<std::uint32_t, std::uint32_t> scaled_size(double pixels_per_metre) const
	{
		const auto ppm = std::max(0.0, pixels_per_metre);
		return {std::max<std::uint32_t>(
						1, static_cast<std::uint32_t>(std::llround(metres_wide * ppm))),
				std::max<std::uint32_t>(
						1, static_cast<std::uint32_t>(std::llround(metres_tall * ppm)))};
	}
};
struct Choice
{
	std::size_t entry = 0;
	double phase_m = 0;
};
enum class Fallback
{
	Own,
	Related,
	Default,
	WholeSet
};
std::uint64_t hash64(std::uint64_t);
double score(const Entry &, double wall_height_m);
std::pair<std::vector<std::size_t>, Fallback> candidates(
		const std::vector<Entry> &, buildings::BuildingCategory);
std::vector<std::size_t> shortlist(
		const std::vector<Entry> &, buildings::BuildingCategory, double wall_height_m);
std::size_t slot(int x, int z, std::size_t count);
std::optional<Choice> choose(const std::vector<Entry> &, buildings::BuildingCategory,
		double wall_height_m, std::uint64_t way_id, int x, int z);
}
