#pragma once
#include "../element_processing/buildings.h"
#include <algorithm>
#include <optional>
#include <string>
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
};
struct Choice
{
	std::size_t entry = 0;
	double phase_m = 0;
};
std::uint64_t hash64(std::uint64_t);
double score(const Entry &, double wall_height_m);
std::vector<std::size_t> shortlist(
		const std::vector<Entry> &, buildings::BuildingCategory, double wall_height_m);
std::size_t slot(int x, int z, std::size_t count);
std::optional<Choice> choose(const std::vector<Entry> &, buildings::BuildingCategory,
		double wall_height_m, std::uint64_t way_id, int x, int z);
}
