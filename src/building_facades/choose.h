#pragma once
#include "../element_processing/buildings.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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
		const auto dimension = [ppm](double metres) {
			// Rust's float-to-u32 cast saturates: NaN/negative values become zero
			// and values above u32::MAX clamp. Preserve that contract without
			// passing an out-of-range value through llround or an integer cast.
			const double rounded = std::round(metres * ppm);
			if (!(rounded > 1.0))
				return std::uint32_t{1};
			constexpr auto max_dimension = std::numeric_limits<std::uint32_t>::max();
			if (rounded >= static_cast<double>(max_dimension))
				return max_dimension;
			return static_cast<std::uint32_t>(rounded);
		};
		return {dimension(metres_wide), dimension(metres_tall)};
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
class FacadeSet;
std::uint64_t hash64(std::uint64_t);
double score(const Entry &, double wall_height_m);
std::pair<std::vector<std::size_t>, Fallback> candidates(
		const FacadeSet &, buildings::BuildingCategory);
std::vector<std::size_t> shortlist(
		const FacadeSet &, buildings::BuildingCategory, double wall_height_m);
std::size_t slot(int x, int z, std::size_t count);
std::optional<Choice> choose(const FacadeSet &, buildings::BuildingCategory,
		double wall_height_m, std::uint64_t way_id, int x, int z);
}
