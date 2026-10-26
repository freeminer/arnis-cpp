#pragma once

#include "../../../arnis_adapter.h"
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::datapack::discoveries
{

enum class Category
{
	Landmark,
	Culture,
	Viewpoint,
	Nature,
	Faith,
	Transport,
	Bridge,
	Sport,
	Civic
};

const char *title(Category category);

struct Place
{
	std::string name;
	Category category = Category::Landmark;
	std::string label;
	std::pair<int, int> min{};
	std::pair<int, int> max{};
	std::pair<int, int> anchor{};
	std::optional<int> ground_y;
	bool notable = false;

	void settle(const std::function<bool(int, int)> &open,
			const std::function<bool(int, int)> &street,
			const std::function<int(int, int)> &ground_y);
};

// Produce the advancement root and per-place/category tree for the three
// vanilla predicate schemas used by successive Java data versions.
std::vector<std::pair<std::string, std::string>> advancement_files(
		const std::string &area, const std::vector<Place> &places);

// Named places intersecting the world bounds, sorted by importance, and the
// highest-ranked named settlement available as a fallback area name.
std::pair<std::vector<Place>, std::optional<std::string>> collect(
		const std::vector<ProcessedElement> &elements, const cartesian::XZBBox &world);

} // namespace arnis::datapack::discoveries
