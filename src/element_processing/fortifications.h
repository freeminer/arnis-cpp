#pragma once

#include "../../../arnis_adapter.h"

#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace arnis::fortifications
{

enum class Kind
{
	Wall,
	Gate
};

bool line_wall(const tags_t &tags);
std::optional<Kind> kind(const tags_t &tags);
bool crenellated(const tags_t &tags, Kind kind);

struct Mass
{
	const std::vector<std::pair<int, int>> &cells;
	Block block;
	int height{};
	std::optional<int> base;
	bool raised{};
	bool crenellated{};
	std::vector<std::pair<int, int>> passages;
	std::vector<std::pair<std::pair<int, int>, int>> footings;
	std::function<bool(int, int)> abutting;
	double scale{1.0};
};

void generate(WorldEditor &editor, const Mass &mass);

} // namespace arnis::fortifications
