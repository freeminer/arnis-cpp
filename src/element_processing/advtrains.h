#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"

namespace arnis::railways::advtrains
{

bool available();
void prepare_network(const std::vector<ProcessedElement> &elements,
		world_editor::WorldEditor &editor);
std::vector<std::pair<int, int>> build_centerline(const ProcessedWay &way);
std::optional<Block> connected_rail(
		const std::vector<std::pair<int, int>> &line, std::size_t index,
		bool use_network = false);
std::vector<int> height_profile(
		world_editor::WorldEditor &editor,
		const std::vector<std::pair<int, int>> &line);
std::optional<Block> slope_rail(const std::vector<std::pair<int, int>> &line,
		const std::vector<int> &heights, std::size_t index);

} // namespace arnis::railways::advtrains
