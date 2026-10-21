#pragma once

#include "../floodfill_cache.h"
#include "../args.h"

#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

namespace arnis::waterfront
{

void generate_pier(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &cache, const Args &args);
void generate_breakwater(world_editor::WorldEditor &editor,
		const ProcessedElement &element, const FloodFillCache &cache, const Args &args);
void generate_dolphin(world_editor::WorldEditor &editor, const ProcessedNode &node);
std::size_t drop_paths_on_piers(std::vector<ProcessedElement> &elements, double scale,
		const FloodFillCache &cache,
		const std::optional<std::chrono::milliseconds> &timeout);

} // namespace arnis::waterfront
