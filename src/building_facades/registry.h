#pragma once

#include "../element_processing/building_facade.h"
#include "choose.h"
#include "image.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::building_facades
{
void reset(bool enabled, const std::optional<std::filesystem::path> &directory,
		std::uint32_t pixels_per_metre, double scale);
void set_world_extent(int min_x, int min_z, int max_x, int max_z);
bool enabled();

// Collect and immediately emit the preset panels for one completed building.
// The Rust implementation queues these by atlas region; the C++ backend owns
// the texture store, so emitting a gathered panel is the equivalent boundary.
void collect(world_editor::WorldEditor &, const std::vector<ProcessedNode> &,
		std::uint64_t way_id, buildings::BuildingCategory,
		const building_facade::FacadePlan &, int base_y, int building_height,
		double scale);
}
