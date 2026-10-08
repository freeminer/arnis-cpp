#pragma once

#include "../element_processing/building_facade.h"
#include "choose.h"
#include "image.h"

#include <cstddef>
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
std::size_t finalize(world_editor::WorldEditor &);

// Collect facade runs with the shared part-group seed. Finalization selects one
// picture per building group and submits panels against the completed world.
building_facade::PointSet collect(world_editor::WorldEditor &,
		const std::vector<ProcessedNode> &, std::uint64_t way_id,
		std::uint64_t group_seed, buildings::BuildingCategory,
		const building_facade::FacadePlan &, int base_y, int building_height,
		double scale);
}
