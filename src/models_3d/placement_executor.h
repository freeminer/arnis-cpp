#pragma once

#include "custom/plane.h"
#include "custom/stadium.h"
#include "provider.h"
#include "three_dmr/client.h"
#include "three_dmr/prescan.h"
#include "wikidata/prescan.h"
#include <cstddef>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::models_3d
{
struct ModelPlacementStats
{
	std::size_t attempted = 0, placed = 0, voxels = 0;
	void add(const ModelPlacementStats &other)
	{
		attempted += other.attempted;
		placed += other.placed;
		voxels += other.voxels;
	}
};

// Execute prescanned provider placements.  These are deliberately independent
// of the world exporter and therefore work for an in-memory mapgen host too.
ModelPlacementStats place_wikidata_prescan(ModelProvider &, world_editor::WorldEditor &,
		const wikidata::PrescanResult &, double blocks_per_meter);
ModelPlacementStats place_three_dmr_prescan(three_dmr::Client &,
		world_editor::WorldEditor &, const three_dmr::PrescanResult &,
		double blocks_per_meter);
ModelPlacementStats place_plane_prescan(ModelProvider &, world_editor::WorldEditor &,
		const std::vector<custom::plane::Placement> &, double blocks_per_meter);
ModelPlacementStats place_stadium_prescan(ModelProvider &, world_editor::WorldEditor &,
		const std::vector<custom::stadium::Placement> &, double blocks_per_meter);
}
