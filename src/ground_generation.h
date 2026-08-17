#pragma once

//#include "../../../arnis_adapter.h"
#include "args.h"
#include "floodfill_cache.h"

namespace arnis::ground_generation
{

// Stable value noise shared by terrain-adjacent feature generators.  This is
// intentionally public: Rust's natural and land-use passes use the same field
// to make borders deterministic across independently generated tiles.
double value_noise_01(int x, int z, int scale);

void generate_ground_layer(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints,
		const CoordinateBitmap *tunnel_footprint = nullptr);
// Tile/streaming entry point.  The shared land-cover grid remains referenced
// to xzbbox while only the inclusive iteration bounds are generated.
void generate_ground_region(WorldEditor &editor, const Args &args, const XZBBox &xzbbox,
		const BuildingFootprintBitmap &building_footprints, int iter_min_x,
		int iter_max_x, int iter_min_z, int iter_max_z,
		const CoordinateBitmap *tunnel_footprint = nullptr);

}
