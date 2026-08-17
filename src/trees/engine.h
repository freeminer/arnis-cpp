#pragma once
#include "region.h"
#include "tree_pack.h"
namespace arnis::trees
{
std::uint64_t tree_seed(int x, int z);
bool place_region_tree(world_editor::WorldEditor &, double lat, double lon,
		const std::filesystem::path &root, Habitat habitat, unsigned width,
		std::uint64_t seed, int x, int ground_y, int z, unsigned rotation = 0);
bool place_region_tree_for_cover(world_editor::WorldEditor &, double lat, double lon,
		const std::filesystem::path &root, std::uint8_t land_cover, unsigned width,
		std::uint64_t seed, int x, int ground_y, int z, unsigned rotation = 0);
bool place_region_tree_at(world_editor::WorldEditor &, double lat, double lon,
		const std::filesystem::path &, std::uint8_t, unsigned, int x, int ground_y, int z,
		unsigned rotation = 0);
// RegionSelector-backed path: selection and placement share the Rust lattice
// slot, so repeated streamed-tile requests stamp the same tree in the same
// orientation rather than consuming a global RNG sequence.
bool place_selected_region_tree(world_editor::WorldEditor &, const RegionSelector &,
		int x, int z, Habitat, int elevation_y, SlotRequest request = {});
bool place_selected_region_tree_for_cover(world_editor::WorldEditor &,
		const RegionSelector &, int x, int z, std::uint8_t land_cover, int elevation_y,
		SlotRequest request = {});
}
