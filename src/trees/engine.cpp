#include "engine.h"
#include "../element_processing/bridges.h"
#include "../block_definitions.h"
#include "../element_processing/tree.h"
#include <array>
#include <algorithm>
namespace arnis::trees
{
std::uint64_t tree_seed(int x, int z)
{
	std::uint64_t h = 1469598103934665603ull;
	h ^= std::uint32_t(x);
	h *= 1099511628211ull;
	h ^= std::uint32_t(z);
	h *= 1099511628211ull;
	h ^= 0x54524545ull;
	h *= 1099511628211ull;
	return h;
}
bool place_region_tree(world_editor::WorldEditor &e, double lat, double lon,
		const std::filesystem::path &root, Habitat h, unsigned width, std::uint64_t seed,
		int x, int ground_y, int z, unsigned rotation)
{
	TreePackSource source(realm_for_latlon(lat, lon), root);
	auto lib = load_combined_region_library(lat, lon, root);
	if (!lib.accepts(seed))
		return false;
	return lib.place(e, source, h, width, seed, subtropical_latitude(lat), x, ground_y, z,
			rotation);
}
bool place_region_tree_for_cover(world_editor::WorldEditor &e, double lat, double lon,
		const std::filesystem::path &root, std::uint8_t lc, unsigned width,
		std::uint64_t seed, int x, int ground_y, int z, unsigned rotation)
{
	return place_region_tree(e, lat, lon, root, habitat_for_land_cover(lc), width, seed,
			x, ground_y, z, rotation);
}
bool place_region_tree_at(world_editor::WorldEditor &e, double lat, double lon,
		const std::filesystem::path &root, std::uint8_t lc, unsigned width, int x,
		int ground_y, int z, unsigned rotation)
{
	return place_region_tree_for_cover(
			e, lat, lon, root, lc, width, tree_seed(x, z), x, ground_y, z, rotation);
}
bool place_selected_region_tree(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int x, int z, Habitat habitat, int elevation_y,
		SlotRequest request, const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	const auto blocked = [&](int sx, int sz) {
		return editor.surface_is_sealed(sx, sz) || editor.is_lc_water(sx, sz) ||
			   (building_footprints && building_footprints->contains(sx, sz)) ||
			   (bridge_surface && bridge_surface->contains(sx, sz)) ||
			   editor.check_for_block(sx, 0, sz,
					   std::optional<std::vector<Block>>(
							   {block_definitions::BLACK_CONCRETE,
									   block_definitions::GRAY_CONCRETE_POWDER,
									   block_definitions::CYAN_TERRACOTTA,
									   block_definitions::GRAY_CONCRETE,
									   block_definitions::LIGHT_GRAY_CONCRETE,
									   block_definitions::DIRT_PATH,
									   block_definitions::SMOOTH_STONE,
									   block_definitions::WATER}));
	};
	if (blocked(x, z))
		return false;
	auto selected = selector.pick_slot(x, z, habitat, elevation_y, request);
	if (!selected || blocked(selected->x, selected->z))
		return false;
	const auto *schem = selector.schematic(selected->schematic_index);
	if (!schem)
		return false;
	// The selected lattice slot can move the trunk off the requested column.
	// Preserve the caller's ground offset while anchoring to that slot's terrain.
	const int offset = elevation_y - editor.get_ground_level(x, z);
	const int sx = selected->x, sz = selected->z;
	const int center = editor.get_absolute_y(sx, offset, sz);
	int fpmin = center;
	const int half = std::clamp(std::max(schem->width, schem->length) / 2, 1, 6);
	for (const auto &[dx, dz] : std::array<std::pair<int, int>, 8>{
				 {{-half, 0}, {half, 0}, {0, -half}, {0, half}, {-half, -half},
						 {half, half}, {-half, half}, {half, -half}}})
		fpmin = std::min(fpmin, editor.get_absolute_y(sx + dx, offset, sz + dz));
	auto blacklist = Tree::get_building_wall_blocks();
	for (const auto &blocks : {Tree::get_building_floor_blocks(),
				 Tree::get_structural_blocks(), Tree::get_functional_blocks()})
		blacklist.insert(blacklist.end(), blocks.begin(), blocks.end());
	blacklist.push_back(block_definitions::WATER);
	return place_schematic_tree(editor, *schem, sx, sz, std::min(center, fpmin + 2),
			selected->rotation, blacklist, building_footprints, offset);
}
bool place_selected_region_tree_for_cover(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int x, int z, std::uint8_t cover, int elevation_y,
		SlotRequest request, const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	return place_selected_region_tree(editor, selector, x, z,
			habitat_for_land_cover(cover), elevation_y, request, building_footprints,
			bridge_surface);
}
}
