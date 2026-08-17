#include "engine.h"
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
		SlotRequest request)
{
	auto selected = selector.pick_slot(x, z, habitat, elevation_y, request);
	if (!selected)
		return false;
	const auto *schem = selector.schematic(selected->schematic_index);
	return schem && place_schematic_rooted(editor, *schem, selected->x, elevation_y,
							selected->z, selected->rotation);
}
bool place_selected_region_tree_for_cover(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int x, int z, std::uint8_t cover, int elevation_y,
		SlotRequest request)
{
	return place_selected_region_tree(
			editor, selector, x, z, habitat_for_land_cover(cover), elevation_y, request);
}
}
