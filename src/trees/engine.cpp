#include "engine.h"
#include "../element_processing/bridges.h"
#include "../block_definitions.h"
#include "../element_processing/tree.h"
#include "mapped.h"
#include "schematic.h"
#include "../ecoregion.h"
#include "../ground_decoration.h"
#include "../land_cover/land_cover.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>
namespace arnis::trees
{
namespace
{
std::optional<Habitat> ecoregion_habitat(
		const world_editor::WorldEditor &editor, int x, int z, std::uint8_t cover)
{
	if (!editor.ground)
		return std::nullopt;
	// Match Rust's rule that water/built/bare covers keep their explicit
	// habitat; the climate map refines only vegetation-bearing cells.
	using namespace land_cover;
	if (cover == LC_WATER || cover == LC_BUILT_UP || cover == LC_BARE ||
			cover == LC_SNOW_ICE)
		return std::nullopt;
	const auto eco = editor.ground->ecoregion_at(editor.ground_point(x, z));
	if (!eco)
		return std::nullopt;
	switch (eco->biome) {
	case ecoregion::EcoBiome::MoistTropical:
	case ecoregion::EcoBiome::DryTropical:
	case ecoregion::EcoBiome::TropicalConifer:
	case ecoregion::EcoBiome::TropicalGrassland:
		return Habitat::Tropical;
	case ecoregion::EcoBiome::TemperateConifer:
	case ecoregion::EcoBiome::Boreal:
	case ecoregion::EcoBiome::MontaneGrassland:
	case ecoregion::EcoBiome::Tundra:
		return Habitat::Conifer;
	case ecoregion::EcoBiome::Flooded:
	case ecoregion::EcoBiome::Mangroves:
		return Habitat::Wet;
	case ecoregion::EcoBiome::Desert:
	case ecoregion::EcoBiome::Mediterranean:
		return Habitat::Dry;
	default:
		return Habitat::Lowland;
	}
}

int schematic_base_y(world_editor::WorldEditor &editor, const Schematic &schem, int x,
		int z, int offset)
{
	const int center = editor.get_absolute_y(x, offset, z);
	int fpmin = center;
	const int half = std::clamp(std::max(schem.width, schem.length) / 2, 1, 6);
	for (const auto &[dx, dz] : std::array<std::pair<int, int>, 8>{
				 {{-half, 0}, {half, 0}, {0, -half}, {0, half}, {-half, -half},
						 {half, half}, {-half, half}, {half, -half}}})
		fpmin = std::min(fpmin, editor.get_absolute_y(x + dx, offset, z + dz));
	return std::min(center, fpmin + 2);
}

bool fits_tree_wood(world_editor::WorldEditor &editor, const Schematic &schem, int x,
		int z, unsigned rotation, int offset, bool allow_on_paved)
{
	const auto keep_off = [&](int wx, int wz) {
		return allow_on_paved ? editor.on_carriageway(wx, wz)
							  : editor.surface_is_sealed(wx, wz);
	};
	return low_wood_clear(editor, schem, rotation, x, z, keep_off,
			[&] { return schematic_base_y(editor, schem, x, z, offset); });
}

bool place_selected_schematic(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int offset, const SlotSelection &selected,
		const BuildingFootprintBitmap *building_footprints, bool allow_on_paved)
{
	const auto *schem = selector.schematic(selected.schematic_index);
	if (!schem)
		return false;
	const int sx = selected.x, sz = selected.z;
	const int base_y = schematic_base_y(editor, *schem, sx, sz, offset);
	auto blacklist = Tree::get_building_wall_blocks();
	for (const auto &blocks : {Tree::get_building_floor_blocks(),
				 Tree::get_structural_blocks(), Tree::get_functional_blocks()})
		blacklist.insert(blacklist.end(), blocks.begin(), blocks.end());
	blacklist.push_back(block_definitions::WATER);
	if (!fits_tree_wood(
				editor, *schem, sx, sz, selected.rotation, offset, allow_on_paved))
		return false;
	const auto &wood = ground_decoration::wood_blocks();
	for (int y = base_y; y <= base_y + 2; ++y)
		if (editor.check_for_block_absolute(sx, y, sz, wood))
			return false;
	if (!editor.claim_tree_slot(sx, sz))
		return false;
	return place_schematic_tree(editor, *schem, sx, sz, base_y, selected.rotation,
			blacklist, building_footprints, offset);
}
}
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
	const auto habitat =
			ecoregion_habitat(e, x, z, lc).value_or(habitat_for_land_cover(lc));
	return place_region_tree(
			e, lat, lon, root, habitat, width, seed, x, ground_y, z, rotation);
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
		const bridges::BridgeSurfaceMap *bridge_surface, bool allow_on_paved)
{
	const auto blocked = [&](int sx, int sz, bool selected_slot) {
		return (!allow_on_paved && editor.surface_is_sealed(sx, sz)) ||
			   (selected_slot && editor.is_lc_water(sx, sz)) ||
			   editor.tree_keep_clear_any(sx, sz) || editor.in_clear_corridor(sx, sz) ||
			   (building_footprints && building_footprints->contains(sx, sz)) ||
			   (bridge_surface && bridge_surface->contains(sx, sz)) ||
			   editor.check_for_block(sx, 0, sz,
					   allow_on_paved
							   ? std::optional<std::vector<Block>>(
										 {block_definitions::WATER})
							   : std::optional<std::vector<Block>>(
										 {block_definitions::BLACK_CONCRETE,
												 block_definitions::GRAY_CONCRETE_POWDER,
												 block_definitions::CYAN_TERRACOTTA,
												 block_definitions::GRAY_CONCRETE,
												 block_definitions::LIGHT_GRAY_CONCRETE,
												 block_definitions::DIRT_PATH,
												 block_definitions::SMOOTH_STONE,
												 block_definitions::WATER}));
	};
	// Match Rust's tile ownership rule: halo requests are ignored, but if the
	// deterministic schematic slot crosses an ownership seam, plant at the request.
	if (!editor.owns(x, z) || blocked(x, z, false))
		return false;
	const int offset = elevation_y - editor.get_ground_level(x, z);
	const auto [requested_slot_x, requested_slot_z] =
			trunk_slot_s(x, z, selector.slot_spacing(request.density_decided));
	const int fit_x =
			editor.owns(requested_slot_x, requested_slot_z) ? requested_slot_x : x;
	const int fit_z =
			editor.owns(requested_slot_x, requested_slot_z) ? requested_slot_z : z;
	auto caller_fit = request.fits;
	request.fits = [&](std::size_t index, unsigned rotation) {
		const auto *schem = selector.schematic(index);
		bool overlaps_kept_water = false;
		if (schem && editor.tree_keep_clear) {
			const int fw = (rotation & 1U) ? schem->length : schem->width;
			const int fl = (rotation & 1U) ? schem->width : schem->length;
			const int x0 = fit_x - (fw - 1) / 2;
			const int z0 = fit_z - (fl - 1) / 2;
			overlaps_kept_water =
					editor.tree_keep_clear->any_in(x0, z0, x0 + fw - 1, z0 + fl - 1);
			if (overlaps_kept_water)
				overlaps_kept_water = std::any_of(schem->voxels.begin(),
						schem->voxels.end(), [&](const auto &voxel) {
							const auto [rx, rz] = rotate_xz(voxel.x, voxel.z,
									schem->width, schem->length, rotation);
							return editor.tree_keep_clear->contains(x0 + rx, z0 + rz);
						});
		}
		return schem && !overlaps_kept_water &&
			   (!caller_fit || caller_fit(index, rotation)) &&
			   fits_tree_wood(
					   editor, *schem, fit_x, fit_z, rotation, offset, allow_on_paved);
	};
	auto fit_selection = selector.pick_slot(x, z, habitat, elevation_y, request);
	if (!fit_selection)
		return false;
	if (!editor.owns(fit_selection->x, fit_selection->z)) {
		fit_selection->x = x;
		fit_selection->z = z;
	}
	if (blocked(fit_selection->x, fit_selection->z, true))
		return false;
	return place_selected_schematic(editor, selector, offset, *fit_selection,
			building_footprints, allow_on_paved);
}
bool place_selected_mapped_region_tree(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int x, int z, int elevation_y, int y_offset,
		const MappedRequest &request, const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface)
{
	const auto blocked = [&](int sx, int sz, bool selected_slot) {
		// Mapped OSM trees are explicitly allowed on paving in Rust. Mapped crown
		// masks suppress nearby procedural vegetation, not the mapped trunk itself.
		return (selected_slot && editor.is_lc_water(sx, sz)) ||
			   editor.in_clear_corridor(sx, sz) ||
			   (building_footprints && building_footprints->contains(sx, sz)) ||
			   (bridge_surface && bridge_surface->contains(sx, sz)) ||
			   editor.check_for_block(sx, 0, sz,
					   std::optional<std::vector<Block>>({block_definitions::WATER}));
	};
	// Rust rejects a mapped tree at its source coordinate when land cover says
	// water; unlike a procedural slot, it must not relocate that tree onto land.
	if (!editor.owns(x, z) || blocked(x, z, true))
		return false;
	auto mapped_request = request;
	const auto caller_fit = request.fits;
	mapped_request.fits = {};
	auto selected =
			selector.pick_mapped(x, z, request.habitat, elevation_y, mapped_request);
	if (!selected)
		return false;
	// Match Rust's tile ownership rule: a mapped tree is never dropped just
	// because its deterministic schematic slot crosses an ownership seam.
	selected->x = x;
	selected->z = z;
	auto fits_at = [&](int ax, int az, std::size_t index,
						   unsigned preferred_rotation) -> std::optional<unsigned> {
		if (!editor.owns(ax, az) || editor.on_carriageway(ax, az) ||
				blocked(ax, az, true))
			return std::nullopt;
		const auto *schem = selector.schematic(index);
		if (!schem)
			return std::nullopt;
		for (unsigned k = 0; k < 4; ++k) {
			const unsigned rotation = (preferred_rotation + k) & 3u;
			if ((!caller_fit || caller_fit(index, rotation)) &&
					fits_tree_wood(editor, *schem, ax, az, rotation, y_offset, true))
				return rotation;
		}
		return std::nullopt;
	};
	if (const auto rotation =
					fits_at(x, z, selected->schematic_index, selected->rotation)) {
		selected->rotation = *rotation;
		return place_selected_schematic(
				editor, selector, y_offset, *selected, building_footprints, true);
	}
	// Rust only resolves the complete same-species and community fallback lists after
	// the mapped model fails at its exact location. Avoid allocating/scanning those
	// pools on the common path where the requested tree already fits.
	auto candidates = selector.pick_mapped_candidates(
			x, z, request.habitat, elevation_y, mapped_request);
	if (!candidates)
		return false;
	std::optional<std::vector<std::pair<int, int>>> offsets;
	auto nearby_offsets = [&]() -> const std::vector<std::pair<int, int>> & {
		if (!offsets) {
			const int reach =
					std::max(3, static_cast<int>(std::lround(8.0 * editor.scale())));
			offsets.emplace();
			for (int dx = -reach; dx <= reach; ++dx)
				for (int dz = -reach; dz <= reach; ++dz)
					if ((dx || dz) && dx * dx + dz * dz <= reach * reach)
						offsets->emplace_back(dx, dz);
			std::sort(
					offsets->begin(), offsets->end(), [&](const auto &a, const auto &b) {
						return std::tuple{a.first * a.first + a.second * a.second,
									   editor.surface_is_sealed(
											   x + a.first, z + a.second),
									   a.first, a.second} <
							   std::tuple{b.first * b.first + b.second * b.second,
									   editor.surface_is_sealed(
											   x + b.first, z + b.second),
									   b.first, b.second};
					});
		}
		return *offsets;
	};
	auto find_nearest =
			[&](const std::vector<std::size_t> &models) -> std::optional<SlotSelection> {
		for (const auto index : models)
			if (const auto rotation = fits_at(x, z, index, selected->rotation))
				return SlotSelection{x, z, index, *rotation};
		for (const auto &[dx, dz] : nearby_offsets())
			for (const auto index : models)
				if (const auto rotation =
								fits_at(x + dx, z + dz, index, selected->rotation))
					return SlotSelection{x + dx, z + dz, index, *rotation};
		return std::nullopt;
	};
	auto placed = [&]() -> std::optional<SlotSelection> {
		if (auto own = find_nearest(candidates->own_models))
			return own;
		return find_nearest(candidates->community_models);
	}();
	if (!placed)
		return false;
	return place_selected_schematic(
			editor, selector, y_offset, *placed, building_footprints, true);
}
bool place_selected_region_tree_for_cover(world_editor::WorldEditor &editor,
		const RegionSelector &selector, int x, int z, std::uint8_t cover, int elevation_y,
		SlotRequest request, const BuildingFootprintBitmap *building_footprints,
		const bridges::BridgeSurfaceMap *bridge_surface, bool allow_on_paved)
{
	if (!request.eco && editor.ground)
		request.eco = editor.ground->ecoregion_at(editor.ground_point(x, z));
	const auto habitat = ecoregion_habitat(editor, x, z, cover)
								 .value_or(habitat_for_land_cover(cover));
	return place_selected_region_tree(editor, selector, x, z, habitat, elevation_y,
			request, building_footprints, bridge_surface, allow_on_paved);
}
}
