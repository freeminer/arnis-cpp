#pragma once

#include <cstddef>
#include <utility>
#include <vector>
#include <string>
#include <optional>
#include <tuple>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "../osm_parser.h"

namespace arnis::structures
{
struct StructureAsset
{
	const char *name;
	const char *schematic;
	int width = 0, height = 0, length = 0;
	unsigned default_rotation = 0;
};
const std::vector<StructureAsset> &rust_structure_assets();
const StructureAsset *find_structure_asset(const std::string &name);
const StructureAsset *find_structure_asset_ci(const std::string &name);
const StructureAsset *find_structure_asset_alias(const std::string &name);
std::string canonical_structure_name(const std::string &name);
std::filesystem::path structure_asset_path(const WorldEditor &, const std::string &name);
bool structure_asset_available(const WorldEditor &, const std::string &name);
std::vector<std::string> missing_structure_assets(const WorldEditor &);
std::optional<std::tuple<int, int, int>> structure_dimensions(
		const WorldEditor &, const std::string &name);
std::vector<std::pair<std::string, std::tuple<int, int, int>>>
available_structure_dimensions(const WorldEditor &);
std::vector<std::string> invalid_structure_assets(const WorldEditor &);
struct StructureAudit
{
	std::size_t total = 0, available = 0, missing = 0, invalid = 0;
};
StructureAudit audit_structures(const WorldEditor &);
std::string format_structure_audit(const StructureAudit &);
bool audit_usable(const StructureAudit &);
std::string structure_registry_fingerprint();
std::string structure_registry_manifest();
bool structure_registry_valid();
bool structure_has_procedural_generator(const std::string &name);
std::vector<StructureAsset> procedural_structure_assets();
std::vector<StructureAsset> schematic_only_structure_assets();
enum class StructureBackend
{
	Procedural,
	Schematic,
	Missing
};
StructureBackend structure_backend(const WorldEditor &, const std::string &name);
struct PlacementResult
{
	bool recognized = false;
	bool placed = false;
	const char *asset = nullptr;
};
std::vector<PlacementResult> place_structures_checked(WorldEditor &,
		const std::vector<std::string> &, int x, int z, unsigned rotation = 0,
		std::size_t area_cells = 0);
PlacementResult place_structure_auto(WorldEditor &, const std::string &, int x, int z,
		unsigned rotation = 0, std::size_t area_cells = 0);
std::vector<PlacementResult> place_structures_auto(WorldEditor &,
		const std::vector<std::string> &, int x, int z, unsigned rotation = 0,
		std::size_t area_cells = 0);
struct PlacementStats
{
	std::size_t recognized = 0, placed = 0, missing = 0;
};
struct CheckedPlacements
{
	PlacementStats stats;
	StructureAudit audit;
	std::vector<PlacementResult> results;
};
CheckedPlacements place_structures_with_audit(WorldEditor &,
		const std::vector<std::string> &, int x, int z, unsigned rotation = 0,
		std::size_t area_cells = 0);
PlacementStats placement_stats(const std::vector<PlacementResult> &);
std::vector<std::string> normalize_structure_names(const std::vector<std::string> &);
std::vector<std::string> valid_structure_names();
PlacementResult place_named_structure_result(
		WorldEditor &, const std::string &, int x, int z, std::size_t area_cells = 0);
PlacementResult place_named_structure_rotated(WorldEditor &, const std::string &, int x,
		int z, unsigned rotation, std::size_t area_cells = 0);
bool place_named_structure(
		WorldEditor &, const std::string &name, int x, int z, std::size_t area_cells = 0);

namespace fountain
{
void place(WorldEditor &editor, int x, int z, std::size_t area_cells);
}

namespace playground
{
void scatter_playgrounds(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace lighthouse
{
void place(WorldEditor &editor, int x, int z);
}

namespace crane
{
void maybe_place_crane(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace excavator
{
void scatter_excavators(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace tractor
{
void maybe_place_tractor(
		WorldEditor &editor, const std::vector<std::pair<int, int>> &cells);
}

namespace boat
{
void scatter_boats(WorldEditor &editor, int min_x, int min_z, int max_x, int max_z);
}

namespace car
{
void maybe_place_car(WorldEditor &editor, int cx, int cz, uint8_t rot_base);
}

namespace helicopter
{
void maybe_place_helicopter(WorldEditor &editor, int cx, int cz);
}

namespace starship
{
inline constexpr uint64_t STARBASE_PAD2_INNER_RING_WAY = 1486752423ULL;
void place_on_launch_mount(WorldEditor &editor, const ProcessedWay &ring);
}

namespace tombstone
{
void maybe_place(WorldEditor &editor, int x, int z, const RoadMaskBitmap &road_mask);
}

namespace windturbine
{
void place(WorldEditor &editor, int x, int z);
}

}
