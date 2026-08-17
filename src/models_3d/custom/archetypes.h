#pragma once
#include <cstdint>
#include <vector>
#include <utility>
#include <optional>
#include "../provider.h"
#include "../../../../arnis_adapter.h"
namespace arnis::models_3d::custom
{
inline constexpr const char *PLANE_MODEL_URL =
		"https://arnismc.com/assets/3dmodels/plane.glb";
inline constexpr const char *PLANE_CACHE_FILE = "plane.glb";
inline constexpr const char *STADIUM_MODEL_URL =
		"https://arnismc.com/assets/3dmodels/stadium.glb";
inline constexpr const char *STADIUM_CACHE_FILE = "stadium.glb";
inline constexpr const char *plane_model_key()
{
	return PLANE_CACHE_FILE;
}
inline constexpr const char *stadium_model_key()
{
	return STADIUM_CACHE_FILE;
}
struct ArchetypeModels
{
	std::optional<ModelAsset> plane, stadium;
};
ArchetypeModels fetch_archetype_models(
		ModelProvider &, bool need_plane, bool need_stadium);
inline constexpr double PLANE_LENGTH_M = 90.0, ASCENDING_PITCH_DEG = 12.0,
						ASCENDING_MIN_LENGTH_M = 1500.0;
inline constexpr double ASCENDING_ELEV_FACTOR = 0.45, ASCENDING_EXTRA_ELEV_M = 20.0;
inline constexpr double RUNWAY_PARK_PROBABILITY = 0.40, TAXIWAY_PARK_PROBABILITY = 0.15;
inline constexpr double PARKED_MIN_LENGTH_M = 120.0, MAX_AEROWAY_LENGTH_M = 8000.0;
inline constexpr double COLLINEAR_TOL_RAD = 0.349, RUNWAY_MAX_PERP_RATIO = 0.5,
						TAXIWAY_MAX_PERP_RATIO = 0.12;
inline constexpr float STADIUM_DEFAULT_HEIGHT_M = 28.0f, STADIUM_HEIGHT_MULTIPLIER = 1.5f;
bool plane_strip_eligible(double length_m, bool runway);
bool stadium_footprint_eligible(
		double area_m2, double long_m, double short_m, bool has_inner_stadium);
struct ModelFootprint
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
	bool contains(int x, int z) const
	{
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
	}
};
double normalize_yaw_degrees(double yaw);
ModelFootprint rotated_footprint(int anchor_x, int anchor_z, double long_blocks,
		double short_blocks, double yaw_degrees);
int ascending_elevation_blocks(double runway_length_m, double scale, double ground_y);
float stadium_target_height_m(float osm_height_m, bool has_height);
double plane_model_scale(const ModelAsset &, double target_length_blocks);
double stadium_model_scale(
		const ModelAsset &, double target_long_blocks, double target_short_blocks);
double way_length_blocks(const ProcessedWay &, double scale);
double way_bearing_degrees(const ProcessedWay &);
float parse_height_m(const std::string &value, bool &valid);
double stadium_yaw_from_extents(double extent_x, double extent_z);
bool valid_footprint(const ModelFootprint &);
bool deterministic_model_chance(
		std::uint64_t id, double probability, std::uint64_t salt = 0);
struct PlanePlacement
{
	std::uint64_t representative_id = 0;
	bool ascending = false;
	int anchor_x = 0, anchor_z = 0, elevation_blocks = 0;
	double yaw_degrees = 0, pitch_degrees = 0;
	ModelFootprint footprint;
};
inline constexpr bool plane_is_ascending(const PlanePlacement &p)
{
	return p.ascending;
}
inline constexpr bool plane_is_parked(const PlanePlacement &p)
{
	return !p.ascending;
}
struct StadiumPlacement
{
	std::uint64_t osm_id = 0;
	int anchor_x = 0, anchor_z = 0;
	float long_m = 0, short_m = 0;
	double yaw_degrees = 0;
	float osm_height_m = 0;
	bool has_height = false;
	ModelFootprint footprint;
};
struct ArchetypePrescan
{
	std::vector<PlanePlacement> planes;
	std::vector<StadiumPlacement> stadiums;
	std::vector<std::uint64_t> suppressed_ids;
	std::vector<std::pair<int, int>> deferred_regions;
};
enum class ArchetypeKind
{
	Plane,
	Stadium
};
struct PlacementRef
{
	ArchetypeKind kind;
	std::size_t index;
};
struct PlacementTransform
{
	int anchor_x = 0, anchor_z = 0, elevation = 0;
	double yaw = 0, pitch = 0;
};
PlacementTransform transform_for(const ArchetypePrescan &, PlacementRef);
const char *model_key_for(PlacementRef);
const ModelFootprint &footprint_for(const ArchetypePrescan &, PlacementRef);
bool valid_placement(const ArchetypePrescan &, PlacementRef);
std::vector<PlacementRef> valid_placement_refs(const ArchetypePrescan &);
double model_scale_for(const ArchetypePrescan &, PlacementRef, const ArchetypeModels &,
		double world_scale);
struct PlacementPlan
{
	PlacementRef ref{};
	PlacementTransform transform{};
	const char *model_key = nullptr;
	double scale = 0;
	ModelFootprint footprint{};
};
std::vector<PlacementPlan> build_placement_plans(
		const ArchetypePrescan &, const ArchetypeModels &, double world_scale);
std::vector<std::pair<int, int>> plan_regions(
		const PlacementPlan &, double world_scale, int region_size = 16);
bool plan_contains(const PlacementPlan &, int x, int z);
std::vector<PlacementRef> placement_refs(const ArchetypePrescan &);
void apply_model_availability(ArchetypePrescan &, const ArchetypeModels &,
		const std::vector<ProcessedElement> &, double scale);
bool suppressed_at(const ArchetypePrescan &, int x, int z);
std::vector<std::pair<int, int>> placement_anchors(const ArchetypePrescan &);
inline std::size_t placement_count(const ArchetypePrescan &p)
{
	return p.planes.size() + p.stadiums.size();
}
void filter_unavailable(ArchetypePrescan &, bool plane_available, bool stadium_available);
void rebuild_deferred_regions(ArchetypePrescan &, double scale);
void rebuild_suppression(ArchetypePrescan &, const std::vector<ProcessedElement> &);
void finalize_prescan(
		ArchetypePrescan &, const std::vector<ProcessedElement> &, double scale);
ArchetypePrescan prescan_archetypes(const std::vector<ProcessedElement> &, double scale);
int deferred_radius(const PlanePlacement &, double scale);
int deferred_radius(const StadiumPlacement &, double scale);
std::vector<std::pair<int, int>> deferred_region_keys(
		int x, int z, int radius, int region_size = 16);
std::vector<StadiumPlacement> prescan_stadiums(
		const std::vector<ProcessedElement> &elements, double scale);
std::vector<PlanePlacement> prescan_planes(
		const std::vector<ProcessedElement> &elements, double scale);
std::vector<std::uint64_t> suppressed_element_ids(
		const std::vector<StadiumPlacement> &placements,
		const std::vector<ProcessedElement> &elements);
std::optional<ModelAsset> fetch_plane_model(
		ModelProvider &, const std::string &key = PLANE_CACHE_FILE);
std::optional<ModelAsset> fetch_stadium_model(
		ModelProvider &, const std::string &key = STADIUM_CACHE_FILE);
}
