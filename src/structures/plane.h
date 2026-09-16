#pragma once

#include "../../../arnis_adapter.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <unordered_map>
#include <vector>

namespace arnis::structures::plane
{
inline constexpr double PLANE_LENGTH_BLOCKS = 40.0;
inline constexpr double NOSE_GEAR_OFFSET_BLOCKS = 6.0;
inline constexpr double PLANE_WINGSPAN_BLOCKS = 45.0;
inline constexpr double MIN_PLANE_SEPARATION_BLOCKS = 55.0;
inline constexpr double STAND_OCCUPANCY_PROBABILITY = 0.55;
inline constexpr double RUNWAY_PARK_PROBABILITY = 0.4;
inline constexpr double TAXIWAY_PARK_PROBABILITY = 0.15;
inline constexpr double PARKED_MIN_LENGTH_M = 120.0;
inline constexpr double ASCENDING_MIN_LENGTH_M = 1500.0;
inline constexpr double ASCENDING_PITCH_DEG = 12.0;
inline constexpr int PLANE_REACH_BLOCKS = 27;
inline constexpr std::size_t MAX_PLANES = 2000;

enum class AerowayKind
{
	Runway,
	Taxiway
};
struct Segment
{
	std::uint64_t way_id = 0;
	std::uint64_t first_node = 0, last_node = 0;
	AerowayKind kind = AerowayKind::Runway;
	std::vector<std::pair<double, double>> points;
	double angle = 0.0;
};
struct Stand
{
	std::uint64_t way_id = 0;
	int x = 0, z = 0;
	double direction_x = 1.0, direction_z = 0.0;
};

// Rust plane.rs::AerowayStrip equivalent.  Coordinates remain in source
// world units; conversion to block scale is deliberately deferred to the
// placement stage.
struct AerowayStrip
{
	std::uint64_t rep_id = 0;
	AerowayKind kind = AerowayKind::Runway;
	double centroid_x = 0.0, centroid_z = 0.0;
	double direction_x = 1.0, direction_z = 0.0;
	double length_blocks = 0.0, perpendicular_blocks = 0.0;
	double min_along = 0.0, max_along = 0.0;
};
enum class PlaneKind
{
	Parked,
	Ascending
};
struct Footprint
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
};
struct Placement
{
	std::uint64_t representative_id = 0;
	PlaneKind kind = PlaneKind::Parked;
	int anchor_x = 0, anchor_z = 0;
	double yaw_degrees = 0.0, pitch_degrees = 0.0;
	int elevation_blocks = 0;
	Footprint footprint{};
};
Footprint plane_footprint(int anchor_x, int anchor_z, double yaw_degrees);
Placement make_placement(std::uint64_t representative_id, PlaneKind kind, int anchor_x,
		int anchor_z, double yaw_degrees, double pitch_degrees, int elevation_blocks);
std::vector<std::pair<int, int>> deferred_region_keys(
		const std::vector<Placement> &placements);
void cap_placements(std::vector<Placement> &placements);
void sort_placements(std::vector<Placement> &placements);
bool placement_separated(
		const Placement &candidate, const std::vector<Placement> &accepted);
void thin_by_spacing(std::vector<Placement> &placements);
void finalize_placements(std::vector<Placement> &placements);
void filter_and_finalize_strips(std::vector<AerowayStrip> &strips, double scale);
bool make_centerline_placement(const AerowayStrip &strip, double scale, Placement &out);
bool make_ascending_placement(const AerowayStrip &strip, double scale, Placement &out);
std::vector<Placement> build_strip_placements(
		const std::vector<AerowayStrip> &strips, double scale);
std::vector<Placement> prescan_placements(
		const std::vector<ProcessedElement> &elements, double scale);
bool place_plane_placement(WorldEditor &editor, const Placement &placement);
std::size_t place_plane_placements(
		WorldEditor &editor, const std::vector<Placement> &placements);

bool collinear(double a, double b, double tolerance = 0.349);
bool extract_aeroway_segment(const ProcessedElement &element, Segment &out);
bool extract_parking_stand(const ProcessedElement &element, Stand &out);
std::vector<Stand> collect_parking_stands(const std::vector<ProcessedElement> &elements);
bool make_stand_placement(const Stand &stand, Placement &out);
std::vector<Segment> collect_aeroway_segments(
		const std::vector<ProcessedElement> &elements);
bool principal_geometry(const std::vector<std::pair<double, double>> &points,
		std::uint64_t rep_id, AerowayKind kind, AerowayStrip &out);
bool eligible_strip(const AerowayStrip &strip, double scale);
std::vector<AerowayStrip> merge_aeroways(const std::vector<Segment> &segments);
std::vector<AerowayStrip> collect_aeroway_strips(
		const std::vector<ProcessedElement> &elements);
}
