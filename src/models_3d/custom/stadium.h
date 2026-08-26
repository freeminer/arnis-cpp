#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace arnis
{
class ProcessedElement;
}
namespace arnis::models_3d::custom::stadium
{
struct Bounds
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
	bool contains(int x, int z) const
	{
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z;
	}
};
struct Placement
{
	std::uint64_t osm_id = 0;
	int anchor_x = 0, anchor_z = 0;
	Bounds footprint{};
	float long_m = 0, short_m = 0;
	double yaw_degrees = 0;
	float osm_height_m = 0;
	bool has_osm_height = false;
};
struct PrescanResult
{
	std::vector<Placement> placements;
	std::vector<std::pair<std::string, std::uint64_t>> suppressed;
};
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double scale,
		const std::vector<std::pair<std::string, std::uint64_t>> &already_suppressed = {});
// Rust fetches the single shared stadium GLB before it lets stadium geometry
// suppress roads/pitches/buildings.  Hosts can apply that decision without
// coupling discovery to a particular HTTP implementation.
void retain_fetchable(PrescanResult &, bool model_fetchable);
std::vector<std::pair<int, int>> deferred_regions(
		const std::vector<Placement> &placements, double scale);
}
