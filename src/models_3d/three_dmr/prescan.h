#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace arnis
{
class ProcessedElement;
}
namespace arnis::models_3d::three_dmr
{
inline constexpr double ASSUMED_HALF_EXTENT_M = 384.0;
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
	std::uint64_t osm_id = 0, model_id = 0;
	int anchor_x = 0, anchor_z = 0;
	Bounds footprint{};
	double world_yaw_degrees = 0;
};
struct PrescanResult
{
	std::vector<Placement> placements;
	std::vector<std::pair<std::string, std::uint64_t>> suppressed;
};
PrescanResult prescan(const std::vector<ProcessedElement> &elements,
		double world_rotation,
		const std::vector<std::pair<std::string, std::uint64_t>> &already_suppressed = {});
std::vector<std::pair<int, int>> deferred_regions(const PrescanResult &, double scale);
}
