#pragma once

#include <cstdint>
#include <vector>

namespace arnis
{
class ProcessedElement;
}

namespace arnis::models_3d::custom::plane
{

enum class Kind
{
	Parked,
	Ascending
};
struct Bounds
{
	int min_x = 0, min_z = 0, max_x = 0, max_z = 0;
};
struct Placement
{
	std::int64_t representative_id = 0;
	Kind kind = Kind::Parked;
	int anchor_x = 0, anchor_z = 0, elevation_blocks = 0;
	double yaw_degrees = 0.0, pitch_degrees = 0.0;
	Bounds footprint{};
};

// Library-only prescan: discovers, joins and deterministically places plane
// archetypes. Model downloading/voxelization stays in the caller pipeline.
std::vector<Placement> prescan(
		const std::vector<ProcessedElement> &elements, double scale);
std::vector<std::pair<int, int>> deferred_regions(
		const std::vector<Placement> &placements, double scale);

}
