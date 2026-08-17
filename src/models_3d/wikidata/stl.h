#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include "../voxelize.h"
namespace arnis::models_3d
{
using Triangle = std::array<std::array<float, 3>, 3>;
std::vector<Triangle> parse_binary_stl(const std::vector<std::uint8_t> &);
std::pair<std::array<float, 3>, std::array<float, 3>> stl_bbox(
		const std::vector<Triangle> &);
std::vector<Voxel> voxelize_stl(const std::vector<Triangle> &, const WorldTransform &);
}
