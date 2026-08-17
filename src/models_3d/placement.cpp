#include "placement.h"
#include <algorithm>
#include <cmath>
namespace arnis::models_3d
{
bool valid_bounds(const ModelAsset &a)
{
	for (int i = 0; i < 3; ++i)
		if (!std::isfinite(a.min[i]) || !std::isfinite(a.max[i]) || a.max[i] < a.min[i])
			return false;
	return true;
}
Placement placement_from_bounds(
		std::array<float, 3> a, std::array<float, 3> b, float x, float y, float z)
{
	return {{x - a[0], y - a[1], z - a[2]}, b[0] - a[0], b[1] - a[1], b[2] - a[2]};
}
Placement normalized_placement(
		const ModelAsset &a, float x, float y, float z, float target_height)
{
	const float h = std::max(0.001f, a.max[1] - a.min[1]);
	const float s = target_height / h;
	auto p = placement_from_bounds(a.min, a.max, x, y, z);
	p.width *= s;
	p.height *= s;
	p.depth *= s;
	return p;
}
}
