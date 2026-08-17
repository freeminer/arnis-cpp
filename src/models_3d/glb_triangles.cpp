#include "voxelize.h"

namespace arnis::models_3d
{
std::vector<std::array<std::array<float, 3>, 3>> glb_triangles(
		const std::vector<std::uint8_t> &bytes)
{
	// Keep the uncoloured API, but source geometry from the same scene-aware
	// reader used by voxelization.  This preserves default-scene selection,
	// node hierarchy transforms, indexed primitives, and accessor strides.
	const auto colored = glb_colored_triangles(bytes);
	std::vector<std::array<std::array<float, 3>, 3>> out;
	out.reserve(colored.size());
	for (const auto &triangle : colored)
		out.push_back(triangle.vertices);
	return out;
}
}
