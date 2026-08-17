#include "voxelize.h"
namespace arnis::models_3d
{
std::vector<Voxel> voxelize_glb(
		const std::vector<std::uint8_t> &bytes, const WorldTransform &t)
{
	return voxelize_colored_triangles(glb_colored_triangles(bytes), t);
}
}
