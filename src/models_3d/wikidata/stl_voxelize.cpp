#include "stl.h"
#include "../../block_definitions.h"
namespace arnis::models_3d
{
std::vector<Voxel> voxelize_stl(
		const std::vector<Triangle> &triangles, const WorldTransform &transform)
{
	return voxelize_uniform_triangles(
			triangles, transform, block_definitions::STONE_BRICKS);
}
}
