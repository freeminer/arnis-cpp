#include "placement_pipeline.h"
#include "placement.h"
#include <cmath>
namespace arnis::models_3d
{
bool place_model_asset(world_editor::WorldEditor &e, const ModelAsset &a, float x,
		float y, float z, float scale, float yaw)
{
	if (scale <= 0.0f || a.bytes.empty() || !valid_bounds(a))
		return false;
	if (!std::isfinite(yaw))
		yaw = 0.0f;
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
		return false;
	WorldTransform t(0, 1, {0, 0, 0}, {scale, scale, scale}, yaw, x, y, z);
	auto v = a.format == ModelFormat::GLB ? voxelize_glb(a.bytes, t)
										  : voxelize_stl(parse_binary_stl(a.bytes), t);
	place_voxels(e, v);
	return !v.empty();
}
}
