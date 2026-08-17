#include "placement.h"
#include "../placement_pipeline.h"
namespace arnis::models_3d::custom
{
bool place_custom(ModelProvider &provider, world_editor::WorldEditor &editor,
		const std::string &key, float x, float y, float z, float scale, float yaw)
{
	auto asset = provider.fetch(key);
	return asset && place_model_asset(editor, *asset, x, y, z, scale, yaw);
}
}
