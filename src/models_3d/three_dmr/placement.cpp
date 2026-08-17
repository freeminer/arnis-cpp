#include "placement.h"
#include "../placement_pipeline.h"
namespace arnis::models_3d::three_dmr
{
bool place_model(ModelProvider &provider, world_editor::WorldEditor &editor,
		const std::string &id, float x, float y, float z, float scale, float yaw)
{
	auto asset = provider.fetch(id);
	return asset && place_model_asset(editor, *asset, x, y, z, scale, yaw);
}
}
