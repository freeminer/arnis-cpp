#pragma once
#include "../provider.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::models_3d::three_dmr
{
bool place_model(ModelProvider &, world_editor::WorldEditor &,
		const std::string &model_id, float x, float y, float z, float scale = 1.0f,
		float yaw = 0.0f);
}
