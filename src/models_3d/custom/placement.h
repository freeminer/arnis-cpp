#pragma once
#include "../provider.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::models_3d::custom
{
bool place_custom(ModelProvider &, world_editor::WorldEditor &, const std::string &,
		float x, float y, float z, float scale = 1.0f, float yaw = 0.0f);
}
