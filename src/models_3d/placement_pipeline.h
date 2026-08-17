#pragma once
#include "provider.h"
#include "voxelize.h"
#include "wikidata/stl.h"
namespace arnis::models_3d
{
bool place_model_asset(world_editor::WorldEditor &, const ModelAsset &, float x, float y,
		float z, float scale = 1.0f, float yaw = 0.0f);
}
