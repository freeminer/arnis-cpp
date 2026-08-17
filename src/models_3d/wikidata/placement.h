#pragma once
#include "../provider.h"
#include "../three_dmr/client.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::models_3d::wikidata
{
bool place_wikidata_model(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float y, float z, float scale = 1.0f,
		float yaw = 0.0f);
float scale_for_height(const ModelAsset &, float target_height);
bool place_wikidata_model_height(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float y, float z, float target_height,
		float yaw = 0.0f);
bool place_wikidata_model_with_info(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, const three_dmr::ModelInfo &, float x, float y, float z,
		float scale = 1.0f, float yaw = 0.0f);
struct PlacementAttribution
{
	bool placed = false;
	std::string qid, label, artist, license, license_url;
};
PlacementAttribution place_wikidata_attributed(ModelProvider &,
		world_editor::WorldEditor &, const std::string &, float x, float y, float z,
		float scale = 1.0f, float yaw = 0.0f);
}
