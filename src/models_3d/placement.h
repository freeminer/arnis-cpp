#pragma once
#include <array>
#include "model_asset.h"
namespace arnis::models_3d
{
struct Placement
{
	std::array<float, 3> anchor{};
	float width{}, height{}, depth{};
};
Placement placement_from_bounds(
		std::array<float, 3> min, std::array<float, 3> max, float x, float y, float z);
Placement normalized_placement(
		const ModelAsset &, float x, float y, float z, float target_height);
bool valid_bounds(const ModelAsset &);
}
