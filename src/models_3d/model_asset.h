#pragma once
#include <cstdint>
#include <filesystem>
#include <vector>
#include <array>
namespace arnis::models_3d
{
enum class ModelFormat
{
	GLB,
	BinarySTL
};
struct ModelAsset
{
	ModelFormat format;
	std::vector<std::uint8_t> bytes;
	std::array<float, 3> min{}, max{};
};
ModelAsset load_model_asset(const std::filesystem::path &, ModelFormat);
ModelAsset load_model_asset_auto(const std::filesystem::path &);
}
