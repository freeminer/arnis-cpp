#pragma once
#include "provider.h"
#include <filesystem>
namespace arnis::models_3d
{
class FileModelProvider : public ModelProvider
{
	std::filesystem::path root_;

public:
	explicit FileModelProvider(std::filesystem::path root) : root_(std::move(root)) {}
	std::optional<ModelAsset> fetch(const std::string &key) override;
};
}
