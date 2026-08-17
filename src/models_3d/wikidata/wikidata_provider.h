#pragma once
#include "../provider.h"
#include "../file_provider.h"
namespace arnis::models_3d
{
class WikidataProvider : public ModelProvider
{
	FileModelProvider files_;

public:
	explicit WikidataProvider(std::filesystem::path root) : files_(std::move(root)) {}
	std::optional<ModelAsset> fetch(const std::string &qid) override;
};
}
