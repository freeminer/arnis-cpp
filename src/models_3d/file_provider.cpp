#include "file_provider.h"
namespace arnis::models_3d
{
std::optional<ModelAsset> FileModelProvider::fetch(const std::string &key)
{
	if (key.empty() || key.size() > 512 || key.front() == ' ' || key.back() == ' ')
		return {};
	const std::filesystem::path relative(key);
	if (relative.is_absolute() || key.find("..") != std::string::npos)
		return {};
	for (const auto &part : relative)
		if (part == ".." || part == ".")
			return {};
	const auto root = std::filesystem::weakly_canonical(root_);
	const auto candidate = std::filesystem::weakly_canonical(root_ / relative);
	if (root.empty() || candidate.empty())
		return {};
	auto root_it = root.begin(), path_it = candidate.begin();
	for (; root_it != root.end() && path_it != candidate.end(); ++root_it, ++path_it)
		if (*root_it != *path_it)
			return {};
	if (root_it != root.end() || !std::filesystem::is_regular_file(candidate))
		return {};
	try {
		return load_model_asset_auto(candidate);
	} catch (...) {
		return {};
	}
}
}
