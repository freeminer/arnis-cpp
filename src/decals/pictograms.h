#pragma once
#include "../../arnis_adapter.h"
#include <filesystem>
#include <optional>
#include <string>
#include <vector>
namespace arnis::decals::pictograms
{
const std::vector<std::string> &names();
std::optional<std::filesystem::path> asset(const std::string &name,
		const std::filesystem::path &root = "assets/decorations/pictograms");
std::optional<std::string> business_kind(const tags_t &tags);
}
