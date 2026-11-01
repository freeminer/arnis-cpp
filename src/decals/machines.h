#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis::decals::machines
{
const std::vector<std::string> &names();
bool is_small(std::string_view name);
std::pair<unsigned, unsigned> tiles(std::string_view name);
std::optional<std::filesystem::path> asset(
		const std::string &name, const std::filesystem::path &root = {});
} // namespace arnis::decals::machines
