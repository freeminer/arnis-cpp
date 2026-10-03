#pragma once

#include <filesystem>

namespace arnis::assets
{
// Configure the directory containing Arnis' bundled data files before
// generation. Paths passed to path() may use the historical "assets/..."
// spelling; that prefix is removed beneath the configured root.
void set_base_directory(std::filesystem::path path);
std::filesystem::path base_directory();
std::filesystem::path path(std::filesystem::path relative_path);
} // namespace arnis::assets
