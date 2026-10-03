#pragma once

#include <filesystem>
#include <string>

namespace arnis::cache
{
// Configure the shared base directory before starting generation. An empty
// path restores the platform default. Provider-specific data stays in named
// subdirectories beneath this base.
void set_base_directory(std::filesystem::path path);
std::filesystem::path base_directory();
std::filesystem::path tile_cache_root();
std::filesystem::path provider_cache_root(const std::string &provider);
std::filesystem::path facade_cache_root();
} // namespace arnis::cache
