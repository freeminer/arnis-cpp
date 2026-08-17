#pragma once
#include <filesystem>
#include <string>
namespace arnis::world_utils {
std::filesystem::path get_bedrock_output_directory();
std::filesystem::path get_luanti_worlds_directory();
std::string sanitize_for_filename(const std::string &name);
}
