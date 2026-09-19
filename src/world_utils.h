#pragma once
#include <filesystem>
#include <cstdint>
#include <string>
#include <vector>
namespace arnis::world_utils
{
bool replace_file_atomically(
		const std::filesystem::path &, const std::vector<std::uint8_t> &);
std::filesystem::path get_bedrock_output_directory();
std::filesystem::path get_luanti_worlds_directory();
std::string sanitize_for_filename(const std::string &name);
}
