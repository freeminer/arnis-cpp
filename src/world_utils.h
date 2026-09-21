#pragma once
#include <filesystem>
#include <cstdint>
#include <string>
#include <vector>
#include "coordinate_system/geographic/llbbox.h"
namespace arnis::world_utils
{
bool replace_file_atomically(
		const std::filesystem::path &, const std::vector<std::uint8_t> &);
std::filesystem::path get_bedrock_output_directory();
std::filesystem::path get_luanti_worlds_directory();
std::string sanitize_for_filename(const std::string &name);
std::string get_area_name_for_bedrock(const geographic::LLBBox &bbox);
std::pair<std::filesystem::path, std::string> build_bedrock_output(
		const geographic::LLBBox &bbox, const std::filesystem::path &output_dir);
}
