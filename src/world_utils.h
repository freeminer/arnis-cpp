#pragma once
#include <filesystem>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>
#include "coordinate_system/geographic/llbbox.h"
namespace arnis::world_utils
{
inline constexpr std::int32_t DIMENSION_FOLDERS_DATA_VERSION = 4772;

struct WorldLayout
{
	enum class Kind
	{
		Legacy,
		Dimensions
	};

	Kind kind{Kind::Legacy};
	static WorldLayout of(const std::filesystem::path &world);
	std::filesystem::path overworld_dir(const std::filesystem::path &world) const;
	std::filesystem::path maps_dir(const std::filesystem::path &world) const;
};

std::optional<std::int32_t> level_data_version(const std::filesystem::path &world);
bool replace_file_atomically(
		const std::filesystem::path &, const std::vector<std::uint8_t> &);
std::filesystem::path get_bedrock_output_directory();
std::filesystem::path get_luanti_worlds_directory();
std::string sanitize_for_filename(const std::string &name);
std::string world_folder_name(const std::string &raw);
// Select a Rust-compatible unique world directory name under base_path.
// Empty/invalid custom names use the automatic "Arnis World N" scheme.
std::string unique_world_folder_name(
		const std::filesystem::path &base_path, const std::string &custom_name = {});
std::string get_area_name_for_bedrock(const geographic::LLBBox &bbox);
std::pair<std::filesystem::path, std::string> build_bedrock_output(
		const geographic::LLBBox &bbox, const std::filesystem::path &output_dir);
}
