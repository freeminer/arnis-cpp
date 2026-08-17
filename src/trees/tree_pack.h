#pragma once
#include <string>
#include <filesystem>
namespace arnis::trees
{
class RegionLibrary;
enum class Habitat
{
	Conifer,
	Wet,
	Lowland,
	Dry,
	Tropical
};
Habitat habitat_from_string(const std::string &name);
std::string realm_for_latlon(double lat, double lon);
class TreePackSource
{
	std::string realm_;
	std::filesystem::path root_;

public:
	explicit TreePackSource(std::string realm, std::filesystem::path root = {});
	const std::string &realm() const { return realm_; }
	std::string realm_file(const std::string &relative) const;
	std::string vanilla_file(const std::string &relative) const;
	std::filesystem::path realm_path(const std::string &relative) const;
	std::filesystem::path vanilla_path(const std::string &relative) const;
	bool has_realm_file(const std::string &relative) const;
	bool has_vanilla_file(const std::string &relative) const;
};
std::filesystem::path resolve_tree_asset(
		const TreePackSource &, const std::string &relative);
RegionLibrary load_region_library(
		double lat, double lon, const std::filesystem::path &root);
RegionLibrary load_combined_region_library(
		double lat, double lon, const std::filesystem::path &root);
}
