#pragma once
#include <string>
#include <filesystem>
#include <optional>
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
// Rust tree-pack selection excludes palm assets outside subtropical latitudes.
bool exclude_palms_for_latitude(double lat);
class TreePackSource
{
	std::string realm_;
	std::filesystem::path root_;

public:
	explicit TreePackSource(std::string realm, std::filesystem::path root = {});
	// Rust's embedded() source resolves against the bundled asset directory;
	// C++ keeps the same API while allowing distributions to override the root.
	static TreePackSource embedded(
			const std::string &realm, std::filesystem::path root = {});
	const std::string &realm() const { return realm_; }
	std::string realm_file(const std::string &relative) const;
	std::string vanilla_file(const std::string &relative) const;
	std::filesystem::path realm_path(const std::string &relative) const;
	std::filesystem::path vanilla_path(const std::string &relative) const;
	std::optional<std::filesystem::path> realm_manifest() const;
	std::optional<std::filesystem::path> vanilla_manifest() const;
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
