#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "tree_library.h"

namespace arnis::trees
{
enum class TreePackMode
{
	Add,
	Replace
};

class PackDir
{
	struct Tree
	{
		std::string realm;
		std::string community;
		std::string species;
		std::string relative;
	};

	std::filesystem::path root_;
	TreePackMode mode_ = TreePackMode::Add;
	std::vector<Tree> trees_;
	std::vector<std::string> skipped_;

public:
	static PackDir scan(const std::filesystem::path &root, TreePackMode mode);
	void add_to_manifest(std::string_view realm, nlohmann::json &manifest) const;
	std::filesystem::path user_path(std::string_view relative) const;
	std::optional<TreeSize> tier_for_path(std::string_view relative) const;
	const std::filesystem::path &root() const { return root_; }
	TreePackMode mode() const { return mode_; }
	std::size_t found() const { return trees_.size(); }
	const std::vector<std::string> &skipped() const { return skipped_; }
	void report() const;
};
}
