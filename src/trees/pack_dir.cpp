#include "pack_dir.h"

#include "schematic.h"
#include "../assets_root.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace arnis::trees
{
namespace
{
constexpr std::uintmax_t MAX_FILE_BYTES = 8u << 20;
constexpr int MAX_SIDE = 64;
constexpr int MAX_HEIGHT = 256;
constexpr std::string_view USER_PREFIX = "user:";

std::string lower(std::string value)
{
	for (auto &c : value)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return value;
}

std::optional<TreeSize> tier_named(std::string_view name)
{
	const auto value = lower(std::string(name));
	if (value == "small")
		return TreeSize::Small;
	if (value == "medium")
		return TreeSize::Medium;
	if (value == "big")
		return TreeSize::Big;
	if (value == "tall")
		return TreeSize::Tall;
	if (value == "giant")
		return TreeSize::Giant;
	return std::nullopt;
}

bool known_realm(std::string_view name)
{
	static constexpr std::array<std::string_view, 10> realms{
			"afr", "asn", "aus", "ena", "eur", "fl", "ind", "sam", "wna", "vanilla-plus"};
	return std::any_of(realms.begin(), realms.end(),
			[&](auto realm) { return lower(std::string(name)) == realm; });
}

std::optional<nlohmann::json> read_manifest(const std::string &realm)
{
	std::ifstream input(
			assets::path(std::filesystem::path("tree-packs") / realm / "region.json"));
	if (!input)
		return std::nullopt;
	try {
		nlohmann::json manifest;
		input >> manifest;
		return manifest;
	} catch (...) {
		return std::nullopt;
	}
}

bool community_exists(
		const nlohmann::json &manifest, std::string_view name, std::string *canonical)
{
	if (!manifest.is_object() || !manifest.contains("communities") ||
			!manifest["communities"].is_array())
		return false;
	for (const auto &community : manifest["communities"]) {
		if (!community.is_object() || !community.contains("name") ||
				!community["name"].is_string())
			continue;
		const auto candidate = community["name"].get<std::string>();
		if (lower(candidate) == lower(std::string(name))) {
			if (canonical)
				*canonical = candidate;
			return true;
		}
	}
	return false;
}

std::string relative_string(
		const std::filesystem::path &path, const std::filesystem::path &root)
{
	std::error_code ec;
	auto relative = std::filesystem::relative(path, root, ec);
	return (ec ? path : relative).generic_string();
}
}

PackDir PackDir::scan(const std::filesystem::path &root, TreePackMode mode)
{
	PackDir result;
	result.root_ = root;
	result.mode_ = mode;
	std::error_code ec;
	if (!std::filesystem::is_directory(root, ec)) {
		result.skipped_.push_back(root.string() + ": no such folder");
		return result;
	}

	std::vector<std::filesystem::path> files;
	for (std::filesystem::recursive_directory_iterator it(
				 root, std::filesystem::directory_options::skip_permission_denied, ec),
			end;
			it != end; it.increment(ec)) {
		if (ec) {
			ec.clear();
			continue;
		}
		if (it->is_regular_file(ec))
			files.push_back(it->path());
	}
	std::sort(files.begin(), files.end(), [&](const auto &a, const auto &b) {
		return relative_string(a, root) < relative_string(b, root);
	});

	std::unordered_map<std::string, std::optional<nlohmann::json>> manifests;
	for (const auto &file : files) {
		const auto ext = lower(file.extension().string());
		if (ext != ".schem" && ext != ".schematic")
			continue;
		const auto relative = std::filesystem::relative(file, root, ec);
		if (ec) {
			ec.clear();
			result.skipped_.push_back(
					relative_string(file, root) + ": cannot resolve path");
			continue;
		}
		std::vector<std::string> parts;
		for (const auto &part : relative)
			parts.push_back(part.string());
		const auto rel = relative.generic_string();
		if (parts.size() < 4 || parts.size() > 5) {
			result.skipped_.push_back(
					rel + ": not in a realm/community/tree type folder");
			continue;
		}
		if (!known_realm(parts[0])) {
			result.skipped_.push_back(rel + ": unknown realm");
			continue;
		}
		if (parts.size() == 5 && !tier_named(parts[3])) {
			result.skipped_.push_back(rel + ": " + parts[3] + " is not a size folder");
			continue;
		}
		const auto realm = lower(parts[0]);
		auto mit = manifests.find(realm);
		if (mit == manifests.end())
			mit = manifests.emplace(realm, read_manifest(realm)).first;
		if (!mit->second) {
			result.skipped_.push_back(rel + ": bundled realm manifest unavailable");
			continue;
		}
		std::string community;
		if (!community_exists(*mit->second, parts[1], &community)) {
			result.skipped_.push_back(
					rel + ": " + parts[1] + " is not a community of " + realm);
			continue;
		}
		const auto size = std::filesystem::file_size(file, ec);
		if (ec || size > MAX_FILE_BYTES) {
			ec.clear();
			result.skipped_.push_back(rel + ": over the 8 MB limit or unreadable");
			continue;
		}
		try {
			const auto schem = load_schem(file);
			if (schem.width > MAX_SIDE || schem.length > MAX_SIDE ||
					schem.height > MAX_HEIGHT) {
				result.skipped_.push_back(
						rel + ": wider than 64 blocks or taller than 256");
				continue;
			}
			if (!has_leaves(schem)) {
				result.skipped_.push_back(rel + ": no leaves");
				continue;
			}
		} catch (const std::exception &e) {
			result.skipped_.push_back(rel + ": " + e.what());
			continue;
		}
		result.trees_.push_back({realm, std::move(community), parts[2], rel});
	}
	return result;
}

void PackDir::add_to_manifest(std::string_view realm, nlohmann::json &manifest) const
{
	std::vector<const Tree *> matching;
	for (const auto &tree : trees_)
		if (tree.realm == realm)
			matching.push_back(&tree);
	if (matching.empty() || !manifest.is_object() || !manifest.contains("communities") ||
			!manifest["communities"].is_array())
		return;
	if (mode_ == TreePackMode::Replace)
		for (auto &community : manifest["communities"])
			if (community.is_object())
				community["species"] = nlohmann::json::array();
	for (const auto *tree : matching) {
		auto community = std::find_if(manifest["communities"].begin(),
				manifest["communities"].end(), [&](const auto &candidate) {
					return candidate.is_object() &&
						   candidate.value("name", std::string{}) == tree->community;
				});
		if (community == manifest["communities"].end())
			continue;
		auto &species = (*community)["species"];
		if (!species.is_array())
			species = nlohmann::json::array();
		auto entry =
				std::find_if(species.begin(), species.end(), [&](const auto &candidate) {
					return candidate.is_object() &&
						   candidate.value("name", std::string{}) == tree->species;
				});
		const auto path = std::string(USER_PREFIX) + tree->relative;
		if (entry == species.end()) {
			species.push_back(
					{{"name", tree->species}, {"w1", nlohmann::json::array({path})}});
		} else {
			auto &variants = (*entry)["w1"];
			if (!variants.is_array())
				variants = nlohmann::json::array();
			variants.push_back(path);
		}
	}
}

std::filesystem::path PackDir::user_path(std::string_view relative) const
{
	return root_ / std::filesystem::path(relative);
}

std::optional<TreeSize> PackDir::tier_for_path(std::string_view relative) const
{
	const std::filesystem::path path(relative);
	return tier_named(path.parent_path().filename().string());
}

void PackDir::report() const
{
	for (const auto &line : skipped_)
		std::cerr << "Warning: tree-pack-dir: skipped " << line << '\n';
	std::cout << "  tree-pack-dir: " << found() << " custom trees (" << skipped_.size()
			  << " skipped, " << (mode_ == TreePackMode::Replace ? "replace" : "add")
			  << ")\n";
}
}
