#include "manifest.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace arnis::building_facades
{
namespace
{
constexpr unsigned SCHEMA_VERSION = 1;
bool safe_name(const std::string &name)
{
	if (name.empty() || name.size() > 128 || name.find("..") != std::string::npos)
		return false;
	return std::all_of(name.begin(), name.end(), [](unsigned char c) {
		return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
			   (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
	});
}

bool has_manifest_entry_schema(const nlohmann::json &entry)
{
	if (!entry.is_object() || !entry.contains("file") || !entry["file"].is_string() ||
			!entry.contains("categories") || !entry["categories"].is_array() ||
			!entry.contains("metres_wide") || !entry["metres_wide"].is_number() ||
			!entry.contains("metres_tall") || !entry["metres_tall"].is_number() ||
			!entry.contains("storeys") || !entry["storeys"].is_number_unsigned() ||
			entry["storeys"].get<std::uint64_t>() >
					std::numeric_limits<std::uint32_t>::max())
		return false;
	for (const auto &name : entry["categories"])
		if (!name.is_string())
			return false;
	for (const char *key : {"tiles_horizontally", "has_ground_floor"})
		if (entry.contains(key) && !entry[key].is_boolean())
			return false;
	return !entry.contains("note") || entry["note"].is_string();
}

std::optional<buildings::BuildingCategory> category(const std::string &name)
{
	using C = buildings::BuildingCategory;
	static const std::pair<const char *, C> values[] = {{"Residential", C::Residential},
			{"House", C::House}, {"Farm", C::Farm}, {"Commercial", C::Commercial},
			{"Office", C::Office}, {"Hotel", C::Hotel}, {"Industrial", C::Industrial},
			{"Warehouse", C::Warehouse}, {"School", C::School}, {"Hospital", C::Hospital},
			{"Religious", C::Religious}, {"TallBuilding", C::TallBuilding},
			{"GlassySkyscraper", C::GlassySkyscraper},
			{"GlassCornerSkyscraper", C::GlassCornerSkyscraper},
			{"GridSkyscraper", C::GridSkyscraper},
			{"ContemporarySkyscraper", C::ContemporarySkyscraper},
			{"ModernSkyscraper", C::ModernSkyscraper},
			{"MasonrySkyscraper", C::MasonrySkyscraper}, {"Historic", C::Historic},
			{"Tower", C::Tower}, {"Garage", C::Garage}, {"Shed", C::Shed},
			{"Greenhouse", C::Greenhouse}, {"Default", C::Default}};
	for (const auto &[text, value] : values)
		if (name == text)
			return value;
	return {};
}
} // namespace
const std::vector<std::size_t> &FacadeSet::in_category(
		buildings::BuildingCategory wanted) const
{
	static const std::vector<std::size_t> empty;
	const auto it = by_category_.find(wanted);
	return it == by_category_.end() ? empty : it->second;
}
std::optional<FacadeSet> FacadeSet::validate(std::vector<Entry> entries)
{
	entries.erase(std::remove_if(entries.begin(), entries.end(),
						  [](const Entry &e) {
							  return e.file.empty() || e.categories.empty() ||
									 !std::isfinite(e.metres_wide) ||
									 !std::isfinite(e.metres_tall) ||
									 e.metres_wide <= 0 || e.metres_tall <= 0 ||
									 e.storeys == 0;
						  }),
			entries.end());
	if (entries.empty())
		return {};
	FacadeSet set;
	set.entries_ = std::move(entries);
	for (std::size_t i = 0; i < set.entries_.size(); ++i) {
		auto &categories = set.entries_[i].categories;
		std::vector<buildings::BuildingCategory> unique;
		unique.reserve(categories.size());
		for (const auto category : categories)
			if (std::find(unique.begin(), unique.end(), category) == unique.end())
				unique.push_back(category);
		categories = std::move(unique);
		for (const auto category : categories)
			set.by_category_[category].push_back(i);
	}
	return set;
}

std::optional<FacadeSet> FacadeSet::load_directory(const std::filesystem::path &directory,
		std::vector<std::string> *rejected, std::string *error)
{
	std::ifstream input(directory / "manifest.json");
	if (!input) {
		if (error)
			*error = "no manifest.json";
		return {};
	}
	nlohmann::json manifest;
	try {
		input >> manifest;
	} catch (const std::exception &e) {
		if (error)
			*error = std::string("invalid manifest JSON: ") + e.what();
		return {};
	}
	if (!manifest.contains("version") || !manifest["version"].is_number_unsigned() ||
			manifest["version"].get<std::uint64_t>() == 0 ||
			manifest["version"].get<std::uint64_t>() >
					std::numeric_limits<std::uint32_t>::max() ||
			!manifest.contains("textures") || !manifest["textures"].is_array()) {
		if (error)
			*error = "manifest must contain a positive version and a textures array";
		return {};
	}
	const auto version =
			static_cast<std::uint32_t>(manifest["version"].get<std::uint64_t>());
	if (version > SCHEMA_VERSION)
		std::cerr << "Warning: preset facade manifest is version " << version
				  << " and this build knows version " << SCHEMA_VERSION
				  << "; reading it and ignoring anything newer.\n";
	if (!std::all_of(manifest["textures"].begin(), manifest["textures"].end(),
				[](const auto &entry) { return has_manifest_entry_schema(entry); })) {
		if (error)
			*error = "manifest contains a texture with invalid field types";
		return {};
	}
	std::vector<Entry> entries;
	for (const auto &raw : manifest["textures"]) {
		std::string entry_file;
		try {
			entry_file = raw.at("file").get<std::string>();
		} catch (const std::exception &) {
			// The schema check above rejects malformed entries as a group. Keep
			// this guard so rejection reporting remains robust if that check changes.
		}
		Entry entry;
		try {
			entry.file = entry_file;
			const std::filesystem::path file(entry.file);
			if (!safe_name(entry.file) || file.is_absolute() || file.has_parent_path()) {
				if (rejected)
					rejected->push_back(entry.file + ": not a plain file name");
				continue;
			}
			if (!std::filesystem::is_regular_file(directory / file)) {
				if (rejected)
					rejected->push_back(entry.file + ": no such file in the set");
				continue;
			}
			entry.metres_wide = raw.at("metres_wide").get<double>();
			if (!std::isfinite(entry.metres_wide) || entry.metres_wide <= 0.0)
				throw std::runtime_error("metres_wide must be positive");
			entry.metres_tall = raw.at("metres_tall").get<double>();
			if (!std::isfinite(entry.metres_tall) || entry.metres_tall <= 0.0)
				throw std::runtime_error("metres_tall must be positive");
			const auto &storeys = raw.at("storeys");
			if (!storeys.is_number_unsigned() ||
					storeys.get<std::uint64_t>() >
							std::numeric_limits<std::uint32_t>::max())
				throw std::runtime_error("storeys must be an unsigned 32-bit integer");
			entry.storeys = static_cast<unsigned>(storeys.get<std::uint64_t>());
			if (entry.storeys == 0)
				throw std::runtime_error("storeys must be at least 1");
			for (const auto &name : raw.at("categories")) {
				const auto category_name = name.get<std::string>();
				auto value = category(category_name);
				if (!value)
					throw std::runtime_error("unknown category " + category_name);
				if (std::find(entry.categories.begin(), entry.categories.end(), *value) ==
						entry.categories.end())
					entry.categories.push_back(*value);
			}
			if (entry.categories.empty())
				throw std::runtime_error("no categories");
			entry.tiles_horizontally = raw.value("tiles_horizontally", false);
			entry.has_ground_floor = raw.value("has_ground_floor", false);
			entries.push_back(std::move(entry));
		} catch (const std::exception &e) {
			if (rejected)
				rejected->push_back(
						entry_file.empty()
								? std::string("invalid manifest entry: ") + e.what()
								: entry_file + ": " + e.what());
		}
	}
	auto set = validate(std::move(entries));
	if (!set && error)
		*error = "no usable textures";
	return set;
}
}
