#include "manifest.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
namespace arnis::building_facades
{
namespace
{
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
	std::vector<Entry> unique;
	for (auto &entry : entries)
		if (std::none_of(unique.begin(), unique.end(),
					[&](const Entry &known) { return known.file == entry.file; }))
			unique.push_back(std::move(entry));
	FacadeSet set;
	set.entries_ = std::move(unique);
	return set;
}

std::optional<FacadeSet> FacadeSet::load_directory(
		const std::filesystem::path &directory, std::vector<std::string> *rejected)
{
	std::ifstream input(directory / "manifest.json");
	if (!input)
		return {};
	nlohmann::json manifest;
	try {
		input >> manifest;
	} catch (...) {
		return {};
	}
	if (!manifest.contains("version") || !manifest["version"].is_number_unsigned() ||
			manifest["version"].get<unsigned>() == 0 || !manifest.contains("textures") ||
			!manifest["textures"].is_array())
		return {};
	std::vector<Entry> entries;
	for (const auto &raw : manifest["textures"]) {
		try {
			Entry entry;
			entry.file = raw.at("file").get<std::string>();
			const std::filesystem::path file(entry.file);
			if (entry.file.empty() || entry.file.size() > 128 || file.is_absolute() ||
					file.has_parent_path() ||
					entry.file.find("..") != std::string::npos ||
					!std::filesystem::is_regular_file(directory / file)) {
				if (rejected)
					rejected->push_back(entry.file + ": missing or unsafe image");
				continue;
			}
			for (const auto &name : raw.at("categories")) {
				auto value = category(name.get<std::string>());
				if (!value)
					throw std::runtime_error("unknown category");
				if (std::find(entry.categories.begin(), entry.categories.end(), *value) ==
						entry.categories.end())
					entry.categories.push_back(*value);
			}
			entry.metres_wide = raw.at("metres_wide").get<double>();
			entry.metres_tall = raw.at("metres_tall").get<double>();
			entry.storeys = raw.at("storeys").get<unsigned>();
			entry.tiles_horizontally = raw.value("tiles_horizontally", false);
			entry.has_ground_floor = raw.value("has_ground_floor", false);
			entries.push_back(std::move(entry));
		} catch (...) {
			if (rejected)
				rejected->push_back("invalid manifest entry");
		}
	}
	return validate(std::move(entries));
}
}
