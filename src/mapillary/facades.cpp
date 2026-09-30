#include "facades.h"

#include <algorithm>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <shared_mutex>

namespace arnis::mapillary::facades
{
namespace
{
std::optional<std::uint64_t> u64(const nlohmann::json &value)
{
	if (value.is_number_unsigned())
		return value.get<std::uint64_t>();
	if (value.is_number_integer() && value.get<std::int64_t>() >= 0)
		return static_cast<std::uint64_t>(value.get<std::int64_t>());
	if (value.is_string()) {
		try {
			return std::stoull(value.get<std::string>());
		} catch (...) {
		}
	}
	return std::nullopt;
}

std::shared_mutex store_mutex;
std::optional<Export> store;

std::optional<Owner> owner_for(const nlohmann::json &record)
{
	const bool relation = record.value("kind", "") == "relation";
	const char *primary = relation ? "relation_id" : "way_id";
	const char *fallback = "osm_id";
	const auto id = record.contains(primary) ? u64(record[primary]) : std::nullopt;
	const auto fallback_id =
			record.contains(fallback) ? u64(record[fallback]) : std::nullopt;
	if (!id && !fallback_id)
		return std::nullopt;
	return Owner{
			relation ? OwnerKind::Relation : OwnerKind::Way, id.value_or(*fallback_id)};
}
} // namespace

std::optional<Export> load_export(
		const std::filesystem::path &directory, std::string *error)
{
	std::error_code ec;
	if (!std::filesystem::is_directory(directory, ec)) {
		if (error)
			*error = "not a facade export directory: " + directory.string();
		return std::nullopt;
	}
	Export result;
	std::set<std::string> images;
	for (const auto &entry : std::filesystem::directory_iterator(directory, ec)) {
		if (ec)
			break;
		if (!entry.is_regular_file(ec) || entry.path().extension() != ".json" ||
				entry.path().filename() == "manifest.json")
			continue;
		std::ifstream input(entry.path());
		if (!input)
			continue;
		try {
			nlohmann::json record;
			input >> record;
			const auto owner = owner_for(record);
			if (!owner)
				continue;
			ExportBuilding building;
			building.owner = *owner;
			if (const auto color = record.value("building_colour", nlohmann::json{});
					color.is_object() && color.contains("rgb") &&
					color["rgb"].is_array() && color["rgb"].size() == 3) {
				imgops::Rgb rgb{};
				for (std::size_t i = 0; i < 3; ++i)
					rgb[i] = static_cast<std::uint8_t>(std::min<std::uint64_t>(
							255, color["rgb"][i].is_number()
										 ? color["rgb"][i].get<std::uint64_t>()
										 : 0));
				building.building_color = rgb;
			}
			for (const auto &wall : record.value("walls", nlohmann::json::array())) {
				const auto tier = wall.value("tier", "D");
				if (tier != "A" && tier != "B")
					continue;
				ExportWall out;
				out.owner = *owner;
				out.tier = tier;
				out.columns = wall.value("cols", wall.value("columns", 0u));
				out.rows = wall.value("rows", 0u);
				if (const auto name = wall.value("png", ""); !name.empty())
					out.color_png = directory / name;
				if (const auto name = wall.value("tex", ""); !name.empty())
					out.texture_png = directory / name;
				for (const auto &view : wall.value("views", nlohmann::json::array())) {
					const auto id = view.is_string() ? view.get<std::string>()
													 : view.value("pano", "");
					if (!id.empty()) {
						out.views.push_back(id);
						images.insert(id);
					}
				}
				building.walls.push_back(std::move(out));
			}
			if (!building.walls.empty())
				result.buildings.push_back(std::move(building));
		} catch (const std::exception &e) {
			if (error)
				*error = entry.path().string() + ": " + e.what();
			return std::nullopt;
		}
	}
	result.image_ids.assign(images.begin(), images.end());
	return result;
}

void install_export(Export value)
{
	std::unique_lock lock(store_mutex);
	store = std::move(value);
}

void clear()
{
	std::unique_lock lock(store_mutex);
	store.reset();
}

bool has_building(std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	return store && std::any_of(store->buildings.begin(), store->buildings.end(),
							[&](const ExportBuilding &b) {
								return b.owner.kind == OwnerKind::Way &&
									   b.owner.id == way_id;
							});
}

std::optional<imgops::Rgb> building_color(std::uint64_t way_id)
{
	std::shared_lock lock(store_mutex);
	if (!store)
		return std::nullopt;
	for (const auto &building : store->buildings)
		if (building.owner.kind == OwnerKind::Way && building.owner.id == way_id)
			return building.building_color;
	return std::nullopt;
}
} // namespace arnis::mapillary::facades
