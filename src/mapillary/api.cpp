#include "api.h"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
namespace arnis::mapillary
{
std::optional<std::vector<SearchCell>> search_cells(
		const SearchCell &bounds, std::size_t maximum)
{
	if (!std::isfinite(bounds.min_longitude) || !std::isfinite(bounds.min_latitude) ||
			!std::isfinite(bounds.max_longitude) || !std::isfinite(bounds.max_latitude) ||
			bounds.max_longitude < bounds.min_longitude ||
			bounds.max_latitude < bounds.min_latitude)
		return {};
	std::size_t lat_steps = std::max<std::size_t>(
			1, std::ceil((bounds.max_latitude - bounds.min_latitude) /
						 mapillary_max_cell_deg));
	std::size_t lon_steps = std::max<std::size_t>(
			1, std::ceil((bounds.max_longitude - bounds.min_longitude) /
						 mapillary_max_cell_deg));
	if (lat_steps > maximum || lon_steps > maximum / lat_steps)
		return {};
	double lat_span = (bounds.max_latitude - bounds.min_latitude) / lat_steps;
	double lon_span = (bounds.max_longitude - bounds.min_longitude) / lon_steps;
	std::vector<SearchCell> result;
	result.reserve(lat_steps * lon_steps);
	for (std::size_t row = 0; row < lat_steps; ++row)
		for (std::size_t column = 0; column < lon_steps; ++column) {
			double min_latitude = bounds.min_latitude + row * lat_span;
			double min_longitude = bounds.min_longitude + column * lon_span;
			result.push_back({min_longitude, min_latitude,
					std::min(bounds.max_longitude, min_longitude + lon_span),
					std::min(bounds.max_latitude, min_latitude + lat_span)});
		}
	return result;
}

std::vector<cache::ImageRecord> parse_search_response(
		const std::vector<std::uint8_t> &bytes)
{
	std::vector<cache::ImageRecord> result;
	try {
		auto response = nlohmann::json::parse(bytes.begin(), bytes.end());
		if (!response.contains("data") || !response["data"].is_array())
			return result;
		for (const auto &entry : response["data"]) {
			auto encoded = entry.dump();
			std::vector<std::uint8_t> one(encoded.begin(), encoded.end());
			auto image = parse_image_record(one);
			if (image && image->panorama)
				result.push_back(std::move(*image));
		}
		std::sort(result.begin(), result.end(),
				[](const auto &a, const auto &b) { return a.id < b.id; });
		result.erase(std::unique(result.begin(), result.end(),
							 [](const auto &a, const auto &b) { return a.id == b.id; }),
				result.end());
	} catch (...) {
	}
	return result;
}

std::optional<cache::ImageRecord> parse_image_record(
		const std::vector<std::uint8_t> &bytes)
{
	try {
		auto j = nlohmann::json::parse(bytes.begin(), bytes.end());
		cache::ImageRecord r;
		r.id = j.value("id", "");
		r.thumb_1024_url = j.value("thumb_1024_url", "");
		r.thumb_2048_url = j.value("thumb_2048_url", "");
		if (j.contains("creator") && j["creator"].is_object())
			r.creator = j["creator"].value("username", "");
		else
			r.creator = j.value("creator", "");
		r.compass_angle =
				j.value("computed_compass_angle", j.value("compass_angle", 0.0));
		r.panorama = j.value("is_pano", false);
		auto geometry = j.contains("computed_geometry")
								? j["computed_geometry"]
								: j.value("geometry", nlohmann::json{});
		if (geometry.contains("coordinates")) {
			auto c = geometry["coordinates"];
			if (c.size() >= 2) {
				r.longitude = c[0].get<double>();
				r.latitude = c[1].get<double>();
			}
		}
		return cache::Layout::safe_key(r.id)
					   ? std::optional<cache::ImageRecord>{std::move(r)}
					   : std::nullopt;
	} catch (...) {
		return {};
	}
}
std::optional<PanoMeta> parse_pano_meta(const std::vector<std::uint8_t> &bytes)
{
	try {
		auto j = nlohmann::json::parse(bytes.begin(), bytes.end());
		PanoMeta result;
		if (!j.contains("id"))
			return {};
		if (j["id"].is_string())
			result.id = j["id"].get<std::string>();
		else if (j["id"].is_number_integer())
			result.id = std::to_string(j["id"].get<std::int64_t>());
		else
			return {};
		const auto *geometry =
				j.contains("computed_geometry") && !j["computed_geometry"].is_null()
						? &j["computed_geometry"]
						: (j.contains("geometry") ? &j["geometry"] : nullptr);
		if (!geometry || !geometry->contains("coordinates") ||
				!(*geometry)["coordinates"].is_array() ||
				(*geometry)["coordinates"].size() < 2)
			return {};
		result.lon = (*geometry)["coordinates"][0].get<double>();
		result.lat = (*geometry)["coordinates"][1].get<double>();
		auto number = [&](const char *key, double fallback = 0.0) {
			return j.contains(key) && j[key].is_number() ? j[key].get<double>()
														 : fallback;
		};
		result.alt = number("computed_altitude");
		const auto compass_key = j.contains("computed_compass_angle") &&
												 j["computed_compass_angle"].is_number()
										 ? "computed_compass_angle"
										 : "compass_angle";
		result.compass = std::fmod(number(compass_key), 360.0);
		if (result.compass < 0.0)
			result.compass += 360.0;
		if (j.contains("computed_rotation") && j["computed_rotation"].is_array() &&
				j["computed_rotation"].size() >= 3)
			result.rotation =
					std::array<double, 3>{j["computed_rotation"][0].get<double>(),
							j["computed_rotation"][1].get<double>(),
							j["computed_rotation"][2].get<double>()};
		if (j.contains("atomic_scale") && j["atomic_scale"].is_number())
			result.atomic_scale = j["atomic_scale"].get<double>();
		result.captured_at = j.value("captured_at", std::int64_t{0});
		result.sequence = j.value("sequence", std::string{});
		result.quality = number("quality_score");
		result.width = j.value("width", 0U);
		result.height = j.value("height", 0U);
		if (j.contains("sfm_cluster") && j["sfm_cluster"].is_object() &&
				j["sfm_cluster"].contains("id"))
			result.cluster_id = j["sfm_cluster"]["id"].is_string()
										? j["sfm_cluster"]["id"].get<std::string>()
										: j["sfm_cluster"]["id"].dump();
		result.geometry_source =
				j.contains("computed_geometry") && !j["computed_geometry"].is_null()
						? GeometrySource::Computed
						: GeometrySource::Gps;
		const auto camera = j.value("camera_type", std::string{});
		result.camera_type =
				camera.empty() ? (j.value("is_pano", false) ? CameraModel::Spherical
															: CameraModel::Perspective)
							   : parse_camera_model(camera);
		if (j.contains("camera_parameters") && j["camera_parameters"].is_array())
			for (const auto &value : j["camera_parameters"])
				if (value.is_number())
					result.camera_params.push_back(value.get<double>());
		return result.valid() ? std::optional<PanoMeta>{std::move(result)} : std::nullopt;
	} catch (...) {
		return {};
	}
}
std::optional<cache::ImageRecord> Client::image(
		const std::string &id, const std::string &url) const
{
	if (auto hit = cache_.load_metadata(id))
		return hit;
	if (!fetch_ || !cache::Layout::safe_key(id))
		return {};
	auto bytes = fetch_(url, 1024 * 1024);
	if (!bytes)
		return {};
	auto record = parse_image_record(*bytes);
	if (!record || record->id != id)
		return {};
	return cache_.save_metadata(*record) ? record : std::optional<cache::ImageRecord>{};
}
std::vector<cache::ImageRecord> Client::search(const SearchCell &bounds,
		const std::string &endpoint, const std::string &token) const
{
	std::vector<cache::ImageRecord> result;
	auto cells = search_cells(bounds);
	if (!fetch_ || !cells)
		return result;
	for (const auto &cell : *cells) {
		std::string bbox = std::to_string(cell.min_longitude) + "," +
						   std::to_string(cell.min_latitude) + "," +
						   std::to_string(cell.max_longitude) + "," +
						   std::to_string(cell.max_latitude);
		std::string url =
				endpoint + "?access_token=" + token +
				"&is_pano=true&fields=id,computed_geometry,geometry,computed_compass_angle,compass_angle,is_pano,thumb_1024_url,thumb_2048_url,creator&bbox=" +
				bbox;
		auto reply = fetch_(url, 16 * 1024 * 1024);
		if (!reply)
			continue;
		auto records = parse_search_response(*reply);
		result.insert(result.end(), std::make_move_iterator(records.begin()),
				std::make_move_iterator(records.end()));
	}
	std::sort(result.begin(), result.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	result.erase(std::unique(result.begin(), result.end(),
						 [](const auto &a, const auto &b) { return a.id == b.id; }),
			result.end());
	return result;
}
}
