#include "api.h"
#include "sfm.h"
#include "../net.h"
#include "httpfetch.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <array>
#include <limits>
#include <numeric>
#include <nlohmann/json.hpp>
#include <thread>
#include <zlib.h>
namespace arnis::mapillary
{
namespace
{
constexpr std::size_t max_inflated_cluster_bytes = 256U * 1024U * 1024U;

std::string encode_query_value(const std::string &value)
{
	static constexpr char hex[] = "0123456789ABCDEF";
	std::string encoded;
	encoded.reserve(value.size());
	for (const unsigned char byte : value) {
		if ((byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z') ||
				(byte >= '0' && byte <= '9') || byte == '-' || byte == '.' ||
				byte == '_' || byte == '~') {
			encoded.push_back(static_cast<char>(byte));
		} else {
			encoded.push_back('%');
			encoded.push_back(hex[byte >> 4]);
			encoded.push_back(hex[byte & 0x0f]);
		}
	}
	return encoded;
}

std::optional<std::vector<std::uint8_t>> normalize_cluster(
		const std::vector<std::uint8_t> &bytes)
{
	if (bytes.empty() || bytes.size() > std::numeric_limits<uInt>::max())
		return {};
	z_stream stream{};
	if (inflateInit(&stream) == Z_OK) {
		stream.next_in =
				const_cast<Bytef *>(reinterpret_cast<const Bytef *>(bytes.data()));
		stream.avail_in = static_cast<uInt>(bytes.size());
		std::array<std::uint8_t, 64 * 1024> chunk{};
		std::size_t inflated = 0;
		int status = Z_OK;
		while (status == Z_OK && inflated <= max_inflated_cluster_bytes) {
			stream.next_out = chunk.data();
			stream.avail_out = static_cast<uInt>(chunk.size());
			status = inflate(&stream, Z_NO_FLUSH);
			inflated += chunk.size() - stream.avail_out;
		}
		inflateEnd(&stream);
		if (inflated > max_inflated_cluster_bytes)
			return {};
		if (status == Z_STREAM_END)
			return bytes;
	}
	try {
		(void)nlohmann::json::parse(bytes.begin(), bytes.end());
	} catch (...) {
		return {};
	}
	uLongf compressed_size = compressBound(static_cast<uLong>(bytes.size()));
	std::vector<std::uint8_t> compressed(compressed_size);
	if (compress2(compressed.data(), &compressed_size, bytes.data(),
				static_cast<uLong>(bytes.size()), Z_DEFAULT_COMPRESSION) != Z_OK)
		return {};
	compressed.resize(compressed_size);
	return compressed;
}
} // namespace

Client::Client(cache::Layout cache) : cache_(std::move(cache))
{
	concurrent_fetch_ = true;
	graph_fetch_ = [](const std::string &url,
						   std::size_t max_bytes) -> std::optional<HttpReply> {
		auto permit = arnis::net::request_permit();
		HTTPFetchRequest request;
		request.url = url;
		request.timeout = 30000L;
		request.connect_timeout = 10000L;
		request.useragent = "Arnis/Cpp (+https://github.com/louis-e/arnis)";
		request.quiet = true;
		HTTPFetchResult response;
		httpfetch_sync(request, response);
		if (response.response_code <= 0 || response.data.size() > max_bytes)
			return std::nullopt;
		return HttpReply{static_cast<int>(response.response_code),
				std::vector<std::uint8_t>(response.data.begin(), response.data.end())};
	};
}

std::optional<std::vector<SearchCell>> search_cells(
		const SearchCell &bounds, std::size_t maximum, std::string *error)
{
	if (error)
		error->clear();
	if (!std::isfinite(bounds.min_longitude) || !std::isfinite(bounds.min_latitude) ||
			!std::isfinite(bounds.max_longitude) || !std::isfinite(bounds.max_latitude) ||
			bounds.max_longitude < bounds.min_longitude ||
			bounds.max_latitude < bounds.min_latitude) {
		if (error)
			*error = "invalid Mapillary search bounds";
		return {};
	}
	std::size_t lat_steps = std::max<std::size_t>(
			1, std::ceil((bounds.max_latitude - bounds.min_latitude) /
						 mapillary_max_cell_deg));
	std::size_t lon_steps = std::max<std::size_t>(
			1, std::ceil((bounds.max_longitude - bounds.min_longitude) /
						 mapillary_max_cell_deg));
	if (lat_steps > maximum || lon_steps > maximum / lat_steps) {
		if (error) {
			const auto required =
					lat_steps > std::numeric_limits<std::size_t>::max() / lon_steps
							? std::numeric_limits<std::size_t>::max()
							: lat_steps * lon_steps;
			*error = "this area needs " + std::to_string(required) +
					 " imagery search cells and the limit is " + std::to_string(maximum) +
					 ". Search a smaller area.";
		}
		return {};
	}
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
		const std::vector<std::uint8_t> &bytes, bool *valid)
{
	if (valid)
		*valid = false;
	std::vector<cache::ImageRecord> result;
	try {
		auto response = nlohmann::json::parse(bytes.begin(), bytes.end());
		if (!response.contains("data") || !response["data"].is_array())
			return result;
		if (valid)
			*valid = true;
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
		r.raw_json = j.dump();
		r.id = j.value("id", "");
		r.thumb_1024_url = j.value("thumb_1024_url", "");
		r.thumb_2048_url = j.value("thumb_2048_url", "");
		r.thumb_original_url = j.value("thumb_original_url", "");
		if (j.contains("creator") && j["creator"].is_object()) {
			r.creator = j["creator"].value("username", "");
			if (j["creator"].contains("id"))
				r.creator_id = j["creator"]["id"].is_string()
									   ? j["creator"]["id"].get<std::string>()
									   : j["creator"]["id"].dump();
		} else
			r.creator = j.value("creator", "");
		auto number = [&](const char *key) -> std::optional<double> {
			const auto it = j.find(key);
			return it != j.end() && it->is_number()
						   ? std::optional<double>{it->get<double>()}
						   : std::nullopt;
		};
		r.compass_angle = number("computed_compass_angle")
								  .value_or(number("compass_angle").value_or(0.0));
		const auto camera_type = j.value("camera_type", std::string{});
		r.panorama = j.value("is_pano", false) || camera_type == "spherical" ||
					 camera_type == "equirectangular";
		const nlohmann::json *geometry =
				j.contains("computed_geometry") && !j["computed_geometry"].is_null()
						? &j["computed_geometry"]
						: (j.contains("geometry") ? &j["geometry"] : nullptr);
		if (geometry && geometry->is_object() && geometry->contains("coordinates")) {
			auto c = (*geometry)["coordinates"];
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

std::optional<std::vector<std::uint8_t>> Client::fetch_bytes(
		const std::string &url, std::size_t max_bytes) const
{
	if (fetch_)
		return fetch_(url, max_bytes);
	auto reply = fetch_reply(url, max_bytes);
	if (!reply || reply->status < 200 || reply->status >= 300 ||
			reply->body.size() > max_bytes)
		return std::nullopt;
	return std::move(reply->body);
}

std::optional<HttpReply> Client::fetch_reply(
		const std::string &url, std::size_t max_bytes) const
{
	if (graph_fetch_) {
		auto reply = graph_fetch_(url, max_bytes);
		return !reply || reply->body.size() > max_bytes ? std::nullopt : reply;
	}
	if (fetch_) {
		auto body = fetch_(url, max_bytes);
		if (body && body->size() <= max_bytes)
			return HttpReply{200, std::move(*body)};
	}
	return std::nullopt;
}

std::optional<cache::ImageRecord> Client::refresh_metadata(const std::string &id) const
{
	std::string endpoint, token;
	{
		std::lock_guard<std::mutex> lock(credentials_->mutex);
		endpoint = credentials_->endpoint;
		token = credentials_->token;
	}
	if (endpoint.empty() || token.empty() || !cache::Layout::safe_key(id))
		return {};
	while (!endpoint.empty() && endpoint.back() == '/')
		endpoint.pop_back();
	const auto url =
			endpoint + "/" + id + "?access_token=" + encode_query_value(token) +
			"&fields=thumb_original_url,thumb_2048_url,thumb_1024_url,sfm_cluster";
	constexpr std::size_t max_metadata_bytes = 1024 * 1024;
	auto reply = fetch_reply(url, max_metadata_bytes);
	if (!reply || reply->status < 200 || reply->status >= 300)
		return {};
	try {
		auto fresh = nlohmann::json::parse(reply->body.begin(), reply->body.end());
		if (!fresh.is_object() ||
				(fresh.contains("id") && fresh.value("id", std::string{}) != id))
			return {};
		if (auto cached = cache_.load_metadata(id); cached && !cached->raw_json.empty()) {
			auto merged = nlohmann::json::parse(cached->raw_json);
			if (!merged.is_object())
				merged = nlohmann::json::object();
			for (auto it = fresh.begin(); it != fresh.end(); ++it)
				merged[it.key()] = it.value();
			fresh = std::move(merged);
		}
		if (!fresh.contains("id"))
			fresh["id"] = id;
		const auto encoded = fresh.dump();
		const std::vector<std::uint8_t> bytes(encoded.begin(), encoded.end());
		auto record = parse_image_record(bytes);
		if (!record || record->id != id)
			return {};
		cache_.save_metadata(*record);
		return record;
	} catch (...) {
		return {};
	}
}

std::optional<cache::ImageRecord> Client::image(
		const std::string &id, const std::string &url) const
{
	if (auto hit = cache_.load_metadata(id))
		return hit;
	if ((!fetch_ && !graph_fetch_) || !cache::Layout::safe_key(id))
		return {};
	auto bytes = fetch_bytes(url, 1024 * 1024);
	if (!bytes)
		return {};
	auto record = parse_image_record(*bytes);
	if (!record || record->id != id)
		return {};
	return cache_.save_metadata(*record) ? record : std::optional<cache::ImageRecord>{};
}
std::optional<PanoMeta> Client::metadata(const cache::ImageRecord &record) const
{
	if (record.raw_json.empty())
		return {};
	const std::vector<std::uint8_t> bytes(record.raw_json.begin(), record.raw_json.end());
	return parse_pano_meta(bytes);
}
std::optional<std::filesystem::path> Client::download_image(
		const cache::ImageRecord &record, cache::ImageSize size) const
{
	const auto path = cache_.image_path(record.id, size);
	if (!path)
		return {};
	if (cache::is_cached(*path))
		return path;
	if (!fetch_ && !graph_fetch_)
		return {};
	auto current = cache_.load_metadata(record.id).value_or(record);
	auto url_for = [size](const cache::ImageRecord &image) -> const std::string & {
		return size == cache::ImageSize::W1024	 ? image.thumb_1024_url
			   : size == cache::ImageSize::W2048 ? image.thumb_2048_url
												 : image.thumb_original_url;
	};
	bool requeried = false;
	if (url_for(current).empty()) {
		auto refreshed = refresh_metadata(record.id);
		if (!refreshed)
			return {};
		current = std::move(*refreshed);
		requeried = true;
	}
	constexpr std::size_t max_image_bytes = 96 * 1024 * 1024;
	for (;;) {
		const auto &url = url_for(current);
		if (url.empty())
			return {};
		auto reply = fetch_reply(url, max_image_bytes);
		if (reply && reply->status >= 200 && reply->status < 300) {
			auto &bytes = reply->body;
			if (bytes.size() <= 4 || bytes[0] != 0xff || bytes[1] != 0xd8 ||
					bytes[2] != 0xff || !cache_.save_image(record.id, size, bytes))
				return {};
			return path;
		}
		if (!requeried && reply && (reply->status == 403 || reply->status == 404)) {
			auto refreshed = refresh_metadata(record.id);
			if (refreshed) {
				current = std::move(*refreshed);
				requeried = true;
				continue;
			}
		}
		return {};
	}
}
std::optional<std::filesystem::path> Client::download_cluster(
		const cache::ImageRecord &record) const
{
	if (record.raw_json.empty())
		return {};
	try {
		const auto metadata = nlohmann::json::parse(record.raw_json);
		if (!metadata.contains("sfm_cluster") || !metadata["sfm_cluster"].is_object())
			return {};
		auto current = cache_.load_metadata(record.id).value_or(record);
		auto cluster_info = [](const cache::ImageRecord &image) {
			nlohmann::json result = nlohmann::json::object();
			try {
				const auto parsed = nlohmann::json::parse(image.raw_json);
				if (parsed.contains("sfm_cluster") && parsed["sfm_cluster"].is_object())
					result = parsed["sfm_cluster"];
			} catch (...) {
			}
			return result;
		};
		auto cluster = cluster_info(current);
		bool requeried = false;
		if (cluster.value("url", std::string{}).empty()) {
			auto refreshed = refresh_metadata(record.id);
			if (!refreshed)
				return {};
			current = std::move(*refreshed);
			cluster = cluster_info(current);
			requeried = true;
		}
		const auto id_value = cluster.value("id", nlohmann::json{});
		const auto cluster_id = id_value.is_string() ? id_value.get<std::string>()
								: id_value.is_number_integer() ? id_value.dump()
															   : std::string{};
		if (cluster.value("url", std::string{}).empty())
			return {};
		const auto cluster_path = cache_.cluster_path(cluster_id);
		if (!cluster_path)
			return {};
		if (cache::is_cached(*cluster_path))
			return cluster_path;
		if (!fetch_ && !graph_fetch_)
			return {};
		constexpr std::size_t max_cluster_bytes = 96 * 1024 * 1024;
		for (;;) {
			const auto url = cluster.value("url", std::string{});
			if (url.empty())
				return {};
			auto reply = fetch_reply(url, max_cluster_bytes);
			if (reply && reply->status >= 200 && reply->status < 300) {
				auto normalized = normalize_cluster(reply->body);
				if (normalized && cache_.save_cluster(cluster_id, *normalized))
					return cluster_path;
				return {};
			}
			if (!requeried && reply && (reply->status == 403 || reply->status == 404)) {
				auto refreshed = refresh_metadata(record.id);
				if (refreshed) {
					current = std::move(*refreshed);
					cluster = cluster_info(current);
					requeried = true;
					continue;
				}
			}
			return {};
		}
	} catch (...) {
		return {};
	}
}
std::optional<nlohmann::json> Client::cluster_document(
		const cache::ImageRecord &record, std::string *error) const
{
	if (error)
		error->clear();
	try {
		const auto metadata = nlohmann::json::parse(record.raw_json);
		if (!metadata.contains("sfm_cluster") || !metadata["sfm_cluster"].is_object()) {
			if (error)
				*error = "image has no SfM cluster metadata";
			return {};
		}
		const auto id_value = metadata["sfm_cluster"].value("id", nlohmann::json{});
		const auto cluster_id = id_value.is_string() ? id_value.get<std::string>()
								: id_value.is_number_integer() ? id_value.dump()
															   : std::string{};
		if (!cache::Layout::safe_key(cluster_id)) {
			if (error)
				*error = "image has an invalid SfM cluster id";
			return {};
		}
		if (!download_cluster(record)) {
			if (error)
				*error = "SfM cluster download failed";
			return {};
		}
		auto bytes = cache_.load_cluster(cluster_id);
		if (!bytes) {
			if (error)
				*error = "SfM cluster cache entry is missing";
			return {};
		}
		return sfm::parse_cluster_bytes(*bytes, error);
	} catch (const std::exception &exception) {
		if (error)
			*error = std::string("invalid SfM metadata: ") + exception.what();
		return {};
	}
}
std::vector<cache::ImageRecord> Client::search(const SearchCell &bounds,
		const std::string &endpoint, const std::string &token, std::size_t maximum,
		std::size_t *failed_cells, std::string *error) const
{
	std::vector<cache::ImageRecord> result;
	if (error)
		error->clear();
	{
		std::lock_guard<std::mutex> lock(credentials_->mutex);
		credentials_->endpoint = endpoint;
		credentials_->token = token;
	}
	if (failed_cells)
		*failed_cells = 0;
	auto cells = search_cells(bounds, maximum, error);
	if (!cells) {
		if (failed_cells)
			*failed_cells = maximum + 1;
		if (error && error->empty())
			*error = "Mapillary search bounds could not be tiled";
		return result;
	}
	if (!fetch_ && !graph_fetch_) {
		if (failed_cells)
			*failed_cells = cells->size();
		if (error)
			*error = "Mapillary search has no configured HTTP fetcher";
		return result;
	}
	std::vector<std::vector<cache::ImageRecord>> cell_results(cells->size());
	std::vector<std::size_t> cell_failures(cells->size(), 0);
	std::vector<std::uint8_t> cell_answered(cells->size(), 0);
	std::vector<std::size_t> cell_refused(cells->size(), 0);
	std::vector<std::string> cell_errors(cells->size());
	auto query_cell = [&](auto &&self, const SearchCell &cell, unsigned depth,
							  std::vector<cache::ImageRecord> &records,
							  std::size_t &failures, std::uint8_t &answered,
							  std::size_t &refused, std::string &failure_reason) -> void {
		std::string bbox = std::to_string(cell.min_longitude) + "," +
						   std::to_string(cell.min_latitude) + "," +
						   std::to_string(cell.max_longitude) + "," +
						   std::to_string(cell.max_latitude);
		std::string url =
				endpoint + "?access_token=" + encode_query_value(token) +
				"&is_pano=true&fields=id,computed_geometry,geometry,computed_compass_angle,compass_angle,computed_rotation,computed_altitude,altitude,atomic_scale,camera_type,is_pano,camera_parameters,quality_score,width,height,captured_at,sequence,creator,thumb_1024_url,thumb_2048_url,thumb_original_url,sfm_cluster&bbox=" +
				bbox;
		constexpr unsigned max_subdivision_depth = 5;
		constexpr unsigned leaf_retry_count = 2;
		constexpr std::size_t max_search_bytes = 16 * 1024 * 1024;
		std::vector<std::uint8_t> body;
		if (graph_fetch_) {
			auto reply = graph_fetch_(url, max_search_bytes);
			if (!reply) {
				++failures;
				if (failure_reason.empty())
					failure_reason = "Mapillary search request could not be delivered";
				return;
			}
			auto describe_failure = [](const HttpReply &failed) {
				std::string body(failed.body.begin(), failed.body.end());
				std::string message;
				std::string kind;
				try {
					const auto parsed = nlohmann::json::parse(body);
					if (parsed.is_object() && parsed.contains("error") &&
							parsed["error"].is_object()) {
						const auto &api_error = parsed["error"];
						message = api_error.value("message", std::string{});
						kind = api_error.value("type", std::string{});
					}
				} catch (...) {
				}
				if (message.empty()) {
					message = body.substr(0, 200);
					if (message.find_first_not_of(" \t\r\n") == std::string::npos)
						message.clear();
				}
				std::string result = "Mapillary API " + std::to_string(failed.status);
				if (!message.empty())
					result += ": " + message;
				const auto lower = [&] {
					std::string value = message;
					std::transform(value.begin(), value.end(), value.begin(),
							[](unsigned char c) {
								return static_cast<char>(std::tolower(c));
							});
					return value;
				}();
				if (failed.status == 401 || kind == "OAuthException" ||
						lower.find("access token") != std::string::npos)
					result += " Check the Mapillary token.";
				return result;
			};
			bool auth_failure = reply->status == 401;
			if (!auth_failure) {
				const auto detail = describe_failure(*reply);
				auth_failure =
						detail.find("Check the Mapillary token.") != std::string::npos;
			}
			if (reply->status == 500 && !auth_failure && depth < max_subdivision_depth) {
				const double mid_lon = (cell.min_longitude + cell.max_longitude) * 0.5;
				const double mid_lat = (cell.min_latitude + cell.max_latitude) * 0.5;
				const SearchCell quadrants[] = {
						{cell.min_longitude, cell.min_latitude, mid_lon, mid_lat},
						{mid_lon, cell.min_latitude, cell.max_longitude, mid_lat},
						{cell.min_longitude, mid_lat, mid_lon, cell.max_latitude},
						{mid_lon, mid_lat, cell.max_longitude, cell.max_latitude}};
				for (const auto &quadrant : quadrants)
					self(self, quadrant, depth + 1, records, failures, answered, refused,
							failure_reason);
				return;
			}
			// Rust retries a non-explicit Graph 500 at the smallest cell before
			// treating it as a coverage hole. Without this, a transient Graph API
			// failure at maximum subdivision depth permanently drops that cell.
			if (reply->status == 500 && !auth_failure && depth >= max_subdivision_depth) {
				bool recovered = false;
				if (failure_reason.empty())
					failure_reason = describe_failure(*reply);
				for (unsigned retry = 1; retry <= leaf_retry_count; ++retry) {
					std::this_thread::sleep_for(std::chrono::milliseconds(750 * retry));
					reply = graph_fetch_(url, max_search_bytes);
					if (!reply)
						continue;
					bool retry_auth_failure = reply->status == 401;
					if (reply->status == 500) {
						std::string response_text(reply->body.begin(), reply->body.end());
						std::transform(response_text.begin(), response_text.end(),
								response_text.begin(), [](unsigned char c) {
									return static_cast<char>(std::tolower(c));
								});
						retry_auth_failure =
								response_text.find("oauthexception") !=
										std::string::npos ||
								response_text.find("access token") != std::string::npos;
					}
					if (reply->status >= 200 && reply->status < 300) {
						recovered = true;
						break;
					}
					if (failure_reason.empty())
						failure_reason = describe_failure(*reply);
					if (retry_auth_failure || reply->status != 500)
						break;
				}
				if (!recovered) {
					++failures;
					++refused;
					if (reply && failure_reason.empty())
						failure_reason = describe_failure(*reply);
					return;
				}
			} else if (reply->status == 500 && auth_failure) {
				++failures;
				if (failure_reason.empty())
					failure_reason = describe_failure(*reply);
				return;
			}
			if (reply->status < 200 || reply->status >= 300) {
				++failures;
				if (failure_reason.empty())
					failure_reason = describe_failure(*reply);
				return;
			}
			body = std::move(reply->body);
		} else {
			auto reply = fetch_(url, max_search_bytes);
			if (!reply) {
				++failures;
				if (failure_reason.empty())
					failure_reason = "Mapillary search request could not be delivered";
				return;
			}
			body = std::move(*reply);
		}
		bool valid_response = false;
		auto cell_records = parse_search_response(body, &valid_response);
		if (!valid_response) {
			++failures;
			if (failure_reason.empty())
				failure_reason = "Mapillary response was not the expected JSON";
		} else {
			answered = 1;
		}
		records.insert(records.end(), std::make_move_iterator(cell_records.begin()),
				std::make_move_iterator(cell_records.end()));
	};
	const auto hardware = std::thread::hardware_concurrency();
	const std::size_t worker_limit =
			concurrent_fetch_
					? std::clamp<std::size_t>(hardware == 0 ? 2 : hardware, 1, 12)
					: 1;
	const std::size_t worker_count = std::min(cells->size(), worker_limit);
	std::atomic_size_t next_cell{0};
	std::vector<std::thread> workers;
	workers.reserve(worker_count);
	for (std::size_t worker = 0; worker < worker_count; ++worker) {
		workers.emplace_back([&] {
			for (;;) {
				const auto index = next_cell.fetch_add(1, std::memory_order_relaxed);
				if (index >= cells->size())
					return;
				try {
					query_cell(query_cell, (*cells)[index], 0, cell_results[index],
							cell_failures[index], cell_answered[index],
							cell_refused[index], cell_errors[index]);
				} catch (...) {
					++cell_failures[index];
					if (cell_errors[index].empty())
						cell_errors[index] = "Mapillary search failed unexpectedly";
				}
			}
		});
	}
	for (auto &worker : workers)
		worker.join();
	for (std::size_t index = 0; index < cells->size(); ++index) {
		if (failed_cells)
			*failed_cells += cell_failures[index];
		result.insert(result.end(), std::make_move_iterator(cell_results[index].begin()),
				std::make_move_iterator(cell_results[index].end()));
	}
	if (error && result.empty() &&
			std::any_of(cell_refused.begin(), cell_refused.end(),
					[](std::size_t refused) { return refused != 0; })) {
		const auto refused =
				std::accumulate(cell_refused.begin(), cell_refused.end(), std::size_t{0});
		*error = "Mapillary refused every one of " + std::to_string(refused) +
				 " search cells for this area. A token the API does not accept is the "
				 "usual cause; a service outage is another.";
	} else if (error && result.empty() &&
			   std::none_of(cell_answered.begin(), cell_answered.end(),
					   [](std::uint8_t answered) { return answered != 0; })) {
		const auto failed = std::find_if(cell_errors.begin(), cell_errors.end(),
				[](const std::string &message) { return !message.empty(); });
		if (failed != cell_errors.end())
			*error = *failed;
	}
	std::sort(result.begin(), result.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	result.erase(std::unique(result.begin(), result.end(),
						 [](const auto &a, const auto &b) { return a.id == b.id; }),
			result.end());
	// Match Rust: deduplicate across all cells first, then write one cache record
	// per image. This also avoids races when buffered search cells overlap.
	for (const auto &record : result)
		cache_.save_metadata(record);
	return result;
}
}
