#include "retrieve_data.h"
#include "args.h"
#include "osm_tiles.h"
#include <algorithm>
#include <cctype>
#include <curl/curl.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace arnis::retrieve_data
{
namespace
{
size_t area_name_write(void *contents, size_t size, size_t count, void *user)
{
	auto *out = static_cast<std::string *>(user);
	out->append(static_cast<const char *>(contents), size * count);
	return size * count;
}
}
std::string url_host(const std::string &url)
{
	const auto scheme = url.find("://");
	const auto begin = scheme == std::string::npos ? 0 : scheme + 3;
	const auto end = url.find_first_of("/?", begin);
	return url.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
}

bool is_osm_xml_path(const std::filesystem::path &path)
{
	auto ext = path.extension().string();
	std::transform(ext.begin(), ext.end(), ext.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	return ext == ".osm" || ext == ".xml";
}

osm_parser::RawOsmDocument load_osm_file(const std::filesystem::path &path)
{
	std::ifstream input(path, std::ios::binary);
	if (!input)
		throw std::runtime_error("Cannot open OSM input " + path.string());
	return is_osm_xml_path(path) ? osm_parser::parse_osm_xml(input)
								 : osm_parser::parse_overpass_json(input);
}

std::string overpass_query(const geographic::LLBBox &bbox)
{
	std::ostringstream out;
	out << "[out:json][timeout:360][bbox:" << bbox.min().lat() << ',' << bbox.min().lng()
		<< ',' << bbox.max().lat() << ',' << bbox.max().lng() << "];\n(\n"
		<< "nwr[\"building\"];nwr[\"building:part\"];relation[\"type\"=\"building\"];nwr[\"highway\"];nwr[\"landuse\"][\"landuse\"!=\"salt_pond\"];"
		<< "nwr[\"natural\"][\"natural\"!=\"coastline\"][\"natural\"!=\"bay\"][\"natural\"!=\"strait\"];nwr[\"leisure\"];"
		<< "nwr[\"water\"][\"water\"!=\"bay\"][\"water\"!=\"ocean\"][\"water\"!=\"sea\"][\"tidal\"!=\"yes\"];nwr[\"waterway\"][\"waterway\"!=\"tidal_channel\"];"
		<< "nwr[\"amenity\"];nwr[\"tourism\"];nwr[\"bridge\"];nwr[\"railway\"];nwr[\"roller_coaster\"];nwr[\"barrier\"];nwr[\"entrance\"];nwr[\"door\"];"
		<< "nwr[\"power\"];nwr[\"historic\"];nwr[\"emergency\"];nwr[\"advertising\"];nwr[\"man_made\"];nwr[\"aeroway\"];nwr[\"3dmr\"];"
		<< "nwr[\"shop\"];nwr[\"office\"];way[\"place\"][\"place\"!~\"^(ocean|sea|bay|strait|sound|fjord)$\"];way;\n)->.relsinbbox;\n"
		<< "(way(r.relsinbbox);)->.waysinbbox;(node(w.waysinbbox);node(w.relsinbbox);)->.nodesinbbox;.relsinbbox out body;.waysinbbox out body;.nodesinbbox out skel qt;";
	return out.str();
}

std::vector<OverpassEndpoint> overpass_request_plan(std::uint64_t seed, bool probe)
{
	(void)seed;
	(void)probe;
	return {{ARNIS_OVERPASS_URL}};
}

bool remark_means_truncated(const std::string &remark)
{
	std::string lower = remark;
	std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	return lower.find("runtime error") != std::string::npos ||
		   lower.find("timed out") != std::string::npos ||
		   lower.find("out of memory") != std::string::npos;
}

const char *fallback_reason(const std::string &error)
{
	static constexpr std::pair<const char *, const char *> buckets[] = {
			{"has no data for this area", "no_data_for_area"},
			{"that area needs", "area_too_large"},
			{"index unreachable", "index_unreachable"},
			{"bad archive index", "bad_index"}, {"unusable name", "bad_index"},
			{"archive is zoom", "zoom_mismatch"}, {"expected 206", "range_http"},
			{"range request to", "network"}, {"could not be read", "network"},
			{"not an Arnis tile payload", "tile_decode"},
			{"truncated tile", "tile_decode"}, {"implausible", "tile_decode"}};
	for (const auto &[needle, bucket] : buckets)
		if (error.find(needle) != std::string::npos)
			return bucket;
	return "other";
}

std::optional<FetchResult> fetch_overpass(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, std::uint64_t seed, bool probe,
		std::string *error)
{
	const auto query = overpass_query(bbox);
	std::size_t answered = 0;
	std::size_t truncated = 0;
	std::string last_error;
	for (const auto &endpoint : overpass_request_plan(seed, probe)) {
		auto response = fetcher(endpoint, query);
		if (!response) {
			last_error = "Overpass request failed at " + url_host(endpoint.url);
			continue;
		}
		if (response->empty()) {
			last_error =
					"Overpass returned an empty response from " + url_host(endpoint.url);
			continue;
		}
		++answered;
		try {
			std::istringstream input(*response);
			auto document = osm_parser::parse_overpass_json(input);
			if (document.remark && remark_means_truncated(*document.remark)) {
				++truncated;
				last_error = "Overpass response was truncated at " +
							 url_host(endpoint.url) + ": " + *document.remark;
				if (truncated >= 2)
					break;
				continue;
			}
			return FetchResult{std::move(document), endpoint.url};
		} catch (const std::exception &e) {
			last_error = "Invalid Overpass response from " + url_host(endpoint.url) +
						 ": " + e.what();
		}
	}
	if (error) {
		if (truncated > 0 && truncated == answered)
			*error =
					"All answering Overpass servers returned partial data. " + last_error;
		else if (truncated > 0)
			*error =
					std::to_string(truncated) + " of " + std::to_string(answered) +
					" answering Overpass server(s) returned partial data; others failed. " +
					last_error;
		else if (!last_error.empty())
			*error = std::move(last_error);
		else
			*error = "All Overpass servers failed without a response";
	}
	return std::nullopt;
}

std::optional<FetchResult> fetch_osm_data(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, bool use_tile_archive,
		const std::string &tiles_url, std::uint64_t seed, bool probe, std::string *error)
{
	std::string archive_error;
	if (use_tile_archive && !tiles_url.empty()) {
		if (auto document =
						osm_tiles::fetch_data_from_tiles(bbox, tiles_url, &archive_error))
			return FetchResult{std::move(*document), tiles_url};
	}
	std::string overpass_error;
	auto result = fetch_overpass(bbox, fetcher, seed, probe, &overpass_error);
	if (!result && error) {
		if (!archive_error.empty())
			*error = "Tile archive unavailable (" + archive_error +
					 "); Overpass fallback failed (" + overpass_error + ")";
		else
			*error = std::move(overpass_error);
	}
	return result;
}

std::optional<FetchResult> fetch_osm_data(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, const arnis::Args &args, std::uint64_t seed,
		bool probe, std::string *error)
{
	return fetch_osm_data(
			bbox, fetcher, !args.no_tile_archive, args.osm_tiles_url, seed, probe, error);
}

std::optional<std::string> area_name_from_nominatim_json(const std::string &body)
{
	const auto json = nlohmann::json::parse(body, nullptr, false);
	if (json.is_discarded() || !json.contains("address") || !json["address"].is_object())
		return std::nullopt;
	for (const char *field : {"city", "town", "village", "county", "borough", "suburb"}) {
		if (!json["address"].contains(field) || !json["address"][field].is_string())
			continue;
		auto name = json["address"][field].get<std::string>();
		std::string lower = name;
		std::transform(lower.begin(), lower.end(), lower.begin(),
				[](unsigned char c) { return char(std::tolower(c)); });
		if (lower.rfind("city of ", 0) == 0)
			name.erase(0, 8);
		return name;
	}
	return std::nullopt;
}

std::optional<std::string> fetch_area_name(double lat, double lon)
{
	const auto url = "https://nominatim.openstreetmap.org/reverse?format=jsonv2&lat=" +
					 std::to_string(lat) + "&lon=" + std::to_string(lon) +
					 "&addressdetails=1";
	CURL *curl = curl_easy_init();
	if (!curl)
		return std::nullopt;
	std::string body;
	curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, area_name_write);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
	curl_easy_setopt(curl, CURLOPT_USERAGENT, OSM_USER_AGENT);
	// Match reqwest/curl --compressed in the Rust downloader.  libcurl then
	// advertises compression and transparently decodes gzip/zstd responses.
	curl_easy_setopt(curl, CURLOPT_ACCEPT_ENCODING, "");
	curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
	long status = 0;
	const auto result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300)
		return std::nullopt;
	return area_name_from_nominatim_json(body);
}
}
