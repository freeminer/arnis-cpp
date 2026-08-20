#include "retrieve_data.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <nlohmann/json.hpp>
#include <random>
#include <sstream>
#include <stdexcept>

namespace arnis::retrieve_data
{
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
		<< "nwr[\"power\"];nwr[\"historic\"];nwr[\"emergency\"];nwr[\"advertising\"];nwr[\"man_made\"];nwr[\"aeroway\"];nwr[\"3dmr\"];way[\"place\"][\"place\"!~\"^(ocean|sea|bay|strait|sound|fjord)$\"];way;\n)->.relsinbbox;\n"
		<< "(way(r.relsinbbox);)->.waysinbbox;(node(w.waysinbbox);node(w.relsinbbox);)->.nodesinbbox;.relsinbbox out body;.waysinbbox out body;.nodesinbbox out skel qt;";
	return out.str();
}

std::vector<OverpassEndpoint> overpass_request_plan(std::uint64_t seed, bool probe)
{
	std::vector<OverpassEndpoint> official{{"https://overpass-api.de/api/interpreter"},
			{"https://lz4.overpass-api.de/api/interpreter"},
			{"https://z.overpass-api.de/api/interpreter"}};
	std::vector<OverpassEndpoint> fallback{
			{"https://maps.mail.ru/osm/tools/overpass/api/interpreter", true, 360},
			{"https://overpass.private.coffee/api/interpreter", true, 120}};
	std::mt19937_64 rng(seed);
	std::shuffle(official.begin(), official.end(), rng);
	std::shuffle(fallback.begin(), fallback.end(), rng);
	std::vector<OverpassEndpoint> plan;
	if (probe && !official.empty()) {
		plan.push_back(official.front());
		official.erase(official.begin());
	}
	plan.push_back({"https://api.arnismc.com/overpass/api/interpreter"});
	plan.insert(plan.end(), official.begin(), official.end());
	plan.insert(plan.end(), fallback.begin(), fallback.end());
	return plan;
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

std::optional<FetchResult> fetch_overpass(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, std::uint64_t seed, bool probe)
{
	const auto query = overpass_query(bbox);
	std::size_t answered = 0;
	std::size_t truncated = 0;
	for (const auto &endpoint : overpass_request_plan(seed, probe)) {
		auto response = fetcher(endpoint, query);
		if (!response || response->empty())
			continue;
		try {
			std::istringstream input(*response);
			auto document = osm_parser::parse_overpass_json(input);
			++answered;
			if (document.remark && remark_means_truncated(*document.remark)) {
				++truncated;
				if (truncated >= 2)
					break;
				continue;
			}
			return FetchResult{std::move(document), endpoint.url};
		} catch (const std::exception &) {
		}
	}
	(void)answered;
	return std::nullopt;
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
}
