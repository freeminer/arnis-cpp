#include "retrieve_data.h"
#include "net.h"
#include "args.h"
#include "osm_tiles.h"
#include "osm_pbf.h"
#include "osm_pbf_cache.h"
#include "one_world.h"
#include "telemetry.h"
#include <algorithm>
#include <cctype>
#include <curl/curl.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

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

nlohmann::json json_tags(const tags_t &tags)
{
	nlohmann::json result = nlohmann::json::object();
	for (const auto &[key, value] : tags)
		result[key] = value;
	return result;
}

nlohmann::json document_json(const osm_parser::RawOsmDocument &document)
{
	nlohmann::json root;
	root["elements"] = nlohmann::json::array();
	for (const auto &node : document.nodes) {
		nlohmann::json element{{"type", "node"}, {"id", node.id}};
		if (node.lat)
			element["lat"] = *node.lat;
		if (node.lon)
			element["lon"] = *node.lon;
		if (!node.tags.empty())
			element["tags"] = json_tags(node.tags);
		root["elements"].push_back(std::move(element));
	}
	for (const auto &way : document.ways) {
		nlohmann::json element{{"type", "way"}, {"id", way.id}, {"nodes", way.node_refs}};
		if (!way.tags.empty())
			element["tags"] = json_tags(way.tags);
		root["elements"].push_back(std::move(element));
	}
	for (const auto &relation : document.relations) {
		nlohmann::json members = nlohmann::json::array();
		for (const auto &member : relation.members)
			members.push_back(
					{{"type", member.type}, {"ref", member.ref}, {"role", member.role}});
		nlohmann::json element{{"type", "relation"}, {"id", relation.id},
				{"members", std::move(members)}};
		if (!relation.tags.empty())
			element["tags"] = json_tags(relation.tags);
		root["elements"].push_back(std::move(element));
	}
	if (document.remark)
		root["remark"] = *document.remark;
	if (document.bounds) {
		const auto &b = *document.bounds;
		root["bounds"] = {
				{"minlat", b[0]}, {"minlon", b[1]}, {"maxlat", b[2]}, {"maxlon", b[3]}};
	}
	return root;
}

class RawPbfHandler : public osmium::handler::Handler
{
	osm_parser::RawOsmDocument &document_;

	static tags_t copy_tags(const osmium::TagList &source)
	{
		tags_t out;
		for (const auto &tag : source)
			out.emplace(tag.key(), tag.value());
		return out;
	}

public:
	explicit RawPbfHandler(osm_parser::RawOsmDocument &document) : document_(document) {}

	void node(const osmium::Node &node)
	{
		osm_parser::RawNode out;
		out.id = static_cast<std::uint64_t>(node.id());
		if (node.location().valid()) {
			out.lat = node.location().lat();
			out.lon = node.location().lon();
		}
		out.tags = copy_tags(node.tags());
		document_.nodes.push_back(std::move(out));
	}

	void way(const osmium::Way &way)
	{
		osm_parser::RawWay out;
		out.id = static_cast<std::uint64_t>(way.id());
		out.node_refs.reserve(way.nodes().size());
		for (const auto &node : way.nodes())
			out.node_refs.push_back(static_cast<std::uint64_t>(node.ref()));
		out.tags = copy_tags(way.tags());
		document_.ways.push_back(std::move(out));
	}

	void relation(const osmium::Relation &relation)
	{
		osm_parser::RawRelation out;
		out.id = static_cast<std::uint64_t>(relation.id());
		out.tags = copy_tags(relation.tags());
		out.members.reserve(relation.members().size());
		for (const auto &member : relation.members()) {
			const char *type = nullptr;
			switch (member.type()) {
			case osmium::item_type::node:
				type = "node";
				break;
			case osmium::item_type::way:
				type = "way";
				break;
			case osmium::item_type::relation:
				type = "relation";
				break;
			default:
				continue;
			}
			out.members.push_back({type, static_cast<std::uint64_t>(member.ref()),
					std::string(member.role())});
		}
		document_.relations.push_back(std::move(out));
	}
};

osm_parser::RawOsmDocument parse_osm_pbf(const std::filesystem::path &path)
{
	osm_parser::RawOsmDocument document;
	osmium::io::Reader reader{osmium::io::File{path.string()}};
	const auto bounds = reader.header().joined_boxes();
	if (bounds.valid())
		document.bounds = std::array<double, 4>{bounds.bottom_left().lat(),
				bounds.bottom_left().lon(), bounds.top_right().lat(),
				bounds.top_right().lon()};
	RawPbfHandler handler(document);
	osmium::apply(reader, handler);
	reader.close();
	document.completeness = osm_parser::analyze_completeness(document);
	return document;
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

namespace
{
bool is_osm_pbf_path(const std::filesystem::path &path)
{
	auto ext = path.extension().string();
	std::transform(ext.begin(), ext.end(), ext.begin(),
			[](unsigned char c) { return char(std::tolower(c)); });
	return ext == ".pbf";
}
}

osm_parser::RawOsmDocument load_osm_file(const std::filesystem::path &path)
{
	if (is_osm_pbf_path(path))
		return parse_osm_pbf(path);
	std::ifstream input(path, std::ios::binary);
	if (!input)
		throw std::runtime_error("Cannot open OSM input " + path.string());
	return is_osm_xml_path(path) ? osm_parser::parse_osm_xml(input)
								 : osm_parser::parse_overpass_json(input);
}

void save_osm_json(osm_parser::RawOsmDocument &document,
		const geographic::LLBBox &requested_bbox, const std::filesystem::path &path)
{
	document.bounds =
			std::array<double, 4>{requested_bbox.min().lat(), requested_bbox.min().lng(),
					requested_bbox.max().lat(), requested_bbox.max().lng()};
	std::ofstream output(path, std::ios::binary | std::ios::trunc);
	if (!output)
		throw std::runtime_error("Cannot create OSM JSON output " + path.string());
	output << document_json(document).dump();
	output.flush();
	if (!output)
		throw std::runtime_error("Cannot write OSM JSON output " + path.string());
	std::cout << "OSM data saved to: " << path.string() << '\n';
}

std::string overpass_query(const geographic::LLBBox &bbox)
{
	std::ostringstream out;
	out << "[out:json][timeout:360][bbox:" << bbox.min().lat() << ',' << bbox.min().lng()
		<< ',' << bbox.max().lat() << ',' << bbox.max().lng() << "];\n(\n"
		<< "nwr[\"building\"];nwr[\"building:part\"];relation[\"type\"=\"building\"];"
		<< "nwr[\"highway\"];nwr[\"landuse\"];nwr[\"natural\"];nwr[\"leisure\"];"
		<< "nwr[\"water\"];nwr[\"waterway\"];nwr[\"amenity\"];nwr[\"tourism\"];"
		<< "nwr[\"bridge\"];nwr[\"railway\"];nwr[\"roller_coaster\"];nwr[\"aerialway\"];"
		<< "node[\"public_transport\"~\"^(stop_position|station)$\"]"
		   "[~\"^(train|tram|subway|light_rail)$\"~\"^yes$\"];"
		<< "nwr[\"attraction\"];nwr[\"barrier\"];nwr[\"entrance\"];nwr[\"door\"];"
		<< "nwr[\"power\"];nwr[\"historic\"];nwr[\"emergency\"];nwr[\"advertising\"];"
		<< "nwr[\"man_made\"];nwr[\"aeroway\"];nwr[\"3dmr\"];nwr[\"shop\"];nwr[\"office\"];"
		<< "node[\"raceway\"];"
		<< "node[\"place\"~\"^(city|town|borough|suburb|village|quarter|neighbourhood|hamlet)$\"];"
		<< "way[~\".\"~\".\"];\n)->.candidates;\n"
		<< "(nwr.candidates[\"landuse\"=\"salt_pond\"];"
		<< "nwr.candidates[\"natural\"~\"^(coastline|bay|strait)$\"];"
		<< "nwr.candidates[\"water\"~\"^(bay|ocean|sea)$\"];"
		<< "nwr.candidates[\"tidal\"=\"yes\"];"
		<< "nwr.candidates[\"waterway\"=\"tidal_channel\"];"
		<< "nwr.candidates[\"place\"~\"^(ocean|sea|bay|strait|sound|fjord)$\"];"
		<< ")->.excluded;\n(.candidates; - .excluded;)->.wanted;\n"
		<< "(way(r.wanted);)->.members;\n"
		<< "(node(w.members);node(w.wanted);)->.waynodes;\n"
		<< ".wanted out body;.members out body;.waynodes out skel qt;";
	return out.str();
}

std::vector<OverpassEndpoint> overpass_request_plan(
		std::uint64_t seed, bool probe, const std::vector<std::string> &configured_urls)
{
	(void)seed;
	(void)probe;
	std::vector<OverpassEndpoint> endpoints;
	if (configured_urls.empty()) {
		endpoints.push_back({ARNIS_OVERPASS_URL});
		return endpoints;
	}
	endpoints.reserve(configured_urls.size());
	for (const auto &url : configured_urls)
		if (!url.empty())
			endpoints.push_back({url});
	if (endpoints.empty())
		endpoints.push_back({ARNIS_OVERPASS_URL});
	return endpoints;
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
	for (const auto &[needle, candidate] : buckets)
		if (error.find(needle) != std::string::npos)
			return candidate;
	return "other";
}

std::string fallback_reason_detail(const std::string &error)
{
	const std::string bucket = fallback_reason(error);
	std::string kind;
	for (std::size_t open = error.find('['); open != std::string::npos;
			open = error.find('[', open + 1)) {
		const auto close = error.find(']', open + 1);
		if (close == std::string::npos)
			break;
		const auto tag = error.substr(open + 1, close - open - 1);
		if (net::is_failure_kind(tag)) {
			kind = tag;
			break;
		}
	}
	if (kind.empty()) {
		const auto marker = error.find("HTTP ");
		if (marker != std::string::npos && marker + 8 <= error.size()) {
			const auto digits = error.substr(marker + 5, 3);
			if (std::all_of(digits.begin(), digits.end(),
						[](unsigned char c) { return std::isdigit(c); }) &&
					digits[0] >= '1' && digits[0] <= '5')
				kind = "http_" + digits;
		}
	}
	return kind.empty() ? bucket : bucket + "/" + kind;
}

std::optional<FetchResult> fetch_overpass(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, std::uint64_t seed, bool probe,
		std::string *error, const std::vector<std::string> &configured_urls)
{
	const auto query = overpass_query(bbox);
	std::size_t answered = 0;
	std::size_t truncated = 0;
	std::string last_error;
	for (const auto &endpoint : overpass_request_plan(seed, probe, configured_urls)) {
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

namespace
{
std::optional<FetchResult> fetch_osm_data_impl(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, bool use_tile_archive,
		const std::string &tiles_url, double scale, std::uint64_t seed, bool probe,
		std::string *error, const std::optional<std::string> &save_file = std::nullopt,
		const std::vector<std::string> &configured_urls = {})
{
	std::string archive_error;
	if (use_tile_archive && !tiles_url.empty()) {
		if (!osm_tiles::check_size(bbox, scale, &archive_error)) {
			std::cerr << "Error! " << archive_error << ".\n";
			if (error)
				*error = AREA_TOO_LARGE;
			return std::nullopt;
		}
		if (auto document = osm_tiles::fetch_data_from_tiles(
					bbox, tiles_url, scale, &archive_error)) {
			if (save_file)
				save_osm_json(*document, bbox, *save_file);
			return FetchResult{std::move(*document), tiles_url};
		}
		if (!archive_error.empty()) {
			std::cerr << "Warning: Tile archive unavailable (" << archive_error
					  << "); falling back to Overpass.\n";
			telemetry::send_log(telemetry::LogLevel::Warning,
					"tile_archive_fallback: " + fallback_reason_detail(archive_error));
		}
	}
	net::note_uncached("OpenStreetMap data from Overpass (its answers are never cached)");
	std::string overpass_error;
	auto result =
			fetch_overpass(bbox, fetcher, seed, probe, &overpass_error, configured_urls);
	if (result && save_file)
		save_osm_json(result->document, bbox, *save_file);
	if (!result && error) {
		if (!archive_error.empty())
			*error = "Tile archive unavailable (" + archive_error +
					 "); Overpass fallback failed (" + overpass_error + ")";
		else
			*error = std::move(overpass_error);
	}
	return result;
}
} // namespace

std::optional<FetchResult> fetch_osm_data(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, bool use_tile_archive,
		const std::string &tiles_url, std::uint64_t seed, bool probe, std::string *error)
{
	return fetch_osm_data_impl(
			bbox, fetcher, use_tile_archive, tiles_url, 1.0, seed, probe, error);
}

std::optional<FetchResult> fetch_osm_data(const geographic::LLBBox &bbox,
		const OverpassFetcher &fetcher, const arnis::Args &args, std::uint64_t seed,
		bool probe, std::string *error)
{
	if (args.osm_pbf && args.file) {
		if (error)
			*error = "--osm-pbf and --file are mutually exclusive";
		return std::nullopt;
	}
	if (args.osm_pbf) {
		const bool automatic = *args.osm_pbf == "geofabrik";
		std::string pbf_error;
		if (args.osm_pbf_url && !automatic) {
			if (error)
				*error = "--osm-pbf-url requires --osm-pbf geofabrik";
			return std::nullopt;
		}
		try {
			const auto path = osm_pbf::locate_extract(
					*args.osm_pbf, args.osm_pbf_url, bbox, &pbf_error);
			if (path) {
				auto document = osm_pbf::cached_cut_extract(*path, bbox,
						one_world::CLIP_PAD_BLOCKS / std::max(args.scale, 1e-6));
				const bool empty = document.nodes.empty() && document.ways.empty() &&
								   document.relations.empty();
				if (!empty || !automatic || net::offline()) {
					if (empty && !automatic && error)
						*error =
								"the .pbf extract holds no OpenStreetMap data for this area";
					if (args.save_json_file)
						save_osm_json(document, bbox, *args.save_json_file);
					if (error)
						error->clear();
					return FetchResult{std::move(document), path->string()};
				}
				pbf_error =
						"the Geofabrik extract holds no OpenStreetMap data for this area";
			}
		} catch (const std::exception &e) {
			pbf_error = e.what();
		}
		// Match Rust: a user-named PBF is authoritative; an automatically
		// selected Geofabrik source may fall through to tiles/Overpass online.
		if (!automatic || net::offline()) {
			if (error)
				*error = pbf_error.empty() ? "could not read the requested .pbf source"
										   : pbf_error;
			return std::nullopt;
		}
		const auto reason = pbf_error.empty() ? std::string("unknown") : pbf_error;
		std::cerr << "Warning: Region Download unavailable (" << reason
				  << "); falling back to the default source.\n";
		telemetry::send_log(telemetry::LogLevel::Warning,
				"region_download_fallback: " + fallback_reason_detail(reason));
	}
	// An explicitly named input is authoritative: do not silently replace a
	// missing or malformed local extract with remote tile/Overpass data. This
	// also makes the .pbf reader available through the same source-selection
	// entry point as the XML and JSON readers.
	if (args.file) {
		try {
			auto document = load_osm_file(*args.file);
			if (args.save_json_file)
				save_osm_json(document, bbox, *args.save_json_file);
			if (error)
				error->clear();
			return FetchResult{std::move(document), *args.file};
		} catch (const std::exception &e) {
			if (error)
				*error = e.what();
			return std::nullopt;
		}
	}
	return fetch_osm_data_impl(bbox, fetcher, !args.no_tile_archive, args.osm_tiles_url,
			args.scale, seed, probe, error, args.save_json_file, args.overpass_url);
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
	// Reverse lookup is optional metadata; an offline run has no cacheable name
	// source and should not report a network-cache miss.
	if (net::offline())
		return std::nullopt;
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
	[[maybe_unused]] auto permit = net::request_permit("Nominatim place lookup");
	const auto result = curl_easy_perform(curl);
	curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);
	curl_easy_cleanup(curl);
	if (result != CURLE_OK || status < 200 || status >= 300)
		return std::nullopt;
	return area_name_from_nominatim_json(body);
}
}
