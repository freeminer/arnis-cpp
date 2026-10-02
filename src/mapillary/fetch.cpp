#include "fetch.h"

#include "../args.h"
#include "../osm_tiles.h"
#include "../coordinate_system/geographic/llbbox.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace arnis::mapillary
{
namespace
{
constexpr double METRES_PER_DEGREE_LAT = 111195.0;

struct Extent
{
	double min_lat = std::numeric_limits<double>::max();
	double min_lon = std::numeric_limits<double>::max();
	double max_lat = std::numeric_limits<double>::lowest();
	double max_lon = std::numeric_limits<double>::lowest();
	bool valid = false;

	void include(double lat, double lon)
	{
		min_lat = std::min(min_lat, lat);
		min_lon = std::min(min_lon, lon);
		max_lat = std::max(max_lat, lat);
		max_lon = std::max(max_lon, lon);
		valid = true;
	}
	bool intersects(const BBox &bbox) const
	{
		return valid && min_lat <= bbox.max_lat && max_lat >= bbox.min_lat &&
			   min_lon <= bbox.max_lon && max_lon >= bbox.min_lon;
	}
};

nlohmann::json tags_json(const tags_t &tags)
{
	return static_cast<const std::unordered_map<std::string, std::string> &>(tags);
}

std::string buildings_query(const BBox &bbox)
{
	std::ostringstream out;
	out << "[out:json][timeout:60][bbox:" << bbox.min_lat << ',' << bbox.min_lon << ','
		<< bbox.max_lat << ',' << bbox.max_lon
		<< "];\n(\n  way[\"building\"];\n  relation[\"building\"];\n);\nout body; >; out skel qt;";
	return out.str();
}
} // namespace

OsmFetchConfig osm_fetch_config(const BBox &bbox, const arnis::Args &args,
		retrieve_data::OverpassFetcher fetcher, double margin_m)
{
	OsmFetchConfig config;
	config.bbox = bbox;
	config.margin_m = margin_m;
	config.osm_tiles = args.no_tile_archive
							   ? std::nullopt
							   : std::optional<std::string>(args.osm_tiles_url);
	config.fetcher = std::move(fetcher);
	return config;
}

BBox pad_osm_bbox(const BBox &bbox, double margin_m)
{
	const double dlat = margin_m / METRES_PER_DEGREE_LAT;
	const double radians = 3.14159265358979323846 / 180.0;
	const double cos_lat = std::max(
			1e-6, std::abs(std::cos(0.5 * (bbox.min_lat + bbox.max_lat) * radians)));
	const double dlon = margin_m / (METRES_PER_DEGREE_LAT * cos_lat);
	return {std::max(-89.9, bbox.min_lat - dlat), std::max(-180.0, bbox.min_lon - dlon),
			std::min(89.9, bbox.max_lat + dlat), std::min(180.0, bbox.max_lon + dlon)};
}

nlohmann::json buildings_as_overpass_json(
		const osm_parser::RawOsmDocument &document, const BBox &bbox)
{
	std::unordered_map<std::uint64_t, std::pair<double, double>> coordinates;
	coordinates.reserve(document.nodes.size());
	for (const auto &node : document.nodes)
		coordinates.emplace(node.id, std::pair{node.lat, node.lon});
	std::unordered_map<std::uint64_t, const osm_parser::RawWay *> ways_by_id;
	ways_by_id.reserve(document.ways.size());
	for (const auto &way : document.ways)
		ways_by_id.emplace(way.id, &way);

	auto way_extent = [&](const osm_parser::RawWay &way) {
		Extent extent;
		for (const auto id : way.node_refs)
			if (const auto it = coordinates.find(id); it != coordinates.end())
				extent.include(it->second.first, it->second.second);
		return extent;
	};
	auto relation_extent = [&](const osm_parser::RawRelation &relation) {
		Extent extent;
		for (const auto &member : relation.members) {
			if (member.type != "way")
				continue;
			const auto it = ways_by_id.find(member.ref);
			if (it == ways_by_id.end())
				continue;
			const auto part = way_extent(*it->second);
			if (part.valid) {
				extent.include(part.min_lat, part.min_lon);
				extent.include(part.max_lat, part.max_lon);
			}
		}
		return extent;
	};

	std::unordered_set<std::uint64_t> selected_ways;
	std::vector<const osm_parser::RawRelation *> selected_relations;
	for (const auto &way : document.ways)
		if (way.tags.contains("building") && way_extent(way).intersects(bbox))
			selected_ways.insert(way.id);
	for (const auto &relation : document.relations) {
		if (!relation.tags.contains("building") ||
				!relation_extent(relation).intersects(bbox))
			continue;
		selected_relations.push_back(&relation);
		for (const auto &member : relation.members)
			if (member.type == "way")
				selected_ways.insert(member.ref);
	}

	std::unordered_set<std::uint64_t> selected_nodes;
	std::vector<nlohmann::json> way_json;
	for (const auto &way : document.ways) {
		if (!selected_ways.contains(way.id))
			continue;
		selected_nodes.insert(way.node_refs.begin(), way.node_refs.end());
		way_json.push_back({{"type", "way"}, {"id", way.id}, {"nodes", way.node_refs},
				{"tags", tags_json(way.tags)}});
	}

	nlohmann::json elements = nlohmann::json::array();
	for (const auto &node : document.nodes)
		if (selected_nodes.contains(node.id))
			elements.push_back({{"type", "node"}, {"id", node.id}, {"lat", node.lat},
					{"lon", node.lon}});
	for (auto &way : way_json)
		elements.push_back(std::move(way));
	for (const auto *relation : selected_relations) {
		nlohmann::json members = nlohmann::json::array();
		for (const auto &member : relation->members)
			members.push_back(
					{{"type", member.type}, {"ref", member.ref}, {"role", member.role}});
		elements.push_back({{"type", "relation"}, {"id", relation->id},
				{"members", std::move(members)}, {"tags", tags_json(relation->tags)}});
	}
	return {{"elements", std::move(elements)}};
}

std::optional<nlohmann::json> fetch_osm(const OsmFetchConfig &config, std::string *error)
{
	if (error)
		error->clear();
	const auto bbox = pad_osm_bbox(config.bbox, config.margin_m);
	if (config.osm_tiles && !config.osm_tiles->empty()) {
		const geographic::LLBBox archive_bbox(
				bbox.min_lat, bbox.min_lon, bbox.max_lat, bbox.max_lon);
		std::string archive_error;
		if (auto document = osm_tiles::fetch_data_from_tiles_quietly(
					archive_bbox, *config.osm_tiles, &archive_error))
			return buildings_as_overpass_json(*document, bbox);
		if (error)
			*error = "tile archive unavailable: " + archive_error;
	}
	if (!config.fetcher || config.overpass.empty()) {
		if (error && error->empty())
			*error = "no OSM source for the facade buildings";
		return std::nullopt;
	}
	const auto query = buildings_query(bbox);
	std::string last_error;
	for (const auto &url : config.overpass) {
		retrieve_data::OverpassEndpoint endpoint{url, false, 60};
		const auto response = config.fetcher(endpoint, query);
		if (!response || response->empty()) {
			last_error = url + ": empty response";
			continue;
		}
		const auto json = nlohmann::json::parse(*response, nullptr, false);
		if (!json.is_discarded() && json.is_object() && json.contains("elements")) {
			if (error)
				error->clear();
			return std::optional<nlohmann::json>{json};
		}
		last_error = url + ": invalid Overpass JSON";
	}
	if (error)
		*error = last_error.empty() ? "every Overpass endpoint failed" : last_error;
	return std::nullopt;
}
} // namespace arnis::mapillary
