#pragma once

#include "../osm_parser.h"
#include "../retrieve_data.h"
#include "types.h"

#include <nlohmann/json.hpp>

#include <optional>
#include <string>
#include <vector>

namespace arnis
{
struct Args;
}

namespace arnis::mapillary
{
struct OsmFetchConfig
{
	BBox bbox;
	double margin_m = 60.0;
	std::optional<std::string> osm_tiles = arnis::osm_tiles::DEFAULT_OSM_TILES_URL;
	std::vector<std::string> overpass = {retrieve_data::ARNIS_OVERPASS_URL};
	retrieve_data::OverpassFetcher fetcher;
};

OsmFetchConfig osm_fetch_config(const BBox &bbox, const arnis::Args &args,
		retrieve_data::OverpassFetcher fetcher, double margin_m = 60.0);
BBox pad_osm_bbox(const BBox &bbox, double margin_m);
nlohmann::json buildings_as_overpass_json(
		const osm_parser::RawOsmDocument &document, const BBox &bbox);
std::optional<nlohmann::json> fetch_osm(
		const OsmFetchConfig &config, std::string *error = nullptr);
} // namespace arnis::mapillary
