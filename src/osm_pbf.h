#pragma once

#include "coordinate_system/geographic/llbbox.h"
#include "osm_parser.h"
#include <nlohmann/json_fwd.hpp>
#include <filesystem>
#include <optional>
#include <string>

namespace arnis::osm_pbf
{
struct GeofabrikExtract
{
	std::string name;
	std::string url;
	double polygon_area_degrees_squared = 0.0;
};

// Select the smallest Geofabrik extract whose (Multi)Polygon contains a 5x5
// sample of the requested area. A bbox-only comparison is insufficient for
// regions with overseas territories or highly non-rectangular boundaries.
std::optional<GeofabrikExtract> select_geofabrik_extract(const nlohmann::json &index,
		const geographic::LLBBox &selection, std::string *error = nullptr);

// Resolve `spec` (`geofabrik` or a local .pbf path) and keep downloaded
// extracts beneath the configured Arnis cache root. Automatic Geofabrik
// selection is capped; an explicitly named URL is caller-authoritative.
std::optional<std::filesystem::path> locate_extract(const std::string &spec,
		const std::optional<std::string> &url, const geographic::LLBBox &selection,
		std::string *error = nullptr);

osm_parser::RawOsmDocument cut_extract(
		const std::filesystem::path &path, const geographic::LLBBox &bbox, double pad_m);

// Cache root for the Geofabrik index, extracts, and later PBF area bakes.
std::filesystem::path cache_root();
} // namespace arnis::osm_pbf
