#pragma once

#include "coordinate_system/geographic/llbbox.h"
#include "osm_parser.h"
#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace arnis::retrieve_data
{
struct OverpassEndpoint { std::string url; bool fallback = false; unsigned timeout_seconds = 360; };
struct FetchResult { osm_parser::RawOsmDocument document; std::string endpoint; };
using OverpassFetcher = std::function<std::optional<std::string>(const OverpassEndpoint &, const std::string &)>;
std::string url_host(const std::string &url);
bool is_osm_xml_path(const std::filesystem::path &path);
osm_parser::RawOsmDocument load_osm_file(const std::filesystem::path &path);
std::string overpass_query(const geographic::LLBBox &bbox);
std::vector<OverpassEndpoint> overpass_request_plan(std::uint64_t seed = 0, bool probe_official_first = false);
std::optional<FetchResult> fetch_overpass(const geographic::LLBBox &, const OverpassFetcher &, std::uint64_t seed = 0, bool probe_official_first = false);
std::optional<std::string> area_name_from_nominatim_json(const std::string &body);
}
