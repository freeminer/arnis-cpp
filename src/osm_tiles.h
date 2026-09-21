#pragma once

#include "osm_parser.h"
#include "coordinate_system/geographic/llbbox.h"
#include "elevation/cache.h"
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis::osm_tiles
{
inline constexpr const char *DEFAULT_OSM_TILES_URL = "https://tiles.arnisproject.com/v1";
std::filesystem::path cache_root();
elevation::CacheClearStats clear_osm_tiles_cache();
inline constexpr std::uint8_t ZOOM = 13;
inline constexpr double COORD_SCALE = 1e6;
inline constexpr std::uint64_t SYNTHETIC_ID_BASE = std::uint64_t{1} << 62;

struct DecodedNode
{
	std::uint64_t id = 0;
	std::int32_t lat = 0, lon = 0;
	tags_t tags;
};
struct DecodedWay
{
	std::uint64_t id = 0;
	bool closed = false;
	tags_t tags;
	std::vector<std::pair<std::int32_t, std::int32_t>> points;
};
struct DecodedRelationMember
{
	std::uint64_t ref = 0;
	std::string role;
};
struct DecodedRelation
{
	std::uint64_t id = 0;
	tags_t tags;
	std::vector<DecodedRelationMember> members;
};
struct DecodedTile
{
	std::vector<DecodedNode> nodes;
	std::vector<DecodedWay> ways;
	std::vector<DecodedRelation> relations;
};

// Decode one compressed AOT1 payload (the tile archive format used by arnis-tiles).
// Errors are returned instead of allowing corrupt archive data to escape into the
// geographic parser.
bool decode(const std::vector<std::uint8_t> &payload, DecodedTile &out,
		std::string *error = nullptr);

// Convert decoded tiles accumulated from multiple archives into the same raw OSM
// document shape produced by Overpass. Duplicate records are intentionally resolved
// by first occurrence, matching the Rust HashMap::entry(...).or_insert behavior.
osm_parser::RawOsmDocument assemble(const std::vector<DecodedTile> &tiles);

// Fetch and decode all z13 archive tiles intersecting a geographic bbox.  A
// null result means the archive was unavailable or had no matching tiles;
// callers can then use the existing Overpass fallback.
std::optional<osm_parser::RawOsmDocument> fetch_data_from_tiles(
		const geographic::LLBBox &bbox, const std::string &base_url,
		std::string *error = nullptr);

} // namespace arnis::osm_tiles
