#pragma once

#include <vector>
#include <optional>
#include <string>
#include <functional>
#include <filesystem>
#include <cstdint>

#include "../../arnis_adapter.h"
#include "coordinate_system/geographic/llbbox.h"
#include "overture/pmtiles.h"

namespace arnis::overture
{

inline constexpr uint64_t OVERTURE_ID_HIGH_BIT = 0x8000000000000000ULL;
inline constexpr double OVERTURE_BUILDINGS_PER_KM2 = 1000.0;
inline constexpr std::size_t MIN_OVERTURE_BUILDINGS = 100000;
inline constexpr std::size_t MAX_OVERTURE_BUILDINGS = 500000;
inline constexpr std::size_t MAX_OVERTURE_OSM_HINTS = 500000;

struct OvertureOsmRef
{
	std::uint64_t id = 0;
	bool relation = false;
	bool operator==(const OvertureOsmRef &) const = default;
};

std::size_t overture_building_budget(const geographic::LLBBox &bbox);

// Provider-neutral decoded GeoParquet row.  A host can use Arrow/Parquet,
// GDAL, or a custom range reader and pass the normalized data here.
struct OvertureBuilding
{
	std::string id;
	std::vector<std::pair<double, double>> exterior_ring; // longitude, latitude
	bool is_osm_sourced = false;
	std::optional<OvertureOsmRef> osm_ref;
	std::optional<double> height, min_height;
	std::optional<int> num_floors;
	std::optional<std::string> subtype, clazz, roof_shape, roof_material,
			roof_orientation, facade_color, roof_color, facade_material;
	std::optional<double> roof_height;
};

std::optional<OvertureOsmRef> parse_overture_osm_reference(const std::string &sources);
// Adds only missing height/level information to corresponding OSM buildings.
// Existing OSM vertical metadata always wins.
std::size_t enrich_osm_buildings_from_overture(
		const std::vector<OvertureBuilding> &, std::vector<ProcessedElement> &);

std::optional<std::vector<std::pair<double, double>>> parse_overture_wkb_polygon(
		const std::vector<std::uint8_t> &wkb);
std::uint64_t gers_id_to_u64(const std::string &gers_id);
std::string overture_class_to_osm_building(const std::optional<std::string> &subtype,
		const std::optional<std::string> &clazz);
std::optional<ProcessedWay> overture_building_to_way(
		const OvertureBuilding &, const geographic::LLBBox &, double scale);
std::vector<ProcessedElement> convert_overture_buildings(
		const std::vector<OvertureBuilding> &, const geographic::LLBBox &, double scale,
		bool include_osm_sourced = false, std::size_t maximum = 100000);

// Source seam for Arrow/GeoParquet/STAC clients.  It intentionally exchanges
// normalized rows, so the mapgen library owns geometry conversion, bounds
// clipping, limits, and OSM-source filtering while an embedding application
// owns network credentials and Parquet decoding.
using BuildingSource = std::function<std::vector<OvertureBuilding>(
		const geographic::LLBBox &, std::size_t maximum)>;

// Decode a local Overture GeoParquet partition through Apache Arrow/Parquet.
// Keeping file access at this seam means an embedding application can provide
// a cached partition, while mapgen retains Overture's schema and bbox logic.
#if defined(USE_ARROW) && USE_ARROW
std::vector<OvertureBuilding> read_overture_geoparquet(
		const std::filesystem::path &, const geographic::LLBBox &, std::size_t maximum);
BuildingSource parquet_building_source(std::filesystem::path);
#endif
std::vector<ProcessedElement> fetch_overture_buildings_from(const BuildingSource &,
		const geographic::LLBBox &, double scale, bool include_osm_sourced = false,
		std::size_t maximum = 0);

// Decode the Overture buildings MVT layer after PMTiles has supplied one
// uncompressed vector tile. A host may fetch ranges over HTTP, disk, or an
// embedded archive without coupling that transport to world generation.
std::vector<OvertureBuilding> decode_overture_building_tile(
		const std::vector<std::uint8_t> &, std::uint32_t tile_x, std::uint32_t tile_y,
		std::uint8_t zoom, std::size_t maximum);
// Adapts a PMTiles archive to the normal source seam.  `read_range` is owned by
// the embedding application, which can add HTTP, release caching, or offline
// archive access without putting transport policy in the map generator.
BuildingSource pmtiles_building_source(pmtiles::Header,
		std::vector<std::uint8_t> root_directory, pmtiles::RangeReader read_range,
		std::uint8_t zoom = 14);
// HTTP-backed PMTiles source.  The URL must point at one buildings.pmtiles
// archive; requests are byte ranges, never a whole planet-scale archive.
BuildingSource http_pmtiles_building_source(std::string archive_url,
		std::uint8_t zoom = 14, std::filesystem::path cache_directory = {});

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug);

std::vector<ProcessedElement> deduplicate_against_osm(
		std::vector<ProcessedElement> overture_elements,
		const std::vector<ProcessedElement> &osm_elements);

}
