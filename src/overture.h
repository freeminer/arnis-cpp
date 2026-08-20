#pragma once

#include <vector>
#include <optional>
#include <string>
#include <functional>

#include "../../arnis_adapter.h"
#include "coordinate_system/geographic/llbbox.h"

namespace arnis::overture
{

inline constexpr uint64_t OVERTURE_ID_HIGH_BIT = 0x8000000000000000ULL;
inline constexpr double OVERTURE_BUILDINGS_PER_KM2 = 1000.0;
inline constexpr std::size_t MIN_OVERTURE_BUILDINGS = 100000;
inline constexpr std::size_t MAX_OVERTURE_BUILDINGS = 500000;

std::size_t overture_building_budget(const geographic::LLBBox &bbox);

// Provider-neutral decoded GeoParquet row.  A host can use Arrow/Parquet,
// GDAL, or a custom range reader and pass the normalized data here.
struct OvertureBuilding
{
	std::string id;
	std::vector<std::pair<double, double>> exterior_ring; // longitude, latitude
	bool is_osm_sourced = false;
	std::optional<double> height, min_height;
	std::optional<int> num_floors;
	std::optional<std::string> subtype, clazz, roof_shape, roof_material,
			roof_orientation, facade_color, roof_color;
};

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
std::vector<ProcessedElement> fetch_overture_buildings_from(const BuildingSource &,
		const geographic::LLBBox &, double scale, bool include_osm_sourced = false,
		std::size_t maximum = 0);

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug);

std::vector<ProcessedElement> deduplicate_against_osm(
		std::vector<ProcessedElement> overture_elements,
		const std::vector<ProcessedElement> &osm_elements);

}
