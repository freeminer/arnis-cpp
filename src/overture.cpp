#include "overture.h"
#include "clipping.h"
#include "coordinate_system/transformation.h"
#include "overture/mvt.h"
#include "overture/cache.h"
#include "overture/pmtiles.h"
#include "../../http.h"

#if defined(USE_ARROW) && USE_ARROW
#include <arrow/api.h>
#include <arrow/io/file.h>
#include <parquet/arrow/reader.h>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace arnis::overture
{

namespace
{

// Keep the closed Overture enums aligned between the Parquet and tile
// transports.  Unknown values are deliberately discarded, matching the Rust
// interning helpers: a newly introduced provider value must not leak through
// as an unrecognised material/shape in the generated world.
std::optional<std::string> intern_roof_shape(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	const auto &v = *value;
	if (v == "gabled" || v == "gable")
		return "gabled";
	if (v == "hipped" || v == "hip")
		return "hipped";
	if (v == "flat")
		return "flat";
	if (v == "pyramidal")
		return "pyramidal";
	if (v == "dome" || v == "onion")
		return "dome";
	if (v == "skillion" || v == "shed")
		return "skillion";
	if (v == "gambrel" || v == "mansard" || v == "round" || v == "half_hipped" ||
			v == "saltbox" || v == "sawtooth" || v == "spherical")
		return v;
	return std::nullopt;
}

std::optional<std::string> intern_roof_material(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	static constexpr std::array<const char *, 14> values = {"concrete", "copper",
			"eternit", "glass", "grass", "gravel", "metal", "plastic", "roof_tiles",
			"slate", "solar_panels", "thatch", "tar_paper", "wood"};
	if (std::find(values.begin(), values.end(), *value) == values.end())
		return std::nullopt;
	return value;
}

std::optional<std::string> intern_roof_orientation(
		const std::optional<std::string> &value)
{
	if (value && (*value == "along" || *value == "across"))
		return value;
	return std::nullopt;
}

std::optional<std::string> intern_facade_material(const std::optional<std::string> &value)
{
	if (!value)
		return std::nullopt;
	static constexpr std::array<const char *, 11> values = {"brick", "cement_block",
			"clay", "concrete", "glass", "metal", "plaster", "plastic", "stone",
			"timber_framing", "wood"};
	if (std::find(values.begin(), values.end(), *value) == values.end())
		return std::nullopt;
	return value;
}

uint32_t read_u32(
		const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
	if (offset + 4 > bytes.size())
		return 0;
	if (little_endian) {
		return static_cast<uint32_t>(bytes[offset]) |
			   (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
			   (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
			   (static_cast<uint32_t>(bytes[offset + 3]) << 24);
	}
	return (static_cast<uint32_t>(bytes[offset]) << 24) |
		   (static_cast<uint32_t>(bytes[offset + 1]) << 16) |
		   (static_cast<uint32_t>(bytes[offset + 2]) << 8) |
		   static_cast<uint32_t>(bytes[offset + 3]);
}

double read_f64(const std::vector<uint8_t> &bytes, std::size_t offset, bool little_endian)
{
	std::array<uint8_t, 8> raw{};
	if (offset + 8 > bytes.size())
		return 0.0;
	if (little_endian) {
		std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset), 8, raw.begin());
	} else {
		std::reverse_copy(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
				bytes.begin() + static_cast<std::ptrdiff_t>(offset + 8), raw.begin());
	}
	double out = 0.0;
	std::memcpy(&out, raw.data(), sizeof(out));
	return out;
}

std::optional<std::vector<std::pair<double, double>>> parse_overture_wkb_polygon_impl(
		const std::vector<uint8_t> &wkb)
{
	if (wkb.size() < 13)
		return std::nullopt;

	const uint8_t byte_order = wkb[0];
	if (byte_order > 1)
		return std::nullopt;
	const bool little_endian = byte_order == 1;

	const uint32_t geom_type = read_u32(wkb, 1, little_endian);
	if ((geom_type % 1000) != 3)
		return std::nullopt;

	const uint32_t num_rings = read_u32(wkb, 5, little_endian);
	if (num_rings == 0)
		return std::nullopt;

	std::size_t offset = 9;
	if (offset + 4 > wkb.size())
		return std::nullopt;
	const uint32_t num_points = read_u32(wkb, offset, little_endian);
	offset += 4;

	const uint32_t dimension = geom_type / 1000;
	const bool has_z = dimension == 1 || dimension == 3;
	const bool has_m = dimension == 2 || dimension == 3;
	const std::size_t point_size = 16 + (has_z ? 8 : 0) + (has_m ? 8 : 0);
	if (offset + static_cast<std::size_t>(num_points) * point_size > wkb.size())
		return std::nullopt;

	std::vector<std::pair<double, double>> coords;
	coords.reserve(num_points);
	for (uint32_t i = 0; i < num_points; ++i) {
		const double lng = read_f64(wkb, offset, little_endian);
		const double lat = read_f64(wkb, offset + 8, little_endian);
		coords.emplace_back(lng, lat);
		offset += point_size;
	}
	return coords;
}

uint64_t gers_id_to_u64_impl(const std::string &gers_id)
{
	constexpr uint64_t FNV_OFFSET = 0xcbf29ce484222325ULL;
	constexpr uint64_t FNV_PRIME = 0x100000001b3ULL;

	uint64_t hash = FNV_OFFSET;
	for (unsigned char byte : gers_id) {
		hash ^= static_cast<uint64_t>(byte);
		hash *= FNV_PRIME;
	}
	return hash | OVERTURE_ID_HIGH_BIT;
}

std::string overture_class_to_osm_building_impl(const std::optional<std::string> &subtype,
		const std::optional<std::string> &clazz)
{
	if (clazz) {
		const auto &c = *clazz;
		if (c == "house" || c == "detached")
			return "house";
		if (c == "apartments" || c == "apartment")
			return "apartments";
		if (c == "residential")
			return "residential";
		if (c == "commercial")
			return "commercial";
		if (c == "retail")
			return "retail";
		if (c == "office")
			return "office";
		if (c == "industrial")
			return "industrial";
		if (c == "warehouse")
			return "warehouse";
		if (c == "garage" || c == "garages")
			return "garage";
		if (c == "shed")
			return "shed";
		if (c == "school")
			return "school";
		if (c == "hospital")
			return "hospital";
		if (c == "church" || c == "mosque" || c == "temple" || c == "synagogue")
			return "church";
		if (c == "hotel")
			return "hotel";
		if (c == "farm" || c == "barn")
			return "farm";
	}

	if (subtype) {
		const auto &s = *subtype;
		if (s == "residential")
			return "residential";
		if (s == "commercial")
			return "commercial";
		if (s == "industrial")
			return "industrial";
		if (s == "agricultural")
			return "farm";
		if (s == "civic" || s == "government" || s == "education")
			return "public";
		if (s == "medical")
			return "hospital";
		if (s == "religious")
			return "church";
		if (s == "transportation")
			return "transportation";
		if (s == "outbuilding")
			return "shed";
	}
	return "yes";
}

std::optional<std::string> attribute_string(
		const mvt::Layer &layer, const mvt::Feature &feature, const std::string &name)
{
	const auto attribute = layer.attribute(feature, name);
	if (!attribute)
		return {};
	if (const auto value = std::get_if<std::string>(&*attribute))
		return *value;
	return {};
}

std::optional<double> attribute_number(
		const mvt::Layer &layer, const mvt::Feature &feature, const std::string &name)
{
	const auto attribute = layer.attribute(feature, name);
	if (!attribute)
		return {};
	if (const auto value = std::get_if<double>(&*attribute))
		return std::isfinite(*value) ? std::optional<double>(*value) : std::nullopt;
	if (const auto value = std::get_if<std::int64_t>(&*attribute))
		return static_cast<double>(*value);
	if (const auto value = std::get_if<std::uint64_t>(&*attribute))
		return static_cast<double>(*value);
	if (const auto value = std::get_if<std::string>(&*attribute)) {
		try {
			const auto parsed = std::stod(*value);
			return std::isfinite(parsed) ? std::optional<double>(parsed) : std::nullopt;
		} catch (...) {
		}
	}
	return {};
}

bool source_is_osm(const std::optional<std::string> &geometry_source,
		const std::optional<std::string> &sources)
{
	return (geometry_source &&
				   geometry_source->find("OpenStreetMap") != std::string::npos) ||
		   (sources && sources->find("OpenStreetMap") != std::string::npos);
}

std::optional<OvertureOsmRef> parse_osm_reference(const std::string &sources)
{
	std::size_t cursor = 0;
	while ((cursor = sources.find("\"record_id\"", cursor)) != std::string::npos) {
		const auto object_begin = sources.rfind('{', cursor);
		const auto object_end = sources.find('}', cursor);
		if (object_begin == std::string::npos || object_end == std::string::npos)
			return {};
		const auto object = sources.substr(object_begin, object_end - object_begin + 1);
		if (object.find("OpenStreetMap") == std::string::npos) {
			cursor = object_end + 1;
			continue;
		}
		const auto colon = sources.find(':', cursor + 11);
		const auto quote = colon == std::string::npos ? colon : sources.find('"', colon);
		const auto end_quote =
				quote == std::string::npos ? quote : sources.find('"', quote + 1);
		if (end_quote == std::string::npos)
			return {};
		const auto value = sources.substr(quote + 1, end_quote - quote - 1);
		if (value.size() < 2 || (value[0] != 'w' && value[0] != 'W' && value[0] != 'r' &&
										value[0] != 'R'))
			return {};
		const auto at = value.find('@');
		try {
			const auto id = std::stoull(value.substr(1, at - 1));
			return OvertureOsmRef{id, value[0] == 'r' || value[0] == 'R'};
		} catch (...) {
			return {};
		}
	}
	return {};
}

double ring_area2(const std::vector<std::pair<double, double>> &ring)
{
	double sum = 0.0;
	for (std::size_t index = 0; index < ring.size(); ++index) {
		const auto &[x1, y1] = ring[index];
		const auto &[x2, y2] = ring[(index + 1) % ring.size()];
		sum += x1 * y2 - x2 * y1;
	}
	return std::abs(sum);
}

bool ring_overlaps_bbox(const std::vector<std::pair<double, double>> &ring,
		const geographic::LLBBox &bbox)
{
	if (ring.empty())
		return false;
	double min_lng = std::numeric_limits<double>::infinity();
	double max_lng = -min_lng, min_lat = min_lng, max_lat = -min_lng;
	for (const auto &[lng, lat] : ring) {
		min_lng = std::min(min_lng, lng);
		max_lng = std::max(max_lng, lng);
		min_lat = std::min(min_lat, lat);
		max_lat = std::max(max_lat, lat);
	}
	return max_lng >= bbox.min().lng() && min_lng <= bbox.max().lng() &&
		   max_lat >= bbox.min().lat() && min_lat <= bbox.max().lat();
}

#if defined(USE_ARROW) && USE_ARROW
std::optional<std::string> string_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		auto scalar = chunk->GetScalar(row);
		return scalar.ok() ? std::optional<std::string>((*scalar)->ToString())
						   : std::nullopt;
	}
	return std::nullopt;
}

std::optional<double> number_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	auto text = string_at(column, row);
	if (!text)
		return std::nullopt;
	try {
		return std::stod(*text);
	} catch (...) {
		return std::nullopt;
	}
}

std::optional<int> integer_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	auto number = number_at(column, row);
	if (!number || *number < std::numeric_limits<int>::min() ||
			*number > std::numeric_limits<int>::max())
		return std::nullopt;
	return static_cast<int>(*number);
}

std::optional<std::vector<std::uint8_t>> binary_at(
		const std::shared_ptr<arrow::ChunkedArray> &column, std::int64_t row)
{
	if (!column || row < 0 || row >= column->length())
		return std::nullopt;
	for (const auto &chunk : column->chunks()) {
		if (row >= chunk->length()) {
			row -= chunk->length();
			continue;
		}
		if (chunk->IsNull(row))
			return std::nullopt;
		if (const auto binary = std::dynamic_pointer_cast<arrow::BinaryArray>(chunk)) {
			const auto value = binary->GetView(row);
			return std::vector<std::uint8_t>(value.begin(), value.end());
		}
		if (const auto binary =
						std::dynamic_pointer_cast<arrow::LargeBinaryArray>(chunk)) {
			const auto value = binary->GetView(row);
			return std::vector<std::uint8_t>(value.begin(), value.end());
		}
		return std::nullopt;
	}
	return std::nullopt;
}
#endif

}

std::size_t overture_building_budget(const geographic::LLBBox &bbox)
{
	return std::clamp(
			static_cast<std::size_t>(bbox.area_km2() * OVERTURE_BUILDINGS_PER_KM2),
			MIN_OVERTURE_BUILDINGS, MAX_OVERTURE_BUILDINGS);
}

std::optional<std::vector<std::pair<double, double>>> parse_overture_wkb_polygon(
		const std::vector<std::uint8_t> &wkb)
{
	return parse_overture_wkb_polygon_impl(wkb);
}
std::uint64_t gers_id_to_u64(const std::string &gers_id)
{
	return gers_id_to_u64_impl(gers_id);
}

std::optional<OvertureOsmRef> parse_overture_osm_reference(const std::string &sources)
{
	return parse_osm_reference(sources);
}

std::size_t enrich_osm_buildings_from_overture(
		const std::vector<OvertureBuilding> &buildings,
		std::vector<ProcessedElement> &elements)
{
	std::unordered_map<std::uint64_t, const OvertureBuilding *> ways;
	std::unordered_map<std::uint64_t, const OvertureBuilding *> relations;
	for (const auto &building : buildings) {
		if (!building.is_osm_sourced || !building.osm_ref ||
				(!building.height && !building.num_floors))
			continue;
		const auto usable_height =
				building.height && *building.height >= 2.5 && *building.height <= 500.0;
		const auto usable_floors = building.num_floors && *building.num_floors >= 2 &&
								   *building.num_floors < 200;
		if (!usable_height && !usable_floors)
			continue;
		auto &hints = building.osm_ref->relation ? relations : ways;
		if (hints.size() < MAX_OVERTURE_OSM_HINTS || hints.contains(building.osm_ref->id))
			hints.try_emplace(building.osm_ref->id, &building);
	}
	std::size_t enriched = 0;
	for (auto &element : elements) {
		const auto &hints = element.is_relation() ? relations : ways;
		if (element.is_node())
			continue;
		const auto hint = hints.find(element.id());
		if (hint == hints.end())
			continue;
		tags_t *tags = nullptr;
		if (element.is_way())
			tags = &std::get<ProcessedWay>(static_cast<variant_t &>(element)).tags;
		else
			tags = &std::get<ProcessedRelation>(static_cast<variant_t &>(element)).tags;
		if (!(tags->contains("building") || tags->contains("building:part")) ||
				tags->contains("height") || tags->contains("building:levels"))
			continue;
		const auto &building = *hint->second;
		if (building.num_floors && *building.num_floors >= 2 &&
				*building.num_floors < 200)
			(*tags)["building:levels"] = std::to_string(*building.num_floors);
		if (building.height && *building.height >= 2.5 && *building.height <= 500.0)
			(*tags)["height"] = std::to_string(*building.height);
		(*tags)["arnis:height_source"] = "overture_maps";
		++enriched;
	}
	return enriched;
}
std::string overture_class_to_osm_building(const std::optional<std::string> &subtype,
		const std::optional<std::string> &clazz)
{
	return overture_class_to_osm_building_impl(subtype, clazz);
}

std::vector<OvertureBuilding> decode_overture_building_tile(
		const std::vector<std::uint8_t> &tile, std::uint32_t tile_x, std::uint32_t tile_y,
		std::uint8_t zoom, std::size_t maximum)
{
	std::vector<OvertureBuilding> buildings;
	if (maximum == 0)
		return buildings;
	const auto layers = mvt::decode(tile);
	if (!layers)
		return buildings;
	for (const auto &layer : *layers) {
		if (layer.name != "building" || layer.extent == 0)
			continue;
		for (const auto &feature : layer.features) {
			if (buildings.size() == maximum)
				return buildings;
			if (feature.geom_type != mvt::GEOM_POLYGON)
				continue;
			const auto id = attribute_string(layer, feature, "id");
			if (!id)
				continue;
			// Keep the tile transport's pre-budget filtering identical to the
			// Rust PMTiles path: known transient ML false positives must not
			// consume a slot that could be used by a real footprint.
			if (*id == "f8c0757e-c059-49e4-9757-7e278751926f")
				continue;
			const mvt::Ring *ring = nullptr;
			for (const auto &candidate : feature.rings)
				if (candidate.exterior() && candidate.points.size() >= 3 &&
						(!ring || candidate.area2 > ring->area2))
					ring = &candidate;
			if (!ring)
				continue;
			OvertureBuilding building;
			building.id = *id;
			const auto extent = double(layer.extent);
			building.exterior_ring.reserve(ring->points.size());
			for (const auto &[x, y] : ring->points)
				building.exterior_ring.emplace_back(
						pmtiles::tile_x_to_longitude(zoom, tile_x, double(x) / extent),
						pmtiles::tile_y_to_latitude(zoom, tile_y, double(y) / extent));
			const auto sources = attribute_string(layer, feature, "sources");
			building.is_osm_sourced = source_is_osm(
					attribute_string(layer, feature, "@geometry_source"), sources);
			if (sources)
				building.osm_ref = parse_osm_reference(*sources);
			building.height = attribute_number(layer, feature, "height");
			building.min_height = attribute_number(layer, feature, "min_height");
			if (const auto floors = attribute_number(layer, feature, "num_floors");
					floors && *floors >= std::numeric_limits<int>::min() &&
					*floors <= std::numeric_limits<int>::max())
				building.num_floors = static_cast<int>(*floors);
			building.subtype = attribute_string(layer, feature, "subtype");
			building.clazz = attribute_string(layer, feature, "class");
			building.roof_shape =
					intern_roof_shape(attribute_string(layer, feature, "roof_shape"));
			building.roof_material = intern_roof_material(
					attribute_string(layer, feature, "roof_material"));
			building.roof_orientation = intern_roof_orientation(
					attribute_string(layer, feature, "roof_orientation"));
			building.facade_color = attribute_string(layer, feature, "facade_color");
			building.roof_color = attribute_string(layer, feature, "roof_color");
			building.roof_height = attribute_number(layer, feature, "roof_height");
			building.facade_material = intern_facade_material(
					attribute_string(layer, feature, "facade_material"));
			buildings.push_back(std::move(building));
		}
	}
	return buildings;
}

BuildingSource pmtiles_building_source(pmtiles::Header header,
		std::vector<std::uint8_t> root_directory, pmtiles::RangeReader read_range,
		std::uint8_t zoom)
{
	return [header, root_directory = std::move(root_directory),
				   read_range = std::move(read_range),
				   zoom](const geographic::LLBBox &bbox, std::size_t maximum) {
		std::vector<OvertureBuilding> buildings;
		std::unordered_map<std::string, std::size_t> by_id;
		if (maximum == 0 || !read_range || zoom < header.min_zoom ||
				zoom > header.max_zoom)
			return buildings;
		const auto [min_x, min_y] =
				pmtiles::lonlat_to_tile(bbox.min().lng(), bbox.max().lat(), zoom);
		const auto [max_x, max_y] =
				pmtiles::lonlat_to_tile(bbox.max().lng(), bbox.min().lat(), zoom);
		constexpr std::uint64_t max_tiles = 4096;
		const auto width = std::uint64_t(max_x) - min_x + 1;
		const auto height = std::uint64_t(max_y) - min_y + 1;
		if (width > max_tiles || height > max_tiles || width * height > max_tiles)
			return buildings;
		for (std::uint32_t x = min_x; x <= max_x; ++x) {
			for (std::uint32_t y = min_y; y <= max_y; ++y) {
				const auto tile = pmtiles::read_tile(
						header, root_directory, zoom, x, y, read_range);
				if (!tile)
					continue;
				auto decoded = decode_overture_building_tile(*tile, x, y, zoom, maximum);
				for (auto &building : decoded) {
					if (!ring_overlaps_bbox(building.exterior_ring, bbox))
						continue;
					const auto previous = by_id.find(building.id);
					if (previous == by_id.end()) {
						by_id.emplace(building.id, buildings.size());
						buildings.push_back(std::move(building));
					} else if (ring_area2(building.exterior_ring) >
							   ring_area2(buildings[previous->second].exterior_ring)) {
						buildings[previous->second] = std::move(building);
					}
				}
				// Stop at the same point as the Rust tile collector: after a tile
				// completes the budget, but not halfway through that tile. This lets
				// duplicate copies still replace clipped footprints with their larger
				// intact ring while bounding memory for dense areas.
				if (buildings.size() >= maximum)
					break;
			}
			if (buildings.size() >= maximum)
				break;
		}
		std::sort(buildings.begin(), buildings.end(),
				[](const OvertureBuilding &a, const OvertureBuilding &b) {
					return a.id < b.id;
				});
		if (buildings.size() > maximum)
			buildings.resize(maximum);
		return buildings;
	};
}

BuildingSource http_pmtiles_building_source(
		std::string archive_url, std::uint8_t zoom, std::filesystem::path cache_directory)
{
	struct State
	{
		std::mutex mutex;
		std::optional<pmtiles::Archive> archive;
	};
	auto state = std::make_shared<State>();
	return [url = std::move(archive_url), state, zoom,
				   cache_directory = std::move(cache_directory)](
				   const geographic::LLBBox &bbox, std::size_t maximum) {
		const pmtiles::RangeReader range = [&url, &cache_directory](std::uint64_t offset,
												   std::uint64_t length)
				-> std::optional<std::vector<std::uint8_t>> {
			const auto cache_file =
					cache_directory.empty()
							? std::filesystem::path{}
							: cache_directory / "ranges" /
									  (std::to_string(offset) + "_" +
											  std::to_string(length) + ".bin");
			if (!cache_file.empty()) {
				if (const auto cached = cache::read(cache_file);
						cached && cached->size() == length)
					return cached;
			}
			const auto bytes = http_get_range(url, offset, length);
			if (bytes.size() != length)
				return {};
			auto data = std::vector<std::uint8_t>(bytes.begin(), bytes.end());
			if (!cache_file.empty())
				cache::write_atomic(cache_file, data);
			return data;
		};
		pmtiles::Archive archive;
		{
			std::lock_guard lock(state->mutex);
			if (!state->archive)
				state->archive = pmtiles::open_archive(range);
			if (!state->archive)
				return std::vector<OvertureBuilding>{};
			archive = *state->archive;
		}
		if (archive.root_directory.empty())
			return std::vector<OvertureBuilding>{};
		return pmtiles_building_source(archive.header, std::move(archive.root_directory),
				range, zoom)(bbox, maximum);
	};
}

#if defined(USE_ARROW) && USE_ARROW
std::vector<OvertureBuilding> read_overture_geoparquet(const std::filesystem::path &path,
		const geographic::LLBBox &bbox, std::size_t maximum)
{
	std::vector<OvertureBuilding> buildings;
	if (maximum == 0 || path.empty())
		return buildings;

	auto input_result = arrow::io::ReadableFile::Open(path.string());
	if (!input_result.ok())
		return buildings;
	auto reader_result =
			parquet::arrow::OpenFile(*input_result, arrow::default_memory_pool());
	if (!reader_result.ok())
		return buildings;
	auto reader = std::move(*reader_result);
	std::shared_ptr<arrow::Table> table;
	if (!reader->ReadTable(&table).ok() || !table)
		return buildings;
	auto compact = table->CombineChunks(arrow::default_memory_pool());
	if (!compact.ok())
		return buildings;
	table = *compact;

	const auto id = table->GetColumnByName("id");
	const auto geometry = table->GetColumnByName("geometry");
	if (!id || !geometry)
		return buildings;
	const auto sources = table->GetColumnByName("sources");
	const auto height = table->GetColumnByName("height");
	const auto min_height = table->GetColumnByName("min_height");
	const auto floors = table->GetColumnByName("num_floors");
	const auto subtype = table->GetColumnByName("subtype");
	const auto clazz = table->GetColumnByName("class");
	const auto roof_shape = table->GetColumnByName("roof_shape");
	const auto roof_material = table->GetColumnByName("roof_material");
	const auto roof_orientation = table->GetColumnByName("roof_orientation");
	const auto facade_color = table->GetColumnByName("facade_color");
	const auto roof_color = table->GetColumnByName("roof_color");
	const auto roof_height = table->GetColumnByName("roof_height");
	const auto facade_material = table->GetColumnByName("facade_material");

	for (std::int64_t row = 0; row < table->num_rows() && buildings.size() < maximum;
			++row) {
		auto value_id = string_at(id, row);
		auto value_geometry = binary_at(geometry, row);
		if (!value_id || !value_geometry)
			continue;
		// Filter before the source budget cap, matching Rust's Parquet path.
		if (*value_id == "f8c0757e-c059-49e4-9757-7e278751926f")
			continue;
		auto ring = parse_overture_wkb_polygon(*value_geometry);
		if (!ring || ring->size() < 3)
			continue;
		bool overlaps = false;
		for (const auto &[longitude, latitude] : *ring) {
			if (longitude >= bbox.min().lng() && longitude <= bbox.max().lng() &&
					latitude >= bbox.min().lat() && latitude <= bbox.max().lat()) {
				overlaps = true;
				break;
			}
		}
		if (!overlaps)
			continue;
		auto source_text = string_at(sources, row);
		buildings.push_back({*value_id, std::move(*ring),
				source_text && source_text->find("OpenStreetMap") != std::string::npos,
				source_text ? parse_osm_reference(*source_text) : std::nullopt,
				number_at(height, row), number_at(min_height, row),
				integer_at(floors, row), string_at(subtype, row), string_at(clazz, row),
				intern_roof_shape(string_at(roof_shape, row)),
				intern_roof_material(string_at(roof_material, row)),
				intern_roof_orientation(string_at(roof_orientation, row)),
				string_at(facade_color, row), string_at(roof_color, row),
				intern_facade_material(string_at(facade_material, row)),
				number_at(roof_height, row)});
	}
	return buildings;
}

BuildingSource parquet_building_source(std::filesystem::path path)
{
	return [path = std::move(path)](const geographic::LLBBox &bbox, std::size_t maximum) {
		return read_overture_geoparquet(path, bbox, maximum);
	};
}
#endif

std::optional<ProcessedWay> overture_building_to_way(
		const OvertureBuilding &building, const geographic::LLBBox &bbox, double scale)
{
	if (building.exterior_ring.size() < 3)
		return std::nullopt;
	const auto [transformer, rectangle] =
			coordinate_system::CoordTransformer::llbbox_to_xzbbox(bbox, scale);
	const auto base = gers_id_to_u64(building.id);
	ProcessedWay way;
	way.id = base;
	double min_lat = std::numeric_limits<double>::infinity(), max_lat = -min_lat;
	double min_lng = min_lat, max_lng = -min_lat;
	for (std::size_t i = 0; i < building.exterior_ring.size(); ++i) {
		const auto [lng, lat] = building.exterior_ring[i];
		if (!std::isfinite(lat) || !std::isfinite(lng) || lat < -90. || lat > 90. ||
				lng < -180. || lng > 180.)
			continue;
		min_lat = std::min(min_lat, lat);
		max_lat = std::max(max_lat, lat);
		min_lng = std::min(min_lng, lng);
		max_lng = std::max(max_lng, lng);
		const auto point = transformer.transform_point(geographic::LLPoint(lat, lng));
		way.nodes.push_back({base + i, tags_t{}, point.x, point.z});
	}
	if (way.nodes.size() < 3 || max_lng < bbox.min().lng() ||
			min_lng > bbox.max().lng() || max_lat < bbox.min().lat() ||
			min_lat > bbox.max().lat())
		return std::nullopt;
	if (way.nodes.front().x != way.nodes.back().x ||
			way.nodes.front().z != way.nodes.back().z)
		way.nodes.push_back({base + building.exterior_ring.size(), {},
				way.nodes.front().x, way.nodes.front().z});
	XZBBox clipbox(
			rectangle.min().x, rectangle.min().z, rectangle.max().x, rectangle.max().z);
	way.nodes = clipping::clip_way_to_bbox(way.nodes, clipbox);
	if (way.nodes.size() < 4)
		return std::nullopt;
	way.tags["building"] =
			overture_class_to_osm_building(building.subtype, building.clazz);
	const bool useful_floors = building.num_floors && *building.num_floors >= 2;
	if (building.height && *building.height > 0. && *building.height < 1000. &&
			((useful_floors && *building.height > 28.) ||
					(!useful_floors && *building.height >= 10.)))
		way.tags["height"] = std::to_string(*building.height);
	if (building.min_height && *building.min_height > 0. && *building.min_height < 1000.)
		way.tags["min_height"] = std::to_string(*building.min_height);
	if (building.num_floors && *building.num_floors >= 2 && *building.num_floors < 200)
		way.tags["building:levels"] = std::to_string(*building.num_floors);
	if (building.roof_shape) {
		static const std::unordered_map<std::string, std::string> shapes{
				{"gable", "gabled"}, {"gabled", "gabled"}, {"hip", "hipped"},
				{"hipped", "hipped"}, {"flat", "flat"}, {"pyramidal", "pyramidal"},
				{"dome", "dome"}, {"onion", "dome"}, {"skillion", "skillion"},
				{"shed", "skillion"}, {"gambrel", "gambrel"}, {"mansard", "mansard"},
				{"round", "round"}};
		way.tags["roof:shape"] = shapes.contains(*building.roof_shape)
										 ? shapes.at(*building.roof_shape)
										 : *building.roof_shape;
	}
	if (building.roof_material)
		way.tags["roof:material"] = *building.roof_material;
	if (building.roof_orientation)
		way.tags["roof:orientation"] = *building.roof_orientation;
	if (building.facade_color)
		way.tags["building:colour"] = *building.facade_color;
	if (building.roof_color)
		way.tags["roof:colour"] = *building.roof_color;
	if (building.facade_material) {
		static const std::unordered_set<std::string> materials{"brick", "cement_block",
				"clay", "concrete", "glass", "metal", "plaster", "plastic", "stone",
				"timber_framing", "wood"};
		if (materials.contains(*building.facade_material))
			way.tags["building:material"] = *building.facade_material;
	}
	if (building.roof_height && *building.roof_height >= 0.5 &&
			*building.roof_height < 100.0 &&
			(!building.height || *building.roof_height < *building.height * 0.75))
		way.tags["roof:height"] = std::to_string(*building.roof_height);
	way.tags["source"] = "overture_maps";
	return way;
}

std::vector<ProcessedElement> convert_overture_buildings(
		const std::vector<OvertureBuilding> &buildings, const geographic::LLBBox &bbox,
		double scale, bool include_osm_sourced, std::size_t maximum)
{
	std::vector<ProcessedElement> out;
	out.reserve(std::min(maximum, buildings.size()));
	for (const auto &building : buildings) {
		if (out.size() >= maximum)
			break;
		// Known transient ML false positive (temporary stage/tent at
		// Königsplatz, Munich).  Rust filters this stable GERS id before
		// converting it into a generated building.
		if (building.id == "f8c0757e-c059-49e4-9757-7e278751926f")
			continue;
		if (!include_osm_sourced && building.is_osm_sourced)
			continue;
		if (auto way = overture_building_to_way(building, bbox, scale))
			out.push_back(ProcessedElement::FromWay(*way));
	}
	return out;
}

std::vector<ProcessedElement> fetch_overture_buildings_from(const BuildingSource &source,
		const geographic::LLBBox &bbox, double scale, bool include_osm_sourced,
		std::size_t maximum)
{
	if (!source || scale <= 0)
		return {};
	if (maximum == 0)
		maximum = overture_building_budget(bbox);
	// Ask the external decoder for no more rows than the conversion cap.  Keep
	// a final cap in convert_overture_buildings in case a source ignores it.
	try {
		auto buildings = source(bbox, std::min(maximum, MAX_OVERTURE_BUILDINGS));
		return convert_overture_buildings(buildings, bbox, scale, include_osm_sourced,
				std::min(maximum, MAX_OVERTURE_BUILDINGS));
	} catch (...) {
		// Rust treats STAC/Parquet failures as optional-data failures and
		// continues with OSM.  Keep that property for host callbacks as well.
		return {};
	}
}

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug)
{
	const geographic::LLBBox bbox(min_lat, min_lng, max_lat, max_lng);
	if (scale <= 0.0 || min_lat > max_lat || min_lng > max_lng)
		return {};
	// Kept as a conservative fallback just as in Rust. An application with a
	// newer release can use http_pmtiles_building_source() directly; this API
	// remains useful for existing callers that only supplied a bounding box.
	// Keep the library usable while release discovery is unavailable, but allow
	// embedders and deployments to select the current immutable release without
	// recompiling (Rust's release selector likewise avoids hard-coding transport
	// policy into the geometry decoder).
	const char *release_env = std::getenv("ARNIS_OVERTURE_RELEASE");
	const std::string release = release_env && *release_env
										? release_env
										: cache::last_good_release(cache::cache_root())
												  .value_or("2026-08-19.0");
	if (release.empty() || release.size() > 64 ||
			release.find_first_not_of(
					"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-") !=
					std::string::npos)
		return {};
	const std::string fallback_archive =
			"https://overturemaps-extras-us-west-2.s3.us-west-2.amazonaws.com/tiles/" +
			release + "/buildings.pmtiles";
	if (debug)
		std::cerr << "Overture Maps: reading PMTiles ranges for "
				  << overture_building_budget(bbox) << " building budget." << std::endl;
	auto result = fetch_overture_buildings_from(
			http_pmtiles_building_source(fallback_archive, 14,
					cache::cache_root() / release / "tiles" / "buildings"),
			bbox, scale);
	if (!result.empty() && !release_env)
		cache::set_last_good_release(cache::cache_root(), release);
	return result;
}

std::vector<ProcessedElement> fetch_overture_buildings(double min_lat, double min_lng,
		double max_lat, double max_lng, double scale, bool debug, OvertureSource source)
{
	const geographic::LLBBox bbox(min_lat, min_lng, max_lat, max_lng);
	if (source == OvertureSource::Parquet) {
#if defined(USE_ARROW) && USE_ARROW
		// The embedding application supplies the cached partition path; the
		// default path keeps this overload deterministic for library callers.
		return fetch_overture_buildings_from(
				parquet_building_source(
						std::filesystem::path("overture-buildings.parquet")),
				bbox, scale);
#else
		if (debug)
			std::cerr
					<< "Overture Maps: GeoParquet requested but Arrow support is disabled.\n";
		return {};
#endif
	}
	// Tiles and Auto both use the range-backed PMTiles provider here. Auto's
	// higher-level caller can retry with Parquet when it has a partition path.
	return fetch_overture_buildings(min_lat, min_lng, max_lat, max_lng, scale, debug);
}

std::vector<ProcessedElement> deduplicate_against_osm(
		std::vector<ProcessedElement> overture_elements,
		const std::vector<ProcessedElement> &osm_elements)
{
	struct BBox
	{
		int min_x;
		int min_z;
		int max_x;
		int max_z;
	};

	std::vector<BBox> osm_buildings;
	for (const auto &element : osm_elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (!(way.tags.contains("building") || way.tags.contains("building:part")) ||
				way.nodes.size() < 3)
			continue;
		BBox b{way.nodes.front().x, way.nodes.front().z, way.nodes.front().x,
				way.nodes.front().z};
		for (const auto &node : way.nodes) {
			b.min_x = std::min(b.min_x, node.x);
			b.min_z = std::min(b.min_z, node.z);
			b.max_x = std::max(b.max_x, node.x);
			b.max_z = std::max(b.max_z, node.z);
		}
		osm_buildings.push_back(b);
	}

	if (osm_buildings.empty())
		return overture_elements;

	constexpr int CELL_SIZE = 64;
	int grid_min_x = osm_buildings.front().min_x;
	int grid_min_z = osm_buildings.front().min_z;
	for (const auto &b : osm_buildings) {
		grid_min_x = std::min(grid_min_x, b.min_x);
		grid_min_z = std::min(grid_min_z, b.min_z);
	}

	struct PairHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept
		{
			return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
										  static_cast<unsigned int>(p.second));
		}
	};
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, PairHash> grid;
	for (std::size_t i = 0; i < osm_buildings.size(); ++i) {
		const auto &b = osm_buildings[i];
		const int cell_x0 = (b.min_x - grid_min_x) / CELL_SIZE;
		const int cell_z0 = (b.min_z - grid_min_z) / CELL_SIZE;
		const int cell_x1 = (b.max_x - grid_min_x) / CELL_SIZE;
		const int cell_z1 = (b.max_z - grid_min_z) / CELL_SIZE;
		for (int cx = cell_x0; cx <= cell_x1; ++cx)
			for (int cz = cell_z0; cz <= cell_z1; ++cz)
				grid[{cx, cz}].push_back(i);
	}

	std::vector<ProcessedElement> out;
	out.reserve(overture_elements.size());
	for (auto &element : overture_elements) {
		if (!element.is_way()) {
			out.push_back(std::move(element));
			continue;
		}
		const auto &way = element.as_way();
		if (way.nodes.empty())
			continue;
		long long sum_x = 0;
		long long sum_z = 0;
		for (const auto &node : way.nodes) {
			sum_x += node.x;
			sum_z += node.z;
		}
		const int cx = static_cast<int>(sum_x / static_cast<long long>(way.nodes.size()));
		const int cz = static_cast<int>(sum_z / static_cast<long long>(way.nodes.size()));
		const auto key = std::make_pair(
				(cx - grid_min_x) / CELL_SIZE, (cz - grid_min_z) / CELL_SIZE);

		bool duplicate = false;
		if (auto it = grid.find(key); it != grid.end()) {
			for (std::size_t idx : it->second) {
				const auto &b = osm_buildings[idx];
				if (cx >= b.min_x && cx <= b.max_x && cz >= b.min_z && cz <= b.max_z) {
					duplicate = true;
					break;
				}
			}
		}
		if (!duplicate)
			out.push_back(std::move(element));
	}
	return out;
}

}
