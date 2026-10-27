#include "building_suppression.h"

#include "data_processing.h"
#include "element_processing/attractions.h"
#include "element_processing/buildings.h"
#include "osm_parser.h"
#include "strict_parse.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arnis
{
namespace
{
constexpr double GRID_CELL_DEGREES = 0.0005;
constexpr double MIN_PART_COVERAGE = 0.5;
constexpr std::uint64_t RELATION_SEED_BIT = std::uint64_t{1} << 63;
constexpr double PI = 3.14159265358979323846;

struct GeoPoint
{
	double lat;
	double lon;
};

bool tag_is(const tags_t &tags, const char *key, const char *value)
{
	const auto it = tags.find(key);
	return it != tags.end() && it->second == value;
}

bool is_building_part(const tags_t &tags)
{
	const auto it = tags.find("building:part");
	if (it == tags.end())
		return false;
	constexpr std::string_view no = "no";
	return !std::equal(it->second.begin(), it->second.end(), no.begin(), no.end(),
			[](unsigned char a, unsigned char b) {
				return std::tolower(a) == std::tolower(b);
			});
}

std::string normalized_role(std::string role)
{
	const auto first = role.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = role.find_last_not_of(" \t\r\n");
	role = role.substr(first, last - first + 1);
	std::transform(role.begin(), role.end(), role.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return role;
}

bool covers_ground(const tags_t &tags)
{
	for (const char *key : {"min_height", "building:min_level"}) {
		const auto it = tags.find(key);
		if (it == tags.end())
			continue;
		const auto &text = it->second;
		const auto start = text.find_first_not_of(" \t\r\n\f\v");
		if (start == std::string::npos)
			continue;
		std::size_t end = start;
		bool dot = false;
		for (; end < text.size(); ++end) {
			const char c = text[end];
			if ((c >= '0' && c <= '9') || ((c == '+' || c == '-') && end == start))
				continue;
			if (c == '.' && !dot) {
				dot = true;
				continue;
			}
			break;
		}
		if (const auto height = strict_parse::f64(
					std::string_view(text).substr(start, end - start));
				height && *height > 0.0)
			return false;
	}
	return true;
}

std::vector<GeoPoint> geo_ring(const ProcessedWay &way)
{
	std::vector<GeoPoint> points;
	points.reserve(way.nodes.size());
	for (const auto &node : way.nodes)
		if (std::isfinite(node.latitude) && std::isfinite(node.longitude))
			points.push_back({node.latitude, node.longitude});
	return points;
}

double ring_area(const std::vector<GeoPoint> &ring)
{
	if (ring.size() < 3)
		return 0;
	const double lon_scale = std::cos(ring.front().lat * PI / 180.0);
	double area = 0;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[(i + 1) % ring.size()];
		area += (a.lon * b.lat - b.lon * a.lat) * lon_scale;
	}
	return std::abs(area * 0.5);
}

bool point_in_ring(double lat, double lon, const std::vector<GeoPoint> &ring)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[j];
		if ((a.lat > lat) != (b.lat > lat) &&
				lon < (b.lon - a.lon) * (lat - a.lat) / (b.lat - a.lat) + a.lon)
			inside = !inside;
		j = i;
	}
	return inside;
}

using GridCell = std::pair<std::int64_t, std::int64_t>;
GridCell grid_cell(double lat, double lon)
{
	return {static_cast<std::int64_t>(std::floor(lat / GRID_CELL_DEGREES)),
			static_cast<std::int64_t>(std::floor(lon / GRID_CELL_DEGREES))};
}

struct Outline
{
	const ProcessedWay *way;
	std::vector<GeoPoint> ring;
	double area;
};

struct RelationFootprint
{
	std::uint64_t id;
	std::vector<std::vector<GeoPoint>> rings;
	std::vector<std::vector<GeoPoint>> holes;
	double area;
	double center_lat;
	double center_lon;
	std::uint64_t seed = 0;
	bool ground = true;
	std::optional<std::uint64_t> way;
	bool listed_part = false;
};

std::vector<std::vector<GeoPoint>> relation_rings(const ProcessedRelation &relation,
		const std::unordered_map<std::uint64_t, const ProcessedWay *> &source_ways,
		bool inner = false)
{
	std::unordered_map<std::uint64_t, GeoPoint> node_coordinates;
	std::vector<std::vector<std::uint64_t>> open;
	auto way_refs = [](const ProcessedWay &way) {
		std::vector<std::uint64_t> refs;
		if (!way.source_node_refs.empty())
			return way.source_node_refs;
		refs.reserve(way.nodes.size());
		for (const auto &node : way.nodes)
			refs.push_back(node.id);
		return refs;
	};
	auto add_segment = [&](std::uint64_t id) {
		const auto way = source_ways.find(id);
		if (way == source_ways.end())
			return;
		auto refs = way_refs(*way->second);
		if (refs.size() < 2)
			return;
		for (const auto &node : way->second->nodes)
			if (std::isfinite(node.latitude) && std::isfinite(node.longitude))
				node_coordinates.emplace(
						node.id, GeoPoint{node.latitude, node.longitude});
		open.push_back(std::move(refs));
	};
	if (!relation.source_members.empty()) {
		// Rust evaluates suppression from the original OSM records, before relation
		// members are clipped for rendering. Keep outer/outline and inner rings
		// separate so courtyard parts do not get attributed to the enclosing block.
		for (const auto &member : relation.source_members) {
			const auto role = normalized_role(member.role);
			if (member.type != "way" ||
					(inner ? role != "inner" : (role != "outer" && role != "outline")))
				continue;
			add_segment(member.id);
		}
	} else {
		for (const auto &member : relation.members)
			if (member.role ==
					(inner ? ProcessedMemberRole::Inner : ProcessedMemberRole::Outer)) {
				const auto refs = way_refs(*member.way);
				if (refs.size() >= 2) {
					for (const auto &node : member.way->nodes)
						if (std::isfinite(node.latitude) && std::isfinite(node.longitude))
							node_coordinates.emplace(
									node.id, GeoPoint{node.latitude, node.longitude});
					open.push_back(refs);
				}
			}
	}
	std::vector<std::vector<GeoPoint>> result;
	while (!open.empty()) {
		auto ring_ids = std::move(open.back());
		open.pop_back();
		while (ring_ids.size() >= 2 && ring_ids.front() != ring_ids.back()) {
			const auto head = ring_ids.front();
			const auto tail = ring_ids.back();
			auto found = std::find_if(open.begin(), open.end(), [&](const auto &segment) {
				return segment.front() == tail || segment.back() == tail ||
					   segment.front() == head || segment.back() == head;
			});
			if (found == open.end())
				break;
			auto segment = std::move(*found);
			const auto first = segment.front();
			const auto last = segment.back();
			if (first == tail) {
				ring_ids.insert(
						ring_ids.end(), std::next(segment.begin()), segment.end());
			} else if (last == tail) {
				std::reverse(segment.begin(), segment.end());
				ring_ids.insert(
						ring_ids.end(), std::next(segment.begin()), segment.end());
			} else if (last == head) {
				segment.pop_back();
				segment.insert(segment.end(), ring_ids.begin(), ring_ids.end());
				ring_ids = std::move(segment);
			} else {
				std::reverse(segment.begin(), segment.end());
				segment.pop_back();
				segment.insert(segment.end(), ring_ids.begin(), ring_ids.end());
				ring_ids = std::move(segment);
			}
			// Rust Vec::swap_remove keeps the same candidate ordering while
			// replacing the removed segment with the last remaining segment.
			if (found + 1 != open.end())
				*found = std::move(open.back());
			open.pop_back();
		}
		if (ring_ids.size() < 4 || ring_ids.front() != ring_ids.back())
			continue;
		std::vector<GeoPoint> ring_points;
		ring_points.reserve(ring_ids.size());
		for (const auto id : ring_ids) {
			// Rust's stitch_rings uses filter_map over node_ll: unavailable
			// coordinates drop only that vertex, not the whole closed ring.
			if (const auto node = node_coordinates.find(id);
					node != node_coordinates.end())
				ring_points.push_back(node->second);
		}
		if (ring_points.size() >= 4)
			result.push_back(std::move(ring_points));
	}
	return result;
}

} // namespace

PreparedBuildingData prepare_building_data(const std::vector<ProcessedElement> &elements)
{
	PreparedBuildingData out;
	std::unordered_map<std::uint64_t, const ProcessedWay *> ways;
	std::vector<const ProcessedWay *> ordered_ways;
	std::vector<const ProcessedRelation *> building_relations;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto [it, inserted] =
					ways.emplace(element.as_way().id, &element.as_way());
			if (inserted)
				ordered_ways.push_back(it->second);
		} else if (element.is_relation() &&
				   tag_is(element.as_relation().tags, "type", "building")) {
			building_relations.push_back(&element.as_relation());
		}
	}

	// Building relation members are prepared in geographic space. The relation
	// owns a stable seed; packed facade/year/housing metadata do not change it.
	std::unordered_set<std::uint64_t> relation_ways;
	for (const auto *relation : building_relations) {
		if (!relation->source_members.empty()) {
			for (const auto &member : relation->source_members)
				if (member.type == "way")
					relation_ways.insert(member.id);
		} else {
			for (const auto &member : relation->members)
				relation_ways.insert(member.way->id);
		}
	}
	// The membership pass owns part relations explicitly listed by a
	// type=building relation. Do not let the spatial fallback reclassify one
	// from a different enclosing outline (notably a residential outline).
	std::unordered_set<std::uint64_t> listed_relation_parts;
	for (const auto *relation : building_relations)
		for (const auto &member : relation->source_members)
			if (member.type == "relation" && normalized_role(member.role) == "part")
				listed_relation_parts.insert(member.id);
	for (const auto *relation : building_relations) {
		const auto hint = [&] {
			auto value = osm_parser::building_style_hint(relation->tags);
			if (value == osm_parser::StyleHint::None) {
				if (!relation->source_members.empty()) {
					for (const auto &member : relation->source_members) {
						if (member.type != "way")
							continue;
						auto it = ways.find(member.id);
						if (it != ways.end()) {
							value = osm_parser::building_style_hint(it->second->tags);
							if (value != osm_parser::StyleHint::None)
								break;
						}
					}
				} else {
					for (const auto &member : relation->members) {
						value = osm_parser::building_style_hint(member.way->tags);
						if (value != osm_parser::StyleHint::None)
							break;
					}
				}
			}
			return value;
		}();
		double part_area = 0;
		bool has_part = false;
		bool has_relation_part = false;
		std::vector<ProcessedMemberRef> fallback_refs;
		const auto &refs = relation->source_members;
		if (refs.empty())
			for (const auto &member : relation->members)
				fallback_refs.push_back({"way", member.way->id,
						member.role == ProcessedMemberRole::Part	? "part"
						: member.role == ProcessedMemberRole::Inner ? "inner"
																	: "outer"});
		const auto &source_refs = refs.empty() ? fallback_refs : refs;
		auto relation_year = osm_parser::building_year_for_parts(relation->tags);
		bool housing = osm_parser::is_housing_type(relation->tags);
		if (!relation_year || !housing) {
			for (const auto &member : source_refs) {
				if ((normalized_role(member.role) != "outer" &&
							normalized_role(member.role) != "outline") ||
						member.type != "way")
					continue;
				const auto way = ways.find(member.id);
				if (way == ways.end())
					continue;
				if (!relation_year)
					relation_year =
							osm_parser::building_year_for_parts(way->second->tags);
				if (!housing)
					housing = osm_parser::is_housing_type(way->second->tags);
				if (relation_year && housing)
					break;
			}
		}
		for (const auto &member : source_refs) {
			if (normalized_role(member.role) != "part" ||
					(member.type != "way" && member.type != "relation"))
				continue;
			has_part = true;
			if (member.type == "relation") {
				has_relation_part = true;
				if (housing)
					out.housing_relation_parts.insert(member.id);
				continue;
			}
			out.part_groups[member.id] = osm_parser::seed_with_metadata(
					RELATION_SEED_BIT | relation->id, hint, relation_year, housing);
			const auto way = ways.find(member.id);
			if (way != ways.end() && covers_ground(way->second->tags))
				part_area += ring_area(geo_ring(*way->second));
		}
		if (!has_part)
			continue;
		for (const auto &member : source_refs) {
			const auto role = normalized_role(member.role);
			if ((role != "outer" && role != "outline") ||
					(member.type != "way" && member.type != "relation"))
				continue;
			if (member.type == "way" && !has_relation_part) {
				const auto way = ways.find(member.id);
				if (way != ways.end()) {
					const double area = ring_area(geo_ring(*way->second));
					if (area > 0 && part_area / area < MIN_PART_COVERAGE)
						continue;
				}
			}
			out.outline_suppression.emplace(member.type, member.id);
		}
	}

	// Match Rust's relation-less S3DB pass: only closed way footprints qualify;
	// candidate outlines are indexed by geographic cells to keep lookup bounded.
	std::vector<Outline> outlines;
	std::vector<const ProcessedWay *> parts;
	for (const auto *way : ordered_ways) {
		const auto id = way->id;
		const auto &refs = way->source_node_refs;
		const bool closed =
				refs.empty() ? way->nodes.size() >= 4 &&
									   way->nodes.front().id == way->nodes.back().id
							 : refs.size() >= 4 && refs.front() == refs.back();
		if (relation_ways.contains(id) || !closed)
			continue;
		const bool part = is_building_part(way->tags);
		if (!part && !way->tags.contains("building"))
			continue;
		if (part) {
			parts.push_back(way);
			continue;
		}
		auto ring = geo_ring(*way);
		const auto area = ring_area(ring);
		if (area > 0)
			outlines.push_back({way, std::move(ring), area});
	}
	std::map<GridCell, std::vector<std::size_t>> grid;
	for (std::size_t i = 0; i < outlines.size(); ++i) {
		const auto &ring = outlines[i].ring;
		if (ring.empty())
			continue;
		double min_lat = ring.front().lat, max_lat = min_lat;
		double min_lon = ring.front().lon, max_lon = min_lon;
		for (const auto &point : ring) {
			min_lat = std::min(min_lat, point.lat);
			max_lat = std::max(max_lat, point.lat);
			min_lon = std::min(min_lon, point.lon);
			max_lon = std::max(max_lon, point.lon);
		}
		const auto [lat0, lon0] = grid_cell(min_lat, min_lon);
		const auto [lat1, lon1] = grid_cell(max_lat, max_lon);
		for (auto lat = lat0; lat <= lat1; ++lat)
			for (auto lon = lon0; lon <= lon1; ++lon)
				grid[{lat, lon}].push_back(i);
	}
	std::vector<double> covered(outlines.size(), 0.0);
	for (const auto *part : parts) {
		auto ring = geo_ring(*part);
		const auto area = ring_area(ring);
		if (area <= 0 || ring.empty())
			continue;
		double lat = 0, lon = 0;
		for (const auto &point : ring) {
			lat += point.lat;
			lon += point.lon;
		}
		lat /= ring.size();
		lon /= ring.size();
		const auto candidates = grid.find(grid_cell(lat, lon));
		if (candidates == grid.end())
			continue;
		std::optional<std::size_t> best;
		for (auto index : candidates->second) {
			if (!point_in_ring(lat, lon, outlines[index].ring))
				continue;
			if (covers_ground(part->tags))
				covered[index] += area;
			if (!best || std::pair{outlines[index].area, outlines[index].way->id} <
								 std::pair{outlines[*best].area, outlines[*best].way->id})
				best = index;
		}
		if (best) {
			auto hint = osm_parser::building_style_hint(outlines[*best].way->tags);
			if (hint == osm_parser::StyleHint::None)
				hint = osm_parser::building_style_hint(part->tags);
			out.part_groups[part->id] =
					osm_parser::seed_with_metadata(outlines[*best].way->id, hint,
							osm_parser::building_year_for_parts(outlines[*best].way->tags)
									? osm_parser::building_year_for_parts(
											  outlines[*best].way->tags)
									: osm_parser::building_year_for_parts(part->tags),
							osm_parser::is_housing_type(outlines[*best].way->tags));
		}
	}
	for (std::size_t i = 0; i < outlines.size(); ++i)
		if (covered[i] / outlines[i].area >= MIN_PART_COVERAGE)
			out.outline_suppression.emplace("way", outlines[i].way->id);
	// Relations whose boundaries are split over member ways need stitched rings. Also
	// include closed building:part ways: Rust's spatial pass treats both sources alike,
	// excludes courtyard holes, and assigns a way to its smallest containing outline.
	std::vector<RelationFootprint> relation_outlines, relation_parts;
	for (const auto &element : elements) {
		if (!element.is_relation())
			continue;
		const auto &relation = element.as_relation();
		const auto type = relation.tags.get("type");
		if (type != "multipolygon" && type != "building")
			continue;
		const bool part = is_building_part(relation.tags);
		const bool outline = !part && relation.tags.contains("building");
		if (!part && !outline)
			continue;
		auto rings = relation_rings(relation, ways);
		auto holes = outline ? relation_rings(relation, ways, true)
							 : std::vector<std::vector<GeoPoint>>{};
		double area = 0, lat_sum = 0, lon_sum = 0;
		std::size_t count = 0;
		for (const auto &ring : rings) {
			area += ring_area(ring);
			for (const auto &point : ring) {
				lat_sum += point.lat;
				lon_sum += point.lon;
				++count;
			}
		}
		if (area <= 0 || count == 0)
			continue;
		RelationFootprint footprint{relation.id, std::move(rings), std::move(holes), area,
				lat_sum / static_cast<double>(count),
				lon_sum / static_cast<double>(count),
				outline ? osm_parser::seed_with_metadata(RELATION_SEED_BIT | relation.id,
								  osm_parser::building_style_hint(relation.tags),
								  osm_parser::building_year_for_parts(relation.tags),
								  osm_parser::is_housing_type(relation.tags))
						: 0,
				covers_ground(relation.tags), std::nullopt,
				part && listed_relation_parts.contains(relation.id)};
		(outline ? relation_outlines : relation_parts).push_back(std::move(footprint));
	}
	for (const auto *way : ordered_ways) {
		if (!is_building_part(way->tags) || buildings::is_underground_building(way->tags))
			continue;
		const auto &refs = way->source_node_refs;
		const bool closed =
				refs.empty() ? way->nodes.size() >= 4 &&
									   way->nodes.front().id == way->nodes.back().id
							 : refs.size() >= 4 && refs.front() == refs.back();
		if (!closed)
			continue;
		auto ring = geo_ring(*way);
		const double area = ring_area(ring);
		if (area <= 0 || ring.empty())
			continue;
		double lat = 0, lon = 0;
		for (const auto &point : ring) {
			lat += point.lat;
			lon += point.lon;
		}
		lat /= ring.size();
		lon /= ring.size();
		relation_parts.push_back({way->id, {std::move(ring)}, {}, area, lat, lon, 0,
				covers_ground(way->tags), way->id});
	}
	std::map<GridCell, std::vector<std::size_t>> relation_grid;
	for (std::size_t i = 0; i < relation_outlines.size(); ++i) {
		const auto &outline = relation_outlines[i];
		double min_lat = std::numeric_limits<double>::infinity();
		double min_lon = std::numeric_limits<double>::infinity();
		double max_lat = -std::numeric_limits<double>::infinity();
		double max_lon = -std::numeric_limits<double>::infinity();
		for (const auto &ring : outline.rings)
			for (const auto &point : ring) {
				min_lat = std::min(min_lat, point.lat);
				min_lon = std::min(min_lon, point.lon);
				max_lat = std::max(max_lat, point.lat);
				max_lon = std::max(max_lon, point.lon);
			}
		if (!std::isfinite(min_lat) || !std::isfinite(min_lon) ||
				!std::isfinite(max_lat) || !std::isfinite(max_lon))
			continue;
		const auto [lat0, lon0] = grid_cell(min_lat, min_lon);
		const auto [lat1, lon1] = grid_cell(max_lat, max_lon);
		for (auto lat = lat0; lat <= lat1; ++lat)
			for (auto lon = lon0; lon <= lon1; ++lon)
				relation_grid[{lat, lon}].push_back(i);
	}
	std::vector<double> relation_coverage(relation_outlines.size(), 0.0);
	for (const auto &part : relation_parts) {
		const auto candidates =
				relation_grid.find(grid_cell(part.center_lat, part.center_lon));
		if (candidates == relation_grid.end())
			continue;
		std::optional<std::size_t> best;
		for (const auto i : candidates->second) {
			const auto &outline = relation_outlines[i];
			if (!std::any_of(outline.rings.begin(), outline.rings.end(),
						[&](const auto &ring) {
							return point_in_ring(part.center_lat, part.center_lon, ring);
						}))
				continue;
			if (part.way && std::any_of(outline.holes.begin(), outline.holes.end(),
									[&](const auto &ring) {
										return point_in_ring(
												part.center_lat, part.center_lon, ring);
									}))
				continue;
			if (part.ground)
				relation_coverage[i] += part.area;
			if (!best || std::pair{outline.area, outline.id} <
								 std::pair{relation_outlines[*best].area,
										 relation_outlines[*best].id})
				best = i;
		}
		if (best) {
			const auto &outline = relation_outlines[*best];
			if (part.way) {
				out.part_groups.try_emplace(*part.way, outline.seed);
			} else if (!part.listed_part &&
					   osm_parser::residential_group_from_seed(outline.seed)) {
				// Rust's spatial relation-part pass carries residential context for
				// the relation itself; unlike a part way it cannot use PartGroups.
				out.housing_relation_parts.insert(part.id);
			}
		}
	}
	for (std::size_t i = 0; i < relation_outlines.size(); ++i)
		if (relation_coverage[i] / relation_outlines[i].area >= MIN_PART_COVERAGE)
			out.suppressed_relations.insert(relation_outlines[i].id);

	// Rust's osm_parser::suppress_ride_parts runs before ways are clipped into
	// chunk views. Keep that ownership here in the shared prepared data so every
	// chunk sees the same complete attraction and building-part geometry.
	std::vector<const ProcessedWay *> ride_ways;
	for (const auto *way : ordered_ways) {
		if (attractions::replaces_parts(way->tags)) {
			out.outline_suppression.erase({"way", way->id});
			ride_ways.push_back(way);
		}
	}
	// Match Rust's early return: most areas have no rides, so avoid a second scan
	// and a temporary list of every building part in those areas.
	if (!ride_ways.empty()) {
		std::vector<const ProcessedWay *> ride_parts;
		for (const auto *way : ordered_ways)
			if (is_building_part(way->tags) && !attractions::claims(way->tags))
				ride_parts.push_back(way);
		// Rust returns when the area has no candidate parts, before copying any
		// ride rings into geographic outlines. Keep that cheap path for areas with
		// attractions but no buildings.
		if (!ride_parts.empty()) {
			std::vector<std::vector<GeoPoint>> ride_outlines;
			ride_outlines.reserve(ride_ways.size());
			for (const auto *way : ride_ways) {
				auto ring = geo_ring(*way);
				if (ring.size() >= 4)
					ride_outlines.push_back(std::move(ring));
			}
			for (const auto *way : ride_parts) {
				const auto ring = geo_ring(*way);
				if (ring.empty())
					continue;
				double latitude = 0.0, longitude = 0.0;
				for (const auto &point : ring) {
					latitude += point.lat;
					longitude += point.lon;
				}
				latitude /= static_cast<double>(ring.size());
				longitude /= static_cast<double>(ring.size());
				for (const auto &outline : ride_outlines)
					if (point_in_ring(latitude, longitude, outline)) {
						out.outline_suppression.emplace("way", way->id);
						break;
					}
			}
		}
	}

	// A material/color mapped on a masonry outline is the building-wide facade;
	// untagged building:parts inherit it, while explicit part palettes still win.
	static constexpr std::array<const char *, 5> facade_keys{"building:material",
			"building:facade:material", "facade:material", "building:colour", "colour"};
	const auto normalized = [](std::string value) {
		std::string result;
		for (unsigned char c : value)
			if (!std::isspace(c) && c != '_' && c != '-')
				result.push_back(static_cast<char>(std::tolower(c)));
		return result;
	};
	const auto masonry = [&](const tags_t &tags) {
		std::string material = tags.get("building:material");
		if (material.empty())
			material = tags.get("building:facade:material");
		const auto value = normalized(std::move(material));
		return value == "brick" || value == "bricks" || value == "stone" ||
			   value == "naturalstone" || value == "limestone" || value == "sandstone" ||
			   value == "marble" || value == "granite" || value == "plaster" ||
			   value == "stucco" || value == "render" || value == "concrete";
	};
	const auto facade_of = [&](const tags_t &tags) {
		tags_t result;
		for (const auto *key : facade_keys)
			if (const auto found = tags.find(key); found != tags.end())
				result.emplace(found->first, found->second);
		return result;
	};
	std::unordered_set<std::uint64_t> groups;
	for (const auto &[_, seed] : out.part_groups)
		groups.insert(osm_parser::group_key(seed));
	std::unordered_map<std::uint64_t, tags_t> facade_by_group;
	std::unordered_map<std::uint64_t, std::uint64_t> relation_outline_group;
	for (const auto *relation : building_relations) {
		const auto key = RELATION_SEED_BIT | relation->id;
		if (!groups.contains(key))
			continue;
		if (masonry(relation->tags)) {
			facade_by_group.emplace(key, facade_of(relation->tags));
			continue;
		}
		if (!relation->source_members.empty()) {
			for (const auto &member : relation->source_members)
				if (member.type == "way" && (normalized(member.role) == "outline" ||
													normalized(member.role) == "outer"))
					relation_outline_group.emplace(member.id, key);
		} else {
			for (const auto &member : relation->members)
				if (member.role == ProcessedMemberRole::Outer)
					relation_outline_group.emplace(member.way->id, key);
		}
	}
	for (const auto *way : ordered_ways) {
		if (!masonry(way->tags))
			continue;
		if (groups.contains(way->id))
			facade_by_group.insert_or_assign(way->id, facade_of(way->tags));
		else if (const auto relation = relation_outline_group.find(way->id);
				 relation != relation_outline_group.end())
			facade_by_group.try_emplace(relation->second, facade_of(way->tags));
	}
	for (const auto *way : ordered_ways) {
		const auto group = out.part_groups.find(way->id);
		if (group == out.part_groups.end())
			continue;
		const auto facade = facade_by_group.find(osm_parser::group_key(group->second));
		if (facade == facade_by_group.end() ||
				std::any_of(std::begin(facade_keys), std::end(facade_keys),
						[&](const char *key) { return way->tags.contains(key); }))
			continue;
		out.inherited_facade_tags.emplace(way->id, facade->second);
	}

	return out;
}
} // namespace arnis
