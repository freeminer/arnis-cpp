#include "building_suppression.h"

#include "data_processing.h"
#include "osm_parser.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
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
		try {
			if (std::stod(text.substr(start, end - start)) > 0.0)
				return false;
		} catch (const std::exception &) {
		}
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
	double area;
	double center_lat;
	double center_lon;
};

std::vector<std::vector<GeoPoint>> relation_rings(const ProcessedRelation &relation)
{
	std::vector<std::vector<const ProcessedWay *>> open;
	for (const auto &member : relation.members)
		if (member.role == ProcessedMemberRole::Outer && member.way.nodes.size() >= 2)
			open.push_back({&member.way});
	std::vector<std::vector<GeoPoint>> result;
	while (!open.empty()) {
		auto segments = std::move(open.back());
		open.pop_back();
		std::vector<const ProcessedNode *> nodes;
		for (const auto *way : segments)
			for (const auto &node : way->nodes)
				nodes.push_back(&node);
		while (nodes.size() >= 2 && nodes.front()->id != nodes.back()->id) {
			const auto head = nodes.front()->id;
			const auto tail = nodes.back()->id;
			auto found = open.end();
			bool reverse = false;
			for (auto it = open.begin(); it != open.end(); ++it) {
				const auto *way = it->front();
				if (way->nodes.front().id == tail || way->nodes.back().id == tail) {
					found = it;
					reverse = way->nodes.back().id == tail;
					break;
				}
				if (way->nodes.front().id == head || way->nodes.back().id == head) {
					found = it;
					reverse = way->nodes.front().id == head;
					break;
				}
			}
			if (found == open.end())
				break;
			auto *way = found->front();
			if (way->nodes.front().id == tail || way->nodes.back().id == tail) {
				if (!reverse)
					for (std::size_t i = 1; i < way->nodes.size(); ++i)
						nodes.push_back(&way->nodes[i]);
				else
					for (std::size_t i = way->nodes.size() - 1; i-- > 0;)
						nodes.push_back(&way->nodes[i]);
			} else {
				std::vector<const ProcessedNode *> prefix;
				if (!reverse) {
					for (std::size_t i = 0; i + 1 < way->nodes.size(); ++i)
						prefix.push_back(&way->nodes[i]);
				} else {
					for (std::size_t i = way->nodes.size(); i-- > 1;)
						prefix.push_back(&way->nodes[i]);
				}
				nodes.insert(nodes.begin(), prefix.begin(), prefix.end());
			}
			open.erase(found);
		}
		if (nodes.size() < 4 || nodes.front()->id != nodes.back()->id)
			continue;
		std::vector<GeoPoint> ring;
		ring.reserve(nodes.size());
		for (const auto *node : nodes) {
			if (!std::isfinite(node->latitude) || !std::isfinite(node->longitude)) {
				ring.clear();
				break;
			}
			ring.push_back({node->latitude, node->longitude});
		}
		if (ring.size() >= 4)
			result.push_back(std::move(ring));
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
				   tag_is(element.as_relation().tags, "type", "building"))
			building_relations.push_back(&element.as_relation());
	}

	// Building relation members are prepared in geographic space. The relation
	// owns a stable seed; style hint bits do not affect sibling variant rolls.
	std::unordered_set<std::uint64_t> relation_ways;
	for (const auto *relation : building_relations) {
		if (!relation->source_members.empty()) {
			for (const auto &member : relation->source_members)
				if (member.type == "way")
					relation_ways.insert(member.id);
		} else {
			for (const auto &member : relation->members)
				relation_ways.insert(member.way.id);
		}
	}
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
						value = osm_parser::building_style_hint(member.way.tags);
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
				fallback_refs.push_back({"way", member.way.id,
						member.role == ProcessedMemberRole::Part	? "part"
						: member.role == ProcessedMemberRole::Inner ? "inner"
																	: "outer"});
		const auto &source_refs = refs.empty() ? fallback_refs : refs;
		for (const auto &member : source_refs) {
			if (normalized_role(member.role) != "part" ||
					(member.type != "way" && member.type != "relation"))
				continue;
			has_part = true;
			if (member.type == "relation") {
				has_relation_part = true;
				continue;
			}
			out.part_groups[member.id] =
					osm_parser::seed_with_hint(RELATION_SEED_BIT | relation->id, hint);
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
		if (relation_ways.contains(id) || way->nodes.size() < 4 ||
				way->nodes.front().id != way->nodes.back().id)
			continue;
		const bool part = way->tags.contains("building:part") &&
						  !tag_is(way->tags, "building:part", "no");
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
					osm_parser::seed_with_hint(outlines[*best].way->id, hint);
		}
	}
	for (std::size_t i = 0; i < outlines.size(); ++i)
		if (covered[i] / outlines[i].area >= MIN_PART_COVERAGE)
			out.outline_suppression.emplace("way", outlines[i].way->id);

	// Relations whose outer boundaries are split over member ways need stitched
	// geographic rings before area and containment can be measured accurately.
	std::vector<RelationFootprint> relation_outlines, relation_parts;
	for (const auto &element : elements) {
		if (!element.is_relation())
			continue;
		const auto &relation = element.as_relation();
		const auto type = relation.tags.get("type");
		if (type != "multipolygon" && type != "building")
			continue;
		const bool part = relation.tags.contains("building:part") &&
						  !tag_is(relation.tags, "building:part", "no");
		if ((!part && !relation.tags.contains("building")) ||
				(part && !covers_ground(relation.tags)))
			continue;
		auto rings = relation_rings(relation);
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
		RelationFootprint footprint{relation.id, std::move(rings), area,
				lat_sum / static_cast<double>(count),
				lon_sum / static_cast<double>(count)};
		(part ? relation_parts : relation_outlines).push_back(std::move(footprint));
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
		for (const auto i : candidates->second)
			if (std::any_of(relation_outlines[i].rings.begin(),
						relation_outlines[i].rings.end(), [&](const auto &ring) {
							return point_in_ring(part.center_lat, part.center_lon, ring);
						}))
				relation_coverage[i] += part.area;
	}
	for (std::size_t i = 0; i < relation_outlines.size(); ++i)
		if (relation_coverage[i] / relation_outlines[i].area >= MIN_PART_COVERAGE)
			out.suppressed_relations.insert(relation_outlines[i].id);

	return out;
}
} // namespace arnis
