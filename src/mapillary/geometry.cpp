#include "geometry.h"

#include <nlohmann/json.hpp>
#include <set>
#include <tuple>

namespace arnis::mapillary
{
namespace
{
using Json = nlohmann::json;

std::map<std::string, std::string> tags_of(const Json &element)
{
	std::map<std::string, std::string> tags;
	if (!element.contains("tags") || !element["tags"].is_object())
		return tags;
	for (const auto &[key, value] : element["tags"].items())
		if (value.is_string())
			tags.emplace(key, value.get<std::string>());
	return tags;
}

std::vector<std::int64_t> node_list(const Json &way)
{
	std::vector<std::int64_t> result;
	if (!way.contains("nodes") || !way["nodes"].is_array())
		return result;
	for (const auto &node : way["nodes"])
		if (node.is_number_integer())
			result.push_back(node.get<std::int64_t>());
	return result;
}

std::optional<std::vector<std::array<double, 2>>> ring_xy(
		const std::vector<std::int64_t> &ids,
		const std::map<std::int64_t, std::pair<double, double>> &nodes,
		const Frame &frame)
{
	std::vector<std::array<double, 2>> result;
	result.reserve(ids.size());
	for (const auto id : ids) {
		const auto it = nodes.find(id);
		if (it == nodes.end())
			return std::nullopt;
		result.push_back(frame.to_enu(it->second.first, it->second.second));
	}
	return result;
}

bool point_in_ring(const std::array<double, 2> &point,
		const std::vector<std::array<double, 2>> &ring)
{
	bool inside = false;
	if (ring.size() < 3)
		return false;
	for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
		const auto &a = ring[i], &b = ring[j];
		const bool crosses =
				((a[1] > point[1]) != (b[1] > point[1])) &&
				(point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]);
		if (crosses)
			inside = !inside;
	}
	return inside;
}

std::vector<std::vector<std::array<double, 2>>> holes_for_outer(
		const std::vector<std::vector<std::array<double, 2>>> &holes,
		const std::vector<std::array<double, 2>> &outer)
{
	std::vector<std::vector<std::array<double, 2>>> result;
	for (const auto &hole : holes)
		if (!hole.empty() && point_in_ring(ring_centroid(hole), outer))
			result.push_back(hole);
	return result;
}

std::optional<Building> make_building(const std::string &key, std::int64_t id,
		OsmKind kind, const std::vector<std::array<double, 2>> &input_ring,
		const std::vector<std::int64_t> &input_ids,
		std::vector<std::vector<std::array<double, 2>>> holes,
		std::map<std::string, std::string> tags, const Params &params,
		const std::optional<std::array<std::array<double, 2>, 4>> &target)
{
	auto [ring, ids] = dedupe_ring(input_ring, input_ids);
	if (ring.size() < 3 || !ring_is_simple(ring) ||
			std::abs(signed_area(ring)) < params.min_ring_area_m2)
		return std::nullopt;
	ensure_ccw(ring, ids);
	holes.erase(std::remove_if(holes.begin(), holes.end(),
						[](const auto &hole) {
							return hole.size() < 3 || std::abs(signed_area(hole)) <= 0.5;
						}),
			holes.end());
	const auto [height, source, min_height] = height_from_tags(tags, params);
	bool is_target = !target.has_value();
	if (target) {
		const auto &lo = (*target)[0];
		const auto &hi = (*target)[2];
		for (const auto &point : ring)
			if (point[0] >= lo[0] && point[0] <= hi[0] && point[1] >= lo[1] &&
					point[1] <= hi[1]) {
				is_target = true;
				break;
			}
	}
	return Building{key, id, kind, std::move(ring), std::move(holes), std::move(ids),
			std::move(tags), height, source, min_height, is_target, {}};
}
}

std::vector<Building> parse_overpass(const std::string &json, const Frame &frame,
		const Params &params, const std::optional<BBox> &target_bbox)
{
	std::vector<Building> result;
	try {
		const auto root = Json::parse(json);
		const auto *elements =
				root.is_object() && root.contains("elements") ? &root["elements"] : &root;
		if (!elements->is_array())
			return result;
		std::map<std::int64_t, std::pair<double, double>> nodes;
		std::map<std::int64_t, const Json *> ways;
		std::vector<std::int64_t> way_order;
		std::vector<const Json *> relations;
		for (const auto &element : *elements) {
			const auto type = element.value("type", "");
			const auto id = element.value("id", std::int64_t{0});
			if (type == "node" && element.contains("lon") && element.contains("lat"))
				nodes[id] = {element["lon"].get<double>(), element["lat"].get<double>()};
			else if (type == "way") {
				ways[id] = &element;
				way_order.push_back(id);
			} else if (type == "relation")
				relations.push_back(&element);
		}
		std::optional<std::array<std::array<double, 2>, 4>> target;
		if (target_bbox)
			target = bbox_polygon_xy(*target_bbox, frame, 0.0);
		std::set<std::int64_t> relation_members;
		for (const auto *relation : relations) {
			const auto tags = tags_of(*relation);
			if (!tags.count("building"))
				continue;
			std::vector<std::vector<std::int64_t>> outer, inner;
			std::vector<std::int64_t> members;
			if (relation->contains("members") && (*relation)["members"].is_array())
				for (const auto &member : (*relation)["members"]) {
					if (member.value("type", "") != "way")
						continue;
					const auto id = member.value("ref", std::int64_t{0});
					const auto way = ways.find(id);
					if (way == ways.end())
						continue;
					const auto ids = node_list(*way->second);
					const auto role = member.value("role", "outer");
					(role == "inner" ? inner : outer).push_back(ids);
					members.push_back(id);
					relation_members.insert(id);
				}
			const auto outer_rings = join_way_segments(outer);
			const auto inner_rings = join_way_segments(inner);
			std::sort(members.begin(), members.end());
			members.erase(std::unique(members.begin(), members.end()), members.end());
			std::vector<std::vector<std::array<double, 2>>> holes;
			for (const auto &ids : inner_rings)
				if (auto xy = ring_xy(ids, nodes, frame))
					holes.push_back(std::move(*xy));
			const auto relation_id = relation->value("id", std::int64_t{0});
			for (std::size_t index = 0; index < outer_rings.size(); ++index) {
				if (auto xy = ring_xy(outer_rings[index], nodes, frame)) {
					auto outer_holes = holes;
					if (const auto outer_xy = ring_xy(outer_rings[index], nodes, frame))
						outer_holes = holes_for_outer(holes, *outer_xy);
					std::string key = building_key(OsmKind::Relation, relation_id);
					if (index)
						key += "_" + std::to_string(index);
					if (auto building = make_building(key, relation_id, OsmKind::Relation,
								*xy, outer_rings[index], std::move(outer_holes), tags,
								params, target)) {
						building->member_ways = members;
						result.push_back(std::move(*building));
					}
				}
			}
		}
		for (const auto id : way_order) {
			if (relation_members.count(id))
				continue;
			const auto way = ways.find(id);
			if (way == ways.end())
				continue;
			const auto tags = tags_of(*way->second);
			if (!tags.count("building"))
				continue;
			const auto ids = node_list(*way->second);
			if (ids.size() < 4 || ids.front() != ids.back())
				continue;
			if (auto xy = ring_xy(ids, nodes, frame))
				if (auto building = make_building(building_key(OsmKind::Way, id), id,
							OsmKind::Way, *xy, ids, {}, tags, params, target))
					result.push_back(std::move(*building));
		}
		std::sort(result.begin(), result.end(), [](const Building &a, const Building &b) {
			return std::tie(a.kind, a.osm_id) < std::tie(b.kind, b.osm_id);
		});
	} catch (...) {
		result.clear();
	}
	return result;
}
} // namespace arnis::mapillary
