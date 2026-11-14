#include "osm_pbf_cut.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <osmium/handler.hpp>
#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/relation.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/visitor.hpp>

namespace arnis::osm_pbf
{
namespace
{
struct E7Box
{
	std::int64_t min_lat, min_lon, max_lat, max_lon;
	static E7Box from_bbox(const geographic::LLBBox &bbox, double pad_m)
	{
		const double middle = (bbox.min().lat() + bbox.max().lat()) * 0.5;
		const double dlat = pad_m / 111320.0;
		const double dlon =
				pad_m /
				(111320.0 * std::max(0.01, std::cos(middle * 0.017453292519943295)));
		return {static_cast<std::int64_t>(
						std::max(-90e7, std::floor((bbox.min().lat() - dlat) * 1e7))),
				static_cast<std::int64_t>(
						std::max(-180e7, std::floor((bbox.min().lng() - dlon) * 1e7))),
				static_cast<std::int64_t>(
						std::min(90e7, std::ceil((bbox.max().lat() + dlat) * 1e7))),
				static_cast<std::int64_t>(
						std::min(180e7, std::ceil((bbox.max().lng() + dlon) * 1e7)))};
	}
	bool intersects(const E7Box &other) const
	{
		return min_lat <= other.max_lat && other.min_lat <= max_lat &&
			   min_lon <= other.max_lon && other.min_lon <= max_lon;
	}
	void include(const E7Box &other)
	{
		min_lat = std::min(min_lat, other.min_lat);
		min_lon = std::min(min_lon, other.min_lon);
		max_lat = std::max(max_lat, other.max_lat);
		max_lon = std::max(max_lon, other.max_lon);
	}
};

struct NodeLoc
{
	osmium::object_id_type id;
	std::int32_t lat, lon;
};

bool candidate(const tags_t &tags, const char *const *keys, std::size_t count)
{
	for (std::size_t i = 0; i < count; ++i)
		if (tags.contains(keys[i]))
			return true;
	return false;
}

bool excluded(const tags_t &tags)
{
	const auto has = [&tags](const char *key, const char *value) {
		auto it = tags.find(key);
		return it != tags.end() && it->second == value;
	};
	return has("landuse", "salt_pond") || has("natural", "coastline") ||
		   has("natural", "bay") || has("natural", "strait") || has("water", "bay") ||
		   has("water", "ocean") || has("water", "sea") || has("tidal", "yes") ||
		   has("waterway", "tidal_channel") || has("place", "ocean") ||
		   has("place", "sea") || has("place", "bay") || has("place", "strait") ||
		   has("place", "sound") || has("place", "fjord");
}

bool wanted(const tags_t &tags, int kind)
{
	static constexpr const char *any[] = {"building", "building:part", "highway",
			"leisure", "amenity", "tourism", "bridge", "railway", "roller_coaster",
			"aerialway", "attraction", "barrier", "entrance", "door", "power", "historic",
			"emergency", "advertising", "man_made", "aeroway", "3dmr", "shop", "office",
			"landuse", "natural", "water", "waterway"};
	if (excluded(tags))
		return false;
	if (candidate(tags, any, std::size(any)))
		return true;
	if (kind == 1)
		return !tags.empty(); // way(".":".") in the Rust source query
	if (kind == 2)
		return tags.contains("type") && tags.at("type") == "building";
	const auto has = [&tags](const char *key, const char *value) {
		auto it = tags.find(key);
		return it != tags.end() && it->second == value;
	};
	const auto place = tags.find("place");
	const bool named_place =
			place != tags.end() &&
			(place->second == "city" || place->second == "town" ||
					place->second == "borough" || place->second == "suburb" ||
					place->second == "village" || place->second == "quarter" ||
					place->second == "neighbourhood" || place->second == "hamlet");
	return named_place || tags.contains("raceway") ||
		   ((has("public_transport", "stop_position") ||
					has("public_transport", "station")) &&
				   (has("train", "yes") || has("tram", "yes") || has("subway", "yes") ||
						   has("light_rail", "yes")));
}

tags_t copy_tags(const osmium::TagList &source)
{
	tags_t tags;
	for (const auto &tag : source)
		tags.emplace(tag.key(), tag.value());
	return tags;
}

struct PassOne : osmium::handler::Handler
{
	std::vector<NodeLoc> &locations;
	std::vector<osm_parser::RawNode> &tagged_nodes;
	std::vector<osm_parser::RawRelation> &relations;
	E7Box box;
	PassOne(std::vector<NodeLoc> &l, std::vector<osm_parser::RawNode> &n,
			std::vector<osm_parser::RawRelation> &r, E7Box b) :
			locations(l), tagged_nodes(n), relations(r), box(b)
	{
	}
	void node(const osmium::Node &node)
	{
		if (!node.location().valid())
			return;
		const auto lat =
				static_cast<std::int32_t>(std::llround(node.location().lat() * 1e7));
		const auto lon =
				static_cast<std::int32_t>(std::llround(node.location().lon() * 1e7));
		locations.push_back({node.id(), lat, lon});
		const auto tags = copy_tags(node.tags());
		if (box.min_lat <= lat && lat <= box.max_lat && box.min_lon <= lon &&
				lon <= box.max_lon && wanted(tags, 0)) {
			tagged_nodes.push_back({static_cast<std::uint64_t>(node.id()),
					static_cast<double>(lat) / 1e7, static_cast<double>(lon) / 1e7,
					tags});
		}
	}
	void relation(const osmium::Relation &relation)
	{
		auto tags = copy_tags(relation.tags());
		if (!wanted(tags, 2))
			return;
		osm_parser::RawRelation out;
		out.id = static_cast<std::uint64_t>(relation.id());
		out.tags = std::move(tags);
		for (const auto &member : relation.members()) {
			const char *type = member.type() == osmium::item_type::way		  ? "way"
							   : member.type() == osmium::item_type::node	  ? "node"
							   : member.type() == osmium::item_type::relation ? "relation"
																			  : nullptr;
			if (type)
				out.members.push_back({type, static_cast<std::uint64_t>(member.ref()),
						std::string(member.role())});
		}
		relations.push_back(std::move(out));
	}
};

struct WayRecord
{
	osm_parser::RawWay way;
	E7Box bounds;
};

} // namespace

osm_parser::RawOsmDocument cut_extract(
		const std::filesystem::path &path, const geographic::LLBBox &bbox, double pad_m)
{
	const auto box = E7Box::from_bbox(bbox, pad_m);
	std::vector<NodeLoc> locations;
	std::vector<osm_parser::RawNode> tagged_nodes;
	std::vector<osm_parser::RawRelation> relations;
	{
		osmium::io::Reader reader{osmium::io::File{path.string()}};
		PassOne handler{locations, tagged_nodes, relations, box};
		osmium::apply(reader, handler);
		reader.close();
	}
	std::sort(locations.begin(), locations.end(),
			[](const NodeLoc &a, const NodeLoc &b) { return a.id < b.id; });
	auto node_location = [&locations](osmium::object_id_type id) -> const NodeLoc * {
		const auto it = std::lower_bound(locations.begin(), locations.end(), id,
				[](const NodeLoc &node, osmium::object_id_type key) {
					return node.id < key;
				});
		return it != locations.end() && it->id == id ? &*it : nullptr;
	};
	auto way_extent =
			[&node_location](
					const std::vector<std::uint64_t> &refs) -> std::optional<E7Box> {
		std::optional<E7Box> extent;
		for (const auto ref : refs) {
			const auto *node = node_location(static_cast<osmium::object_id_type>(ref));
			if (!node)
				continue;
			const E7Box point{node->lat, node->lon, node->lat, node->lon};
			if (extent)
				extent->include(point);
			else
				extent = point;
		}
		return extent;
	};
	std::unordered_set<std::uint64_t> relation_way_ids;
	for (const auto &relation : relations)
		for (const auto &member : relation.members)
			if (member.type == "way")
				relation_way_ids.insert(member.ref);

	std::vector<WayRecord> selected_ways;
	std::unordered_map<std::uint64_t, E7Box> member_extents;
	{
		osmium::io::Reader reader{
				osmium::io::File{path.string()}, osmium::osm_entity_bits::way};
		while (osmium::memory::Buffer buffer = reader.read()) {
			for (const auto &item : buffer) {
				if (item.type() != osmium::item_type::way)
					continue;
				const auto &way = static_cast<const osmium::Way &>(item);
				std::vector<std::uint64_t> refs;
				refs.reserve(way.nodes().size());
				for (const auto &ref : way.nodes())
					refs.push_back(static_cast<std::uint64_t>(ref.ref()));
				const auto extent = way_extent(refs);
				if (!extent)
					continue;
				auto tags = copy_tags(way.tags());
				if (wanted(tags, 1) && extent->intersects(box)) {
					selected_ways.push_back({{static_cast<std::uint64_t>(way.id()),
													 std::move(refs), std::move(tags)},
							*extent});
				} else if (relation_way_ids.contains(
								   static_cast<std::uint64_t>(way.id()))) {
					member_extents.emplace(static_cast<std::uint64_t>(way.id()), *extent);
				}
			}
		}
		reader.close();
	}
	std::unordered_set<std::uint64_t> selected_ids;
	for (const auto &way : selected_ways)
		selected_ids.insert(way.way.id);
	std::vector<const osm_parser::RawRelation *> selected_relations;
	for (const auto &relation : relations) {
		std::optional<E7Box> extent;
		bool hits = false;
		for (const auto &member : relation.members) {
			if (member.type != "way")
				continue;
			if (selected_ids.contains(member.ref)) {
				hits = true;
				break;
			}
			const auto it = member_extents.find(member.ref);
			if (it != member_extents.end()) {
				if (extent)
					extent->include(it->second);
				else
					extent = it->second;
			}
		}
		if (hits || (extent && extent->intersects(box)))
			selected_relations.push_back(&relation);
	}
	std::sort(selected_relations.begin(), selected_relations.end(),
			[](const auto *a, const auto *b) { return a->id < b->id; });
	std::unordered_set<std::uint64_t> missing_members;
	for (const auto *relation : selected_relations)
		for (const auto &member : relation->members)
			if (member.type == "way" && !selected_ids.contains(member.ref) &&
					member_extents.contains(member.ref))
				missing_members.insert(member.ref);
	if (!missing_members.empty()) {
		osmium::io::Reader reader{
				osmium::io::File{path.string()}, osmium::osm_entity_bits::way};
		while (osmium::memory::Buffer buffer = reader.read()) {
			for (const auto &item : buffer) {
				if (item.type() != osmium::item_type::way)
					continue;
				const auto &way = static_cast<const osmium::Way &>(item);
				const auto id = static_cast<std::uint64_t>(way.id());
				if (!missing_members.erase(id))
					continue;
				std::vector<std::uint64_t> refs;
				refs.reserve(way.nodes().size());
				for (const auto &ref : way.nodes())
					refs.push_back(static_cast<std::uint64_t>(ref.ref()));
				selected_ways.push_back({{id, std::move(refs), copy_tags(way.tags())},
						member_extents.at(id)});
			}
		}
		reader.close();
	}

	std::unordered_set<std::uint64_t> needed_nodes;
	std::unordered_map<std::uint64_t, tags_t> node_tags;
	for (auto &node : tagged_nodes) {
		node_tags.emplace(node.id, std::move(node.tags));
		needed_nodes.insert(node.id);
	}
	for (const auto &way : selected_ways)
		for (const auto ref : way.way.node_refs)
			if (node_location(static_cast<osmium::object_id_type>(ref)))
				needed_nodes.insert(ref);
	std::vector<std::uint64_t> node_ids(needed_nodes.begin(), needed_nodes.end());
	std::sort(node_ids.begin(), node_ids.end());
	std::unordered_map<std::uint64_t, const NodeLoc *> locations_by_id;
	locations_by_id.reserve(node_ids.size());
	for (const auto id : node_ids)
		locations_by_id.emplace(
				id, node_location(static_cast<osmium::object_id_type>(id)));

	osm_parser::RawOsmDocument document;
	document.bounds = std::array<double, 4>{static_cast<double>(box.min_lat) / 1e7,
			static_cast<double>(box.min_lon) / 1e7,
			static_cast<double>(box.max_lat) / 1e7,
			static_cast<double>(box.max_lon) / 1e7};
	for (const auto id : node_ids) {
		const auto *loc = locations_by_id.at(id);
		const auto tags = node_tags.find(id);
		document.nodes.push_back({id, static_cast<double>(loc->lat) / 1e7,
				static_cast<double>(loc->lon) / 1e7,
				tags == node_tags.end() ? tags_t{} : std::move(tags->second)});
	}
	std::sort(selected_ways.begin(), selected_ways.end(),
			[](const auto &a, const auto &b) { return a.way.id < b.way.id; });
	selected_ways.erase(
			std::unique(selected_ways.begin(), selected_ways.end(),
					[](const auto &a, const auto &b) { return a.way.id == b.way.id; }),
			selected_ways.end());
	for (auto &way : selected_ways)
		document.ways.push_back(std::move(way.way));
	for (const auto *relation : selected_relations)
		document.relations.push_back(*relation);
	document.completeness = osm_parser::analyze_completeness(document);
	return document;
}
} // namespace arnis::osm_pbf
