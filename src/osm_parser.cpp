#include "osm_parser.h"

#include <regex>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#include <cctype>
#include <nlohmann/json.hpp>

namespace arnis::osm_parser
{
namespace
{

std::string norm_style(std::string v)
{
	std::string out;
	for (unsigned char c : v)
		if (!std::isspace(c) && c != '_' && c != '-') out.push_back(std::tolower(c));
	return out;
}

std::string decode_xml(std::string value)
{
	const std::pair<const char *, const char *> entities[] = {{"&amp;", "&"},
			{"&quot;", "\""}, {"&apos;", "'"}, {"&lt;", "<"}, {"&gt;", ">"}};
	for (const auto &[from, to] : entities) {
		std::size_t pos = 0;
		while ((pos = value.find(from, pos)) != std::string::npos) {
			value.replace(pos, std::strlen(from), to);
			pos += std::strlen(to);
		}
	}
	return value;
}

std::unordered_map<std::string, std::string> attrs(const std::string &text)
{
	std::unordered_map<std::string, std::string> out;
	static const std::regex attr_re(
			R"ATTR(([A-Za-z_:][A-Za-z0-9_.:-]*)\s*=\s*"([^"]*)")ATTR");
	for (std::sregex_iterator it(text.begin(), text.end(), attr_re), end; it != end; ++it)
		out[(*it)[1].str()] = decode_xml((*it)[2].str());
	return out;
}

std::uint64_t integer(
		const std::unordered_map<std::string, std::string> &a, const char *key)
{
	auto it = a.find(key);
	if (it == a.end())
		throw std::runtime_error(std::string("OSM element missing ") + key);
	return std::stoull(it->second);
}

void read_tags(const std::string &body, tags_t &tags)
{
	static const std::regex tag_re(R"(<tag\b([^>]*)/?>)");
	for (std::sregex_iterator it(body.begin(), body.end(), tag_re), end; it != end;
			++it) {
		auto a = attrs((*it)[1].str());
		auto k = a.find("k");
		auto v = a.find("v");
		if (k != a.end() && v != a.end())
			tags[k->second] = v->second;
	}
}

void filter_tags(tags_t &tags)
{
	static constexpr const char *ignored[] = {
			"created_by", "note", "fixme", "FIXME", "todo", "TODO", "wikipedia",
			"wikimedia_commons", "import_uuid", "import", "old_name", "loc_name",
			"official_name", "alt_name", "operator", "phone", "fax", "email", "url",
			"website", "opening_hours", "description", "attribution", "check_date",
			"survey:date", "ref:bag", "ref:bygningsnr"};
		const bool building = tags.contains("building") || tags.contains("building:part");
		for (auto it = tags.begin(); it != tags.end();) {
			const auto &k = it->first;
			bool remove = k == "start_date" ? !building : false;
			for (const auto *name : ignored)
				remove = remove || k == name;
			if (k != "addr:housenumber") {
				static constexpr const char *prefixes[] = {"addr:", "source", "name:",
						"alt_name:", "contact:", "is_in:", "operator:", "tiger:", "NHD:",
						"lacounty:", "nysgissam:", "ref:ruian:", "building:ruian:", "osak:",
						"gnis:", "yh:", "check_date:"};
				for (const auto *prefix : prefixes)
					if (k.starts_with(prefix)) remove = true;
			}
			if (remove) it = tags.erase(it); else ++it;
		}
}

} // namespace

StyleHint building_style_hint(const tags_t &tags)
{
	const auto material = norm_style(!tags.get("building:material").empty()
			? tags.get("building:material")
			: !tags.get("building:facade:material").empty()
					? tags.get("building:facade:material") : tags.get("facade:material"));
	if (material == "glass" || material == "mirror" || norm_style(tags.get("roof:material")) == "glass")
		return StyleHint::Glass;
	auto present = [&](const char *k) { return !tags.get(k).empty() && norm_style(tags.get(k)) != "no"; };
	if (present("historic") || present("heritage") || tags.contains("ref:nrhp") || present("listed_status"))
		return StyleHint::Masonry;
	static constexpr const char *masonry[] = {"brick","bricks","redbrick","silicatebrick","stone","naturalstone","sandstone","limestone","masonry","granite","marble","terracotta","adobe","stucco","pebbledash"};
	for (const auto *v : masonry) if (material == v) return StyleHint::Masonry;
	const auto cladding = norm_style(tags.get("building:cladding").empty() ?
			tags.get("cladding") : tags.get("building:cladding"));
	static constexpr const char *cladding_types[] = {"brick", "brickmonolith", "plaster", "rendered", "rendering", "stone", "tiling"};
	for (const auto *v : cladding_types) if (cladding == v) return StyleHint::Masonry;
	const auto arch = norm_style(!tags.get("building:architecture").empty() ? tags.get("building:architecture") : tags.get("architecture"));
	static constexpr const char *ornate[] = {"artdeco","artnouveau","gothic","neogothic","gothicrevival","baroque","neobaroque","rococo","renaissance","neorenaissance","romanesque","victorian","georgian","federal","italianate","beauxarts","queenanne","classicism","neoclassical"};
	for (const auto *v : ornate) if (arch == v) return StyleHint::Masonry;
	static constexpr const char *brutalist[] = {"brutalist", "constructivism", "stalinistneoclassicism"};
	for (const auto *v : brutalist) if (arch == v) return StyleHint::Masonry;
	static constexpr const char *modern[] = {"modern","contemporary","modernism","functionalism","newobjectivity","bauhaus","postmodern"};
	for (const auto *v : modern) if (arch == v) return StyleHint::Contemporary;
	for (const char *key : {"start_date", "construction_date", "year_of_construction"}) {
		const auto value = tags.get(key);
		for (std::size_t i = 0; i + 3 < value.size(); ++i) {
			if (std::isdigit(static_cast<unsigned char>(value[i])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 1])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 2])) &&
					std::isdigit(static_cast<unsigned char>(value[i + 3]))) {
				if (std::stoi(value.substr(i, 4)) < 1945) return StyleHint::Masonry;
				break;
			}
		}
	}
	if (material == "concrete" || material == "reinforcedconcrete" || material == "concretereinforced" || material == "concretemasonryunit")
		return StyleHint::Contemporary;
	return StyleHint::None;
}

ArchEra arch_era_from_hint(StyleHint hint)
{
	switch (hint) {
	case StyleHint::Masonry: return ArchEra::TraditionalPreWar;
	case StyleHint::Contemporary:
	case StyleHint::Glass: return ArchEra::Contemporary;
	case StyleHint::None: return ArchEra::Unknown;
	}
	return ArchEra::Unknown;
}

RawOsmDocument::Completeness analyze_completeness(const RawOsmDocument &document)
{
	std::unordered_set<std::uint64_t> node_ids;
	node_ids.reserve(document.nodes.size());
	for (const auto &node : document.nodes)
		node_ids.insert(node.id);
	RawOsmDocument::Completeness result;
	for (const auto &way : document.ways) {
		bool missing = false;
		for (const auto ref : way.node_refs) {
			++result.total_node_refs;
			if (!node_ids.contains(ref)) {
				++result.unresolved_node_refs;
				missing = true;
			}
		}
		if (missing)
			++result.ways_missing_nodes;
	}
	return result;
}

void report_incomplete_source(const RawOsmDocument::Completeness &completeness)
{
	if (completeness.unresolved_node_refs == 0)
		return;
	const double percent = 100.0 * completeness.unresolved_node_refs /
						   std::max<std::uint64_t>(1, completeness.total_node_refs);
	std::cerr << "Warning: " << completeness.unresolved_node_refs << " of "
			  << completeness.total_node_refs << " way node references (" << percent
			  << "%) are missing from the source data, affecting "
			  << completeness.ways_missing_nodes
			  << " ways. Buildings and other areas that lost a corner cannot be filled "
				 "and will not appear in the world.\n";
}

RawOsmDocument parse_osm_xml(std::istream &input)
{
	std::ostringstream buffer;
	buffer << input.rdbuf();
	const std::string xml = buffer.str();
	RawOsmDocument document;

	static const std::regex bounds_re(R"(<bounds\b([^>]*)/?>)");
	std::smatch match;
	if (std::regex_search(xml, match, bounds_re)) {
		auto a = attrs(match[1].str());
		if (a.contains("minlat") && a.contains("minlon") && a.contains("maxlat") &&
				a.contains("maxlon"))
			document.bounds =
					std::array<double, 4>{std::stod(a["minlat"]), std::stod(a["minlon"]),
							std::stod(a["maxlat"]), std::stod(a["maxlon"])};
	}

	static const std::regex node_re(
			R"(<node\b([^>]*?)(?:/>|>(.*?)</node>))", std::regex::icase);
	for (std::sregex_iterator it(xml.begin(), xml.end(), node_re), end; it != end; ++it) {
		auto a = attrs((*it)[1].str());
		RawNode node;
		node.id = integer(a, "id");
		node.lat = std::stod(a.at("lat"));
		node.lon = std::stod(a.at("lon"));
		if ((*it).size() > 2)
			read_tags((*it)[2].str(), node.tags);
		filter_tags(node.tags);
		document.nodes.push_back(std::move(node));
	}

	static const std::regex way_re(R"(<way\b([^>]*?)>(.*?)</way>)", std::regex::icase);
	static const std::regex nd_re(R"(<nd\b([^>]*)/?>)");
	for (std::sregex_iterator it(xml.begin(), xml.end(), way_re), end; it != end; ++it) {
		auto a = attrs((*it)[1].str());
		RawWay way;
		way.id = integer(a, "id");
		const std::string body = (*it)[2].str();
		for (std::sregex_iterator ni(body.begin(), body.end(), nd_re), ne; ni != ne; ++ni)
			way.node_refs.push_back(integer(attrs((*ni)[1].str()), "ref"));
		read_tags(body, way.tags);
		filter_tags(way.tags);
		document.ways.push_back(std::move(way));
	}

	static const std::regex relation_re(
			R"(<relation\b([^>]*?)>(.*?)</relation>)", std::regex::icase);
	static const std::regex member_re(R"(<member\b([^>]*)/?>)");
	for (std::sregex_iterator it(xml.begin(), xml.end(), relation_re), end; it != end;
			++it) {
		auto a = attrs((*it)[1].str());
		RawRelation relation;
		relation.id = integer(a, "id");
		const std::string body = (*it)[2].str();
		for (std::sregex_iterator mi(body.begin(), body.end(), member_re), me; mi != me;
				++mi) {
			auto m = attrs((*mi)[1].str());
			RawRelationMember member;
			member.type = m.contains("type") ? m["type"] : "";
			member.ref = m.contains("ref") ? std::stoull(m["ref"]) : 0;
			member.role = m.contains("role") ? m["role"] : "";
			relation.members.push_back(std::move(member));
		}
		read_tags(body, relation.tags);
		filter_tags(relation.tags);
		document.relations.push_back(std::move(relation));
	}
	document.completeness = analyze_completeness(document);
	report_incomplete_source(document.completeness);
	return document;
}

RawOsmDocument parse_overpass_json(std::istream &input)
{
	nlohmann::json root;
	input >> root;
	if (!root.is_object() || !root.contains("elements") || !root["elements"].is_array())
		throw std::runtime_error("Invalid Overpass JSON: missing elements array");
	RawOsmDocument document;
	if (root.contains("remark") && root["remark"].is_string())
		document.remark = root["remark"].get<std::string>();
	auto parse_tags = [](const nlohmann::json &element) {
		tags_t tags;
		if (element.contains("tags") && element["tags"].is_object())
			for (auto it = element["tags"].begin(); it != element["tags"].end(); ++it)
				if (it.value().is_string())
					tags[it.key()] = it.value().get<std::string>();
		return tags;
	};
	for (const auto &element : root["elements"]) {
		if (!element.is_object() || !element.contains("type") ||
				!element["type"].is_string() || !element.contains("id") ||
				!element["id"].is_number_integer())
			continue;
		const auto type = element["type"].get<std::string>();
		const auto id = element["id"].get<std::uint64_t>();
		if (type == "node") {
			if (!element.contains("lat") || !element.contains("lon") ||
					!element["lat"].is_number() || !element["lon"].is_number())
				continue;
			auto node_tags = parse_tags(element);
			filter_tags(node_tags);
			document.nodes.push_back({id, element["lat"].get<double>(),
					element["lon"].get<double>(), std::move(node_tags)});
		} else if (type == "way") {
			RawWay way;
			way.id = id;
		way.tags = parse_tags(element);
		filter_tags(way.tags);
			if (element.contains("nodes") && element["nodes"].is_array())
				for (const auto &node : element["nodes"])
					if (node.is_number_integer())
						way.node_refs.push_back(node.get<std::uint64_t>());
			document.ways.push_back(std::move(way));
		} else if (type == "relation") {
			RawRelation relation;
			relation.id = id;
		relation.tags = parse_tags(element);
		filter_tags(relation.tags);
			if (element.contains("members") && element["members"].is_array())
				for (const auto &value : element["members"]) {
					if (!value.is_object())
						continue;
					RawRelationMember member;
					if (value.contains("type") && value["type"].is_string())
						member.type = value["type"].get<std::string>();
					if (value.contains("ref") && value["ref"].is_number_integer())
							member.ref = value["ref"].get<std::uint64_t>();
					if (value.contains("role") && value["role"].is_string())
						member.role = value["role"].get<std::string>();
					relation.members.push_back(std::move(member));
				}
			document.relations.push_back(std::move(relation));
		}
	}
	document.completeness = analyze_completeness(document);
	report_incomplete_source(document.completeness);
	return document;
}

} // namespace arnis::osm_parser
