#include "osm_parser.h"

#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <cctype>
#include <charconv>
#include <nlohmann/json.hpp>
#include "strict_parse.h"

namespace arnis::osm_parser
{
namespace
{

std::string norm_style(std::string v)
{
	std::string out;
	for (unsigned char c : v)
		if (!std::isspace(c) && c != '_' && c != '-')
			out.push_back(std::tolower(c));
	return out;
}

std::optional<std::int32_t> first_year(std::string_view value)
{
	for (std::size_t i = 0; i + 4 <= value.size(); ++i) {
		bool digits = true;
		for (std::size_t j = 0; j < 4; ++j)
			digits &= std::isdigit(static_cast<unsigned char>(value[i + j]));
		if (digits) {
			try {
				return std::stoi(std::string(value.substr(i, 4)));
			} catch (...) {
				return std::nullopt;
			}
		}
	}
	return std::nullopt;
}

std::string first_architecture(const tags_t &tags)
{
	std::string value;
	if (const auto it = tags.find("building:architecture"); it != tags.end()) {
		value = it->second;
	} else {
		value = tags.get("architecture");
	}
	if (const auto semi = value.find(';'); semi != std::string::npos)
		value.resize(semi);
	return norm_style(std::move(value));
}

bool is_listed(const tags_t &tags)
{
	auto positive = [&](const char *key) {
		const auto it = tags.find(key);
		if (it == tags.end())
			return false;
		const auto &value = it->second;
		return !(value.size() == 2 &&
				 std::tolower(static_cast<unsigned char>(value[0])) == 'n' &&
				 std::tolower(static_cast<unsigned char>(value[1])) == 'o');
	};
	return positive("historic") || positive("heritage") || tags.contains("ref:nrhp") ||
		   positive("listed_status");
}

bool is_place_of_worship(const tags_t &tags)
{
	if (tags.get("amenity") == "place_of_worship")
		return true;
	const auto building = tags.get("building");
	return building == "church" || building == "cathedral" || building == "chapel" ||
		   building == "basilica" || building == "mosque" || building == "synagogue" ||
		   building == "temple" || building == "shrine" || building == "monastery";
}

bool housing_type_impl(const tags_t &tags)
{
	const auto type = tags.get("building");
	return type == "residential" || type == "apartments" || type == "house" ||
		   type == "detached" || type == "semidetached_house" || type == "terrace" ||
		   type == "dormitory" || type == "bungalow" || type == "villa";
}

std::string decode_xml(std::string value)
{
	// Match quick-xml's XML 1.0 attribute normalization: literal tabs and line
	// breaks in an attribute become spaces before entity references are decoded.
	// Numeric references such as "&#10;" are consequently preserved as the
	// referenced character, just as Rust's normalized_value() does.
	std::string normalized;
	normalized.reserve(value.size());
	for (std::size_t i = 0; i < value.size(); ++i) {
		if (value[i] == '\r') {
			if (i + 1 < value.size() && value[i + 1] == '\n')
				++i;
			normalized.push_back(' ');
		} else if (value[i] == '\n' || value[i] == '\t') {
			normalized.push_back(' ');
		} else {
			normalized.push_back(value[i]);
		}
	}
	value = std::move(normalized);
	auto append_codepoint = [](std::string &out, unsigned long codepoint) {
		if (codepoint <= 0x7f)
			out.push_back(static_cast<char>(codepoint));
		else if (codepoint <= 0x7ff) {
			out.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
			out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
		} else if (codepoint <= 0xffff) {
			out.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
			out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
			out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
		} else {
			out.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
			out.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
			out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
			out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
		}
	};
	std::string decoded;
	decoded.reserve(value.size());
	for (std::size_t pos = 0; pos < value.size();) {
		if (value[pos] != '&') {
			decoded.push_back(value[pos++]);
			continue;
		}
		const auto end = value.find(';', pos + 1);
		if (end == std::string::npos)
			throw std::runtime_error("unterminated XML entity reference");
		const auto entity = std::string_view(value).substr(pos + 1, end - pos - 1);
		if (entity == "amp")
			decoded.push_back('&');
		else if (entity == "quot")
			decoded.push_back('"');
		else if (entity == "apos")
			decoded.push_back('\'');
		else if (entity == "lt")
			decoded.push_back('<');
		else if (entity == "gt")
			decoded.push_back('>');
		else if (!entity.empty() && entity.front() == '#') {
			const bool hexadecimal =
					entity.size() > 2 && (entity[1] == 'x' || entity[1] == 'X');
			const auto digits = entity.substr(hexadecimal ? 2 : 1);
			unsigned long codepoint = 0;
			const auto [tail, error] = std::from_chars(digits.data(),
					digits.data() + digits.size(), codepoint, hexadecimal ? 16 : 10);
			const bool valid_xml_character =
					codepoint == 0x9 || codepoint == 0xa || codepoint == 0xd ||
					(codepoint >= 0x20 && codepoint <= 0xd7ff) ||
					(codepoint >= 0xe000 && codepoint <= 0xfffd) ||
					(codepoint >= 0x10000 && codepoint <= 0x10ffff);
			if (digits.empty() || error != std::errc{} ||
					tail != digits.data() + digits.size() || !valid_xml_character)
				throw std::runtime_error("invalid numeric XML entity");
			append_codepoint(decoded, codepoint);
		} else {
			throw std::runtime_error("unknown XML entity: &" + std::string(entity) + ";");
		}
		pos = end + 1;
	}
	return decoded;
}

struct XmlEvent
{
	std::string name;
	std::unordered_map<std::string, std::string> attributes;
	bool closing{};
	bool empty{};
};

bool xml_space(char c)
{
	return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

void skip_xml_until(std::istream &input, std::string_view terminator)
{
	std::size_t matched = 0;
	char c{};
	while (input.get(c)) {
		if (c == terminator[matched]) {
			if (++matched == terminator.size())
				return;
		} else {
			matched = c == terminator.front() ? 1 : 0;
		}
	}
	throw std::runtime_error("unterminated XML markup");
}

std::string trim_xml_space(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}

std::string required_xml_attribute(const XmlEvent &event, const char *name)
{
	const auto it = event.attributes.find(name);
	if (it == event.attributes.end())
		throw std::runtime_error(std::string("OSM XML element missing ") + name);
	return it->second;
}

std::uint64_t xml_u64(const XmlEvent &event, const char *name)
{
	const auto value = trim_xml_space(required_xml_attribute(event, name));
	std::uint64_t parsed{};
	const auto [tail, error] =
			std::from_chars(value.data(), value.data() + value.size(), parsed);
	if (value.empty() || error != std::errc{} || tail != value.data() + value.size())
		throw std::runtime_error(
				std::string("invalid OSM XML integer attribute: ") + name);
	return parsed;
}

double xml_f64(const XmlEvent &event, const char *name)
{
	const auto value = trim_xml_space(required_xml_attribute(event, name));
	const auto parsed = strict_parse::f64(value);
	if (!parsed)
		throw std::runtime_error(
				std::string("invalid OSM XML number attribute: ") + name);
	return *parsed;
}

std::optional<XmlEvent> next_xml_event(std::istream &input)
{
	char c{};
	for (;;) {
		while (input.get(c) && c != '<') {
		}
		if (!input) {
			if (input.bad())
				throw std::runtime_error("failed while reading OSM XML");
			return std::nullopt;
		}
		if (!input.get(c))
			throw std::runtime_error("truncated XML markup");
		if (c == '?') {
			skip_xml_until(input, "?>");
			continue;
		}
		if (c != '!')
			break;
		char first{}, second{};
		if (!input.get(first) || !input.get(second))
			throw std::runtime_error("truncated XML declaration");
		if (first == '-' && second == '-') {
			skip_xml_until(input, "-->");
		} else if (first == '[' && second == 'C') {
			for (const char expected : std::string_view("DATA[")) {
				if (!input.get(c) || c != expected)
					throw std::runtime_error("malformed CDATA section");
			}
			skip_xml_until(input, "]]>");
		} else {
			// Skip a doctype or declaration while respecting quotes and an internal
			// subset, so a `>` inside either cannot expose fake OSM elements.
			int subset_depth = 0;
			char declaration_quote = 0;
			for (;;) {
				if (!input.get(c))
					throw std::runtime_error("unterminated XML declaration");
				if (declaration_quote) {
					if (c == declaration_quote)
						declaration_quote = 0;
				} else if (c == '\'' || c == '"') {
					declaration_quote = c;
				} else if (c == '[') {
					++subset_depth;
				} else if (c == ']' && subset_depth > 0) {
					--subset_depth;
				} else if (c == '>' && subset_depth == 0) {
					break;
				}
			}
		}
	}

	XmlEvent event;
	if (c == '/') {
		event.closing = true;
		if (!input.get(c))
			throw std::runtime_error("truncated XML closing tag");
	}
	std::string raw;
	char quote = 0;
	for (;;) {
		if (c == '\'' || c == '"') {
			if (!quote)
				quote = c;
			else if (quote == c)
				quote = 0;
		}
		if (c == '>' && !quote)
			break;
		raw.push_back(c);
		if (!input.get(c))
			throw std::runtime_error("unterminated XML tag");
	}
	if (quote)
		throw std::runtime_error("unterminated XML attribute");
	std::size_t pos = 0;
	while (pos < raw.size() && xml_space(raw[pos]))
		++pos;
	const auto name_start = pos;
	while (pos < raw.size() && !xml_space(raw[pos]) && raw[pos] != '/')
		++pos;
	if (name_start == pos)
		throw std::runtime_error("empty XML element name");
	event.name = raw.substr(name_start, pos - name_start);
	if (event.closing) {
		while (pos < raw.size() && xml_space(raw[pos]))
			++pos;
		if (pos != raw.size())
			throw std::runtime_error("invalid XML closing tag");
		return event;
	}
	while (pos < raw.size()) {
		while (pos < raw.size() && xml_space(raw[pos]))
			++pos;
		if (pos == raw.size())
			break;
		if (raw[pos] == '/') {
			++pos;
			while (pos < raw.size() && xml_space(raw[pos]))
				++pos;
			if (pos != raw.size())
				throw std::runtime_error("invalid self-closing XML tag");
			event.empty = true;
			break;
		}
		const auto key_start = pos;
		while (pos < raw.size() && !xml_space(raw[pos]) && raw[pos] != '=')
			++pos;
		if (key_start == pos)
			throw std::runtime_error("invalid XML attribute name");
		const auto key = raw.substr(key_start, pos - key_start);
		while (pos < raw.size() && xml_space(raw[pos]))
			++pos;
		if (pos == raw.size() || raw[pos++] != '=')
			throw std::runtime_error("XML attribute missing '='");
		while (pos < raw.size() && xml_space(raw[pos]))
			++pos;
		if (pos == raw.size() || (raw[pos] != '\'' && raw[pos] != '"'))
			throw std::runtime_error("XML attribute value must be quoted");
		const char delimiter = raw[pos++];
		const auto value_start = pos;
		while (pos < raw.size() && raw[pos] != delimiter)
			++pos;
		if (pos == raw.size())
			throw std::runtime_error("unterminated XML attribute value");
		auto value = decode_xml(raw.substr(value_start, pos - value_start));
		++pos;
		if (!event.attributes.emplace(key, std::move(value)).second)
			throw std::runtime_error("duplicate XML attribute: " + key);
	}
	return event;
}

void set_xml_tag(tags_t &tags, const XmlEvent &event)
{
	const auto key = event.attributes.find("k");
	const auto value = event.attributes.find("v");
	if (key != event.attributes.end() && value != event.attributes.end())
		tags[key->second] = value->second;
}

void filter_tags_impl(tags_t &tags)
{
	static constexpr const char *ignored[] = {"created_by", "note", "fixme", "FIXME",
			"todo", "TODO", "wikimedia_commons", "import_uuid", "import", "old_name",
			"loc_name", "official_name", "alt_name", "operator", "phone", "fax", "email",
			"url", "website", "opening_hours", "description", "attribution", "check_date",
			"survey:date", "ref:bag", "ref:bygningsnr"};
	const bool building = tags.contains("building") || tags.contains("building:part");
	for (auto it = tags.begin(); it != tags.end();) {
		const auto &k = it->first;
		if (k == "wikipedia") {
			// Discoveries only need the article marker, not a language/title string.
			it->second.clear();
			++it;
			continue;
		}
		bool remove = k == "start_date" ? !building : false;
		for (const auto *name : ignored)
			remove = remove || k == name;
		if (k != "addr:housenumber") {
			static constexpr const char *prefixes[] = {"addr:", "source",
					"name:", "alt_name:", "contact:", "is_in:", "operator:", "tiger:",
					"NHD:", "lacounty:", "nysgissam:", "ref:ruian:", "building:ruian:",
					"osak:", "gnis:", "yh:", "check_date:"};
			for (const auto *prefix : prefixes)
				if (k.starts_with(prefix))
					remove = true;
		}
		if (remove)
			it = tags.erase(it);
		else
			++it;
	}
	const auto highway = tags.get("highway");
	if (highway == "proposed" || highway == "razed") {
		tags.erase("highway");
		if (!tags.contains("railway")) {
			tags.erase("bridge");
			tags.erase("tunnel");
			tags.erase("layer");
		}
	}
	static const std::unordered_set<std::string> landforms{"peninsula", "isthmus", "cape",
			"valley", "plateau", "massif", "mountain_range", "gorge", "landform", "plain",
			"arete", "couloir", "coastline"};
	if (landforms.contains(tags.get("natural")))
		tags.erase("natural");
	// A swimming pool is a built basin, not a lake. Keep overlapping OSM water
	// classifications from routing it through the natural-water generator.
	if (tags.get("leisure") == "swimming_pool") {
		if (tags.get("natural") == "water")
			tags.erase("natural");
		tags.erase("water");
	}
}

} // namespace

std::size_t get_priority(const ProcessedElement &element)
{
	static constexpr std::array<const char *, 6> priority_order{
			"entrance", "building", "highway", "waterway", "water", "barrier"};
	for (std::size_t i = 0; i < priority_order.size(); ++i)
		if (element.tags().contains(priority_order[i]))
			return i;
	const auto man_made = element.tags().get("man_made");
	if (man_made == "antenna" || man_made == "mast" || man_made == "chimney")
		return priority_order.size() + 1;
	return priority_order.size();
}

void filter_tags(tags_t &tags)
{
	filter_tags_impl(tags);
}

StyleHint building_style_hint(const tags_t &tags)
{
	const auto first_value =
			[&](std::initializer_list<const char *> keys) -> std::string_view {
		for (const auto *key : keys)
			if (auto it = tags.find(key); it != tags.end())
				return it->second;
		return {};
	};
	const auto material = norm_style(std::string(first_value(
			{"building:material", "building:facade:material", "facade:material"})));
	if (material == "glass" || material == "mirror" ||
			norm_style(tags.get("roof:material")) == "glass")
		return StyleHint::Glass;
	const auto arch = first_architecture(tags);
	const auto year = building_start_year(tags);
	const bool modern_arch = arch == "modern" || arch == "contemporary" ||
							 arch == "modernism" || arch == "functionalism" ||
							 arch == "newobjectivity" || arch == "postmodern" ||
							 arch == "bauhaus";
	if (is_listed(tags) &&
			!(modern_arch || (year && *year >= 1945 && !is_place_of_worship(tags))))
		return StyleHint::Masonry;
	auto matches = [](std::string_view value,
						   std::initializer_list<std::string_view> choices) {
		return std::find(choices.begin(), choices.end(), value) != choices.end();
	};
	if (matches(material,
				{"brick", "bricks", "redbrick", "silicatebrick", "stone", "naturalstone",
						"sandstone", "limestone", "masonry", "granite", "marble",
						"terracotta", "adobe", "stucco", "pebbledash"}))
		return StyleHint::Masonry;
	if (matches(norm_style(std::string(first_value({"building:cladding"}))),
				{"brick", "brickmonolith", "plaster", "rendered", "rendering", "stone",
						"tiling"}))
		return StyleHint::Masonry;
	if (matches(arch,
				{"artdeco", "artnouveau", "gothic", "neogothic", "gothicrevival",
						"neoclassicism", "neoclassical", "classicism", "classicalrevival",
						"greekrevival", "baroque", "neobaroque", "rococo", "barocco",
						"historicism", "eclectic", "renaissance", "neorenaissance",
						"romanesque", "neoromanesque", "romanesquerevival", "victorian",
						"georgian", "federal", "italianate", "beauxarts",
						"wilhelminianstyle", "queenanne", "brutalist", "constructivism",
						"stalinistneoclassicism"}))
		return StyleHint::Masonry;
	if (modern_arch)
		return StyleHint::Contemporary;
	if (year && *year < 1945)
		return StyleHint::Masonry;
	if (matches(material, {"concrete", "reinforcedconcrete", "concretereinforced",
								  "concretemasonryunit"}))
		return StyleHint::Contemporary;
	return StyleHint::None;
}

std::optional<std::int32_t> building_start_year(const tags_t &tags)
{
	for (const char *key : {"start_date", "construction_date", "year_of_construction"})
		if (const auto year = first_year(tags.get(key)))
			return year;
	return std::nullopt;
}

bool is_housing_type(const tags_t &tags)
{
	return housing_type_impl(tags);
}

bool part_covers_ground(const tags_t &tags)
{
	const auto leading_number = [](std::string_view text) -> std::optional<double> {
		const auto first = text.find_first_not_of(" \t\r\n\f\v");
		if (first == std::string_view::npos)
			return std::nullopt;
		text.remove_prefix(first);
		std::size_t end = 0;
		bool dot = false;
		for (; end < text.size(); ++end) {
			const char c = text[end];
			if ((c >= '0' && c <= '9') || ((c == '+' || c == '-') && end == 0))
				continue;
			if (c == '.' && !dot) {
				dot = true;
				continue;
			}
			break;
		}
		return strict_parse::f64(text.substr(0, end));
	};
	for (const char *key : {"min_height", "building:min_level"})
		if (const auto it = tags.find(key); it != tags.end())
			if (const auto value = leading_number(it->second); value && *value > 0.0)
				return false;
	return true;
}

std::optional<std::int32_t> building_year_for_parts(const tags_t &tags)
{
	const auto year = building_start_year(tags);
	if (!year)
		return std::nullopt;
	const auto era = building_arch_era_dated(tags, year);
	return (*year < 1945 || era == ArchEra::PostWarPanel || era == ArchEra::Contemporary)
				   ? year
				   : std::nullopt;
}

std::uint64_t seed_with_metadata(std::uint64_t seed, StyleHint hint,
		std::optional<std::int32_t> year, bool residential)
{
	const auto raw_year = year ? std::clamp(*year - YEAR_BASE, 1, 255) : 0;
	seed = seed_without_hint(seed);
	seed |= static_cast<std::uint64_t>(hint) << STYLE_HINT_SHIFT;
	seed |= static_cast<std::uint64_t>(raw_year) << YEAR_SHIFT;
	if (residential)
		seed |= RESIDENTIAL_GROUP_BIT;
	return seed;
}

ArchEra era_for_year(std::int32_t year)
{
	return year < 1945	 ? ArchEra::TraditionalPreWar
		   : year < 1980 ? ArchEra::PostWarPanel
						 : ArchEra::Contemporary;
}

bool is_art_deco(const tags_t &tags)
{
	return first_architecture(tags) == "artdeco";
}

ArchEra building_arch_era_dated(const tags_t &tags, std::optional<std::int32_t> year)
{
	const auto arch = first_architecture(tags);
	const auto matches = [&](std::initializer_list<std::string_view> values) {
		return std::find(values.begin(), values.end(), arch) != values.end();
	};
	if (matches({"artdeco", "artnouveau", "gothic", "neogothic", "gothicrevival",
				"neoclassicism", "neoclassical", "classicism", "classicalrevival",
				"greekrevival", "baroque", "neobaroque", "rococo", "barocco",
				"historicism", "eclectic", "renaissance", "neorenaissance", "romanesque",
				"neoromanesque", "romanesquerevival", "victorian", "georgian", "federal",
				"italianate", "beauxarts", "wilhelminianstyle", "queenanne"}))
		return ArchEra::HistoricOrnate;
	if (matches({"brutalist", "constructivism", "stalinistneoclassicism"}))
		return ArchEra::PostWarPanel;
	if (matches({"modern", "contemporary", "modernism", "functionalism", "newobjectivity",
				"postmodern", "bauhaus"}))
		return year ? era_for_year(*year) : ArchEra::Contemporary;
	if (is_listed(tags)) {
		if (year && *year >= 1945 && !is_place_of_worship(tags))
			return era_for_year(*year);
		return ArchEra::HistoricOrnate;
	}
	if (year)
		return era_for_year(*year);
	const auto material =
			norm_style(tags.get("building:material").empty()
							   ? (tags.get("building:facade:material").empty()
												 ? tags.get("facade:material")
												 : tags.get("building:facade:material"))
							   : tags.get("building:material"));
	if (material == "panel" || material == "panels" || material == "prefab" ||
			material == "prefabricated" || material == "panelhouse")
		return ArchEra::PostWarPanel;
	if (material == "brick" || material == "bricks" || material == "redbrick" ||
			material == "silicatebrick" || material == "stone" ||
			material == "naturalstone" || material == "sandstone" ||
			material == "limestone" || material == "masonry" || material == "granite" ||
			material == "marble" || material == "terracotta" || material == "adobe" ||
			material == "stucco" || material == "pebbledash")
		return ArchEra::TraditionalPreWar;
	if (material == "concrete" || material == "reinforcedconcrete" ||
			material == "concretereinforced" || material == "concretemasonryunit" ||
			material == "glass" || material == "mirror")
		return ArchEra::Contemporary;
	const auto cladding = norm_style(tags.get("building:cladding"));
	if (cladding == "brick" || cladding == "brickmonolith" || cladding == "plaster" ||
			cladding == "rendered" || cladding == "rendering" || cladding == "stone" ||
			cladding == "tiling")
		return ArchEra::TraditionalPreWar;
	return ArchEra::Unknown;
}

ArchEra arch_era_from_hint(StyleHint hint)
{
	switch (hint) {
	case StyleHint::Masonry:
		return ArchEra::TraditionalPreWar;
	case StyleHint::Contemporary:
	case StyleHint::Glass:
		return ArchEra::Contemporary;
	case StyleHint::None:
		return ArchEra::Unknown;
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
		for (const auto &ref : way.node_refs) {
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

std::optional<std::array<double, 4>> resolve_bbox(const RawOsmDocument &document)
{
	auto valid_bbox = [](const std::array<double, 4> &b) {
		return std::all_of(b.begin(), b.end(),
					   [](double value) { return std::isfinite(value); }) &&
			   b[0] >= -90.0 && b[2] <= 90.0 && b[1] >= -180.0 && b[3] <= 180.0 &&
			   b[0] < b[2] && b[1] < b[3];
	};
	if (document.bounds) {
		const auto &b = *document.bounds;
		if (valid_bbox(b))
			return document.bounds;
	}
	if (document.nodes.empty())
		return std::nullopt;
	std::optional<std::array<double, 4>> result;
	for (const auto &node : document.nodes) {
		if (!node.lat || !node.lon || !std::isfinite(*node.lat) ||
				!std::isfinite(*node.lon))
			continue;
		if (!result) {
			result = std::array<double, 4>{*node.lat, *node.lon, *node.lat, *node.lon};
			continue;
		}
		(*result)[0] = std::min((*result)[0], *node.lat);
		(*result)[1] = std::min((*result)[1], *node.lon);
		(*result)[2] = std::max((*result)[2], *node.lat);
		(*result)[3] = std::max((*result)[3], *node.lon);
	}
	if (!result || !valid_bbox(*result))
		return std::nullopt;
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
	RawOsmDocument document;
	std::optional<RawNode> node;
	std::optional<RawWay> way;
	std::optional<RawRelation> relation;
	auto finish_current = [&]() {
		if (node) {
			filter_tags_impl(node->tags);
			document.nodes.push_back(std::move(*node));
			node.reset();
		} else if (way) {
			filter_tags_impl(way->tags);
			document.ways.push_back(std::move(*way));
			way.reset();
		} else if (relation) {
			filter_tags_impl(relation->tags);
			document.relations.push_back(std::move(*relation));
			relation.reset();
		}
	};
	while (const auto event = next_xml_event(input)) {
		if (event->closing) {
			if (event->name == "node" || event->name == "way" ||
					event->name == "relation")
				finish_current();
			continue;
		}
		if (event->name == "bounds") {
			const auto minlat = event->attributes.find("minlat");
			const auto minlon = event->attributes.find("minlon");
			const auto maxlat = event->attributes.find("maxlat");
			const auto maxlon = event->attributes.find("maxlon");
			if (minlat != event->attributes.end() && minlon != event->attributes.end() &&
					maxlat != event->attributes.end() &&
					maxlon != event->attributes.end()) {
				auto parse_bound = [](const std::string &value) {
					const auto parsed = strict_parse::f64(trim_xml_space(value));
					if (!parsed)
						throw std::runtime_error("invalid OSM XML bounds coordinate");
					return *parsed;
				};
				const std::array<double, 4> bounds{parse_bound(minlat->second),
						parse_bound(minlon->second), parse_bound(maxlat->second),
						parse_bound(maxlon->second)};
				if (!std::all_of(bounds.begin(), bounds.end(),
							[](double value) { return std::isfinite(value); }) ||
						bounds[0] < -90.0 || bounds[2] > 90.0 || bounds[1] < -180.0 ||
						bounds[3] > 180.0 || bounds[0] >= bounds[2] ||
						bounds[1] >= bounds[3])
					throw std::runtime_error("invalid OSM XML bounds");
				document.bounds = bounds;
			}
		} else if (event->name == "node") {
			if (node || way || relation)
				throw std::runtime_error("nested OSM element in XML input");
			node.emplace();
			node->id = xml_u64(*event, "id");
			if (event->attributes.contains("lat"))
				node->lat = xml_f64(*event, "lat");
			if (event->attributes.contains("lon"))
				node->lon = xml_f64(*event, "lon");
		} else if (event->name == "way") {
			if (node || way || relation)
				throw std::runtime_error("nested OSM element in XML input");
			way.emplace();
			way->id = xml_u64(*event, "id");
		} else if (event->name == "relation") {
			if (node || way || relation)
				throw std::runtime_error("nested OSM element in XML input");
			relation.emplace();
			relation->id = xml_u64(*event, "id");
		} else if (event->name == "tag") {
			if (node)
				set_xml_tag(node->tags, *event);
			else if (way)
				set_xml_tag(way->tags, *event);
			else if (relation)
				set_xml_tag(relation->tags, *event);
		} else if (event->name == "nd" && way) {
			if (event->attributes.contains("ref"))
				way->node_refs.push_back(xml_u64(*event, "ref"));
		} else if (event->name == "member" && relation) {
			RawRelationMember member;
			if (const auto it = event->attributes.find("type");
					it != event->attributes.end())
				member.type = it->second;
			if (event->attributes.contains("ref"))
				member.ref = xml_u64(*event, "ref");
			if (const auto it = event->attributes.find("role");
					it != event->attributes.end())
				member.role = it->second;
			relation->members.push_back(std::move(member));
		}
		if (event->empty && (event->name == "node" || event->name == "way" ||
									event->name == "relation"))
			finish_current();
	}
	if (node || way || relation)
		throw std::runtime_error("unterminated OSM XML element");
	document.completeness = analyze_completeness(document);
	if (!document.bounds)
		document.bounds = resolve_bbox(document);
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
	if (root.contains("bounds") && root["bounds"].is_object()) {
		const auto &b = root["bounds"];
		if (b.contains("minlat") && b.contains("minlon") && b.contains("maxlat") &&
				b.contains("maxlon") && b["minlat"].is_number() &&
				b["minlon"].is_number() && b["maxlat"].is_number() &&
				b["maxlon"].is_number())
			document.bounds = std::array<double, 4>{b["minlat"].get<double>(),
					b["minlon"].get<double>(), b["maxlat"].get<double>(),
					b["maxlon"].get<double>()};
	}
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
				(!element["id"].is_number_integer() &&
						!element["id"].is_number_unsigned()))
			continue;
		const auto type = element["type"].get<std::string>();
		const auto id = element["id"].get<std::uint64_t>();
		if (type == "node") {
			auto node_tags = parse_tags(element);
			filter_tags_impl(node_tags);
			const auto lat = element.contains("lat") && element["lat"].is_number()
									 ? std::optional<double>(element["lat"].get<double>())
									 : std::nullopt;
			const auto lon = element.contains("lon") && element["lon"].is_number()
									 ? std::optional<double>(element["lon"].get<double>())
									 : std::nullopt;
			document.nodes.push_back({id, lat, lon, std::move(node_tags)});
		} else if (type == "way") {
			RawWay way;
			way.id = id;
			way.tags = parse_tags(element);
			filter_tags_impl(way.tags);
			if (element.contains("nodes") && element["nodes"].is_array())
				for (const auto &node : element["nodes"])
					if (node.is_number_integer() || node.is_number_unsigned())
						way.node_refs.push_back(node.get<std::uint64_t>());
			document.ways.push_back(std::move(way));
		} else if (type == "relation") {
			RawRelation relation;
			relation.id = id;
			relation.tags = parse_tags(element);
			filter_tags_impl(relation.tags);
			if (element.contains("members") && element["members"].is_array())
				for (const auto &value : element["members"]) {
					if (!value.is_object())
						continue;
					RawRelationMember member;
					if (value.contains("type") && value["type"].is_string())
						member.type = value["type"].get<std::string>();
					if (value.contains("ref") &&
							(value["ref"].is_number_integer() ||
									value["ref"].is_number_unsigned()))
						member.ref = value["ref"].get<std::uint64_t>();
					if (value.contains("role") && value["role"].is_string())
						member.role = value["role"].get<std::string>();
					relation.members.push_back(std::move(member));
				}
			document.relations.push_back(std::move(relation));
		}
	}
	document.completeness = analyze_completeness(document);
	if (!document.bounds)
		document.bounds = resolve_bbox(document);
	report_incomplete_source(document.completeness);
	return document;
}

} // namespace arnis::osm_parser
