#include "discoveries.h"
#include "../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string_view>
#include <tuple>

namespace arnis::datapack::discoveries
{
namespace
{
constexpr int MAX_PLACES = 72;
constexpr int MAX_PER_CATEGORY = 16;
constexpr int POINT_RADIUS = 8;
constexpr int OUTLINE_PAD = 3;
constexpr int SETTLE_RADIUS = 48;
constexpr int STREET_SLACK = 6;

std::string_view tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? std::string_view{} : it->second;
}

bool has(const tags_t &tags, const char *key)
{
	return tags.find(key) != tags.end();
}

std::string trim(std::string_view text)
{
	const auto whitespace = [](std::uint32_t cp) {
		return (cp >= 0x09 && cp <= 0x0D) || cp == 0x20 || cp == 0x85 || cp == 0xA0 ||
			   cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200A) || cp == 0x2028 ||
			   cp == 0x2029 || cp == 0x202F || cp == 0x205F || cp == 0x3000;
	};
	const auto decode = [&](std::size_t offset, std::uint32_t &cp, std::size_t &bytes) {
		const auto first = static_cast<unsigned char>(text[offset]);
		if (first < 0x80) {
			cp = first;
			bytes = 1;
			return true;
		}
		if ((first & 0xE0) == 0xC0)
			bytes = 2;
		else if ((first & 0xF0) == 0xE0)
			bytes = 3;
		else if ((first & 0xF8) == 0xF0)
			bytes = 4;
		else
			return false;
		if (offset + bytes > text.size())
			return false;
		cp = first & (bytes == 2 ? 0x1F : bytes == 3 ? 0x0F : 0x07);
		for (std::size_t i = 1; i < bytes; ++i) {
			const auto continuation = static_cast<unsigned char>(text[offset + i]);
			if ((continuation & 0xC0) != 0x80)
				return false;
			cp = (cp << 6) | (continuation & 0x3F);
		}
		if ((bytes == 2 && cp < 0x80) || (bytes == 3 && cp < 0x800) ||
				(bytes == 4 && cp < 0x10000) || (cp >= 0xD800 && cp <= 0xDFFF) ||
				cp > 0x10FFFF)
			return false;
		return true;
	};
	std::size_t first = 0;
	while (first < text.size()) {
		std::uint32_t cp{};
		std::size_t bytes{};
		if (!decode(first, cp, bytes) || !whitespace(cp))
			break;
		first += bytes;
	}
	std::size_t last = text.size();
	while (last > first) {
		std::size_t start = last - 1;
		while (start > first && (static_cast<unsigned char>(text[start]) & 0xC0) == 0x80)
			--start;
		std::uint32_t cp{};
		std::size_t bytes{};
		if (!decode(start, cp, bytes) || start + bytes != last || !whitespace(cp))
			break;
		last = start;
	}
	return std::string(text.substr(first, last - first));
}

bool ascii_equal(std::string_view a, std::string_view b)
{
	if (a.size() != b.size())
		return false;
	const auto ascii_lower = [](unsigned char c) {
		return c >= 'A' && c <= 'Z' ? static_cast<unsigned char>(c + ('a' - 'A')) : c;
	};
	return std::equal(
			a.begin(), a.end(), b.begin(), [&](unsigned char x, unsigned char y) {
				return ascii_lower(x) == ascii_lower(y);
			});
}

int weight(Category category)
{
	switch (category) {
	case Category::Landmark:
		return 50;
	case Category::Culture:
		return 45;
	case Category::Viewpoint:
		return 35;
	case Category::Nature:
		return 30;
	case Category::Transport:
		return 30;
	case Category::Faith:
		return 25;
	case Category::Bridge:
		return 25;
	case Category::Sport:
		return 25;
	case Category::Civic:
		return 20;
	}
	return 0;
}

struct ScoredPlace
{
	Place place;
	int score = 0;
};

std::optional<unsigned> settlement_rank(const tags_t &tags)
{
	const auto p = tag(tags, "place");
	if (p == "city")
		return 6;
	if (p == "town")
		return 5;
	if (p == "borough" || p == "suburb")
		return 4;
	if (p == "village")
		return 3;
	if (p == "quarter" || p == "neighbourhood")
		return 2;
	if (p == "hamlet")
		return 1;
	return std::nullopt;
}

std::optional<std::pair<Category, const char *>> classify(const ProcessedElement &e)
{
	const auto &t = e.tags();
	const auto get = [&](const char *k) { return tag(t, k); };
	const bool node = e.is_node();
	const bool notable = has(t, "wikidata") || has(t, "wikipedia");

	const auto tourism = get("tourism");
	if (tourism == "museum")
		return std::make_pair(Category::Culture, "Museum");
	if (tourism == "gallery")
		return std::make_pair(Category::Culture, "Gallery");
	if (tourism == "zoo")
		return std::make_pair(Category::Culture, "Zoo");
	if (tourism == "aquarium")
		return std::make_pair(Category::Culture, "Aquarium");
	if (tourism == "theme_park")
		return std::make_pair(Category::Culture, "Theme park");
	if (tourism == "viewpoint")
		return std::make_pair(Category::Viewpoint, "Viewpoint");

	const auto historic = get("historic");
	if (historic == "castle")
		return std::make_pair(Category::Landmark, "Castle");
	if (historic == "palace")
		return std::make_pair(Category::Landmark, "Palace");
	if (historic == "monument")
		return std::make_pair(Category::Landmark, "Monument");
	if (historic == "city_gate")
		return std::make_pair(Category::Landmark, "City gate");
	if (historic == "fort")
		return std::make_pair(Category::Landmark, "Fort");
	if (historic == "ruins")
		return std::make_pair(Category::Landmark, "Ruins");
	if (historic == "tower")
		return std::make_pair(Category::Landmark, "Historic tower");
	if (historic == "archaeological_site")
		return std::make_pair(Category::Landmark, "Archaeological site");
	if (historic == "memorial" && notable)
		return std::make_pair(Category::Landmark, "Memorial");

	const auto amenity = get("amenity");
	if (amenity == "theatre")
		return std::make_pair(Category::Culture, "Theatre");
	if (amenity == "arts_centre")
		return std::make_pair(Category::Culture, "Arts centre");
	if (amenity == "library")
		return std::make_pair(Category::Culture, "Library");
	if (amenity == "concert_hall")
		return std::make_pair(Category::Culture, "Concert hall");
	if (amenity == "place_of_worship" && (!node || notable)) {
		const auto religion = get("religion");
		const char *label = religion == "christian" ? "Church"
							: religion == "muslim"	? "Mosque"
							: religion == "jewish"	? "Synagogue"
							: (religion == "buddhist" || religion == "hindu" ||
									  religion == "shinto" || religion == "sikh" ||
									  religion == "jain")
									? "Temple"
									: "Place of worship";
		return std::make_pair(Category::Faith, label);
	}
	if (amenity == "townhall")
		return std::make_pair(Category::Civic, "Town hall");
	if (amenity == "university")
		return std::make_pair(Category::Civic, "University");
	if (amenity == "courthouse")
		return std::make_pair(Category::Civic, "Courthouse");
	if (amenity == "ferry_terminal")
		return std::make_pair(Category::Transport, "Ferry terminal");
	if (amenity == "bus_station")
		return std::make_pair(Category::Transport, "Bus station");
	if (tourism == "attraction")
		return std::make_pair(Category::Landmark, "Attraction");

	const auto man_made = get("man_made");
	if (man_made == "lighthouse")
		return std::make_pair(Category::Landmark, "Lighthouse");
	if (man_made == "windmill")
		return std::make_pair(Category::Landmark, "Windmill");
	if (man_made == "obelisk")
		return std::make_pair(Category::Landmark, "Obelisk");
	if (man_made == "tower" && (notable || get("tower:type") == "observation"))
		return std::make_pair(Category::Landmark, "Tower");
	if (man_made == "bridge")
		return std::make_pair(Category::Bridge, "Bridge");
	if (get("natural") == "peak")
		return std::make_pair(Category::Viewpoint, "Peak");

	if (!node) {
		const auto bridge = get("bridge");
		if (!bridge.empty() && bridge != "no" && (has(t, "highway") || has(t, "railway")))
			return std::make_pair(Category::Bridge, "Bridge");
		const auto leisure = get("leisure"), natural = get("natural");
		const auto landuse = get("landuse");
		if (leisure == "park")
			return std::make_pair(Category::Nature, "Park");
		if (leisure == "garden")
			return std::make_pair(Category::Nature, "Garden");
		if (leisure == "nature_reserve")
			return std::make_pair(Category::Nature, "Nature reserve");
		if (leisure == "stadium")
			return std::make_pair(Category::Sport, "Stadium");
		if (leisure == "sports_centre")
			return std::make_pair(Category::Sport, "Sports centre");
		if (natural == "beach")
			return std::make_pair(Category::Nature, "Beach");
		if (natural == "water" && get("water") != "river" && get("water") != "canal" &&
				get("water") != "stream")
			return std::make_pair(Category::Nature, "Lake");
		if (natural == "wood" || landuse == "forest")
			return std::make_pair(Category::Nature, "Forest");
		if (get("place") == "island")
			return std::make_pair(Category::Nature, "Island");
	}
	const bool station = get("railway") == "station" ||
						 (get("public_transport") == "station" &&
								 (has(t, "railway") || get("train") == "yes" ||
										 get("subway") == "yes"));
	if (station)
		return std::make_pair(Category::Transport, "Station");
	if (get("aeroway") == "aerodrome")
		return std::make_pair(Category::Transport, "Airport");
	return std::nullopt;
}

struct Extent
{
	int min_x, min_z, max_x, max_z;
};

std::optional<std::pair<Extent, bool>> outline(const ProcessedElement &e)
{
	Extent b{std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
			std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest()};
	bool any = false;
	const auto add = [&](const ProcessedNode &n) {
		b.min_x = std::min(b.min_x, n.x);
		b.min_z = std::min(b.min_z, n.z);
		b.max_x = std::max(b.max_x, n.x);
		b.max_z = std::max(b.max_z, n.z);
		any = true;
	};
	if (e.is_node()) {
		add(e.as_node());
		return std::make_pair(b, true);
	}
	if (e.is_way()) {
		for (const auto &n : e.as_way().nodes)
			add(n);
	} else {
		for (const auto &member : e.as_relation().members)
			if (member.role != ProcessedMemberRole::Inner && member.way)
				for (const auto &n : member.way->nodes)
					add(n);
	}
	return any ? std::optional<std::pair<Extent, bool>>{{b, false}} : std::nullopt;
}

bool same_name(const Place &a, const Place &b)
{
	return a.category == b.category && ascii_equal(a.name, b.name);
}

std::vector<Place> select_places(std::vector<ScoredPlace> found)
{
	std::stable_sort(found.begin(), found.end(),
			[](const ScoredPlace &a, const ScoredPlace &b) { return a.score > b.score; });
	std::vector<ScoredPlace> merged;
	for (auto &p : found) {
		auto it = std::find_if(merged.begin(), merged.end(), [&](const ScoredPlace &m) {
			return same_name(m.place, p.place) &&
				   p.place.min.first <= m.place.max.first + 64 &&
				   p.place.max.first >= m.place.min.first - 64 &&
				   p.place.min.second <= m.place.max.second + 64 &&
				   p.place.max.second >= m.place.min.second - 64;
		});
		if (it != merged.end()) {
			it->place.min = {std::min(it->place.min.first, p.place.min.first),
					std::min(it->place.min.second, p.place.min.second)};
			it->place.max = {std::max(it->place.max.first, p.place.max.first),
					std::max(it->place.max.second, p.place.max.second)};
			it->place.notable = it->place.notable || p.place.notable;
			continue;
		}
		if (std::any_of(merged.begin(), merged.end(),
					[&](const ScoredPlace &m) { return same_name(m.place, p.place); }))
			continue;
		merged.push_back(std::move(p));
	}
	std::map<Category, int> counts;
	merged.erase(std::remove_if(merged.begin(), merged.end(),
						 [&](const ScoredPlace &p) {
							 return ++counts[p.place.category] > MAX_PER_CATEGORY;
						 }),
			merged.end());
	if (merged.size() > MAX_PLACES)
		merged.resize(MAX_PLACES);
	std::vector<Place> places;
	places.reserve(merged.size());
	for (auto &candidate : merged)
		places.push_back(std::move(candidate.place));
	return places;
}
} // namespace

const char *title(Category category)
{
	switch (category) {
	case Category::Landmark:
		return "Landmarks";
	case Category::Culture:
		return "Culture";
	case Category::Viewpoint:
		return "Views and Peaks";
	case Category::Nature:
		return "Parks and Nature";
	case Category::Faith:
		return "Places of Worship";
	case Category::Transport:
		return "Stations";
	case Category::Bridge:
		return "Bridges";
	case Category::Sport:
		return "Sport";
	case Category::Civic:
		return "Civic";
	}
	return "";
}

void Place::settle(const std::function<bool(int, int)> &open,
		const std::function<bool(int, int)> &street,
		const std::function<int(int, int)> &height_at)
{
	const auto [cx, cz] = anchor;
	auto first_open_at = [&](int r) -> std::optional<std::pair<int, int>> {
		for (int d = -r; d <= r; ++d) {
			const std::array<std::pair<int, int>, 4> cells{{{cx + d, cz - r},
					{cx + d, cz + r}, {cx - r, cz + d}, {cx + r, cz + d}}};
			for (const auto &cell : cells)
				if (open(cell.first, cell.second))
					return cell;
		}
		return std::nullopt;
	};
	std::optional<std::pair<int, int>> first;
	int first_radius = 0;
	for (int r = 0; r <= SETTLE_RADIUS; ++r)
		if ((first = first_open_at(r))) {
			first_radius = r;
			break;
		}
	if (first) {
		anchor = *first;
		for (int r = first_radius;
				r <= std::min(first_radius + STREET_SLACK, SETTLE_RADIUS); ++r) {
			std::optional<std::pair<int, int>> preferred;
			for (int d = -r; d <= r && !preferred; ++d) {
				const std::array<std::pair<int, int>, 4> cells{{{cx + d, cz - r},
						{cx + d, cz + r}, {cx - r, cz + d}, {cx + r, cz + d}}};
				for (const auto &cell : cells)
					if (open(cell.first, cell.second) &&
							street(cell.first, cell.second)) {
						preferred = cell;
						break;
					}
			}
			if (preferred) {
				anchor = *preferred;
				break;
			}
		}
	}
	ground_y = height_at(anchor.first, anchor.second);
}

std::pair<std::vector<Place>, std::optional<std::string>> collect(
		const std::vector<ProcessedElement> &elements, const cartesian::XZBBox &world)
{
	std::vector<ScoredPlace> found;
	std::optional<std::pair<unsigned, std::string>> settlement;
	for (const auto &e : elements) {
		const auto &tags = e.tags();
		if (const auto rank = settlement_rank(tags);
				rank && tags.contains("name") &&
				(!settlement || *rank > settlement->first))
			settlement = std::make_pair(*rank, tags.at("name"));
		const auto name_it = tags.find("name");
		if (name_it == tags.end() || trim(name_it->second).empty())
			continue;
		const auto classification = classify(e);
		const auto extent = outline(e);
		if (!classification || !extent)
			continue;
		const auto &[b, point] = *extent;
		const int pad = point ? POINT_RADIUS : OUTLINE_PAD;
		const std::int64_t area =
				point ? 0 : std::int64_t(b.max_x - b.min_x + 1) * (b.max_z - b.min_z + 1);
		const std::pair<int, int> lo{std::max(b.min_x - pad, world.min_x()),
				std::max(b.min_z - pad, world.min_z())};
		const std::pair<int, int> hi{std::min(b.max_x + pad, world.max_x()),
				std::min(b.max_z + pad, world.max_z())};
		if (lo.first > hi.first || lo.second > hi.second)
			continue;
		const auto [category, label] = *classification;
		const bool notable = has(tags, "wikidata") || has(tags, "wikipedia");
		int score = weight(category) + (has(tags, "wikidata") ? 100 : 0) +
					(has(tags, "wikipedia") ? 60 : 0) + (has(tags, "heritage") ? 30 : 0) +
					(tag(tags, "tourism") == "attraction" ? 20 : 0);
		if (area > 0)
			score += static_cast<int>(
					std::min(std::log2(static_cast<double>(area)) * 3.0, 30.0));
		const auto anchor =
				point ? std::pair{e.as_node().x, e.as_node().z}
					  : std::pair{(lo.first + hi.first) / 2, (lo.second + hi.second) / 2};
		found.push_back({Place{trim(name_it->second), category, label, lo, hi, anchor,
								 std::nullopt, notable},
				score});
	}
	return {select_places(std::move(found)),
			settlement ? std::optional<std::string>(settlement->second) : std::nullopt};
}
} // namespace arnis::datapack::discoveries
