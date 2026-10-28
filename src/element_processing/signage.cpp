#include "signage.h"
#include "../block_definitions.h"
#include "../fm_native_signs.h"
#include "../decals/font.h"
#include "../decals/pictograms.h"
#include "../bresenham.h"
#include "../strict_parse.h"
#include "road_mask_search.h"
#include "highways.h"
#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <iostream>
namespace arnis::signage
{
namespace arnis_highways = ::arnis::highways;
bool place_post(world_editor::WorldEditor &editor, int x, int z,
		const decals::DecalKey &key, bool all_faces = false, bool glow = false,
		std::int8_t selected_facing = 3);
namespace
{
std::string clean_street_name(const std::string &raw)
{
	std::string out;
	bool pending_space = false;
	for (unsigned char ch : raw) {
		if (std::isspace(ch)) {
			pending_space = !out.empty();
			continue;
		}
		if (pending_space && out.size() < 60)
			out.push_back(' ');
		pending_space = false;
		if (out.size() < 60)
			out.push_back(static_cast<char>(ch));
	}
	return out;
}

std::string clean_name(const std::string &raw)
{
	std::string out;
	std::size_t chars = 0;
	bool pending_space = false;
	for (std::size_t i = 0; i < raw.size();) {
		const auto lead = static_cast<unsigned char>(raw[i]);
		const std::size_t length = lead < 0x80			   ? 1
								   : (lead & 0xE0) == 0xC0 ? 2
								   : (lead & 0xF0) == 0xE0 ? 3
								   : (lead & 0xF8) == 0xF0 ? 4
														   : 1;
		const auto end = std::min(raw.size(), i + length);
		std::uint32_t cp = lead;
		if (length == 2 && end - i == 2)
			cp = ((lead & 0x1F) << 6) | (static_cast<unsigned char>(raw[i + 1]) & 0x3F);
		else if (length == 3 && end - i == 3)
			cp = ((lead & 0x0F) << 12) |
				 ((static_cast<unsigned char>(raw[i + 1]) & 0x3F) << 6) |
				 (static_cast<unsigned char>(raw[i + 2]) & 0x3F);
		else if (length == 4 && end - i == 4)
			cp = ((lead & 0x07) << 18) |
				 ((static_cast<unsigned char>(raw[i + 1]) & 0x3F) << 12) |
				 ((static_cast<unsigned char>(raw[i + 2]) & 0x3F) << 6) |
				 (static_cast<unsigned char>(raw[i + 3]) & 0x3F);
		const bool whitespace =
				(cp >= 0x09 && cp <= 0x0D) || cp == 0x20 || cp == 0x85 || cp == 0xA0 ||
				cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200A) || cp == 0x2028 ||
				cp == 0x2029 || cp == 0x202F || cp == 0x205F || cp == 0x3000;
		if (whitespace) {
			pending_space = !out.empty();
		} else if (chars < 60) {
			if (pending_space) {
				out.push_back(' ');
				++chars;
				pending_space = false;
			}
			if (chars < 60) {
				out.append(raw, i, end - i);
				++chars;
			}
		}
		i = end;
		if (chars >= 60)
			break;
	}
	return out;
}

std::string trim_ascii(std::string value)
{
	const auto first = value.find_first_not_of(" \t\r\n\f\v");
	if (first == std::string::npos)
		return {};
	const auto last = value.find_last_not_of(" \t\r\n\f\v");
	return value.substr(first, last - first + 1);
}

bool underground_building(const tags_t &tags)
{
	const auto location = tags.get("location");
	if (location == "underground" || location == "subway")
		return true;
	if (location == "surface" || location == "overground" || location == "roof")
		return false;
	if (strict_parse::i32_or_zero(tags.get("layer")) < 0 ||
			strict_parse::i32_or_zero(tags.get("level")) < 0)
		return true;
	return tags.contains("building:levels:underground") &&
		   !tags.contains("building:levels");
}

std::size_t utf8_char_count(std::string_view value)
{
	return static_cast<std::size_t>(std::count_if(value.begin(), value.end(),
			[](unsigned char c) { return (c & 0xC0) != 0x80; }));
}

std::uint8_t fascia_cols(const std::string &name)
{
	const auto chars = utf8_char_count(name);
	return chars <= 26 ? 2 : chars <= 44 ? 3 : 4;
}

bool named_street(const std::string &highway)
{
	static const std::set<std::string> kinds{"motorway", "trunk", "primary", "secondary",
			"tertiary", "motorway_link", "trunk_link", "primary_link", "secondary_link",
			"tertiary_link", "unclassified", "residential", "living_street", "pedestrian",
			"service", "road"};
	return kinds.contains(highway);
}

std::optional<std::pair<double, double>> way_direction_at(
		const std::vector<ProcessedNode> &nodes, std::size_t i)
{
	if (nodes.empty() || i >= nodes.size())
		return std::nullopt;
	const auto &prev = nodes[i ? i - 1 : i];
	const auto &next = nodes[i + 1 < nodes.size() ? i + 1 : i];
	double dx = next.x - prev.x, dz = next.z - prev.z;
	const double length = std::hypot(dx, dz);
	if (length <= 0.0)
		return std::nullopt;
	return std::pair{dx / length, dz / length};
}

bool signed_road(const std::string &h)
{
	static const std::set<std::string> kinds{"motorway", "motorway_link", "trunk",
			"trunk_link", "primary", "primary_link", "secondary", "secondary_link",
			"tertiary", "tertiary_link", "unclassified", "residential", "living_street"};
	return kinds.contains(h);
}
decals::ShieldStyle shield(
		const std::string &highway, const std::string &ref, decals::SignRegion region)
{
	if (region == decals::SignRegion::NorthAmerica)
		return ref.starts_with('I') ? decals::ShieldStyle::Interstate
									: decals::ShieldStyle::White;
	if (region == decals::SignRegion::UkIreland) {
		if (ref.starts_with('M') || highway.starts_with("motorway"))
			return decals::ShieldStyle::Blue;
		if (highway.starts_with("trunk") || ref.starts_with('N'))
			return decals::ShieldStyle::Green;
		return decals::ShieldStyle::WhitePlate;
	}
	if (region == decals::SignRegion::Germanic)
		return ref.starts_with('A') || highway.starts_with("motorway")
					   ? decals::ShieldStyle::Blue
					   : decals::ShieldStyle::Yellow;
	return highway.starts_with("motorway") || ref.starts_with('A') || ref.starts_with('E')
				   ? decals::ShieldStyle::Blue
				   : decals::ShieldStyle::Yellow;
}
std::optional<std::pair<std::uint16_t, bool>> maxspeed(
		const tags_t &tags, decals::SignRegion region)
{
	auto raw = tags.get("maxspeed");
	const auto first = raw.find_first_not_of(" \t\r\n\f\v");
	if (first == std::string::npos)
		return std::nullopt;
	const auto last = raw.find_last_not_of(" \t\r\n\f\v");
	raw = raw.substr(first, last - first + 1);
	std::transform(raw.begin(), raw.end(), raw.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	std::size_t n = 0;
	while (n < raw.size() && std::isdigit((unsigned char)raw[n]))
		++n;
	if (!n)
		return std::nullopt;
	unsigned value = 0;
	auto result = std::from_chars(raw.data(), raw.data() + n, value);
	if (result.ec != std::errc{} || value == 0 || value > 200)
		return std::nullopt;
	bool mph = raw.find("mph") != std::string::npos ||
			   (raw.find("km") == std::string::npos && decals::default_mph(region));
	return std::pair{std::uint16_t(value), mph};
}

struct NativeSign
{
	Block block;
	std::string text;
	bool facedir{false};
};

std::int8_t streets_facedir(std::int8_t facing)
{
	switch (facing) {
	case 2:
		return 0;
	case 3:
		return 2;
	case 4:
		return 1;
	case 5:
		return 3;
	default:
		return 0;
	}
}

std::optional<NativeSign> native_street_sign(
		const decals::DecalKey &key, decals::SignRegion region)
{
	using namespace block_definitions;
	Block block = AIR;
	std::string text;
	const bool eu = region == decals::SignRegion::Europe ||
					region == decals::SignRegion::Germanic ||
					region == decals::SignRegion::UkIreland;
	const bool us = region == decals::SignRegion::NorthAmerica;
	// Check each registered node independently; a missing unrelated sign must not
	// disable the rest of a mod's catalogue.
	const char *candidate = nullptr;
	if (const auto *traffic = std::get_if<decals::TrafficKey>(&key)) {
		switch (traffic->sign) {
		case decals::TrafficSign::PriorityRoad:
			if (eu)
				candidate = "streets:sign_eu_majorroad_center";
			break;
		case decals::TrafficSign::NoParking:
			candidate = eu	 ? "streets:sign_eu_noparking_center"
						: us ? "streets:sign_us_noparkingboth_center"
							 : nullptr;
			break;
		case decals::TrafficSign::DeadEnd:
			candidate = eu	 ? "streets:sign_eu_deadendstreet_center"
						: us ? "streets:sign_us_deadend_center"
							 : nullptr;
			break;
		case decals::TrafficSign::OneWay:
			candidate = eu	 ? "streets:sign_eu_onewayright_center"
						: us ? "streets:sign_us_onewayright_center"
							 : nullptr;
			break;
		case decals::TrafficSign::Bicycle:
			candidate = eu	 ? "streets:sign_eu_cyclepath_center"
						: us ? "streets:sign_us_bikelane_center"
							 : nullptr;
			break;
		case decals::TrafficSign::Motorway:
			if (eu)
				candidate = "streets:sign_eu_highway_center";
			break;
		case decals::TrafficSign::MotorwayEnd:
			if (eu)
				candidate = "streets:sign_eu_highwayend_center";
			break;
		default:
			break;
		}
	} else if (const auto *icon = std::get_if<decals::PictogramKey>(&key)) {
		if (icon->name == "parking")
			candidate = "streets:sign_eu_parkingsite_center";
		else if (icon->name == "information")
			candidate = "streets:sign_eu_info_center";
		else if (eu && icon->name == "bus_stop")
			candidate = "streets:sign_eu_busstation_center";
	} else if (const auto *speed = std::get_if<decals::SpeedLimitKey>(&key)) {
		if (us && speed->mph && speed->value == 50)
			candidate = "streets:sign_us_speedlimit50_center";
	} else if (const auto *label = std::get_if<decals::TextKey>(&key)) {
		const auto length = utf8_char_count(label->text);
		if (label->text.find('\n') == std::string::npos) {
			if (label->style.kind == decals::TextStyleKind::StreetName && length <= 40) {
				block = native_sign_node("street_signs:sign_basic_top_only");
				if (block.id() != CONTENT_AIR)
					return NativeSign{block, label->text, true};
			}
			if (length <= 15 &&
					(label->style.kind == decals::TextStyleKind::HouseNumber ||
							label->style.kind == decals::TextStyleKind::Plaque)) {
				block = native_sign_node("default:sign_wall_steel");
				if (block.id() == CONTENT_AIR)
					block = native_sign_node("default:sign_wall_wood");
			} else if (length <= 50) {
				block = native_sign_node("street_signs:sign_highway_small_green");
			} else if (length <= 80) {
				block = native_sign_node("street_signs:sign_highway_large_green");
			}
			if (block.id() != CONTENT_AIR)
				return NativeSign{block, label->text, false};
		}
	}
	if (candidate) {
		block = native_sign_node(candidate);
		if (block.id() != CONTENT_AIR)
			return NativeSign{block, {}, true};
	}
	if ((eu || us) && std::holds_alternative<decals::TrafficKey>(key)) {
		const auto sign = std::get<decals::TrafficKey>(key).sign;
		switch (sign) {
		case decals::TrafficSign::Stop:
			block = eu ? STREETS_EU_SIGN_STOP : STREETS_US_SIGN_STOP;
			break;
		case decals::TrafficSign::GiveWay:
			block = eu ? STREETS_EU_SIGN_YIELD : STREETS_US_SIGN_YIELD;
			break;
		case decals::TrafficSign::NoEntry:
			block = eu ? STREETS_EU_SIGN_NO_ENTRY : STREETS_US_SIGN_NO_ENTRY;
			break;
		case decals::TrafficSign::Crossing:
			block = eu ? STREETS_EU_SIGN_CROSSING : STREETS_US_SIGN_CROSSING;
			break;
		case decals::TrafficSign::LevelCrossing:
			block = eu ? STREETS_EU_SIGN_CROSSBUCK : STREETS_US_SIGN_CROSSBUCK;
			break;
		default:
			break;
		}
		if (block.id() != CONTENT_AIR)
			return NativeSign{block, {}, true};
	}
	if (eu && std::holds_alternative<decals::SpeedLimitKey>(key)) {
		const auto &speed = std::get<decals::SpeedLimitKey>(key);
		if (!speed.mph) {
			constexpr std::array<std::uint16_t, 6> values{{10, 30, 50, 70, 100, 120}};
			const auto found = std::find(values.begin(), values.end(), speed.value);
			if (found != values.end()) {
				block = STREETS_EU_SPEED_SIGNS[static_cast<std::size_t>(
						std::distance(values.begin(), found))];
				if (block.id() != CONTENT_AIR)
					return NativeSign{block, {}, true};
			}
		}
	}

	block = AIR;
	if (std::holds_alternative<decals::TrafficKey>(key)) {
		switch (std::get<decals::TrafficKey>(key).sign) {
		case decals::TrafficSign::Stop:
			block = STREET_SIGN_STOP;
			break;
		case decals::TrafficSign::GiveWay:
			block = STREET_SIGN_YIELD;
			break;
		case decals::TrafficSign::NoEntry:
			block = STREET_SIGN_DO_NOT_ENTER;
			break;
		case decals::TrafficSign::Crossing:
			block = STREET_SIGN_PEDESTRIAN_CROSSING;
			break;
		case decals::TrafficSign::LevelCrossing:
			block = STREET_SIGN_RR_CROSSBUCK;
			break;
		default:
			break;
		}
	} else if (std::holds_alternative<decals::SpeedLimitKey>(key)) {
		const auto &speed = std::get<decals::SpeedLimitKey>(key);
		block = STREET_SIGN_SPEED_LIMIT;
		text = std::to_string(speed.value);
	} else if (std::holds_alternative<decals::RouteShieldKey>(key)) {
		const auto &route = std::get<decals::RouteShieldKey>(key);
		if (route.style == decals::ShieldStyle::Interstate)
			block = STREET_SIGN_US_INTERSTATE;
		else if (route.style == decals::ShieldStyle::White)
			block = STREET_SIGN_US_ROUTE;
		text = route.text;
	}
	if (block.id() == CONTENT_AIR)
		return std::nullopt;
	return NativeSign{block, std::move(text), false};
}

std::int8_t facing_for_dir(double dx, double dz)
{
	if (std::abs(dx) >= std::abs(dz))
		return dx >= 0.0 ? 5 : 4;
	return dz >= 0.0 ? 3 : 2;
}

std::int8_t opposite(std::int8_t f)
{
	switch (f) {
	case 2:
		return 3;
	case 3:
		return 2;
	case 4:
		return 5;
	case 5:
		return 4;
	default:
		return f;
	}
}

std::pair<int, int> right_dir(std::int8_t facing)
{
	// facing: 2=N, 3=S, 4=W, 5=E
	// right perpendicular direction
	switch (facing) {
	case 2:
		return {1, 0}; // N -> E
	case 3:
		return {-1, 0}; // S -> W
	case 4:
		return {0, -1}; // W -> S
	case 5:
		return {0, 1}; // E -> N
	default:
		return {0, 1};
	}
}

std::optional<std::pair<int, int>> get_nearest_road_block(
		int x, int z, int radius, const RoadMaskBitmap &road_mask)
{
	return element_processing::get_nearest_road_block(x, z, radius, road_mask);
}

void place_roadside_sign(world_editor::WorldEditor &editor, const char *kind,
		const std::vector<std::pair<int, int>> &cells, std::size_t idx, int half_width,
		const decals::DecalKey &key, bool reverse,
		const BuildingFootprintBitmap &footprints)
{
	if (cells.size() < 3 || !editor.signage_context)
		return;
	const auto &ctx = *editor.signage_context;
	idx = std::clamp<std::size_t>(idx, 1, cells.size() - 2);
	const int x = cells[idx].first;
	const int z = cells[idx].second;
	// Rust gates placement on ownership of the road sample as well as the
	// eventual post cell. This keeps adjacent tile passes from claiming the
	// same roadside sign through opposite shoulders.
	if (!editor.owns(x, z))
		return;
	if (!ctx.has(key))
		return;
	const int bx = cells[idx - 1].first, bz = cells[idx - 1].second;
	const int fx = cells[idx + 1].first, fz = cells[idx + 1].second;
	double dx = fx - bx, dz = fz - bz;
	const double len = std::hypot(dx, dz);
	if (len == 0.0)
		return;
	dx /= len;
	dz /= len;
	if (reverse) {
		dx = -dx;
		dz = -dz;
	}
	const double px = decals::drives_on_left(ctx.region) ? dz : -dz;
	const double pz = decals::drives_on_left(ctx.region) ? -dx : dx;
	for (int k = 0; k < 5; ++k) {
		const double offset = half_width + 2 + k;
		const int sx = x + static_cast<int>(std::round(px * offset));
		const int sz = z + static_cast<int>(std::round(pz * offset));
		if (!editor.owns(sx, sz) || ctx.carriageway.contains(sx, sz))
			continue;
		if (footprints.contains(sx, sz) || editor.is_lc_water(sx, sz))
			continue;
		const std::int8_t facing = facing_for_dir(-dx, -dz);
		if (place_post(editor, sx, sz, key, false, false, facing)) {
			const int head = editor.get_absolute_y(sx, 0, sz) + 3;
			ctx.note(kind, sx, head, sz);
			return;
		}
	}
}

} // anonymous namespace

bool place_native_sign_panel(world_editor::WorldEditor &editor, int x, int y, int z,
		std::int8_t facing, const decals::DecalKey &key, bool require_hosts)
{
	if (facing < 2 || facing > 5)
		return false;
	const auto region = editor.signage_context ? editor.signage_context->region
											   : decals::SignRegion::Europe;
	const auto native = native_street_sign(key, region);
	if (!native || (require_hosts && editor.cell_open_at(x, y, z)))
		return false;
	const auto [fx, fy, fz] =
			world_editor::WorldEditor::decal_frame_cell(x, y, z, facing);
	if (!editor.owns(fx, fz) || fy - editor.get_ground_level(fx, fz) < 1 ||
			editor.cell_has_frame(fx, fy, fz) || !editor.cell_open_at(fx, fy, fz))
		return false;
	return editor.place_sign_node(native->block, fx, fy, fz,
			native->facedir ? streets_facedir(facing) : facing, native->text);
}

IntersectionIndex build_intersection_index(
		const std::vector<ProcessedElement> &elements, double scale)
{
	struct Street
	{
		const ProcessedWay *way;
		std::string name;
		int half_width;
	};
	struct Hit
	{
		std::uint64_t way_id;
		std::string name;
		std::pair<double, double> direction;
		int half_width;
	};
	std::vector<Street> streets;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway = way.tags.get("highway");
		if (!named_street(highway) || way.tags.get("area") == "yes" ||
				(way.tags.contains("tunnel") && way.tags.get("tunnel") != "no") ||
				way.tags.get("indoor") == "yes")
			continue;
		auto name = clean_street_name(way.tags.get("name"));
		if (name.empty() || !decals::font::supports(name))
			continue;
		streets.push_back({&way, std::move(name),
				arnis_highways::highway_block_range(highway, way.tags, scale)});
	}

	std::unordered_map<std::uint64_t, unsigned> counts;
	for (const auto &street : streets)
		for (const auto &node : street.way->nodes)
			++counts[node.id];
	std::unordered_map<std::uint64_t, std::vector<Hit>> by_node;
	for (const auto &street : streets) {
		for (std::size_t i = 0; i < street.way->nodes.size(); ++i) {
			const auto &node = street.way->nodes[i];
			if (counts[node.id] < 2)
				continue;
			if (auto direction = way_direction_at(street.way->nodes, i))
				by_node[node.id].push_back(
						{street.way->id, street.name, *direction, street.half_width});
		}
	}

	IntersectionIndex result;
	for (auto &[node_id, hits] : by_node) {
		std::map<std::string, std::vector<const Hit *>> names;
		std::uint64_t owner = std::numeric_limits<std::uint64_t>::max();
		for (const auto &hit : hits) {
			names[hit.name].push_back(&hit);
			owner = std::min(owner, hit.way_id);
		}
		if (names.size() < 2)
			continue;
		IntersectionPost post;
		post.owner_way = owner;
		for (const auto &[name, same_name] : names) {
			if (post.blades.size() == 3)
				break;
			std::pair<double, double> sum{0.0, 0.0};
			int half_width = 0;
			for (const Hit *hit : same_name) {
				auto direction = hit->direction;
				if (direction.first * sum.first + direction.second * sum.second < 0.0) {
					direction.first = -direction.first;
					direction.second = -direction.second;
				}
				sum.first += direction.first;
				sum.second += direction.second;
				half_width = std::max(half_width, hit->half_width);
			}
			const double length = std::hypot(sum.first, sum.second);
			if (length > 0.0) {
				sum.first /= length;
				sum.second /= length;
			} else {
				sum = {1.0, 0.0};
			}
			post.blades.push_back({name, sum, half_width});
		}
		result.emplace(node_id, std::move(post));
	}
	return result;
}

WaySigns highway_way_signs(const tags_t &tags, decals::SignRegion region)
{
	WaySigns out;
	const auto highway = tags.get("highway");
	const auto tunnel = tags.find("tunnel");
	if (tags.get("area") == "yes" || (tunnel != tags.end() && tunnel->second != "no") ||
			tags.get("indoor") == "yes")
		return out;
	if (highway == "cycleway") {
		out.cycleway = decals::TrafficKey{decals::TrafficSign::Bicycle};
		return out;
	}
	if (!signed_road(highway))
		return out;
	if (tags.get("oneway") == "yes" || tags.get("oneway") == "-1")
		out.no_entry = decals::TrafficKey{decals::TrafficSign::NoEntry};
	if (auto speed = maxspeed(tags, region))
		out.speed = decals::SpeedLimitKey{
				speed->first, speed->second, decals::speed_style(region)};
	if (highway == "motorway" || highway == "trunk" || highway == "primary" ||
			highway == "secondary" || highway == "tertiary") {
		auto ref = tags.get("ref");
		if (auto semi = ref.find(';'); semi != std::string::npos)
			ref.resize(semi);
		const auto first = ref.find_first_not_of(" \t\r\n\f\v");
		if (first == std::string::npos)
			ref.clear();
		else
			ref = ref.substr(first, ref.find_last_not_of(" \t\r\n\f\v") - first + 1);
		std::size_t chars = 0, bytes = 0;
		while (bytes < ref.size() && chars < 6) {
			const auto lead = static_cast<unsigned char>(ref[bytes]);
			const std::size_t length = lead < 0x80			   ? 1
									   : (lead & 0xE0) == 0xC0 ? 2
									   : (lead & 0xF0) == 0xE0 ? 3
									   : (lead & 0xF8) == 0xF0 ? 4
															   : 1;
			bytes = std::min(ref.size(), bytes + length);
			++chars;
		}
		if (bytes < ref.size())
			ref.resize(bytes);
		if (!ref.empty() && decals::font::supports(ref))
			out.shield = decals::RouteShieldKey{shield(highway, ref, region), ref};
	}
	return out;
}
std::optional<decals::DecalKey> highway_node_sign(const tags_t &tags, SignageLevel level)
{
	if (tags.get("railway") == "level_crossing")
		return decals::TrafficKey{decals::TrafficSign::LevelCrossing};
	const auto h = tags.get("highway");
	if (h == "stop")
		return decals::TrafficKey{decals::TrafficSign::Stop};
	if (h == "give_way")
		return decals::TrafficKey{decals::TrafficSign::GiveWay};
	if (h == "crossing" && level == SignageLevel::Full &&
			tags.get("crossing") != "unmarked" && tags.get("crossing") != "no" &&
			tags.get("crossing") != "traffic_signals")
		return decals::TrafficKey{decals::TrafficSign::Crossing};
	return std::nullopt;
}

namespace
{
bool minor_service_road(const ProcessedWay &way)
{
	const auto service = way.tags.get("service");
	return service == "driveway" || service == "parking_aisle" ||
		   service == "drive-through";
}

struct SignRoadNetwork
{
	std::vector<const ProcessedWay *> roads;
	std::unordered_map<std::uint64_t, std::pair<unsigned, unsigned>> ends;
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> tips;

	explicit SignRoadNetwork(const std::vector<ProcessedElement> &elements)
	{
		for (const auto &element : elements) {
			if (!element.is_way())
				continue;
			const auto &way = element.as_way();
			const auto type = highways::surface_highway_type(way);
			if (way.nodes.size() < 2 || !type || !highways::is_carriageway(*type))
				continue;
			const std::size_t road = roads.size();
			roads.push_back(&way);
			const std::size_t last = way.nodes.size() - 1;
			const bool minor = minor_service_road(way);
			for (std::size_t i = 0; i < way.nodes.size(); ++i) {
				const auto &node = way.nodes[i];
				const bool tip = i == 0 || i == last;
				const unsigned add = tip ? 1 : 2;
				auto &count = ends[node.id];
				count.first = std::min(255u, count.first + add);
				if (!minor)
					count.second = std::min(255u, count.second + add);
				if (tip)
					tips[node.id].push_back(road);
			}
		}
	}

	bool is_junction(std::uint64_t id, bool minor) const
	{
		const auto found = ends.find(id);
		if (found == ends.end())
			return false;
		return (minor ? found->second.first : found->second.second) >= 3;
	}

	std::optional<double> distance_to_junction(
			std::size_t road, std::size_t index, bool forward) const
	{
		constexpr double max_walk = 400.0;
		const bool minor = minor_service_road(*roads[road]);
		double distance = 0.0;
		for (unsigned hop = 0; hop < 8; ++hop) {
			const auto &nodes = roads[road]->nodes;
			for (;;) {
				if ((forward && index + 1 >= nodes.size()) || (!forward && index == 0))
					break;
				const std::size_t next = forward ? index + 1 : index - 1;
				distance += std::hypot(double(nodes[next].x - nodes[index].x),
						double(nodes[next].z - nodes[index].z));
				index = next;
				if (is_junction(nodes[index].id, minor))
					return distance;
			}
			if (distance > max_walk)
				return std::nullopt;
			const auto tip_id = nodes[index].id;
			const auto found = tips.find(tip_id);
			if (found == tips.end())
				return std::nullopt;
			std::optional<std::size_t> onward;
			for (const auto candidate : found->second) {
				if (candidate == road ||
						(!minor && minor_service_road(*roads[candidate])))
					continue;
				if (onward)
					return std::nullopt;
				onward = candidate;
			}
			if (!onward)
				return std::nullopt;
			road = *onward;
			const auto &next_nodes = roads[road]->nodes;
			if (next_nodes.front().id == tip_id) {
				index = 0;
				forward = true;
			} else {
				index = next_nodes.size() - 1;
				forward = false;
			}
		}
		return std::nullopt;
	}
};
} // namespace

std::unordered_map<std::uint64_t, NodeSignHeading> build_node_headings(
		const std::vector<ProcessedElement> &elements, SignageLevel level)
{
	std::unordered_map<std::uint64_t, std::pair<bool, std::optional<bool>>> signs;
	for (const auto &element : elements) {
		if (!element.is_node())
			continue;
		const auto &node = element.as_node();
		const auto key = highway_node_sign(node.tags, level);
		if (!key)
			continue;
		const auto *traffic = std::get_if<decals::TrafficKey>(&*key);
		const bool one_way =
				traffic && (traffic->sign == decals::TrafficSign::Stop ||
								   traffic->sign == decals::TrafficSign::GiveWay);
		const auto direction = node.tags.get("direction");
		std::optional<bool> forward;
		if (direction == "forward")
			forward = true;
		else if (direction == "backward")
			forward = false;
		signs.emplace(node.id, std::make_pair(one_way, forward));
	}
	std::unordered_map<std::uint64_t, NodeSignHeading> result;
	if (signs.empty())
		return result;
	const SignRoadNetwork net(elements);
	for (std::size_t road = 0; road < net.roads.size(); ++road) {
		const auto &way = *net.roads[road];
		const bool minor = minor_service_road(way);
		const auto oneway = way.tags.get("oneway");
		std::optional<bool> flow;
		if (oneway == "yes" || oneway == "true" || oneway == "1")
			flow = true;
		else if (oneway == "-1")
			flow = false;
		else if (way.tags.get("junction") == "roundabout")
			flow = true;
		for (std::size_t i = 0; i < way.nodes.size(); ++i) {
			const auto &node = way.nodes[i];
			const auto sign = signs.find(node.id);
			if (sign == signs.end() || result.contains(node.id) ||
					net.is_junction(node.id, minor))
				continue;
			const auto direction = way_direction_at(way.nodes, i);
			if (!direction)
				continue;
			bool ahead = true;
			if (sign->second.first) {
				if (sign->second.second)
					ahead = *sign->second.second;
				else if (flow)
					ahead = *flow;
				else {
					const auto forward = net.distance_to_junction(road, i, true);
					const auto backward = net.distance_to_junction(road, i, false);
					if (forward && backward)
						ahead = *forward <= *backward;
					else if (forward)
						ahead = true;
					else if (backward)
						ahead = false;
					else
						continue;
				}
			}
			auto dir = *direction;
			if (!ahead) {
				dir.first = -dir.first;
				dir.second = -dir.second;
			}
			result.emplace(node.id, NodeSignHeading{dir, !sign->second.first});
		}
	}
	return result;
}
std::optional<decals::DecalKey> power_sign(const tags_t &tags)
{
	const auto p = tags.get("power");
	return p == "substation" || p == "plant" || p == "generator" || p == "transformer"
				   ? std::optional<decals::DecalKey>{decals::TrafficKey{
							 decals::TrafficSign::HighVoltage}}
				   : std::nullopt;
}
std::vector<decals::DecalKey> advertising_keys(const tags_t &tags, std::uint64_t id)
{
	std::vector<decals::DecalKey> out;
	const auto a = tags.get("advertising");
	if (a == "billboard")
		out.emplace_back(decals::PosterKey{std::uint8_t(id % 6)});
	else if (a == "column")
		for (unsigned i = 0; i < 4; ++i)
			out.emplace_back(decals::ColumnPosterKey{std::uint8_t((id + i) % 5)});
	else if (a == "poster_box") {
		out.emplace_back(decals::ColumnPosterKey{std::uint8_t(id % 5)});
		out.emplace_back(decals::ColumnPosterKey{std::uint8_t((id + 2) % 5)});
	}
	return out;
}
std::optional<decals::DecalKey> information_key(const ProcessedNode &node)
{
	if (node.tags.get("tourism") != "information")
		return std::nullopt;
	const auto type = node.tags.get("information");
	if (type == "office" || type == "visitor_centre")
		return std::nullopt;
	if (type == "map" || type == "board" || type == "terminal")
		return decals::LocalMapKey{node.x, node.z};
	return decals::PictogramKey{"information"};
}
std::optional<decals::DecalKey> furniture_pictogram(const tags_t &tags)
{
	const auto a = tags.get("amenity");
	if ((a == "recycling" && tags.get("recycling_type") == "container") ||
			a == "waste_basket" || a == "waste_disposal")
		return decals::PictogramKey{"recycling"};
	if (a == "vending_machine")
		return decals::PictogramKey{"vending_machine"};
	if (a == "atm")
		return decals::PictogramKey{"atm"};
	if (tags.get("emergency") == "fire_hydrant" &&
			tags.get("fire_hydrant:type") != "underground" &&
			tags.get("fire_hydrant:type") != "wall" &&
			tags.get("fire_hydrant:type") != "pond")
		return decals::PictogramKey{"hydrant"};
	return std::nullopt;
}

void SignageContext::note(const char *kind, int x, int y, int z) const
{
	if (!kind)
		return;
	std::lock_guard<std::mutex> lock(report_mutex);
	auto &[count, samples] = report[kind];
	++count;
	if (samples.size() < 4)
		samples.emplace(x, y, z);
}

std::string SignageContext::summary(bool verbose) const
{
	std::lock_guard<std::mutex> lock(report_mutex);
	if (report.empty())
		return "Signage: nothing placed";
	std::ostringstream out;
	out << "Signage placed: ";
	bool first = true;
	for (const auto &[kind, entry] : report) {
		if (!first)
			out << ", ";
		first = false;
		out << entry.first << ' ' << kind;
	}
	if (verbose)
		for (const auto &[kind, entry] : report) {
			out << "\n  " << kind << ": ";
			bool first_sample = true;
			for (const auto &[x, y, z] : entry.second) {
				if (!first_sample)
					out << "  ";
				first_sample = false;
				out << "/tp " << x << ' ' << y << ' ' << z;
			}
		}
	return out.str();
}

std::shared_ptr<const decals::DecalRegistry> build_registry(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region, double scale, int first_map_id)
{
	if (level == SignageLevel::None)
		return {};
	auto intersections = std::make_shared<IntersectionIndex>(
			build_intersection_index(elements, scale));
	std::set<decals::DecalKey> keys;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			// Match Rust's key pre-pass: collect any tagged building way here.
			// poi_name()/house_number() apply their own signage-level filtering;
			// even unused building:part keys affect the deterministic decal ID map.
			if (way.tags.contains("building") && !underground_building(way.tags)) {
				if (const auto name = poi_name(way.tags, level); name && name->key)
					keys.insert(*name->key);
				if (const auto number = house_number(way.tags, level))
					keys.insert(*number);
			}
			auto signs = highway_way_signs(way.tags, region);
			for (const auto *key :
					{&signs.speed, &signs.shield, &signs.no_entry, &signs.cycleway})
				if (*key)
					keys.insert(**key);
			if (way.nodes.size() > 2) {
				if (auto key = power_sign(way.tags))
					keys.insert(*key);
			}
			if (way.nodes.size() > 2 && way.tags.get("amenity") == "parking")
				keys.insert(decals::PictogramKey{"parking"});
		} else if (element.is_node()) {
			const auto &node = element.as_node();
			const auto &tags = node.tags;
			if (const auto name = poi_name(tags, level); name && name->key)
				keys.insert(*name->key);
			if (auto key = highway_node_sign(node.tags, level))
				keys.insert(*key);
			if (auto key = information_key(node))
				keys.insert(*key);
			if (auto key = furniture_pictogram(node.tags))
				keys.insert(*key);
			for (auto &key : advertising_keys(node.tags, node.id))
				keys.insert(std::move(key));
			if (tags.get("highway") == "bus_stop") {
				keys.insert(decals::PictogramKey{"bus_stop"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StopName}, name, 1));
			}
			const auto railway = tags.get("railway");
			const bool subway =
					(railway == "station" || railway == "halt") &&
					(tags.get("station") == "subway" || tags.get("subway") == "yes");
			if ((railway == "station" || railway == "halt") && !subway) {
				keys.insert(decals::PictogramKey{"train"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StationBoard}, name, 3));
			} else if (railway == "tram_stop") {
				keys.insert(decals::PictogramKey{"tram"});
				const auto name = clean_name(tags.get("name"));
				if (!name.empty() && decals::font::supports(name))
					keys.insert(decals::DecalKey::text(
							{decals::TextStyleKind::StopName}, name, 1));
			} else if (railway == "subway_entrance")
				keys.insert(decals::PictogramKey{decals::metro_logo(region)});
			else if (subway)
				keys.insert(decals::PictogramKey{decals::metro_logo(region)});
			if (auto key = plaque_key(tags))
				keys.insert(*key);
		}
	}
	const decals::TextStyle blade_style{
			decals::TextStyleKind::StreetName, decals::blade_style(region)};
	const auto tile_count = [](const decals::DecalKey &key) {
		const auto [cols, rows] = key.dims();
		return cols * rows;
	};
	std::uint32_t total_tiles = 0;
	for (const auto &key : keys)
		total_tiles += tile_count(key);
	constexpr std::uint32_t max_decal_tiles = 30'000;
	if (total_tiles > max_decal_tiles) {
		const auto is_building_key = [](const decals::DecalKey &key) {
			const auto *text = std::get_if<decals::TextKey>(&key);
			return text &&
				   (text->style.kind == decals::TextStyleKind::Fascia ||
						   text->style.kind == decals::TextStyleKind::HouseNumber);
		};
		const auto before = keys.size();
		std::erase_if(keys, is_building_key);
		std::uint32_t remaining_tiles = 0;
		for (const auto &key : keys)
			remaining_tiles += tile_count(key);
		std::cerr << "Note: signage capped at " << max_decal_tiles << " map tiles ("
				  << total_tiles << " needed); dropped " << before - keys.size()
				  << " building name plates, " << remaining_tiles << " tiles remain.\n";
	}
	for (const auto &[node_id, post] : *intersections) {
		(void)node_id;
		for (const auto &blade : post.blades)
			keys.insert(decals::DecalKey::text(blade_style, blade.name, 1));
	}
	return keys.empty() ? std::shared_ptr<const decals::DecalRegistry>{}
						: std::make_shared<const decals::DecalRegistry>(
								  decals::DecalRegistry::from_keys_starting_at(
										  keys, first_map_id));
}

std::shared_ptr<const SignageContext> build_context(
		const std::vector<ProcessedElement> &elements, SignageLevel level,
		decals::SignRegion region, double scale, const RoadMaskBitmap &carriageway,
		int first_map_id)
{
	if (level == SignageLevel::None)
		return {};
	auto registry = build_registry(elements, level, region, scale, first_map_id);
	if (!registry)
		return {};
	auto context = std::make_shared<SignageContext>();
	context->registry = std::move(registry);
	context->level = level;
	context->region = region;
	context->intersections = build_intersection_index(elements, scale);
	context->node_headings = build_node_headings(elements, level);
	context->scale = scale;
	context->carriageway = carriageway;
	return context;
}

bool place_post(world_editor::WorldEditor &editor, int x, int z,
		const decals::DecalKey &key, bool all_faces, bool glow,
		std::int8_t selected_facing)
{
	if (!editor.signage_enabled() || !editor.owns(x, z) ||
			!editor.decal_registry->contains(key))
		return false;
	const int ground = editor.get_absolute_y(x, 0, z);
	for (int dy = 1; dy <= 3; ++dy)
		if (!editor.cell_open_at(x, ground + dy, z) ||
				editor.cell_has_frame(x, ground + dy, z))
			return false;
	const auto region = editor.signage_context ? editor.signage_context->region
											   : decals::SignRegion::Europe;
	if (!all_faces && block_definitions::STREETS_RRXING_AVAILABLE &&
			std::holds_alternative<decals::TrafficKey>(key) &&
			std::get<decals::TrafficKey>(key).sign ==
					decals::TrafficSign::LevelCrossing) {
		const auto param2 = streets_facedir(selected_facing);
		for (const auto &[dy, source] : std::array<std::pair<int, Block>, 3>{
					 {{1, block_definitions::STREETS_RRXING_BOTTOM},
							 {2, block_definitions::STREETS_RRXING_MIDDLE},
							 {3, block_definitions::STREETS_RRXING_TOP}}}) {
			auto block = source;
			block.setParam2(static_cast<std::uint8_t>(param2));
			editor.set_block_absolute(block, x, ground + dy, z);
		}
		return true;
	}
	editor.set_block_absolute(block_definitions::STREETS_POLE, x, ground + 1, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::STREETS_POLE, x, ground + 2, z,
			std::nullopt, std::nullopt);
	const int head = ground + 3;
	// Prefer street_signs' purpose-built mesh and pole mount. Text-bearing signs
	// receive the same generated metadata as default/signs_lib wall signs.
	if (!all_faces)
		if (const auto native = native_street_sign(key, region);
				native && editor.place_sign_node(native->block, x, head, z,
								  native->facedir ? streets_facedir(selected_facing)
												  : selected_facing,
								  native->text))
			return true;
	editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z,
			std::nullopt, std::nullopt);
	bool placed = false;
	for (const std::int8_t facing : std::array<std::int8_t, 4>{2, 3, 4, 5}) {
		placed |= editor.place_decal_panel(
				x, head, z, all_faces ? facing : selected_facing, key, glow, false);
		if (!all_faces)
			break;
	}
	return placed;
}

void place_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		SignageLevel level, decals::SignRegion region)
{
	(void)region;
	if (!editor.signage_enabled() || !editor.owns(node.x, node.z))
		return;
	if (const auto key = highway_node_sign(node.tags, level)) {
		place_post(editor, node.x, node.z, *key);
		return;
	}
	if (const auto key = information_key(node)) {
		place_post(editor, node.x, node.z, *key, true);
		return;
	}
	if (const auto key = furniture_pictogram(node.tags)) {
		place_post(editor, node.x, node.z, *key, true);
		return;
	}

	const int ground = editor.get_absolute_y(node.x, 0, node.z);
	if (node.tags.get("advertising") == "column") {
		for (int face = 0; face < 4; ++face)
			editor.place_decal_panel(node.x, ground + 3, node.z, std::int8_t(face + 2),
					decals::ColumnPosterKey{std::uint8_t((node.id + face) % 5)});
	} else if (node.tags.get("advertising") == "poster_box") {
		editor.place_decal_panel(node.x + 1, ground + 3, node.z, 2,
				decals::ColumnPosterKey{std::uint8_t(node.id % 5)}, true);
		editor.place_decal_panel(node.x, ground + 3, node.z, 2,
				decals::ColumnPosterKey{std::uint8_t((node.id + 2) % 5)}, true);
	}
}

void place_way_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		decals::SignRegion region)
{
	if (!editor.signage_enabled() || way.nodes.empty())
		return;
	const auto signs = highway_way_signs(way.tags, region);
	const auto &node = way.nodes.front();
	for (const auto *key : {&signs.speed, &signs.shield, &signs.cycleway})
		if (*key && place_post(editor, node.x, node.z, **key))
			break;
	if (const auto key = power_sign(way.tags))
		place_post(editor, node.x, node.z, *key, true);
	if (way.tags.get("amenity") == "parking")
		place_post(editor, node.x, node.z, decals::PictogramKey{"parking"}, true);
}

// Building and POI signage implementation
namespace
{

std::optional<std::string> get_name(const tags_t &tags)
{
	auto it = tags.find("name");
	if (it != tags.end() && !it->second.empty())
		return it->second;
	return std::nullopt;
}

std::optional<NameSign> make_name_sign(
		const tags_t &tags, decals::TextStyle style, std::uint8_t cols)
{
	auto raw = get_name(tags);
	if (!raw)
		return std::nullopt;
	const auto name = clean_name(*raw);
	if (name.empty())
		return std::nullopt;
	NameSign result;
	result.text = name;
	if (decals::font::supports(name))
		result.key = decals::DecalKey::text(style, name, cols);
	return result;
}

bool place_name_sign(WorldEditor &editor, int x, int y, int z, std::int8_t facing,
		const NameSign &name, bool require_hosts = true)
{
	if (name.key) {
		const auto [cols, rows] = name.key->dims();
		(void)rows;
		const auto [left_x, left_z] =
				WorldEditor::panel_left_anchor(x, z, facing, static_cast<int>(cols));
		return editor.place_decal_panel(
				left_x, y, left_z, facing, *name.key, false, require_hosts);
	}
	// place_text_sign writes the wall-sign node itself, while decal coordinates
	// name the backing block.  Offset to the exterior cell like Rust's
	// place_wall_sign so the sign does not collide with its host wall.
	switch (facing) {
	case 2:
		--z;
		break;
	case 3:
		++z;
		break;
	case 4:
		--x;
		break;
	case 5:
		++x;
		break;
	default:
		break;
	}
	return editor.place_text_sign(x, y, z, facing, name.text, true);
}

std::optional<std::string> get_house_number(const tags_t &tags)
{
	auto it = tags.find("addr:housenumber");
	if (it != tags.end()) {
		auto value = trim_ascii(it->second);
		if (!value.empty())
			return value;
	}
	return std::nullopt;
}

} // anonymous namespace

std::optional<NameSign> poi_name(const tags_t &tags, SignageLevel level)
{
	if (level != SignageLevel::Full)
		return std::nullopt;
	const auto amenity = tags.get("amenity");
	static const std::set<std::string> furniture{"recycling", "waste_basket",
			"waste_disposal", "vending_machine", "atm", "bench", "drinking_water",
			"fountain", "bicycle_parking", "shelter", "post_box", "parking_space"};
	if (furniture.contains(amenity) || tags.get("tourism") == "information" ||
			!decals::pictograms::business_kind(tags))
		return std::nullopt;
	auto raw = get_name(tags);
	if (!raw)
		return std::nullopt;
	const auto name = clean_name(*raw);
	return make_name_sign(tags, {decals::TextStyleKind::Fascia}, fascia_cols(name));
}

std::optional<decals::DecalKey> house_number(const tags_t &tags, SignageLevel level)
{
	if (level != SignageLevel::Full)
		return std::nullopt;
	auto num = get_house_number(tags);
	if (!num || utf8_char_count(*num) > 8 || !decals::font::supports(*num))
		return std::nullopt;
	return decals::DecalKey::text({decals::TextStyleKind::HouseNumber}, *num, 1);
}

void generate_building_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const std::optional<building_facade::FacadeAnchor> &anchor)
{
	if (!editor.signage_enabled() || !editor.signage_context || !anchor ||
			way.tags.contains("building:part") || !editor.owns(anchor->x, anchor->z))
		return;
	const auto level = editor.signage_context->level;
	const auto facing =
			WorldEditor::facing_for_normal(anchor->normal.first, anchor->normal.second);
	if (const auto name = poi_name(way.tags, level)) {
		if (!name->key || editor.signage_context->has(*name->key))
			for (int dy = 0; dy < 2; ++dy)
				if (place_name_sign(editor, anchor->x, anchor->fascia_y + dy, anchor->z,
							facing, *name)) {
					editor.signage_context->note("shop name plates", anchor->x,
							anchor->fascia_y + dy, anchor->z);
					break;
				}
	}
	if (const auto number = house_number(way.tags, level);
			number && editor.signage_context->has(*number)) {
		const auto &[rx, rz] = right_dir(facing);
		const auto first =
				anchor->door
						? std::pair{anchor->door->first + rx, anchor->door->second + rz}
						: std::pair{anchor->x - rx * 2, anchor->z - rz * 2};
		for (const auto &[x, z] :
				{first, std::pair{first.first - rx * 2, first.second - rz * 2}})
			if (editor.place_decal_panel(
						x, anchor->number_y, z, facing, *number, false, true)) {
				editor.signage_context->note("house numbers", x, anchor->number_y, z);
				break;
			}
	}
}

std::optional<std::tuple<int, int, std::int8_t>> nearest_wall_from_inside(
		const WorldEditor &editor, int x, int z, int sign_y,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	std::optional<std::pair<int, std::tuple<int, int, std::int8_t>>> best;
	for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
				 std::pair{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
		bool left_footprint = false;
		for (int k = 1; k <= 24; ++k) {
			const int cx = x + dx * k, cz = z + dz * k;
			if (!left_footprint) {
				if (footprints.contains(cx, cz))
					continue;
				left_footprint = true;
			}
			if (!editor.cell_open_at(cx, sign_y, cz))
				continue;
			const int hx = cx - dx, hz = cz - dz;
			if (editor.cell_open_at(hx, sign_y, hz))
				break;
			int score = k;
			bool road_near = false;
			for (int j = 1; j <= 8; ++j)
				road_near |= road_mask.contains(cx + dx * j, cz + dz * j);
			if (!road_near)
				score += 6;
			const auto facing = WorldEditor::facing_for_normal(dx, dz);
			if (!best || score < best->first)
				best = {score, std::tuple{hx, hz, facing}};
			break;
		}
	}
	return best ? std::optional{best->second} : std::nullopt;
}

void generate_node_facade_signage(world_editor::WorldEditor &editor,
		const ProcessedNode &node, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled() || !editor.signage_context ||
			!editor.owns(node.x, node.z))
		return;
	const auto &ctx = *editor.signage_context;
	const auto &tags = node.tags;
	if (footprints.contains(node.x, node.z) && tags.contains("addr:housenumber")) {
		const int row = editor.get_absolute_y(node.x, 0, node.z) + 3;
		if (const auto wall = nearest_wall_from_inside(
					editor, node.x, node.z, row, footprints, road_mask)) {
			const auto &[hx, hz, facing] = *wall;
			if (const auto number = house_number(tags, ctx.level);
					number && ctx.has(*number)) {
				const int number_y = editor.get_absolute_y(hx, 0, hz) + 2;
				if (editor.place_decal_panel(
							hx, number_y, hz, facing, *number, false, true))
					ctx.note("house numbers", hx, number_y, hz);
			}
		}
	}
}

void generate_node_signage(world_editor::WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled() || !editor.signage_context ||
			!editor.owns(node.x, node.z))
		return;
	const auto &ctx = *editor.signage_context;
	const auto &tags = node.tags;
	auto place_rust_traffic_post = [&](int x, int z) -> std::optional<int> {
		const int ground = editor.get_absolute_y(x, 0, z);
		for (int dy = 1; dy <= 3; ++dy) {
			const int y = ground + dy;
			const auto block = editor.get_block_absolute(x, y, z);
			if (editor.cell_has_frame(x, y, z) ||
					(block && *block != block_definitions::AIR))
				return std::nullopt;
		}
		for (int dy = 1; dy <= 2; ++dy)
			editor.set_block_absolute(
					block_definitions::STONE_BRICK_WALL, x, ground + dy, z);
		const int head = ground + 3;
		editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z);
		const auto placed_head = editor.get_block_absolute(x, head, z);
		if (!placed_head || *placed_head == block_definitions::AIR)
			return std::nullopt;
		return head;
	};
	auto kerb_for_heading = [&](double dx,
									double dz) -> std::optional<std::pair<int, int>> {
		const double side_x = decals::drives_on_left(ctx.region) ? dz : -dz;
		const double side_z = decals::drives_on_left(ctx.region) ? -dx : dx;
		int edge = 0;
		for (int k = 1; k <= 12; ++k) {
			const int cx = node.x + static_cast<int>(std::round(side_x * k));
			const int cz = node.z + static_cast<int>(std::round(side_z * k));
			if (!ctx.carriageway.contains(cx, cz)) {
				edge = k;
				break;
			}
		}
		if (!edge)
			return std::nullopt;
		for (int k = edge; k < edge + 4; ++k) {
			const int cx = node.x + static_cast<int>(std::round(side_x * k));
			const int cz = node.z + static_cast<int>(std::round(side_z * k));
			if (ctx.carriageway.contains(cx, cz) || footprints.contains(cx, cz) ||
					editor.is_lc_water(cx, cz))
				continue;
			const int ground = editor.get_absolute_y(cx, 0, cz);
			bool open = true;
			for (int dy = 1; dy <= 3; ++dy)
				if (!editor.cell_open_at(cx, ground + dy, cz) ||
						editor.cell_has_frame(cx, ground + dy, cz)) {
					open = false;
					break;
				}
			if (open)
				return std::pair{cx, cz};
		}
		return std::nullopt;
	};
	// Rust resolves POI facade signage in the node dispatcher, before processing
	// the node's feature. Keep that ordering: whether a wall already exists is
	// part of the generation result.
	const bool poi = tags.contains("shop") || tags.contains("amenity") ||
					 tags.contains("office") || tags.contains("tourism") ||
					 tags.contains("leisure") || tags.contains("healthcare") ||
					 tags.contains("craft");
	if (poi && footprints.contains(node.x, node.z)) {
		if (const auto name = poi_name(tags, ctx.level);
				name && (!name->key || ctx.has(*name->key))) {
			const int ground = editor.get_absolute_y(node.x, 0, node.z);
			if (const auto wall = nearest_wall_from_inside(
						editor, node.x, node.z, ground + 3, footprints, road_mask)) {
				const auto &[x, z, facing] = *wall;
				const int row = editor.get_absolute_y(x, 0, z) + 3;
				for (int dy = 0; dy < 2; ++dy)
					if (place_name_sign(editor, x, row + dy, z, facing, *name)) {
						ctx.note("shop name plates", x, row + dy, z);
						break;
					}
			}
		}
	}

	auto generate_highway_node_sign = [&] {
		if (const auto key = highway_node_sign(tags, ctx.level); key && ctx.has(*key)) {
			const auto heading_it = ctx.node_headings.find(node.id);
			const NodeSignHeading *heading =
					heading_it == ctx.node_headings.end() ? nullptr : &heading_it->second;
			const auto kerb =
					heading ? kerb_for_heading(heading->dir.first, heading->dir.second)
							: std::nullopt;
			int sx = node.x, sz = node.z;
			if (kerb) {
				std::tie(sx, sz) = *kerb;
			} else if (ctx.carriageway.contains(sx, sz)) {
				const auto shoulder = element_processing::get_nearest_non_road_block(
						node.x, node.z, 10, ctx.carriageway);
				if (!shoulder)
					return;
				std::tie(sx, sz) = *shoulder;
			}
			if (editor.is_lc_water(sx, sz))
				return;
			std::vector<std::int8_t> facings;
			if (heading) {
				const auto facing =
						facing_for_dir(-heading->dir.first, -heading->dir.second);
				facings.push_back(facing);
				if (heading->both_ways)
					facings.push_back(opposite(facing));
			} else if (std::pair{sx, sz} != std::pair{node.x, node.z}) {
				facings.push_back(facing_for_dir(node.x - sx, node.z - sz));
			} else if (const auto road = get_nearest_road_block(sx, sz, 8, road_mask)) {
				facings.push_back(facing_for_dir(road->first - sx, road->second - sz));
			} else {
				facings.push_back(3);
			}
			const auto head = place_rust_traffic_post(sx, sz);
			if (!head)
				return;
			bool placed = false;
			for (const auto facing : facings)
				placed |= editor.place_decal(sx, *head, sz, facing, *key);
			if (placed)
				ctx.note("stop/give-way/crossing signs", sx, *head, sz);
		}
	};
	generate_highway_node_sign();

	const auto railway = tags.get("railway");
	if (railway == "level_crossing" || railway.empty())
		return;
	const bool subway = (railway == "station" || railway == "halt") &&
						(tags.get("station") == "subway" || tags.get("subway") == "yes");
	if (railway == "station" || railway == "halt") {
		const decals::DecalKey icon =
				decals::PictogramKey{subway ? decals::metro_logo(ctx.region) : "train"};
		if (!ctx.has(icon))
			return;
		if (subway) {
			if (const auto head = place_rust_traffic_post(node.x, node.z)) {
				for (const std::int8_t facing :
						{std::int8_t(2), std::int8_t(3), std::int8_t(4), std::int8_t(5)})
					editor.place_decal_panel(
							node.x, *head, node.z, facing, icon, true, false);
				ctx.note("metro entrance signs", node.x, *head, node.z);
			}
			return;
		}
		const int ground = editor.get_absolute_y(node.x, 0, node.z), beam = ground + 3;
		bool clear = true;
		for (const int dx : {-1, 1})
			for (int dy = 1; dy <= 3; ++dy) {
				const int x = node.x + dx, y = ground + dy;
				const auto block = editor.get_block_absolute(x, y, node.z);
				clear &=
						!editor.cell_has_frame(x, y, node.z) && (!block || *block == AIR);
			}
		const auto beam_block = editor.get_block_absolute(node.x, beam, node.z);
		clear &= !editor.cell_has_frame(node.x, beam, node.z) &&
				 (!beam_block || *beam_block == AIR);
		if (!clear)
			return;
		for (const int dx : {-1, 1})
			for (int dy = 1; dy <= 2; ++dy)
				editor.set_block_absolute(block_definitions::STONE_BRICK_WALL,
						node.x + dx, ground + dy, node.z);
		for (int dx = -1; dx <= 1; ++dx)
			editor.set_block_absolute(
					block_definitions::LIGHT_GRAY_CONCRETE, node.x + dx, beam, node.z);
		const auto name = make_name_sign(tags, {decals::TextStyleKind::StationBoard}, 3);
		bool named = false;
		if (name && (!name->key || ctx.has(*name->key)))
			for (const std::int8_t facing : {std::int8_t(2), std::int8_t(3)})
				named |= place_name_sign(
						editor, node.x, beam, node.z, facing, *name, true);
		if (!named)
			for (const std::int8_t facing :
					{std::int8_t(2), std::int8_t(3), std::int8_t(4), std::int8_t(5)})
				editor.place_decal(node.x, beam, node.z, facing, icon);
		else {
			editor.place_decal(node.x - 1, beam, node.z, 4, icon);
			editor.place_decal(node.x + 1, beam, node.z, 5, icon);
		}
		ctx.note("station boards", node.x, beam, node.z);
		return;
	}
	if (railway == "tram_stop" || railway == "subway_entrance") {
		const decals::DecalKey icon = decals::PictogramKey{
				railway == "tram_stop" ? "tram" : decals::metro_logo(ctx.region)};
		if (!ctx.has(icon) || ctx.carriageway.contains(node.x, node.z))
			return;
		const auto head = place_rust_traffic_post(node.x, node.z);
		if (!head)
			return;
		const bool glow = railway == "subway_entrance";
		for (const std::int8_t facing :
				{std::int8_t(2), std::int8_t(3), std::int8_t(4), std::int8_t(5)})
			editor.place_decal_panel(node.x, *head, node.z, facing, icon, glow, false);
		ctx.note(
				glow ? "metro entrance signs" : "tram stop signs", node.x, *head, node.z);
		if (railway == "tram_stop") {
			const auto name = make_name_sign(tags, {decals::TextStyleKind::StopName}, 1);
			if (name && name->key && ctx.has(*name->key) &&
					name->key->dims().first == 1) {
				editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, node.x,
						*head + 1, node.z);
				const auto road = get_nearest_road_block(node.x, node.z, 8, road_mask);
				const auto facing =
						road ? facing_for_dir(road->first - node.x, road->second - node.z)
							 : std::int8_t(3);
				editor.place_decal(node.x, *head + 1, node.z, facing, *name->key);
				editor.place_decal(
						node.x, *head + 1, node.z, opposite(facing), *name->key);
			}
		}
	}
}

void generate_power_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return;

	auto sign = power_sign(way.tags);
	if (!sign || !editor.signage_context->has(*sign))
		return;

	if (way.nodes.empty())
		return;

	const auto &node = way.nodes.front();
	int x = node.x, z = node.z;
	if (editor.signage_context->carriageway.contains(x, z) || !editor.owns(x, z))
		return;
	const int ground = editor.get_absolute_y(x, 0, z);
	for (int dy = 1; dy <= 3; ++dy) {
		if (editor.cell_has_frame(x, ground + dy, z))
			return;
		const auto block = editor.get_block_absolute(x, ground + dy, z);
		if (block && *block != AIR)
			return;
	}
	editor.set_block_absolute(STONE_BRICK_WALL, x, ground + 1, z);
	editor.set_block_absolute(STONE_BRICK_WALL, x, ground + 2, z);
	const int head = ground + 3;
	editor.set_block_absolute(LIGHT_GRAY_CONCRETE, x, head, z);
	std::int8_t facing = 3;
	if (const auto road = get_nearest_road_block(x, z, 12, road_mask))
		facing = facing_for_dir(road->first - x, road->second - z);
	if (editor.place_decal(x, head, z, facing, *sign))
		editor.signage_context->note("high-voltage signs", x, head, z);
	editor.place_decal(x, head, z, opposite(facing), *sign);
}

static void place_street_name_post(world_editor::WorldEditor &editor,
		const ProcessedNode &node, const std::pair<double, double> &own_direction,
		const IntersectionPost &post, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &road_mask)
{
	if (!editor.decal_registry || !editor.signage_context)
		return;
	int widest = 1, width_x = 0, width_z = 0;
	for (const auto &blade : post.blades) {
		widest = std::max(widest, blade.half_width);
		if (std::abs(blade.direction.second) > std::abs(blade.direction.first))
			width_x = std::max(width_x, blade.half_width);
		else
			width_z = std::max(width_z, blade.half_width);
	}
	if (!width_x)
		width_x = widest;
	if (!width_z)
		width_z = widest;
	auto cross = std::pair{-own_direction.second, own_direction.first};
	for (const auto &blade : post.blades) {
		const double dot = blade.direction.first * own_direction.first +
						   blade.direction.second * own_direction.second;
		if (std::abs(dot) < 0.9) {
			cross = blade.direction;
			break;
		}
	}
	const int qx = -(own_direction.first + cross.first) >= 0.0 ? 1 : -1;
	const int qz = -(own_direction.second + cross.second) >= 0.0 ? 1 : -1;
	std::vector<std::pair<int, int>> offsets;
	for (int dx = 1; dx <= 9; ++dx)
		for (int dz = 1; dz <= 9; ++dz)
			offsets.emplace_back(dx, dz);
	std::sort(offsets.begin(), offsets.end(), [](const auto &a, const auto &b) {
		return a.first * a.first + a.second * a.second <
			   b.first * b.first + b.second * b.second;
	});
	std::optional<std::pair<int, int>> position;
	for (const auto &[sx, sz] : std::array<std::pair<int, int>, 4>{
				 {{qx, qz}, {qx, -qz}, {-qx, qz}, {-qx, -qz}}}) {
		for (const auto &[dx, dz] : offsets) {
			const int x = node.x + sx * (width_x + dx);
			const int z = node.z + sz * (width_z + dz);
			if (!editor.owns(x, z) || road_mask.contains(x, z) ||
					footprints.contains(x, z) || editor.is_lc_water(x, z))
				continue;
			const int ground = editor.get_absolute_y(x, 0, z);
			bool clear = true;
			for (int dy = 1; dy <= static_cast<int>(post.blades.size()) + 2; ++dy)
				clear &= editor.cell_open_at(x, ground + dy, z) &&
						 !editor.cell_has_frame(x, ground + dy, z);
			if (clear) {
				position = {x, z};
				break;
			}
		}
		if (position)
			break;
	}
	if (!position)
		return;
	const auto &[x, z] = *position;
	const int ground = editor.get_absolute_y(x, 0, z);
	// street_signs:sign_basic is the native two-up intersection post described by
	// the mod itself. Use it for the common two-street case and retain the decal
	// stack below for three-way names or games without street_signs.
	if (block_definitions::STREET_SIGNS_AVAILABLE && post.blades.size() <= 2) {
		std::string text;
		for (const auto &blade : post.blades) {
			if (!text.empty())
				text.push_back('\n');
			text += blade.name;
		}
		const std::int8_t facedir =
				std::abs(own_direction.first) >= std::abs(own_direction.second) ? 0 : 1;
		if (editor.place_sign_node(block_definitions::STREET_SIGN_BASIC, x, ground + 1, z,
					facedir, text)) {
			editor.signage_context->note("street name posts", x, ground + 1, z);
			return;
		}
	}
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 1, z,
			std::nullopt, std::nullopt);
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 2, z,
			std::nullopt, std::nullopt);
	const decals::TextStyle style{decals::TextStyleKind::StreetName,
			decals::blade_style(editor.signage_context->region)};
	int row = 0;
	for (const auto &blade : post.blades) {
		const auto key = decals::DecalKey::text(style, blade.name, 1);
		if (!editor.decal_registry->contains(key))
			continue;
		const int y = ground + 3 + row++;
		editor.set_block_absolute(block_definitions::POLISHED_ANDESITE, x, y, z,
				std::nullopt, std::nullopt);
		const bool along_x =
				std::abs(blade.direction.first) >= std::abs(blade.direction.second);
		const std::int8_t first = along_x ? 2 : 4;
		editor.place_decal_panel(x, y, z, first, key, false, false);
		editor.place_decal_panel(x, y, z, opposite(first), key, false, false);
	}
	if (row > 0)
		editor.signage_context->note("street name posts", x, ground + 3, z);
}

void generate_highway_way_signage(world_editor::WorldEditor &editor,
		const ProcessedWay &way, const BuildingFootprintBitmap &footprints)
{
	// Highway signage: speed limit signs, route shields, cycleway signs, no-entry and periodic signs
	if (!editor.signage_enabled() || !editor.signage_context || way.nodes.size() < 2)
		return;
	const auto &ctx = *editor.signage_context;
	const auto &road_mask = ctx.carriageway;

	// Skip elevated/deck/tunnel/layer!=0 highways
	const auto bridge = way.tags.find("bridge");
	const auto tunnel = way.tags.find("tunnel");
	const auto layer = way.tags.find("layer");
	const bool elevated = (bridge != way.tags.end() && bridge->second != "no") ||
						  (tunnel != way.tags.end() && tunnel->second != "no") ||
						  (layer != way.tags.end() &&
								  strict_parse::i32(layer->second).value_or(0) != 0);
	if (elevated)
		return;

	// Skip area highways
	if (way.tags.get("area") == "yes")
		return;

	const std::string highway = way.tags.get("highway");
	const bool oneway = way.tags.get("oneway") == "yes" || way.tags.get("oneway") == "-1";
	const bool reversed = way.tags.get("oneway") == "-1";
	const int half_width =
			arnis_highways::highway_block_range(highway, way.tags, ctx.scale);

	// A single owner way places each shared-node post, preventing duplicate
	// blades when OSM splits the same intersection into several ways.
	{
		for (std::size_t i = 0; i < way.nodes.size(); ++i) {
			const auto found = ctx.intersections.find(way.nodes[i].id);
			if (found == ctx.intersections.end() || found->second.owner_way != way.id)
				continue;
			if (!editor.owns(way.nodes[i].x, way.nodes[i].z))
				continue;
			if (auto direction = way_direction_at(way.nodes, i))
				place_street_name_post(editor, way.nodes[i], *direction, found->second,
						footprints, road_mask);
		}
	}

	// Build Bresenham line along the way
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &from = way.nodes[i - 1];
		const auto &to = way.nodes[i];
		const auto line = bresenham::bresenham_line(from.x, 0, from.z, to.x, 0, to.z);
		for (const auto &pt : line)
			cells.emplace_back(std::get<0>(pt), std::get<2>(pt));
	}

	const auto signs = highway_way_signs(way.tags, ctx.region);
	if (!signs.speed.has_value() && !signs.shield.has_value() &&
			!signs.no_entry.has_value() && !signs.cycleway.has_value())
		return;
	if (cells.size() < 24)
		return;

	// Place cycleway signs (if cycleway, place just these and return)
	if (signs.cycleway.has_value()) {
		place_roadside_sign(editor, "cycleway signs", cells, 6, half_width,
				signs.cycleway.value(), false, footprints);
		return;
	}

	// Place speed limit signs at start and periodically (every 8 cells for short gaps)
	if (signs.speed.has_value()) {
		{
			place_roadside_sign(editor, "speed limits", cells, 8, half_width,
					signs.speed.value(), reversed, footprints);
			if (!oneway && cells.size() >= 60) {
				place_roadside_sign(editor, "speed limits", cells, cells.size() - 9,
						half_width, signs.speed.value(), true, footprints);
			}
		}
	}

	// No-entry where wrong-way traffic would enter a one-way street
	if (signs.no_entry.has_value()) {
		if (reversed) {
			place_roadside_sign(editor, "no-entry signs", cells, 4, half_width,
					signs.no_entry.value(), false, footprints);
		} else if (oneway) {
			place_roadside_sign(editor, "no-entry signs", cells, cells.size() - 5,
					half_width, signs.no_entry.value(), true, footprints);
		}
	}

	// Reassurance shields: thinned so a road split into many OSM ways is not carpeted
	if (signs.shield.has_value() && cells.size() >= 40) {
		if (cells.size() >= 120 || way.id % 3 == 0)
			place_roadside_sign(editor, "route shields", cells,
					std::min<std::size_t>(40, cells.size() / 2), half_width,
					signs.shield.value(), false, footprints);

		std::size_t pos = 250;
		while (pos + 30 < cells.size()) {
			place_roadside_sign(editor, "route shields", cells, pos, half_width,
					signs.shield.value(), false, footprints);
			pos += 250;
		}
	}

	(void)way.tags;
	(void)footprints;
}

void generate_parking_signage(world_editor::WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &road_mask)
{
	// Parking lot: a "P" post at the first outline node, off the road.
	if (!editor.signage_enabled())
		return;

	const auto key = decals::PictogramKey{"parking"};
	if (!editor.decal_registry || !editor.decal_registry->contains(key))
		return;

	if (way.nodes.size() < 3)
		return;

	const auto &node = way.nodes.front();
	const int x = node.x;
	const int z = node.z;

	// Don't place on carriageway
	if (road_mask.contains(x, z))
		return;

	if (!editor.owns(x, z))
		return;

	const int ground = editor.get_absolute_y(x, 0, z);
	for (int dy = 1; dy <= 3; ++dy) {
		if (editor.cell_has_frame(x, ground + dy, z))
			return;
		const auto block = editor.get_block_absolute(x, ground + dy, z);
		if (block && *block != AIR)
			return;
	}
	const int head = ground + 3;
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 1, z);
	editor.set_block_absolute(block_definitions::STONE_BRICK_WALL, x, ground + 2, z);
	editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE, x, head, z);
	const auto placed_head = editor.get_block_absolute(x, head, z);
	if (!placed_head || *placed_head == block_definitions::AIR)
		return;

	std::int8_t facing = 3; // default south
	if (const auto road = get_nearest_road_block(x, z, 10, road_mask))
		facing = facing_for_dir(road->first - x, road->second - z);
	const bool placed = editor.place_decal(x, head, z, facing, key);
	editor.place_decal(x, head, z, opposite(facing), key);
	if (placed && editor.signage_context)
		editor.signage_context->note("parking signs", x, head, z);
}

bool generate_billboard(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return false;

	decals::DecalKey key = decals::PosterKey{static_cast<std::uint8_t>(node.id % 6)};
	if (!editor.decal_registry->contains(key))
		return false;

	int x = node.x;
	int z = node.z;

	// Face the nearest road; the panel then runs perpendicular to that direction.
	std::int8_t facing = 3;
	auto road_pos = get_nearest_road_block(x, z, 20, road_mask);
	if (road_pos) {
		facing = facing_for_dir(road_pos->first - x, road_pos->second - z);
	}

	auto [rx, rz] = right_dir(facing);
	int ground = editor.get_absolute_y(x, 0, z);
	int height = 6;
	auto it_height = node.tags.find("height");
	if (it_height != node.tags.end()) {
		std::string h_str = it_height->second;
		while (!h_str.empty() && h_str.back() == 'm')
			h_str.pop_back();
		const auto first = h_str.find_first_not_of(" \t\r\n\f\v");
		const auto last = h_str.find_last_not_of(" \t\r\n\f\v");
		if (first == std::string::npos)
			h_str.clear();
		else
			h_str = h_str.substr(first, last - first + 1);
		try {
			std::size_t parsed = 0;
			const double h_val = std::stod(h_str, &parsed);
			if (parsed == h_str.size()) {
				const double rounded = std::round(h_val);
				height = std::isnan(rounded) ? 0
						 : rounded <= 4.0	 ? 4
						 : rounded >= 12.0	 ? 12
											 : static_cast<int>(rounded);
			}
		} catch (...) {
			height = 6;
		}
	}
	height = std::clamp(height, 4, 12);
	int top = ground + height;

	for (int k : {-1, 1}) {
		for (int dy = 1; dy <= height - 2; ++dy) {
			editor.set_block_absolute(STONE_BRICK_WALL, x + rx * k, ground + dy,
					z + rz * k, std::nullopt, std::nullopt);
		}
	}
	for (int k = -1; k <= 1; ++k) {
		for (int y = top - 1; y <= top; ++y) {
			editor.set_block_absolute(BLACK_CONCRETE, x + rx * k, y, z + rz * k,
					std::nullopt, std::nullopt);
		}
	}

	auto [lx, lz] = WorldEditor::panel_left_anchor(x, z, facing, 3);
	const bool placed = editor.place_decal_panel(lx, top, lz, facing, key, false, false);

	std::int8_t back = opposite(facing);
	auto [bx, bz] = WorldEditor::panel_left_anchor(x, z, back, 3);
	editor.place_decal_panel(bx, top, bz, back, key, false, false);
	if (placed && editor.signage_context)
		editor.signage_context->note("billboards", x, top, z);
	return true;
}

bool generate_column(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled())
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_absolute_y(x, 0, z);
	for (int dy = 1; dy <= 3; ++dy) {
		editor.set_block_absolute(
				GRAY_CONCRETE, x, ground + dy, z, std::nullopt, std::nullopt);
	}
	editor.set_block_absolute(
			STONE_BRICK_SLAB, x, ground + 4, z, std::nullopt, std::nullopt);

	bool any = false;
	for (int f = 0; f < 4; ++f) {
		std::int8_t facing = static_cast<std::int8_t>(f + 2);
		decals::DecalKey key =
				decals::ColumnPosterKey{static_cast<std::uint8_t>((node.id + f) % 5)};
		if (editor.decal_registry->contains(key)) {
			any |= editor.place_decal_panel(x, ground + 3, z, facing, key, false, false);
		}
	}
	if (any && editor.signage_context)
		editor.signage_context->note("advertising columns", x, ground + 3, z);
	return any;
}

bool generate_poster_box_posters(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled())
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_absolute_y(x, 0, z);

	decals::DecalKey a = decals::ColumnPosterKey{static_cast<std::uint8_t>(node.id % 5)};
	decals::DecalKey b =
			decals::ColumnPosterKey{static_cast<std::uint8_t>((node.id + 2) % 5)};

	if (!editor.decal_registry->contains(a) || !editor.decal_registry->contains(b))
		return false;

	// North face: viewer's left is +x.
	editor.place_decal_panel(x + 1, ground + 3, z, 2, a, true, false);
	editor.place_decal_panel(x, ground + 3, z, 2, b, true, false);
	editor.place_decal_panel(x, ground + 3, z, 3, a, true, false);
	editor.place_decal_panel(x + 1, ground + 3, z, 3, b, true, false);
	return true;
}

bool generate_information_board(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask)
{
	if (!editor.signage_enabled())
		return false;

	// Get information decal key based on node tags
	std::optional<decals::DecalKey> key;
	auto it_tourism = node.tags.find("tourism");
	if (it_tourism == node.tags.end() || it_tourism->second != "information")
		return false;

	auto it_info = node.tags.find("information");
	if (it_info != node.tags.end()) {
		if (it_info->second == "map" || it_info->second == "board" ||
				it_info->second == "terminal") {
			key = decals::DecalKey(decals::LocalMapKey{node.x, node.z});
		} else {
			key = decals::DecalKey(decals::PictogramKey{"information"});
		}
	}
	if (!key.has_value())
		return false;
	if (!editor.decal_registry->contains(key.value()))
		return false;

	int x = node.x;
	int z = node.z;
	int ground = editor.get_absolute_y(x, 0, z);

	// Face nearest road
	std::int8_t facing = 3;
	auto road_pos = get_nearest_road_block(x, z, 10, road_mask);
	if (road_pos) {
		facing = facing_for_dir(road_pos->first - x, road_pos->second - z);
	}

	auto [rx, rz] = right_dir(facing);

	bool is_local_map = std::holds_alternative<decals::LocalMapKey>(key.value());
	if (is_local_map) {
		// Board back: 2 wide x 2 tall dark oak panel on two short legs
		auto [lx, lz] = WorldEditor::panel_left_anchor(x, z, facing, 2);
		for (int c = 0; c < 2; ++c) {
			int bx = lx + rx * c;
			int bz = lz + rz * c;
			editor.set_block_absolute(
					OAK_FENCE, bx, ground + 1, bz, std::nullopt, std::nullopt);
			for (int y = ground + 2; y <= ground + 3; ++y) {
				editor.set_block_absolute(
						DARK_OAK_PLANKS, bx, y, bz, std::nullopt, std::nullopt);
			}
		}
		bool ok = editor.place_decal_panel(
				lx, ground + 3, lz, facing, key.value(), false, false);
		if (ok && editor.signage_context)
			editor.signage_context->note("you-are-here boards", x, ground + 3, z);
		return ok;
	} else {
		// "i" post
		int head = ground + 3;
		editor.set_block_absolute(
				STONE_BRICK_WALL, x, ground + 1, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(
				STONE_BRICK_WALL, x, ground + 2, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(
				LIGHT_GRAY_CONCRETE, x, head, z, std::nullopt, std::nullopt);

		bool any = false;
		for (std::int8_t f : {2, 3, 4, 5}) {
			any |= editor.place_decal_panel(x, head, z, f, key.value(), false, false);
		}
		if (any && editor.signage_context)
			editor.signage_context->note("information posts", x, head, z);
		return any;
	}
}

std::optional<decals::DecalKey> plaque_key(const tags_t &tags)
{
	if (tags.get("historic") != "memorial" || tags.get("memorial") != "plaque")
		return std::nullopt;
	const auto inscription = tags.find("inscription");
	const auto name = tags.find("name");
	if (inscription == tags.end() && name == tags.end())
		return std::nullopt;
	const auto text =
			clean_name(inscription != tags.end() ? inscription->second : name->second);
	if (text.empty() || !decals::font::supports(text))
		return std::nullopt;
	return decals::DecalKey::text({decals::TextStyleKind::Plaque}, text, 1);
}

bool generate_plaque(WorldEditor &editor, const ProcessedNode &node)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return false;
	const auto key = plaque_key(node.tags);
	if (!key || !editor.signage_context->has(*key))
		return false;
	const int x = node.x, z = node.z;
	const int y = editor.get_absolute_y(x, 0, z) + 2;
	for (const auto &[dx, dz] : std::array<std::pair<int, int>, 4>{
				 std::pair{0, -1}, {0, 1}, {-1, 0}, {1, 0}}) {
		if (!editor.cell_open_at(x + dx, y, z + dz) && editor.cell_open_at(x, y, z)) {
			const auto facing = WorldEditor::facing_for_normal(-dx, -dz);
			const bool placed = editor.place_decal(x + dx, y, z + dz, facing, *key);
			if (placed)
				editor.signage_context->note("memorial plaques", x, y, z);
			return placed;
		}
	}
	return false;
}

void place_bus_stop_signs(
		WorldEditor &editor, const tags_t &tags, int x, int sign_y, int z)
{
	if (!editor.signage_enabled() || !editor.signage_context)
		return;
	const auto &ctx = *editor.signage_context;
	const decals::DecalKey icon = decals::PictogramKey{"bus_stop"};
	if (ctx.has(icon)) {
		editor.place_decal(x + 1, sign_y, z, 2, icon);
		editor.place_decal(x + 1, sign_y, z, 3, icon);
		ctx.note("bus stops", x + 1, sign_y, z);
	}
	const auto name = make_name_sign(tags, {decals::TextStyleKind::StopName}, 1);
	if (name && (!name->key || ctx.has(*name->key)))
		for (const std::int8_t facing : {std::int8_t(2), std::int8_t(3)})
			place_name_sign(editor, x, sign_y, z, facing, *name, true);
}

} // namespace arnis::signage
