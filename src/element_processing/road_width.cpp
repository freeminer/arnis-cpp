#include "road_width.h"

#include "highways.h"
#include "raceways.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <string_view>

namespace arnis::road_width
{
namespace
{
constexpr double LANE_M = 3.25;
constexpr double FAST_LANE_M = 3.5;
constexpr double PARALLEL_PARKING_M = 2.25;
constexpr double ANGLED_PARKING_M = 5.0;
constexpr double CYCLE_LANE_M = 1.5;

std::string_view trim(std::string_view s)
{
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
		s.remove_prefix(1);
	while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
		s.remove_suffix(1);
	return s;
}

std::optional<double> number(std::string_view s)
{
	s = trim(s);
	if (s.empty())
		return std::nullopt;
	std::string value(s);
	char *end = nullptr;
	const double result = std::strtod(value.c_str(), &end);
	if (end != value.c_str() + value.size() || !std::isfinite(result))
		return std::nullopt;
	return result;
}

std::optional<std::string_view> get(const auto &tags, const std::string &key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? std::nullopt : std::optional<std::string_view>(it->second);
}

std::optional<double> optional_metres(std::optional<std::string_view> value)
{
	return value ? metres(*value) : std::nullopt;
}

std::optional<double> tag_metres(const auto &tags, const std::string &key)
{
	return optional_metres(get(tags, key));
}

std::optional<int> tag_count(const auto &tags, const std::string &key)
{
	const auto value = get(tags, key);
	return value ? count(*value) : std::nullopt;
}

bool any_key(const auto &tags, std::string_view prefix)
{
	return std::any_of(tags.begin(), tags.end(), [&](const auto &entry) {
		return std::string_view(entry.first).starts_with(prefix);
	});
}

double half_of(double metres)
{
	return std::round((metres - 1.0) / 2.0);
}

std::pair<double, double> carriageway_m(std::string_view highway)
{
	if (highway == "motorway" || highway == "trunk")
		return {11.0, FAST_LANE_M};
	if (highway == "motorway_link" || highway == "trunk_link")
		return {5.0, FAST_LANE_M};
	if (highway == "primary")
		return {8.0, LANE_M};
	if (highway == "secondary")
		return {7.0, LANE_M};
	if (highway == "tertiary")
		return {6.5, LANE_M};
	if (highway == "secondary_link" || highway == "tertiary_link")
		return {3.0, LANE_M};
	return {5.0, LANE_M};
}

std::optional<int> fixed_half(std::string_view highway,
		const std::unordered_map<std::string, std::string> &tags)
{
	if (highway == "footway" || highway == "pedestrian" || highway == "path" ||
			highway == "cycleway" || highway == "track" || highway == "escape" ||
			highway == "steps")
		return 1;
	if (highway == "raceway")
		return raceways::default_half_width(tags);
	return std::nullopt;
}

double street_parking_share(std::string_view position)
{
	if (position == "lane" || position == "street_side" || position == "shoulder" ||
			position == "yes")
		return 1.0;
	if (position == "half_on_kerb")
		return .5;
	return 0.0;
}

double lane_parking_share(std::string_view position)
{
	if (position == "on_street" || position == "painted_area_only" ||
			position == "street_side" || position == "lay_by" || position == "shoulder")
		return 1.0;
	if (position == "half_on_kerb")
		return .5;
	return 0.0;
}

double parking_depth_m(std::optional<std::string_view> orientation)
{
	return orientation == "diagonal" || orientation == "perpendicular"
				   ? ANGLED_PARKING_M
				   : PARALLEL_PARKING_M;
}

std::pair<double, double> parking_side_m(
		const std::unordered_map<std::string, std::string> &tags, std::string_view side)
{
	if (!any_key(tags, "parking:"))
		return {0.0, 0.0};
	const auto street = [&](std::string_view suffix) {
		if (auto v = get(tags, "parking:" + std::string(side) + std::string(suffix)))
			return v;
		return get(tags, "parking:both" + std::string(suffix));
	};
	if (const auto position = street("")) {
		const double share = street_parking_share(*position);
		if (share == 0.0)
			return {0.0, 0.0};
		const double depth = optional_metres(street(":width"))
									 .value_or(parking_depth_m(street(":orientation")));
		return {share * depth, *position == "street_side" ? depth : 0.0};
	}
	const auto lane = [&](std::string_view suffix) {
		if (auto v = get(tags, "parking:lane:" + std::string(side) + std::string(suffix)))
			return v;
		return get(tags, "parking:lane:both" + std::string(suffix));
	};
	const auto raw_orientation = lane("");
	if (!raw_orientation)
		return {0.0, 0.0};
	std::string_view orientation = *raw_orientation;
	if (orientation == "marked")
		orientation = "parallel";
	else if (orientation != "parallel" && orientation != "diagonal" &&
			 orientation != "perpendicular")
		return {0.0, 0.0};
	const std::string suffix = ":" + std::string(orientation);
	const auto position = lane(suffix);
	const double depth = position ? lane_parking_share(*position) : 1.0;
	const double metres = depth * parking_depth_m(orientation);
	return {metres, position && (*position == "street_side" || *position == "lay_by")
							? metres
							: 0.0};
}

std::array<std::pair<double, double>, 2> parking_sides_m(
		const std::unordered_map<std::string, std::string> &tags)
{
	const auto bridge = get(tags, "bridge");
	const bool on_bridge = bridge && *bridge != "no";
	std::array<std::pair<double, double>, 2> result;
	for (std::size_t i = 0; i < 2; ++i) {
		const auto [parked, bays] = parking_side_m(tags, i == 0 ? "left" : "right");
		result[i] = on_bridge ? std::pair{parked - bays, 0.0} : std::pair{parked, bays};
	}
	return result;
}

bool one_way(const std::unordered_map<std::string, std::string> &tags)
{
	const auto oneway = get(tags, "oneway");
	const auto junction = get(tags, "junction");
	return (oneway && (*oneway == "yes" || *oneway == "true" || *oneway == "1" ||
							  *oneway == "-1" || *oneway == "reversible" ||
							  *oneway == "alternating")) ||
		   (junction && (*junction == "roundabout" || *junction == "circular"));
}

double cycle_lanes_m(const std::unordered_map<std::string, std::string> &tags)
{
	if (!any_key(tags, "cycleway"))
		return 0.0;
	const auto pick = [&](std::string_view side, std::string_view suffix) {
		if (auto v = get(tags, "cycleway:" + std::string(side) + std::string(suffix)))
			return v;
		if (auto v = get(tags, "cycleway:both" + std::string(suffix)))
			return v;
		return get(tags, "cycleway" + std::string(suffix));
	};
	double total = 0.0;
	for (const auto side : {"left", "right"})
		if (pick(side, "") == "lane" && pick(side, ":lane") == "exclusive")
			total += optional_metres(pick(side, ":width")).value_or(CYCLE_LANE_M);
	return total;
}

bool two_way_street(std::string_view highway,
		const std::unordered_map<std::string, std::string> &tags)
{
	if (highway != "residential" && highway != "unclassified")
		return false;
	if (tags.contains("lanes") || one_way(tags))
		return false;
	for (const auto key :
			{"sidewalk", "sidewalk:both", "sidewalk:left", "sidewalk:right"})
		if (const auto it = tags.find(key);
				it != tags.end() && it->second != "no" && it->second != "none")
			return true;
	return false;
}

int fit(int half, double scale)
{
	half = std::clamp(half, 1, MAX_HALF);
	return scale < 1.0 ? std::max(1, static_cast<int>(std::floor(half * scale))) : half;
}

std::pair<RoadHalves, std::pair<int, int>> widths_at(const std::string &requested_highway,
		const std::unordered_map<std::string, std::string> &tags, double scale,
		bool street_lanes)
{
	std::string_view highway = requested_highway;
	if (highway == "construction") {
		if (const auto construction = get(tags, "construction");
				construction && !construction->empty() && *construction != "construction")
			highway = *construction;
	}
	const auto width = tag_metres(tags, "width");
	int lanes = 1, kerb = 1;
	std::pair<int, int> sides{1, 1};
	if (const auto fixed = fixed_half(highway, tags)) {
		const int half = width ? static_cast<int>(std::lround(half_of(*width))) : *fixed;
		lanes = kerb = half;
		sides = {half, half};
	} else {
		const auto [default_m, lane_m] = carriageway_m(highway);
		const auto tagged_lanes = tag_count(tags, "lanes");
		const int lane_count =
				std::clamp(tagged_lanes.value_or(
								   highways::highway_default_lanes(std::string(highway))),
						1, 16);
		const double base_m = tagged_lanes && one_way(tags) && highway != "motorway" &&
											  highway != "trunk"
									  ? 0.0
									  : default_m;
		const auto parking = parking_sides_m(tags);
		const double left = parking[0].first, right = parking[1].first;
		const double parked = left + right;
		const double bays = parking[0].second + parking[1].second;
		const double street_m =
				street_lanes && parked == 0.0 && two_way_street(highway, tags)
						? 2.0 * lane_m
						: 0.0;
		const double driving = std::max({base_m, lane_count * lane_m, street_m});
		const double traffic = driving + cycle_lanes_m(tags);
		const auto tagged_carriageway = tag_metres(tags, "width:carriageway");
		const auto tagged = tagged_carriageway ? tagged_carriageway : width;
		const double kerb_m =
				tagged ? (parked > 0.0 ? std::max(*tagged + bays, lane_m + parked)
									   : *tagged)
					   : traffic + parked;
		const double lanes_m =
				parked > 0.0 ? std::max(kerb_m - parked, tagged ? lane_m : traffic)
							 : kerb_m;
		const int left_half =
				static_cast<int>(std::lround(half_of(lanes_m + 2.0 * left)));
		const int right_half =
				static_cast<int>(std::lround(half_of(lanes_m + 2.0 * right)));
		sides = {left_half, right_half};
		const double lane_width = street_lanes ? std::max(lanes_m - cycle_lanes_m(tags),
														 std::min(driving, lanes_m))
											   : lanes_m;
		lanes = static_cast<int>(std::lround(half_of(std::min(lane_width, kerb_m))));
		kerb = std::max(left_half, right_half);
	}
	kerb = fit(kerb, scale);
	lanes = std::min(fit(lanes, scale), kerb);
	const auto side = [&](int half) { return std::clamp(fit(half, scale), lanes, kerb); };
	return {{lanes, kerb}, {side(sides.first), side(sides.second)}};
}
} // namespace

std::optional<double> metres(std::string_view raw)
{
	raw = trim(raw);
	if (const auto semicolon = raw.find(';'); semicolon != std::string_view::npos)
		raw = trim(raw.substr(0, semicolon));
	std::string normalized(raw);
	std::replace(normalized.begin(), normalized.end(), ',', '.');
	raw = normalized;
	double value = 0.0;
	if (const auto quote = raw.find('\''); quote != std::string_view::npos) {
		const auto feet = number(raw.substr(0, quote));
		if (!feet)
			return std::nullopt;
		auto inch_text = trim(raw.substr(quote + 1));
		if (!inch_text.empty() && inch_text.back() == '"')
			inch_text.remove_suffix(1);
		const double inches = inch_text.empty() ? 0.0 : number(inch_text).value_or(-1.0);
		if (inches < 0.0)
			return std::nullopt;
		value = *feet * .3048 + inches * .0254;
	} else if (raw.ends_with("ft")) {
		const auto feet = number(trim(raw.substr(0, raw.size() - 2)));
		if (!feet)
			return std::nullopt;
		value = *feet * .3048;
	} else {
		if (raw.ends_with('m'))
			raw = trim(raw.substr(0, raw.size() - 1));
		const auto parsed = number(raw);
		if (!parsed)
			return std::nullopt;
		value = *parsed;
	}
	return std::isfinite(value) && value > 0.0 ? std::optional<double>(value)
											   : std::nullopt;
}

std::optional<int> count(std::string_view raw)
{
	if (const auto semicolon = raw.find(';'); semicolon != std::string_view::npos)
		raw = raw.substr(0, semicolon);
	raw = trim(raw);
	if (raw.empty())
		return std::nullopt;
	int result = 0;
	const auto [end, error] =
			std::from_chars(raw.data(), raw.data() + raw.size(), result);
	return error == std::errc{} && end == raw.data() + raw.size()
				   ? std::optional<int>(result)
				   : std::nullopt;
}

RoadHalves road_halves(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale)
{
	return widths_at(highway_type, tags, scale, true).first;
}

std::pair<int, int> kerb_sides(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale)
{
	return widths_at(highway_type, tags, scale, true).second;
}

int wall_half(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale)
{
	const int lanes = widths_at(highway_type, tags, scale, false).first.lanes;
	auto width = tag_metres(tags, "width:carriageway");
	if (!width)
		width = tag_metres(tags, "width");
	return width ? std::max(lanes,
						   fit(static_cast<int>(std::lround(half_of(*width))), scale))
				 : lanes;
}

bool parks_along(const std::unordered_map<std::string, std::string> &tags)
{
	const auto sides = parking_sides_m(tags);
	return sides[0].first > 0.0 || sides[1].first > 0.0;
}

std::optional<std::string_view> parking_orientation(
		const std::unordered_map<std::string, std::string> &tags, std::string_view side)
{
	if (!any_key(tags, "parking:"))
		return std::nullopt;
	const auto kind = [](std::optional<std::string_view> value) -> std::string_view {
		return value == "diagonal"		  ? "diagonal"
			   : value == "perpendicular" ? "perpendicular"
										  : "parallel";
	};
	const auto street = [&](std::string_view suffix) {
		if (auto value = get(tags, "parking:" + std::string(side) + std::string(suffix)))
			return value;
		return get(tags, "parking:both" + std::string(suffix));
	};
	if (const auto position = street("")) {
		if (street_parking_share(*position) <= 0.0)
			return std::nullopt;
		return kind(street(":orientation"));
	}
	const auto lane = [&](std::string_view suffix) {
		if (auto value = get(
					tags, "parking:lane:" + std::string(side) + std::string(suffix)))
			return value;
		return get(tags, "parking:lane:both" + std::string(suffix));
	};
	auto orientation = lane("");
	if (!orientation)
		return std::nullopt;
	if (*orientation == "marked")
		orientation = "parallel";
	else if (*orientation != "parallel" && *orientation != "diagonal" &&
			 *orientation != "perpendicular")
		return std::nullopt;
	const auto position = lane(":" + std::string(*orientation));
	const double share = position ? lane_parking_share(*position) : 1.0;
	return share > 0.0 ? std::optional<std::string_view>(kind(orientation))
					   : std::nullopt;
}

} // namespace arnis::road_width
