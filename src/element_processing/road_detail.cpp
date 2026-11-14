#include "road_detail.h"

#include <algorithm>
#include <array>

namespace arnis::road_detail
{
namespace
{
bool pedestrian_grade(const std::string &highway)
{
	static const std::array<const char *, 9> values{"footway", "path", "cycleway",
			"steps", "corridor", "pedestrian", "platform", "bus_stop", "track"};
	return std::any_of(values.begin(), values.end(),
			[&](const char *value) { return highway == value; });
}

bool minor_service(const tags_t &tags)
{
	const auto service = tags.get("service");
	return service == "parking_aisle" || service == "driveway" || service == "alley" ||
		   service == "drive-through" || service == "drive_through";
}
} // namespace

RoadDetail from_string(const std::string &value)
{
	if (value == "compact")
		return RoadDetail::Compact;
	if (value == "clean")
		return RoadDetail::Clean;
	return RoadDetail::Max;
}

const char *name(RoadDetail detail)
{
	switch (detail) {
	case RoadDetail::Max:
		return "max";
	case RoadDetail::Clean:
		return "clean";
	case RoadDetail::Compact:
		return "compact";
	}
	return "max";
}

bool skips_highway(RoadDetail detail, const tags_t &tags)
{
	if (detail != RoadDetail::Compact)
		return false;
	const auto highway = tags.get("highway");
	if (highway == "footway" || highway == "path" || highway == "cycleway" ||
			highway == "steps" || highway == "track" || highway == "crossing")
		return true;
	return highway == "service" && minor_service(tags);
}

bool drops_element(RoadDetail detail, const ProcessedElement &element)
{
	if (detail != RoadDetail::Compact || element.is_relation() ||
			!skips_highway(detail, element.tags()) ||
			element.tags().get("man_made") == "pier")
		return false;
	static const std::array<const char *, 10> other_handlers{"building", "building:part",
			"aerialway", "aeroway", "amenity", "attraction", "barrier", "door",
			"entrance", "natural"};
	return std::none_of(other_handlers.begin(), other_handlers.end(),
			[&](const char *key) { return element.tags().contains(key); });
}

void retain(std::vector<ProcessedElement> &elements, RoadDetail detail)
{
	if (detail == RoadDetail::Max)
		return;
	elements.erase(
			std::remove_if(elements.begin(), elements.end(),
					[&](const auto &element) { return drops_element(detail, element); }),
			elements.end());
}

bool keeps_lane_line(RoadDetail detail, const std::string &highway, const tags_t &tags,
		int road_width, bool centre, bool oneway)
{
	if (detail == RoadDetail::Max)
		return true;
	const bool unmarked = road_width < 4 || highway == "service" ||
						  pedestrian_grade(highway) ||
						  tags.get("service") == "parking_aisle";
	return (centre || (oneway && road_width > 5)) && !unmarked;
}

std::pair<std::uint32_t, std::uint32_t> dash_pattern(
		RoadDetail detail, std::pair<std::uint32_t, std::uint32_t> pattern)
{
	if (detail != RoadDetail::Compact)
		return pattern;
	const auto on = std::max(4u, pattern.first);
	const auto off = std::max(
			4u, pattern.second > pattern.first ? pattern.second - pattern.first : 0u);
	return {on, on + off};
}

bool bar(RoadDetail detail, int index)
{
	return detail == RoadDetail::Max ? index % 2 == 0 : ((index % 4) + 4) % 4 < 2;
}

bool paints_crossings(RoadDetail detail)
{
	return detail != RoadDetail::Compact;
}

} // namespace arnis::road_detail
