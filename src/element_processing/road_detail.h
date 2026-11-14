#pragma once

#include "../args.h"
#include "../../arnis_types.h"

namespace arnis::road_detail
{

RoadDetail from_string(const std::string &value);
const char *name(RoadDetail detail);
bool skips_highway(RoadDetail detail, const tags_t &tags);
bool drops_element(RoadDetail detail, const ProcessedElement &element);
void retain(std::vector<ProcessedElement> &elements, RoadDetail detail);
bool keeps_lane_line(RoadDetail detail, const std::string &highway, const tags_t &tags,
		int road_width, bool centre, bool oneway);
std::pair<std::uint32_t, std::uint32_t> dash_pattern(
		RoadDetail detail, std::pair<std::uint32_t, std::uint32_t> pattern);
bool bar(RoadDetail detail, int index);
bool paints_crossings(RoadDetail detail);

} // namespace arnis::road_detail
