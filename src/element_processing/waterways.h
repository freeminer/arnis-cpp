#pragma once

#include "../../../arnis_types.h"

namespace arnis::waterways
{
int get_waterway_width(const std::string &waterway_type);
bool is_channel_waterway(const std::string &waterway_type);
bool is_underground_waterway(const tags_t &tags);
constexpr int MAX_WATERWAY_WIDTH = 128;
int waterway_width(const std::string &waterway_type, const tags_t &tags);
void generate_waterways(WorldEditor &, const ProcessedWay &);
}
