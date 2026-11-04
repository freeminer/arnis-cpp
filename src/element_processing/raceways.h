#pragma once

#include "../block_definitions.h"
#include "../../../arnis_adapter.h"

#include <optional>
#include <span>
#include <unordered_map>

namespace arnis::raceways
{
using Tags = std::unordered_map<std::string, std::string>;
int default_half_width(const Tags &tags);
std::span<const Block> unsurfaced_palette(const Tags &tags);
std::optional<Block> track_area_block(const Tags &tags);
int deck(const Tags &tags);
bool is_paved(const Tags &tags);
}
