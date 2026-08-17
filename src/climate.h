#pragma once
#include "biome.h"
#include "../../arnis_block.h"
#include <optional>
namespace arnis::climate
{
arnis::biome::Climate from_koppen_class(unsigned char c);
// Köppen-Geiger 0.1° classification at a geographic point / bbox centre.
// Missing or malformed bundled data deliberately falls back to Temperate.
arnis::biome::Climate classify(double latitude, double longitude);
arnis::biome::Climate classify_bbox(double min_latitude, double min_longitude,
		double max_latitude, double max_longitude);
std::optional<std::pair<Block,Block>> surface_palette(arnis::biome::Climate, std::uint8_t, int, int);
}
