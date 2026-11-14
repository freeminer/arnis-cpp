#pragma once
#include "biome.h"
#include "../../arnis_block.h"
#include <cstdint>
#include <filesystem>
#include <optional>
#include <functional>
#include <string>
#include <vector>
namespace arnis::geographic
{
class LLBBox;
}
namespace arnis::climate
{
enum class ClimateMode
{
	Origin,
	PerPosition
};

using GeoFunction = std::function<std::pair<double, double>(double, double)>;

class ClimateField
{
	GeoFunction geo_;
	int cell_ = 4;
	int origin_x_ = 0, origin_z_ = 0;
	std::size_t width_ = 0, height_ = 0;
	std::vector<arnis::biome::Climate> climates_;

public:
	ClimateField(const GeoFunction &geo, std::pair<int, int> alignment,
			std::size_t world_width, std::size_t world_height);
	arnis::biome::Climate climate(int x, int z) const;
	double latitude(int x, int z) const;
};

arnis::biome::Climate from_koppen_class(unsigned char c);
// Köppen-Geiger 0.1° classification at a geographic point / bbox centre.
// Missing or malformed bundled data deliberately falls back to Temperate.
arnis::biome::Climate classify(double latitude, double longitude);
arnis::biome::Climate classify_warped(double latitude, double longitude);
double koppen_cell_degrees();
arnis::biome::Climate classify_bbox(double min_latitude, double min_longitude,
		double max_latitude, double max_longitude);
// Write a north-up <prefix>.png climate preview and return its CLIMATEMAP JSON
// payload. The caller owns output routing; generation never writes diagnostics
// implicitly.
std::optional<std::string> render_climate_map(const std::filesystem::path &prefix,
		const geographic::LLBBox &bbox, std::string *error = nullptr);
std::optional<std::pair<Block, Block>> surface_palette(
		arnis::biome::Climate, std::uint8_t, int, int, bool red_earth = false);
Block speckle(Block block, int x, int z);
}
