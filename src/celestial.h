#pragma once

#include <string_view>
#include <utility>

#include "block_definitions.h"

namespace arnis
{

// Rust's celestial.rs counterpart.  Earth remains the default, preserving the
// existing API for callers that do not opt into a planetary terrain source.
enum class CelestialBody
{
	Earth,
	Moon,
	Mars
};

constexpr double EARTH_RADIUS_M = 6'371'000.0;

std::string_view celestial_body_name(CelestialBody body);
std::string_view celestial_body_display_name(CelestialBody body);
CelestialBody celestial_body_from_string(std::string_view value);
std::string_view celestial_body_biome(CelestialBody body);
double celestial_radius_m(CelestialBody body);
double celestial_meters_per_block(CelestialBody body);
double celestial_scale_ratio(CelestialBody body);
double celestial_height_gain(CelestialBody body);
double celestial_vertical_exaggeration(CelestialBody body);
double celestial_world_scale(CelestialBody body);
bool is_earth(CelestialBody body);

// Surface/sub-surface palette for bodies without terrestrial land-cover data.
std::pair<Block, Block> celestial_surface_palette(CelestialBody body, int slope,
		double latitude_degrees, int ground_y, int x, int z);

} // namespace arnis
