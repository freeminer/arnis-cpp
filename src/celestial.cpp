#include "celestial.h"

#include "ground_generation.h"
#include "land_cover/land_cover.h"

#include "../../arnis_adapter.h"

#include <cmath>
#include <cctype>
#include <string>

namespace arnis
{

std::string_view celestial_body_name(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Earth:
		return "earth";
	case CelestialBody::Moon:
		return "moon";
	case CelestialBody::Mars:
		return "mars";
	}
	return "earth";
}

std::string_view celestial_body_display_name(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Earth:
		return "Earth";
	case CelestialBody::Moon:
		return "Moon";
	case CelestialBody::Mars:
		return "Mars";
	}
	return "Earth";
}

CelestialBody celestial_body_from_string(std::string_view value)
{
	std::string normalized(value);
	for (char &c : normalized)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	const auto first = normalized.find_first_not_of(" \t\r\n");
	if (first != std::string::npos)
		normalized.erase(0, first);
	const auto last = normalized.find_last_not_of(" \t\r\n");
	if (last != std::string::npos)
		normalized.erase(last + 1);
	if (normalized == "moon" || normalized == "luna")
		return CelestialBody::Moon;
	if (normalized == "mars")
		return CelestialBody::Mars;
	return CelestialBody::Earth;
}

std::string_view celestial_body_biome(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Moon:
		return "minecraft:stony_peaks";
	case CelestialBody::Mars:
		return "minecraft:badlands";
	case CelestialBody::Earth:
		return "minecraft:plains";
	}
	return "minecraft:plains";
}

bool is_earth(CelestialBody body)
{
	return body == CelestialBody::Earth;
}

double celestial_radius_m(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Moon:
		return 1'737'400.0;
	case CelestialBody::Mars:
		return 3'396'000.0;
	case CelestialBody::Earth:
		return EARTH_RADIUS_M;
	}
	return EARTH_RADIUS_M;
}

double celestial_meters_per_block(CelestialBody body)
{
	switch (body) {
	case CelestialBody::Moon:
		return 200.0;
	case CelestialBody::Mars:
		return 500.0;
	case CelestialBody::Earth:
		return 1.0;
	}
	return 1.0;
}

double celestial_scale_ratio(CelestialBody body)
{
	return celestial_radius_m(body) / EARTH_RADIUS_M;
}

double celestial_height_gain(CelestialBody body)
{
	return EARTH_RADIUS_M / celestial_radius_m(body);
}

double celestial_vertical_exaggeration(CelestialBody body)
{
	return is_earth(body) ? 1.0 : 4.0;
}

double celestial_world_scale(CelestialBody body)
{
	return is_earth(body)
				   ? 1.0
				   : celestial_scale_ratio(body) / celestial_meters_per_block(body);
}

std::pair<Block, Block> celestial_surface_palette(CelestialBody body, int slope,
		double latitude_degrees, int ground_y, int x, int z)
{
	using namespace block_definitions;
	if (body == CelestialBody::Earth)
		return {GRASS_BLOCK, DIRT};
	if (body == CelestialBody::Moon)
		return {END_STONE, END_STONE};

	const auto h = land_cover::coord_hash(x, z);
	if (std::abs(latitude_degrees) > 74.0 && slope <= 4) {
		switch (h % 12) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			return {SNOW_BLOCK, WHITE_TERRACOTTA};
		case 7:
		case 8:
		case 9:
			return {WHITE_CONCRETE, WHITE_TERRACOTTA};
		default:
			return {WHITE_TERRACOTTA, WHITE_TERRACOTTA};
		}
	}
	if (slope > 8) {
		switch ((ground_y % 9 + 9) % 9) {
		case 0:
		case 1:
		case 2:
		case 3:
			return {RED_TERRACOTTA, RED_TERRACOTTA};
		case 4:
		case 5:
		case 6:
			return {BROWN_TERRACOTTA, BROWN_TERRACOTTA};
		default:
			return {TERRACOTTA, TERRACOTTA};
		}
	}
	if (slope > 6) {
		const auto k = h % 20;
		return k <= 10	 ? std::pair{TERRACOTTA, RED_TERRACOTTA}
			   : k <= 15 ? std::pair{GRANITE, RED_TERRACOTTA}
						 : std::pair{RED_TERRACOTTA, RED_TERRACOTTA};
	}
	if (slope > 4) {
		switch (h % 12) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			return {ORANGE_TERRACOTTA, RED_TERRACOTTA};
		case 5:
		case 6:
		case 7:
			return {TERRACOTTA, RED_TERRACOTTA};
		case 8:
		case 9:
			return {GRANITE, RED_TERRACOTTA};
		default:
			return {GRAVEL, RED_TERRACOTTA};
		}
	}
	const double drift = ground_generation::value_noise_01(x, z, 6);
	if (drift > .72)
		return {TERRACOTTA, RED_TERRACOTTA};
	if (drift < .26)
		return {RED_TERRACOTTA, RED_TERRACOTTA};
	switch (h % 20) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
	case 10:
	case 11:
	case 12:
	case 13:
		return {ORANGE_TERRACOTTA, RED_TERRACOTTA};
	case 14:
	case 15:
	case 16:
	case 17:
		return {TERRACOTTA, RED_TERRACOTTA};
	default:
		return {BROWN_TERRACOTTA, RED_TERRACOTTA};
	}
}

} // namespace arnis
