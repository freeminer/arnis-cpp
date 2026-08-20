#pragma once
#include <cstdint>
#include <tuple>
namespace arnis::map_palette
{
constexpr std::uint8_t map_color_id(std::uint8_t base, std::uint8_t shade)
{
	return base * 4 + shade;
}
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> map_color_rgb(std::uint8_t color_id);
std::uint8_t nearest_map_color(std::uint8_t r, std::uint8_t g, std::uint8_t b);
}
inline constexpr std::uint8_t TRANSPARENT = 0;
