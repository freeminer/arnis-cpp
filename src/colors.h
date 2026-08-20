#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <array>

using RGBTuple = std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>;
using RGB = RGBTuple;

std::optional<RGBTuple> full_hex_color_to_rgb_tuple(std::string_view text);

std::optional<RGBTuple> short_hex_color_to_rgb_tuple(std::string_view text);

std::optional<RGBTuple> color_name_to_rgb_tuple(std::string_view text);

std::optional<RGBTuple> color_text_to_rgb_tuple(std::string_view text);

std::uint32_t rgb_distance(const RGBTuple &from, const RGBTuple &to);
// Squared perceptual distance in Oklab; used by 3D model material mapping.
float oklab_distance(const RGBTuple &from, const RGBTuple &to);
std::array<float, 3> oklab_components(const RGBTuple &color);
