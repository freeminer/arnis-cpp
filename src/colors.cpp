#include <string>
#include <string_view>
#include <optional>
#include <tuple>
#include <cctype>
#include <algorithm>
#include <cstdint>
#include <charconv>
#include <array>
#include <cmath>

#include "colors.h"
#include "unicode_text.h"

namespace
{
bool is_unicode_whitespace(std::uint32_t codepoint)
{
	return (codepoint >= 0x0009 && codepoint <= 0x000d) || codepoint == 0x0020 ||
		   codepoint == 0x0085 || codepoint == 0x00a0 || codepoint == 0x1680 ||
		   (codepoint >= 0x2000 && codepoint <= 0x200a) || codepoint == 0x2028 ||
		   codepoint == 0x2029 || codepoint == 0x202f || codepoint == 0x205f ||
		   codepoint == 0x3000;
}

std::string_view trim_unicode_whitespace(std::string_view text)
{
	std::size_t begin = 0;
	while (begin < text.size()) {
		const auto [codepoint, length] = arnis::unicode_text::codepoint_at(text, begin);
		if (!is_unicode_whitespace(codepoint))
			break;
		begin += length;
	}
	std::size_t end = text.size();
	while (end > begin) {
		std::size_t start = end - 1;
		while (start > begin && (static_cast<unsigned char>(text[start]) & 0xc0) == 0x80)
			--start;
		const auto [codepoint, length] = arnis::unicode_text::codepoint_at(text, start);
		if (start + length != end || !is_unicode_whitespace(codepoint))
			break;
		end = start;
	}
	return text.substr(begin, end - begin);
}

std::string normalize_color_name(std::string_view text)
{
	const auto lower = arnis::unicode_text::lowercase(text);
	std::string normalized;
	normalized.reserve(lower.size());
	for (std::size_t offset = 0; offset < lower.size();) {
		const auto [codepoint, length] = arnis::unicode_text::codepoint_at(lower, offset);
		if (!is_unicode_whitespace(codepoint) && codepoint != '_' && codepoint != '-')
			normalized.append(lower, offset, length);
		offset += length;
	}
	return normalized;
}
} // namespace

std::optional<RGBTuple> color_text_to_rgb_tuple(std::string_view text)
{
	text = trim_unicode_whitespace(text);
	if (auto rgb = full_hex_color_to_rgb_tuple(text))
		return rgb;
	if (auto rgb = short_hex_color_to_rgb_tuple(text))
		return rgb;
	if (auto rgb = color_name_to_rgb_tuple(text))
		return rgb;
	return std::nullopt;
}
std::uint32_t rgb_distance(const RGBTuple &from, const RGBTuple &to)
{
	std::int32_t dr = static_cast<std::int32_t>(std::get<0>(from)) -
					  static_cast<std::int32_t>(std::get<0>(to));
	std::int32_t dg = static_cast<std::int32_t>(std::get<1>(from)) -
					  static_cast<std::int32_t>(std::get<1>(to));
	std::int32_t db = static_cast<std::int32_t>(std::get<2>(from)) -
					  static_cast<std::int32_t>(std::get<2>(to));
	std::int32_t dist = dr * dr + dg * dg + db * db;
	return static_cast<std::uint32_t>(dist);
}
std::array<float, 3> oklab_components(const RGBTuple &c)
{
	auto linear = [](std::uint8_t v) {
		const float x = float(v) / 255.f;
		return x <= .04045f ? x / 12.92f : std::pow((x + .055f) / 1.055f, 2.4f);
	};
	const float r = linear(std::get<0>(c)), g = linear(std::get<1>(c)),
				b = linear(std::get<2>(c));
	const float l = std::cbrt(.41222147f * r + .53633255f * g + .051445995f * b);
	const float m = std::cbrt(.2119035f * r + .6806995f * g + .10739696f * b);
	const float s = std::cbrt(.08830246f * r + .28171884f * g + .6299787f * b);
	return {.21045426f * l + .7936178f * m - .004072047f * s,
			1.9779985f * l - 2.4285922f * m + .4505937f * s,
			.025904037f * l + .78277177f * m - .80867577f * s};
}
float oklab_distance(const RGBTuple &from, const RGBTuple &to)
{
	const auto a = oklab_components(from), b = oklab_components(to);
	return oklab_distance_lab(a, b);
}
float oklab_distance_lab(const std::array<float, 3> &from, const std::array<float, 3> &to)
{
	const float dl = from[0] - to[0], da = from[1] - to[1], db = from[2] - to[2];
	return dl * dl + da * da + db * db;
}
std::array<float, 3> rgb_to_oklab(std::uint8_t red, std::uint8_t green, std::uint8_t blue)
{
	return oklab_components(RGBTuple{red, green, blue});
}
std::optional<RGBTuple> color_name_to_rgb_tuple(std::string_view text)
{
	const std::string normalized = normalize_color_name(text);
	text = normalized;
	if (text == "aqua" || text == "cyan")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 255, 255));
	if (text == "beige")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(187, 173, 142));
	if (text == "black")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 0, 0));
	if (text == "blue")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 0, 255));
	if (text == "brown")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(128, 64, 0));
	if (text == "darkgray" || text == "darkgrey")
		return RGBTuple{96, 96, 96};
	if (text == "darkbrown")
		return RGBTuple{90, 50, 20};
	if (text == "darkred")
		return RGBTuple{139, 0, 0};
	if (text == "dimgray" || text == "dimgrey")
		return RGBTuple{105, 105, 105};
	if (text == "firebrick")
		return RGBTuple{178, 34, 34};
	if (text == "fuchsia" || text == "magenta")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(255, 0, 255));
	if (text == "gold")
		return RGBTuple{255, 215, 0};
	if (text == "gray" || text == "grey")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(128, 128, 128));
	if (text == "green")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 128, 0));
	if (text == "ivory")
		return RGBTuple{255, 255, 240};
	if (text == "khaki")
		return RGBTuple{240, 230, 140};
	if (text == "lightblue")
		return RGBTuple{173, 216, 230};
	if (text == "lightgray" || text == "lightgrey")
		return RGBTuple{211, 211, 211};
	if (text == "lightgreen")
		return RGBTuple{144, 238, 144};
	if (text == "lightyellow")
		return RGBTuple{255, 255, 224};
	if (text == "lime")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 255, 0));
	if (text == "limestone")
		return RGBTuple{246, 240, 208};
	if (text == "maroon")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(128, 0, 0));
	if (text == "navy")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 0, 128));
	if (text == "olive")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(128, 128, 0));
	if (text == "orange")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(255, 128, 0));
	if (text == "pink")
		return RGBTuple{255, 192, 203};
	if (text == "purple")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(128, 0, 128));
	if (text == "red")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(255, 0, 0));
	if (text == "salmon")
		return RGBTuple{250, 128, 114};
	if (text == "sandstone")
		return RGBTuple{215, 188, 138};
	if (text == "silver")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(192, 192, 192));
	if (text == "tan")
		return RGBTuple{210, 180, 140};
	if (text == "teal")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(0, 128, 128));
	if (text == "white")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(255, 255, 255));
	if (text == "yellow")
		return std::make_optional(
				std::make_tuple<std::uint8_t, std::uint8_t, std::uint8_t>(255, 255, 0));
	return std::nullopt;
}
std::optional<RGBTuple> short_hex_color_to_rgb_tuple(std::string_view text)
{
	if (text.size() != 4 || text.front() != '#' ||
			!std::all_of(text.begin() + 1, text.end(), [](char c) {
				return std::isxdigit(static_cast<unsigned char>(c)) != 0;
			})) {
		return std::nullopt;
	}

	unsigned int v;
	auto res = std::from_chars(text.data() + 1, text.data() + 2, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t r = static_cast<std::uint8_t>(v);
	r = static_cast<std::uint8_t>(r | (r << 4));

	res = std::from_chars(text.data() + 2, text.data() + 3, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t g = static_cast<std::uint8_t>(v);
	g = static_cast<std::uint8_t>(g | (g << 4));

	res = std::from_chars(text.data() + 3, text.data() + 4, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t b = static_cast<std::uint8_t>(v);
	b = static_cast<std::uint8_t>(b | (b << 4));

	return std::make_optional(std::make_tuple(r, g, b));
}
std::optional<RGBTuple> full_hex_color_to_rgb_tuple(std::string_view text)
{
	if (text.size() != 7 || text.front() != '#' ||
			!std::all_of(text.begin() + 1, text.end(), [](char c) {
				return std::isxdigit(static_cast<unsigned char>(c)) != 0;
			})) {
		return std::nullopt;
	}

	unsigned int v;
	auto res = std::from_chars(text.data() + 1, text.data() + 3, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t r = static_cast<std::uint8_t>(v);

	res = std::from_chars(text.data() + 3, text.data() + 5, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t g = static_cast<std::uint8_t>(v);

	res = std::from_chars(text.data() + 5, text.data() + 7, v, 16);
	if (res.ec != std::errc())
		return std::nullopt;
	std::uint8_t b = static_cast<std::uint8_t>(v);

	return std::make_optional(std::make_tuple(r, g, b));
}
