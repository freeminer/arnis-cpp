#pragma once

#include "trees/unicode_alphabetic.h"
#include "trees/unicode_case.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

namespace arnis::unicode_text
{
inline constexpr std::array<trees::mapped::detail::CodepointRange, 137> NUMBER_RANGES{{
		{0x0030, 0x0039},
		{0x00b2, 0x00b3},
		{0x00b9, 0x00b9},
		{0x00bc, 0x00be},
		{0x0660, 0x0669},
		{0x06f0, 0x06f9},
		{0x07c0, 0x07c9},
		{0x0966, 0x096f},
		{0x09e6, 0x09ef},
		{0x09f4, 0x09f9},
		{0x0a66, 0x0a6f},
		{0x0ae6, 0x0aef},
		{0x0b66, 0x0b6f},
		{0x0b72, 0x0b77},
		{0x0be6, 0x0bf2},
		{0x0c66, 0x0c6f},
		{0x0c78, 0x0c7e},
		{0x0ce6, 0x0cef},
		{0x0d58, 0x0d5e},
		{0x0d66, 0x0d78},
		{0x0de6, 0x0def},
		{0x0e50, 0x0e59},
		{0x0ed0, 0x0ed9},
		{0x0f20, 0x0f33},
		{0x1040, 0x1049},
		{0x1090, 0x1099},
		{0x1369, 0x137c},
		{0x16ee, 0x16f0},
		{0x17e0, 0x17e9},
		{0x17f0, 0x17f9},
		{0x1810, 0x1819},
		{0x1946, 0x194f},
		{0x19d0, 0x19da},
		{0x1a80, 0x1a89},
		{0x1a90, 0x1a99},
		{0x1b50, 0x1b59},
		{0x1bb0, 0x1bb9},
		{0x1c40, 0x1c49},
		{0x1c50, 0x1c59},
		{0x2070, 0x2070},
		{0x2074, 0x2079},
		{0x2080, 0x2089},
		{0x2150, 0x2182},
		{0x2185, 0x2189},
		{0x2460, 0x249b},
		{0x24ea, 0x24ff},
		{0x2776, 0x2793},
		{0x2cfd, 0x2cfd},
		{0x3007, 0x3007},
		{0x3021, 0x3029},
		{0x3038, 0x303a},
		{0x3192, 0x3195},
		{0x3220, 0x3229},
		{0x3248, 0x324f},
		{0x3251, 0x325f},
		{0x3280, 0x3289},
		{0x32b1, 0x32bf},
		{0xa620, 0xa629},
		{0xa6e6, 0xa6ef},
		{0xa830, 0xa835},
		{0xa8d0, 0xa8d9},
		{0xa900, 0xa909},
		{0xa9d0, 0xa9d9},
		{0xa9f0, 0xa9f9},
		{0xaa50, 0xaa59},
		{0xabf0, 0xabf9},
		{0xff10, 0xff19},
		{0x10107, 0x10133},
		{0x10140, 0x10178},
		{0x1018a, 0x1018b},
		{0x102e1, 0x102fb},
		{0x10320, 0x10323},
		{0x10341, 0x10341},
		{0x1034a, 0x1034a},
		{0x103d1, 0x103d5},
		{0x104a0, 0x104a9},
		{0x10858, 0x1085f},
		{0x10879, 0x1087f},
		{0x108a7, 0x108af},
		{0x108fb, 0x108ff},
		{0x10916, 0x1091b},
		{0x109bc, 0x109bd},
		{0x109c0, 0x109cf},
		{0x109d2, 0x109ff},
		{0x10a40, 0x10a48},
		{0x10a7d, 0x10a7e},
		{0x10a9d, 0x10a9f},
		{0x10aeb, 0x10aef},
		{0x10b58, 0x10b5f},
		{0x10b78, 0x10b7f},
		{0x10ba9, 0x10baf},
		{0x10cfa, 0x10cff},
		{0x10d30, 0x10d39},
		{0x10e60, 0x10e7e},
		{0x10f1d, 0x10f26},
		{0x10f51, 0x10f54},
		{0x10fc5, 0x10fcb},
		{0x11052, 0x1106f},
		{0x110f0, 0x110f9},
		{0x11136, 0x1113f},
		{0x111d0, 0x111d9},
		{0x111e1, 0x111f4},
		{0x112f0, 0x112f9},
		{0x11450, 0x11459},
		{0x114d0, 0x114d9},
		{0x11650, 0x11659},
		{0x116c0, 0x116c9},
		{0x11730, 0x1173b},
		{0x118e0, 0x118f2},
		{0x11950, 0x11959},
		{0x11c50, 0x11c6c},
		{0x11d50, 0x11d59},
		{0x11da0, 0x11da9},
		{0x11f50, 0x11f59},
		{0x11fc0, 0x11fd4},
		{0x12400, 0x1246e},
		{0x16a60, 0x16a69},
		{0x16ac0, 0x16ac9},
		{0x16b50, 0x16b59},
		{0x16b5b, 0x16b61},
		{0x16e80, 0x16e96},
		{0x1d2c0, 0x1d2d3},
		{0x1d2e0, 0x1d2f3},
		{0x1d360, 0x1d378},
		{0x1d7ce, 0x1d7ff},
		{0x1e140, 0x1e149},
		{0x1e2f0, 0x1e2f9},
		{0x1e4f0, 0x1e4f9},
		{0x1e8c7, 0x1e8cf},
		{0x1e950, 0x1e959},
		{0x1ec71, 0x1ecab},
		{0x1ecad, 0x1ecaf},
		{0x1ecb1, 0x1ecb4},
		{0x1ed01, 0x1ed2d},
		{0x1ed2f, 0x1ed3d},
		{0x1f100, 0x1f10c},
		{0x1fbf0, 0x1fbf9},
}};

inline std::pair<std::uint32_t, std::size_t> codepoint_at(
		std::string_view text, std::size_t offset)
{
	const auto first = static_cast<unsigned char>(text[offset]);
	if (first < 0x80)
		return {first, 1};
	const std::size_t length = (first & 0xe0) == 0xc0	? 2
							   : (first & 0xf0) == 0xe0 ? 3
							   : (first & 0xf8) == 0xf0 ? 4
														: 1;
	if (length == 1 || offset + length > text.size())
		return {first, 1};
	std::uint32_t codepoint = first & ((1u << (7 - length)) - 1);
	for (std::size_t i = 1; i < length; ++i) {
		const auto byte = static_cast<unsigned char>(text[offset + i]);
		if ((byte & 0xc0) != 0x80)
			return {first, 1};
		codepoint = (codepoint << 6) | (byte & 0x3f);
	}
	return {codepoint, length};
}

inline bool is_whitespace(std::uint32_t cp)
{
	return (cp >= 0x09 && cp <= 0x0d) || cp == 0x20 || cp == 0x85 || cp == 0xa0 ||
		   cp == 0x1680 || (cp >= 0x2000 && cp <= 0x200a) || cp == 0x2028 ||
		   cp == 0x2029 || cp == 0x202f || cp == 0x205f || cp == 0x3000;
}

inline std::pair<std::size_t, std::size_t> trim_whitespace(std::string_view text)
{
	std::size_t begin = 0;
	while (begin < text.size()) {
		const auto [cp, length] = codepoint_at(text, begin);
		if (!is_whitespace(cp))
			break;
		begin += length;
	}
	std::size_t end = text.size();
	while (end > begin) {
		std::size_t start = end - 1;
		while (start > begin && (static_cast<unsigned char>(text[start]) & 0xc0) == 0x80)
			--start;
		const auto [cp, length] = codepoint_at(text, start);
		if (start + length != end || !is_whitespace(cp))
			break;
		end = start;
	}
	return {begin, end};
}

inline bool is_alphanumeric(std::uint32_t codepoint)
{
	if (codepoint < 0x80)
		return (codepoint >= '0' && codepoint <= '9') ||
			   (codepoint >= 'A' && codepoint <= 'Z') ||
			   (codepoint >= 'a' && codepoint <= 'z');
	if (trees::mapped::detail::is_alphabetic(codepoint))
		return true;
	const auto it = std::lower_bound(NUMBER_RANGES.begin(), NUMBER_RANGES.end(),
			codepoint,
			[](const auto &range, std::uint32_t value) { return range.last < value; });
	return it != NUMBER_RANGES.end() && it->first <= codepoint;
}

inline void append_utf8(std::string &out, std::uint32_t codepoint)
{
	if (codepoint <= 0x7f) {
		out.push_back(static_cast<char>(codepoint));
	} else if (codepoint <= 0x7ff) {
		out.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
		out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
	} else if (codepoint <= 0xffff) {
		out.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
		out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
		out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
	} else {
		out.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
		out.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
		out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
		out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
	}
}

inline std::string lowercase(std::string_view text)
{
	std::string result;
	result.reserve(text.size());
	for (std::size_t offset = 0; offset < text.size();) {
		const auto [codepoint, length] = codepoint_at(text, offset);
		if (length == 1 && codepoint >= 0x80) {
			result.push_back(text[offset++]);
			continue;
		}
		std::array<std::uint32_t, 3> mapped{};
		const auto count = trees::mapped::detail::case_map(codepoint, false, mapped);
		for (std::size_t i = 0; i < count; ++i)
			append_utf8(result, mapped[i]);
		offset += length;
	}
	return result;
}
} // namespace arnis::unicode_text
