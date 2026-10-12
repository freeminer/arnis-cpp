#pragma once

#include <charconv>
#include <cmath>
#include <optional>
#include <string_view>

namespace arnis::strict_parse
{
// Match Rust's str::parse::<i32> semantics: accept a complete decimal integer
// (including a leading '+'), but never accept a numeric prefix or whitespace.
inline std::optional<int> i32(std::string_view text)
{
	if (text.empty())
		return std::nullopt;
	const char *first = text.data();
	const char *last = first + text.size();
	if (*first == '+') {
		++first;
		if (first == last || *first == '+' || *first == '-')
			return std::nullopt;
	}
	if (first == last)
		return std::nullopt;
	int value = 0;
	const auto [end, error] = std::from_chars(first, last, value);
	if (error != std::errc{} || end != last)
		return std::nullopt;
	return value;
}

// Rust call sites commonly use parse::<i32>().unwrap_or(0) for optional OSM
// layer/level tags. Keep that fallback explicit while rejecting numeric
// prefixes that std::stoi would otherwise accept.
inline int i32_or_zero(std::string_view text)
{
	return i32(text).value_or(0);
}

inline std::optional<double> f64(std::string_view text)
{
	if (text.empty())
		return std::nullopt;
	const char *first = text.data();
	const char *last = first + text.size();
	if (*first == '+') {
		++first;
		if (first == last || *first == '+' || *first == '-')
			return std::nullopt;
	}
	if (first == last)
		return std::nullopt;
	double value = 0.0;
	const auto [end, error] =
			std::from_chars(first, last, value, std::chars_format::general);
	if (error != std::errc{} || end != last)
		return std::nullopt;
	return value;
}
} // namespace arnis::strict_parse
