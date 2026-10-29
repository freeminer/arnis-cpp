#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace arnis::rust_numeric
{
// Rust's float-to-integer `as` conversion saturates and maps NaN to zero.
inline std::int32_t saturating_i32(double value) noexcept
{
	if (std::isnan(value))
		return 0;
	if (value >= static_cast<double>(std::numeric_limits<std::int32_t>::max()))
		return std::numeric_limits<std::int32_t>::max();
	if (value <= static_cast<double>(std::numeric_limits<std::int32_t>::min()))
		return std::numeric_limits<std::int32_t>::min();
	return static_cast<std::int32_t>(value);
}

// Rust's `value.round() as i32`: halfway values round away from zero first.
inline std::int32_t round_i32(double value) noexcept
{
	return saturating_i32(std::round(value));
}

// Rust's `value.round() as usize`, including saturating float conversion.
inline std::size_t round_usize(double value) noexcept
{
	const double rounded = std::round(value);
	if (std::isnan(rounded) || rounded <= 0.0)
		return 0;
	if (!std::isfinite(rounded) ||
			rounded >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
		return std::numeric_limits<std::size_t>::max();
	return static_cast<std::size_t>(rounded);
}
} // namespace arnis::rust_numeric
