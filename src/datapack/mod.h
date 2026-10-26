#pragma once

#include <cmath>
#include <cstddef>

namespace arnis::datapack
{
inline constexpr std::size_t MAX_DESTINATIONS = 20;

// Keep macro arguments off the integer-coordinate path: Minecraft's macro
// parser recentres whole x/z values onto the adjacent block.
inline double macro_coord(double value)
{
	const double rounded = std::round(value * 100.0) / 100.0;
	return rounded == std::trunc(rounded) ? rounded + 0.01 : rounded;
}
} // namespace arnis::datapack
