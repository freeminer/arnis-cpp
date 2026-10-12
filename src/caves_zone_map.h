#pragma once

#include "caves_shape.h"
#include "caves_theme.h"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace arnis::caves
{
// Render the same two cave-theme bands as Rust's caves::zone_map diagnostic.
// Coordinates are inclusive world-block bounds; output files are named
// `<prefix>-upper.png` and `<prefix>-deep.png`. Returns the `ZONEMAP` JSON
// payload on success, or an error message on failure.
std::optional<std::string> render_zone_map(const std::filesystem::path &prefix,
		const CaveRect &bounds, std::string_view biome_spec = {},
		std::optional<std::uint32_t> sample_step = std::nullopt,
		int floor_y = VANILLA_FLOOR, std::string *error = nullptr);
} // namespace arnis::caves
