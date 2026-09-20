#pragma once

#include "fit.h"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace arnis::world_editor
{
struct WorldEditor;
}

namespace arnis::building_facades
{
struct RgbImage
{
	std::uint32_t width = 0, height = 0;
	std::vector<std::uint8_t> pixels;
	bool valid() const { return width && height && pixels.size() == width * height * 3; }
	const std::uint8_t *pixel(std::uint32_t x, std::uint32_t y) const;
};

// Decode an entry's PNG and resample it to the requested pixels-per-metre.
// The decoded image is RGB (not RGBA), matching Rust's RgbImage contract.
std::optional<RgbImage> load_image(const std::filesystem::path &directory,
		const Entry &entry, double pixels_per_metre);

// Gather a wall region through Fit's lookup tables.  The source image is
// already scaled to pixels_per_metre, so this never stretches individual
// pixels and preserves Rust's bottom-anchored vertical mapping.
std::optional<RgbImage> gather_region(const Fit &, const RgbImage &, double x0_m,
		double x1_m, double y0_m, double y1_m);

bool submit_panel(arnis::world_editor::WorldEditor &, int x, int y, int z,
		std::int8_t facing, const RgbImage &);
}
