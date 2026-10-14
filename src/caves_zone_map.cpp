#include "caves_zone_map.h"

#include "args.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <png.h>
#include <vector>

#include <nlohmann/json.hpp>

namespace arnis::caves
{
namespace
{
constexpr std::int64_t cave_seed = 0xCA7ECA7E;
constexpr std::uint32_t max_side = 1536;
constexpr std::uint64_t max_pixels = 64ull * 1024 * 1024;

struct Rgba
{
	std::uint8_t r, g, b, a;
};

std::optional<std::pair<Rgba, const char *>> style(CaveTheme theme)
{
	switch (theme) {
	case CaveTheme::Normal:
		return std::nullopt;
	case CaveTheme::Lush:
		return std::pair{Rgba{108, 207, 95, 210}, "lush"};
	case CaveTheme::Dripstone:
		return std::pair{Rgba{201, 141, 90, 210}, "dripstone"};
	case CaveTheme::DeepDark:
		return std::pair{Rgba{31, 66, 92, 220}, "deepdark"};
	case CaveTheme::Mushroom:
		return std::pair{Rgba{176, 123, 168, 210}, "mushroom"};
	case CaveTheme::Ice:
		return std::pair{Rgba{159, 216, 255, 210}, "ice"};
	case CaveTheme::Amethyst:
		return std::pair{Rgba{154, 107, 216, 210}, "amethyst"};
	case CaveTheme::Volcanic:
		return std::pair{Rgba{224, 102, 60, 220}, "volcanic"};
	}
	return std::nullopt;
}

int square_center(int min, int max, std::uint32_t index, int step)
{
	const auto start =
			static_cast<std::int64_t>(min) + static_cast<std::int64_t>(index) * step;
	const auto len = std::min<std::int64_t>(start + step - 1, max) - start + 1;
	return static_cast<int>(start + len / 2);
}

bool save_png(const std::filesystem::path &path, std::uint32_t width,
		std::uint32_t height, const std::vector<std::uint8_t> &rgba)
{
	png_image image{};
	image.version = PNG_IMAGE_VERSION;
	image.width = width;
	image.height = height;
	image.format = PNG_FORMAT_RGBA;
	return png_image_write_to_file(
				   &image, path.string().c_str(), 0, rgba.data(), 0, nullptr) != 0;
}
} // namespace

std::optional<std::string> render_zone_map(const std::filesystem::path &prefix,
		const CaveRect &bounds, std::string_view biome_spec,
		std::optional<std::uint32_t> sample_step, int floor_y, std::string *error)
{
	if (error)
		error->clear();
	auto fail = [&](std::string message) -> std::optional<std::string> {
		if (error)
			*error = std::move(message);
		return std::nullopt;
	};
	if (bounds.min_x > bounds.max_x || bounds.min_z > bounds.max_z)
		return fail("cave zone map bounds are empty");
	BiomeAmounts amounts;
	if (!biome_spec.empty()) {
		std::string parse_error;
		amounts = BiomeAmounts::parse(biome_spec, &parse_error);
		if (!parse_error.empty())
			return fail("--cave-biomes: " + parse_error);
	}
	const auto span_x = static_cast<std::uint64_t>(
			static_cast<std::int64_t>(bounds.max_x) - bounds.min_x + 1);
	const auto span_z = static_cast<std::uint64_t>(
			static_cast<std::int64_t>(bounds.max_z) - bounds.min_z + 1);
	const auto step =
			sample_step ? static_cast<int>(std::clamp(*sample_step, 1u, 512u))
						: static_cast<int>(std::max<std::uint64_t>(1,
								  (std::max(span_x, span_z) + max_side - 1) / max_side));
	const auto width64 = std::max<std::uint64_t>(1, (span_x + step - 1) / step);
	const auto height64 = std::max<std::uint64_t>(1, (span_z + step - 1) / step);
	if (width64 > std::numeric_limits<std::uint32_t>::max() ||
			height64 > std::numeric_limits<std::uint32_t>::max() ||
			width64 * height64 > max_pixels)
		return fail("cave zone map raster exceeds the supported image size");
	const auto width = static_cast<std::uint32_t>(width64);
	const auto height = static_cast<std::uint32_t>(height64);
	const auto pixel_count = static_cast<std::size_t>(width64 * height64);
	const ThemeSelector decor(cave_seed, amounts, floor_y);
	const auto upper_y = floor_y + (-20 - VANILLA_FLOOR);
	const auto deep_y = floor_y + (-48 - VANILLA_FLOOR);
	const auto high_surface = floor_y + (200 - VANILLA_FLOOR);
	const std::array<std::pair<const char *, int>, 2> bands{
			{{"upper", upper_y}, {"deep", deep_y}}};
	nlohmann::json result = nlohmann::json::object();
	for (const auto &[tag, y] : bands) {
		std::vector<std::uint8_t> rgba(pixel_count * 4, 0);
		std::map<std::string, std::uint64_t> counts;
		for (std::uint32_t pz = 0; pz < height; ++pz) {
			const int bz = square_center(bounds.min_z, bounds.max_z, pz, step);
			for (std::uint32_t px = 0; px < width; ++px) {
				const int bx = square_center(bounds.min_x, bounds.max_x, px, step);
				Rgba color{0, 0, 0, 0};
				if (const auto themed = style(decor.at(bx, y, bz, high_surface))) {
					color = themed->first;
					++counts[themed->second];
				} else if (decor.coral_zone(bx, bz)) {
					color = {255, 126, 168, 160};
					++counts["coral"];
				} else {
					++counts["plain"];
				}
				const auto offset = (static_cast<std::size_t>(pz) * width + px) * 4;
				rgba[offset] = color.r;
				rgba[offset + 1] = color.g;
				rgba[offset + 2] = color.b;
				rgba[offset + 3] = color.a;
			}
		}
		const auto path = std::filesystem::path(prefix.string() + "-" + tag + ".png");
		if (!save_png(path, width, height, rgba))
			return fail("write " + path.string() + ": PNG encoding failed");
		auto &stats = result[tag];
		for (const auto &[name, count] : counts) {
			const double pct = double(count) * 100.0 / double(pixel_count);
			stats[name] = std::round(pct * 10.0) / 10.0;
		}
		stats["_file"] = path.string();
	}
	result["_bbox_blocks"] = {bounds.min_x, bounds.min_z, bounds.max_x, bounds.max_z};
	return "ZONEMAP " + result.dump();
}

std::optional<std::string> render_zone_map(const ::arnis::Args &args,
		const CaveRect &bounds, int floor_y, std::string *error)
{
	if (!args.cave_zone_map) {
		if (error)
			*error = "cave-zone map output path is not configured";
		return std::nullopt;
	}
	return render_zone_map(*args.cave_zone_map, bounds,
			args.cave_biomes.value_or(std::string{}), args.cave_zone_map_step, floor_y,
			error);
}
} // namespace arnis::caves
