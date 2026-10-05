#include "map_item_palette.h"
#include "colors.h"
#include <array>
#include <limits>
#include <cstdint>
namespace arnis::map_palette
{
namespace
{
using Color = std::array<std::uint8_t, 3>;
constexpr std::uint32_t MEMO_BITS = 14;
constexpr std::size_t MEMO_SLOTS = std::size_t{1} << MEMO_BITS;
constexpr std::array<unsigned, 4> shades{180, 220, 255, 135};
constexpr std::array<Color, 62> base{{{0, 0, 0}, {127, 178, 56}, {247, 233, 163},
		{199, 199, 199}, {255, 0, 0}, {160, 160, 255}, {167, 167, 167}, {0, 124, 0},
		{255, 255, 255}, {164, 168, 184}, {151, 109, 77}, {112, 112, 112}, {64, 64, 255},
		{143, 119, 72}, {255, 252, 245}, {216, 127, 51}, {178, 76, 216}, {102, 153, 216},
		{229, 229, 51}, {127, 204, 25}, {242, 127, 165}, {76, 76, 76}, {153, 153, 153},
		{76, 127, 153}, {127, 63, 178}, {51, 76, 178}, {102, 76, 51}, {102, 127, 51},
		{153, 51, 51}, {25, 25, 25}, {250, 238, 77}, {92, 219, 213}, {74, 128, 255},
		{0, 217, 58}, {129, 86, 49}, {112, 2, 0}, {209, 177, 161}, {159, 82, 36},
		{149, 87, 108}, {112, 108, 138}, {186, 133, 36}, {103, 117, 53}, {160, 77, 78},
		{57, 41, 35}, {135, 107, 98}, {87, 92, 92}, {122, 73, 88}, {76, 62, 92},
		{76, 50, 35}, {76, 82, 42}, {142, 60, 46}, {37, 22, 16}, {189, 48, 49},
		{148, 63, 97}, {92, 25, 29}, {22, 126, 134}, {58, 142, 140}, {86, 44, 62},
		{20, 180, 133}, {100, 100, 100}, {216, 175, 147}, {127, 167, 150}}};
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> shaded(std::uint8_t id)
{
	const auto &c = base[id / 4];
	const auto m = shades[id % 4];
	return {std::uint8_t(unsigned(c[0]) * m / 255),
			std::uint8_t(unsigned(c[1]) * m / 255),
			std::uint8_t(unsigned(c[2]) * m / 255)};
}

struct PaletteEntry
{
	std::uint8_t id;
	Color rgb;
	std::array<float, 3> lab;
};

const std::array<PaletteEntry, 61 * 4> &shaded_palette()
{
	static const auto palette = [] {
		std::array<PaletteEntry, 61 * 4> result{};
		for (std::size_t i = 0; i < result.size(); ++i) {
			const auto id = static_cast<std::uint8_t>(i + 4);
			const auto color = shaded(id);
			const auto rgb =
					Color{std::get<0>(color), std::get<1>(color), std::get<2>(color)};
			result[i] = {id, rgb, oklab_components(RGBTuple{rgb[0], rgb[1], rgb[2]})};
		}
		return result;
	}();
	return palette;
}

std::uint8_t nearest_map_color_uncached(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
	const auto target_rgb = Color{r, g, b};
	const auto target_lab =
			oklab_components(RGBTuple{target_rgb[0], target_rgb[1], target_rgb[2]});
	std::uint8_t best = 4;
	float distance = std::numeric_limits<float>::max();
	for (const auto &entry : shaded_palette()) {
		if (entry.rgb == target_rgb)
			return entry.id;
		const float dl = target_lab[0] - entry.lab[0];
		const float da = target_lab[1] - entry.lab[1];
		const float db = target_lab[2] - entry.lab[2];
		const float d = dl * dl + da * da + db * db;
		if (d < distance) {
			distance = d;
			best = entry.id;
		}
	}
	return best;
}
}
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> map_color_rgb(std::uint8_t id)
{
	return id < 4 ? std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>{0, 0, 0}
				  : shaded(id);
}
std::uint8_t nearest_map_color(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
	// Direct-mapped per-thread memo: atlas and map renders repeatedly quantize
	// the same colors, while this fixed table remains bounded across worlds.
	thread_local std::array<std::uint32_t, MEMO_SLOTS> memo{};
	const std::uint32_t rgb = (std::uint32_t(r) << 16) | (std::uint32_t(g) << 8) | b;
	const auto slot = static_cast<std::size_t>((rgb * 0x9E3779B1u) >> (32 - MEMO_BITS));
	const std::uint32_t entry = memo[slot];
	if (entry && (entry >> 8) == rgb)
		return static_cast<std::uint8_t>(entry);
	const auto id = nearest_map_color_uncached(r, g, b);
	memo[slot] = (rgb << 8) | id;
	return id;
}
}
