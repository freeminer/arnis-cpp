#include "map_item_palette.h"
#include "colors.h"
#include <array>
#include <limits>
namespace arnis::map_palette
{
namespace
{
using Color = std::array<std::uint8_t, 3>;
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
}
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t> map_color_rgb(std::uint8_t id)
{
	return id < 4 ? std::tuple<std::uint8_t, std::uint8_t, std::uint8_t>{0, 0, 0}
				  : shaded(id);
}
std::uint8_t nearest_map_color(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
	const RGBTuple target{r, g, b};
	std::uint8_t best = 4;
	float distance = std::numeric_limits<float>::max();
	for (unsigned id = 4; id < base.size() * 4; ++id) {
		const auto color = shaded(std::uint8_t(id));
		if (color == target)
			return std::uint8_t(id);
		const float d = oklab_distance(target, color);
		if (d < distance) {
			distance = d;
			best = std::uint8_t(id);
		}
	}
	return best;
}
}
