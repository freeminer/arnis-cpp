#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace arnis::decals
{
inline constexpr std::uint32_t TILE = 128;
namespace colors
{
constexpr std::uint8_t id(std::uint8_t base, std::uint8_t shade)
{
	return base * 4 + shade;
}
inline constexpr auto WHITE = id(8, 2), OFF_WHITE = id(14, 2), LIGHT_GRAY = id(3, 2),
					  GRAY = id(22, 2), DARK_GRAY = id(21, 2), NEAR_BLACK = id(21, 3),
					  BLACK = id(29, 2), RED = id(4, 0), BRIGHT_RED = id(4, 2),
					  DARK_RED = id(28, 1), ORANGE = id(15, 2), YELLOW = id(18, 2),
					  GOLD = id(30, 2), GREEN = id(7, 1), LIGHT_GREEN = id(19, 2),
					  SIGN_GREEN = id(27, 1), BLUE = id(25, 2), DARK_BLUE = id(25, 0),
					  LIGHT_BLUE = id(17, 2), SKY = id(32, 2), CYAN = id(23, 2),
					  PURPLE = id(24, 2), MAGENTA = id(16, 2), PINK = id(20, 2),
					  BROWN = id(26, 2), WOOD = id(13, 2), SAND = id(2, 2),
					  TEAL = id(55, 2);
}

class Canvas
{
	std::vector<std::uint8_t> pixels_;

public:
	std::uint32_t width{0};
	std::uint32_t height{0};
	Canvas() = default;
	Canvas(std::uint32_t cols, std::uint32_t rows);
	static Canvas with_size(std::uint32_t width, std::uint32_t height);
	std::uint8_t get(int x, int y) const;
	void set(int x, int y, std::uint8_t color);
	void fill(std::uint8_t color);
	void fill_rect(int x, int y, int width, int height, std::uint8_t color);
	void stroke_rect(
			int x, int y, int width, int height, int thickness, std::uint8_t color);
	void rounded_rect(
			int x, int y, int width, int height, int radius, std::uint8_t color);
	void stroke_rounded_rect(int x, int y, int width, int height, int radius,
			int thickness, std::uint8_t color);
	void disc(int center_x, int center_y, int radius, std::uint8_t color);
	void ring(int center_x, int center_y, int outer_radius, int inner_radius,
			std::uint8_t color);
	void polygon(const std::vector<std::pair<float, float>> &points, std::uint8_t color);
	void regular_polygon(int center_x, int center_y, float radius, std::size_t sides,
			float rotation, std::uint8_t color);
	void line(int x0, int y0, int x1, int y1, int line_width, std::uint8_t color);
	void blit_rgba(const std::uint8_t *pixels, std::uint32_t source_width,
			std::uint32_t source_height, int x = 0, int y = 0);
	std::vector<std::int8_t> tile(std::uint32_t col, std::uint32_t row) const;
	const std::vector<std::uint8_t> &pixels() const { return pixels_; }
};

std::uint8_t mix(std::uint8_t first, std::uint8_t second, float first_weight);
constexpr std::uint8_t darker(std::uint8_t color)
{
	return colors::id(color / 4, 0);
}
}
