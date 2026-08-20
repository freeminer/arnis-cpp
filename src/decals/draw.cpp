#include "draw.h"

#include "../map_item_palette.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace arnis::decals
{
namespace
{
bool inside_rounded(int x, int y, int width, int height, int radius)
{
	if (x < 0 || y < 0 || x >= width || y >= height)
		return false;
	if (radius <= 0)
		return true;
	const int cx = x < radius ? radius - 1 : x >= width - radius ? width - radius : x;
	const int cy = y < radius ? radius - 1 : y >= height - radius ? height - radius : y;
	if (cx == x || cy == y)
		return true;
	const float dx = x - cx, dy = y - cy;
	return dx * dx + dy * dy <= (radius + .5f) * (radius + .5f);
}
}

Canvas::Canvas(std::uint32_t cols, std::uint32_t rows) :
		Canvas(with_size(cols * TILE, rows * TILE))
{
}
Canvas Canvas::with_size(std::uint32_t w, std::uint32_t h)
{
	Canvas canvas;
	canvas.width = w;
	canvas.height = h;
	canvas.pixels_.assign(std::size_t(w) * h, TRANSPARENT);
	return canvas;
}
std::uint8_t Canvas::get(int x, int y) const
{
	return x < 0 || y < 0 || x >= int(width) || y >= int(height)
				   ? TRANSPARENT
				   : pixels_[std::size_t(y) * width + x];
}
void Canvas::set(int x, int y, std::uint8_t color)
{
	if (x >= 0 && y >= 0 && x < int(width) && y < int(height))
		pixels_[std::size_t(y) * width + x] = color;
}
void Canvas::fill(std::uint8_t color)
{
	std::fill(pixels_.begin(), pixels_.end(), color);
}
void Canvas::fill_rect(int x, int y, int w, int h, std::uint8_t color)
{
	for (int py = y; py < y + h; ++py)
		for (int px = x; px < x + w; ++px)
			set(px, py, color);
}
void Canvas::stroke_rect(int x, int y, int w, int h, int t, std::uint8_t color)
{
	fill_rect(x, y, w, t, color);
	fill_rect(x, y + h - t, w, t, color);
	fill_rect(x, y, t, h, color);
	fill_rect(x + w - t, y, t, h, color);
}
void Canvas::rounded_rect(int x, int y, int w, int h, int radius, std::uint8_t color)
{
	radius = std::clamp(radius, 0, std::min(w / 2, h / 2));
	for (int py = 0; py < h; ++py)
		for (int px = 0; px < w; ++px)
			if (inside_rounded(px, py, w, h, radius))
				set(x + px, y + py, color);
}
void Canvas::stroke_rounded_rect(
		int x, int y, int w, int h, int radius, int t, std::uint8_t color)
{
	radius = std::clamp(radius, 0, std::min(w / 2, h / 2));
	for (int py = 0; py < h; ++py)
		for (int px = 0; px < w; ++px)
			if (inside_rounded(px, py, w, h, radius) &&
					!(px >= t && py >= t && px < w - t && py < h - t &&
							inside_rounded(px - t, py - t, w - 2 * t, h - 2 * t,
									std::max(0, radius - t))))
				set(x + px, y + py, color);
}
void Canvas::disc(int cx, int cy, int radius, std::uint8_t color)
{
	const float rr = (radius + .5f) * (radius + .5f);
	for (int y = cy - radius; y <= cy + radius; ++y)
		for (int x = cx - radius; x <= cx + radius; ++x) {
			const float dx = x - cx, dy = y - cy;
			if (dx * dx + dy * dy <= rr)
				set(x, y, color);
		}
}
void Canvas::ring(int cx, int cy, int outer, int inner, std::uint8_t color)
{
	const float ro = (outer + .5f) * (outer + .5f), ri = (inner + .5f) * (inner + .5f);
	for (int y = cy - outer; y <= cy + outer; ++y)
		for (int x = cx - outer; x <= cx + outer; ++x) {
			const float dx = x - cx, dy = y - cy, d = dx * dx + dy * dy;
			if (d <= ro && d > ri)
				set(x, y, color);
		}
}
void Canvas::polygon(
		const std::vector<std::pair<float, float>> &points, std::uint8_t color)
{
	if (points.size() < 3)
		return;
	const auto [min_it, max_it] = std::minmax_element(points.begin(), points.end(),
			[](const auto &a, const auto &b) { return a.second < b.second; });
	for (int y = int(std::floor(min_it->second)); y <= int(std::ceil(max_it->second));
			++y) {
		std::vector<float> xs;
		const float scan = y + .5f;
		for (std::size_t i = 0; i < points.size(); ++i) {
			const auto [x0, y0] = points[i];
			const auto [x1, y1] = points[(i + 1) % points.size()];
			if ((y0 <= scan && y1 > scan) || (y1 <= scan && y0 > scan))
				xs.push_back(x0 + (scan - y0) * (x1 - x0) / (y1 - y0));
		}
		std::sort(xs.begin(), xs.end());
		for (std::size_t i = 1; i < xs.size(); i += 2)
			for (int x = int(std::lround(xs[i - 1])); x < int(std::lround(xs[i])); ++x)
				set(x, y, color);
	}
}
void Canvas::regular_polygon(int cx, int cy, float radius, std::size_t sides,
		float rotation, std::uint8_t color)
{
	std::vector<std::pair<float, float>> points;
	for (std::size_t i = 0; i < sides; ++i) {
		const float angle = rotation + i * 2.0f * std::numbers::pi_v<float> / sides;
		points.emplace_back(
				cx + .5f + radius * std::sin(angle), cy + .5f - radius * std::cos(angle));
	}
	polygon(points, color);
}
void Canvas::line(int x0, int y0, int x1, int y1, int line_width, std::uint8_t color)
{
	const float dx = x1 - x0, dy = y1 - y0, length = std::max(1.0f, std::hypot(dx, dy));
	const int steps = int(std::ceil(length)) * 2,
			  radius = std::max(0, (line_width - 1) / 2);
	for (int step = 0; step <= steps; ++step) {
		const float t = float(step) / steps;
		const int x = int(std::lround(x0 + dx * t)), y = int(std::lround(y0 + dy * t));
		if (radius)
			disc(x, y, radius, color);
		else
			set(x, y, color);
	}
}
void Canvas::blit_rgba(const std::uint8_t *pixels, std::uint32_t source_width,
		std::uint32_t source_height, int x0, int y0)
{
	if (!pixels)
		return;
	for (std::uint32_t y = 0; y < source_height; ++y)
		for (std::uint32_t x = 0; x < source_width; ++x) {
			const auto *pixel = pixels + (std::size_t(y) * source_width + x) * 4;
			if (pixel[3] >= 128)
				set(x0 + x, y0 + y,
						map_palette::nearest_map_color(pixel[0], pixel[1], pixel[2]));
		}
}
std::vector<std::int8_t> Canvas::tile(std::uint32_t col, std::uint32_t row) const
{
	std::vector<std::int8_t> result(TILE * TILE, std::int8_t(TRANSPARENT));
	for (std::uint32_t y = 0; y < TILE; ++y)
		for (std::uint32_t x = 0; x < TILE; ++x)
			result[y * TILE + x] =
					std::int8_t(get(int(col * TILE + x), int(row * TILE + y)));
	return result;
}
std::uint8_t mix(std::uint8_t first, std::uint8_t second, float weight)
{
	const auto [ar, ag, ab] = map_palette::map_color_rgb(first);
	const auto [br, bg, bb] = map_palette::map_color_rgb(second);
	const auto blend = [weight](std::uint8_t a, std::uint8_t b) {
		return std::uint8_t(
				std::clamp(std::lround(a * weight + b * (1.0f - weight)), 0L, 255L));
	};
	return map_palette::nearest_map_color(blend(ar, br), blend(ag, bg), blend(ab, bb));
}
}
