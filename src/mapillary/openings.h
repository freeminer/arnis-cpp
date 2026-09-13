#pragma once
#include "project.h"
#include <cstddef>
#include <queue>
#include <vector>
namespace arnis::mapillary::openings
{
inline constexpr std::size_t pixels_per_metre = 8;
inline constexpr unsigned char cls_wall = 255, cls_window = 192, cls_door = 128,
							   cls_unknown = 64, cls_nodata = 0;
enum class Kind
{
	Window,
	Door,
	Shop,
	Rejected
};
struct Rect
{
	Kind kind{Kind::Rejected};
	double x0{}, y0{}, x1{}, y1{};
	const char *reason{};
	double contrast{}, chroma{}, std_l{}, fill{1};
	std::size_t columns{}, rows{};
	double width_m() const { return (x1 - x0) / pixels_per_metre; }
	double height_m() const { return (y1 - y0) / pixels_per_metre; }
};
struct Texture
{
	projection::Image image;
	std::vector<bool> valid;
	bool consistent() const
	{
		return image.pixels.size() == std::size_t(image.width) * image.height &&
			   valid.size() == image.pixels.size();
	}
};
struct Result
{
	std::size_t rows{}, columns{};
	std::vector<unsigned char> classes;
	std::vector<projection::Rgb> colours;
	std::vector<double> evidence;
	std::vector<Rect> rectangles;
	std::vector<bool> mask;
};
inline bool opening_candidate(const projection::Rgb &pixel, const projection::Rgb &wall)
{
	int light =
				[](const projection::Rgb &p) {
					return (299 * p[0] + 587 * p[1] + 114 * p[2]) / 1000;
				}(pixel),
		reference = [](const projection::Rgb &p) {
			return (299 * p[0] + 587 * p[1] + 114 * p[2]) / 1000;
		}(wall);
	return light < reference * .88 || std::abs(int(pixel[2]) - int(wall[2])) > 35;
}
inline std::vector<Rect> components(
		const std::vector<bool> &mask, std::size_t width, std::size_t height)
{
	std::vector<Rect> out;
	std::vector<bool> seen(mask.size());
	for (std::size_t start = 0; start < mask.size(); ++start)
		if (mask[start] && !seen[start]) {
			std::queue<std::size_t> todo;
			todo.push(start);
			seen[start] = true;
			std::size_t x0 = width, y0 = height, x1 = 0, y1 = 0, n = 0;
			while (!todo.empty()) {
				auto i = todo.front();
				todo.pop();
				auto x = i % width, y = i / width;
				x0 = std::min(x0, x);
				y0 = std::min(y0, y);
				x1 = std::max(x1, x);
				y1 = std::max(y1, y);
				++n;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx) {
						long nx = long(x) + dx, ny = long(y) + dy;
						if (nx >= 0 && ny >= 0 && nx < long(width) && ny < long(height)) {
							auto ni = std::size_t(ny) * width + nx;
							if (mask[ni] && !seen[ni]) {
								seen[ni] = true;
								todo.push(ni);
							}
						}
					}
			}
			Rect rect;
			rect.kind = Kind::Rejected;
			rect.x0 = x0;
			rect.y0 = y0;
			rect.x1 = x1 + 1;
			rect.y1 = y1 + 1;
			rect.fill = double(n) / ((x1 - x0 + 1) * (y1 - y0 + 1));
			rect.columns = x1 - x0 + 1;
			rect.rows = y1 - y0 + 1;
			out.push_back(rect);
		}
	return out;
}
inline Result classify_grid(const std::vector<projection::Rgb> &colours,
		std::size_t columns, std::size_t rows)
{
	Result out;
	out.columns = columns;
	out.rows = rows;
	out.colours = colours;
	out.classes.assign(colours.size(), cls_wall);
	out.evidence.assign(colours.size(), 0);
	out.mask.assign(colours.size(), false);
	if (colours.size() != columns * rows)
		return out;
	for (std::size_t r = 0; r < rows; ++r)
		for (std::size_t c = 0; c < columns; ++c) {
			std::array<unsigned, 3> sum{};
			std::size_t n = 0;
			for (long y = long(r) - 1; y <= long(r) + 1; ++y)
				for (long x = long(c) - 1; x <= long(c) + 1; ++x)
					if (x >= 0 && y >= 0 && x < long(columns) && y < long(rows)) {
						auto i = std::size_t(y) * columns + x;
						for (unsigned k = 0; k < 3; ++k)
							sum[k] += colours[i][k];
						++n;
					}
			projection::Rgb wall{std::uint8_t(sum[0] / n), std::uint8_t(sum[1] / n),
					std::uint8_t(sum[2] / n)};
			auto i = r * columns + c;
			if (opening_candidate(colours[i], wall)) {
				out.mask[i] = true;
				out.evidence[i] = 1;
				out.classes[i] = cls_window;
			}
		}
	out.rectangles = components(out.mask, columns, rows);
	for (auto &rect : out.rectangles)
		rect.kind = Kind::Window;
	return out;
}
} // namespace arnis::mapillary::openings
