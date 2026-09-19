#pragma once
#include "project.h"
#include <cmath>
#include <cstddef>
#include <queue>
#include <string>
#include <vector>
namespace arnis::mapillary::openings
{
inline constexpr std::size_t pixels_per_metre = 8;
inline constexpr std::size_t subcell_pixels = 4;
inline constexpr std::size_t reference_window_subcells = 7;
inline constexpr double dark_threshold = .12;
inline constexpr double chroma_threshold = .045;
inline constexpr double min_width_m = .5, min_height_m = .4, min_fill = .45;
inline constexpr double door_min_width_m = .8, door_max_width_m = 2.2;
inline constexpr double door_min_height_m = 1.5, shop_min_height_m = 1.2;
inline constexpr double shop_max_height_m = 5.0, plinth_max_height_m = 2.0;
inline constexpr double base_tolerance_m = 1.5;
inline constexpr double large_width_m = 4.5, large_height_m = 3.5;
inline constexpr double split_width_m = 2.5, split_height_m = 2.8;
inline constexpr double narrow_width_m = 1.8, deep_glass = .32;
inline constexpr double faint_contrast = .10, shadow_chroma = .025;
inline constexpr double shadow_contrast = .22, lively_lightness_std = .07;
inline constexpr double floor_gap_m = 1.2, bay_gap_m = .5;
inline constexpr double unknown_valid = .7, minimum_evidence = .12;
inline constexpr double rhythm_tolerance_m = .35;
inline constexpr std::size_t rhythm_min_windows = 4;
inline constexpr double rhythm_min_support = .6, extend_min_support = .5;
inline constexpr double extend_evidence = .03;
inline constexpr bool template_pass = false;
inline constexpr double template_score = .70;
inline constexpr std::size_t PPM = pixels_per_metre;
inline constexpr bool TEMPLATE_PASS = template_pass;
inline constexpr double TM_SCORE = template_score;
inline constexpr unsigned char cls_wall = 255, cls_window = 192, cls_door = 128,
							   cls_unknown = 64, cls_nodata = 0;
enum class Kind
{
	Window,
	Door,
	Shop,
	Rejected
};
inline const char *kind_name(Kind kind)
{
	switch (kind) {
	case Kind::Window:
		return "window";
	case Kind::Door:
		return "door";
	case Kind::Shop:
		return "shop";
	case Kind::Rejected:
		return "rejected";
	}
	return "rejected";
}
inline Kind parse_kind(const std::string &value)
{
	if (value == "window")
		return Kind::Window;
	if (value == "door")
		return Kind::Door;
	if (value == "shop")
		return Kind::Shop;
	return Kind::Rejected;
}
struct Rect
{
	Kind kind{Kind::Rejected};
	double x0{}, y0{}, x1{}, y1{};
	const char *reason{};
	double contrast{}, chroma{}, std_l{}, fill{1};
	std::size_t columns{}, rows{};
	double width_m() const { return (x1 - x0) / pixels_per_metre; }
	double height_m() const { return (y1 - y0) / pixels_per_metre; }
	bool valid() const
	{
		return x1 > x0 && y1 > y0 && std::isfinite(x0) && std::isfinite(y0) &&
			   std::isfinite(x1) && std::isfinite(y1);
	}
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
	bool consistent() const
	{
		const auto cells = rows * columns;
		return colours.size() == cells && classes.size() == cells &&
			   evidence.size() == cells && mask.size() == cells;
	}
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
	const double local_lightness = reference / 255.0;
	return light < reference * (1.0 - dark_threshold * local_lightness) ||
		   std::abs(int(pixel[2]) - int(wall[2])) > chroma_threshold * 255.0;
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
	for (auto &rect : out.rectangles) {
		if (rect.width_m() < min_width_m || rect.height_m() < min_height_m ||
				rect.fill < min_fill) {
			rect.kind = Kind::Rejected;
			rect.reason = "small";
			continue;
		}
		const bool at_base =
				rect.y1 >= double(rows) - base_tolerance_m * pixels_per_metre;
		if (at_base && rect.height_m() >= door_min_height_m &&
				rect.width_m() >= door_min_width_m && rect.width_m() <= door_max_width_m)
			rect.kind = Kind::Door;
		else if (at_base && rect.height_m() >= shop_min_height_m &&
				 rect.height_m() <= shop_max_height_m &&
				 rect.width_m() > door_max_width_m)
			rect.kind = Kind::Shop;
		else
			rect.kind = Kind::Window;
		const auto cls = rect.kind == Kind::Door ? cls_door : cls_window;
		if (rect.kind != Kind::Rejected)
			for (std::size_t y = std::size_t(rect.y0);
					y < std::size_t(rect.y1) && y < rows; ++y)
				for (std::size_t x = std::size_t(rect.x0);
						x < std::size_t(rect.x1) && x < columns; ++x)
					if (out.mask[y * columns + x])
						out.classes[y * columns + x] = cls;
	}
	return out;
}
} // namespace arnis::mapillary::openings
