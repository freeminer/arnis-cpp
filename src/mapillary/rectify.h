#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include "project.h"
#include "pose.h"
namespace arnis::mapillary::rectify
{
inline constexpr double max_elevation_deg = 80, bottom_margin_m = 1.5,
						cloud_occlusion_margin_m = 1.5, los_inside_tolerance_m = 1,
						own_shrink_m = .3, sample_front_m = .1;
enum Occlusion : unsigned char
{
	Visible = 0,
	Outside = 1,
	Cloud = 2,
	Vegetation = 4,
	Sky = 8
};
struct Crop
{
	double s0{}, s1{}, z0{}, z1{}, pixels_per_metre{};
	unsigned width{}, height{};
};
inline Crop extent(double wall_length, double base_z, double top_z, double ppm,
		double bottom_margin = bottom_margin_m)
{
	Crop out;
	out.s1 = std::max(0., wall_length);
	out.z0 = base_z - bottom_margin;
	out.z1 = std::max(out.z0, top_z);
	out.pixels_per_metre = std::max(1., ppm);
	out.width = unsigned(std::ceil((out.s1 - out.s0) * out.pixels_per_metre));
	out.height = unsigned(std::ceil((out.z1 - out.z0) * out.pixels_per_metre));
	return out;
}
inline double choose_ppm(double distance_m, unsigned image_width, double requested,
		double min_ppm = 2, double max_ppm = 16)
{
	if (distance_m <= 0 || !image_width)
		return min_ppm;
	return std::clamp(std::min(requested, double(image_width) / (2 * distance_m)),
			min_ppm, max_ppm);
}
inline projection::Rgb bilinear(const projection::Image &image, double x, double y)
{
	projection::Rgb out{};
	if (!image.width || !image.height)
		return out;
	long x0 = long(std::floor(x)), y0 = long(std::floor(y));
	double tx = x - x0, ty = y - y0;
	auto at = [&](long px, long py) {
		px = std::clamp(px, 0L, long(image.width) - 1);
		py = std::clamp(py, 0L, long(image.height) - 1);
		return image.pixels[std::size_t(py) * image.width + px];
	};
	for (unsigned c = 0; c < 3; ++c) {
		double value =
				at(x0, y0)[c] * (1 - tx) * (1 - ty) + at(x0 + 1, y0)[c] * tx * (1 - ty) +
				at(x0, y0 + 1)[c] * (1 - tx) * ty + at(x0 + 1, y0 + 1)[c] * tx * ty;
		out[c] = std::uint8_t(std::clamp(std::round(value), 0., 255.));
	}
	return out;
}
inline std::vector<std::array<float, 2>> pixel_map(const pose::Projector &projector,
		const std::vector<std::array<double, 3>> &points)
{
	std::vector<std::array<float, 2>> result;
	result.reserve(points.size());
	for (const auto &point : points) {
		auto pixel = projector.pixel(point);
		result.push_back(pixel.value_or(std::array<float, 2>{-1, -1}));
	}
	return result;
}
inline bool usable(unsigned char flags)
{
	return flags == Visible;
}
} // namespace arnis::mapillary::rectify
