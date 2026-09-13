#pragma once

#include "project.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace arnis::mapillary::fuse
{
inline constexpr double border_falloff_m = 1.0;
inline constexpr double border_floor = .15;

inline std::size_t reflect101(long value, std::size_t size)
{
	if (size <= 1)
		return 0;
	long period = 2 * long(size - 1), index = value % period;
	if (index < 0)
		index += period;
	return std::size_t(index >= long(size) ? period - index : index);
}
inline double grey(const projection::Rgb &pixel)
{
	return (std::int64_t(pixel[0]) * 9798 + std::int64_t(pixel[1]) * 19235 +
				   std::int64_t(pixel[2]) * 3735 + 16384) /
		   8355840.0;
}
inline std::vector<double> gradient_magnitude(
		const projection::Image &image, const std::vector<bool> *valid = nullptr)
{
	std::size_t w = image.width, h = image.height;
	std::vector<double> result(w * h);
	if (image.pixels.size() < w * h)
		return result;
	for (std::size_t y = 0; y < h; ++y)
		for (std::size_t x = 0; x < w; ++x) {
			double gx = 0, gy = 0;
			for (long dy = -1; dy <= 1; ++dy)
				for (long dx = -1; dx <= 1; ++dx) {
					double value = grey(image.pixels[reflect101(long(y) + dy, h) * w +
													 reflect101(long(x) + dx, w)]);
					int sx = dx == 0 ? 0 : (dx < 0 ? -1 : 1),
						sy = dy == 0 ? 0 : (dy < 0 ? -1 : 1);
					int wx = dy == 0 ? 2 : 1, wy = dx == 0 ? 2 : 1;
					gx += sx * wx * value;
					gy += sy * wy * value;
				}
			result[y * w + x] = std::hypot(gx, gy);
		}
	if (valid && valid->size() == result.size())
		for (std::size_t i = 0; i < result.size(); ++i)
			if (!(*valid)[i])
				result[i] = 0;
	return result;
}
inline projection::Image shift_image(const projection::Image &image, double dx, double dy)
{
	projection::Image out{
			image.width, image.height, std::vector<projection::Rgb>(image.pixels.size())};
	for (unsigned y = 0; y < image.height; ++y)
		for (unsigned x = 0; x < image.width; ++x) {
			double sx = x - dx, sy = y - dy;
			long x0 = long(std::floor(sx)), y0 = long(std::floor(sy));
			double tx = sx - x0, ty = sy - y0;
			projection::Rgb pixel{};
			for (unsigned c = 0; c < 3; ++c) {
				auto at = [&](long px, long py) {
					return px < 0 || py < 0 || px >= long(image.width) ||
										   py >= long(image.height)
								   ? 0.0
								   : image.pixels[std::size_t(py) * image.width + px][c];
				};
				pixel[c] = std::uint8_t(
						std::clamp(std::round(at(x0, y0) * (1 - tx) * (1 - ty) +
											  at(x0 + 1, y0) * tx * (1 - ty) +
											  at(x0, y0 + 1) * (1 - tx) * ty +
											  at(x0 + 1, y0 + 1) * tx * ty),
								0.0, 255.0));
			}
			out.pixels[std::size_t(y) * image.width + x] = pixel;
		}
	return out;
}
inline std::vector<double> border_weight(
		const std::vector<bool> &valid, unsigned w, unsigned h, double pixels_per_metre)
{
	std::vector<double> result(std::size_t(w) * h);
	for (unsigned y = 0; y < h; ++y)
		for (unsigned x = 0; x < w; ++x) {
			std::size_t i = std::size_t(y) * w + x;
			if (i >= valid.size() || !valid[i])
				continue;
			unsigned distance = std::min({x, y, w - 1 - x, h - 1 - y});
			double t =
					std::clamp(double(distance + 1) /
									   std::max(1.0, border_falloff_m * pixels_per_metre),
							0.0, 1.0);
			result[i] =
					border_floor +
					(1 - border_floor) * (.5 - .5 * std::cos(3.1415926535897932385 * t));
		}
	return result;
}
inline std::pair<projection::Image, std::vector<bool>> weighted_median(
		const std::vector<projection::Image> &images,
		const std::vector<std::vector<double>> &weights, double minimum_weight)
{
	if (images.empty())
		return {};
	auto result = images.front();
	std::vector<bool> valid(result.pixels.size());
	for (std::size_t i = 0; i < result.pixels.size(); ++i) {
		double total = 0;
		for (const auto &weight : weights)
			if (i < weight.size())
				total += weight[i];
		valid[i] = total >= minimum_weight;
		if (!valid[i]) {
			result.pixels[i] = {};
			continue;
		}
		for (unsigned c = 0; c < 3; ++c) {
			std::vector<std::pair<double, double>> entries;
			for (std::size_t v = 0; v < images.size(); ++v)
				entries.emplace_back(images[v].pixels[i][c],
						i < weights[v].size() ? weights[v][i] : 0);
			std::sort(entries.begin(), entries.end());
			double sum = 0;
			for (const auto &[value, weight] : entries) {
				sum += weight;
				if (sum >= total * .5) {
					result.pixels[i][c] = std::uint8_t(value);
					break;
				}
			}
		}
	}
	return {std::move(result), std::move(valid)};
}
} // namespace arnis::mapillary::fuse
