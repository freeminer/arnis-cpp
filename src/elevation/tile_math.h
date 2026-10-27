#pragma once

#include "providers.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <utility>

namespace arnis::elevation::providers::tile_math
{
inline constexpr double SMOOTH_PIXEL_CELLS = 1.5;
inline constexpr double EARTH_RADIUS_M = 6'378'137.0;

inline unsigned smooth_below(const GeoBBox &bbox, unsigned zoom, std::size_t width,
		std::size_t height, unsigned tile_pixels)
{
	const auto [width_m, height_m] = bbox_dimensions_m(bbox);
	const double cell_m = std::max(
			0.05, std::min(width_m / static_cast<double>(width > 1 ? width - 1 : 1),
						  height_m / static_cast<double>(height > 1 ? height - 1 : 1)));
	const double mid_lat = (bbox.min_lat + bbox.max_lat) * 0.5;
	const double circumference = 2.0 * std::numbers::pi * EARTH_RADIUS_M;
	for (unsigned z = 0; z <= zoom; ++z) {
		const double metres_per_pixel =
				circumference * std::abs(std::cos(mid_lat * std::numbers::pi / 180.0)) /
				(static_cast<double>(tile_pixels) * std::ldexp(1.0, static_cast<int>(z)));
		if (metres_per_pixel <= SMOOTH_PIXEL_CELLS * cell_m)
			return z;
	}
	return zoom + 1;
}

inline double in_tile(double global, double origin, unsigned tile_pixels)
{
	const double local = global - origin;
	return local >= tile_pixels ? static_cast<double>(tile_pixels) - 0.5 : local;
}

// Include every tile touched by the spline's outer taps. Sampling positions
// can sit just inside a tile boundary, so extending only the far side of the
// chunk misses valid neighbour pixels at its near edge.
inline std::pair<unsigned, unsigned> tap_tile_range(
		unsigned zoom, double lo, double hi, unsigned tile_pixels)
{
	constexpr double tap_reach_px = 3.0;
	const auto tile_count = std::int64_t{1} << zoom;
	const double world_pixels = static_cast<double>(tile_count) * tile_pixels;
	const auto tile = [&](double norm, double reach) {
		const auto index = static_cast<std::int64_t>(
				std::floor((norm * world_pixels + reach) / tile_pixels));
		return static_cast<unsigned>(std::clamp<std::int64_t>(index, 0, tile_count - 1));
	};
	return {tile(std::min(lo, hi), -tap_reach_px), tile(std::max(lo, hi), tap_reach_px)};
}

inline std::array<double, 4> bspline_weights(double t)
{
	const double s = 1.0 - t;
	const double t2 = t * t;
	const double t3 = t2 * t;
	return {s * s * s / 6.0, (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0,
			(-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0, t3 / 6.0};
}

template <typename Pixel>
double sample_pixels(double px, double py, bool smooth, Pixel &&pixel)
{
	px -= 0.5;
	py -= 0.5;
	const double fx = std::floor(px), fy = std::floor(py);
	const double dx = px - fx, dy = py - fy;
	const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
	if (smooth) {
		const auto wx = bspline_weights(dx), wy = bspline_weights(dy);
		double sum = 0.0, weight = 0.0;
		for (std::size_t j = 0; j < wy.size(); ++j)
			for (std::size_t i = 0; i < wx.size(); ++i) {
				const double value =
						pixel(x0 - 1 + static_cast<int>(i), y0 - 1 + static_cast<int>(j));
				if (!std::isfinite(value))
					continue;
				const double w = wx[i] * wy[j];
				sum += w * value;
				weight += w;
			}
		if (weight > 0.0)
			return sum / weight;
	}
	return blend_finite_samples(pixel(x0, y0), pixel(x0 + 1, y0), pixel(x0, y0 + 1),
			pixel(x0 + 1, y0 + 1), dx, dy);
}
} // namespace arnis::elevation::providers::tile_math
