#pragma once

#include "imgops.h"
#include "sfm.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace arnis::mapillary::registration
{
inline constexpr double dt_far = 1e5;
struct OutlineDT
{
	std::array<double, 2> centre{}, origin{};
	double resolution{}, size_m{};
	std::size_t size{};
	std::vector<float> values;
	bool contains(const std::array<double, 2> &point) const
	{
		double x = (point[0] - origin[0]) / resolution,
			   y = (point[1] - origin[1]) / resolution;
		return x >= 0 && y >= 0 && x <= double(size - 1) && y <= double(size - 1);
	}
	float distance(const std::array<double, 2> &point) const
	{
		if (!contains(point))
			return dt_far;
		double x = (point[0] - origin[0]) / resolution,
			   y = (point[1] - origin[1]) / resolution;
		std::size_t x0 = std::size_t(std::floor(x)), y0 = std::size_t(std::floor(y)),
					x1 = std::min(x0 + 1, size - 1), y1 = std::min(y0 + 1, size - 1);
		double tx = x - x0, ty = y - y0;
		double top = values[y0 * size + x0] * (1 - tx) + values[y0 * size + x1] * tx,
			   bottom = values[y1 * size + x0] * (1 - tx) + values[y1 * size + x1] * tx;
		return float(top * (1 - ty) + bottom * ty);
	}
};
inline std::vector<double> envelope(const std::vector<double> &f)
{
	std::size_t n = f.size();
	std::vector<double> out(n);
	if (!n)
		return out;
	std::vector<std::size_t> v(n);
	std::vector<double> z(n + 1);
	std::size_t k = 0;
	z[0] = -std::numeric_limits<double>::infinity();
	z[1] = std::numeric_limits<double>::infinity();
	for (std::size_t q = 1; q < n; ++q) {
		if (!std::isfinite(f[q]))
			continue;
		for (;;) {
			auto p = v[k];
			double s = std::isfinite(f[p])
							   ? ((f[q] + double(q * q)) - (f[p] + double(p * p))) /
										 (2.0 * double(q) - 2.0 * double(p))
							   : -std::numeric_limits<double>::infinity();
			if (s <= z[k] && k) {
				--k;
				continue;
			}
			if (s <= z[k]) {
				v[0] = q;
				z[0] = -std::numeric_limits<double>::infinity();
				z[1] = std::numeric_limits<double>::infinity();
			} else {
				++k;
				v[k] = q;
				z[k] = s;
				z[k + 1] = std::numeric_limits<double>::infinity();
			}
			break;
		}
	}
	k = 0;
	for (std::size_t q = 0; q < n; ++q) {
		while (z[k + 1] < double(q))
			++k;
		auto p = v[k];
		out[q] = std::isfinite(f[p]) ? std::pow(double(q) - double(p), 2) + f[p]
									 : std::numeric_limits<double>::infinity();
	}
	return out;
}
inline std::vector<double> edt(const std::vector<bool> &mask, std::size_t n)
{
	std::vector<double> work(n * n), out(n * n), line(n);
	for (std::size_t i = 0; i < work.size(); ++i)
		work[i] = mask[i] ? 0 : std::numeric_limits<double>::infinity();
	for (std::size_t x = 0; x < n; ++x) {
		for (std::size_t y = 0; y < n; ++y)
			line[y] = work[y * n + x];
		auto d = envelope(line);
		for (std::size_t y = 0; y < n; ++y)
			out[y * n + x] = d[y];
	}
	for (std::size_t y = 0; y < n; ++y) {
		std::copy_n(out.begin() + y * n, n, line.begin());
		auto d = envelope(line);
		std::copy(d.begin(), d.end(), out.begin() + y * n);
	}
	return out;
}
inline bool clip_line(std::int64_t width, std::int64_t height,
		std::array<std::int64_t, 2> &a, std::array<std::int64_t, 2> &b)
{
	if (width <= 0 || height <= 0)
		return false;
	const auto right = width - 1, bottom = height - 1;
	auto code = [&](std::int64_t x, std::int64_t y) {
		return int(x < 0) + int(x > right) * 2 + int(y < 0) * 4 + int(y > bottom) * 8;
	};
	auto x1 = a[0], y1 = a[1], x2 = b[0], y2 = b[1];
	int c1 = code(x1, y1), c2 = code(x2, y2);
	if ((c1 & c2) == 0 && (c1 | c2) != 0) {
		if (c1 & 12) {
			const auto y = c1 < 8 ? 0 : bottom;
			if (y2 == y1)
				return false;
			x1 += (y - y1) * (x2 - x1) / (y2 - y1);
			y1 = y;
			c1 = int(x1 < 0) + int(x1 > right) * 2;
		}
		if (c2 & 12) {
			const auto y = c2 < 8 ? 0 : bottom;
			if (y1 == y2)
				return false;
			x2 += (y - y2) * (x1 - x2) / (y1 - y2);
			y2 = y;
			c2 = int(x2 < 0) + int(x2 > right) * 2;
		}
		if ((c1 & c2) == 0 && (c1 | c2) != 0) {
			if (c1) {
				const auto x = c1 == 1 ? 0 : right;
				if (x2 == x1)
					return false;
				y1 += (x - x1) * (y2 - y1) / (x2 - x1);
				x1 = x;
				c1 = 0;
			}
			if (c2) {
				const auto x = c2 == 1 ? 0 : right;
				if (x1 == x2)
					return false;
				y2 += (x - x2) * (y1 - y2) / (x1 - x2);
				x2 = x;
				c2 = 0;
			}
		}
	}
	if ((c1 | c2) != 0)
		return false;
	a = {x1, y1};
	b = {x2, y2};
	return true;
}

// Match cv::polylines(LINE_8, shift=3): clipping and fixed-point DDA matter
// because the distance transform is sampled bilinearly afterward.
inline void draw_line(std::vector<bool> &mask, std::size_t n,
		std::array<std::int64_t, 2> a, std::array<std::int64_t, 2> b)
{
	constexpr std::int64_t shift = 16, one = std::int64_t{1} << shift;
	constexpr std::int64_t poly_scale = std::int64_t{1} << (shift - 3);
	a[0] *= poly_scale;
	a[1] *= poly_scale;
	b[0] *= poly_scale;
	b[1] *= poly_scale;
	const auto scaled = static_cast<std::int64_t>(n) << shift;
	if (!clip_line(scaled, scaled, a, b))
		return;
	auto dx = b[0] - a[0], dy = b[1] - a[1];
	const std::int64_t j = dx < 0 ? -1 : 0;
	const auto ax = (dx ^ j) - j;
	const std::int64_t i = dy < 0 ? -1 : 0;
	const auto ay = (dy ^ i) - i;
	std::int64_t x_step, y_step, count;
	if (ax > ay) {
		dy = (dy ^ j) - j;
		if (j != 0)
			std::swap(a, b);
		x_step = one;
		y_step = (dy * one) / (ax | 1);
		count = (b[0] - a[0]) >> shift;
	} else {
		dx = (dx ^ i) - i;
		if (i != 0)
			std::swap(a, b);
		x_step = (dx * one) / (ay | 1);
		y_step = one;
		count = (b[1] - a[1]) >> shift;
	}
	const auto half = one >> 1;
	const std::array<std::int64_t, 2> end{(b[0] + half) >> shift, (b[1] + half) >> shift};
	a[0] += half;
	a[1] += half;
	auto set = [&](std::int64_t x, std::int64_t y) {
		if (x >= 0 && y >= 0 && x < static_cast<std::int64_t>(n) &&
				y < static_cast<std::int64_t>(n))
			mask[static_cast<std::size_t>(y) * n + static_cast<std::size_t>(x)] = true;
	};
	for (std::int64_t step = 0; step <= count; ++step) {
		set(a[0] >> shift, a[1] >> shift);
		a[0] += x_step;
		a[1] += y_step;
	}
	set(end[0], end[1]);
}

inline void raster_line(std::vector<bool> &mask, std::size_t n, std::array<double, 2> a,
		std::array<double, 2> b)
{
	const auto fixed = [](double value) {
		return static_cast<std::int64_t>(imgops::round_half_even(value * 8.0));
	};
	draw_line(mask, n, {fixed(a[0]), fixed(a[1])}, {fixed(b[0]), fixed(b[1])});
}
inline OutlineDT outline_dt(const std::vector<std::vector<std::array<double, 2>>> &rings,
		std::array<double, 2> centre, double size_m, double resolution)
{
	OutlineDT result;
	result.centre = centre;
	result.resolution = resolution;
	result.size_m = size_m;
	result.size = std::size_t(std::ceil(size_m / resolution)) + 1;
	result.origin = {centre[0] - .5 * size_m, centre[1] - .5 * size_m};
	std::vector<bool> mask(result.size * result.size);
	for (const auto &ring : rings)
		if (ring.size() > 1)
			for (std::size_t i = 0; i < ring.size(); ++i) {
				auto a = ring[(i + ring.size() - 1) % ring.size()], b = ring[i];
				a = {(a[0] - result.origin[0]) / resolution,
						(a[1] - result.origin[1]) / resolution};
				b = {(b[0] - result.origin[0]) / resolution,
						(b[1] - result.origin[1]) / resolution};
				raster_line(mask, result.size, a, b);
			}
	auto squares = edt(mask, result.size);
	result.values.resize(squares.size());
	for (std::size_t i = 0; i < squares.size(); ++i)
		result.values[i] = float(std::sqrt(squares[i]) * resolution);
	return result;
}
inline std::array<double, 3> search_shift(const OutlineDT &dt,
		const std::vector<std::array<double, 2>> &points, double range_m, double step_m)
{
	std::array<double, 3> best{0, 0, std::numeric_limits<double>::infinity()};
	long steps = long(std::llround(range_m / step_m));
	for (long y = -steps; y <= steps; ++y)
		for (long x = -steps; x <= steps; ++x) {
			double cost = 0;
			for (auto p : points) {
				double d = std::min<double>(
						3, dt.distance({p[0] + x * step_m, p[1] + y * step_m}));
				cost += d * d;
			}
			cost /= std::max<std::size_t>(1, points.size());
			if (cost < best[2])
				best = {x * step_m, y * step_m, cost};
		}
	return best;
}

inline constexpr std::size_t reg_max_points = 4000;
inline constexpr std::size_t reg_min_points = 100;
inline constexpr double cost_cap_m = 3.0;
inline constexpr double inlier_m = 1.0;
inline constexpr double distinct_min_m = 4.0;
inline constexpr double max_shift_m = 12.0;
inline constexpr double global_margin_m = 50.0;
inline constexpr double global_range_m = 8.0;
inline constexpr double global_step_m = 0.25;
inline constexpr std::size_t global_max_points = 6000;
inline constexpr double global_min_fraction = 0.08;
inline constexpr double global_min_ratio = 2.0;

enum class ThetaMode
{
	Auto,
	Never,
	Always
};

struct Registration
{
	std::string pano_id;
	RegResult result;
	std::optional<std::array<double, 3>> second_best;
};

std::vector<std::array<double, 3>> facade_band(const sfm::Cluster &cluster,
		const std::array<double, 3> &centre, double ground_z, double radius_m,
		const std::array<double, 2> &band);
OutlineDT dt_for_cameras(const std::vector<Building> &buildings,
		const std::unordered_map<std::string, Camera> &cameras, const Params &params);
std::tuple<double, double, double, std::vector<std::array<double, 3>>> search_shift(
		const std::vector<std::array<double, 2>> &points, const OutlineDT &dt,
		const std::array<double, 2> &centre, const Params &params, double theta_deg);
double inlier_share(const std::vector<std::array<double, 2>> &points, const OutlineDT &dt,
		double dx, double dy, double theta_deg, const std::array<double, 2> &centre);
Registration register_band(const std::string &pano_id,
		const std::vector<std::array<double, 2>> &band, std::size_t band_points,
		const std::array<double, 2> &centre, const OutlineDT &dt, const Params &params,
		std::optional<std::array<double, 3>> global_shift = std::nullopt,
		ThetaMode theta_mode = ThetaMode::Auto);
Registration register_pano(const Camera &camera, const sfm::Cluster *cluster,
		const OutlineDT &dt, const Params &params,
		std::optional<std::array<double, 3>> global_shift = std::nullopt,
		ThetaMode theta_mode = ThetaMode::Auto);
Registration apply_global_fallback(const Registration &local, const Camera &camera,
		const sfm::Cluster &cluster, const OutlineDT &dt, const Params &params,
		std::optional<std::array<double, 3>> global_shift);
std::optional<std::array<double, 3>> register_cluster_global(const sfm::Cluster &cluster,
		const std::vector<Building> &buildings, const std::array<double, 4> &bbox_xy,
		const Params &params, std::optional<double> ground_z = std::nullopt,
		const OutlineDT *dt = nullptr);
Camera apply_registration(const Camera &camera, const RegResult &registration);
} // namespace arnis::mapillary::registration
