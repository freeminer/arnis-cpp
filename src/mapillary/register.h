#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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
inline void raster_line(std::vector<bool> &mask, std::size_t n, std::array<double, 2> a,
		std::array<double, 2> b)
{
	double dx = b[0] - a[0], dy = b[1] - a[1];
	unsigned steps = unsigned(std::ceil(std::max(std::abs(dx), std::abs(dy)) * 8));
	for (unsigned i = 0; i <= steps; ++i) {
		double t = steps ? double(i) / steps : 0;
		long x = std::lround(a[0] + t * dx), y = std::lround(a[1] + t * dy);
		if (x >= 0 && y >= 0 && x < long(n) && y < long(n))
			mask[std::size_t(y) * n + x] = true;
	}
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
} // namespace arnis::mapillary::registration
