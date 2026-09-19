#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>
#include <vector>

namespace arnis::mapillary::projection
{
using Vec3 = std::array<double, 3>;
using Rgb = std::array<std::uint8_t, 3>;

struct CameraPose
{
	Vec3 centre{};
	double compass_angle{};
	std::optional<std::array<Vec3, 3>> axes;

	static CameraPose level(double x, double y, double z, double compass_deg)
	{
		return {{x, y, z}, compass_deg, {}};
	}
	static CameraPose oriented(double x, double y, double z, double compass_deg,
			const Vec3 &rotation, double bearing_offset_deg)
	{
		auto r = rodrigues(rotation);
		for (auto &v : r)
			v = normalize(turn_about_up(enu_to_world(v), bearing_offset_deg));
		return {{x, y, z}, compass_deg, r};
	}

private:
	static double dot(const Vec3 &a, const Vec3 &b)
	{
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}
	static Vec3 normalize(const Vec3 &v)
	{
		double n = std::sqrt(dot(v, v));
		return n < 1e-12 ? Vec3{0, 0, 1} : Vec3{v[0] / n, v[1] / n, v[2] / n};
	}
	static std::array<Vec3, 3> rodrigues(const Vec3 &r)
	{
		double theta = std::sqrt(dot(r, r));
		if (theta < 1e-12)
			return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
		Vec3 k{r[0] / theta, r[1] / theta, r[2] / theta};
		double s = std::sin(theta), c = std::cos(theta);
		double cross[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
		std::array<Vec3, 3> m{};
		for (unsigned i = 0; i < 3; ++i)
			for (unsigned j = 0; j < 3; ++j)
				m[i][j] = (i == j ? 1.0 : 0.0) * c + s * cross[i][j] +
						  (1.0 - c) * k[i] * k[j];
		return m;
	}
	static Vec3 enu_to_world(const Vec3 &v) { return {v[0], v[2], -v[1]}; }
	static Vec3 turn_about_up(const Vec3 &v, double degrees)
	{
		if (std::abs(degrees) < std::numeric_limits<double>::epsilon())
			return v;
		double r = degrees * 0.017453292519943295769;
		double s = std::sin(r), c = std::cos(r);
		return {v[0] * c - v[2] * s, v[1], v[2] * c + v[0] * s};
	}

	friend std::array<double, 2> project(const CameraPose &, const Vec3 &);
};

inline double bearing_deg(double dx, double dz)
{
	return std::atan2(dx, -dz) * 57.295779513082320876;
}
inline double wrap180(double degree)
{
	double d = std::fmod(degree + 180.0, 360.0);
	if (d < 0)
		d += 360.0;
	return d - 180.0;
}
inline std::array<double, 2> project(const CameraPose &camera, const Vec3 &point)
{
	Vec3 d{point[0] - camera.centre[0], point[1] - camera.centre[1],
			point[2] - camera.centre[2]};
	if (camera.axes) {
		const auto &[right, down, forward] = *camera.axes;
		auto dot = [](const Vec3 &a, const Vec3 &b) {
			return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		};
		double cx = dot(d, right), cy = dot(d, down), cz = dot(d, forward);
		double u = .5 + std::atan2(cx, cz) / 6.2831853071795864769;
		double len = std::max(1e-12, std::sqrt(cx * cx + cy * cy + cz * cz));
		return {u,
				.5 + std::asin(std::clamp(cy / len, -1.0, 1.0)) / 3.1415926535897932385};
	}
	double rel = wrap180(bearing_deg(d[0], d[2]) - camera.compass_angle);
	double horizontal = std::hypot(d[0], d[2]);
	return {.5 + rel / 360.0,
			.5 - std::atan2(d[1], horizontal) * 57.295779513082320876 / 180.0};
}

// Rust's projection API accepts scalar world coordinates at call sites.  Keep
// that form as a forwarding overload so facade code does not need temporary
// arrays (and so the C++ API has the same coordinate convention as project.rs).
inline std::array<double, 2> project(
		const CameraPose &camera, double x, double y, double z)
{
	return project(camera, Vec3{x, y, z});
}

struct Image
{
	unsigned width{}, height{};
	std::vector<Rgb> pixels;
	std::optional<Rgb> sample(double u, double v) const
	{
		if (!(v >= 0 && v < 1) || !std::isfinite(u) || !width || !height ||
				pixels.size() < std::size_t(width) * height)
			return {};
		double wrapped = u - std::floor(u);
		unsigned x = unsigned(wrapped * width) % width;
		unsigned y = std::min(unsigned(v * height), height - 1);
		return pixels[std::size_t(y) * width + x];
	}
};

inline std::optional<Rgb> sample_pixel(const Image &image, double u, double v)
{
	return image.sample(u, v);
}
enum class Reject
{
	TooDark,
	TooBright,
	Vegetation,
	Sky
};
inline std::optional<Rgb> classify(const Rgb &color, Reject *why = nullptr)
{
	int r = color[0], g = color[1], b = color[2],
		luma = (299 * r + 587 * g + 114 * b) / 1000;
	auto reject = [&](Reject reason) -> std::optional<Rgb> {
		if (why)
			*why = reason;
		return {};
	};
	if (luma < 30)
		return reject(Reject::TooDark);
	if (luma > 245)
		return reject(Reject::TooBright);
	if (g > r + 12 && g > b + 12)
		return reject(Reject::Vegetation);
	if (b > r + 25 && b > g + 15 && luma > 150)
		return reject(Reject::Sky);
	return color;
}
inline std::optional<Rgb> median_color(const std::vector<Rgb> &samples)
{
	if (samples.empty())
		return {};
	Rgb result{};
	for (unsigned c = 0; c < 3; ++c) {
		std::vector<std::uint8_t> values;
		values.reserve(samples.size());
		for (const auto &s : samples)
			values.push_back(s[c]);
		std::nth_element(
				values.begin(), values.begin() + values.size() / 2, values.end());
		result[c] = values[values.size() / 2];
	}
	return result;
}
} // namespace arnis::mapillary::projection
