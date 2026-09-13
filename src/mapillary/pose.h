#pragma once

#include "types.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace arnis::mapillary::pose
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;
inline constexpr double radial_limit_max = 6.0;
inline constexpr double radial_limit_safety = .9;

inline double dot(const Vec3 &a, const Vec3 &b)
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(const Vec3 &a)
{
	return std::sqrt(dot(a, a));
}
inline Vec3 scale(const Vec3 &a, double k)
{
	return {a[0] * k, a[1] * k, a[2] * k};
}
inline Vec3 add(const Vec3 &a, const Vec3 &b)
{
	return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline Vec3 sub(const Vec3 &a, const Vec3 &b)
{
	return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

inline Mat3 axes_from_rotation(const Vec3 &rotation)
{
	double theta = norm(rotation);
	if (theta < 1e-12)
		return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
	Vec3 k = scale(rotation, 1.0 / theta);
	double s = std::sin(theta), c = std::cos(theta);
	double skew[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
	Mat3 result{};
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			result[i][j] =
					(i == j ? 1.0 : 0.0) * c + s * skew[i][j] + (1.0 - c) * k[i] * k[j];
	return result;
}
inline Mat3 level_axes(double compass_deg, double roll_deg = 0, double pitch_deg = 0)
{
	double h = compass_deg * .017453292519943295769;
	double sh = std::sin(h), ch = std::cos(h);
	Vec3 forward{sh, ch, 0}, right{ch, -sh, 0}, down{0, 0, -1};
	double p = pitch_deg * .017453292519943295769;
	Vec3 f2 = sub(scale(forward, std::cos(p)), scale(down, std::sin(p)));
	Vec3 d2 = add(scale(down, std::cos(p)), scale(forward, std::sin(p)));
	double r = roll_deg * .017453292519943295769;
	return {add(scale(right, std::cos(r)), scale(d2, std::sin(r))),
			sub(scale(d2, std::cos(r)), scale(right, std::sin(r))), f2};
}
inline double radial_limit(CameraModel model, const std::vector<double> &parameters)
{
	double k1 = 0, k2 = 0, k3 = 0;
	if (model == CameraModel::Brown && parameters.size() >= 9) {
		k1 = parameters[4];
		k2 = parameters[5];
		k3 = parameters[8];
	} else if (parameters.size() >= 2) {
		k1 = parameters[1];
		if (parameters.size() >= 3)
			k2 = parameters[2];
	}
	for (unsigned i = 0; i < 1201; ++i) {
		double r = radial_limit_max * i / 1200.0;
		double x = model == CameraModel::Fisheye ? std::atan(r) : r, x2 = x * x;
		if (1 + 3 * k1 * x2 + 5 * k2 * x2 * x2 + 7 * k3 * x2 * x2 * x2 <= 0)
			return radial_limit_safety * r;
	}
	return radial_limit_max;
}

struct Projection
{
	double u{std::numeric_limits<double>::quiet_NaN()};
	double v{std::numeric_limits<double>::quiet_NaN()};
	double length_m{};
	bool valid() const { return std::isfinite(u) && std::isfinite(v); }
	bool inside_image() const { return valid() && u >= 0 && u < 1 && v >= 0 && v < 1; }
};

class Projector
{
	Camera camera_;
	double radial_limit_squared_;
	double big_;
	std::pair<double, double> distort(double x, double y) const
	{
		const auto &p = camera_.parameters;
		double r2 = x * x + y * y;
		if (camera_.model == CameraModel::Brown && p.size() >= 9) {
			double d = 1 + p[4] * r2 + p[5] * r2 * r2 + p[8] * r2 * r2 * r2;
			return {p[0] * (x * d + 2 * p[6] * x * y + p[7] * (r2 + 2 * x * x)) + p[2],
					p[1] * (y * d + p[6] * (r2 + 2 * y * y) + 2 * p[7] * x * y) + p[3]};
		}
		double f = p.empty() ? (camera_.focal > 0 ? camera_.focal / big_ : .85) : p[0];
		double k1 = p.size() > 1 ? p[1] : 0, k2 = p.size() > 2 ? p[2] : 0;
		if (camera_.model == CameraModel::Fisheye) {
			double r = std::sqrt(r2), theta = std::atan(r), t2 = theta * theta;
			double scale = r > 1e-12 ? theta * (1 + k1 * t2 + k2 * t2 * t2) / r : 1;
			return {f * x * scale, f * y * scale};
		}
		double d = 1 + k1 * r2 + k2 * r2 * r2;
		return {f * x * d, f * y * d};
	}

public:
	explicit Projector(const Camera &camera) :
			camera_(camera),
			radial_limit_squared_(
					camera.model == CameraModel::Spherical
							? std::numeric_limits<double>::infinity()
							: std::pow(radial_limit(camera.model, camera.parameters), 2)),
			big_(std::max({camera.width, camera.height, 1.0}))
	{
	}
	bool spherical() const { return camera_.model == CameraModel::Spherical; }
	Vec3 to_camera(const Vec3 &point) const
	{
		Vec3 d = sub(point, camera_.centre);
		return {dot(d, camera_.axes[0]), dot(d, camera_.axes[1]),
				dot(d, camera_.axes[2])};
	}
	Projection project(const Vec3 &point) const
	{
		Vec3 c = to_camera(point);
		double length = norm(c);
		if (spherical())
			return {.5 + std::atan2(c[0], c[2]) / 6.2831853071795864769,
					.5 + std::asin(
								 std::clamp(c[1] / std::max(length, 1e-12), -1.0, 1.0)) /
									3.1415926535897932385,
					length};
		if (c[2] <= 1e-6)
			return {{}, {}, length};
		double x = c[0] / c[2], y = c[1] / c[2];
		if (x * x + y * y >= radial_limit_squared_)
			return {{}, {}, length};
		auto [xi, yi] = distort(x, y);
		return {.5 + xi * big_ / std::max(camera_.width, 1.0),
				.5 + yi * big_ / std::max(camera_.height, 1.0), length};
	}
	std::optional<std::array<float, 2>> pixel(const Vec3 &point) const
	{
		auto p = project(point);
		if (!p.valid() || !camera_.width || !camera_.height)
			return {};
		if (spherical()) {
			double x = std::fmod(p.u * camera_.width - .5, camera_.width);
			if (x < 0)
				x += camera_.width;
			return std::array<float, 2>{
					float(x), float(std::clamp(p.v * camera_.height - .5, 0.0,
									  camera_.height - 1))};
		}
		return p.inside_image()
					   ? std::optional<std::array<float, 2>>{{float(p.u * camera_.width -
																	  .5),
								 float(p.v * camera_.height - .5)}}
					   : std::nullopt;
	}
};
} // namespace arnis::mapillary::pose
