#pragma once
#include <array>
#include <cmath>
#include <optional>
#include <vector>
namespace arnis::mapillary
{
enum class CameraModel
{
	Spherical,
	Perspective,
	Brown,
	Fisheye
};
struct Camera
{
	CameraModel model{CameraModel::Spherical};
	std::array<double, 3> centre{};
	std::array<std::array<double, 3>, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
	double width{0}, height{0}, focal{0};
	// OpenSfM camera parameters.  Perspective uses [f, k1, k2], Brown uses
	// [fx, fy, cx, cy, k1, k2, p1, p2, k3].
	std::vector<double> parameters;
};
inline bool finite(const std::array<double, 3> &v)
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline std::optional<std::array<double, 2>> project(
		const Camera &c, const std::array<double, 3> &p)
{
	if (!finite(p) || c.width <= 0 || c.height <= 0)
		return {};
	std::array<double,3> d{p[0]-c.centre[0],p[1]-c.centre[1],p[2]-c.centre[2};
	double x = d[0] * c.axes[0][0] + d[1] * c.axes[0][1] + d[2] * c.axes[0][2],
		   y = d[0] * c.axes[1][0] + d[1] * c.axes[1][1] + d[2] * c.axes[1][2],
		   z = d[0] * c.axes[2][0] + d[1] * c.axes[2][1] + d[2] * c.axes[2][2];
	if (c.model == CameraModel::Spherical) {
		constexpr double pi = 3.14159265358979323846;
		double n = std::hypot(x, y, z);
		if (n <= 1e-12)
			return {};
		return std::array<double, 2>{(std::atan2(x, z) / (2 * pi) + .5) * c.width,
				(.5 - std::asin(y / n) / pi) * c.height};
	}
	if (z <= 1e-9 || c.focal <= 0)
		return {};
	return std::array<double, 2>{
			c.width * .5 + c.focal * x / z, c.height * .5 + c.focal * y / z};
}
}
