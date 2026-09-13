#pragma once
#include "types.h"
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
namespace arnis::mapillary
{
struct Wall
{
	std::array<double, 3> a{}, b{};
	double base_z{0}, height_m{0};
};
inline std::optional<std::array<double, 2>> wall_tangent(const Wall &w)
{
	double x = w.b[0] - w.a[0], y = w.b[1] - w.a[1], n = std::hypot(x, y);
	return n > 1e-9 ? std::optional<std::array<double, 2>>{{x / n, y / n}} : std::nullopt;
}
inline std::optional<std::array<double, 2>> wall_outward(const Wall &w)
{
	auto t = wall_tangent(w);
	return t ? std::optional<std::array<double, 2>>{{(*t)[1], -(*t)[0]}} : std::nullopt;
}
inline std::array<double, 3> wall_point(const Wall &w, double s, double h)
{
	auto t = wall_tangent(w).value_or(std::array<double, 2>{0, 0});
	return {w.a[0] + t[0] * s, w.a[1] + t[1] * s, w.base_z + h};
}
inline bool faces_camera(const Wall &w, const Camera &c)
{
	auto n = wall_outward(w);
	if (!n)
		return false;
	return (c.centre[0] - w.a[0]) * (*n)[0] + (c.centre[1] - w.a[1]) * (*n)[1] > 0;
}
inline std::optional<std::array<double, 2>> project_wall_point(
		const Camera &c, const Wall &w, double s, double h)
{
	return faces_camera(w, c) ? project(c, wall_point(w, s, h)) : std::nullopt;
}

// WGS84 conversion used by Rust when OpenSfM's topocentric cloud coordinates
// are brought into the map frame.  Keeping this ellipsoidal path avoids the
// metre-scale drift caused by using a spherical shortcut for only the cloud.
namespace geodesy
{
inline constexpr double wgs84_a = 6378137.0;
inline constexpr double wgs84_e2 = 6.69437999014e-3;
inline std::array<double, 3> lla_to_ecef(
		double longitude, double latitude, double altitude)
{
	double lon = longitude * .017453292519943295769,
		   lat = latitude * .017453292519943295769;
	double sl = std::sin(lat), cl = std::cos(lat);
	double n = wgs84_a / std::sqrt(1 - wgs84_e2 * sl * sl);
	return {(n + altitude) * cl * std::cos(lon), (n + altitude) * cl * std::sin(lon),
			(n * (1 - wgs84_e2) + altitude) * sl};
}
inline std::array<double, 3> ecef_to_lla(const std::array<double, 3> &point)
{
	double longitude = std::atan2(point[1], point[0]);
	double horizontal = std::hypot(point[0], point[1]);
	double latitude = std::atan2(point[2], horizontal * (1 - wgs84_e2)), altitude = 0;
	for (unsigned i = 0; i < 6; ++i) {
		double sl = std::sin(latitude), n = wgs84_a / std::sqrt(1 - wgs84_e2 * sl * sl);
		altitude = horizontal / std::cos(latitude) - n;
		latitude = std::atan2(point[2], horizontal * (1 - wgs84_e2 * n / (n + altitude)));
	}
	return {longitude * 57.295779513082320876, latitude * 57.295779513082320876,
			altitude};
}
inline std::array<std::array<double, 3>, 3> enu_rows(double longitude, double latitude)
{
	double lon = longitude * .017453292519943295769,
		   lat = latitude * .017453292519943295769;
	double slon = std::sin(lon), clon = std::cos(lon), slat = std::sin(lat),
		   clat = std::cos(lat);
	return {{{-slon, clon, 0}, {-slat * clon, -slat * slon, clat},
			{clat * clon, clat * slon, slat}}};
}
inline std::array<double, 3> topocentric_to_lla(
		const std::array<double, 3> &point, const std::array<double, 3> &reference_lla)
{
	auto ecef = lla_to_ecef(reference_lla[0], reference_lla[1], reference_lla[2]);
	auto rows = enu_rows(reference_lla[0], reference_lla[1]);
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			ecef[j] += point[i] * rows[i][j];
	return ecef_to_lla(ecef);
}
inline std::array<double, 3> lla_to_topocentric(double longitude, double latitude,
		double altitude, const std::array<double, 3> &reference_lla)
{
	auto origin = lla_to_ecef(reference_lla[0], reference_lla[1], reference_lla[2]);
	auto ecef = lla_to_ecef(longitude, latitude, altitude);
	auto rows = enu_rows(reference_lla[0], reference_lla[1]);
	std::array<double, 3> result{};
	for (unsigned i = 0; i < 3; ++i)
		for (unsigned j = 0; j < 3; ++j)
			result[i] += rows[i][j] * (ecef[j] - origin[j]);
	return result;
}
inline std::optional<double> parse_length_m(const std::string &text)
{
	std::size_t begin = 0, end = text.size();
	while (begin < end && std::isspace(static_cast<unsigned char>(text[begin])))
		++begin;
	while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])))
		--end;
	std::size_t split = begin;
	while (split < end && (std::isdigit(static_cast<unsigned char>(text[split])) ||
								  text[split] == '.' || text[split] == ',' ||
								  text[split] == '+' || text[split] == '-'))
		++split;
	std::string number = text.substr(begin, split - begin);
	std::replace(number.begin(), number.end(), ',', '.');
	if (number.empty() || std::count(number.begin(), number.end(), '.') > 1)
		return {};
	char *tail = nullptr;
	double value = std::strtod(number.c_str(), &tail);
	if (!tail || *tail)
		return {};
	std::string unit = text.substr(split, end - split);
	while (!unit.empty() && std::isspace(static_cast<unsigned char>(unit.front())))
		unit.erase(unit.begin());
	for (char &c : unit)
		c = char(std::tolower(static_cast<unsigned char>(c)));
	if (unit.empty() || unit == "m" || unit == "meter" || unit == "meters" ||
			unit == "metre" || unit == "metres")
		return value;
	if (unit == "ft" || unit == "feet" || unit == "foot" || unit == "'")
		return value * .3048;
	return {};
}
} // namespace geodesy
}
