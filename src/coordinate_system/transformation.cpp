#include "transformation.h"
#include "../projection/web_mercator.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
namespace arnis::coordinate_system
{
double lon_to_mercator_x(double lng)
{
	return lng * 3.14159265358979323846 / 180.0 * 6378137.0;
}
double lat_to_mercator_y(double lat)
{
	lat = std::clamp(lat, -85.05112878, 85.05112878);
	double r = lat * 3.14159265358979323846 / 180.0;
	return 6378137.0 * std::log(std::tan(0.7853981633974483 + r * 0.5));
}
double mercator_x_to_lon(double x)
{
	return x / 6378137.0 * 180.0 / 3.14159265358979323846;
}
double mercator_y_to_lat(double y)
{
	return (2 * std::atan(std::exp(y / 6378137.0)) - 1.5707963267948966) * 180.0 /
		   3.14159265358979323846;
}
static constexpr double R = 6371000.0, PI = 3.14159265358979323846;
double lat_distance(double a, double b)
{
	double d = (b - a) * PI / 180.0, s = std::sin(d / 2);
	return R * 2 * std::atan2(std::abs(s), std::sqrt(std::max(0.0, 1 - s * s)));
}
double lon_distance(double lat, double a, double b)
{
	double d = (b - a) * PI / 180.0, s = std::cos(lat * PI / 180.0) * std::sin(d / 2),
		   q = s * s;
	return R * 2 * std::atan2(std::abs(s), std::sqrt(std::max(0.0, 1 - q)));
}
std::pair<double, double> geo_distance(
		::arnis::geographic::LLPoint a, ::arnis::geographic::LLPoint b)
{
	return {lat_distance(a.lat(), b.lat()),
			lon_distance((a.lat() + b.lat()) / 2, a.lng(), b.lng())};
}
::arnis::cartesian::XZPoint CoordTransformer::transform_point(
		::arnis::geographic::LLPoint p) const
{
	if (mercator_) {
		const double lat = p.lat();
		const double x = R * (p.lng() - origin_lng_) * PI / 180.0 * cos_ref_ * scale_;
		const double z =
				-R * std::log(std::tan(PI / 4 + lat * PI / 360.0)) * cos_ref_ * scale_ +
				z_offset_;
		return {static_cast<int>(std::floor(x)), static_cast<int>(std::floor(z))};
	}
	double rx = len_lng_ == 0 ? 0 : (p.lng() - min_lng_) / len_lng_,
		   rz = len_lat_ == 0 ? 0 : 1 - (p.lat() - min_lat_) / len_lat_;
	return {int(rx * sx_), int(rz * sz_)};
}
std::pair<CoordTransformer, ::arnis::cartesian::XZBBoxRect>
CoordTransformer::with_web_mercator(const ::arnis::geographic::LLBBox &b, double scale)
{
	return with_web_mercator(b, scale, (b.min().lat() + b.max().lat()) * 0.5,
			(b.min().lng() + b.max().lng()) * 0.5);
}

std::pair<CoordTransformer, ::arnis::cartesian::XZBBoxRect>
CoordTransformer::with_web_mercator(const ::arnis::geographic::LLBBox &b, double scale,
		double origin_lat, double origin_lng)
{
	if (scale <= 0)
		throw std::invalid_argument("scale <= 0");
	double la0 = b.min().lat(), la1 = b.max().lat(), lo0 = b.min().lng(),
		   lo1 = b.max().lng();
	const double cos_ref = std::cos(origin_lat * PI / 180.0);
	const double z_offset =
			R * std::log(std::tan(PI / 4 + origin_lat * PI / 360.0)) * cos_ref * scale;
	// Keep this local frame identical to WebMercatorProjection: easting is
	// relative to the selected origin longitude, and north maps to negative Z.
	// Calculating bounds in global Mercator coordinates used to return an AABB
	// thousands of kilometres away from transform_point's local coordinates.
	auto project = [&](double lat, double lon) {
		return std::pair<double, double>{
				R * (lon - origin_lng) * PI / 180.0 * cos_ref * scale,
				-R * std::log(std::tan(PI / 4 + lat * PI / 360.0)) * cos_ref * scale +
						z_offset};
	};
	const double x_west = project(la1, lo0).first;
	const double x_east = project(la1, lo1).first;
	const double z_north = project(la1, lo0).second;
	const double z_south = project(la0, lo0).second;
	constexpr double WORLD_BORDER = 29999984.0;
	const double edges[] = {x_west, x_east, z_north, z_south};
	for (const double edge : edges)
		if (!std::isfinite(edge) || std::abs(edge) > WORLD_BORDER)
			throw std::invalid_argument("projected bbox is outside the world border");
	const int x_min = projection::snap_edge(x_west, false);
	const int z_min = projection::snap_edge(z_north, false);
	const int x_max = std::max(projection::snap_edge(x_east, true) - 1, x_min);
	const int z_max = std::max(projection::snap_edge(z_south, true) - 1, z_min);
	CoordTransformer t(
			la1 - la0, lo1 - lo0, x_max - x_min + 1, z_max - z_min + 1, la0, lo0);
	t.mercator_ = true;
	t.origin_lat_ = origin_lat;
	t.origin_lng_ = origin_lng;
	t.scale_ = scale;
	t.cos_ref_ = cos_ref;
	t.z_offset_ = z_offset;
	return {t, ::arnis::cartesian::XZBBoxRect({x_min, z_min}, {x_max, z_max})};
}
std::pair<CoordTransformer, ::arnis::cartesian::XZBBoxRect>
CoordTransformer::llbbox_to_xzbbox(const ::arnis::geographic::LLBBox &b, double scale)
{
	if (scale <= 0)
		throw std::invalid_argument("scale <= 0");
	auto d = geo_distance(b.min(), b.max());
	double sz = std::floor(d.first) * scale, sx = std::floor(d.second) * scale;
	auto x = ::arnis::cartesian::XZBBoxRect({0, 0}, {int(sx), int(sz)});
	return {CoordTransformer(b.max().lat() - b.min().lat(), b.max().lng() - b.min().lng(),
					sx, sz, b.min().lat(), b.min().lng()),
			x};
}
}
