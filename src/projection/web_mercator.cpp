#include "web_mercator.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace arnis::projection
{
namespace
{
constexpr double R = 6371000.0;
constexpr double PI = 3.14159265358979323846;
double rad(double d)
{
	return d * PI / 180.0;
}
}
std::string to_string(ProjectionKind kind)
{
	return kind == ProjectionKind::WebMercator ? "web_mercator" : "local";
}
ProjectionKind projection_kind_from_string(const std::string &text)
{
	std::string value;
	value.reserve(text.size());
	for (unsigned char c : text)
		value.push_back(char(std::tolower(c)));
	if (value == "web_mercator" || value == "webmercator" || value == "mercator")
		return ProjectionKind::WebMercator;
	if (value == "local")
		return ProjectionKind::Local;
	throw std::invalid_argument("unknown projection kind: '" + text + "'");
}
WebMercatorProjection::WebMercatorProjection(double lat, double lon, double scale) :
		origin_lat_(lat), origin_lon_(lon), scale_(scale)
{
	if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(scale) || scale <= 0)
		throw std::invalid_argument("invalid Web Mercator projection origin or scale");
	z_offset_ = R * std::log(std::tan(PI / 4 + rad(origin_lat_) / 2)) *
				std::cos(rad(origin_lat_)) * scale_;
}
std::pair<double, double> WebMercatorProjection::forward(double lat, double lon) const
{
	double x = R * rad(lon - origin_lon_) * std::cos(rad(origin_lat_)) * scale_;
	double z = -R * std::log(std::tan(PI / 4 + rad(lat) / 2)) *
					   std::cos(rad(origin_lat_)) * scale_ +
			   z_offset_;
	return {x, z};
}
std::pair<double, double> WebMercatorProjection::inverse(double x, double z) const
{
	double lon =
			origin_lon_ + (x / (R * std::cos(rad(origin_lat_)) * scale_)) * 180.0 / PI;
	double y = -(z - z_offset_) / (R * std::cos(rad(origin_lat_)) * scale_);
	double lat = 2 * (std::atan(std::exp(y)) - PI / 4) * 180.0 / PI;
	return {lat, lon};
}
double WebMercatorProjection::x_for_lon(double longitude) const
{
	return forward(origin_lat_, longitude).first;
}
double WebMercatorProjection::z_for_lat(double latitude) const
{
	return forward(latitude, origin_lon_).second;
}
double WebMercatorProjection::lon_for_x(double x) const
{
	return inverse(x, 0.0).second;
}
double WebMercatorProjection::lat_for_z(double z) const
{
	return inverse(0.0, z).first;
}
int snap_edge(double value, bool round_up)
{
	const double nearest = std::round(value);
	const double snapped = std::abs(value - nearest) < 1e-6
								   ? nearest
								   : (round_up ? std::ceil(value) : std::floor(value));
	return static_cast<int>(std::clamp(snapped, double(std::numeric_limits<int>::min()),
			double(std::numeric_limits<int>::max())));
}
}
