#include "web_mercator.h"
#include <algorithm>
#include <cctype>
#include <cmath>
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
	origin_lat_ = std::clamp(origin_lat_, -85.05112878, 85.05112878);
	z_offset_ = R * std::log(std::tan(PI / 4 + rad(origin_lat_) / 2)) * scale_;
}
std::pair<double, double> WebMercatorProjection::forward(double lat, double lon) const
{
	lat = std::clamp(lat, -85.05112878, 85.05112878);
	double x = R * rad(lon - origin_lon_) * std::cos(rad(origin_lat_)) * scale_;
	double z = -R * std::log(std::tan(PI / 4 + rad(lat) / 2)) * scale_ + z_offset_;
	return {x, z};
}
std::pair<double, double> WebMercatorProjection::inverse(double x, double z) const
{
	double lon =
			origin_lon_ + (x / (R * std::cos(rad(origin_lat_)) * scale_)) * 180.0 / PI;
	double y = -(z - z_offset_) / (R * scale_);
	double lat = 2 * (std::atan(std::exp(y)) - PI / 4) * 180.0 / PI;
	return {std::clamp(lat, -85.05112878, 85.05112878), lon};
}
}
