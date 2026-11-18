#include "crs.h"

#include "crs_internal.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>

namespace arnis::survey::crs
{
namespace
{
std::optional<double> parameter(const std::string &wkt, const std::string &name)
{
	const auto key = "PARAMETER[\"" + name + "\",";
	auto at = wkt.find(key);
	if (at == std::string::npos)
		return std::nullopt;
	at += key.size();
	const auto end = wkt.find_first_of("],", at);
	if (end == std::string::npos)
		return std::nullopt;
	try {
		std::size_t used = 0;
		const double value = std::stod(wkt.substr(at, end - at), &used);
		if (used != end - at)
			return std::nullopt;
		return value;
	} catch (...) {
		return std::nullopt;
	}
}
} // namespace

unsigned utm_zone(double longitude)
{
	const auto zone = static_cast<long long>(std::floor((longitude + 180.0) / 6.0));
	return static_cast<unsigned>((zone % 60 + 60) % 60) + 1;
}

std::pair<double, double> Crs::forward(double latitude, double longitude) const
{
	switch (kind) {
	case Kind::Utm:
	case Kind::Nad83Utm:
		return utm::to_utm(latitude, longitude, zone);
	case Kind::Lambert93:
		return detail::lambert93_forward(latitude, longitude);
	case Kind::RdNew:
		return detail::rd_forward(latitude, longitude);
	case Kind::Lv95:
		return detail::lv95_forward(latitude, longitude);
	case Kind::Lest97:
		return detail::lest97_forward(latitude, longitude);
	case Kind::Luref:
		return detail::luref_forward(latitude, longitude);
	case Kind::WebMercator:
		return detail::mercator_forward(latitude, longitude);
	case Kind::Lks92: {
		const utm::Series series(6378137.0, 1.0 / 298.257222101);
		const utm::Tm tm{24.0, 0.0, 0.9996, 500000.0, -6000000.0};
		return tm.forward(series, latitude, longitude);
	}
	}
	return {};
}

std::pair<double, double> Crs::inverse(double easting, double northing) const
{
	switch (kind) {
	case Kind::Utm:
	case Kind::Nad83Utm:
		return utm::from_utm(easting, northing, zone);
	case Kind::Lambert93:
		return detail::lambert93_inverse(easting, northing);
	case Kind::RdNew:
		return detail::rd_inverse(easting, northing);
	case Kind::Lv95:
		return detail::lv95_inverse(easting, northing);
	case Kind::Lest97:
		return detail::lest97_inverse(easting, northing);
	case Kind::Luref:
		return detail::luref_inverse(easting, northing);
	case Kind::WebMercator:
		return detail::mercator_inverse(easting, northing);
	case Kind::Lks92: {
		const utm::Series series(6378137.0, 1.0 / 298.257222101);
		const utm::Tm tm{24.0, 0.0, 0.9996, 500000.0, -6000000.0};
		return tm.inverse(series, easting, northing);
	}
	}
	return {};
}

std::uint32_t Crs::epsg() const
{
	switch (kind) {
	case Kind::Utm:
		return 25800 + zone;
	case Kind::Lambert93:
		return 2154;
	case Kind::RdNew:
		return 28992;
	case Kind::Lv95:
		return 2056;
	case Kind::Lest97:
		return 3301;
	case Kind::Luref:
		return 2169;
	case Kind::Nad83Utm:
		if (zone <= 23)
			return 26900 + zone;
		if (zone == 55)
			return 8693;
		if (zone == 59)
			return 3372;
		if (zone == 60)
			return 3373;
		return 32600 + zone;
	case Kind::WebMercator:
		return 3857;
	case Kind::Lks92:
		return 3059;
	}
	return 0;
}

bool Crs::north_first() const
{
	return kind == Kind::Lest97 || kind == Kind::Luref || kind == Kind::Lks92;
}

std::optional<Crs> from_wkt(const std::string &wkt)
{
	const auto start = wkt.find("PROJCS[");
	if (start == std::string::npos)
		return std::nullopt;
	auto projected = wkt.substr(start);
	const auto vertical = projected.find("VERT_CS[");
	if (vertical != std::string::npos)
		projected.resize(vertical);
	std::string lower = wkt;
	std::transform(lower.begin(), lower.end(), lower.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (projected.find("PROJECTION[\"Transverse_Mercator\"]") == std::string::npos ||
			lower.find("foot") != std::string::npos)
		return std::nullopt;
	const auto lat0 = parameter(projected, "latitude_of_origin");
	const auto lon0 = parameter(projected, "central_meridian");
	const auto scale = parameter(projected, "scale_factor");
	const auto fe = parameter(projected, "false_easting");
	const auto fn = parameter(projected, "false_northing");
	if (!lat0 || !lon0 || !scale || !fe || !fn)
		return std::nullopt;
	const double zone = (*lon0 + 183.0) / 6.0;
	if (*lat0 == 0.0 && std::abs(*scale - 0.9996) < 1e-9 && *fe == 500000.0 &&
			*fn == 0.0 && zone >= 1.0 && zone <= 60.0 && std::floor(zone) == zone)
		return Crs{Kind::Nad83Utm, static_cast<unsigned>(zone)};
	return std::nullopt;
}

Affine Affine::fit(Crs from, Crs to, double x, double y, double step)
{
	auto sample = [&](double dx, double dy) {
		const auto [lat, lon] = to.inverse(x + dx, y + dy);
		return from.forward(lat, lon);
	};
	const auto centre = sample(0.0, 0.0);
	const auto east = sample(step, 0.0), west = sample(-step, 0.0);
	const auto north = sample(0.0, step), south = sample(0.0, -step);
	const double a = (east.first - west.first) / (2.0 * step);
	const double b = (north.first - south.first) / (2.0 * step);
	const double c = (east.second - west.second) / (2.0 * step);
	const double d = (north.second - south.second) / (2.0 * step);
	const double det = a * d - b * c;
	const double ia = d / det, ib = -b / det, ic = -c / det, id = a / det;
	return {{ia, ib, x - ia * centre.first - ib * centre.second, ic, id,
			y - ic * centre.first - id * centre.second}};
}

std::pair<double, double> Affine::apply(double x, double y) const
{
	return {m[0] * x + m[1] * y + m[2], m[3] * x + m[4] * y + m[5]};
}

Affine Affine::inverse() const
{
	const double det = m[0] * m[4] - m[1] * m[3];
	const double a = m[4] / det, b = -m[1] / det, c = -m[3] / det, d = m[0] / det;
	return {{a, b, -a * m[2] - b * m[5], c, d, -c * m[2] - d * m[5]}};
}

std::array<double, 4> Affine::rect(double x0, double y0, double x1, double y1) const
{
	const std::array points{apply(x0, y0), apply(x0, y1), apply(x1, y0), apply(x1, y1)};
	double min_x = std::numeric_limits<double>::max();
	double min_y = min_x, max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
	for (const auto &[x, y] : points) {
		min_x = std::min(min_x, x);
		min_y = std::min(min_y, y);
		max_x = std::max(max_x, x);
		max_y = std::max(max_y, y);
	}
	return {min_x, min_y, max_x, max_y};
}
} // namespace arnis::survey::crs
