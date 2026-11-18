#include "crs_internal.h"

#include "utm.h"

#include <array>
#include <cmath>

namespace arnis::survey::crs::detail
{
namespace
{
constexpr double PI = 3.1415926535897932384626433832795;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

struct ConeParams
{
	double lat0, lat1, lat2, lon0, x0, y0;
};
constexpr ConeParams LAMBERT93{46.5, 49.0, 44.0, 3.0, 700000.0, 6600000.0};
constexpr ConeParams LEST97{
		57.51755393055556, 59.33333333333334, 58.0, 24.0, 500000.0, 6375000.0};

double cone_t(double e, double phi)
{
	const double es = e * std::sin(phi);
	return std::tan(PI / 4.0 - phi / 2.0) * std::pow((1.0 + es) / (1.0 - es), e / 2.0);
}

std::pair<double, double> cone(const ConeParams &p, double lat, double lon, bool inverse)
{
	constexpr double a = 6378137.0, f = 1.0 / 298.257222101;
	const double e = std::sqrt(f * (2.0 - f));
	auto m = [e](double phi) {
		return std::cos(phi) / std::sqrt(1.0 - std::pow(e * std::sin(phi), 2));
	};
	const double p0 = p.lat0 * DEG_TO_RAD, p1 = p.lat1 * DEG_TO_RAD,
				 p2 = p.lat2 * DEG_TO_RAD;
	const double n = (std::log(m(p1)) - std::log(m(p2))) /
					 (std::log(cone_t(e, p1)) - std::log(cone_t(e, p2)));
	const double af = a * m(p1) / (n * std::pow(cone_t(e, p1), n));
	const double rho0 = af * std::pow(cone_t(e, p0), n);
	if (!inverse) {
		const double rho = af * std::pow(cone_t(e, lat * DEG_TO_RAD), n);
		const double theta = n * (lon - p.lon0) * DEG_TO_RAD;
		return {p.x0 + rho * std::sin(theta), p.y0 + rho0 - rho * std::cos(theta)};
	}
	const double dx = lat - p.x0, dy = rho0 - (lon - p.y0);
	const double rho = std::hypot(dx, dy), tp = std::pow(rho / af, 1.0 / n);
	const double out_lon = p.lon0 + std::atan2(dx, dy) / n * RAD_TO_DEG;
	double phi = PI / 2.0 - 2.0 * std::atan(tp);
	for (int i = 0; i < 10; ++i) {
		const double es = e * std::sin(phi);
		const double next =
				PI / 2.0 -
				2.0 * std::atan(tp * std::pow((1.0 - es) / (1.0 + es), e / 2.0));
		const bool done = std::abs(next - phi) < 1e-13;
		phi = next;
		if (done)
			break;
	}
	return {phi * RAD_TO_DEG, out_lon};
}

struct Term
{
	double c;
	int p, q;
};
template <std::size_t N>
double poly(const std::array<Term, N> &terms, double a, double b)
{
	double out = 0.0;
	for (const auto &term : terms)
		out += term.c * std::pow(a, term.p) * std::pow(b, term.q);
	return out;
}
constexpr std::array<Term, 9> KX{{{190094.945, 0, 1}, {-11832.228, 1, 1},
		{-114.221, 2, 1}, {-32.391, 0, 3}, {-0.705, 1, 0}, {-2.340, 3, 1}, {-0.608, 1, 3},
		{-0.008, 0, 2}, {0.148, 2, 3}}};
constexpr std::array<Term, 10> KY{{{309056.544, 1, 0}, {3638.893, 0, 2}, {73.077, 2, 0},
		{-157.984, 1, 2}, {59.788, 3, 0}, {0.433, 0, 1}, {-6.439, 2, 2}, {-0.032, 1, 1},
		{0.092, 0, 4}, {-0.054, 1, 4}}};
constexpr std::array<Term, 11> KLAT{
		{{3235.65389, 0, 1}, {-32.58297, 2, 0}, {-0.24750, 0, 2}, {-0.84978, 2, 1},
				{-0.06550, 0, 3}, {-0.01709, 2, 2}, {-0.00738, 1, 0}, {0.00530, 4, 0},
				{-0.00039, 2, 3}, {0.00033, 4, 1}, {-0.00012, 1, 1}}};
constexpr std::array<Term, 12> KLON{
		{{5260.52916, 1, 0}, {105.94684, 1, 1}, {2.45656, 1, 2}, {-0.81885, 3, 0},
				{0.05594, 1, 3}, {-0.05607, 3, 1}, {0.01199, 0, 1}, {-0.00256, 3, 2},
				{0.00128, 1, 4}, {0.00022, 0, 2}, {-0.00022, 2, 0}, {0.00026, 5, 0}}};
} // namespace

std::pair<double, double> lambert93_forward(double lat, double lon)
{
	return cone(LAMBERT93, lat, lon, false);
}
std::pair<double, double> lambert93_inverse(double x, double y)
{
	return cone(LAMBERT93, x, y, true);
}
std::pair<double, double> lest97_forward(double lat, double lon)
{
	return cone(LEST97, lat, lon, false);
}
std::pair<double, double> lest97_inverse(double x, double y)
{
	return cone(LEST97, x, y, true);
}

std::pair<double, double> rd_forward(double lat, double lon)
{
	const double dp = 0.36 * (lat - 52.15517440), dl = 0.36 * (lon - 5.38720621);
	return {155000.0 + poly(KX, dp, dl), 463000.0 + poly(KY, dp, dl)};
}
std::pair<double, double> rd_inverse(double x, double y)
{
	const double dx = (x - 155000.0) * 1e-5, dy = (y - 463000.0) * 1e-5;
	return {52.15517440 + poly(KLAT, dx, dy) / 3600.0,
			5.38720621 + poly(KLON, dx, dy) / 3600.0};
}

std::pair<double, double> lv95_forward(double lat, double lon)
{
	const double p = (lat * 3600.0 - 169028.66) / 10000.0;
	const double l = (lon * 3600.0 - 26782.5) / 10000.0;
	const double e = 2600072.37 + 211455.93 * l - 10938.51 * l * p - 0.36 * l * p * p -
					 44.54 * std::pow(l, 3);
	const double n = 1200147.07 + 308807.95 * p + 3745.25 * l * l + 76.63 * p * p -
					 194.56 * l * l * p + 119.79 * std::pow(p, 3);
	return {e, n};
}
std::pair<double, double> lv95_inverse(double e, double n)
{
	const double y = (e - 2600000.0) / 1e6, x = (n - 1200000.0) / 1e6;
	const double l = 2.6779094 + 4.728982 * y + 0.791484 * y * x + 0.1306 * y * x * x -
					 0.0436 * std::pow(y, 3);
	const double p = 16.9023892 + 3.238272 * x - 0.270978 * y * y - 0.002528 * x * x -
					 0.0447 * y * y * x - 0.0140 * std::pow(x, 3);
	double lat = p * 100.0 / 36.0, lon = l * 100.0 / 36.0;
	for (int i = 0; i < 3; ++i) {
		const auto [fe, fn] = lv95_forward(lat, lon);
		constexpr double d = 1e-5;
		const auto [e_lat, n_lat] = lv95_forward(lat + d, lon);
		const auto [e_lon, n_lon] = lv95_forward(lat, lon + d);
		const double a = (e_lat - fe) / d, b = (e_lon - fe) / d;
		const double c = (n_lat - fn) / d, dd = (n_lon - fn) / d;
		const double det = a * dd - b * c, re = e - fe, rn = n - fn;
		lat += (dd * re - b * rn) / det;
		lon += (a * rn - c * re) / det;
	}
	return {lat, lon};
}

std::pair<double, double> mercator_forward(double lat, double lon)
{
	constexpr double r = 6378137.0;
	const double phi = lat * DEG_TO_RAD;
	return {r * lon * DEG_TO_RAD, r * std::log(std::tan(PI / 4.0 + phi / 2.0))};
}
std::pair<double, double> mercator_inverse(double x, double y)
{
	constexpr double r = 6378137.0;
	return {(2.0 * std::atan(std::exp(y / r)) - PI / 2.0) * RAD_TO_DEG,
			x / r * RAD_TO_DEG};
}

std::pair<double, double> luref_forward(double lat, double lon)
{
	constexpr double a = 6378388.0, f = 1.0 / 297.0;
	const double phi = lat * DEG_TO_RAD, lambda = lon * DEG_TO_RAD;
	const double e2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563);
	const double sp = std::sin(phi), cp = std::cos(phi), sl = std::sin(lambda),
				 cl = std::cos(lambda);
	const double nu = 6378137.0 / std::sqrt(1.0 - e2 * sp * sp);
	std::array<double, 3> wgs{nu * cp * cl, nu * cp * sl, nu * (1.0 - e2) * sp};
	constexpr std::array<double, 3> t{-189.6806, 18.3463, -42.7695};
	constexpr std::array<double, 3> rotation{0.33746, 3.09264, -2.53861};
	const double rx = rotation[0] / 3600.0 * DEG_TO_RAD;
	const double ry = rotation[1] / 3600.0 * DEG_TO_RAD;
	const double rz = rotation[2] / 3600.0 * DEG_TO_RAD;
	const double m[3][3]{{1.0, rz, -ry}, {-rz, 1.0, rx}, {ry, -rx, 1.0}};
	constexpr double scale = 1.0 + 0.4598e-6;
	std::array<double, 3> local_xyz{};
	for (int i = 0; i < 3; ++i)
		local_xyz[i] = (m[0][i] * (wgs[0] - t[0]) + m[1][i] * (wgs[1] - t[1]) +
							   m[2][i] * (wgs[2] - t[2])) /
					   scale;
	const double local_e2 = f * (2.0 - f);
	const double radius = std::hypot(local_xyz[0], local_xyz[1]);
	double local_lat = std::atan2(local_xyz[2], radius * (1.0 - local_e2));
	for (int i = 0; i < 6; ++i) {
		const double s = std::sin(local_lat);
		const double local_nu = a / std::sqrt(1.0 - local_e2 * s * s);
		local_lat = std::atan2(local_xyz[2] + local_e2 * local_nu * s, radius);
	}
	const double local_lon = std::atan2(local_xyz[1], local_xyz[0]);
	const utm::Series series(a, f);
	const utm::Tm tm{6.0 + 10.0 / 60.0, 49.0 + 50.0 / 60.0, 1.0, 80000.0, 100000.0};
	return tm.forward(series, local_lat * RAD_TO_DEG, local_lon * RAD_TO_DEG);
}
std::pair<double, double> luref_inverse(double easting, double northing)
{
	constexpr double a = 6378388.0, f = 1.0 / 297.0;
	const utm::Series series(a, f);
	const utm::Tm tm{6.0 + 10.0 / 60.0, 49.0 + 50.0 / 60.0, 1.0, 80000.0, 100000.0};
	const auto [local_lat_deg, local_lon_deg] = tm.inverse(series, easting, northing);
	const double phi = local_lat_deg * DEG_TO_RAD, lambda = local_lon_deg * DEG_TO_RAD;
	const double e2 = f * (2.0 - f);
	const double sp = std::sin(phi), cp = std::cos(phi), sl = std::sin(lambda),
				 cl = std::cos(lambda);
	const double nu = a / std::sqrt(1.0 - e2 * sp * sp);
	const std::array<double, 3> local{nu * cp * cl, nu * cp * sl, nu * (1.0 - e2) * sp};
	constexpr std::array<double, 3> t{-189.6806, 18.3463, -42.7695};
	constexpr std::array<double, 3> rotation{0.33746, 3.09264, -2.53861};
	const double rx = rotation[0] / 3600.0 * DEG_TO_RAD;
	const double ry = rotation[1] / 3600.0 * DEG_TO_RAD;
	const double rz = rotation[2] / 3600.0 * DEG_TO_RAD;
	const double m[3][3]{{1.0, rz, -ry}, {-rz, 1.0, rx}, {ry, -rx, 1.0}};
	constexpr double scale = 1.0 + 0.4598e-6;
	std::array<double, 3> wgs{};
	for (int i = 0; i < 3; ++i)
		wgs[i] = t[i] +
				 scale * (m[i][0] * local[0] + m[i][1] * local[1] + m[i][2] * local[2]);
	constexpr double wa = 6378137.0, wf = 1.0 / 298.257223563;
	const double we2 = wf * (2.0 - wf), r = std::hypot(wgs[0], wgs[1]);
	double lat = std::atan2(wgs[2], r * (1.0 - we2));
	for (int i = 0; i < 6; ++i) {
		const double s = std::sin(lat), wnu = wa / std::sqrt(1.0 - we2 * s * s);
		lat = std::atan2(wgs[2] + we2 * wnu * s, r);
	}
	return {lat * RAD_TO_DEG, std::atan2(wgs[1], wgs[0]) * RAD_TO_DEG};
}
} // namespace arnis::survey::crs::detail
