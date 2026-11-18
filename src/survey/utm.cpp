#include "utm.h"

#include <cmath>

namespace arnis::survey::utm
{
namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = 1.0 / 298.257223563;
constexpr double GRS80_F = 1.0 / 298.257222101;
constexpr double UTM_K0 = 0.9996;
constexpr double FALSE_EASTING = 500000.0;
constexpr double DEG_TO_RAD = 3.1415926535897932384626433832795 / 180.0;
constexpr double RAD_TO_DEG = 180.0 / 3.1415926535897932384626433832795;
}

Series::Series(double a, double f)
{
	const double n = f / (2.0 - f);
	const double n2 = n * n, n3 = n2 * n, n4 = n3 * n, n5 = n4 * n, n6 = n5 * n;
	big_a_ = a / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);
	alpha_ = {n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 -
					  127.0 * n5 / 288.0 + 7891.0 * n6 / 37800.0,
			13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0 -
					1983433.0 * n6 / 1935360.0,
			61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0 +
					167603.0 * n6 / 181440.0,
			49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0 + 6601661.0 * n6 / 7257600.0,
			34729.0 * n5 / 80640.0 - 3418889.0 * n6 / 1995840.0,
			212378941.0 * n6 / 319334400.0};
	beta_ = {n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0 - n4 / 360.0 -
					 81.0 * n5 / 512.0 + 96199.0 * n6 / 604800.0,
			n2 / 48.0 + n3 / 15.0 - 437.0 * n4 / 1440.0 + 46.0 * n5 / 105.0 -
					1118711.0 * n6 / 3870720.0,
			17.0 * n3 / 480.0 - 37.0 * n4 / 840.0 - 209.0 * n5 / 4480.0 +
					5569.0 * n6 / 90720.0,
			4397.0 * n4 / 161280.0 - 11.0 * n5 / 504.0 - 830251.0 * n6 / 7257600.0,
			4583.0 * n5 / 161280.0 - 108847.0 * n6 / 3991680.0,
			20648693.0 * n6 / 638668800.0};
	eccentricity_ = std::sqrt(f * (2.0 - f));
}

std::pair<double, double> Series::forward(double latitude, double longitude_delta) const
{
	const double phi = latitude * DEG_TO_RAD;
	const double sin_phi = std::sin(phi);
	const double t = std::sinh(
			std::atanh(sin_phi) - eccentricity_ * std::atanh(eccentricity_ * sin_phi));
	const double xi_p = std::atan2(t, std::cos(longitude_delta));
	const double eta_p = std::atanh(std::sin(longitude_delta) / std::sqrt(1.0 + t * t));
	double xi = xi_p, eta = eta_p;
	for (std::size_t j = 0; j < alpha_.size(); ++j) {
		const double k = 2.0 * (j + 1);
		const double alpha = alpha_[j];
		xi += alpha * std::sin(k * xi_p) * std::cosh(k * eta_p);
		eta += alpha * std::cos(k * xi_p) * std::sinh(k * eta_p);
	}
	return {eta, xi};
}

std::pair<double, double> Series::inverse(double eta, double xi) const
{
	double xi_p = xi, eta_p = eta;
	for (std::size_t j = 0; j < beta_.size(); ++j) {
		const double k = 2.0 * (j + 1);
		const double beta = beta_[j];
		xi_p -= beta * std::sin(k * xi) * std::cosh(k * eta);
		eta_p -= beta * std::cos(k * xi) * std::sinh(k * eta);
	}
	const double tau_p = std::sin(xi_p) / std::sqrt(std::pow(std::sinh(eta_p), 2) +
													std::pow(std::cos(xi_p), 2));
	double tau = tau_p;
	const double e = eccentricity_;
	for (int i = 0; i < 5; ++i) {
		const double sigma =
				std::sinh(e * std::atanh(e * tau / std::sqrt(1.0 + tau * tau)));
		const double tau_i =
				tau * std::sqrt(1.0 + sigma * sigma) - sigma * std::sqrt(1.0 + tau * tau);
		const double d = (tau_p - tau_i) / std::sqrt(1.0 + tau_i * tau_i) *
						 (1.0 + (1.0 - e * e) * tau * tau) /
						 ((1.0 - e * e) * std::sqrt(1.0 + tau * tau));
		tau += d;
		if (std::abs(d) < 1e-14)
			break;
	}
	return {std::atan(tau) * RAD_TO_DEG, std::atan2(std::sinh(eta_p), std::cos(xi_p))};
}

std::pair<double, double> Tm::forward(
		const Series &series, double latitude, double longitude) const
{
	const auto [eta, xi] = series.forward(
			latitude, longitude * DEG_TO_RAD - longitude_origin * DEG_TO_RAD);
	double xi0 = 0.0;
	if (latitude_origin != 0.0)
		xi0 = series.forward(latitude_origin, 0.0).second;
	return {false_easting + scale * series.big_a() * eta,
			false_northing + scale * series.big_a() * (xi - xi0)};
}

std::pair<double, double> Tm::inverse(
		const Series &series, double easting, double northing) const
{
	double xi0 = 0.0;
	if (latitude_origin != 0.0)
		xi0 = series.forward(latitude_origin, 0.0).second;
	const double xi = (northing - false_northing) / (scale * series.big_a()) + xi0;
	const double eta = (easting - false_easting) / (scale * series.big_a());
	const auto [latitude, longitude_delta] = series.inverse(eta, xi);
	return {latitude, (longitude_origin * DEG_TO_RAD + longitude_delta) * RAD_TO_DEG};
}

const Series &wgs84()
{
	static const Series value(WGS84_A, WGS84_F);
	return value;
}

const Series &grs80()
{
	static const Series value(WGS84_A, GRS80_F);
	return value;
}

Tm utm_zone(unsigned zone)
{
	return {zone * 6.0 - 183.0, 0.0, UTM_K0, FALSE_EASTING, 0.0};
}

std::pair<double, double> to_utm(double latitude, double longitude, unsigned zone)
{
	return utm_zone(zone).forward(wgs84(), latitude, longitude);
}

std::pair<double, double> from_utm(double easting, double northing, unsigned zone)
{
	return utm_zone(zone).inverse(wgs84(), easting, northing);
}
} // namespace arnis::survey::utm
