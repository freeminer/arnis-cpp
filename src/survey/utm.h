#pragma once

#include <array>
#include <utility>

namespace arnis::survey::utm
{
// Sixth-order Krueger series for Transverse Mercator on a specified ellipsoid.
class Series
{
	double big_a_ = 0.0;
	std::array<double, 6> alpha_{};
	std::array<double, 6> beta_{};
	double eccentricity_ = 0.0;

public:
	Series(double semi_major_axis, double flattening);
	std::pair<double, double> forward(
			double latitude, double longitude_from_central_meridian) const;
	std::pair<double, double> inverse(double eta, double xi) const;
	double big_a() const { return big_a_; }
};

struct Tm
{
	double longitude_origin = 0.0;
	double latitude_origin = 0.0;
	double scale = 0.9996;
	double false_easting = 500000.0;
	double false_northing = 0.0;

	std::pair<double, double> forward(
			const Series &series, double latitude, double longitude) const;
	std::pair<double, double> inverse(
			const Series &series, double easting, double northing) const;
};

const Series &wgs84();
const Series &grs80();
Tm utm_zone(unsigned zone);
std::pair<double, double> to_utm(double latitude, double longitude, unsigned zone);
std::pair<double, double> from_utm(double easting, double northing, unsigned zone);
} // namespace arnis::survey::utm
