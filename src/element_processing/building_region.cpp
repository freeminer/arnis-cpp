#include "building_region.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace arnis::building_style
{
namespace
{
bool in(double value, double lo, double hi)
{
	return value >= lo && value < hi;
}

bool in_closed(double value, double lo, double hi)
{
	return value >= lo && value <= hi;
}

double interpolate(const std::pair<double, double> *points, std::size_t count, double x)
{
	if (x <= points[0].first)
		return points[0].second;
	for (std::size_t i = 1; i < count; ++i) {
		const auto &[x0, y0] = points[i - 1];
		const auto &[x1, y1] = points[i];
		if (x <= x1)
			return y0 + (x - x0) / (x1 - x0) * (y1 - y0);
	}
	return points[count - 1].second;
}

MaterialRegion north_america(double lat, double lon)
{
	// Greenland east of Baffin Bay.
	if (lat > 59.5 && (lon > -60.0 || (lat > 76.0 && lon > -75.0)))
		return MaterialRegion::Nordic;
	const double border = lon <= -114.8	  ? 32.53
						  : lon <= -111.0 ? 32.53 - (lon + 114.8) * 0.316
						  : lon <= -106.5 ? 31.33
						  : lon <= -97.1  ? 31.76 - 0.645 * (lon + 106.5)
										  : 24.4;
	if (lat < border || (lon > -79.5 && lat < 27.5))
		return MaterialRegion::LatinAmerica;

	bool canadian = false;
	if (lon >= -141.0) {
		if (in_closed(lon, -125.0, -123.2) && lat >= 48.3) {
			canadian = true;
		} else if (in(lon, -83.13, -82.415)) {
			const double river = lon < -82.9 ? 42.325 + (lon + 83.04) * 0.35
											 : 42.36 + (lon + 82.9) * 0.55;
			canadian = lat >= 41.7 && lat < river;
		} else {
			const double us_border =
					lon < -95.15   ? 49.0
					: lon < -90.5  ? 48.605
					: lon < -89.5  ? 48.0
					: lon < -84.5  ? 47.8
					: lon < -83.13 ? 46.25
					: lon < -78.95 ? 41.75 + (lon + 82.5) * 0.24
					: lon < -76.35 ? 43.65
					: lon < -74.73
							? interpolate(
									  std::array<std::pair<double, double>, 6>{
											  {{-76.35, 44.17}, {-75.9, 44.36},
													  {-75.68, 44.585}, {-75.5, 44.70},
													  {-75.1, 44.88}, {-74.73, 45.0}}}
											  .data(),
									  6, lon)
					: lon < -71.5 ? 45.0
					: lon < -69.2 ? 45.0 + (lon + 71.5) * 1.07
					: lon < -67.8 ? 47.25
					: lon < -67.0 ? 45.0
								  : 40.0;
			canadian = lat >= us_border;
		}
	}
	if (canadian)
		return MaterialRegion::Canada;
	if ((lat < 42.0 && lon < -114.0) || (lat < 37.5 && lon < -103.0) ||
			(lat < 31.0 && lon > -87.7))
		return MaterialRegion::NorthAmericaSunbelt;
	return MaterialRegion::NorthAmerica;
}

MaterialRegion europe(double lat, double lon)
{
	const bool maghreb = lon < 11.6 && lat < (lon < -1.5 ? 35.95 : 37.5);
	const double levant_border = lon >= 42.4 ? 37.4 : lon >= 40.0 ? 37.15 : 37.0;
	const bool levant = lon >= 35.7 && lat < levant_border;
	if (maghreb || levant)
		return MaterialRegion::MiddleEast;

	const bool caucasus = (lat >= 41.45 && lon >= 40.0 && lat < 43.35) ||
						  (lon >= 43.45 && in(lat, 40.3, 43.35)) ||
						  (lon >= 44.4 && in(lat, 38.85, 43.35));
	if (caucasus)
		return MaterialRegion::CentralAsia;

	const double prut =
			lat >= 47.2 ? 27.7 - (lat - 47.2) * 1.19 : 28.1 - (lat - 46.0) * 0.33;
	const bool east = (lat >= 48.0 && lon >= 23.6 && (lat < 59.7 || lon >= 28.0) &&
							  (lat < 60.5 || lon >= 31.0)) ||
					  (lat >= 54.35 && lon >= 19.6 && lat < 59.7) ||
					  (in(lat, 45.5, 48.5) && lon >= prut) ||
					  (lat >= 44.3 && lon >= 29.7) || (lat >= 43.35 && lon >= 37.0);
	if (east)
		return MaterialRegion::EastEurope;

	const double british_south = lon < -2.0 ? 49.85 : lon < 1.25 ? 50.3 : 51.0;
	const bool british =
			in_closed(lon, -11.0, 1.8) && lat <= 61.0 && lat >= british_south;
	if (british)
		return MaterialRegion::BritishIsles;
	if (lat >= 62.0 || (lat >= 54.5 && lon >= 4.5 && (lon < 21.0 || lat > 59.5)))
		return MaterialRegion::Nordic;
	if (lat < 44.3 || (lat < 46.0 && in(lon, 6.6, 13.9)))
		return MaterialRegion::Mediterranean;
	const bool low_countries = in_closed(lat, 50.25, 53.7) && lon >= 2.4 &&
							   lon < (lat >= 52.0 ? 7.05 : 6.05);
	const bool north_coast = lat >= 53.0 && in(lon, 7.0, 24.0);
	const bool westphalia = lat >= 51.7 && in(lon, 6.0, 9.0);
	if (low_countries || north_coast || westphalia)
		return MaterialRegion::BrickNorth;

	const double france_east = lat >= 49.0 ? 6.5 : lat >= 47.6 ? 7.6 : 6.0;
	const bool france = lon >= -5.5 && lat <= 51.2 && lon < france_east;
	if (france)
		return MaterialRegion::France;
	if (in(lat, 49.6, 52.0) && lon >= 10.5)
		return MaterialRegion::CentralEast;
	return MaterialRegion::Central;
}

double mongolia_south_border(double lon)
{
	static constexpr std::array<std::pair<double, double>, 6> points = {{{87.7, 49.2},
			{91.0, 46.6}, {97.0, 42.75}, {105.0, 41.6}, {112.0, 43.7}, {116.0, 45.5}}};
	return interpolate(points.data(), points.size(), lon);
}

MaterialRegion elsewhere(double lat, double lon)
{
	using R = MaterialRegion;
	if ((lat <= -10.5 && in_closed(lon, 112.0, 155.0)) || (lat <= -33.5 && lon >= 165.0))
		return R::Australasia;
	const bool japan =
			(in_closed(lat, 30.0, 45.6) && in_closed(lon, 129.0, 146.0) &&
					!(lon < 129.6 && lat >= 34.8) && !(lat > 42.3 && lon < 139.0)) ||
			(in(lat, 24.0, 30.0) && in_closed(lon, 122.9, 131.5));
	if (japan)
		return R::Japan;
	if (in_closed(lon, 124.5, 130.7) && lat >= 33.0 &&
			lat < std::min(39.9 + (lon - 124.3) * 0.75, 42.3))
		return R::Korea;
	const bool siberia = (lon >= 131.0 && lat >= 42.4) || (lat >= 53.6 && lon >= 87.5) ||
						 (lat >= 50.0 && in(lon, 87.5, 119.5)) ||
						 (lat >= 49.6 && in(lon, 127.6, 131.0));
	const bool steppe = (in(lon, 45.0, 87.5) && lat >= 49.2) ||
						(in(lon, 45.0, 53.0) && lat >= 45.0) ||
						(in(lon, 53.0, 82.5) && lat >= 46.0) ||
						(in(lon, 82.5, 85.7) && lat >= 47.2);
	const bool mongolia = in(lon, 87.7, 116.0) && lat >= mongolia_south_border(lon);
	if (siberia || steppe || mongolia)
		return R::EastEurope;
	const bool central_asia = (in(lon, 45.0, 53.0) && lat >= 38.5) ||
							  (in(lon, 53.0, 73.5) && lat >= 37.4) ||
							  (in(lon, 73.5, 80.2) && lat >= 39.5) ||
							  (in(lon, 80.2, 82.5) && lat >= 45.0);
	if (central_asia)
		return R::CentralAsia;
	if (lon < -13.0 && in(lat, 27.5, 40.0))
		return R::Mediterranean;
	const double afghan_east = lat < 31.0	? 66.5
							   : lat < 33.0 ? 69.0
							   : lat < 34.6 ? 70.6
							   : lat < 36.5 ? 71.3
											: 74.9;
	const bool afghanistan = in(lat, 29.4, 38.5) && in(lon, 60.5, afghan_east);
	const bool middle_east = (lat >= 15.5 && in(lon, -18.0, 61.6)) ||
							 (lat >= 12.0 && in(lon, 42.5, 61.6)) ||
							 (lat >= -1.7 && in(lon, 43.0, 61.6));
	if ((afghanistan || middle_east) && lat < 45.0)
		return R::MiddleEast;
	const double himalaya = lon < 77.8	 ? 37.1
							: lon < 80.0 ? 35.5
							: lon < 81.0 ? 30.4
							: lon < 88.2 ? 30.4 - (lon - 81.0) * 0.34
										 : 28.0;
	const double burma = lat < 24.0 ? 92.6 : lat < 27.0 ? 94.6 : 97.5;
	if (in(lon, 60.8, burma) && lat >= -1.0 && lat < himalaya)
		return R::SouthAsia;
	const double china_south = lon < 97.5	 ? 28.5
							   : lon < 100.5 ? 24.0
							   : lon < 106.5 ? 22.9
							   : lon < 108.5 ? 22.0
							   : lon < 119.0 ? 17.5
											 : 21.3;
	const bool pacific = (lon >= 141.0 || lon <= -130.0) && in(lat, -30.0, 25.0);
	if ((in(lon, 92.6, 141.0) && in(lat, -11.0, china_south)) || pacific)
		return R::SoutheastAsia;
	if (in_closed(lon, 73.5, 135.0) && in(lat, 17.5, 53.6))
		return R::EastAsia;
	if (in_closed(lon, -25.0, 58.0) && in(lat, -35.5, 15.5))
		return R::SubSaharanAfrica;
	if (in_closed(lon, -120.0, -30.0) && in(lat, -60.0, 15.0))
		return lat < -30.0 ? R::SouthernCone : R::LatinAmerica;
	if (lat >= 66.0 && in_closed(lon, -25.0, 45.0))
		return R::Nordic;
	return R::Other;
}
} // namespace

MaterialRegion material_region_at(double latitude, double longitude)
{
	if (in_closed(latitude, 15.0, 75.0) && in_closed(longitude, -170.0, -50.0))
		return north_america(latitude, longitude);
	if (in_closed(latitude, 34.0, 72.0) && in_closed(longitude, -25.0, 45.0))
		return europe(latitude, longitude);
	return elsewhere(latitude, longitude);
}

BuildingRegion building_region(MaterialRegion region)
{
	switch (region) {
	case MaterialRegion::NorthAmerica:
	case MaterialRegion::NorthAmericaSunbelt:
	case MaterialRegion::Canada:
		return BuildingRegion::NorthAmerica;
	case MaterialRegion::BritishIsles:
	case MaterialRegion::BrickNorth:
	case MaterialRegion::France:
	case MaterialRegion::Central:
	case MaterialRegion::CentralEast:
	case MaterialRegion::Nordic:
	case MaterialRegion::Mediterranean:
	case MaterialRegion::EastEurope:
		return BuildingRegion::Europe;
	default:
		return BuildingRegion::Other;
	}
}

BuildingRegion building_region_at(double latitude, double longitude)
{
	return building_region(material_region_at(latitude, longitude));
}

std::optional<RoofHabit> roof_habit(MaterialRegion region)
{
	using R = MaterialRegion;
	switch (region) {
	case R::Japan:
		return RoofHabit{.93, .35, 10.0, .4};
	case R::Korea:
		return RoofHabit{.45, .05, 10.0, .35};
	case R::EastAsia:
		return RoofHabit{.40, .10, 10.0, .25};
	case R::SoutheastAsia:
		return RoofHabit{.75, .20, 13.0, .6};
	case R::SouthAsia:
		return RoofHabit{.15, .03, 10.0, .4};
	case R::MiddleEast:
		return RoofHabit{.04, 0.0, 0.0, .3};
	case R::SubSaharanAfrica:
		return RoofHabit{.75, .25, 10.0, .65};
	case R::LatinAmerica:
		return RoofHabit{.40, .10, 10.0, .35};
	case R::SouthernCone:
		return RoofHabit{.55, .12, 10.0, .4};
	case R::EastEurope:
		return RoofHabit{.92, .55, 23.0, .6};
	case R::CentralAsia:
		return RoofHabit{.70, .15, 13.0, .5};
	case R::Australasia:
		return RoofHabit{.97, .7, 13.0, .7};
	default:
		return std::nullopt;
	}
}
} // namespace arnis::building_style
