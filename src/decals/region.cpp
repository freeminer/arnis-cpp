#include "region.h"

namespace arnis::decals
{
namespace
{
bool within(double value, double low, double high)
{
	return value >= low && value < high;
}
}

SignRegion detect_region(double lat, double lon)
{
	if (within(lat, 15.0, 85.0) && within(lon, -170.0, -50.0)) {
		const bool canada = lat >= 49.0 || (lat > 44.9 && within(lon, -84.0, -71.0)) ||
							(within(lat, 43.0, 44.9) && within(lon, -80.0, -78.0)) ||
							(lat > 44.5 && within(lon, -66.0, -52.0));
		return canada ? SignRegion::Canada : SignRegion::NorthAmerica;
	}
	if (within(lat, -48.0, -9.0) && within(lon, 110.0, 180.0))
		return SignRegion::Oceania;
	if (within(lat, 30.0, 46.0) && within(lon, 128.0, 146.5))
		return SignRegion::Japan;
	if ((within(lat, 49.9, 61.0) && within(lon, -11.0, 1.8)) ||
			(within(lat, 51.5, 55.1) && within(lon, -11.0, -5.3)))
		return SignRegion::UkIreland;
	if ((within(lat, 47.2, 55.1) && within(lon, 5.9, 15.1)) ||
			(within(lat, 45.8, 47.3) && within(lon, 5.9, 10.5)) ||
			(within(lat, 46.3, 49.0) && within(lon, 10.5, 17.2)))
		return SignRegion::Germanic;
	return SignRegion::Europe;
}

BladeStyle blade_style(SignRegion region)
{
	if (region == SignRegion::UkIreland)
		return BladeStyle::White;
	if (region == SignRegion::NorthAmerica || region == SignRegion::Canada ||
			region == SignRegion::Oceania)
		return BladeStyle::Green;
	return BladeStyle::Blue;
}
SpeedStyle speed_style(SignRegion region)
{
	return region == SignRegion::NorthAmerica ? SpeedStyle::UsPlate
		   : region == SignRegion::Canada	  ? SpeedStyle::CaPlate
											  : SpeedStyle::Disc;
}
bool default_mph(SignRegion region)
{
	return region == SignRegion::UkIreland || region == SignRegion::NorthAmerica;
}
bool drives_on_left(SignRegion region)
{
	return region == SignRegion::UkIreland || region == SignRegion::Oceania ||
		   region == SignRegion::Japan;
}
const char *metro_logo(SignRegion region)
{
	return region == SignRegion::Germanic ? "metro_u" : "metro_m";
}
}
