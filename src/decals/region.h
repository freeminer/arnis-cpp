#pragma once

namespace arnis::decals
{
enum class SpeedStyle
{
	Disc,
	UsPlate,
	CaPlate
};
enum class BladeStyle
{
	Blue,
	Green,
	White
};
enum class SignRegion
{
	Europe,
	Germanic,
	UkIreland,
	NorthAmerica,
	Canada,
	Oceania,
	Japan
};

SignRegion detect_region(double latitude, double longitude);
BladeStyle blade_style(SignRegion region);
SpeedStyle speed_style(SignRegion region);
bool default_mph(SignRegion region);
bool drives_on_left(SignRegion region);
const char *metro_logo(SignRegion region);
}
