#include "llbbox.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
namespace arnis::geographic
{
LLBBox::LLBBox(double a, double b, double c, double d) : min_(a, b), max_(c, d)
{
	if (b >= d || a >= c)
		throw std::invalid_argument("invalid geographic bbox");
}
LLBBox LLBBox::from_str(const std::string &s)
{
	std::string t = s;
	for (char &c : t)
		if (c == ',')
			c = ' ';
	std::stringstream q(t);
	double a, b, c, d;
	if (!(q >> a >> b >> c >> d))
		throw std::invalid_argument("invalid bbox string");
	return LLBBox(a, b, c, d);
}
bool LLBBox::contains(const LLPoint &p) const
{
	return p.lat() >= min_.lat() && p.lat() <= max_.lat() && p.lng() >= min_.lng() &&
		   p.lng() <= max_.lng();
}

double LLBBox::area_km2() const
{
	const double middle_lat = (min_.lat() + max_.lat()) * 0.5 * std::acos(-1.0) / 180.0;
	const double width_m = (max_.lng() - min_.lng()) * 111320.0 * std::cos(middle_lat);
	const double height_m = (max_.lat() - min_.lat()) * 111320.0;
	return std::abs(width_m * height_m) / 1000000.0;
}
}
