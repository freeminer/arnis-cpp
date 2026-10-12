#include "llpoint.h"
#include <cmath>
#include <stdexcept>
namespace arnis::geographic
{
LLPoint::LLPoint(double lat, double lng) : lat_(lat), lng_(lng)
{
	if (!std::isfinite(lat) || lat < -90 || lat > 90)
		throw std::invalid_argument("latitude out of range");
	if (!std::isfinite(lng) || lng < -180 || lng > 180)
		throw std::invalid_argument("longitude out of range");
}
}
