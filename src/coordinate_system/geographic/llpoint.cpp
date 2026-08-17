#include "llpoint.h"
#include <stdexcept>
namespace arnis::geographic
{
LLPoint::LLPoint(double lat, double lng) : lat_(lat), lng_(lng)
{
	if (lat < -90 || lat > 90)
		throw std::invalid_argument("latitude out of range");
	if (lng < -180 || lng > 180)
		throw std::invalid_argument("longitude out of range");
}
}
