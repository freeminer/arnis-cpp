#include "xzpoint.h"
#include <string>
namespace arnis::cartesian
{
std::string XZPoint::to_string() const
{
	return "XZPoint(" + std::to_string(x) + ", " + std::to_string(z) + ")";
}
}
