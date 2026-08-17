#include "xzvector.h"
#include <string>
namespace arnis::cartesian
{
std::string XZVector::to_string() const
{
	return "XZVector(" + std::to_string(dx) + ", " + std::to_string(dz) + ")";
}
}
