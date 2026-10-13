#include "rectangle.h"

namespace arnis::cartesian
{

std::string XZBBoxRect::to_string() const
{
	return "Rect(" + min_.to_string() + " -> " + max_.to_string() + ")";
}

} // namespace arnis::cartesian
