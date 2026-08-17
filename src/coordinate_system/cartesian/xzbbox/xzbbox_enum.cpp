#include "xzbbox_enum.h"
#include <stdexcept>
#include <limits>
namespace arnis::cartesian
{
XZBBox XZBBox::rect_from_xz_lengths(double x, double z)
{
	if (x < 0 || z < 0 || x > std::numeric_limits<int>::max() ||
			z > std::numeric_limits<int>::max())
		throw std::invalid_argument("invalid bbox lengths");
	return XZBBox(XZBBoxRect({0, 0}, {int(x), int(z)}));
}
XZBBox XZBBox::rect_from_min_max(int a, int b, int c, int d)
{
	return XZBBox(XZBBoxRect({a, b}, {c, d}));
}
}
