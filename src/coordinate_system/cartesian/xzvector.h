#pragma once
#include <string>
namespace arnis::cartesian
{
struct XZVector
{
	int dx{}, dz{};
	XZVector operator+(const XZVector &o) const { return {dx + o.dx, dz + o.dz}; }
	XZVector operator-(const XZVector &o) const { return {dx - o.dx, dz - o.dz}; }
	XZVector &operator+=(const XZVector &o)
	{
		dx += o.dx;
		dz += o.dz;
		return *this;
	}
	XZVector &operator-=(const XZVector &o)
	{
		dx -= o.dx;
		dz -= o.dz;
		return *this;
	}
	std::string to_string() const;
};
}
