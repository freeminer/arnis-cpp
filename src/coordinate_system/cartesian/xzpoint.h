#pragma once
#include "xzvector.h"
namespace arnis::cartesian
{
struct XZPoint
{
	int x{}, z{};
	XZPoint operator+(XZVector v) const { return {x + v.dx, z + v.dz}; }
	XZPoint operator-(XZVector v) const { return {x - v.dx, z - v.dz}; }
	XZVector operator-(XZPoint p) const { return {x - p.x, z - p.z}; }
	XZPoint &operator+=(XZVector v)
	{
		x += v.dx;
		z += v.dz;
		return *this;
	}
	XZPoint &operator-=(XZVector v)
	{
		x -= v.dx;
		z -= v.dz;
		return *this;
	}
	std::string to_string() const;
};
}
