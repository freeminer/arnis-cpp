#pragma once
#include "rectangle.h"
namespace arnis::cartesian
{
class XZBBox
{
	XZBBoxRect rect_;
	explicit XZBBox(XZBBoxRect r) : rect_(r) {}

public:
	static XZBBox rect_from_xz_lengths(double, double);
	static XZBBox rect_from_min_max(int, int, int, int);
	bool contains(XZPoint p) const { return rect_.contains(p); }
	XZBBoxRect bounding_rect() const { return rect_; }
	int min_x() const { return rect_.min().x; }
	int max_x() const { return rect_.max().x; }
	int min_z() const { return rect_.min().z; }
	int max_z() const { return rect_.max().z; }
	XZBBox operator+(XZVector v) const { return XZBBox(rect_ + v); }
	XZBBox operator-(XZVector v) const { return XZBBox(rect_ - v); }
	XZBBox &operator+=(XZVector v)
	{
		rect_ += v;
		return *this;
	}
	XZBBox &operator-=(XZVector v)
	{
		rect_ -= v;
		return *this;
	}
};
}
