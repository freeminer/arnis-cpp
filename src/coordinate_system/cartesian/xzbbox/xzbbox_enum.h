#pragma once
#include "rectangle.h"
#include "../../../fm_xz_bbox_shape.h"
#include <memory>
namespace arnis::cartesian
{
class XZBBox
{
	XZBBoxRect rect_;
	std::shared_ptr<const ::arnis::XZBBoxShape> outline_;
	explicit XZBBox(XZBBoxRect r) : rect_(r) {}

public:
	static XZBBox rect_from_xz_lengths(double, double);
	static XZBBox rect_from_min_max(int, int, int, int);
	bool contains(XZPoint p) const
	{
		return rect_.contains(p) && (!outline_ || outline_->contains(p.x, p.z));
	}
	bool covers_rect(int min_x, int min_z, int max_x, int max_z) const
	{
		if (min_x < rect_.min().x || max_x > rect_.max().x || min_z < rect_.min().z ||
				max_z > rect_.max().z)
			return false;
		return !outline_ || outline_->covers(min_x, min_z, max_x, max_z);
	}
	bool touches_rect(int min_x, int min_z, int max_x, int max_z) const
	{
		if (!outline_)
			return min_x <= rect_.max().x && max_x >= rect_.min().x &&
				   min_z <= rect_.max().z && max_z >= rect_.min().z;
		return outline_->touches(min_x, min_z, max_x, max_z);
	}
	XZBBox crop(int min_x, int min_z, int max_x, int max_z) const
	{
		XZBBoxRect rect{{min_x, min_z}, {max_x, max_z}};
		XZBBox result(rect);
		if (outline_)
			result.outline_ =
					std::make_shared<const ::arnis::XZBBoxShape>(outline_->cropped(rect));
		return result;
	}
	XZBBox with_outline(std::shared_ptr<const ::arnis::XZBBoxShape> shape) const
	{
		XZBBox result = *this;
		result.outline_ = std::move(shape);
		return result;
	}
	const std::shared_ptr<const ::arnis::XZBBoxShape> &outline() const
	{
		return outline_;
	}
	XZBBoxRect bounding_rect() const { return rect_; }
	int min_x() const { return rect_.min().x; }
	int max_x() const { return rect_.max().x; }
	int min_z() const { return rect_.min().z; }
	int max_z() const { return rect_.max().z; }
	std::pair<int, int> center_spawn() const
	{
		const XZPoint center{(min_x() + max_x()) / 2, (min_z() + max_z()) / 2};
		if (outline_)
			if (const auto point = outline_->nearest_inside(center))
				return {point->x, point->z};
		return {center.x, center.z};
	}
	std::pair<int, int> default_spawn() const
	{
		if (outline_)
			if (const auto point = outline_->first_inside())
				return {point->x, point->z};
		return {min_x() + 1, min_z() + 1};
	}
	XZBBox operator+(XZVector v) const
	{
		XZBBoxRect rect = rect_ + v;
		XZBBox result(rect);
		if (outline_)
			result.outline_ = std::make_shared<const ::arnis::XZBBoxShape>(
					outline_->translated(v.dx, v.dz, rect));
		return result;
	}
	XZBBox operator-(XZVector v) const { return *this + XZVector{-v.dx, -v.dz}; }
	XZBBox &operator+=(XZVector v)
	{
		const auto translated_rect = rect_ + v;
		if (outline_)
			outline_ = std::make_shared<const ::arnis::XZBBoxShape>(
					outline_->translated(v.dx, v.dz, translated_rect));
		rect_ = translated_rect;
		return *this;
	}
	XZBBox &operator-=(XZVector v) { return *this += XZVector{-v.dx, -v.dz}; }
};
}
