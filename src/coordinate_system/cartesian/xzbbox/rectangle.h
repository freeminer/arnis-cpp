#pragma once
#include "../xzpoint.h"
#include <stdexcept>
#include <cstdint>
#include <string>
namespace arnis::cartesian
{
class XZBBoxRect
{
	XZPoint min_, max_;

public:
	XZBBoxRect(XZPoint min, XZPoint max) : min_(min), max_(max)
	{
		if (max.x < min.x || max.z < min.z)
			throw std::invalid_argument("invalid XZ bbox");
	}
	XZPoint min() const { return min_; }
	XZPoint max() const { return max_; }
	std::string to_string() const;
	std::uint32_t total_blocks_x() const { return std::uint32_t(max_.x - min_.x + 1); }
	std::uint32_t total_blocks_z() const { return std::uint32_t(max_.z - min_.z + 1); }
	std::uint64_t total_blocks() const
	{
		return std::uint64_t(total_blocks_x()) * total_blocks_z();
	}
	bool contains(XZPoint p) const
	{
		return p.x >= min_.x && p.x <= max_.x && p.z >= min_.z && p.z <= max_.z;
	}
	bool operator==(const XZBBoxRect &other) const
	{
		return min_.x == other.min_.x && min_.z == other.min_.z &&
			   max_.x == other.max_.x && max_.z == other.max_.z;
	}
	XZBBoxRect operator+(XZVector v) const { return {min_ + v, max_ + v}; }
	XZBBoxRect operator-(XZVector v) const { return {min_ - v, max_ - v}; }
	XZBBoxRect &operator+=(XZVector v)
	{
		min_ += v;
		max_ += v;
		return *this;
	}
	XZBBoxRect &operator-=(XZVector v)
	{
		min_ -= v;
		max_ -= v;
		return *this;
	}
};
}
