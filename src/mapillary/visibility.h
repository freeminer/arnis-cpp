#pragma once
#include "geometry.h"
#include <algorithm>
#include <cstddef>
#include <vector>
namespace arnis::mapillary
{
// Compact renderer-independent counterpart of Rust's wall visibility masks.
class Coverage
{
	std::size_t width_, height_;
	std::vector<unsigned char> covered_;

public:
	Coverage(std::size_t width, std::size_t height) :
			width_(width), height_(height), covered_(width * height)
	{
	}
	bool mark(double u, double v)
	{
		if (width_ == 0 || height_ == 0 || !std::isfinite(u) || !std::isfinite(v))
			return false;
		auto x = std::clamp<long>(std::lround(u), 0, long(width_ - 1));
		auto y = std::clamp<long>(std::lround(v), 0, long(height_ - 1));
		covered_[std::size_t(y) * width_ + std::size_t(x)] = 1;
		return true;
	}
	bool project_and_mark(const Camera &camera, const Wall &wall, double s, double h)
	{
		auto p = project_wall_point(camera, wall, s, h);
		return p && mark((*p)[0], (*p)[1]);
	}
	double fraction() const
	{
		if (covered_.empty())
			return 0;
		return double(std::count(covered_.begin(), covered_.end(), unsigned char{1})) /
			   double(covered_.size());
	}
	bool usable(double minimum = .10) const { return fraction() >= minimum; }
};
}
