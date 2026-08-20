#pragma once
#include "llpoint.h"
#include <string>
namespace arnis::geographic
{
class LLBBox
{
	LLPoint min_, max_;

public:
	LLBBox(double, double, double, double);
	static LLBBox from_str(const std::string &);
	const LLPoint &min() const { return min_; }
	const LLPoint &max() const { return max_; }
	bool contains(const LLPoint &) const;
	double area_km2() const;
};
}
