#pragma once
#include "choose.h"
namespace arnis::building_facades
{
enum class Run
{
	Forward,
	Mirrored
};
struct Fit
{
	Entry entry;
	double width, height, phase;
	std::pair<double, Run> map_u(double) const;
	double map_v(double) const;
};
}
