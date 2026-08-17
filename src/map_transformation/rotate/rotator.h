#pragma once

#include <utility>
#include <vector>
#include <string>

#include "../../floodfill_cache.h"
#include "../operator.h"

namespace arnis::map_transformation::rotate
{

void rotate_world(
		double angle_degrees, std::vector<ProcessedElement> &elements, cartesian::XZBBox &xzbbox);

struct Rotator : map_transformation::Operator
{
	double angle_degrees = 0.0;
	std::string repr() const { return "rotate " + std::to_string(angle_degrees) + "°"; }
	void operate(std::vector<ProcessedElement> &, cartesian::XZBBox &, Ground &) override;
};

std::pair<int, int> rotate_xz_point(
		int x, int z, double angle_degrees, const cartesian::XZBBox &xzbbox);

void rotate_world(
		double angle_degrees, std::vector<ProcessedElement> &elements, cartesian::XZBBox &xzbbox);
void rotate_world_with_ground(double angle_degrees,
		std::vector<ProcessedElement> &elements, cartesian::XZBBox &xzbbox, Ground &ground);

}
