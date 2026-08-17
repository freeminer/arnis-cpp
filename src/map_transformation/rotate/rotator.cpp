#include "rotator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>

namespace arnis::map_transformation::rotate
{

void Rotator::operate(std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox, Ground &ground)
{
	rotate_world_with_ground(angle_degrees, elements, bbox, ground);
}

namespace
{

std::pair<double, double> rotate_point(
		double x, double z, double cx, double cz, double sin_r, double cos_r)
{
	const double dx = x - cx;
	const double dz = z - cz;
	return {dx * cos_r + dz * sin_r + cx, -dx * sin_r + dz * cos_r + cz};
}

void rotate_node(ProcessedNode &node, double cx, double cz, double sin_r, double cos_r)
{
	auto [rx, rz] = rotate_point(node.x, node.z, cx, cz, sin_r, cos_r);
	node.x = static_cast<int>(std::round(rx));
	node.z = static_cast<int>(std::round(rz));
}

}

std::pair<int, int> rotate_xz_point(
		int x, int z, double angle_degrees, const cartesian::XZBBox &xzbbox)
{
	if (std::abs(angle_degrees) < std::numeric_limits<double>::epsilon())
		return {x, z};
	const double rad = -angle_degrees * M_PI / 180.0;
	const double cx = (xzbbox.min_x() + xzbbox.max_x()) / 2.0;
	const double cz = (xzbbox.min_z() + xzbbox.max_z()) / 2.0;
	auto [rx, rz] = rotate_point(x, z, cx, cz, std::sin(rad), std::cos(rad));
	return {static_cast<int>(std::round(rx)), static_cast<int>(std::round(rz))};
}

void rotate_world(
		double angle_degrees, std::vector<ProcessedElement> &elements, cartesian::XZBBox &xzbbox)
{
	if (std::abs(angle_degrees) < std::numeric_limits<double>::epsilon())
		return;

	const double rad = -angle_degrees * M_PI / 180.0;
	const double sin_r = std::sin(rad);
	const double cos_r = std::cos(rad);
	const double cx = (xzbbox.min_x() + xzbbox.max_x()) / 2.0;
	const double cz = (xzbbox.min_z() + xzbbox.max_z()) / 2.0;

	const std::pair<double, double> corners[] = {
			{xzbbox.min_x(), xzbbox.min_z()},
			{xzbbox.min_x(), xzbbox.max_z()},
			{xzbbox.max_x(), xzbbox.min_z()},
			{xzbbox.max_x(), xzbbox.max_z()},
	};

	double min_x = std::numeric_limits<double>::infinity();
	double min_z = std::numeric_limits<double>::infinity();
	double max_x = -std::numeric_limits<double>::infinity();
	double max_z = -std::numeric_limits<double>::infinity();
	for (const auto &[x, z] : corners) {
		auto [rx, rz] = rotate_point(x, z, cx, cz, sin_r, cos_r);
		min_x = std::min(min_x, rx);
		min_z = std::min(min_z, rz);
		max_x = std::max(max_x, rx);
		max_z = std::max(max_z, rz);
	}

	for (auto &element : elements) {
		if (element.is_node()) {
			auto &node = std::get<ProcessedNode>(element);
			rotate_node(node, cx, cz, sin_r, cos_r);
		} else if (element.is_way()) {
			auto &way = std::get<ProcessedWay>(element);
			for (auto &node : way.nodes)
				rotate_node(node, cx, cz, sin_r, cos_r);
		} else if (element.is_relation()) {
			auto &rel = std::get<ProcessedRelation>(element);
			for (auto &member : rel.members)
				for (auto &node : member.way.nodes)
					rotate_node(node, cx, cz, sin_r, cos_r);
		}
	}

	xzbbox = cartesian::XZBBox::rect_from_min_max(static_cast<int>(std::floor(min_x)),
			static_cast<int>(std::floor(min_z)), static_cast<int>(std::ceil(max_x)),
			static_cast<int>(std::ceil(max_z)));
}

void rotate_world_with_ground(double angle_degrees,
		std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox, Ground &ground)
{
	if (std::abs(angle_degrees) < std::numeric_limits<double>::epsilon())
		return;
	const double rad = -angle_degrees * M_PI / 180.0;
	const double cx = (bbox.min_x() + bbox.max_x()) / 2.0;
	const double cz = (bbox.min_z() + bbox.max_z()) / 2.0;
	const Ground::RotationMask mask{cx, cz, -std::sin(rad), std::cos(rad), bbox.min_x(),
			bbox.max_x(), bbox.min_z(), bbox.max_z()};
	rotate_world(angle_degrees, elements, bbox);
	// Ground's source grids remain in their original orientation; the mask
	// inverse-transforms generated points to that source footprint.  This is
	// the important safety half of Rust's rotation flow and prevents the
	// expanded AABB corners from generating terrain outside the original map.
	ground.set_rotation_mask(mask);
}

}
