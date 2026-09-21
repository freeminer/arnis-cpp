#include "rotator.h"
#include "../../elevation/elevation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>
#include <cstdint>
#include <utility>

namespace arnis::map_transformation::rotate
{

void Rotator::operate(
		std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox, Ground &ground)
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

void rotate_world(double angle_degrees, std::vector<ProcessedElement> &elements,
		cartesian::XZBBox &xzbbox)
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
	const auto original_bbox = bbox;
	const bool had_elevation = ground.elevation_enabled;
	const bool had_land_cover = ground.has_land_cover();
	const bool had_canopy = ground.has_canopy();
	const double rad = -angle_degrees * M_PI / 180.0;
	const double cx = (bbox.min_x() + bbox.max_x()) / 2.0;
	const double cz = (bbox.min_z() + bbox.max_z()) / 2.0;
	const Ground::RotationMask mask{cx, cz, -std::sin(rad), std::cos(rad), bbox.min_x(),
			bbox.max_x(), bbox.min_z(), bbox.max_z()};
	rotate_world(angle_degrees, elements, bbox);
	const auto new_width = static_cast<std::size_t>(bbox.max_x() - bbox.min_x() + 1);
	const auto new_height = static_cast<std::size_t>(bbox.max_z() - bbox.min_z() + 1);
	if (new_width == 0 || new_height == 0)
		return;
	const auto grid_width =
			std::min<std::size_t>(new_width, elevation::MAX_ELEVATION_GRID_DIM);
	const auto grid_height =
			std::min<std::size_t>(new_height, elevation::MAX_ELEVATION_GRID_DIM);

	// Re-sample every optional raster through the inverse rotation.  This is
	// the same source-coordinate mapping used by Rust's rotator: generated
	// columns are expressed in the enlarged output bbox, while Ground accessors
	// continue to receive coordinates relative to the original bbox.
	std::vector<std::vector<double>> heights;
	if (had_elevation)
		heights.assign(grid_height, std::vector<double>(grid_width));
	std::vector<std::vector<std::uint8_t>> cover, water;
	if (had_land_cover) {
		cover.assign(grid_height, std::vector<std::uint8_t>(grid_width));
		water.assign(grid_height, std::vector<std::uint8_t>(grid_width));
	}
	std::vector<std::uint8_t> canopy;
	if (had_canopy)
		canopy.assign(grid_width * grid_height, canopy::CANOPY_NODATA);
	for (std::size_t zi = 0; zi < grid_height; ++zi) {
		for (std::size_t xi = 0; xi < grid_width; ++xi) {
			const int wx = bbox.min_x() +
						   static_cast<int>(std::llround(
								   double(xi) / std::max<std::size_t>(1, grid_width - 1) *
								   (new_width - 1)));
			const int wz =
					bbox.min_z() +
					static_cast<int>(std::llround(
							double(zi) / std::max<std::size_t>(1, grid_height - 1) *
							(new_height - 1)));
			auto [ox, oz] = rotate_point(wx, wz, cx, cz, -std::sin(rad), std::cos(rad));
			const XZPoint source{
					static_cast<int>(std::llround(ox)) - original_bbox.min_x(),
					static_cast<int>(std::llround(oz)) - original_bbox.min_z()};
			if (had_elevation)
				heights[zi][xi] = ground.level(source);
			if (had_land_cover) {
				cover[zi][xi] = ground.cover_class(source);
				water[zi][xi] = ground.water_distance(source);
			}
			if (had_canopy) {
				canopy[zi * grid_width + xi] =
						ground.canopy_height_m(source).value_or(canopy::CANOPY_NODATA);
			}
		}
	}
	// Integer coordinate resampling introduces stair-steps. Rust smooths the
	// rotated relief, but never across water or a water-adjacent shoreline.
	if (had_elevation && grid_width > 2 && grid_height > 2)
		for (int pass = 0; pass < 3; ++pass) {
			auto previous = heights;
			for (std::size_t z = 1; z + 1 < grid_height; ++z)
				for (std::size_t x = 1; x + 1 < grid_width; ++x) {
					if (had_land_cover &&
							(cover[z][x] == land_cover::LC_WATER ||
									cover[z - 1][x] == land_cover::LC_WATER ||
									cover[z + 1][x] == land_cover::LC_WATER ||
									cover[z][x - 1] == land_cover::LC_WATER ||
									cover[z][x + 1] == land_cover::LC_WATER))
						continue;
					const double neighbours = previous[z - 1][x] + previous[z + 1][x] +
											  previous[z][x - 1] + previous[z][x + 1];
					heights[z][x] = .7 * previous[z][x] + .075 * neighbours;
				}
		}
	if (had_elevation)
		ground.set_elevation_data(
				heights, grid_width, grid_height, new_width, new_height);
	ground.set_world_dims(new_width, new_height);
	if (had_land_cover)
		ground.set_land_cover_data(cover, water, new_width, new_height);
	if (had_canopy)
		ground.set_canopy_data(
				canopy::CanopyData(std::move(canopy), grid_width, grid_height), new_width,
				new_height);
	// Ground's source grids remain in their original orientation; the mask
	// inverse-transforms generated points to that source footprint.  This is
	// the important safety half of Rust's rotation flow and prevents the
	// expanded AABB corners from generating terrain outside the original map.
	ground.set_rotation_mask(mask);
}

}
