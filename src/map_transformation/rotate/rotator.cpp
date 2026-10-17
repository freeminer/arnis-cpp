#include "rotator.h"
#include "../../ecoregion.h"
#include "../../elevation/elevation.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <cstdint>
#include <utility>
#include <cstdlib>

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

std::optional<double> compass_bearing_degrees(std::string_view value)
{
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
		value.remove_prefix(1);
	while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
		value.remove_suffix(1);
	std::string lowered(value);
	std::transform(lowered.begin(), lowered.end(), lowered.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	if (lowered == "n" || lowered == "north")
		return 0.0;
	if (lowered == "nne")
		return 22.5;
	if (lowered == "ne")
		return 45.0;
	if (lowered == "ene")
		return 67.5;
	if (lowered == "e" || lowered == "east")
		return 90.0;
	if (lowered == "ese")
		return 112.5;
	if (lowered == "se")
		return 135.0;
	if (lowered == "sse")
		return 157.5;
	if (lowered == "s" || lowered == "south")
		return 180.0;
	if (lowered == "ssw")
		return 202.5;
	if (lowered == "sw")
		return 225.0;
	if (lowered == "wsw")
		return 247.5;
	if (lowered == "w" || lowered == "west")
		return 270.0;
	if (lowered == "wnw")
		return 292.5;
	if (lowered == "nw")
		return 315.0;
	if (lowered == "nnw")
		return 337.5;
	if (lowered.empty())
		return std::nullopt;
	char *end = nullptr;
	const double degrees = std::strtod(lowered.c_str(), &end);
	if (end != lowered.c_str() + lowered.size() || !std::isfinite(degrees))
		return std::nullopt;
	return degrees;
}

void rotate_bearing_tags(tags_t &tags, double angle_degrees)
{
	auto it = tags.find("roof:direction");
	if (it == tags.end())
		return;
	const auto bearing = compass_bearing_degrees(it->second);
	if (!bearing)
		return;
	double turned = std::fmod(*bearing + angle_degrees, 360.0);
	if (!std::isfinite(turned))
		turned = 0.0;
	else if (turned < 0.0)
		turned += 360.0;
	const auto rounded = static_cast<std::int64_t>(std::llround(turned));
	it->second = std::to_string(rounded % 360);
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
			rotate_bearing_tags(node.tags, angle_degrees);
		} else if (element.is_way()) {
			auto &way = std::get<ProcessedWay>(element);
			for (auto &node : way.nodes)
				rotate_node(node, cx, cz, sin_r, cos_r);
			rotate_bearing_tags(way.tags, angle_degrees);
		} else if (element.is_relation()) {
			auto &rel = std::get<ProcessedRelation>(element);
			for (auto &member : rel.members) {
				// Arc::make_mut in the Rust implementation: relation ways can be
				// shared with other relations until a transform changes their points.
				if (member.way.use_count() != 1)
					member.way = std::make_shared<ProcessedWay>(*member.way);
				for (auto &node : member.way->nodes)
					rotate_node(node, cx, cz, sin_r, cos_r);
				rotate_bearing_tags(member.way->tags, angle_degrees);
			}
			rotate_bearing_tags(rel.tags, angle_degrees);
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
	const auto *original_ecoregions = ground.ecoregion_map();
	std::optional<ecoregion::EcoMap> materialized_ecoregions;
	// Rust Ground always owns an area-local EcoMap and rotates that map with
	// the other ground grids. Freeminer may attach the global geographic raster
	// instead; materialize the current bbox into local coordinates first so the
	// same inverse-rotation resampling applies to both host configurations.
	if (original_ecoregions && !original_ecoregions->is_local() && ground.mg) {
		const auto source_width =
				static_cast<std::int64_t>(bbox.max_x()) - bbox.min_x() + 1;
		const auto source_height =
				static_cast<std::int64_t>(bbox.max_z()) - bbox.min_z() + 1;
		if (source_width > 0 && source_height > 0 &&
				source_width <= std::numeric_limits<std::int32_t>::max() &&
				source_height <= std::numeric_limits<std::int32_t>::max()) {
			const int source_min_x = bbox.min_x();
			const int source_min_z = bbox.min_z();
			if (auto local = ecoregion::EcoMap::build(
						static_cast<std::size_t>(source_width),
						static_cast<std::size_t>(source_height),
						{source_min_x, source_min_z},
						[&ground, source_min_x, source_min_z](double x, double z) {
							const auto ll = ground.mg->pos_to_ll(
									static_cast<pos_t>(source_min_x + std::floor(x)),
									static_cast<pos_t>(source_min_z + std::floor(z)));
							return std::pair{static_cast<double>(ll.lat),
									static_cast<double>(ll.lon)};
						})) {
				materialized_ecoregions = std::move(*local);
				original_ecoregions = &*materialized_ecoregions;
			}
		}
	}
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
	if (original_ecoregions && original_ecoregions->is_local()) {
		if (auto rotated = original_ecoregions->resample(
					new_width, new_height, [&](std::int32_t x, std::int32_t z) {
						const double wx = double(bbox.min_x()) + x;
						const double wz = double(bbox.min_z()) + z;
						const auto [ox, oz] = rotate_point(
								wx, wz, cx, cz, -std::sin(rad), std::cos(rad));
						return std::pair<std::int32_t, std::int32_t>{
								static_cast<std::int32_t>(std::lround(ox)) -
										original_bbox.min_x(),
								static_cast<std::int32_t>(std::lround(oz)) -
										original_bbox.min_z()};
					}))
			ground.set_ecoregion_map(std::move(*rotated));
	}

	// Re-sample every optional raster through the inverse rotation.  This is
	// the same source-coordinate mapping used by Rust's rotator: generated
	// columns are expressed in the enlarged output bbox, while Ground accessors
	// continue to receive coordinates relative to the original bbox.
	std::vector<std::vector<double>> heights;
	std::vector<std::vector<std::uint8_t>> has_data;
	if (had_elevation) {
		heights.assign(grid_height, std::vector<double>(grid_width));
		has_data.assign(grid_height, std::vector<std::uint8_t>(grid_width, 0));
	}
	const int original_width = original_bbox.max_x() - original_bbox.min_x() + 1;
	const int original_height = original_bbox.max_z() - original_bbox.min_z() + 1;
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
			if (had_elevation) {
				heights[zi][xi] = ground.level(source);
				has_data[zi][xi] = source.x >= 0 && source.z >= 0 &&
								   (source.x < original_width) &&
								   (source.z < original_height);
			}
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
					if (!has_data[z][x])
						continue;
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
		ground.set_land_cover_data(cover, water, grid_width, grid_height);
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
