#include "projection_spec.h"

#include "../args.h"
#include "../one_world.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace arnis::projection
{
namespace
{
int floor_div(int value, int divisor)
{
	const int q = value / divisor;
	return q - (value % divisor < 0 ? 1 : 0);
}

int ceil_div(int value, int divisor)
{
	return -floor_div(-value, divisor);
}

void check_edges(const std::array<double, 4> &edges)
{
	for (const auto edge : edges)
		if (!std::isfinite(edge))
			throw std::invalid_argument(
					"bounding box is outside the Web Mercator domain");
		else if (std::abs(edge) > WORLD_BORDER_BLOCKS)
			throw std::invalid_argument(
					"this area lies past the Minecraft world border in this world's frame");
}
}

ProjectionSpec ProjectionSpec::from_args(const ::arnis::Args &args)
{
	if (args.one_world_run)
		return {ProjectionKind::WebMercator,
				std::pair{args.one_world_run->origin_lat, args.one_world_run->origin_lon},
				args.scale, one_world::CLIP_PAD_BLOCKS};
	return {args.projection, std::nullopt, args.scale, 0};
}

WebMercatorProjection ProjectionSpec::mercator(const geographic::LLBBox &bbox) const
{
	const auto center = std::pair{(bbox.min().lat() + bbox.max().lat()) * .5,
			(bbox.min().lng() + bbox.max().lng()) * .5};
	const auto [lat, lon] = origin.value_or(center);
	return WebMercatorProjection(lat, lon, scale);
}

std::pair<coordinate_system::CoordTransformer, cartesian::XZBBoxRect>
ProjectionSpec::transformer(const geographic::LLBBox &bbox) const
{
	if (kind == ProjectionKind::Local)
		return coordinate_system::CoordTransformer::llbbox_to_xzbbox(bbox, scale);
	if (origin)
		return coordinate_system::CoordTransformer::with_web_mercator(
				bbox, scale, origin->first, origin->second);
	return coordinate_system::CoordTransformer::with_web_mercator(bbox, scale);
}

cartesian::XZBBoxRect ProjectionSpec::clip_bbox(const cartesian::XZBBoxRect &bbox) const
{
	if (clip_pad <= 0)
		return bbox;
	const auto clamp = [](std::int64_t value) {
		return static_cast<int>(std::clamp<std::int64_t>(
				value, std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
	};
	return {{clamp(std::int64_t(bbox.min().x) - clip_pad),
					clamp(std::int64_t(bbox.min().z) - clip_pad)},
			{clamp(std::int64_t(bbox.max().x) + clip_pad),
					clamp(std::int64_t(bbox.max().z) + clip_pad)}};
}

std::pair<cartesian::XZBBoxRect, geographic::LLBBox> snap_bbox_to_chunks(
		const WebMercatorProjection &projection, const geographic::LLBBox &bbox)
{
	const double west = projection.x_for_lon(bbox.min().lng());
	const double east = projection.x_for_lon(bbox.max().lng());
	const double north = projection.z_for_lat(bbox.max().lat());
	const double south = projection.z_for_lat(bbox.min().lat());
	check_edges({west, east, north, south});

	const int cx0 = floor_div(snap_edge(west, false), CHUNK_BLOCKS);
	int cx1 = ceil_div(snap_edge(east, true), CHUNK_BLOCKS);
	const int cz0 = floor_div(snap_edge(north, false), CHUNK_BLOCKS);
	int cz1 = ceil_div(snap_edge(south, true), CHUNK_BLOCKS);
	if (cx1 <= cx0)
		cx1 = cx0 + 1;
	if (cz1 <= cz0)
		cz1 = cz0 + 1;
	const std::int64_t min_x = std::int64_t(cx0) * CHUNK_BLOCKS;
	const std::int64_t max_x = std::int64_t(cx1) * CHUNK_BLOCKS;
	const std::int64_t min_z = std::int64_t(cz0) * CHUNK_BLOCKS;
	const std::int64_t max_z = std::int64_t(cz1) * CHUNK_BLOCKS;
	check_edges({double(min_x), double(max_x), double(min_z), double(max_z)});
	const cartesian::XZBBoxRect rect({static_cast<int>(min_x), static_cast<int>(min_z)},
			{static_cast<int>(max_x - 1), static_cast<int>(max_z - 1)});
	const geographic::LLBBox effective(projection.lat_for_z(double(max_z)),
			projection.lon_for_x(double(min_x)), projection.lat_for_z(double(min_z)),
			projection.lon_for_x(double(max_x)));
	return {rect, effective};
}

geographic::LLBBox llbbox_for_rect(
		const WebMercatorProjection &projection, const cartesian::XZBBoxRect &bbox)
{
	return {projection.lat_for_z(double(bbox.max().z) + 1.0),
			projection.lon_for_x(double(bbox.min().x)),
			projection.lat_for_z(double(bbox.min().z)),
			projection.lon_for_x(double(bbox.max().x) + 1.0)};
}
}
