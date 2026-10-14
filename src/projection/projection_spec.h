#pragma once

#include "../coordinate_system/cartesian/xzbbox/rectangle.h"
#include "../coordinate_system/geographic/llbbox.h"
#include "../coordinate_system/transformation.h"
#include "web_mercator.h"

#include <optional>
#include <utility>

namespace arnis
{
struct Args;
}

namespace arnis::projection
{
// One run's projection policy. Keep origin, scale and clipping in one value so
// geometry conversion and ground-grid preparation cannot silently diverge.
struct ProjectionSpec
{
	ProjectionKind kind = ProjectionKind::Local;
	std::optional<std::pair<double, double>> origin;
	double scale = 1.0;
	int clip_pad = 0;

	static ProjectionSpec from_args(const ::arnis::Args &args);
	WebMercatorProjection mercator(const geographic::LLBBox &bbox) const;
	std::pair<coordinate_system::CoordTransformer, cartesian::XZBBoxRect> transformer(
			const geographic::LLBBox &bbox) const;
	cartesian::XZBBoxRect clip_bbox(const cartesian::XZBBoxRect &bbox) const;
};

// Align a geographic request to whole Web-Mercator chunks, returning both the
// inclusive block rectangle and the geographic bbox covered by its outer edges.
std::pair<cartesian::XZBBoxRect, geographic::LLBBox> snap_bbox_to_chunks(
		const WebMercatorProjection &projection, const geographic::LLBBox &bbox);
geographic::LLBBox llbbox_for_rect(
		const WebMercatorProjection &projection, const cartesian::XZBBoxRect &bbox);

inline constexpr double WORLD_BORDER_BLOCKS = 29'999'984.0;
inline constexpr int CHUNK_BLOCKS = 16;
}
