#pragma once
#include "confidence.h"
#include "visibility.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
namespace arnis::mapillary
{
struct FacadeView
{
	std::size_t index{};
	ViewQuality quality{};
};

inline std::optional<FacadeView> select_facade_view(
		const std::vector<FacadeView> &views, double minimum = .10)
{
	std::optional<FacadeView> best;
	for (const auto &view : views)
		if (confidence(view.quality) >= minimum &&
				(!best || confidence(view.quality) > confidence(best->quality)))
			best = view;
	return best;
}

inline ViewQuality wall_view_quality(const Camera &camera, const Wall &wall,
		const Coverage &coverage, double distance_m, double sharpness, PoseSource pose)
{
	auto n = wall_outward(wall);
	double facing = 0;
	if (n) {
		double dx = camera.centre[0] - wall.a[0];
		double dy = camera.centre[1] - wall.a[1];
		double length = std::hypot(dx, dy);
		if (length > 1e-9)
			facing = std::max(0., (dx * (*n)[0] + dy * (*n)[1]) / length);
	}
	return {coverage.fraction(), facing, distance_m, sharpness, pose};
}
}
