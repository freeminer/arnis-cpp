#pragma once
#include <algorithm>
#include <cmath>
namespace arnis::mapillary
{
enum class PoseSource
{
	Sfm,
	Sequence,
	Autolevel,
	Heading
};
inline double pose_factor(PoseSource s)
{
	switch (s) {
	case PoseSource::Sfm:
		return 1.;
	case PoseSource::Sequence:
		return .8;
	case PoseSource::Autolevel:
		return .65;
	case PoseSource::Heading:
		return .4;
	}
	return .4;
}
struct ViewQuality
{
	double coverage = 0, facing = 0, distance_m = 0, sharpness = 0;
	PoseSource pose{PoseSource::Heading};
};
inline double confidence(const ViewQuality &q)
{
	if (!std::isfinite(q.coverage) || !std::isfinite(q.facing) ||
			!std::isfinite(q.distance_m) || !std::isfinite(q.sharpness) ||
			q.distance_m <= 0)
		return 0;
	const double distance = 1. / (1. + q.distance_m / 25.);
	return std::clamp(q.coverage, 0., 1.) * std::clamp(q.facing, 0., 1.) *
		   std::clamp(q.sharpness, 0., 1.) * distance * pose_factor(q.pose);
}
inline bool better_view(const ViewQuality &a, const ViewQuality &b)
{
	return confidence(a) > confidence(b);
}
}
