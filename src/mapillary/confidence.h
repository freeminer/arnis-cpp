#pragma once
#include <algorithm>
#include <cmath>
#include "types.h"
namespace arnis::mapillary
{
inline constexpr double FACTOR_FLOOR = .05;
inline constexpr double SCALE_SUSPECT_MULT = .8;
inline constexpr double HEIGHT_CONFLICT_MULT = .7;
inline constexpr double SINGLE_VIEW_FACTOR = .7;
inline constexpr double NO_VIEW_FACTOR = .4;
struct Factors
{
	double pose{}, registration{}, plane{}, extent{}, height{}, lean{};
	double plane_gate{}, views{}, occlusion{}, image{};
	std::array<double, 10> as_array() const
	{
		return {pose, registration, plane, extent, height, lean, plane_gate, views,
				occlusion, image};
	}
	Factors clipped() const
	{
		Factors result = *this;
		for (double *value : {&result.pose, &result.registration, &result.plane,
					 &result.extent, &result.height, &result.lean, &result.plane_gate,
					 &result.views, &result.occlusion, &result.image})
			*value = std::clamp(*value, FACTOR_FLOOR, 1.0);
		return result;
	}
};
inline double registration_factor(const RegResult *registration)
{
	if (!registration)
		return .5;
	double base = .5;
	switch (registration->source) {
	case RegSource::Local:
		base = 1.;
		break;
	case RegSource::Global:
		base = .8;
		break;
	case RegSource::Unregistered:
		base = .6;
		break;
	case RegSource::NoCluster:
		base = .5;
		break;
	}
	return registration->scale_suspect ? base * SCALE_SUSPECT_MULT : base;
}
inline double plane_factor(const PlaneFit *fit)
{
	if (!fit || fit->source == PlaneSource::OsmRaw)
		return .5;
	return fit->source == PlaneSource::Cloud ? 1. : .75;
}
inline double extent_factor(const std::string &a, const std::string &b)
{
	const auto one = [](const std::string &source) {
		if (source == "cloud-corner")
			return 1.;
		if (source == "edge-joint")
			return .9;
		if (source == "edge-shift")
			return .8;
		return .6;
	};
	return .5 * (one(a) + one(b));
}
inline double height_factor(const std::string &source,
		const std::vector<std::string> &flags, const std::string &osm_source = {})
{
	const auto one = [](const std::string &value) -> double {
		if (value == "sky+cloud")
			return 1.;
		if (value == "sky")
			return .9;
		if (value == "cloud")
			return .85;
		if (value == "tag")
			return .75;
		if (value == "levels")
			return .6;
		if (value == "default")
			return .35;
		return -1.;
	};
	double result = one(source);
	if (source == "osm" || result < 0.)
		result = one(osm_source);
	if (result < 0.)
		result = .35;
	if (std::find(flags.begin(), flags.end(), "HEIGHT_CONFLICT") != flags.end())
		result *= HEIGHT_CONFLICT_MULT;
	return result;
}
inline double views_factor(double agreement_m, std::size_t count)
{
	if (!count)
		return NO_VIEW_FACTOR;
	if (count == 1)
		return SINGLE_VIEW_FACTOR;
	if (!std::isfinite(agreement_m))
		agreement_m = 2.;
	if (agreement_m <= .5)
		return 1.;
	if (agreement_m >= 2.)
		return .4;
	return 1. - .6 * (agreement_m - .5) / 1.5;
}
inline double confidence_score(const Factors &raw)
{
	const auto values = raw.clipped().as_array();
	double log_sum = 0.;
	for (const double value : values)
		log_sum += std::log(value);
	return std::exp(log_sum / values.size());
}
inline Factors factors_for(PoseSource pose, const RegResult *registration,
		const PlaneFit *plane, const WallDecision &decision,
		const std::vector<std::string> &flags, double agreement_m, std::size_t view_count,
		double unknown_share, double best_quality, double blur_factor,
		const std::string &osm_source = {}, double lean = .85, double plane_gate = .8)
{
	Factors result;
	result.pose = pose_factor(pose);
	result.registration = registration_factor(registration);
	result.plane = plane_factor(plane);
	result.extent = extent_factor(decision.source_a, decision.source_b);
	result.height = height_factor(decision.height_source, flags, osm_source);
	result.lean = lean;
	result.plane_gate = plane_gate;
	result.views = views_factor(agreement_m, view_count);
	result.occlusion = 1. - unknown_share;
	result.image = best_quality * blur_factor;
	return result.clipped();
}
inline Tier tier_for_confidence(double value, double tier_a, double tier_b, double tier_c)
{
	if (value >= tier_a)
		return Tier::A;
	if (value >= tier_b)
		return Tier::B;
	if (value >= tier_c)
		return Tier::C;
	return Tier::D;
}
inline Tier score_tier(const Factors &factors, double tier_a, double tier_b,
		double tier_c, double tier_a_max_unknown = .4, double tier_b_max_unknown = .8)
{
	Tier tier = tier_for_confidence(confidence_score(factors), tier_a, tier_b, tier_c);
	const double unknown = 1. - factors.clipped().occlusion;
	if (tier == Tier::A && unknown > tier_a_max_unknown)
		tier = Tier::B;
	if ((tier == Tier::A || tier == Tier::B) && unknown > tier_b_max_unknown)
		tier = Tier::C;
	return tier;
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
