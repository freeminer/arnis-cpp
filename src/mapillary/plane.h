#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace arnis::mapillary::plane
{
inline constexpr double distinct_plane_m = .5;
struct Wall
{
	std::string key;
	std::array<double, 2> a{}, b{}, normal{};
	double length() const { return std::hypot(b[0] - a[0], b[1] - a[1]); }
	std::array<double, 2> midpoint() const
	{
		return {(a[0] + b[0]) * .5, (a[1] + b[1]) * .5};
	}
};
enum class Source
{
	OsmRaw,
	OsmRegistered,
	Cloud
};
struct Fit
{
	std::string wall_key;
	std::array<double, 2> normal{};
	double offset{};
	Source source{Source::OsmRaw};
	std::size_t inliers{};
	double points_per_m{}, rms_m{}, angle_vs_osm_deg{}, offset_vs_osm_m{}, ambiguity{};
	std::optional<double> z_top98;
	bool z_continuous{};
	std::optional<std::array<double, 2>> a_ref, b_ref;
};
struct Parameters
{
	double band_m{.25}, max_angle_deg{10}, angle_step_deg{.25};
};
inline double dot(const std::array<double, 2> &a, const std::array<double, 2> &b)
{
	return a[0] * b[0] + a[1] * b[1];
}
inline std::array<double, 2> project_onto(
		const Fit &fit, const std::array<double, 2> &point)
{
	double off = fit.offset - dot(fit.normal, point);
	return {point[0] + off * fit.normal[0], point[1] + off * fit.normal[1]};
}
inline double angle_of(const std::array<double, 2> &normal, const Wall &wall)
{
	return std::acos(std::clamp(std::abs(dot(normal, wall.normal)), -1.0, 1.0)) *
		   57.295779513082320876;
}
inline Fit fallback(const Wall &wall, bool registered = false)
{
	Fit fit;
	fit.wall_key = wall.key;
	fit.normal = wall.normal;
	fit.offset = dot(wall.normal, wall.a);
	fit.source = registered ? Source::OsmRegistered : Source::OsmRaw;
	fit.a_ref = wall.a;
	fit.b_ref = wall.b;
	return fit;
}
inline std::array<double, 2> pca_normal(const std::vector<std::array<double, 2>> &points,
		const std::array<double, 2> &towards, double &offset)
{
	double cx = 0, cy = 0;
	for (const auto &p : points) {
		cx += p[0];
		cy += p[1];
	}
	cx /= points.size();
	cy /= points.size();
	double xx = 0, xy = 0, yy = 0;
	for (const auto &p : points) {
		double x = p[0] - cx, y = p[1] - cy;
		xx += x * x;
		xy += x * y;
		yy += y * y;
	}
	xx /= points.size();
	xy /= points.size();
	yy /= points.size();
	double half = .5 * (xx - yy), spread = std::hypot(half, xy);
	std::array<double, 2> n = half >= 0 ? std::array<double, 2>{xy, -(half + spread)}
										: std::array<double, 2>{half - spread, xy};
	double length = std::hypot(n[0], n[1]);
	if (length < 1e-300)
		n = {1, 0};
	else {
		n[0] /= length;
		n[1] /= length;
	}
	if (dot(n, towards) < 0) {
		n[0] = -n[0];
		n[1] = -n[1];
	}
	offset = n[0] * cx + n[1] * cy;
	return n;
}
inline bool continuous(const std::vector<double> &values, double top)
{
	if (values.empty())
		return false;
	double low = *std::min_element(values.begin(), values.end());
	int lo = int(std::floor(low)), hi = int(std::ceil(top));
	if (hi - lo < 1)
		return true;
	std::vector<unsigned> bins(std::size_t(hi - lo));
	for (double z : values)
		if (z <= top) {
			int i = std::min(hi - lo - 1, int(std::floor(z)) - lo);
			if (i >= 0)
				++bins[std::size_t(i)];
		}
	unsigned run = 0;
	for (auto count : bins) {
		run = count ? 0 : run + 1;
		if (run >= 4)
			return false;
	}
	return true;
}
inline Fit fit_wall(const Wall &wall, const std::vector<std::array<double, 3>> &points,
		const Parameters &parameters = {}, bool registered = false)
{
	Fit result = fallback(wall, registered);
	if (points.size() < 2)
		return result;
	std::vector<std::array<double, 2>> xy;
	xy.reserve(points.size());
	for (const auto &p : points)
		xy.push_back({p[0], p[1]});
	std::size_t required = std::max<std::size_t>(
						30, std::size_t(std::ceil(4 * wall.length()))),
				support = 0, rival = 0;
	std::array<double, 2> best_n = wall.normal;
	double best_d = 0;
	long steps = std::max(
			0L, long(std::llround(parameters.max_angle_deg / parameters.angle_step_deg)));
	struct Candidate
	{
		std::array<double, 2> n;
		double d;
		std::size_t count;
	};
	std::vector<Candidate> candidates;
	for (long step = -steps; step <= steps; ++step) {
		double angle = step * parameters.angle_step_deg * .017453292519943295769,
			   s = std::sin(angle), c = std::cos(angle);
		std::array<double, 2> n{c * wall.normal[0] - s * wall.normal[1],
				s * wall.normal[0] + c * wall.normal[1]};
		std::vector<double> projection;
		for (auto p : xy)
			projection.push_back(dot(p, n));
		std::sort(projection.begin(), projection.end());
		std::size_t end = 0;
		for (std::size_t begin = 0; begin < projection.size(); ++begin) {
			end = std::max(end, begin + 1);
			while (end < projection.size() &&
					projection[end] < projection[begin] + 2 * parameters.band_m)
				++end;
			double d = .5 * (projection[begin] + projection[end - 1]);
			candidates.push_back({n, d, end - begin});
			if (end - begin > support ||
					(end - begin == support &&
							std::abs(step) <
									std::abs(long(std::llround(
											std::atan2(best_n[0] * wall.normal[1] -
															   best_n[1] * wall.normal[0],
													dot(best_n, wall.normal)) /
											parameters.angle_step_deg))))) {
				support = end - begin;
				best_n = n;
				best_d = d;
			}
		}
	}
	for (const auto &candidate : candidates) {
		double da = candidate.d - dot(candidate.n, wall.a),
			   db = candidate.d - dot(candidate.n, wall.b),
			   ba = best_d - dot(best_n, wall.a), bb = best_d - dot(best_n, wall.b);
		if ((da - ba > distinct_plane_m && db - bb > distinct_plane_m) ||
				(da - ba < -distinct_plane_m && db - bb < -distinct_plane_m))
			rival = std::max(rival, candidate.count);
	}
	std::vector<bool> keep(xy.size());
	for (unsigned pass = 0; pass < 2; ++pass) {
		for (std::size_t i = 0; i < xy.size(); ++i)
			keep[i] = std::abs(dot(xy[i], best_n) - best_d) < parameters.band_m;
		std::vector<std::array<double, 2>> included;
		for (std::size_t i = 0; i < xy.size(); ++i)
			if (keep[i])
				included.push_back(xy[i]);
		if (included.size() < 2)
			break;
		best_n = pca_normal(included, wall.normal, best_d);
	}
	std::vector<double> z;
	double sum = 0;
	for (std::size_t i = 0; i < xy.size(); ++i)
		if (keep[i]) {
			double d = dot(xy[i], best_n) - best_d;
			sum += d * d;
			z.push_back(points[i][2]);
		}
	result.inliers = z.size();
	result.points_per_m = result.inliers / std::max(1e-6, wall.length());
	result.rms_m = z.empty() ? 0 : std::sqrt(sum / z.size());
	result.angle_vs_osm_deg = angle_of(best_n, wall);
	result.offset_vs_osm_m = best_d - dot(best_n, wall.midpoint());
	result.ambiguity = double(rival) / std::max<std::size_t>(1, support);
	if (!z.empty()) {
		auto sorted = z;
		std::sort(sorted.begin(), sorted.end());
		double top = sorted[std::min(
				sorted.size() - 1, std::size_t(std::ceil(.98 * sorted.size())) - 1)];
		result.z_top98 = top;
		result.z_continuous = continuous(z, top);
	}
	if (result.inliers >= required &&
			result.angle_vs_osm_deg < parameters.max_angle_deg) {
		result.source = Source::Cloud;
		result.normal = best_n;
		result.offset = best_d;
		result.a_ref = project_onto(result, wall.a);
		result.b_ref = project_onto(result, wall.b);
	}
	return result;
}
} // namespace arnis::mapillary::plane
