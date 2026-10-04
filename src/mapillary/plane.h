#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "types.h"

namespace arnis::mapillary::plane
{
inline constexpr double distinct_plane_m = .5;
inline constexpr double min_corner_deg = 30.0;
inline constexpr double max_corner_jump_m = 3.0;
inline constexpr double gap_m = 4.0;
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
/// Move a typed OSM wall onto its fitted plane, preserving the fit's endpoints
/// and identifiers. This is the wall geometry consumed by rectification.
inline ::arnis::mapillary::Wall fitted_wall(
		const ::arnis::mapillary::Wall &wall, const ::arnis::mapillary::PlaneFit &fit)
{
	auto project = [&](const std::array<double, 2> &point) {
		const double offset = fit.d - dot(fit.normal, point);
		return std::array<double, 2>{
				point[0] + offset * fit.normal[0], point[1] + offset * fit.normal[1]};
	};
	auto result = wall;
	result.a = project(fit.a_ref.value_or(wall.a));
	result.b = project(fit.b_ref.value_or(wall.b));
	result.length = std::hypot(result.b[0] - result.a[0], result.b[1] - result.a[1]);
	if (result.length < 1e-6) {
		const std::array<double, 2> tangent{-fit.normal[1], fit.normal[0]};
		result.a = project(wall.a);
		result.b = {result.a[0] + wall.length * tangent[0],
				result.a[1] + wall.length * tangent[1]};
		result.length = wall.length;
	}
	result.normal = fit.normal;
	return result;
}
inline std::array<double, 2> project_onto(
		const Fit &fit, const std::array<double, 2> &point)
{
	double off = fit.offset - dot(fit.normal, point);
	return {point[0] + off * fit.normal[0], point[1] + off * fit.normal[1]};
}
inline std::array<double, 2> project_onto(
		const ::arnis::mapillary::PlaneFit &fit, const std::array<double, 2> &point)
{
	const double off = fit.d - dot(fit.normal, point);
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

// Exact deterministic band search used by Rust plane::exact_line. At each
// normal, every maximal half-open 2*band window is examined. The rival test
// compares the candidate plane's offsets at both wall ends, so a tilted plane
// crossing the winner is not mistaken for a separate facade.
struct ExactLine
{
	std::array<double, 2> normal{};
	double offset{};
	std::size_t support{}, second{};
};
inline ExactLine exact_line(const std::vector<std::array<double, 2>> &xy,
		const std::array<double, 2> &wall_normal, const std::array<double, 2> &wall_a,
		const std::array<double, 2> &wall_b, double band, double cone_deg,
		double step_deg)
{
	if (xy.empty() || !(step_deg > 0.0))
		return {wall_normal, dot(wall_normal, wall_a), 0, 0};
	const auto steps = static_cast<std::int64_t>(std::llround(cone_deg / step_deg));
	struct Band
	{
		double offset;
		std::size_t count;
	};
	struct Row
	{
		std::array<double, 2> normal;
		std::vector<Band> bands;
	};
	std::vector<Row> rows;
	rows.reserve(static_cast<std::size_t>(2 * steps + 1));
	ExactLine best{wall_normal, 0.0, 0, 0};
	std::int64_t best_k = std::numeric_limits<std::int64_t>::max();
	for (std::int64_t k = -steps; k <= steps; ++k) {
		const double theta =
				(static_cast<double>(k) * step_deg) * 0.017453292519943295769;
		const double sn = std::sin(theta), cs = std::cos(theta);
		const std::array<double, 2> n{cs * wall_normal[0] - sn * wall_normal[1],
				sn * wall_normal[0] + cs * wall_normal[1]};
		std::vector<double> projections;
		projections.reserve(xy.size());
		for (const auto &p : xy)
			projections.push_back(dot(p, n));
		std::sort(projections.begin(), projections.end());
		Row row;
		row.normal = n;
		row.bands.reserve(projections.size());
		std::size_t end = 0, top = 0;
		double top_offset = 0.0;
		for (std::size_t begin = 0; begin < projections.size(); ++begin) {
			end = std::max(end, begin + 1);
			while (end < projections.size() &&
					projections[end] < projections[begin] + 2.0 * band)
				++end;
			const auto count = end - begin;
			const double offset = 0.5 * (projections[begin] + projections[end - 1]);
			row.bands.push_back({offset, count});
			if (count > top) {
				top = count;
				top_offset = offset;
			}
		}
		if (top > best.support ||
				(top == best.support && std::abs(k) < std::abs(best_k))) {
			best = {n, top_offset, top, 0};
			best_k = k;
		}
		rows.push_back(std::move(row));
	}
	const double base_a = best.offset - dot(best.normal, wall_a);
	const double base_b = best.offset - dot(best.normal, wall_b);
	for (const auto &row : rows) {
		if (row.bands.empty())
			continue;
		const auto n = row.normal;
		const double da = dot(n, wall_a) + base_a;
		const double db = dot(n, wall_b) + base_b;
		const double lo = std::min(da, db) - distinct_plane_m;
		const double hi = std::max(da, db) + distinct_plane_m;
		for (const auto &candidate : row.bands)
			if ((candidate.offset < lo || candidate.offset > hi) &&
					candidate.count > best.second)
				best.second = candidate.count;
	}
	return best;
}

inline std::array<double, 2> pca_line(const std::vector<std::array<double, 2>> &xy,
		const std::array<double, 2> &wall_normal, double &offset)
{
	if (xy.empty()) {
		offset = 0.0;
		return wall_normal;
	}
	double cx = 0.0, cy = 0.0;
	for (const auto &p : xy) {
		cx += p[0];
		cy += p[1];
	}
	cx /= xy.size();
	cy /= xy.size();
	double xx = 0.0, xy_cov = 0.0, yy = 0.0;
	for (const auto &p : xy) {
		const double x = p[0] - cx, y = p[1] - cy;
		xx += x * x;
		xy_cov += x * y;
		yy += y * y;
	}
	const double inv = 1.0 / xy.size();
	xx *= inv;
	xy_cov *= inv;
	yy *= inv;
	const double half = 0.5 * (xx - yy);
	const double spread = std::hypot(half, xy_cov);
	std::array<double, 2> n = half >= 0.0
									  ? std::array<double, 2>{xy_cov, -(half + spread)}
									  : std::array<double, 2>{half - spread, xy_cov};
	const double length = std::hypot(n[0], n[1]);
	if (length < 1e-300)
		n = {1.0, 0.0};
	else {
		n[0] /= length;
		n[1] /= length;
	}
	if (dot(n, wall_normal) < 0.0) {
		n[0] = -n[0];
		n[1] = -n[1];
	}
	offset = dot(n, {cx, cy});
	return n;
}

inline bool height_continuous(const std::vector<double> &z, double top)
{
	if (z.empty())
		return false;
	const double lo = std::floor(*std::min_element(z.begin(), z.end()));
	const double hi = std::ceil(top);
	if (hi - lo < 1.0)
		return true;
	const auto bins_n = static_cast<std::size_t>(hi - lo);
	std::vector<std::size_t> bins(bins_n);
	for (double value : z) {
		if (value > top)
			continue;
		std::size_t bin = value == hi ? bins_n - 1
									  : static_cast<std::size_t>(
												std::max(0.0, std::floor(value - lo)));
		if (bin < bins_n)
			++bins[bin];
	}
	std::size_t run = 0, worst = 0;
	for (auto count : bins) {
		run = count == 0 ? run + 1 : 0;
		worst = std::max(worst, run);
	}
	return static_cast<double>(worst) < gap_m;
}

inline PlaneFit fallback_plane(const ::arnis::mapillary::Wall &wall,
		bool registration_accepted = false,
		std::optional<std::string> pano_id = std::nullopt,
		std::optional<std::string> cluster_id = std::nullopt)
{
	PlaneFit fit;
	fit.wall_key = wall.key;
	fit.normal = wall.normal;
	fit.d = dot(wall.normal, wall.a);
	fit.source = registration_accepted ? PlaneSource::OsmRegistered : PlaneSource::OsmRaw;
	fit.a_ref = wall.a;
	fit.b_ref = wall.b;
	fit.pano_id = std::move(pano_id);
	fit.cluster_id = std::move(cluster_id);
	return fit;
}

inline PlaneFit fit_wall_plane(const ::arnis::mapillary::Wall &wall,
		const std::vector<std::array<double, 3>> &points, const Params &params,
		std::optional<std::string> pano_id = std::nullopt,
		std::optional<std::string> cluster_id = std::nullopt,
		bool registration_accepted = false)
{
	auto fallback = fallback_plane(wall, registration_accepted, pano_id, cluster_id);
	if (points.size() < 2)
		return fallback;
	const auto need = std::max<std::size_t>(
			30, static_cast<std::size_t>(std::ceil(4.0 * wall.length)));
	std::vector<std::array<double, 2>> xy;
	xy.reserve(points.size());
	for (const auto &p : points)
		xy.push_back({p[0], p[1]});
	const auto line = exact_line(xy, wall.normal, wall.a, wall.b, params.plane_band_m,
			params.plane_max_angle_deg, params.plane_angle_step_deg);
	auto n = line.normal;
	double d = line.offset;
	std::vector<bool> inliers(xy.size());
	for (unsigned pass = 0; pass < 2; ++pass) {
		std::vector<std::array<double, 2>> kept;
		for (std::size_t i = 0; i < xy.size(); ++i)
			if ((inliers[i] = std::abs(dot(xy[i], n) - d) < params.plane_band_m))
				kept.push_back(xy[i]);
		if (kept.size() < 2)
			break;
		n = pca_line(kept, wall.normal, d);
	}
	std::size_t count = 0;
	double squared = 0.0;
	std::vector<double> heights;
	for (std::size_t i = 0; i < xy.size(); ++i) {
		inliers[i] = std::abs(dot(xy[i], n) - d) < params.plane_band_m;
		if (!inliers[i])
			continue;
		const double residual = dot(xy[i], n) - d;
		squared += residual * residual;
		heights.push_back(points[i][2]);
		++count;
	}
	const double angle = std::acos(std::clamp(std::abs(dot(n, wall.normal)), -1.0, 1.0)) *
						 57.295779513082320876;
	const double offset = d - dot(n, wall.midpoint());
	const bool accepted = count >= need && angle < params.plane_max_angle_deg;
	PlaneFit fit = accepted ? PlaneFit{} : fallback;
	if (accepted) {
		fit.wall_key = wall.key;
		fit.normal = n;
		fit.d = d;
		fit.source = PlaneSource::Cloud;
		fit.pano_id = std::move(pano_id);
		fit.cluster_id = std::move(cluster_id);
		auto project = [&](const std::array<double, 2> &p) {
			const double delta = fit.d - dot(fit.normal, p);
			return std::array<double, 2>{
					p[0] + delta * fit.normal[0], p[1] + delta * fit.normal[1]};
		};
		fit.a_ref = project(wall.a);
		fit.b_ref = project(wall.b);
	}
	fit.n_inliers = count;
	fit.points_per_m = static_cast<double>(count) / std::max(1e-6, wall.length);
	fit.rms_m = count ? std::sqrt(squared / count) : 0.0;
	fit.angle_vs_osm_deg = angle;
	fit.offset_vs_osm_m = offset;
	fit.ambiguity =
			static_cast<double>(line.second) / std::max<std::size_t>(1, line.support);
	if (!heights.empty()) {
		std::sort(heights.begin(), heights.end());
		const double pos = 0.98 * static_cast<double>(heights.size() - 1);
		const auto lo = static_cast<std::size_t>(std::floor(pos));
		fit.z_top98 =
				lo + 1 >= heights.size()
						? heights.back()
						: heights[lo] + (pos - lo) * (heights[lo + 1] - heights[lo]);
		fit.z_continuous = height_continuous(heights, *fit.z_top98);
	}
	return fit;
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

enum class EndSource
{
	Osm,
	CloudCorner
};
struct Corners
{
	std::array<double, 2> a_ref{}, b_ref{};
	EndSource src_a{EndSource::Osm}, src_b{EndSource::Osm};
};
struct ViewOffset
{
	double dn_m{}, ds_m{}, dtheta_deg{};
};
inline std::optional<std::array<double, 2>> intersect_planes(
		const PlaneFit &p, const PlaneFit &q)
{
	const double det = p.normal[0] * q.normal[1] - p.normal[1] * q.normal[0];
	if (std::abs(det) < std::sin(min_corner_deg * 0.017453292519943295769))
		return std::nullopt;
	return std::array<double, 2>{(p.d * q.normal[1] - q.d * p.normal[1]) / det,
			(p.normal[0] * q.d - q.normal[0] * p.d) / det};
}
inline double corner_angle_deg(
		const ::arnis::mapillary::Wall &wall, const ::arnis::mapillary::Wall &other)
{
	return std::acos(std::clamp(dot(wall.tangent(), other.tangent()), -1.0, 1.0)) *
		   57.295779513082320876;
}
inline std::pair<const ::arnis::mapillary::Wall *, const ::arnis::mapillary::Wall *>
adjacent_walls(const ::arnis::mapillary::Wall &wall,
		const std::vector<::arnis::mapillary::Wall> &siblings)
{
	const ::arnis::mapillary::Wall *previous = nullptr, *next = nullptr;
	if (wall.piece == 0)
		for (const auto &other : siblings)
			if (other.key != wall.key && other.index != wall.index &&
					other.node_b == wall.node_a && other.piece + 1 == other.pieces) {
				previous = &other;
				break;
			}
	if (wall.piece + 1 == wall.pieces)
		for (const auto &other : siblings)
			if (other.key != wall.key && other.index != wall.index &&
					other.node_a == wall.node_b && other.piece == 0) {
				next = &other;
				break;
			}
	return {previous, next};
}
inline Corners refine_corners(const ::arnis::mapillary::Wall &wall, const PlaneFit &fit,
		const std::vector<::arnis::mapillary::Wall> &siblings,
		const std::function<std::optional<PlaneFit>(const ::arnis::mapillary::Wall &)>
				&adjacent_fit)
{
	Corners result{project_onto(fit, wall.a), project_onto(fit, wall.b)};
	if (fit.source != PlaneSource::Cloud)
		return result;
	const auto [previous, next] = adjacent_walls(wall, siblings);
	const std::array<
			std::tuple<bool, const ::arnis::mapillary::Wall *, std::array<double, 2>>, 2>
			ends{{{true, previous, wall.a}, {false, next, wall.b}}};
	for (const auto &[is_a, other, node] : ends) {
		if (!other || corner_angle_deg(wall, *other) <= min_corner_deg)
			continue;
		const auto other_fit = adjacent_fit(*other);
		if (!other_fit || other_fit->source != PlaneSource::Cloud ||
				other_fit->pano_id != fit.pano_id)
			continue;
		const auto intersection = intersect_planes(fit, *other_fit);
		if (!intersection || std::hypot((*intersection)[0] - node[0],
									 (*intersection)[1] - node[1]) > max_corner_jump_m)
			continue;
		if (is_a) {
			result.a_ref = *intersection;
			result.src_a = EndSource::CloudCorner;
		} else {
			result.b_ref = *intersection;
			result.src_b = EndSource::CloudCorner;
		}
	}
	return result;
}
inline std::optional<double> cloud_height(const PlaneFit &fit, double z_base)
{
	if (fit.source != PlaneSource::Cloud || fit.n_inliers < 30 || !fit.z_continuous ||
			!fit.z_top98)
		return std::nullopt;
	return *fit.z_top98 - z_base;
}
inline ViewOffset per_view_offset(const PlaneFit &frame_fit, const PlaneFit &other,
		const ::arnis::mapillary::Wall &wall)
{
	const auto midpoint = wall.midpoint();
	const double dn = (other.d - dot(other.normal, midpoint)) -
					  (frame_fit.d - dot(frame_fit.normal, midpoint));
	const auto framed = fitted_wall(wall, frame_fit);
	const auto a_other = other.a_ref.value_or(wall.a);
	const auto tangent = framed.tangent();
	const double ds = (a_other[0] - framed.a[0]) * tangent[0] +
					  (a_other[1] - framed.a[1]) * tangent[1];
	const double angle_frame = std::atan2(frame_fit.normal[1], frame_fit.normal[0]);
	const double angle_other = std::atan2(other.normal[1], other.normal[0]);
	constexpr double pi = 3.14159265358979323846;
	double delta = std::fmod(angle_other - angle_frame + pi, 2.0 * pi);
	if (delta < 0.0)
		delta += 2.0 * pi;
	delta -= pi;
	return {dn, ds, delta * 57.295779513082320876};
}
} // namespace arnis::mapillary::plane
