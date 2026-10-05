#include "register.h"

#include "pose.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace arnis::mapillary::registration
{
namespace
{
using XY = std::array<double, 2>;

std::vector<XY> rotate_points(
		const std::vector<XY> &points, const XY &centre, double theta)
{
	if (std::abs(theta) < 1e-12)
		return points;
	const auto radians = theta * 0.017453292519943295769;
	const auto s = std::sin(radians), c = std::cos(radians);
	std::vector<XY> result;
	result.reserve(points.size());
	for (const auto &point : points) {
		const auto x = point[0] - centre[0], y = point[1] - centre[1];
		result.push_back({c * x - s * y + centre[0], s * x + c * y + centre[1]});
	}
	return result;
}

std::vector<double> offset_grid(
		double range, double step, std::size_t *side_out = nullptr)
{
	const auto n = static_cast<std::int64_t>(std::llround(range / step));
	const auto side = static_cast<std::size_t>(2 * n + 1);
	if (side_out)
		*side_out = side;
	std::vector<double> xy;
	xy.reserve(side * side * 2);
	// Rust's meshgrid order: x varies fastest, rows are y.
	for (std::int64_t y = -n; y <= n; ++y)
		for (std::int64_t x = -n; x <= n; ++x) {
			xy.push_back(static_cast<double>(x) * step);
			xy.push_back(static_cast<double>(y) * step);
		}
	return xy;
}

std::vector<double> costs(const OutlineDT &dt, const std::vector<XY> &points,
		const std::vector<double> &offsets)
{
	std::vector<double> result(offsets.size() / 2);
	for (std::size_t i = 0; i < result.size(); ++i) {
		double total = 0.0;
		for (const auto &point : points) {
			const float distance = std::min(dt.distance({point[0] + offsets[2 * i],
													point[1] + offsets[2 * i + 1]}),
					static_cast<float>(cost_cap_m));
			const float squared = distance * distance;
			total += static_cast<double>(squared);
		}
		result[i] = total / static_cast<double>(std::max<std::size_t>(1, points.size()));
	}
	return result;
}

std::vector<std::size_t> local_minima(const std::vector<double> &values, std::size_t side)
{
	std::vector<std::size_t> result;
	for (std::size_t y = 0; y < side; ++y)
		for (std::size_t x = 0; x < side; ++x) {
			const auto centre = values[y * side + x];
			bool minimum = true;
			for (int dy = -1; dy <= 1 && minimum; ++dy)
				for (int dx = -1; dx <= 1; ++dx) {
					if (dx == 0 && dy == 0)
						continue;
					const auto nx = static_cast<std::int64_t>(x) + dx;
					const auto ny = static_cast<std::int64_t>(y) + dy;
					if (nx >= 0 && ny >= 0 && nx < static_cast<std::int64_t>(side) &&
							ny < static_cast<std::int64_t>(side) &&
							centre > values[static_cast<std::size_t>(ny) * side +
											 static_cast<std::size_t>(nx)]) {
						minimum = false;
						break;
					}
				}
			if (minimum)
				result.push_back(y * side + x);
		}
	return result;
}

std::size_t argmin(const std::vector<double> &values)
{
	return static_cast<std::size_t>(std::distance(
			values.begin(), std::min_element(values.begin(), values.end())));
}

std::size_t argmax(const std::vector<double> &values)
{
	return static_cast<std::size_t>(std::distance(
			values.begin(), std::max_element(values.begin(), values.end())));
}

std::array<double, 3> fine_search(const OutlineDT &dt, const std::vector<XY> &points,
		XY around, const Params &params)
{
	auto offsets = offset_grid(params.reg_fine_m, params.reg_step_fine);
	for (std::size_t i = 0; i < offsets.size() / 2; ++i) {
		offsets[2 * i] += around[0];
		offsets[2 * i + 1] += around[1];
	}
	const auto cost = costs(dt, points, offsets);
	const auto best = argmin(cost);
	return {offsets[2 * best], offsets[2 * best + 1], cost[best]};
}

std::optional<std::array<double, 3>> second_distinct(
		const std::vector<std::array<double, 3>> &ranked, double dx, double dy)
{
	for (const auto &entry : ranked)
		if (std::hypot(entry[0] - dx, entry[1] - dy) > distinct_min_m)
			return entry;
	return std::nullopt;
}

Registration with_global(const Registration &local, const std::vector<XY> &band,
		const OutlineDT &dt, const XY &centre,
		const std::optional<std::array<double, 3>> &global)
{
	if (!global)
		return local;
	Registration result = local;
	result.result.dx = (*global)[0];
	result.result.dy = (*global)[1];
	result.result.theta_deg = 0.0;
	result.result.inliers_after =
			inlier_share(band, dt, (*global)[0], (*global)[1], 0.0, centre);
	result.result.accepted = true;
	result.result.source = RegSource::Global;
	result.result.scale_suspect = false;
	return result;
}

std::vector<std::vector<XY>> outline_rings(const std::vector<Building> &buildings)
{
	std::vector<std::vector<XY>> result;
	for (const auto &building : buildings) {
		if (!building.ring.empty())
			result.push_back(building.ring);
		result.insert(result.end(), building.holes.begin(), building.holes.end());
	}
	return result;
}

double median(std::vector<double> values)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	const auto middle = values.size() / 2;
	std::nth_element(values.begin(), values.begin() + middle, values.end());
	const auto high = values[middle];
	if (values.size() % 2)
		return high;
	const auto low = *std::max_element(values.begin(), values.begin() + middle);
	return 0.5 * (low + high);
}

} // namespace

std::vector<std::array<double, 3>> facade_band(const sfm::Cluster &cluster,
		const std::array<double, 3> &centre, double ground_z, double radius_m,
		const std::array<double, 2> &band)
{
	std::vector<std::array<double, 3>> result;
	for (const auto &point : cluster.nearr({centre[0], centre[1]}, radius_m))
		if (point[2] >= ground_z + band[0] && point[2] <= ground_z + band[1])
			result.push_back(point);
	return result;
}

OutlineDT dt_for_cameras(const std::vector<Building> &buildings,
		const std::unordered_map<std::string, Camera> &cameras, const Params &params)
{
	XY low{std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity()};
	XY high{-std::numeric_limits<double>::infinity(),
			-std::numeric_limits<double>::infinity()};
	for (const auto &[id, camera] : cameras) {
		(void)id;
		for (std::size_t axis = 0; axis < 2; ++axis) {
			low[axis] = std::min(low[axis], camera.centre[axis]);
			high[axis] = std::max(high[axis], camera.centre[axis]);
		}
	}
	if (!std::isfinite(low[0]))
		low = high = {0.0, 0.0};
	const XY centre{0.5 * (low[0] + high[0]), 0.5 * (low[1] + high[1])};
	const auto margin =
			params.reg_radius_m + params.reg_coarse_m + params.reg_fine_m + 5.0;
	const auto size = std::max(high[0] - low[0], high[1] - low[1]) + 2.0 * margin;
	return outline_dt(outline_rings(buildings), centre, size, 0.5);
}

std::tuple<double, double, double, std::vector<std::array<double, 3>>> search_shift(
		const std::vector<XY> &points, const OutlineDT &dt, const XY &centre,
		const Params &params, double theta_deg)
{
	const auto rotated = rotate_points(points, centre, theta_deg);
	std::size_t side = 0;
	const auto offsets = offset_grid(params.reg_coarse_m, 1.0, &side);
	const auto cost = costs(dt, rotated, offsets);
	auto minima = local_minima(cost, side);
	std::stable_sort(minima.begin(), minima.end(),
			[&](auto a, auto b) { return cost[a] < cost[b]; });
	if (minima.empty())
		minima.push_back(argmin(cost));
	std::vector<std::array<double, 3>> ranked;
	ranked.reserve(minima.size());
	for (const auto index : minima)
		ranked.push_back({offsets[2 * index], offsets[2 * index + 1], cost[index]});
	const auto best = fine_search(dt, rotated,
			{offsets[2 * minima.front()], offsets[2 * minima.front() + 1]}, params);
	ranked.front() = best;
	for (std::size_t i = 1; i < ranked.size(); ++i) {
		if (std::hypot(ranked[i][0] - best[0], ranked[i][1] - best[1]) <= distinct_min_m)
			continue;
		ranked[i] = fine_search(dt, rotated, {ranked[i][0], ranked[i][1]}, params);
		break;
	}
	std::stable_sort(ranked.begin(), ranked.end(),
			[](const auto &a, const auto &b) { return a[2] < b[2]; });
	return {best[0], best[1], best[2], std::move(ranked)};
}

double inlier_share(const std::vector<XY> &points, const OutlineDT &dt, double dx,
		double dy, double theta_deg, const XY &centre)
{
	if (points.empty())
		return 0.0;
	const auto rotated = rotate_points(points, centre, theta_deg);
	std::size_t hits = 0;
	for (const auto &point : rotated)
		if (dt.distance({point[0] + dx, point[1] + dy}) < inlier_m)
			++hits;
	return static_cast<double>(hits) / static_cast<double>(rotated.size());
}

Registration register_band(const std::string &pano_id, const std::vector<XY> &band,
		std::size_t band_points, const XY &centre, const OutlineDT &dt,
		const Params &params, std::optional<std::array<double, 3>> global_shift,
		ThetaMode theta_mode)
{
	Registration result;
	result.pano_id = pano_id;
	result.result.source = RegSource::Unregistered;
	result.result.n_points = band_points;
	if (band_points < reg_min_points)
		return with_global(result, band, dt, centre, global_shift);
	result.result.inliers_before = inlier_share(band, dt, 0.0, 0.0, 0.0, centre);
	auto [dx0, dy0, cost0, ranked0] = search_shift(band, dt, centre, params, 0.0);
	double dx = dx0, dy = dy0, cost = cost0, theta = 0.0;
	auto ranked = std::move(ranked0);
	double inliers = inlier_share(band, dt, dx, dy, 0.0, centre);
	const bool sweep =
			theta_mode == ThetaMode::Always ||
			(theta_mode == ThetaMode::Auto && inliers < params.reg_min_inliers);
	if (sweep) {
		const auto steps = static_cast<std::int64_t>(
				std::llround(params.reg_theta_deg / params.reg_theta_step));
		auto best_cost = cost;
		for (std::int64_t k = -steps; k <= steps; ++k) {
			if (!k)
				continue;
			const auto candidate_theta = static_cast<double>(k) * params.reg_theta_step;
			auto [candidate_x, candidate_y, candidate_cost, candidate_ranked] =
					search_shift(band, dt, centre, params, candidate_theta);
			if (candidate_cost < best_cost) {
				best_cost = candidate_cost;
				dx = candidate_x;
				dy = candidate_y;
				theta = candidate_theta;
				ranked = std::move(candidate_ranked);
			}
		}
		cost = best_cost;
		inliers = inlier_share(band, dt, dx, dy, theta, centre);
	}
	result.second_best = second_distinct(ranked, dx, dy);
	const auto ratio = result.second_best
							   ? (*result.second_best)[2] / std::max(1e-9, cost)
							   : std::numeric_limits<double>::infinity();
	std::vector<XY> near;
	for (const auto &point : band)
		if (std::hypot(point[0] - centre[0], point[1] - centre[1]) <=
				params.reg_check_radius_m)
			near.push_back(point);
	double agreement = std::numeric_limits<double>::infinity();
	if (near.size() >= reg_min_points / 2) {
		const auto [dx25, dy25, ignored_cost, ignored_ranked] =
				search_shift(near, dt, centre, params, theta);
		(void)ignored_cost;
		(void)ignored_ranked;
		agreement = std::hypot(dx - dx25, dy - dy25);
	}
	const bool accepted =
			inliers >= params.reg_min_inliers && agreement <= params.reg_fine_m &&
			ratio >= params.reg_uniqueness && std::hypot(dx, dy) <= max_shift_m;
	result.result.dx = dx;
	result.result.dy = dy;
	result.result.theta_deg = theta;
	result.result.inliers_after = inliers;
	result.result.ambiguity_ratio = std::isfinite(ratio) ? ratio : 99.0;
	result.result.radius_agreement_m = std::isfinite(agreement) ? agreement : 99.0;
	result.result.accepted = accepted;
	result.result.source = accepted ? RegSource::Local : RegSource::Unregistered;
	return accepted ? result : with_global(result, band, dt, centre, global_shift);
}

Registration register_pano(const Camera &camera, const sfm::Cluster *cluster,
		const OutlineDT &dt, const Params &params,
		std::optional<std::array<double, 3>> global_shift, ThetaMode theta_mode)
{
	if (!cluster || cluster->points.empty()) {
		Registration result;
		result.pano_id = camera.pano_id;
		result.result.source = RegSource::NoCluster;
		return result;
	}
	const auto ground = std::isfinite(camera.ground_z)
								? camera.ground_z
								: camera.centre[2] - camera.cam_height_m;
	auto cloud = facade_band(
			*cluster, camera.centre, ground, params.reg_radius_m, params.reg_band);
	std::vector<XY> points;
	points.reserve(cloud.size());
	for (const auto &point : cloud)
		points.push_back({point[0], point[1]});
	return register_band(camera.pano_id, points, cloud.size(),
			{camera.centre[0], camera.centre[1]}, dt, params, global_shift, theta_mode);
}

Registration apply_global_fallback(const Registration &local, const Camera &camera,
		const sfm::Cluster &cluster, const OutlineDT &dt, const Params &params,
		std::optional<std::array<double, 3>> global_shift)
{
	const auto band3 = facade_band(cluster, camera.centre, camera.ground_z,
			params.reg_radius_m, params.reg_band);
	std::vector<XY> band;
	band.reserve(band3.size());
	for (const auto &point : band3)
		band.push_back({point[0], point[1]});
	auto result = with_global(
			local, band, dt, {camera.centre[0], camera.centre[1]}, global_shift);
	if (!result.result.accepted) {
		const auto old = result.result;
		result.result = {};
		result.result.inliers_before = old.inliers_before;
		result.result.inliers_after = old.inliers_after;
		result.result.ambiguity_ratio = old.ambiguity_ratio;
		result.result.radius_agreement_m = old.radius_agreement_m;
		result.result.n_points = old.n_points;
		result.result.accepted = false;
		result.result.source = RegSource::Unregistered;
	}
	return result;
}

std::optional<std::array<double, 3>> register_cluster_global(const sfm::Cluster &cluster,
		const std::vector<Building> &buildings, const std::array<double, 4> &bbox_xy,
		const Params &params, std::optional<double> ground_z,
		const OutlineDT *provided_dt)
{
	if (cluster.points.empty())
		return std::nullopt;
	if (!ground_z) {
		if (cluster.shots.empty()) {
			std::vector<double> heights;
			heights.reserve(cluster.points.size());
			for (const auto &point : cluster.points)
				heights.push_back(point[2]);
			ground_z = imgops::percentile(std::move(heights), 5.0);
		} else {
			std::vector<double> heights;
			heights.reserve(cluster.shots.size());
			for (const auto &shot : cluster.shots)
				heights.push_back(shot.centre[2]);
			ground_z = median(std::move(heights)) - params.rig_height_default_m;
		}
	}
	std::vector<XY> points;
	for (const auto &point : cluster.points)
		if (point[0] >= bbox_xy[0] - global_margin_m &&
				point[0] <= bbox_xy[2] + global_margin_m &&
				point[1] >= bbox_xy[1] - global_margin_m &&
				point[1] <= bbox_xy[3] + global_margin_m &&
				point[2] >= *ground_z + params.reg_band[0] &&
				point[2] <= *ground_z + params.reg_band[1])
			points.push_back({point[0], point[1]});
	if (points.size() < reg_min_points)
		return std::nullopt;
	OutlineDT built;
	const OutlineDT *dt = provided_dt;
	if (!dt) {
		const XY centre{0.5 * (bbox_xy[0] + bbox_xy[2]), 0.5 * (bbox_xy[1] + bbox_xy[3])};
		const auto size = std::max(bbox_xy[2] - bbox_xy[0], bbox_xy[3] - bbox_xy[1]) +
						  2.0 * (global_margin_m + global_range_m + 5.0);
		built = outline_dt(outline_rings(buildings), centre, size, 0.5);
		dt = &built;
	}
	const auto offsets = offset_grid(global_range_m, global_step_m);
	std::vector<double> shares(offsets.size() / 2, 0.0);
	std::size_t zero_hits = 0;
	for (const auto &point : points)
		if (dt->distance(point) < inlier_m)
			++zero_hits;
	for (std::size_t i = 0; i < shares.size(); ++i) {
		std::size_t hits = 0;
		for (const auto &point : points)
			if (dt->distance({point[0] + offsets[2 * i], point[1] + offsets[2 * i + 1]}) <
					inlier_m)
				++hits;
		shares[i] = static_cast<double>(hits) / static_cast<double>(points.size());
	}
	const auto best = argmax(shares);
	const auto fraction = shares[best];
	const auto dx = offsets[2 * best], dy = offsets[2 * best + 1];
	const auto zero = static_cast<double>(zero_hits) / static_cast<double>(points.size());
	if (fraction < global_min_fraction ||
			(fraction < global_min_ratio * std::max(zero, 1e-9) &&
					std::hypot(dx, dy) >= 0.5))
		return std::nullopt;
	return std::array<double, 3>{dx, dy, fraction};
}

Camera apply_registration(const Camera &camera, const RegResult &registration)
{
	Camera result = camera;
	if (registration.accepted) {
		result.centre[0] += registration.dx;
		result.centre[1] += registration.dy;
		if (std::abs(registration.theta_deg) > 1e-12)
			result.axes = pose::rotate_axes_about_z(result.axes, registration.theta_deg);
	}
	result.registration = registration;
	return result;
}

} // namespace arnis::mapillary::registration
