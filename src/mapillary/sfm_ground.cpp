#include "sfm.h"

#include <algorithm>
#include <cmath>

namespace arnis::mapillary::sfm
{
namespace
{
double percentile(std::vector<double> values, double q)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	std::sort(values.begin(), values.end());
	const double position = std::clamp(q, 0.0, 100.0) / 100.0 * (values.size() - 1);
	const auto lower = static_cast<std::size_t>(std::floor(position));
	const double fraction = position - lower;
	if (lower + 1 >= values.size())
		return values.back();
	return values[lower] + fraction * (values[lower + 1] - values[lower]);
}

double median(std::vector<double> values)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	return percentile(std::move(values), 50.0);
}
} // namespace

std::pair<std::optional<double>, std::size_t> ground_level(
		const Cluster &cluster, const std::array<double, 3> &centre, const Params &params)
{
	const auto points = cluster.near_indices({centre[0], centre[1]}, params.ground_r[1]);
	std::vector<double> heights;
	heights.reserve(points.size());
	for (const auto index : points) {
		const auto &point = cluster.points[index];
		const double radius = std::hypot(point[0] - centre[0], point[1] - centre[1]);
		if (radius > params.ground_r[0] && point[2] < centre[2])
			heights.push_back(point[2]);
	}
	const auto count = heights.size();
	if (count < params.ground_min_pts)
		return {std::nullopt, count};
	return {percentile(std::move(heights), params.ground_pct), count};
}

void camera_heights(std::unordered_map<std::string, Camera> &cameras,
		const std::unordered_map<std::string, const PanoMeta *> &metas,
		const std::unordered_map<std::string, Cluster> &clusters, const Params &params)
{
	std::unordered_map<std::string, std::vector<double>> sequence_heights;
	std::vector<std::string> pending;
	for (auto &[pano_id, camera] : cameras) {
		const auto cloud =
				camera.cluster_id ? clusters.find(*camera.cluster_id) : clusters.end();
		const auto ground =
				cloud != clusters.end() && camera.shot_id
						? ground_level(cloud->second, camera.centre, params).first
						: std::optional<double>{};
		if (!ground) {
			pending.push_back(pano_id);
			continue;
		}
		const auto &range = camera.is_spherical() ? params.cam_height_range
												  : params.persp_cam_height_range;
		const double height = std::clamp(camera.centre[2] - *ground, range[0], range[1]);
		camera.cam_height_m = height;
		camera.ground_z = camera.centre[2] - height;
		camera.ground_source = GroundSource::Cloud;
		if (const auto meta = metas.find(pano_id); meta != metas.end() && meta->second)
			sequence_heights[meta->second->sequence].push_back(height);
	}
	for (const auto &pano_id : pending) {
		auto camera = cameras.find(pano_id);
		if (camera == cameras.end())
			continue;
		const auto meta = metas.find(pano_id);
		const auto sequence = meta != metas.end() && meta->second
									  ? sequence_heights.find(meta->second->sequence)
									  : sequence_heights.end();
		const bool have_sequence =
				sequence != sequence_heights.end() && !sequence->second.empty();
		const double height = have_sequence ? median(sequence->second)
											: params.height_prior(camera->second.model);
		camera->second.cam_height_m = height;
		camera->second.ground_z = camera->second.centre[2] - height;
		camera->second.ground_source =
				have_sequence ? GroundSource::Sequence : GroundSource::Default;
	}
}

std::pair<double, const char *> wall_foot_z(const Cluster &cluster, const Wall &wall,
		const std::array<double, 3> &shift, double ground_z,
		const std::array<double, 2> &registered_centre, const Params &params)
{
	const auto midpoint = wall.midpoint();
	const std::array<double, 2> search_centre{
			midpoint[0] + wall.normal[0] * 0.5 * params.foot_search_m,
			midpoint[1] + wall.normal[1] * 0.5 * params.foot_search_m};
	const double radius = 0.5 * wall.length + params.foot_search_m + 2.0;
	auto points = cluster.near(
			{search_centre[0] - shift[0], search_centre[1] - shift[1]}, radius);
	if (points.empty())
		return {ground_z, z_base_pano};
	shift_points(points, shift, registered_centre);
	std::vector<double> heights;
	for (const auto &point : points) {
		const auto local = wall.sh_of(point, 0.0);
		if (local[0] >= 0.0 && local[0] <= wall.length && local[2] > 0.0 &&
				local[2] <= params.foot_search_m && point[2] >= ground_z - 2.0 &&
				point[2] <= ground_z + 1.5)
			heights.push_back(point[2]);
	}
	if (heights.size() < params.foot_min_pts)
		return {ground_z, z_base_pano};
	return {percentile(std::move(heights), params.foot_pct), z_base_foot};
}

std::vector<std::array<double, 3>> wall_points(const Cluster &cluster, const Wall &wall,
		const std::array<double, 3> &shift, double ground_z,
		const std::array<double, 2> &registered_centre, const Params &params)
{
	const auto midpoint = wall.midpoint();
	const double radius = 0.5 * wall.length + params.plane_search_m + 2.0;
	auto points = cluster.near({midpoint[0] - shift[0], midpoint[1] - shift[1]}, radius);
	shift_points(points, shift, registered_centre);
	points.erase(std::remove_if(points.begin(), points.end(),
						 [&](const auto &point) {
							 const auto local = wall.sh_of(point, 0.0);
							 return local[0] < -1.0 || local[0] > wall.length + 1.0 ||
									std::abs(local[2]) > params.plane_search_m ||
									point[2] < ground_z + params.plane_z_range[0] ||
									point[2] > ground_z + params.plane_z_range[1];
						 }),
			points.end());
	return points;
}
} // namespace arnis::mapillary::sfm
