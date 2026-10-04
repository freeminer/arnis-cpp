#include "sfm.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace arnis::mapillary::sfm
{
namespace
{
double median(std::vector<double> values)
{
	if (values.empty())
		return std::numeric_limits<double>::quiet_NaN();
	std::sort(values.begin(), values.end());
	const auto middle = values.size() / 2;
	return values.size() % 2 ? values[middle]
							 : 0.5 * (values[middle - 1] + values[middle]);
}
} // namespace

std::unordered_map<std::string, std::string> map_shots(const Cluster &cluster,
		const std::vector<const PanoMeta *> &metas, const Params &params)
{
	std::unordered_map<std::int64_t, std::vector<std::size_t>> by_millisecond;
	for (std::size_t i = 0; i < cluster.shots.size(); ++i)
		by_millisecond[static_cast<std::int64_t>(
							   std::llround(cluster.shots[i].capture_time * 1000.0))]
				.push_back(i);
	std::unordered_map<std::string, std::string> result;
	std::unordered_set<std::size_t> used;
	for (const auto *meta : metas) {
		if (!meta)
			continue;
		const auto exact = by_millisecond.find(meta->captured_at);
		if (exact == by_millisecond.end())
			continue;
		std::vector<std::size_t> free;
		for (const auto index : exact->second)
			if (!used.contains(index))
				free.push_back(index);
		if (free.size() == 1) {
			result.emplace(meta->id, cluster.shots[free.front()].id);
			used.insert(free.front());
		}
	}
	for (const auto *meta : metas) {
		if (!meta || result.contains(meta->id))
			continue;
		const double time = static_cast<double>(meta->captured_at) / 1000.0;
		std::vector<std::pair<double, std::size_t>> candidates;
		for (std::size_t i = 0; i < cluster.shots.size(); ++i) {
			const auto &shot = cluster.shots[i];
			const double delta = std::abs(shot.capture_time - time);
			if (shot.sequence == meta->sequence && !used.contains(i) &&
					delta <= params.shot_time_tol_s)
				candidates.emplace_back(delta, i);
		}
		std::sort(
				candidates.begin(), candidates.end(), [&](const auto &a, const auto &b) {
					if (a.first != b.first)
						return a.first < b.first;
					return cluster.shots[a.second].id < cluster.shots[b.second].id;
				});
		if (candidates.empty() ||
				(candidates.size() > 1 &&
						candidates[1].first - candidates[0].first <= 1e-3))
			continue;
		result.emplace(meta->id, cluster.shots[candidates[0].second].id);
		used.insert(candidates[0].second);
	}
	return result;
}

void apply_shot_poses(std::unordered_map<std::string, Camera> &cameras,
		const Cluster &cluster,
		const std::unordered_map<std::string, std::string> &shot_map,
		const std::unordered_map<std::string, const PanoMeta *> &metas,
		const std::unordered_map<std::string, std::vector<const PanoMeta *>> &by_sequence,
		const Params &params)
{
	for (const auto &[pano_id, shot_id] : shot_map) {
		auto camera = cameras.find(pano_id);
		const auto *shot = cluster.shot(shot_id);
		if (camera == cameras.end() || !shot)
			continue;
		auto &value = camera->second;
		value.shot_id = shot_id;
		value.cluster_id = cluster.id;
		value.centre = shot->centre;
		value.ground_z = value.centre[2] - value.cam_height_m;
		if (value.pose_source == PoseSource::Sfm)
			continue;
		const auto meta = metas.find(pano_id);
		if (meta == metas.end() || !meta->second)
			continue;
		const auto sequence = by_sequence.find(meta->second->sequence);
		const std::vector<const PanoMeta *> empty;
		const auto &sequence_metas =
				sequence == by_sequence.end() ? empty : sequence->second;
		const auto rebuilt =
				pose::camera_from_meta(*meta->second, cluster.frame, sequence_metas,
						params, std::optional<std::array<double, 3>>(shot->rotation));
		if (rebuilt.pose_source != PoseSource::Sfm)
			continue;
		value.axes = rebuilt.axes;
		value.roll_deg = rebuilt.roll_deg;
		value.pitch_deg = rebuilt.pitch_deg;
		value.pose_source = rebuilt.pose_source;
		value.pose_factor = rebuilt.pose_factor;
	}
}

std::pair<double, bool> metric_check(Cluster &cluster,
		const std::vector<const PanoMeta *> &metas,
		const std::unordered_map<std::string, std::string> &shot_map, const Frame &frame,
		const Params &params)
{
	std::unordered_map<std::string, const PanoMeta *> by_id;
	for (const auto *meta : metas)
		if (meta)
			by_id.emplace(meta->id, meta);
	std::vector<std::array<double, 2>> shot_xy, graph_xy;
	for (const auto &[pano_id, shot_id] : shot_map) {
		const auto meta = by_id.find(pano_id);
		const auto *shot = cluster.shot(shot_id);
		if (meta == by_id.end() || !shot)
			continue;
		shot_xy.push_back({shot->centre[0], shot->centre[1]});
		graph_xy.push_back(frame.to_enu(meta->second->lon, meta->second->lat));
	}
	if (shot_xy.size() < 2)
		return {std::numeric_limits<double>::quiet_NaN(), true};
	std::vector<double> ratios;
	for (std::size_t i = 0; i < shot_xy.size(); ++i)
		for (std::size_t j = i + 1; j < shot_xy.size(); ++j) {
			const double graph_distance = std::hypot(
					graph_xy[i][0] - graph_xy[j][0], graph_xy[i][1] - graph_xy[j][1]);
			if (graph_distance <= 2.0)
				continue;
			const double shot_distance = std::hypot(
					shot_xy[i][0] - shot_xy[j][0], shot_xy[i][1] - shot_xy[j][1]);
			ratios.push_back(shot_distance / graph_distance);
		}
	if (ratios.empty())
		return {std::numeric_limits<double>::quiet_NaN(), true};
	const double ratio = median(std::move(ratios));
	const bool accepted = std::abs(ratio - 1.0) <= params.metric_tol;
	cluster.metric_ratio = ratio;
	cluster.scale_ok = accepted;
	return {ratio, accepted};
}
} // namespace arnis::mapillary::sfm
