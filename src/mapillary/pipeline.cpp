#include "pipeline.h"

#include "pose.h"
#include "register.h"
#include "sfm.h"
#include "imgops.h"
#include "stb_image.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <utility>

namespace arnis::mapillary
{
namespace
{
SearchCell pad_pano_bounds(const SearchCell &bounds, double margin_m)
{
	constexpr double metres_per_degree_latitude = 111195.0;
	const double dlat = margin_m / metres_per_degree_latitude;
	const double radians = 3.14159265358979323846 / 180.0;
	const double cos_lat = std::max(
			1e-6, std::abs(std::cos(
						  0.5 * (bounds.min_latitude + bounds.max_latitude) * radians)));
	const double dlon = margin_m / (metres_per_degree_latitude * cos_lat);
	return {std::max(-180.0, bounds.min_longitude - dlon),
			std::max(-89.9, bounds.min_latitude - dlat),
			std::min(180.0, bounds.max_longitude + dlon),
			std::min(89.9, bounds.max_latitude + dlat)};
}
} // namespace

std::optional<Geometry> stage_geometry(const PipelineConfig &config,
		const PipelineResult &fetched, const Client &client, std::string *error)
{
	if (error)
		error->clear();
	if (config.cancelled()) {
		if (error)
			*error = "cancelled";
		return std::nullopt;
	}
	if (!fetched.osm) {
		if (error)
			*error = fetched.osm_error.value_or("OSM data is unavailable");
		return std::nullopt;
	}

	const BBox bbox{config.bounds.min_latitude, config.bounds.min_longitude,
			config.bounds.max_latitude, config.bounds.max_longitude};
	Geometry geometry;
	geometry.frame = build_frame(bbox);
	geometry.buildings =
			parse_overpass(fetched.osm->dump(), geometry.frame, config.params, bbox);
	if (geometry.buildings.empty()) {
		if (error)
			*error = "no buildings in the area";
		return std::nullopt;
	}
	geometry.walls =
			walls_from_buildings(geometry.buildings, config.params, geometry.frame, bbox);

	std::vector<PanoMeta> metas;
	metas.reserve(fetched.metas.size());
	for (const auto &meta : fetched.metas)
		if (config.params.admits(meta.camera_type))
			metas.push_back(meta);
	std::unordered_map<std::string, std::vector<const PanoMeta *>> by_sequence;
	std::unordered_map<std::string, const PanoMeta *> meta_refs;
	for (const auto &meta : metas) {
		by_sequence[meta.sequence].push_back(&meta);
		meta_refs.emplace(meta.id, &meta);
	}
	for (const auto &meta : metas) {
		const auto sequence = by_sequence.find(meta.sequence);
		const std::vector<const PanoMeta *> empty;
		geometry.cameras.emplace(
				meta.id, pose::camera_from_meta(meta, geometry.frame,
								 sequence == by_sequence.end() ? empty : sequence->second,
								 config.params));
		geometry.metas.emplace(meta.id, meta);
	}

	// Cluster downloads are deduplicated by acquisition; resolve refs to their
	// cached metadata record and load each point cloud once for all its panos.
	std::unordered_map<std::string, const cache::ImageRecord *> records;
	for (const auto &image : fetched.images)
		if (records.emplace(image.id, &image).second)
			geometry.image_records.emplace(image.id, image);
	for (const auto &reference : fetched.clusters) {
		if (config.cancelled()) {
			if (error)
				*error = "cancelled";
			return std::nullopt;
		}
		const auto record = records.find(reference.pano_id);
		if (record == records.end())
			continue;
		std::string cluster_error;
		auto document = client.cluster_document(*record->second, &cluster_error);
		if (!document) {
			continue; // Match Rust's non-fatal per-cluster download/parse failure.
		}
		auto cluster = sfm::load_cluster(
				*document, reference.id, geometry.frame, &cluster_error);
		if (!cluster)
			continue;
		std::vector<const PanoMeta *> cluster_metas;
		for (const auto &meta : metas)
			if (meta.cluster_id && *meta.cluster_id == reference.id)
				cluster_metas.push_back(&meta);
		const auto shot_map = sfm::map_shots(*cluster, cluster_metas, config.params);
		(void)sfm::metric_check(
				*cluster, cluster_metas, shot_map, geometry.frame, config.params);
		sfm::apply_shot_poses(geometry.cameras, *cluster, shot_map, meta_refs,
				by_sequence, config.params);
		geometry.clusters.emplace(reference.id, std::move(*cluster));
	}
	sfm::camera_heights(geometry.cameras, meta_refs, geometry.clusters, config.params);
	const auto lo = geometry.frame.to_enu(bbox.min_lon, bbox.min_lat);
	const auto hi = geometry.frame.to_enu(bbox.max_lon, bbox.max_lat);
	geometry.bbox_xy = {lo[0], lo[1], hi[0], hi[1]};
	return geometry;
}

RegistrationStage stage_registration(
		const PipelineConfig &config, const Geometry &geometry)
{
	using namespace registration;
	RegistrationStage stage;
	const auto dt = dt_for_cameras(geometry.buildings, geometry.cameras, config.params);
	for (const auto &[pano_id, camera] : geometry.cameras) {
		const sfm::Cluster *cluster = nullptr;
		if (camera.cluster_id) {
			const auto found = geometry.clusters.find(*camera.cluster_id);
			if (found != geometry.clusters.end())
				cluster = &found->second;
		}
		stage.registrations.emplace(
				pano_id, register_pano(camera, cluster, dt, config.params));
	}

	std::set<std::string> clusters_needing_global;
	for (const auto &[pano_id, camera] : geometry.cameras) {
		const auto reg = stage.registrations.find(pano_id);
		if (reg == stage.registrations.end() || reg->second.result.accepted ||
				!camera.cluster_id)
			continue;
		if (geometry.clusters.contains(*camera.cluster_id))
			clusters_needing_global.insert(*camera.cluster_id);
	}
	for (const auto &cluster_id : clusters_needing_global) {
		std::vector<double> ground_levels;
		for (const auto &[pano_id, camera] : geometry.cameras) {
			(void)pano_id;
			if (camera.cluster_id == cluster_id && std::isfinite(camera.ground_z))
				ground_levels.push_back(camera.ground_z);
		}
		std::optional<double> cluster_ground;
		if (!ground_levels.empty())
			cluster_ground = imgops::median(std::move(ground_levels));
		const auto cluster = geometry.clusters.find(cluster_id);
		std::optional<std::array<double, 3>> shift;
		if (cluster != geometry.clusters.end())
			shift = register_cluster_global(cluster->second, geometry.buildings,
					geometry.bbox_xy, config.params, cluster_ground);
		stage.global_shifts.emplace(cluster_id, std::move(shift));
	}

	for (const auto &[pano_id, camera] : geometry.cameras) {
		auto reg = stage.registrations.at(pano_id);
		const sfm::Cluster *cluster = nullptr;
		if (camera.cluster_id) {
			const auto found = geometry.clusters.find(*camera.cluster_id);
			if (found != geometry.clusters.end())
				cluster = &found->second;
		}
		if (!reg.result.accepted && cluster && camera.cluster_id) {
			stage.local_attempts.emplace(pano_id, reg);
			const auto global = stage.global_shifts.find(*camera.cluster_id);
			std::optional<std::array<double, 3>> shift;
			if (global != stage.global_shifts.end())
				shift = global->second;
			reg = apply_global_fallback(
					reg, camera, *cluster, dt, config.params, std::move(shift));
			stage.registrations[pano_id] = reg;
		}
		stage.cameras.emplace(pano_id, apply_registration(camera, reg.result));
		if (cluster)
			stage.ground_points.emplace(pano_id,
					sfm::ground_level(*cluster, camera.centre, config.params).second);
	}

	auto &stats = stage.stats;
	stats.panos = stage.registrations.size();
	std::vector<double> shifts, accepted_inliers, all_inliers;
	for (const auto &[pano_id, registration] : stage.registrations) {
		(void)pano_id;
		const auto &result = registration.result;
		switch (result.source) {
		case RegSource::Local:
			++stats.local;
			break;
		case RegSource::Global:
			++stats.global;
			break;
		case RegSource::Unregistered:
			++stats.none;
			break;
		case RegSource::NoCluster:
			++stats.no_cluster;
			break;
		}
		if (result.source != RegSource::NoCluster) {
			++stats.with_cluster;
			all_inliers.push_back(result.inliers_before);
		}
		if (result.accepted) {
			shifts.push_back(std::hypot(result.dx, result.dy));
			accepted_inliers.push_back(result.inliers_after);
			if (std::abs(result.theta_deg) > 1e-9)
				++stats.theta_used;
		}
	}
	if (stats.with_cluster) {
		stats.acceptance_rate_local =
				double(stats.local) / static_cast<double>(stats.with_cluster);
		stats.acceptance_rate_any = double(stats.local + stats.global) /
									static_cast<double>(stats.with_cluster);
	}
	if (!shifts.empty())
		stats.median_shift_m = imgops::median(std::move(shifts));
	if (!accepted_inliers.empty())
		stats.median_inliers_after = imgops::median(std::move(accepted_inliers));
	if (!all_inliers.empty())
		stats.median_inliers_before = imgops::median(std::move(all_inliers));
	for (const auto &[pano_id, registration] : stage.registrations) {
		if (registration.result.source == RegSource::Local ||
				registration.result.source == RegSource::NoCluster)
			continue;
		const auto attempt = stage.local_attempts.find(pano_id);
		const auto &local = attempt == stage.local_attempts.end()
									? registration.result
									: attempt->second.result;
		std::vector<std::string> reasons;
		if (local.n_points < reg_min_points) {
			reasons.emplace_back("only");
		} else {
			if (local.inliers_after < config.params.reg_min_inliers)
				reasons.emplace_back("inliers");
			if (local.radius_agreement_m > config.params.reg_fine_m)
				reasons.emplace_back("25/40");
			if (local.ambiguity_ratio < config.params.reg_uniqueness)
				reasons.emplace_back("ambiguous");
			if (std::hypot(local.dx, local.dy) > max_shift_m)
				reasons.emplace_back("shift");
		}
		if (reasons.empty())
			reasons.emplace_back("accepted");
		for (const auto &reason : reasons)
			++stats.local_rejection_reasons[reason];
	}
	return stage;
}

CandidateStage stage_candidates(const PipelineConfig &config, const Geometry &geometry,
		const RegistrationStage &registered)
{
	CandidateStage stage;
	stage.footprints = Footprints(geometry.buildings);
	stage.walls = geometry.walls;
	stage.by_wall = mark_reachable(
			stage.walls, registered.cameras, stage.footprints, config.params, true);
	for (const auto &wall : stage.walls)
		if (wall.reachable)
			++stage.reachable;
	for (const auto &[wall_key, candidates] : stage.by_wall) {
		(void)wall_key;
		for (const auto &candidate : candidates)
			if (!candidate.rejected_reason)
				stage.requested_images.insert(candidate.pano_id);
	}
	return stage;
}

PlaneStage stage_planes(const PipelineConfig &config, const Geometry &geometry,
		const RegistrationStage &registered, const CandidateStage &candidates)
{
	PlaneStage stage;
	std::map<std::string, const Wall *> walls;
	for (const auto &wall : candidates.walls)
		walls.emplace(wall.key, &wall);
	for (const auto &[wall_key, views] : candidates.by_wall) {
		const auto wall_it = walls.find(wall_key);
		if (wall_it == walls.end())
			continue;
		const Wall &wall = *wall_it->second;
		for (const auto &candidate : views) {
			// Rejected LOS/geometry candidates are never read by Rust's plane
			// stage. Image-gate failures happen later and still retain a fit.
			if (candidate.rejected_reason)
				continue;
			const auto camera_it = registered.cameras.find(candidate.pano_id);
			if (camera_it == registered.cameras.end())
				continue;
			const Camera &camera = camera_it->second;
			const std::string key = wall_key + "__" + candidate.pano_id;
			const sfm::Cluster *cluster = nullptr;
			if (camera.cluster_id) {
				const auto found = geometry.clusters.find(*camera.cluster_id);
				if (found != geometry.clusters.end())
					cluster = &found->second;
			}
			if (!cluster) {
				stage.per_view.emplace(
						key, plane::fallback_plane(
									 wall, false, candidate.pano_id, camera.cluster_id));
				stage.z_bases.emplace(key, std::make_pair(camera.ground_z, "pano"));
				continue;
			}
			const bool registration_accepted =
					camera.registration && camera.registration->accepted;
			const std::array<double, 3> shift = registration_accepted
														? camera.registration->shift()
														: std::array<double, 3>{};
			const std::array<double, 2> centre{camera.centre[0], camera.centre[1]};
			auto points = sfm::wall_points(
					*cluster, wall, shift, camera.ground_z, centre, config.params);
			stage.per_view.emplace(key,
					plane::fit_wall_plane(wall, points, config.params, candidate.pano_id,
							camera.cluster_id, registration_accepted));
			stage.z_bases.emplace(key, sfm::wall_foot_z(*cluster, wall, shift,
											   camera.ground_z, centre, config.params));
		}
	}
	return stage;
}

AlignmentStage stage_align_visibility(const PipelineConfig &config,
		const Geometry &geometry, const RegistrationStage &registered,
		CandidateStage candidates, const PlaneStage &plane_stage,
		const ImageLoader &load_image)
{
	AlignmentStage result;
	result.per_view = plane_stage.per_view;
	result.z_bases = plane_stage.z_bases;
	result.reachable = candidates.reachable;
	std::map<std::string, const Wall *> walls;
	std::map<std::string, std::vector<const Wall *>> siblings;
	for (const auto &wall : candidates.walls) {
		walls.emplace(wall.key, &wall);
		siblings[wall.building_key].push_back(&wall);
	}
	std::map<std::string, std::vector<std::pair<std::string, std::size_t>>> by_pano;
	std::set<std::string> candidate_panos;
	for (const auto &[wall_key, views] : candidates.by_wall)
		for (std::size_t i = 0; i < views.size(); ++i)
			if (!views[i].rejected_reason) {
				by_pano[views[i].pano_id].emplace_back(wall_key, i);
				candidate_panos.insert(views[i].pano_id);
			}

	// Rust only builds a pano's point-depth image after the registered candidate
	// set establishes that its pixels may actually be read by a wall crop.
	for (const auto &pano_id : candidate_panos) {
		const auto camera_it = registered.cameras.find(pano_id);
		if (camera_it == registered.cameras.end() || !camera_it->second.cluster_id)
			continue;
		const auto cluster_it = geometry.clusters.find(*camera_it->second.cluster_id);
		if (cluster_it == geometry.clusters.end())
			continue;
		const auto &camera = camera_it->second;
		const std::array<double, 3> shift =
				camera.registration && camera.registration->accepted
						? camera.registration->shift()
						: std::array<double, 3>{};
		result.depths.emplace(
				pano_id, sfm::depth_map(cluster_it->second, camera,
								 config.params.depth_size[0], config.params.depth_size[1],
								 config.params.depth_radius_m, shift));
	}

	for (const auto &[pano_id, pairs] : by_pano) {
		auto image = load_image ? load_image(pano_id) : std::nullopt;
		const auto camera_it = registered.cameras.find(pano_id);
		if (!image || camera_it == registered.cameras.end()) {
			for (const auto &[wall_key, index] : pairs) {
				auto &candidate = candidates.by_wall.at(wall_key).at(index);
				candidate.rejected_reason = "no_image";
				++result.candidates_no_image;
			}
			continue;
		}
		const auto meta_it = geometry.metas.find(pano_id);
		PanoMeta meta_stub;
		meta_stub.id = pano_id;
		meta_stub.quality = 1.0;
		const PanoMeta &meta =
				meta_it == geometry.metas.end() ? meta_stub : meta_it->second;
		for (const auto &[wall_key, index] : pairs) {
			const auto wall_it = walls.find(wall_key);
			if (wall_it == walls.end())
				continue;
			auto &candidate = candidates.by_wall.at(wall_key).at(index);
			const auto view_key = wall_key + "__" + pano_id;
			const auto fit_it = result.per_view.find(view_key);
			const PlaneFit fit = fit_it == result.per_view.end()
										 ? plane::fallback_plane(*wall_it->second)
										 : fit_it->second;
			const auto base_it = result.z_bases.find(view_key);
			const double z_base = base_it == result.z_bases.end()
										  ? camera_it->second.ground_z
										  : base_it->second.first;
			const auto depth_it = result.depths.find(pano_id);
			const auto preview = preview_crop(plane::fitted_wall(*wall_it->second, fit),
					camera_it->second, z_base, *image, config.params,
					PreviewOptions{
							depth_it == result.depths.end() ? nullptr : &depth_it->second,
							std::nullopt, candidate.s_vis, std::nullopt});
			apply_image_gates(
					candidate, image_gates(preview.rgb, preview.occlusion, meta));
		}
	}

	for (const auto &wall : candidates.walls) {
		auto candidate_it = candidates.by_wall.find(wall.key);
		std::vector<ViewCandidate> empty;
		auto &views =
				candidate_it == candidates.by_wall.end() ? empty : candidate_it->second;
		auto selected = select_views(views, registered.cameras, config.params, 3);
		const auto clean = [](const ViewCandidate &candidate) {
			if (!candidate.rejected_reason)
				return true;
			const auto &reason = *candidate.rejected_reason;
			for (const char *prefix : {"blur", "night", "dark", "exposure", "quality",
						 "occluded", "no_image"})
				if (reason.starts_with(prefix))
					return true;
			return false;
		};
		std::optional<std::string> best;
		if (!selected.empty()) {
			best = selected.front().pano_id;
		} else {
			const ViewCandidate *winner = nullptr;
			for (const auto &candidate : views)
				if (clean(candidate) &&
						(!winner || candidate.g_score > winner->g_score ||
								(candidate.g_score == winner->g_score &&
										candidate.pano_id > winner->pano_id)))
					winner = &candidate;
			if (winner)
				best = winner->pano_id;
		}
		PlaneFit fit = plane::fallback_plane(wall);
		if (best) {
			const auto fit_it = result.per_view.find(wall.key + "__" + *best);
			if (fit_it != result.per_view.end())
				fit = fit_it->second;
		}
		result.planes.emplace(wall.key, fit);
		AlignedWall aligned;
		aligned.plane = fit;
		aligned.best_pano = best;
		aligned.selected = std::move(selected);
		aligned.candidates = views;
		aligned.single_view =
				!aligned.selected.empty() &&
				aligned.selected.front().gate(GateName::Positions).has_value() &&
				!aligned.selected.front().gate(GateName::Positions)->passed;
		result.walls.emplace(wall.key, std::move(aligned));
		result.walls_with_views += !result.walls.at(wall.key).selected.empty();
	}

	for (const auto &wall : candidates.walls) {
		auto aligned_it = result.walls.find(wall.key);
		if (aligned_it == result.walls.end())
			continue;
		const auto group_it = siblings.find(wall.building_key);
		std::vector<Wall> group;
		if (group_it != siblings.end())
			for (const auto *sibling : group_it->second)
				group.push_back(*sibling);
		auto &aligned = aligned_it->second;
		const auto corners = plane::refine_corners(wall, aligned.plane, group,
				[&](const Wall &adjacent) -> std::optional<PlaneFit> {
					if (aligned.best_pano) {
						const auto same_view = result.per_view.find(
								adjacent.key + "__" + *aligned.best_pano);
						if (same_view != result.per_view.end() &&
								same_view->second.source == PlaneSource::Cloud)
							return same_view->second;
					}
					const auto frame = result.planes.find(adjacent.key);
					return frame == result.planes.end()
								   ? std::nullopt
								   : std::optional<PlaneFit>(frame->second);
				});
		aligned.corner_a = corners.src_a;
		aligned.corner_b = corners.src_b;
		aligned.plane.a_ref = corners.a_ref;
		aligned.plane.b_ref = corners.b_ref;
		result.planes[wall.key] = aligned.plane;
		const auto candidate_it = candidates.by_wall.find(wall.key);
		if (candidate_it != candidates.by_wall.end())
			for (const auto &candidate : candidate_it->second) {
				const auto key = wall.key + "__" + candidate.pano_id;
				const auto view_fit = result.per_view.find(key);
				if (view_fit != result.per_view.end())
					result.per_view_offsets.emplace(
							key, plane::per_view_offset(
										 aligned.plane, view_fit->second, wall));
			}
	}
	return result;
}

AlignmentStage stage_align_visibility(const PipelineConfig &config,
		const Geometry &geometry, const RegistrationStage &registered,
		CandidateStage candidates, const PlaneStage &planes,
		const std::map<std::string, projection::Image> &images)
{
	return stage_align_visibility(config, geometry, registered, std::move(candidates),
			planes, [&](const std::string &pano_id) -> std::optional<projection::Image> {
				const auto found = images.find(pano_id);
				return found == images.end()
							   ? std::nullopt
							   : std::optional<projection::Image>(found->second);
			});
}

std::optional<projection::Image> decode_cached_image(
		const std::filesystem::path &path, std::string *error)
{
	if (error)
		error->clear();
	std::ifstream input(path, std::ios::binary);
	if (!input) {
		if (error)
			*error = "cannot read image cache entry: " + path.string();
		return std::nullopt;
	}
	std::vector<std::uint8_t> bytes(
			(std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
	if (bytes.empty() ||
			bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
		if (error)
			*error = bytes.empty() ? "empty image cache entry: " + path.string()
								   : "image cache entry is too large: " + path.string();
		return std::nullopt;
	}
	int width = 0, height = 0, channels = 0;
	stbi_uc *decoded = stbi_load_from_memory(
			bytes.data(), static_cast<int>(bytes.size()), &width, &height, &channels, 3);
	if (!decoded || width <= 0 || height <= 0) {
		if (decoded)
			stbi_image_free(decoded);
		std::error_code ignored;
		std::filesystem::remove(path, ignored);
		if (error)
			*error = "cached image did not decode and was removed: " + path.string();
		return std::nullopt;
	}
	projection::Image result;
	result.width = static_cast<unsigned>(width);
	result.height = static_cast<unsigned>(height);
	const auto pixels =
			static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
	result.pixels.resize(pixels);
	for (std::size_t i = 0; i < pixels; ++i)
		result.pixels[i] = {decoded[3 * i], decoded[3 * i + 1], decoded[3 * i + 2]};
	stbi_image_free(decoded);
	return result;
}

AlignmentRun run_alignment(
		const Client &client, const PipelineConfig &config, const Geometry &geometry)
{
	AlignmentRun result;
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}
	result.registration = stage_registration(config, geometry);
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}
	result.candidates = stage_candidates(config, geometry, result.registration);
	result.images_requested = result.candidates.requested_images.size();
	std::map<std::string, std::filesystem::path> image_paths;
	for (const auto &pano_id : result.candidates.requested_images) {
		if (config.cancelled()) {
			result.cancelled = true;
			return result;
		}
		const auto record = geometry.image_records.find(pano_id);
		if (record == geometry.image_records.end())
			continue;
		const auto path = client.download_image(record->second, cache::ImageSize::W2048);
		if (!path)
			continue;
		image_paths.emplace(pano_id, *path);
	}
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}
	const auto planes =
			stage_planes(config, geometry, result.registration, result.candidates);
	result.alignment = stage_align_visibility(config, geometry, result.registration,
			std::move(result.candidates), planes,
			[&](const std::string &pano_id) -> std::optional<projection::Image> {
				const auto found = image_paths.find(pano_id);
				if (found == image_paths.end())
					return std::nullopt;
				auto image = decode_cached_image(found->second);
				if (image)
					++result.images_decoded;
				return image;
			});
	result.cancelled = config.cancelled();
	return result;
}

PipelineResult acquire_images(const Client &client, const PipelineConfig &config)
{
	PipelineResult result;
	// Keep the selected cache root visible to later facade stages and library
	// hosts, even when acquisition returns early (cache-only or cancelled).
	result.export_dir = config.facade_cache;
	const auto started = std::chrono::steady_clock::now();
	auto fetch_osm_data = [&] {
		if (!config.osm_fetch)
			return;
		std::string osm_error;
		result.osm = fetch_osm(*config.osm_fetch, &osm_error);
		if (!result.osm && !osm_error.empty())
			result.osm_error = std::move(osm_error);
	};
	// Cache-only runs must never fall through to the Graph client.  The full
	// facade cache reader is supplied by the host, so an empty result here is
	// preferable to violating Rust's no-network contract. Rust still fetches
	// OSM geometry in this mode, so keep that separate from Graph acquisition.
	if (config.cancelled() || config.cache_only) {
		result.cancelled = config.cancelled();
		result.cache_only = config.cache_only.has_value();
		if (!result.cancelled)
			fetch_osm_data();
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	const auto search_bounds = pad_pano_bounds(config.bounds, config.pano_margin_m);
	const auto cells = search_cells(search_bounds, config.maximum_cells);
	if (!cells) {
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	result.cells = cells->size();
	if (config.cancelled()) {
		result.cancelled = true;
		result.stats.total_seconds =
				std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
						.count();
		return result;
	}
	result.images = client.search(
			search_bounds, config.endpoint, config.access_token, config.maximum_cells);
	if (result.images.empty())
		result.failed_cells = result.cells;
	std::sort(result.images.begin(), result.images.end(),
			[](const auto &a, const auto &b) { return a.id < b.id; });
	result.images.erase(
			std::unique(result.images.begin(), result.images.end(),
					[](const auto &a, const auto &b) { return a.id == b.id; }),
			result.images.end());
	result.stats.images = result.images.size();
	result.metas.reserve(result.images.size());
	std::unordered_set<std::string> seen_clusters;
	for (const auto &image : result.images) {
		auto meta = client.metadata(image);
		if (!meta)
			continue;
		if (!config.camera_types.empty() &&
				std::find(config.camera_types.begin(), config.camera_types.end(),
						meta->camera_type) == config.camera_types.end())
			continue;
		const auto handle = image.creator.empty() ? image.creator_id : image.creator;
		const auto title = config.area_label && !config.area_label->empty()
								   ? *config.area_label
								   : "Mapillary image " + meta->id;
		result.credits.push_back({meta->id, title, image.creator,
				"https://www.mapillary.com/app/user/" + handle,
				"https://www.mapillary.com/app/?pKey=" + meta->id + "&focus=photo"});
		if (meta->cluster_id && seen_clusters.insert(*meta->cluster_id).second)
			result.clusters.push_back({*meta->cluster_id, meta->id});
		result.metas.push_back(std::move(*meta));
	}
	result.stats.clusters = result.clusters.size();
	if (config.image_sink)
		for (const auto &image : result.images)
			config.image_sink(image);
	// OSM follows image metadata, and an OSM outage must not discard imagery.
	fetch_osm_data();
	result.stats.total_seconds =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
					.count();
	result.stats.fetch_seconds = result.stats.total_seconds;
	return result;
}

AlignmentPipelineResult run_alignment_pipeline(
		const Client &client, const PipelineConfig &config)
{
	AlignmentPipelineResult output;
	const auto pipeline_started = std::chrono::steady_clock::now();
	const auto finish = [&] {
		output.acquisition.stats.total_seconds = std::chrono::duration<double>(
				std::chrono::steady_clock::now() - pipeline_started)
														 .count();
	};
	output.acquisition = acquire_images(client, config);
	if (output.acquisition.cancelled || config.cancelled()) {
		output.error = "cancelled";
		finish();
		return output;
	}
	const auto geometry_started = std::chrono::steady_clock::now();
	output.geometry = stage_geometry(config, output.acquisition, client, &output.error);
	output.acquisition.stats.geometry_seconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - geometry_started)
														.count();
	if (!output.geometry) {
		finish();
		return output;
	}
	output.acquisition.stats.buildings = output.geometry->buildings.size();
	output.acquisition.stats.walls = output.geometry->walls.size();
	const auto alignment_started = std::chrono::steady_clock::now();
	output.alignment = run_alignment(client, config, *output.geometry);
	output.acquisition.stats.align_seconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - alignment_started)
													 .count();
	if (output.alignment->cancelled) {
		output.error = "cancelled";
		finish();
		return output;
	}
	output.acquisition.stats.reachable = output.alignment->alignment.reachable;
	output.acquisition.stats.walls_with_views =
			output.alignment->alignment.walls_with_views;
	output.acquisition.stats.images_downloaded = output.alignment->images_decoded;
	finish();
	return output;
}
} // namespace arnis::mapillary
