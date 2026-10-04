#include "pipeline.h"

#include "pose.h"

#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
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
		records.emplace(image.id, &image);
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
} // namespace arnis::mapillary
