#include "pipeline.h"

#include "pose.h"
#include "register.h"
#include "sfm.h"
#include "imgops.h"
#include "stb_image.h"
#include "facade_cache.h"
#include "confidence.h"
#include "rectify.h"
#include "refine.h"
#include "fuse.h"
#include "bands.h"
#include "openings.h"
#include "../../../../../util/png.h"
#include "../retrieve_data.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <iostream>
#include <sstream>
#include <tuple>
#include <atomic>
#include <future>
#include <thread>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <utility>
#include <nlohmann/json.hpp>

#if defined(_WIN32)
#include <process.h>
#define ARNIS_PIPELINE_GETPID _getpid
#else
#include <unistd.h>
#define ARNIS_PIPELINE_GETPID getpid
#endif

namespace arnis::mapillary
{
namespace
{
using Json = nlohmann::json;
constexpr double GREEN_A = -0.04;
constexpr double SKY_B = -0.05;
constexpr double COLOUR_MAD_SCALE = 50.0;
constexpr double HIRES_MAX_DIST_M = 30.0;
constexpr std::size_t DEFAULT_IMAGE_DOWNLOAD_THREADS = 12;
constexpr const char *BLOCK_NOTE =
		"Blocks are 1 m x 1 m at Arnis scale 1 (worlds with scale != 1 must "
		"resample). Columns run from node_a (col 0, s = s_l) towards node_b; if the "
		"Arnis edge runs node_b -> node_a, flip the columns. Rows run from the top "
		"(row 0) to the wall base (last row). Run frame is ENU metres about "
		"frame.lon0/lat0 (x east, y north); Arnis world plan = [x, -y].";

bool write_bytes(
		const std::filesystem::path &path, const std::string &bytes, std::string *error)
{
	std::ofstream out(path, std::ios::binary | std::ios::trunc);
	if (!out || !out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()))) {
		if (error)
			*error = path.string() + ": failed to write file";
		return false;
	}
	return true;
}

std::string rgb_hex(const imgops::Rgb &rgb)
{
	std::ostringstream out;
	out << '#' << std::hex << std::nouppercase << std::setfill('0') << std::setw(2)
		<< static_cast<unsigned>(rgb[0]) << std::setw(2) << static_cast<unsigned>(rgb[1])
		<< std::setw(2) << static_cast<unsigned>(rgb[2]);
	return out.str();
}

std::vector<std::int64_t> wall_node_ids(const Wall &wall)
{
	if (wall.edges.empty())
		return {wall.node_a, wall.node_b};
	std::vector<std::int64_t> ids;
	ids.reserve(wall.edges.size() + 1);
	ids.push_back(wall.edges.front().node_a);
	for (const auto &edge : wall.edges)
		ids.push_back(edge.node_b);
	return ids;
}

bool write_rgba_png(const std::filesystem::path &path,
		const std::vector<std::uint8_t> &rgba, std::uint32_t width, std::uint32_t height,
		std::string *error)
{
	if (!width || !height || std::uint64_t(width) * height * 4 != rgba.size()) {
		if (error)
			*error = path.string() + ": invalid RGBA dimensions";
		return false;
	}
	const auto bytes = encodePNG(rgba.data(), width, height, 6);
	return write_bytes(path, bytes, error);
}

bool write_wall_images(
		const std::filesystem::path &dir, const WallProduct &product, std::string *error)
{
	constexpr std::size_t max_export_cells = 16 * 1024 * 1024;
	if (!cache::Layout::safe_key(product.wall_key) || !product.valid() ||
			product.cols > 16384 || product.rows > 16384 ||
			product.cell_count() > max_export_cells ||
			(!product.texture_rgba.empty() &&
					std::size_t(product.texture_width) * product.texture_height >
							max_export_cells)) {
		if (error)
			*error = product.wall_key + ": invalid facade wall product";
		return false;
	}
	std::vector<std::uint8_t> blocks(product.cell_count() * 4);
	std::vector<std::uint8_t> classes(product.cell_count() * 4);
	for (std::size_t i = 0; i < product.cell_count(); ++i) {
		for (std::size_t c = 0; c < 3; ++c) {
			blocks[i * 4 + c] = product.rgb[i][c];
			classes[i * 4 + c] = product.classes[i];
		}
		blocks[i * 4 + 3] = product.classes[i];
		classes[i * 4 + 3] = 255;
	}
	if (!write_rgba_png(dir / (product.wall_key + ".png"), blocks, product.cols,
				product.rows, error) ||
			!write_rgba_png(dir / (product.wall_key + "_cls.png"), classes, product.cols,
					product.rows, error))
		return false;
	if (!product.texture_rgba.empty() &&
			!write_rgba_png(dir / (product.wall_key + "_tex.png"), product.texture_rgba,
					product.texture_width, product.texture_height, error))
		return false;
	return true;
}

Json edge_intervals(const WallProduct &product)
{
	Json edges = Json::array();
	const double cols = product.cols;
	const std::vector<WallEdge> fallback{{std::numeric_limits<std::size_t>::max(),
			product.node_a, product.node_b, 0.0, cols}};
	const auto &source = product.edges.empty() ? fallback : product.edges;
	for (const auto &edge : source) {
		const double c0 = edge.s0 - product.col0_m;
		const double c1 = edge.s1 - product.col0_m;
		edges.push_back({{"edge_idx",
								 edge.edge_idx == std::numeric_limits<std::size_t>::max()
										 ? -1
										 : static_cast<std::int64_t>(edge.edge_idx)},
				{"node_a", edge.node_a}, {"node_b", edge.node_b}, {"s0", edge.s0},
				{"s1", edge.s1},
				{"col0", std::clamp(static_cast<std::int64_t>(std::floor(c0 + 1e-9)),
								 std::int64_t{0},
								 static_cast<std::int64_t>(product.cols))},
				{"col1", std::clamp(static_cast<std::int64_t>(std::ceil(c1 - 1e-9)),
								 std::int64_t{0},
								 static_cast<std::int64_t>(product.cols))}});
	}
	return edges;
}

Json wall_record(const WallProduct &product, const Wall &wall, const Frame &frame,
		std::int64_t osm_id)
{
	const auto a = frame.to_lonlat(wall.a), b = frame.to_lonlat(wall.b);
	const bool exported = tier_has_blocks(product.tier) && product.cols > 0;
	Json node_ids = Json::array();
	if (wall.edges.empty()) {
		node_ids.push_back(wall.node_a);
		node_ids.push_back(wall.node_b);
	} else {
		node_ids.push_back(wall.edges.front().node_a);
		for (const auto &edge : wall.edges)
			node_ids.push_back(edge.node_b);
	}
	Json views = product.views;
	return {{"key", product.wall_key}, {"building_key", product.building_key},
			{"osm_id", osm_id}, {"idx", wall.index}, {"piece", wall.piece},
			{"n_pieces", wall.pieces}, {"s_offset", wall.s_offset},
			{"node_a", product.node_a}, {"node_b", product.node_b},
			{"node_ids", node_ids}, {"a_lonlat", {a[0], a[1]}},
			{"b_lonlat", {b[0], b[1]}}, {"a_enu", wall.a}, {"b_enu", wall.b},
			{"length_m", wall.length}, {"cols", exported ? Json(product.cols) : Json()},
			{"rows", exported ? Json(product.rows) : Json()},
			{"edges", edge_intervals(product)},
			{"height_osm_m", wall.height_osm ? Json(*wall.height_osm) : Json()},
			{"height_osm_source", height_source_name(wall.height_source)},
			{"height_used_m", exported ? Json(product.height_used_m) : Json()},
			{"extent", exported ? Json{{"s_l", product.col0_m},
										  {"s_r", product.col0_m + product.cols}}
								: Json()},
			{"views", views}, {"confidence", product.confidence},
			{"tier", tier_name(product.tier)}, {"flags", product.flags},
			{"unknown_share", product.unknown_share}, {"reachable", wall.reachable},
			{"unreachable_reason", wall.unreachable_reason},
			{"png", exported ? Json(product.wall_key + ".png") : Json()},
			{"cls_png", exported ? Json(product.wall_key + "_cls.png") : Json()},
			{"tex", exported && !product.texture_rgba.empty()
							? Json(product.wall_key + "_tex.png")
							: Json()}};
}

std::pair<std::optional<imgops::Rgb>, double> building_colour(
		const std::vector<const WallProduct *> &products)
{
	struct Sample
	{
		imgops::Lab lab;
		double weight;
	};
	std::vector<Sample> samples;
	std::vector<double> confidences;
	for (const auto *product : products) {
		if (!product->cols || !product->rows)
			continue;
		const double height =
				product->height_used_m > 0.0 ? product->height_used_m : product->rows;
		const bool any_band = [&] {
			for (std::uint32_t row = 0; row < product->rows; ++row) {
				const double center = product->rows - row - .5;
				if (center >= 2.0 && center <= height - 1.0)
					return true;
			}
			return false;
		}();
		bool contributed = false;
		for (std::uint32_t row = 0; row < product->rows; ++row) {
			const double center = product->rows - row - .5;
			if (any_band && !(center >= 2.0 && center <= height - 1.0))
				continue;
			for (std::uint32_t col = 0; col < product->cols; ++col) {
				const auto i = std::size_t(row) * product->cols + col;
				if (i >= product->classes.size() || i >= product->observed.size() ||
						product->classes[i] != CLS_WALL || !product->observed[i])
					continue;
				const auto lab = imgops::srgb_to_oklab(product->rgb[i]);
				if (lab[1] < GREEN_A || (lab[2] < SKY_B && lab[0] > .6))
					continue;
				samples.push_back({lab, std::max(1e-3, product->confidence)});
				contributed = true;
			}
		}
		if (contributed)
			confidences.push_back(product->confidence);
	}
	if (samples.empty())
		return {std::nullopt, 0.0};
	imgops::Lab med{};
	for (std::size_t channel = 0; channel < 3; ++channel) {
		std::vector<std::size_t> order(samples.size());
		for (std::size_t i = 0; i < order.size(); ++i)
			order[i] = i;
		std::sort(order.begin(), order.end(), [&](auto a, auto b) {
			return samples[a].lab[channel] < samples[b].lab[channel];
		});
		double total = 0.0;
		for (const auto &sample : samples)
			total += sample.weight;
		double cumulative = 0.0;
		for (const auto i : order) {
			cumulative += samples[i].weight;
			if (cumulative >= .5 * total) {
				med[channel] = samples[i].lab[channel];
				break;
			}
		}
	}
	const auto rgb = imgops::oklab_to_rgb8(med);
	std::vector<double> deviations;
	deviations.reserve(samples.size());
	for (const auto &sample : samples) {
		const auto p = imgops::oklab_to_rgb8(sample.lab);
		const double d =
				(std::abs(int(p[0]) - int(rgb[0])) + std::abs(int(p[1]) - int(rgb[1])) +
						std::abs(int(p[2]) - int(rgb[2]))) /
				3.0;
		deviations.push_back(d);
	}
	const double mad = imgops::median(std::move(deviations));
	const double best = *std::max_element(confidences.begin(), confidences.end());
	const double confidence = std::clamp(1.0 - mad / COLOUR_MAD_SCALE, 0.0, 1.0) *
							  std::clamp(best, 0.0, 1.0);
	return {rgb, confidence};
}

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

bool write_export(const PipelineResult &result, const std::filesystem::path &dir,
		std::string *error)
{
	if (error)
		error->clear();
	std::error_code ec;
	if (!dir.parent_path().empty())
		std::filesystem::create_directories(dir.parent_path(), ec);
	if (ec) {
		if (error)
			*error = "create " + dir.parent_path().string() + ": " + ec.message();
		return false;
	}
	if (!std::filesystem::create_directory(dir, ec)) {
		if (error)
			*error = "claim " + dir.string() + ": " +
					 (ec ? ec.message() : "directory already exists");
		return false;
	}

	std::map<std::string, const Wall *> walls;
	for (const auto &wall : result.walls)
		walls.emplace(wall.key, &wall);
	std::map<std::string, std::vector<const WallProduct *>> by_building;
	for (const auto &product : result.products)
		by_building[product.building_key].push_back(&product);

	std::size_t building_count = 0, exported_wall_count = 0;
	for (const auto &building : result.buildings) {
		if (!cache::Layout::safe_key(building.key)) {
			if (error)
				*error = building.key + ": invalid facade building key";
			return false;
		}
		const auto group = by_building.find(building.key);
		if (group == by_building.end())
			continue;
		auto ordered = group->second;
		std::sort(ordered.begin(), ordered.end(), [&](const auto *a, const auto *b) {
			const auto wa = walls.find(a->wall_key), wb = walls.find(b->wall_key);
			const auto ka = wa == walls.end()
									? std::pair{std::numeric_limits<std::size_t>::max(),
											  std::size_t{0}}
									: std::pair{wa->second->index, wa->second->piece};
			const auto kb = wb == walls.end()
									? std::pair{std::numeric_limits<std::size_t>::max(),
											  std::size_t{0}}
									: std::pair{wb->second->index, wb->second->piece};
			return ka < kb;
		});
		std::vector<const WallProduct *> colour_products;
		for (const auto *product : ordered)
			if (product->tier != Tier::D && product->cols > 0)
				colour_products.push_back(product);
		const auto [colour, colour_confidence] = building_colour(colour_products);

		Json records = Json::array();
		std::size_t exported = 0;
		for (const auto *product : ordered) {
			const auto wall = walls.find(product->wall_key);
			if (wall == walls.end())
				continue;
			records.push_back(
					wall_record(*product, *wall->second, result.frame, building.osm_id));
			if (!tier_has_blocks(product->tier) || !product->cols)
				continue;
			if (!write_wall_images(dir, *product, error))
				return false;
			++exported;
		}
		if (!exported)
			continue;
		Json tiers = Json::object();
		for (const auto tier : {Tier::A, Tier::B, Tier::C, Tier::D})
			tiers[tier_name(tier)] = std::count_if(ordered.begin(), ordered.end(),
					[&](const auto *p) { return p->tier == tier; });
		Json tags = Json::object();
		static const std::set<std::string> exported_tags = {"building", "name",
				"building:levels", "height", "roof:levels", "addr:street",
				"addr:housenumber"};
		for (const auto &[key, value] : building.tags)
			if (exported_tags.contains(key))
				tags[key] = value;
		Json ring = Json::array();
		for (const auto &point : building.ring) {
			const auto ll = result.frame.to_lonlat(point);
			ring.push_back({ll[0], ll[1]});
		}
		Json building_colour_json;
		if (colour)
			building_colour_json = {{"rgb", *colour}, {"hex", rgb_hex(*colour)},
					{"confidence", colour_confidence}};
		Json member_ways = building.member_ways;
		Json node_ids = building.node_ids;
		const auto kind = osm_kind_name(building.kind);
		Json record = {{"key", building.key}, {"osm_id", building.osm_id}, {"kind", kind},
				{"way_id",
						building.kind == OsmKind::Way ? Json(building.osm_id) : Json()},
				{"relation_id", building.kind == OsmKind::Relation ? Json(building.osm_id)
																   : Json()},
				{"member_ways", member_ways},
				{"height_osm_m",
						building.height_osm ? Json(*building.height_osm) : Json()},
				{"height_source", height_source_name(building.height_source)},
				{"min_height_m", building.min_height}, {"target", building.target},
				{"tags", tags},
				{"frame",
						{{"lon0", result.frame.lon0}, {"lat0", result.frame.lat0},
								{"radius_m", EARTH_RADIUS_M},
								{"note",
										"ENU metres about lon0/lat0; Arnis world plan = [x, -y]"}}},
				{"block_note", BLOCK_NOTE},
				{"building_colour",
						building_colour_json.is_null() ? Json() : building_colour_json},
				{"ring_lonlat", ring}, {"node_ids", node_ids}, {"tiers", tiers},
				{"n_walls", records.size()}, {"walls", records},
				{"exported_walls", exported}};
		const auto path = dir / (building.key + ".json");
		if (!write_bytes(path, record.dump(), error))
			return false;
		++building_count;
		exported_wall_count += exported;
	}
	Json credits = Json::array();
	for (const auto &credit : result.credits)
		credits.push_back({{"id", credit.pano_id}, {"title", credit.title},
				{"creator", credit.creator}, {"creator_url", credit.creator_url},
				{"image_url", credit.image_url}});
	const Json manifest = {{"generator", "src/mapillary/pipeline.rs"}, {"min_tier", "B"},
			{"buildings", building_count}, {"walls", exported_wall_count},
			{"block_size_m", 1.0},
			{"cells", "floor bands + completed window lattice (bands.rs)"},
			{"layout", "<building_key>.json + <wall_key>.png (RGB colour, alpha class) "
					   "+ <wall_key>_cls.png + <wall_key>_tex.png (8 px/m)"},
			{"licence", "Mapillary imagery is CC BY-SA 4.0"}, {"credits", credits}};
	return write_bytes(dir / "manifest.json", manifest.dump(2), error);
}

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
		aligned.reachable = wall.reachable;
		aligned.unreachable_reason = wall.unreachable_reason;
		aligned.n_los_clean = static_cast<std::size_t>(
				std::count_if(views.begin(), views.end(), clean));
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

WallProduct make_no_view_product(const Wall &wall, const AlignedWall &view_record)
{
	std::string reason;
	if (!view_record.reachable)
		reason = view_record.unreachable_reason.empty() ? "unreachable"
														: view_record.unreachable_reason;
	else if (view_record.n_los_clean > 0)
		reason = "no_view_passed_image_gates";
	else
		reason = "no_line_of_sight";
	std::string reason_code = reason;
	std::transform(reason_code.begin(), reason_code.end(), reason_code.begin(),
			[](unsigned char c) { return static_cast<char>(std::toupper(c)); });

	WallProduct product;
	product.wall_key = wall.key;
	product.building_key = wall.building_key;
	product.node_a = wall.node_a;
	product.node_b = wall.node_b;
	product.piece = wall.piece;
	product.pieces = wall.pieces;
	product.edges = wall.edges;
	product.tier =
			view_record.reachable && view_record.n_los_clean > 0 ? Tier::C : Tier::D;
	product.confidence = 0.0;
	product.unknown_share = 1.0;
	product.flags =
			reason_codes(view_record.candidates, std::nullopt, std::vector<std::string>{},
					nullptr, std::vector<std::string>{std::move(reason_code)});
	return product;
}

WallProduct make_uncached_product(const Wall &wall)
{
	WallProduct product;
	product.wall_key = wall.key;
	product.building_key = wall.building_key;
	product.node_a = wall.node_a;
	product.node_b = wall.node_b;
	product.piece = wall.piece;
	product.pieces = wall.pieces;
	product.edges = wall.edges;
	product.tier = Tier::D;
	product.confidence = 0.0;
	product.unknown_share = 1.0;
	product.flags = reason_codes(std::vector<ViewCandidate>{}, std::nullopt,
			std::vector<std::string>{}, nullptr,
			std::vector<std::string>{"NOT_PRECOMPUTED"});
	return product;
}

namespace
{
struct LoadedWallView
{
	std::string pano_id;
	double score{}, distance_m{}, ds_m{}, z_base{}, ground_dz{};
	std::array<double, 2> visible_s{};
	std::optional<double> cloud_height;
	PlaneFit fit;
	rectify::LooseCrop crop;
	refine::Refinement refinement;
	projection::Image image;
};

std::optional<projection::Image> texture_image(
		const AlignmentRun &run, const std::string &pano_id, double distance_m)
{
	if (distance_m < HIRES_MAX_DIST_M) {
		const auto original = run.original_paths.find(pano_id);
		if (original != run.original_paths.end())
			if (auto image = decode_cached_image(original->second))
				return image;
	}
	const auto thumbnail = run.image_paths.find(pano_id);
	return thumbnail == run.image_paths.end() ? std::nullopt
											  : decode_cached_image(thumbnail->second);
}

std::vector<std::array<std::uint8_t, 3>> row_band_colours(
		const bands::Structure &structure)
{
	std::vector<std::array<std::uint8_t, 3>> result;
	result.reserve(structure.rows);
	for (std::size_t row = 0; row < structure.rows; ++row) {
		const auto band = std::find_if(
				structure.bands.begin(), structure.bands.end(), [&](const auto &entry) {
					return entry.first <= row && row <= entry.last;
				});
		result.push_back(band != structure.bands.end() && band->rgb
								 ? *band->rgb
								 : std::array<std::uint8_t, 3>{128, 128, 128});
	}
	return result;
}

WallProduct texture_wall(const PipelineConfig &config, const Geometry &geometry,
		const AlignmentRun &run, const Wall &wall, const AlignedWall &views)
{
	const auto frame_it = run.alignment.planes.find(wall.key);
	const PlaneFit frame_fit = frame_it == run.alignment.planes.end()
									   ? plane::fallback_plane(wall)
									   : frame_it->second;
	const auto ppb = config.params.tex_ppb;
	std::vector<LoadedWallView> loaded;
	loaded.reserve(views.selected.size());
	for (const auto &candidate : views.selected) {
		const auto camera_it = run.registration.cameras.find(candidate.pano_id);
		if (camera_it == run.registration.cameras.end())
			continue;
		auto image = texture_image(run, candidate.pano_id, candidate.dist_m);
		if (!image)
			continue;

		const auto key = wall.key + "__" + candidate.pano_id;
		const bool is_frame =
				frame_fit.pano_id && *frame_fit.pano_id == candidate.pano_id;
		PlaneFit fit = frame_fit;
		if (!is_frame) {
			const auto per_view = run.alignment.per_view.find(key);
			if (per_view != run.alignment.per_view.end())
				fit = per_view->second;
		}
		double ds_m = 0.0;
		if (!is_frame) {
			const auto offset = run.alignment.per_view_offsets.find(key);
			if (offset != run.alignment.per_view_offsets.end())
				ds_m = offset->second.ds_m;
		}
		const auto base = run.alignment.z_bases.find(key);
		const double z_base = base == run.alignment.z_bases.end()
									  ? camera_it->second.ground_z
									  : base->second.first;
		const auto cloud_height = plane::cloud_height(fit, z_base);
		const rectify::View view{
				&camera_it->second, &fit, z_base, candidate.dist_m, candidate.s_vis};
		const auto depth = run.alignment.depths.find(candidate.pano_id);
		const auto crop = rectify::loose_crop(wall, view, *image,
				depth == run.alignment.depths.end() ? nullptr : &depth->second,
				geometry.buildings, config.params);
		refine::ViewInputs inputs;
		inputs.wall = &wall;
		inputs.pano_id = candidate.pano_id;
		inputs.fitted_length = plane::fitted_wall(wall, fit).length;
		inputs.visible_s = candidate.s_vis;
		inputs.cloud_height = cloud_height;
		inputs.ground_source = camera_it->second.ground_source;
		auto refined = refine::refine_view(crop, inputs, config.params);
		LoadedWallView entry;
		entry.pano_id = candidate.pano_id;
		entry.score = candidate.score;
		entry.distance_m = candidate.dist_m;
		entry.fit = fit;
		entry.ds_m = ds_m;
		entry.z_base = crop.z_base;
		entry.ground_dz = refined.refinement.ground_dz;
		entry.visible_s = candidate.s_vis;
		entry.cloud_height = cloud_height;
		entry.crop = crop;
		entry.refinement = std::move(refined.refinement);
		entry.image = std::move(*image);
		loaded.push_back(std::move(entry));
	}
	if (loaded.empty()) {
		auto product = make_no_view_product(wall, views);
		if (!views.selected.empty()) {
			product.flags.push_back("NO_IMAGE");
			std::sort(product.flags.begin(), product.flags.end());
			product.flags.erase(std::unique(product.flags.begin(), product.flags.end()),
					product.flags.end());
		}
		return product;
	}

	const bool fix_a = views.corner_a == plane::EndSource::CloudCorner;
	const bool fix_b = views.corner_b == plane::EndSource::CloudCorner;
	std::vector<refine::ViewEvidence::CropEvidence> crop_evidence;
	crop_evidence.reserve(loaded.size());
	for (const auto &view : loaded)
		crop_evidence.push_back(
				{&view.crop.rgb, &view.crop.occl, view.crop.ppm, view.crop.s0});
	std::vector<refine::ViewEvidence> evidence;
	evidence.reserve(loaded.size());
	for (std::size_t i = 0; i < loaded.size(); ++i) {
		const auto &view = loaded[i];
		evidence.push_back({&view.refinement, view.score, view.ds_m,
				view.cloud_height
						? std::optional<double>(*view.cloud_height - view.ground_dz)
						: std::nullopt,
				&crop_evidence[i]});
	}
	auto decision = refine::decide_wall(wall, plane::fitted_wall(wall, frame_fit).length,
			fix_a, fix_b, evidence, std::vector<std::string>{}, config.params);
	if (!std::isfinite(decision.h_used) || decision.h_used < 1.0) {
		std::vector<double> heights;
		for (const auto &view : loaded)
			if (view.cloud_height)
				heights.push_back(*view.cloud_height);
		const double fallback = heights.empty() ? wall.height_m(config.params)
												: imgops::median(std::move(heights));
		decision.h_used = std::clamp(fallback, 3.0, 80.0);
		decision.height_source = "default";
	}
	const auto cols = static_cast<std::uint32_t>(
			std::max<std::int64_t>(1, static_cast<std::int64_t>(imgops::round_half_even(
											  decision.s_right - decision.s_left))));
	const auto rows = static_cast<std::uint32_t>(std::max<std::int64_t>(
			1, static_cast<std::int64_t>(imgops::round_half_even(decision.h_used))));
	const double s_center = .5 * (decision.s_left + decision.s_right);
	const double s_left = s_center - .5 * cols;
	const double s_right = s_center + .5 * cols;
	std::vector<double> ground_offsets;
	ground_offsets.reserve(loaded.size());
	for (const auto &view : loaded)
		ground_offsets.push_back(view.ground_dz);
	const double wall_dz = imgops::median(std::move(ground_offsets));

	std::vector<fuse::ViewTexture> textures;
	textures.reserve(loaded.size());
	for (const auto &view : loaded) {
		const auto camera = run.registration.cameras.find(view.pano_id);
		if (camera == run.registration.cameras.end())
			continue;
		const rectify::View rect_view{
				&camera->second, &view.fit, view.z_base, view.distance_m, view.visible_s};
		const std::array<double, 4> rectangle{s_left - view.ds_m, s_right - view.ds_m,
				wall_dz, static_cast<double>(rows) + wall_dz};
		const auto rectified = rectify::resample_rect(wall, rect_view, view.image,
				rectangle, &view.refinement.h_shear, ppb, &view.crop, config.params);
		textures.push_back({view.pano_id, rectified.rgb, rectified.valid, view.score});
	}
	if (textures.empty())
		return make_no_view_product(wall, views);
	const auto fused = fuse::fuse(wall.key, textures, ppb, config.params);
	const auto phase = refine::phase_search(fused.rgb, fused.valid, ppb, config.params);
	const std::pair<int, int> origin_px{
			static_cast<int>(imgops::round_half_even(phase[0] * ppb)),
			static_cast<int>(imgops::round_half_even(phase[1] * ppb))};
	decision.phase = {phase[0], phase[1]};
	decision.bimodality = phase[2];
	openings::Texture wall_texture{fused.rgb, fused.valid};
	const auto structure = bands::analyse_texture(
			wall_texture, rows, cols, origin_px, nullptr, bands::band_tau);

	const auto cell_count = std::size_t(rows) * cols;
	double unknown_share = 0.0;
	if (structure.openings && cell_count) {
		std::size_t unknown = 0;
		for (const auto cls : structure.openings->classes)
			unknown += cls == openings::cls_unknown || cls == openings::cls_nodata;
		unknown_share = static_cast<double>(unknown) / cell_count;
	}
	const auto &best = loaded.front();
	const auto candidate = std::find_if(views.candidates.begin(), views.candidates.end(),
			[&](const auto &entry) { return entry.pano_id == best.pano_id; });
	const auto meta = geometry.metas.find(best.pano_id);
	const double quality =
			candidate != views.candidates.end() && candidate->gate(GateName::Quality)
					? candidate->gate(GateName::Quality)->value
			: meta != geometry.metas.end() ? meta->second.quality
										   : 1.0;
	const double blur =
			candidate != views.candidates.end() && candidate->gate(GateName::BlurRel)
					? std::clamp(candidate->gate(GateName::BlurRel)->value, .35, 1.0)
					: 1.0;
	const auto camera = run.registration.cameras.find(best.pano_id);
	if (camera == run.registration.cameras.end())
		return make_no_view_product(wall, views);
	const auto &refs = loaded;
	std::vector<refine::Refinement> refinements;
	std::vector<std::string> refinement_flags;
	refinements.reserve(refs.size());
	for (const auto &view : refs) {
		refinements.push_back(view.refinement);
		for (const auto *flag : {&view.refinement.lean_flag, &view.refinement.plane_flag,
					 &view.refinement.roof_flag, &view.refinement.ground_flag})
			refinement_flags.push_back(*flag);
	}
	double lean_factor = .85;
	if (!refinements.empty() && refinements.front().lean_flag == refine::shear_ok)
		lean_factor = 1.0;
	else if (!refinements.empty() &&
			 refinements.front().lean_flag == refine::shear_rejected)
		lean_factor = .7;
	double plane_gate_factor = .8;
	if (std::any_of(refinements.begin(), refinements.end(),
				[](const auto &ref) { return ref.plane_flag == refine::plane_mismatch; }))
		plane_gate_factor = .3;
	else if (std::any_of(refinements.begin(), refinements.end(),
					 [](const auto &ref) { return ref.plane_flag == refine::on_plane; }))
		plane_gate_factor = 1.0;
	const auto factors = factors_for(camera->second.pose_source,
			camera->second.registration ? &*camera->second.registration : nullptr,
			&frame_fit, decision, decision.flags, fused.agreement_m, loaded.size(),
			unknown_share, std::max(.05, quality), blur,
			height_source_name(wall.height_source), lean_factor, plane_gate_factor);
	const auto [confidence_score, tier] = score(factors, config.params);
	std::set<std::string> flags(decision.flags.begin(), decision.flags.end());
	flags.insert(fused.flags.begin(), fused.flags.end());
	if (fused.mode == fuse::FuseMode::Single)
		flags.insert("SINGLE_VIEW");
	const auto reasons = reason_codes(views.candidates,
			std::optional<WallDecision>{decision}, refinement_flags,
			camera->second.registration ? &*camera->second.registration : nullptr);
	flags.insert(reasons.begin(), reasons.end());

	WallProduct product;
	product.wall_key = wall.key;
	product.building_key = wall.building_key;
	product.node_a = wall.node_a;
	product.node_b = wall.node_b;
	product.piece = wall.piece;
	product.pieces = wall.pieces;
	product.edges = wall.edges;
	product.col0_m = s_left;
	product.cols = cols;
	product.rows = rows;
	product.rgb = structure.rgb;
	product.classes = structure.classes;
	product.observed.resize(cell_count, true);
	if (structure.openings)
		for (std::size_t i = 0; i < cell_count; ++i)
			product.observed[i] = i >= structure.openings->classes.size() ||
								  structure.openings->classes[i] != openings::cls_nodata;
	product.bands = row_band_colours(structure);
	product.texture_width = fused.rgb.width;
	product.texture_height = fused.rgb.height;
	product.texture_rgba.resize(std::size_t(fused.rgb.width) * fused.rgb.height * 4);
	for (std::size_t i = 0; i < std::size_t(fused.rgb.width) * fused.rgb.height; ++i) {
		const auto pixel =
				i < fused.rgb.pixels.size() ? fused.rgb.pixels[i] : projection::Rgb{};
		for (std::size_t channel = 0; channel < 3; ++channel)
			product.texture_rgba[i * 4 + channel] = pixel[channel];
		product.texture_rgba[i * 4 + 3] =
				i < fused.valid.size() && fused.valid[i] ? 255 : 0;
	}
	product.tier = tier;
	product.confidence = confidence_score;
	product.height_used_m = decision.h_used;
	product.unknown_share = unknown_share;
	for (const auto &view : loaded)
		product.views.push_back(view.pano_id);
	product.flags.assign(flags.begin(), flags.end());
	return product;
}

double search_reach_m(
		const PipelineConfig &config, const std::array<double, 4> &bbox, const Wall &wall)
{
	const auto reach = [&](const std::array<double, 2> &point) {
		return std::min({point[0] - (bbox[0] - config.pano_margin_m),
				(bbox[2] + config.pano_margin_m) - point[0],
				point[1] - (bbox[1] - config.pano_margin_m),
				(bbox[3] + config.pano_margin_m) - point[1]});
	};
	return std::clamp(
			std::min(reach(wall.a), reach(wall.b)), 0.0, config.params.far_dist_m);
}

std::filesystem::path mint_export_dir(const PipelineConfig &config)
{
	static std::atomic<std::uint64_t> counter{0};
	std::uint64_t hash = 14695981039346656037ull;
	const std::array bounds{config.bounds.min_latitude, config.bounds.min_longitude,
			config.bounds.max_latitude, config.bounds.max_longitude};
	for (const auto value : bounds) {
		const double rounded = std::round(value * 1e7);
		std::uint64_t bits{};
		std::memcpy(&bits, &rounded, sizeof(bits));
		for (unsigned shift = 0; shift < 64; shift += 8) {
			hash ^= static_cast<std::uint8_t>(bits >> shift);
			hash *= 1099511628211ull;
		}
	}
	const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch())
							 .count();
	std::ostringstream name;
	name << std::hex << std::setfill('0') << std::setw(16) << hash << '-' << std::setw(1)
		 << now << '-' << ARNIS_PIPELINE_GETPID() << '-'
		 << counter.fetch_add(1, std::memory_order_relaxed);
	return config.exports_dir() / name.str();
}

void sweep_exports(
		const std::filesystem::path &exports, const std::filesystem::path &keep)
{
	constexpr auto retention = std::chrono::hours(1);
	constexpr std::size_t keep_areas = 8;
	std::error_code ec;
	std::vector<std::filesystem::directory_entry> entries;
	for (std::filesystem::directory_iterator it(exports, ec), end; !ec && it != end;
			it.increment(ec))
		entries.push_back(*it);
	if (ec)
		return;

	using Clock = std::filesystem::file_time_type::clock;
	struct Candidate
	{
		Clock::time_point modified;
		std::filesystem::path path;
	};
	const auto valid_export_name = [](const std::filesystem::path &path) {
		const auto name = path.filename().string();
		if (name.size() < 18 || name[16] != '-')
			return false;
		return name.find_first_not_of("0123456789abcdef", 0) >= 16;
	};
	std::map<std::string, Candidate> newest;
	std::vector<std::filesystem::path> aged;
	for (const auto &entry : entries) {
		const auto path = entry.path();
		if (path == keep || !valid_export_name(path))
			continue;
		std::error_code type_error, metadata_error;
		if (!entry.is_directory(type_error) || type_error)
			continue;
		const auto modified = entry.last_write_time(metadata_error);
		if (metadata_error)
			continue;
		const auto name = path.filename().string();
		const auto area = name.substr(0, name.find('-'));
		auto [it, inserted] = newest.try_emplace(area, Candidate{modified, path});
		if (inserted)
			continue;
		if (modified > it->second.modified ||
				(modified == it->second.modified && path > it->second.path)) {
			aged.push_back(std::move(it->second.path));
			it->second = {modified, path};
		} else {
			aged.push_back(path);
		}
	}

	const auto keep_name = keep.filename().string();
	const auto keep_area = keep_name.substr(0, keep_name.find('-'));
	if (const auto it = newest.find(keep_area); it != newest.end()) {
		aged.push_back(std::move(it->second.path));
		newest.erase(it);
	}
	std::vector<Candidate> survivors;
	survivors.reserve(newest.size());
	for (auto &[area, candidate] : newest) {
		(void)area;
		survivors.push_back(std::move(candidate));
	}
	std::sort(survivors.begin(), survivors.end(), [](const auto &a, const auto &b) {
		return a.modified != b.modified ? a.modified > b.modified : a.path < b.path;
	});
	for (std::size_t i = keep_areas; i < survivors.size(); ++i)
		aged.push_back(std::move(survivors[i].path));

	const auto now = Clock::now();
	for (const auto &path : aged) {
		std::error_code metadata_error;
		const auto modified = std::filesystem::last_write_time(path, metadata_error);
		if (metadata_error || now - modified <= retention)
			continue;
		std::error_code remove_error;
		std::filesystem::remove_all(path, remove_error);
	}
}
} // namespace

TextureStage stage_texture(
		const PipelineConfig &config, const Geometry &geometry, const AlignmentRun &run)
{
	TextureStage stage;
	stage.products.reserve(geometry.walls.size());
	for (const auto &wall : geometry.walls) {
		static const AlignedWall empty_views{};
		const auto aligned = run.alignment.walls.find(wall.key);
		const auto &view_record =
				aligned == run.alignment.walls.end() ? empty_views : aligned->second;
		if (config.cancelled()) {
			stage.products.push_back(make_no_view_product(wall, view_record));
			continue;
		}
		const auto key = cache::wall_cache_key(wall_node_ids(wall), wall.piece);
		if (const auto cached = cache::load_wall(config.facade_cache, key);
				cached && cached->product.cols > 0) {
			auto product = cached->product;
			product.wall_key = wall.key;
			product.building_key = wall.building_key;
			stage.products.push_back(std::move(product));
			++stage.cache_hits;
			continue;
		}
		if (aligned == run.alignment.walls.end() || aligned->second.selected.empty()) {
			stage.products.push_back(make_no_view_product(wall, view_record));
			continue;
		}
		++stage.cache_misses;
		auto product = texture_wall(config, geometry, run, wall, aligned->second);
		if (product.cols > 0) {
			std::string error;
			if (cache::store_wall(config.facade_cache, product,
						search_reach_m(config, geometry.bbox_xy, wall), &error)
							.empty() &&
					!error.empty())
				std::cerr << "Note: facade cache write for " << wall.key << ": " << error
						  << '\n';
		}
		stage.products.push_back(std::move(product));
	}
	return stage;
}

namespace
{
// Rust caches a negative wall verdict only when the search had imagery to
// judge and no candidate was lost to a failed image download.  A failed
// download is not evidence that the wall has no usable facade.
void store_blank_verdicts(const PipelineConfig &config, const Geometry &geometry,
		const AlignmentRun &run, const std::vector<WallProduct> &products,
		bool search_complete)
{
	if (!search_complete || geometry.metas.empty() ||
			products.size() != geometry.walls.size())
		return;

	for (std::size_t i = 0; i < products.size(); ++i) {
		const auto &product = products[i];
		if (product.cols != 0)
			continue;

		const auto aligned = run.alignment.walls.find(geometry.walls[i].key);
		const bool lost_download =
				aligned != run.alignment.walls.end() &&
				std::any_of(aligned->second.candidates.begin(),
						aligned->second.candidates.end(),
						[&](const ViewCandidate &candidate) {
							return candidate.rejected_reason == "no_image" &&
								   !run.image_paths.contains(candidate.pano_id);
						});
		if (lost_download)
			continue;

		std::string error;
		if (cache::store_wall(config.facade_cache, product,
					search_reach_m(config, geometry.bbox_xy, geometry.walls[i]), &error)
						.empty() &&
				!error.empty())
			std::cerr << "Note: facade cache verdict for " << product.wall_key << ": "
					  << error << '\n';
	}
}
} // namespace

namespace
{
TextureStage stage_texture_from_cache(
		const PipelineConfig &config, const Geometry &geometry)
{
	TextureStage stage;
	stage.products.reserve(geometry.walls.size());
	for (const auto &wall : geometry.walls) {
		const auto key = cache::wall_cache_key(wall_node_ids(wall), wall.piece);
		auto cached = cache::load_wall(config.facade_cache, key);
		if (cached && (cached->product.cols > 0 ||
							  cached->reach_m + 1.0 >=
									  search_reach_m(config, geometry.bbox_xy, wall))) {
			++stage.cache_entries;
			auto product = std::move(cached->product);
			product.wall_key = wall.key;
			product.building_key = wall.building_key;
			stage.cache_hits += product.cols > 0;
			stage.products.push_back(std::move(product));
			continue;
		}
		++stage.cache_misses;
		stage.products.push_back(make_uncached_product(wall));
	}
	return stage;
}
} // namespace

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
	const std::vector<std::string> requested_images(
			result.candidates.requested_images.begin(),
			result.candidates.requested_images.end());
	std::vector<std::optional<std::filesystem::path>> downloaded_images(
			requested_images.size());
	auto parallel_for = [&](std::size_t count, const auto &work) {
		if (count == 0)
			return;
		const std::size_t requested =
				config.threads == 0 ? DEFAULT_IMAGE_DOWNLOAD_THREADS : config.threads;
		const std::size_t worker_count =
				std::max<std::size_t>(1, std::min(count, requested));
		std::atomic_size_t next{0};
		std::vector<std::thread> workers;
		workers.reserve(worker_count);
		for (std::size_t worker = 0; worker < worker_count; ++worker) {
			workers.emplace_back([&] {
				for (;;) {
					const auto index = next.fetch_add(1, std::memory_order_relaxed);
					if (index >= count || config.cancelled())
						return;
					work(index);
				}
			});
		}
		for (auto &worker : workers)
			worker.join();
	};
	parallel_for(requested_images.size(), [&](std::size_t index) {
		const auto &pano_id = requested_images[index];
		const auto record = geometry.image_records.find(pano_id);
		if (record != geometry.image_records.end())
			downloaded_images[index] =
					client.download_image(record->second, cache::ImageSize::W2048);
	});
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}
	for (std::size_t index = 0; index < requested_images.size(); ++index)
		if (downloaded_images[index])
			image_paths.emplace(requested_images[index], *downloaded_images[index]);
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
	result.image_paths = std::move(image_paths);
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}

	// Match Rust's fetch_originals(): only selected near views on walls that
	// cannot be satisfied from the facade cache need full-resolution downloads.
	std::set<std::string> originals_needed;
	for (const auto &wall : geometry.walls) {
		const auto aligned = result.alignment.walls.find(wall.key);
		if (aligned == result.alignment.walls.end() || aligned->second.selected.empty())
			continue;
		const auto key = cache::wall_cache_key(wall_node_ids(wall), wall.piece);
		const auto paths = cache::wall_paths(config.facade_cache, key);
		std::error_code ec_json, ec_blocks;
		if (paths && std::filesystem::is_regular_file(std::get<0>(*paths), ec_json) &&
				std::filesystem::is_regular_file(std::get<1>(*paths), ec_blocks))
			continue;
		for (const auto &view : aligned->second.selected)
			if (view.dist_m < HIRES_MAX_DIST_M)
				originals_needed.insert(view.pano_id);
	}
	const std::vector<std::string> original_ids(
			originals_needed.begin(), originals_needed.end());
	std::vector<std::optional<std::filesystem::path>> downloaded_originals(
			original_ids.size());
	parallel_for(original_ids.size(), [&](std::size_t index) {
		const auto record = geometry.image_records.find(original_ids[index]);
		if (record != geometry.image_records.end())
			downloaded_originals[index] =
					client.download_image(record->second, cache::ImageSize::Original);
	});
	if (config.cancelled()) {
		result.cancelled = true;
		return result;
	}
	for (std::size_t index = 0; index < original_ids.size(); ++index)
		if (downloaded_originals[index])
			result.original_paths.emplace(
					original_ids[index], *downloaded_originals[index]);
	result.cancelled = config.cancelled();
	return result;
}

PipelineResult acquire_images(const Client &client, const PipelineConfig &config,
		const nlohmann::json *preloaded_osm)
{
	PipelineResult result;
	// Keep the selected cache root visible to later facade stages and library
	// hosts, even when acquisition returns early (cache-only or cancelled).
	result.export_dir = config.facade_cache;
	const auto started = std::chrono::steady_clock::now();
	auto fetch_osm_data = [&] {
		if (preloaded_osm) {
			result.osm = *preloaded_osm;
			return;
		}
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
	std::string search_error;
	const auto cells = search_cells(search_bounds, config.maximum_cells, &search_error);
	if (!cells) {
		result.search_error = search_error.empty()
									  ? "Mapillary search bounds could not be tiled"
									  : std::move(search_error);
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
	result.images = client.search(search_bounds, config.endpoint, config.access_token,
			config.maximum_cells, &result.failed_cells, &search_error);
	if (!search_error.empty())
		result.search_error = std::move(search_error);
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
	auto write_products = [&] {
		// Rust credits only imagery that contributed pixels to an exported wall.
		// Resolve metadata from the persistent image cache too, since a cache-hit
		// run may use photographs absent from this run's Graph search response.
		std::set<std::string> used_images;
		for (const auto &product : output.acquisition.products)
			used_images.insert(product.views.begin(), product.views.end());
		output.acquisition.credits.clear();
		for (const auto &id : used_images) {
			const auto metadata = client.cached_metadata(id);
			const std::string creator = metadata ? metadata->creator : std::string{};
			const std::string creator_id =
					metadata ? metadata->creator_id : std::string{};
			const std::string handle = creator.empty() ? creator_id : creator;
			const std::string title = config.area_label && !config.area_label->empty()
											  ? *config.area_label
											  : "Mapillary image " + id;
			output.acquisition.credits.push_back({id, title, creator,
					handle.empty() ? std::string{}
								   : "https://www.mapillary.com/app/user/" + handle,
					"https://www.mapillary.com/app/?pKey=" + id + "&focus=photo"});
		}
		std::set<std::string> exported_buildings;
		output.acquisition.stats.exported_walls = 0;
		for (const auto &product : output.acquisition.products)
			if (tier_has_blocks(product.tier) && product.cols > 0) {
				exported_buildings.insert(product.building_key);
				++output.acquisition.stats.exported_walls;
			}
		output.acquisition.stats.exported_buildings = exported_buildings.size();
		output.acquisition.export_dir = mint_export_dir(config);
		const auto export_started = std::chrono::steady_clock::now();
		if (!write_export(
					output.acquisition, output.acquisition.export_dir, &output.error))
			return false;
		sweep_exports(config.exports_dir(), output.acquisition.export_dir);
		output.acquisition.stats.export_seconds = std::chrono::duration<double>(
				std::chrono::steady_clock::now() - export_started)
														  .count();
		return true;
	};
	auto attach_geometry = [&] {
		output.acquisition.stats.buildings = output.geometry->buildings.size();
		output.acquisition.stats.walls = output.geometry->walls.size();
		output.acquisition.frame = output.geometry->frame;
		output.acquisition.buildings = output.geometry->buildings;
		output.acquisition.walls = output.geometry->walls;
	};
	if (config.cache_only) {
		std::error_code ec;
		if (!std::filesystem::exists(config.facade_cache, ec)) {
			output.error = *config.cache_only;
			finish();
			return output;
		}
	}

	// Rust obtains OSM before looking at imagery. The OSM geometry determines
	// the exact wall keys, so a complete (or permitted partial) facade cache can
	// answer without any Mapillary Graph requests.
	std::optional<nlohmann::json> preloaded_osm;
	const auto osm_started = std::chrono::steady_clock::now();
	if (config.osm_fetch) {
		std::string osm_error;
		preloaded_osm = fetch_osm(*config.osm_fetch, &osm_error);
		if (!preloaded_osm) {
			output.error = osm_error.empty() ? "OSM data is unavailable" : osm_error;
			finish();
			return output;
		}
	} else if (config.cache_only) {
		output.error = *config.cache_only;
		finish();
		return output;
	}
	if (config.cancelled()) {
		output.error = "cancelled";
		finish();
		return output;
	}
	const double osm_seconds =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - osm_started)
					.count();

	if (preloaded_osm) {
		PipelineResult osm_result;
		osm_result.osm = *preloaded_osm;
		std::optional<Geometry> cached_geometry;
		const auto cache_geometry_started = std::chrono::steady_clock::now();
		std::string cache_geometry_error;
		cached_geometry =
				stage_geometry(config, osm_result, client, &cache_geometry_error);
		const double cache_geometry_seconds = std::chrono::duration<double>(
				std::chrono::steady_clock::now() - cache_geometry_started)
													  .count();
		if (cached_geometry && !cached_geometry->buildings.empty() &&
				std::filesystem::exists(config.facade_cache)) {
			output.geometry = std::move(cached_geometry);
			output.acquisition = std::move(osm_result);
			attach_geometry();
			const auto texture = stage_texture_from_cache(config, *output.geometry);
			const bool complete = texture.cache_misses == 0;
			const bool partial_cache_only =
					config.cache_only && texture.cache_entries > 0;
			if (complete || partial_cache_only) {
				output.acquisition.products = texture.products;
				output.acquisition.stats.cache_hits = texture.cache_hits;
				output.acquisition.stats.cache_misses = texture.cache_misses;
				output.acquisition.stats.reachable = texture.cache_hits;
				output.acquisition.stats.walls_with_views = texture.cache_hits;
				output.acquisition.stats.geometry_seconds = cache_geometry_seconds;
				output.acquisition.stats.fetch_seconds = osm_seconds;
				for (std::size_t i = 0; i < output.acquisition.walls.size(); ++i)
					output.acquisition.walls[i].reachable =
							output.acquisition.products[i].cols > 0;
				output.cache_reused = true;
				output.cache_only_complete = config.cache_only.has_value();
				if (!write_products()) {
					finish();
					return output;
				}
				finish();
				return output;
			}
			if (config.cache_only) {
				output.error = *config.cache_only;
				finish();
				return output;
			}
		}
		if (config.cache_only) {
			output.error = *config.cache_only;
			finish();
			return output;
		}
	}

	output.acquisition =
			acquire_images(client, config, preloaded_osm ? &*preloaded_osm : nullptr);
	output.acquisition.stats.fetch_seconds += osm_seconds;
	if (output.acquisition.search_error) {
		output.error = *output.acquisition.search_error;
		finish();
		return output;
	}
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
	attach_geometry();
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
	output.acquisition.stats.originals_downloaded =
			output.alignment->original_paths.size();
	const auto texture_started = std::chrono::steady_clock::now();
	auto texture = stage_texture(config, *output.geometry, *output.alignment);
	if (config.cancelled()) {
		output.error = "cancelled";
		finish();
		return output;
	}
	store_blank_verdicts(config, *output.geometry, *output.alignment, texture.products,
			output.acquisition.failed_cells == 0);
	output.acquisition.products = std::move(texture.products);
	output.acquisition.stats.cache_hits = texture.cache_hits;
	output.acquisition.stats.cache_misses = texture.cache_misses;
	output.acquisition.stats.texture_seconds = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - texture_started)
													   .count();
	if (!write_products()) {
		finish();
		return output;
	}
	finish();
	return output;
}

struct FacadeJob::State
{
	std::shared_ptr<std::atomic_bool> cancel;
	std::shared_future<std::optional<std::filesystem::path>> result;
};

FacadeJob::~FacadeJob()
{
	if (state_ && state_.use_count() == 1 && state_->cancel)
		state_->cancel->store(true, std::memory_order_release);
}

FacadeJob FacadeJob::start(Client client, PipelineConfig config)
{
	auto state = std::make_shared<State>();
	state->cancel =
			config.cancel ? config.cancel : std::make_shared<std::atomic_bool>(false);
	config.cancel = state->cancel;
	auto promise = std::make_shared<std::promise<std::optional<std::filesystem::path>>>();
	state->result = promise->get_future().share();
	try {
		std::thread([client = std::move(client), config = std::move(config),
							promise]() mutable {
			try {
				if (!config.area_label) {
					const double latitude =
							(config.bounds.min_latitude + config.bounds.max_latitude) *
							0.5;
					const double longitude =
							(config.bounds.min_longitude + config.bounds.max_longitude) *
							0.5;
					config.area_label =
							retrieve_data::fetch_area_name(latitude, longitude);
				}
				auto run = run_alignment_pipeline(client, config);
				if (run.succeeded())
					promise->set_value(std::move(run.acquisition.export_dir));
				else
					promise->set_value(std::nullopt);
			} catch (...) {
				try {
					promise->set_value(std::nullopt);
				} catch (...) {
				}
			}
		}).detach();
	} catch (...) {
		state->cancel->store(true, std::memory_order_release);
		promise->set_value(std::nullopt);
	}
	return FacadeJob(std::move(state));
}

bool FacadeJob::is_running() const
{
	return state_ && state_->result.valid() &&
		   state_->result.wait_for(std::chrono::seconds(0)) != std::future_status::ready;
}

void FacadeJob::cancel() const
{
	if (state_ && state_->cancel)
		state_->cancel->store(true, std::memory_order_release);
}

std::optional<std::filesystem::path> FacadeJob::join() const
{
	if (!state_ || !state_->result.valid())
		return std::nullopt;
	try {
		return state_->result.get();
	} catch (...) {
		return std::nullopt;
	}
}
} // namespace arnis::mapillary
