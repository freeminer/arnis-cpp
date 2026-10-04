#pragma once

#include "api.h"
#include "cache.h"
#include "fetch.h"
#include "geometry.h"
#include "sfm.h"
#include "types.h"

#include <cstddef>
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>

namespace arnis::mapillary
{
// Rust's pipeline keeps acquisition separate from geometry/facade synthesis.
// This C++ contract exposes the same reusable first stage to library hosts.
struct PipelineConfig
{
	SearchCell bounds{};
	Params params{};
	std::string endpoint = "https://graph.mapillary.com/images";
	std::string access_token;
	std::size_t maximum_cells = mapillary_max_cells;
	// Expand panorama coverage beyond the requested box, as Rust does, so
	// views just outside the world bounds can still see facade walls inside.
	double pano_margin_m = 45.0;
	// Rust's later stages share these controls even when the caller uses a
	// cache-only run.  Keeping them in the C++ contract avoids silently losing
	// settings when a library host switches from acquisition to facades.
	std::filesystem::path facade_cache = cache::default_root();
	std::size_t threads = 0;
	std::shared_ptr<std::atomic_bool> cancel;
	std::optional<std::string> cache_only;
	std::optional<std::string> area_label;
	std::vector<CameraModel> camera_types;
	std::function<void(const cache::ImageRecord &)> image_sink;
	// Optional OSM side of Rust's FetchConfig. Set through osm_fetch_config()
	// to inherit --no-tile-archive and --osm-tiles-url from Args.
	std::optional<OsmFetchConfig> osm_fetch;
	bool cancelled() const { return cancel && cancel->load(std::memory_order_relaxed); }
};

struct PipelineStats
{
	std::size_t images = 0;
	std::size_t images_downloaded = 0;
	std::size_t originals_downloaded = 0;
	std::size_t clusters = 0;
	std::size_t buildings = 0;
	std::size_t walls = 0;
	std::size_t reachable = 0;
	std::size_t walls_with_views = 0;
	std::array<std::size_t, 4> tiers{};
	std::size_t cache_hits = 0;
	std::size_t cache_misses = 0;
	double fetch_seconds = 0.0;
	double geometry_seconds = 0.0;
	double align_seconds = 0.0;
	double texture_seconds = 0.0;
	double export_seconds = 0.0;
	double total_seconds = 0.0;
};

struct PipelineResult
{
	std::vector<cache::ImageRecord> images;
	std::vector<PanoMeta> metas;
	struct Credit
	{
		std::string pano_id, title, creator, creator_url, image_url;
	};
	struct ClusterRef
	{
		std::string id, pano_id;
	};
	std::vector<Credit> credits;
	std::vector<ClusterRef> clusters;
	std::optional<nlohmann::json> osm;
	std::optional<std::string> osm_error;
	std::size_t cells = 0;
	std::size_t failed_cells = 0;
	bool cancelled = false;
	bool cache_only = false;
	PipelineStats stats;
	std::filesystem::path export_dir;
};

// Geometry-stage output: owned data so subsequent alignment/texture stages do
// not depend on the lifetime of the acquisition result or its JSON buffers.
struct Geometry
{
	Frame frame;
	std::vector<Building> buildings;
	std::vector<Wall> walls;
	std::unordered_map<std::string, Camera> cameras;
	std::unordered_map<std::string, PanoMeta> metas;
	std::unordered_map<std::string, sfm::Cluster> clusters;
	std::array<double, 4> bbox_xy{};
};

PipelineResult acquire_images(const Client &, const PipelineConfig &);
std::optional<Geometry> stage_geometry(const PipelineConfig &, const PipelineResult &,
		const Client &, std::string *error = nullptr);
} // namespace arnis::mapillary
