#pragma once

#include "api.h"
#include "cache.h"
#include "fetch.h"
#include "geometry.h"
#include "register.h"
#include "plane.h"
#include "sfm.h"
#include "types.h"
#include "visibility.h"

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
#include <map>
#include <set>
#include <memory>

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
	std::filesystem::path facade_cache = cache::Layout{}.facade_dir(params.digest(), 1);
	std::filesystem::path exports_dir() const { return facade_cache / "exports"; }
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
	std::size_t exported_buildings = 0;
	std::size_t exported_walls = 0;
	double fetch_seconds = 0.0;
	double geometry_seconds = 0.0;
	double align_seconds = 0.0;
	double texture_seconds = 0.0;
	double export_seconds = 0.0;
	double total_seconds = 0.0;
};

struct PipelineResult
{
	Frame frame;
	std::vector<Building> buildings;
	std::vector<Wall> walls;
	std::vector<WallProduct> products;
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
	std::unordered_map<std::string, cache::ImageRecord> image_records;
	std::unordered_map<std::string, sfm::Cluster> clusters;
	std::array<double, 4> bbox_xy{};
};

struct RegistrationStats
{
	std::size_t panos = 0, with_cluster = 0, local = 0, global = 0, none = 0,
				no_cluster = 0, theta_used = 0;
	double acceptance_rate_local = 0.0, acceptance_rate_any = 0.0;
	double median_shift_m = 0.0, median_inliers_after = 0.0, median_inliers_before = 0.0;
	std::map<std::string, std::size_t> local_rejection_reasons;
};

struct RegistrationStage
{
	std::map<std::string, registration::Registration> registrations;
	std::map<std::string, registration::Registration> local_attempts;
	std::map<std::string, std::optional<std::array<double, 3>>> global_shifts;
	std::map<std::string, Camera> cameras;
	std::map<std::string, std::size_t> ground_points;
	RegistrationStats stats;
};

struct CandidateStage
{
	Footprints footprints;
	std::vector<Wall> walls;
	std::map<std::string, std::vector<ViewCandidate>> by_wall;
	std::set<std::string> requested_images;
	std::size_t reachable = 0;
};

struct PlaneStage
{
	// Keys use Rust's stable "<wall key>__<pano id>" representation.
	std::map<std::string, PlaneFit> per_view;
	std::map<std::string, std::pair<double, std::string>> z_bases;
};

struct AlignedWall
{
	PlaneFit plane;
	std::optional<std::string> best_pano;
	std::vector<ViewCandidate> selected, candidates;
	plane::EndSource corner_a{plane::EndSource::Osm}, corner_b{plane::EndSource::Osm};
	std::string unreachable_reason;
	std::size_t n_los_clean{};
	bool reachable{};
	bool single_view{};
};
struct AlignmentStage
{
	std::map<std::string, sfm::DepthMap> depths;
	std::map<std::string, PlaneFit> planes, per_view;
	std::map<std::string, std::pair<double, std::string>> z_bases;
	std::map<std::string, plane::ViewOffset> per_view_offsets;
	std::map<std::string, AlignedWall> walls;
	std::size_t candidates_no_image{}, reachable{}, walls_with_views{};
};

struct AlignmentRun
{
	RegistrationStage registration;
	CandidateStage candidates;
	AlignmentStage alignment;
	std::map<std::string, std::filesystem::path> image_paths;
	std::map<std::string, std::filesystem::path> original_paths;
	std::size_t images_requested{}, images_decoded{};
	bool cancelled{};
};

struct AlignmentPipelineResult
{
	PipelineResult acquisition;
	std::optional<Geometry> geometry;
	std::optional<AlignmentRun> alignment;
	bool cache_only_complete{};
	bool cache_reused{};
	std::string error;
	bool succeeded() const
	{
		return geometry.has_value() &&
			   (alignment.has_value() || cache_only_complete || cache_reused) &&
			   error.empty();
	}
};

struct TextureStage
{
	std::vector<WallProduct> products;
	std::size_t cache_hits{}, cache_misses{}, cache_entries{};
};

// Shared asynchronous lifetime for facade precomputation. A host can start it
// while preparing terrain/OSM, then attach it to GenerationOptions; world
// generation joins once, immediately before facade data is consumed.
class FacadeJob
{
	struct State;
	std::shared_ptr<State> state_;
	explicit FacadeJob(std::shared_ptr<State> state) : state_(std::move(state)) {}

public:
	FacadeJob() = default;
	~FacadeJob();
	static FacadeJob start(Client client, PipelineConfig config);
	bool is_running() const;
	void cancel() const;
	std::optional<std::filesystem::path> join() const;
};

PipelineResult acquire_images(const Client &, const PipelineConfig &,
		const nlohmann::json *preloaded_osm = nullptr);
std::optional<Geometry> stage_geometry(const PipelineConfig &, const PipelineResult &,
		const Client &, std::string *error = nullptr);
RegistrationStage stage_registration(const PipelineConfig &, const Geometry &);
CandidateStage stage_candidates(
		const PipelineConfig &, const Geometry &, const RegistrationStage &);
PlaneStage stage_planes(const PipelineConfig &, const Geometry &,
		const RegistrationStage &, const CandidateStage &);
AlignmentStage stage_align_visibility(const PipelineConfig &, const Geometry &,
		const RegistrationStage &, CandidateStage, const PlaneStage &,
		const std::map<std::string, projection::Image> &images);
using ImageLoader = std::function<std::optional<projection::Image>(const std::string &)>;
AlignmentStage stage_align_visibility(const PipelineConfig &, const Geometry &,
		const RegistrationStage &, CandidateStage, const PlaneStage &,
		const ImageLoader &load_image);
std::optional<projection::Image> decode_cached_image(
		const std::filesystem::path &, std::string *error = nullptr);
WallProduct make_no_view_product(const Wall &, const AlignedWall &);
WallProduct make_uncached_product(const Wall &);
TextureStage stage_texture(
		const PipelineConfig &, const Geometry &, const AlignmentRun &);
AlignmentRun run_alignment(const Client &, const PipelineConfig &, const Geometry &);
AlignmentPipelineResult run_alignment_pipeline(const Client &, const PipelineConfig &);
// Write a Rust-compatible facade export directory. The caller owns directory
// naming/retention; this function claims a fresh directory and never mixes runs.
bool write_export(const PipelineResult &, const std::filesystem::path &,
		std::string *error = nullptr);
} // namespace arnis::mapillary
