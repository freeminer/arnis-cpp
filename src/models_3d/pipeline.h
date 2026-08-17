#pragma once
#include "provider.h"
#include "placement_executor.h"
#include "custom/plane.h"
#include "custom/stadium.h"
#include "three_dmr/prescan.h"
#include "wikidata/prescan.h"
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>
namespace arnis
{
class ProcessedElement;
}
namespace arnis::world_editor
{
struct WorldEditor;
}
namespace arnis::models_3d
{
struct ModelDiscovery
{
	std::vector<std::pair<std::string, std::int64_t>> suppressed;
	std::vector<std::pair<int, int>> deferred_regions;
	std::size_t plane_count = 0, stadium_count = 0, three_dmr_count = 0,
				wikidata_count = 0;
};
// Library equivalent of Rust's Models3dPipeline.  It deliberately separates
// discovery from model fetching/voxelization so a mapgen host can keep only
// the affected regions resident and choose its own providers.
class Models3dPipeline
{
	three_dmr::PrescanResult three_dmr_;
	wikidata::PrescanResult wikidata_;
	custom::stadium::PrescanResult stadium_;
	std::vector<custom::plane::Placement> plane_;
	// Claims visible before Wikidata is considered (caller + 3DMR).  Retention
	// uses this to avoid leaving stale Wikidata claims in `suppressed_`.
	std::vector<std::pair<std::string, std::int64_t>> pre_wikidata_suppressed_;
	std::vector<std::pair<std::string, std::int64_t>> suppressed_;

public:
	static Models3dPipeline prescan(const std::vector<ProcessedElement> &, double scale,
			double world_rotation = 0.0,
			const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed =
					{});
	// Fetch-aware discovery matches Rust's provider ordering: a Wikidata model
	// that cannot be fetched makes no claims, allowing stadium generation to
	// consume its OSM geometry in the same pass.
	static Models3dPipeline prescan_fetchable_wikidata(
			const std::vector<ProcessedElement> &, double scale,
			const std::function<bool(const std::string &)> &, double world_rotation = 0.0,
			const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed =
					{});
	// Full Rust-equivalent custom-model ordering: Wikidata and stadium claims
	// enter the OSM suppression set only if their respective model is ready.
	static Models3dPipeline prescan_fetchable_models(
			const std::vector<ProcessedElement> &, double scale,
			const std::function<bool(const std::string &)> &wikidata_fetchable,
			const std::function<bool()> &stadium_fetchable, double world_rotation = 0.0,
			const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed =
					{});
	const three_dmr::PrescanResult &three_dmr() const { return three_dmr_; }
	const wikidata::PrescanResult &wikidata() const { return wikidata_; }
	const custom::stadium::PrescanResult &stadium() const { return stadium_; }
	const std::vector<custom::plane::Placement> &planes() const { return plane_; }
	const std::vector<std::pair<std::string, std::int64_t>> &suppressed() const
	{
		return suppressed_;
	}
	void retain_fetchable_wikidata(const std::function<bool(const std::string &)> &);
	std::vector<std::pair<int, int>> deferred_regions(double scale) const;
	ModelDiscovery discovery(double scale) const;
};
// Shared terrain anchoring and streamed-region retention helpers from
// models_3d/mod.rs.  These are useful to custom hosts independently of any
// exporter.
int lowest_ground_in_bbox(
		const world_editor::WorldEditor &, int min_x, int min_z, int max_x, int max_z);
std::vector<std::pair<int, int>> region_keys_around(
		int center_x, int center_z, int radius);
// Offline prescan shared by the generation pipeline. Fetching/model placement
// remains intentionally separate so this works in library and streamed modes.
ModelDiscovery discover_models(const std::vector<ProcessedElement> &, double scale,
		double world_rotation = 0.0,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed = {});
// Same discovery result, but performs model availability selection before any
// Wikidata geometry is suppressed.  `fetchable` should use the host's model
// cache/provider and may return false without treating it as a fatal error.
ModelDiscovery discover_models_fetchable_wikidata(const std::vector<ProcessedElement> &,
		double scale, const std::function<bool(const std::string &)> &fetchable,
		double world_rotation = 0.0,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed = {});
ModelDiscovery discover_models_fetchable(const std::vector<ProcessedElement> &,
		double scale, const std::function<bool(const std::string &)> &wikidata_fetchable,
		const std::function<bool()> &stadium_fetchable, double world_rotation = 0.0,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed = {});
// Execute the two Arnis-hosted archetype providers selected by a pipeline
// prescan.  `provider` may be custom::Client (including its host fetch
// callback) or any ModelProvider; unavailable assets simply yield zero placed
// models, preserving the discovery fallback contract.
ModelPlacementStats place_custom_models(const Models3dPipeline &, ModelProvider &,
		world_editor::WorldEditor &, double blocks_per_meter);
bool place_provider_model(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float y, float z, float scale = 1.0f,
		float yaw = 0.0f);
bool place_provider_model_height(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float y, float z, float target_height,
		float yaw = 0.0f);
bool place_provider_model_rooted(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float ground_y, float z, float scale = 1.0f,
		float yaw = 0.0f);
bool place_provider_model_height_rooted(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float ground_y, float z, float target_height,
		float yaw = 0.0f);
bool place_provider_model_centered(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float ground_y, float z, float scale = 1.0f,
		float yaw = 0.0f);
bool place_provider_model_height_centered(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float ground_y, float z, float target_height,
		float yaw = 0.0f);
std::array<float, 3> rotated_center_offset(const ModelAsset &, float scale, float yaw);
std::array<float, 2> rotated_footprint(const ModelAsset &, float scale, float yaw);
bool footprint_within(
		const ModelAsset &, float scale, float yaw, float max_x, float max_z);
bool provider_footprint_within(ModelProvider &, const std::string &, float scale,
		float yaw, float max_x, float max_z);
bool place_provider_model_centered_limited(ModelProvider &, world_editor::WorldEditor &,
		const std::string &, float x, float ground_y, float z, float scale, float yaw,
		float max_x, float max_z);
bool place_provider_model_height_centered_limited(ModelProvider &,
		world_editor::WorldEditor &, const std::string &, float x, float ground_y,
		float z, float target_height, float yaw, float max_x, float max_z);
}
