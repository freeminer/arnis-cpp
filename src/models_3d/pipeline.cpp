#include "pipeline.h"
#include "placement_pipeline.h"
#include "custom/plane.h"
#include "custom/stadium.h"
#include "three_dmr/prescan.h"
#include "wikidata/prescan.h"
#include "../../../arnis_adapter.h"
#include <array>
#include <cmath>
#include <limits>
namespace arnis::models_3d
{
namespace
{
template <typename T>
void append_unique(std::vector<T> &to, const std::vector<T> &from)
{
	to.insert(to.end(), from.begin(), from.end());
	std::sort(to.begin(), to.end());
	to.erase(std::unique(to.begin(), to.end()), to.end());
}
}

Models3dPipeline Models3dPipeline::prescan(const std::vector<ProcessedElement> &elements,
		double scale, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	Models3dPipeline out;
	out.suppressed_ = already_suppressed;
	// Rust order is intentional: explicit 3DMR, then Wikidata, then custom
	// stadiums.  Every later provider observes all previous claims.
	out.three_dmr_ = three_dmr::prescan(elements, world_rotation, out.suppressed_);
	append_unique(out.suppressed_, out.three_dmr_.suppressed);
	out.pre_wikidata_suppressed_ = out.suppressed_;
	out.wikidata_ = wikidata::prescan(elements, world_rotation, scale, out.suppressed_);
	append_unique(out.suppressed_, out.wikidata_.suppressed);
	out.stadium_ = custom::stadium::prescan(elements, scale, out.suppressed_);
	append_unique(out.suppressed_, out.stadium_.suppressed);
	// Planes are decorative runway props and deliberately suppress nothing.
	out.plane_ = custom::plane::prescan(elements, scale);
	return out;
}

Models3dPipeline Models3dPipeline::prescan_fetchable_wikidata(
		const std::vector<ProcessedElement> &elements, double scale,
		const std::function<bool(const std::string &)> &fetchable, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	Models3dPipeline out;
	out.suppressed_ = already_suppressed;
	out.three_dmr_ = three_dmr::prescan(elements, world_rotation, out.suppressed_);
	append_unique(out.suppressed_, out.three_dmr_.suppressed);
	out.pre_wikidata_suppressed_ = out.suppressed_;
	out.wikidata_ = wikidata::prescan(elements, world_rotation, scale, out.suppressed_);
	wikidata::retain_fetchable(out.wikidata_, fetchable);
	append_unique(out.suppressed_, out.wikidata_.suppressed);
	out.stadium_ = custom::stadium::prescan(elements, scale, out.suppressed_);
	append_unique(out.suppressed_, out.stadium_.suppressed);
	out.plane_ = custom::plane::prescan(elements, scale);
	return out;
}

Models3dPipeline Models3dPipeline::prescan_fetchable_models(
		const std::vector<ProcessedElement> &elements, double scale,
		const std::function<bool(const std::string &)> &wikidata_fetchable,
		const std::function<bool()> &stadium_fetchable, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	Models3dPipeline out;
	out.suppressed_ = already_suppressed;
	out.three_dmr_ = three_dmr::prescan(elements, world_rotation, out.suppressed_);
	append_unique(out.suppressed_, out.three_dmr_.suppressed);
	out.pre_wikidata_suppressed_ = out.suppressed_;
	out.wikidata_ = wikidata::prescan(elements, world_rotation, scale, out.suppressed_);
	wikidata::retain_fetchable(out.wikidata_, wikidata_fetchable);
	append_unique(out.suppressed_, out.wikidata_.suppressed);
	out.stadium_ = custom::stadium::prescan(elements, scale, out.suppressed_);
	custom::stadium::retain_fetchable(
			out.stadium_, !stadium_fetchable || stadium_fetchable());
	append_unique(out.suppressed_, out.stadium_.suppressed);
	out.plane_ = custom::plane::prescan(elements, scale);
	return out;
}

void Models3dPipeline::retain_fetchable_wikidata(
		const std::function<bool(const std::string &)> &fetchable)
{
	wikidata::retain_fetchable(wikidata_, fetchable);
	// Existing stadium candidates were scanned against the old set and cannot
	// be rediscovered here, but stale Wikidata claims must still be released.
	// prescan_fetchable_wikidata is the fully ordered API for new callers.
	suppressed_ = pre_wikidata_suppressed_;
	append_unique(suppressed_, wikidata_.suppressed);
	append_unique(suppressed_, stadium_.suppressed);
}

std::vector<std::pair<int, int>> Models3dPipeline::deferred_regions(double scale) const
{
	std::vector<std::pair<int, int>> out;
	append_unique(out, three_dmr::deferred_regions(three_dmr_, scale));
	append_unique(out, wikidata::deferred_regions(wikidata_, scale));
	append_unique(out, custom::stadium::deferred_regions(stadium_.placements, scale));
	append_unique(out, custom::plane::deferred_regions(plane_, scale));
	return out;
}

ModelDiscovery Models3dPipeline::discovery(double scale) const
{
	ModelDiscovery out;
	out.suppressed = suppressed_;
	out.plane_count = plane_.size();
	out.stadium_count = stadium_.placements.size();
	out.three_dmr_count = three_dmr_.placements.size();
	out.wikidata_count = wikidata_.placements.size();
	out.deferred_regions = deferred_regions(scale);
	return out;
}

int lowest_ground_in_bbox(const world_editor::WorldEditor &editor, int min_x, int min_z,
		int max_x, int max_z)
{
	if (min_x > max_x)
		std::swap(min_x, max_x);
	if (min_z > max_z)
		std::swap(min_z, max_z);
	const int stride = std::clamp(std::max(max_x - min_x, max_z - min_z) / 16, 1, 8);
	int lowest = std::numeric_limits<int>::max();
	for (int x = min_x; x <= max_x; x += stride)
		for (int z = min_z; z <= max_z; z += stride)
			lowest = std::min(lowest, editor.get_ground_level(x, z));
	for (const auto &[x, z] : std::array<std::pair<int, int>, 4>{
				 {{min_x, min_z}, {max_x, min_z}, {min_x, max_z}, {max_x, max_z}}})
		lowest = std::min(lowest, editor.get_ground_level(x, z));
	return lowest == std::numeric_limits<int>::max()
				   ? editor.get_ground_level((min_x + max_x) / 2, (min_z + max_z) / 2)
				   : lowest;
}

std::vector<std::pair<int, int>> region_keys_around(int cx, int cz, int radius)
{
	radius = std::max(0, radius);
	const int rx0 = ((cx - radius) >> 9) - 1, rx1 = ((cx + radius) >> 9) + 1;
	const int rz0 = ((cz - radius) >> 9) - 1, rz1 = ((cz + radius) >> 9) + 1;
	std::vector<std::pair<int, int>> out;
	out.reserve(std::size_t(rx1 - rx0 + 1) * std::size_t(rz1 - rz0 + 1));
	for (int rx = rx0; rx <= rx1; ++rx)
		for (int rz = rz0; rz <= rz1; ++rz)
			out.emplace_back(rx, rz);
	return out;
}

ModelDiscovery discover_models(const std::vector<ProcessedElement> &elements,
		double scale, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	return Models3dPipeline::prescan(elements, scale, world_rotation, already_suppressed)
			.discovery(scale);
}

ModelDiscovery discover_models_fetchable_wikidata(
		const std::vector<ProcessedElement> &elements, double scale,
		const std::function<bool(const std::string &)> &fetchable, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	return Models3dPipeline::prescan_fetchable_wikidata(
			elements, scale, fetchable, world_rotation, already_suppressed)
			.discovery(scale);
}

ModelDiscovery discover_models_fetchable(const std::vector<ProcessedElement> &elements,
		double scale, const std::function<bool(const std::string &)> &wikidata_fetchable,
		const std::function<bool()> &stadium_fetchable, double world_rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already_suppressed)
{
	return Models3dPipeline::prescan_fetchable_models(elements, scale, wikidata_fetchable,
			stadium_fetchable, world_rotation, already_suppressed)
			.discovery(scale);
}

ModelPlacementStats place_custom_models(const Models3dPipeline &pipeline,
		ModelProvider &provider, world_editor::WorldEditor &editor,
		double blocks_per_meter)
{
	ModelPlacementStats out;
	const auto planes =
			place_plane_prescan(provider, editor, pipeline.planes(), blocks_per_meter);
	const auto stadiums = place_stadium_prescan(
			provider, editor, pipeline.stadium().placements, blocks_per_meter);
	out.attempted = planes.attempted + stadiums.attempted;
	out.placed = planes.placed + stadiums.placed;
	out.voxels = planes.voxels + stadiums.voxels;
	return out;
}
std::array<float, 3> rotated_center_offset(const ModelAsset &a, float s, float yaw)
{
	const float cx = (a.min[0] + a.max[0]) * 0.5f * s,
				cz = (a.min[2] + a.max[2]) * 0.5f * s,
				r = yaw * 3.141592653589793f / 180.0f;
	return {cx * std::cos(r) - cz * std::sin(r), 0.0f,
			cx * std::sin(r) + cz * std::cos(r)};
}
std::array<float, 2> rotated_footprint(const ModelAsset &a, float s, float yaw)
{
	const float r = std::abs(yaw) * 3.141592653589793f / 180.0f,
				w = (a.max[0] - a.min[0]) * s, d = (a.max[2] - a.min[2]) * s;
	return {std::abs(w * std::cos(r)) + std::abs(d * std::sin(r)),
			std::abs(w * std::sin(r)) + std::abs(d * std::cos(r))};
}
bool footprint_within(const ModelAsset &a, float s, float yaw, float mx, float mz)
{
	if (mx <= 0.0f || mz <= 0.0f)
		return false;
	auto f = rotated_footprint(a, s, yaw);
	return f[0] <= mx && f[1] <= mz;
}
bool provider_footprint_within(
		ModelProvider &p, const std::string &k, float s, float yaw, float mx, float mz)
{
	auto a = p.fetch(k);
	return a && footprint_within(*a, s, yaw, mx, mz);
}
bool place_provider_model_centered_limited(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float gy, float z, float s, float yaw, float mx,
		float mz)
{
	auto a = p.fetch(k);
	if (!a || !footprint_within(*a, s, yaw, mx, mz))
		return false;
	auto off = rotated_center_offset(*a, s, yaw);
	return place_model_asset(e, *a, x - off[0], gy - a->min[1] * s, z - off[2], s, yaw);
}
bool place_provider_model_height_centered_limited(ModelProvider &p,
		world_editor::WorldEditor &e, const std::string &k, float x, float gy, float z,
		float h, float yaw, float mx, float mz)
{
	auto a = p.fetch(k);
	if (!a || h <= 0.0f)
		return false;
	const float mh = a->max[1] - a->min[1];
	if (mh <= 0.0f)
		return false;
	const float s = h / mh;
	if (!footprint_within(*a, s, yaw, mx, mz))
		return false;
	auto off = rotated_center_offset(*a, s, yaw);
	return place_model_asset(e, *a, x - off[0], gy - a->min[1] * s, z - off[2], s, yaw);
}
bool place_provider_model(ModelProvider &provider, world_editor::WorldEditor &editor,
		const std::string &key, float x, float y, float z, float scale, float yaw)
{
	auto asset = provider.fetch(key);
	return asset && place_model_asset(editor, *asset, x, y, z, scale, yaw);
}
bool place_provider_model_height(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float y, float z, float h, float yaw)
{
	auto a = p.fetch(k);
	if (!a || h <= 0.0f)
		return false;
	const float model_h = a->max[1] - a->min[1];
	if (model_h <= 0.0f)
		return false;
	return place_model_asset(e, *a, x, y, z, h / model_h, yaw);
}
bool place_provider_model_rooted(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float gy, float z, float scale, float yaw)
{
	auto a = p.fetch(k);
	if (!a)
		return false;
	return place_model_asset(e, *a, x, gy - a->min[1] * scale, z, scale, yaw);
}
bool place_provider_model_height_rooted(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float gy, float z, float h, float yaw)
{
	auto a = p.fetch(k);
	if (!a || h <= 0.0f)
		return false;
	const float mh = a->max[1] - a->min[1];
	if (mh <= 0.0f)
		return false;
	const float s = h / mh;
	return place_model_asset(e, *a, x, gy - a->min[1] * s, z, s, yaw);
}
bool place_provider_model_centered(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float gy, float z, float scale, float yaw)
{
	auto a = p.fetch(k);
	if (!a)
		return false;
	auto off = rotated_center_offset(*a, scale, yaw);
	return place_model_asset(
			e, *a, x - off[0], gy - a->min[1] * scale, z - off[2], scale, yaw);
}
bool place_provider_model_height_centered(ModelProvider &p, world_editor::WorldEditor &e,
		const std::string &k, float x, float gy, float z, float h, float yaw)
{
	auto a = p.fetch(k);
	if (!a || h <= 0.0f)
		return false;
	const float mh = a->max[1] - a->min[1];
	if (mh <= 0.0f)
		return false;
	const float s = h / mh;
	auto off = rotated_center_offset(*a, s, yaw);
	return place_model_asset(e, *a, x - off[0], gy - a->min[1] * s, z - off[2], s, yaw);
}
}
