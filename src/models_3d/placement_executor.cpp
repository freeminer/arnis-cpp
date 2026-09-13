#include "placement_executor.h"
#include "custom/archetypes.h"
#include "pipeline.h"
#include "voxelize.h"
#include "wikidata/placement.h"
#include "wikidata/stl.h"
#include "../land_cover/land_cover.h"
#include "palette.h"
#include "../colors.h"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <optional>

namespace arnis::models_3d
{
namespace
{
std::vector<Voxel> voxelize_asset(
		const ModelAsset &asset, const WorldTransform &transform)
{
	return asset.format == ModelFormat::GLB
				   ? voxelize_glb(asset.bytes, transform)
				   : voxelize_stl(parse_binary_stl(asset.bytes), transform);
}

// Rust's Wikidata STL path treats STL as an interchange format rather than
// assuming its axes or unit scale.  Most files are Z-up, but scanned landmarks
// are commonly Y-up; infer the exceptional axis from the dimensions and fit
// the horizontal footprint and height independently when OSM supplies both.
struct StlFit
{
	std::array<std::pair<std::size_t, float>, 3> axes{};
	std::array<float, 3> scale{};
};

std::size_t stl_up_axis(const std::array<float, 3> &extent)
{
	std::array<std::pair<std::size_t, float>, 3> sorted{
			{{0, extent[0]}, {1, extent[1]}, {2, extent[2]}}};
	std::sort(sorted.begin(), sorted.end(),
			[](const auto &a, const auto &b) { return a.second < b.second; });
	const auto [min_axis, min_extent] = sorted[0];
	const auto [unused, median_extent] = sorted[1];
	const auto [max_axis, max_extent] = sorted[2];
	(void)unused;
	if (median_extent <= 1e-3f)
		return 2;
	if (max_extent / median_extent > 1.10f)
		return max_axis;
	if (min_extent / median_extent < .5f)
		return min_axis;
	return 2;
}

std::optional<StlFit> derive_stl_fit(const std::array<float, 3> &minimum,
		const std::array<float, 3> &maximum, const wikidata::Placement &placement,
		double blocks_per_meter)
{
	std::array<float, 3> raw{};
	for (std::size_t i = 0; i < raw.size(); ++i)
		raw[i] = maximum[i] - minimum[i];
	if (std::all_of(raw.begin(), raw.end(), [](float v) { return v < 1e-3f; }))
		return {};
	const auto up = stl_up_axis(raw);
	StlFit fit;
	if (up == 0)
		fit.axes = {{{1, -1}, {0, 1}, {2, 1}}};
	else if (up == 1)
		fit.axes = {{{0, 1}, {1, 1}, {2, 1}}};
	else
		fit.axes = {{{0, 1}, {2, 1}, {1, -1}}};
	std::array<float, 3> world_extent{};
	for (std::size_t i = 0; i < world_extent.size(); ++i)
		world_extent[i] = raw[fit.axes[i].first];
	std::optional<float> xz, y;
	if (placement.xz_extent_m && *placement.xz_extent_m > 0) {
		const auto widest = std::max(world_extent[0], world_extent[2]);
		if (widest > 1e-3f)
			xz = float(*placement.xz_extent_m * blocks_per_meter) / widest;
	}
	if (placement.height_m && *placement.height_m > 0 && world_extent[1] > 1e-3f)
		y = float(*placement.height_m * blocks_per_meter) / world_extent[1];
	if (!xz && !y)
		return {};
	const float sx = xz.value_or(*y), sy = y.value_or(*xz), sz = xz.value_or(*y);
	if (!std::isfinite(sx) || !std::isfinite(sy) || !std::isfinite(sz) || sx <= 0 ||
			sy <= 0 || sz <= 0)
		return {};
	fit.scale = {sx, sy, sz};
	return fit;
}

std::array<float, 3> apply_stl_fit(const std::array<float, 3> &point, const StlFit &fit)
{
	std::array<float, 3> out{};
	for (std::size_t axis = 0; axis < out.size(); ++axis)
		out[axis] = point[fit.axes[axis].first] * fit.axes[axis].second * fit.scale[axis];
	return out;
}

std::vector<Voxel> voxelize_fitted_stl(const ModelAsset &asset,
		const wikidata::Placement &placement, double blocks_per_meter,
		const WorldTransform &transform)
{
	const auto triangles = parse_binary_stl(asset.bytes);
	const auto bounds = stl_bbox(triangles);
	auto fit = derive_stl_fit(bounds.first, bounds.second, placement, blocks_per_meter);
	if (!fit)
		return {};
	const auto centre = [&](std::size_t axis) {
		const auto [source, sign] = fit->axes[axis];
		return (bounds.first[source] + bounds.second[source]) * .5f * sign *
			   fit->scale[axis];
	};
	const float centre_x = centre(0), centre_z = centre(2);
	std::vector<Triangle> fitted;
	fitted.reserve(triangles.size());
	for (const auto &triangle : triangles) {
		Triangle out{};
		for (std::size_t v = 0; v < triangle.size(); ++v) {
			out[v] = apply_stl_fit(triangle[v], *fit);
			out[v][0] -= centre_x;
			out[v][2] -= centre_z;
		}
		fitted.push_back(out);
	}
	return voxelize_uniform_triangles(fitted, transform, block_definitions::STONE_BRICKS);
}

Block named_palette_block(const std::string &name)
{
	std::string n = name;
	n.erase(n.begin(), std::find_if(n.begin(), n.end(),
							   [](unsigned char c) { return !std::isspace(c); }));
	n.erase(std::find_if(n.rbegin(), n.rend(),
					[](unsigned char c) { return !std::isspace(c); })
					.base(),
			n.end());
	std::transform(n.begin(), n.end(), n.begin(),
			[](unsigned char c) { return std::toupper(c); });
	if (n == "STONE_BRICKS")
		return STONE_BRICKS;
	if (n == "STONE")
		return STONE;
	if (n == "COBBLESTONE")
		return COBBLESTONE;
	if (n == "CRACKED_STONE_BRICKS")
		return CRACKED_STONE_BRICKS;
	if (n == "CHISELED_STONE_BRICKS")
		return CHISELED_STONE_BRICKS;
	if (n == "ANDESITE")
		return ANDESITE;
	if (n == "POLISHED_ANDESITE")
		return POLISHED_ANDESITE;
	if (n == "SMOOTH_STONE")
		return SMOOTH_STONE;
	if (n == "DEEPSLATE_BRICKS")
		return DEEPSLATE_BRICKS;
	if (n == "WHITE_CONCRETE")
		return WHITE_CONCRETE;
	if (n == "QUARTZ_BLOCK")
		return QUARTZ_BLOCK;
	if (n == "SANDSTONE")
		return SANDSTONE;
	if (n == "GRAY_CONCRETE")
		return GRAY_CONCRETE;
	if (n == "LIGHT_GRAY_CONCRETE")
		return LIGHT_GRAY_CONCRETE;
	if (n == "BRICK")
		return BRICK;
	if (n == "MUD_BRICKS")
		return MUD_BRICKS;
	if (n == "GRANITE")
		return GRANITE;
	if (n == "POLISHED_GRANITE")
		return POLISHED_GRANITE;
	if (n == "DIORITE")
		return DIORITE;
	if (n == "POLISHED_DIORITE")
		return POLISHED_DIORITE;
	if (n == "BLACKSTONE")
		return BLACKSTONE;
	if (n == "POLISHED_BLACKSTONE")
		return POLISHED_BLACKSTONE;
	if (n == "POLISHED_BLACKSTONE_BRICKS")
		return POLISHED_BLACKSTONE_BRICKS;
	if (n == "COBBLED_DEEPSLATE")
		return COBBLED_DEEPSLATE;
	if (n == "DEEPSLATE")
		return DEEPSLATE;
	if (n == "POLISHED_DEEPSLATE")
		return POLISHED_DEEPSLATE;
	if (n == "OAK_PLANKS")
		return OAK_PLANKS;
	if (n == "SPRUCE_PLANKS")
		return SPRUCE_PLANKS;
	if (n == "DARK_OAK_PLANKS")
		return DARK_OAK_PLANKS;
	if (n == "OAK_LOG")
		return OAK_LOG;
	if (n == "SPRUCE_LOG")
		return SPRUCE_LOG;
	if (n == "DARK_OAK_LOG")
		return DARK_OAK_LOG;
	if (n == "TERRACOTTA")
		return TERRACOTTA;
	if (n == "WHITE_TERRACOTTA")
		return WHITE_TERRACOTTA;
	if (n == "BROWN_TERRACOTTA")
		return BROWN_TERRACOTTA;
	if (n == "RED_TERRACOTTA")
		return RED_TERRACOTTA;
	if (n == "BLUE_TERRACOTTA")
		return BLUE_TERRACOTTA;
	if (n == "CYAN_TERRACOTTA")
		return CYAN_TERRACOTTA;
	if (n == "BLACK_CONCRETE")
		return BLACK_CONCRETE;
	if (n == "BLUE_CONCRETE")
		return BLUE_CONCRETE;
	if (n == "GREEN_CONCRETE")
		return GREEN_CONCRETE;
	if (n == "RED_CONCRETE")
		return RED_CONCRETE;
	if (n == "YELLOW_CONCRETE")
		return YELLOW_CONCRETE;
	if (n == "ORANGE_TERRACOTTA")
		return ORANGE_TERRACOTTA;
	if (n == "GOLD_BLOCK")
		return GOLD_BLOCK;
	if (n == "IRON_BLOCK")
		return IRON_BLOCK;
	if (n == "SNOW_BLOCK")
		return SNOW_BLOCK;
	return Block{};
}

std::vector<std::pair<float, std::vector<Block>>> resolve_layers(
		const std::vector<WikidataEntry::PaletteLayer> &layers)
{
	std::vector<std::pair<float, std::vector<Block>>> out;
	for (const auto &layer : layers) {
		if (!std::isfinite(layer.y_max_frac))
			continue;
		std::vector<Block> pool;
		if (layer.hex) {
			const auto hex = layer.hex->starts_with('#') ? *layer.hex : "#" + *layer.hex;
			if (auto rgb = color_text_to_rgb_tuple(hex))
				pool = closest_blocks(*rgb, 5);
		} else
			for (const auto &name : layer.blocks) {
				const auto b = named_palette_block(name);
				if (b != Block{})
					pool.push_back(b);
			}
		std::sort(pool.begin(), pool.end(),
				[](const Block &a, const Block &b) { return a.id() < b.id(); });
		pool.erase(
				std::unique(pool.begin(), pool.end(),
						[](const Block &a, const Block &b) { return a.id() == b.id(); }),
				pool.end());
		if (!pool.empty())
			out.emplace_back(std::clamp(layer.y_max_frac, 0.0f, 1.0f), std::move(pool));
	}
	std::sort(out.begin(), out.end(),
			[](const auto &a, const auto &b) { return a.first < b.first; });
	return out;
}

bool place_grounded_voxels(world_editor::WorldEditor &editor, std::vector<Voxel> voxels,
		int ground_y, int elevation, ModelPlacementStats &stats,
		const std::vector<Block> *fallback_palette = nullptr,
		const std::vector<std::pair<float, std::vector<Block>>> *layers = nullptr,
		std::uint64_t palette_seed = 0)
{
	if (voxels.empty())
		return false;
	if (layers && !layers->empty()) {
		int min_y = voxels.front().position[1], max_y = min_y;
		for (const auto &v : voxels) {
			min_y = std::min(min_y, v.position[1]);
			max_y = std::max(max_y, v.position[1]);
		}
		const float span = float(std::max(1, max_y - min_y));
		for (auto &voxel : voxels)
			if (voxel.block == block_definitions::STONE_BRICKS) {
				const float frac =
						std::clamp(float(voxel.position[1] - min_y) / span, 0.0f, 1.0f);
				auto it = std::find_if(layers->begin(), layers->end(),
						[frac](const auto &l) { return frac <= l.first; });
				const auto &pool = (it == layers->end() ? layers->back() : *it).second;
				const auto h =
						land_cover::coord_hash(voxel.position[0], voxel.position[2]) ^
						palette_seed;
				voxel.block = pool[h % pool.size()];
			}
	} else if (fallback_palette && !fallback_palette->empty())
		for (auto &voxel : voxels)
			if (voxel.block == block_definitions::STONE_BRICKS) {
				const auto h =
						land_cover::coord_hash(voxel.position[0], voxel.position[2]) ^
						palette_seed;
				voxel.block = (*fallback_palette)[h % fallback_palette->size()];
			}
	int min_y = voxels.front().position[1];
	for (const auto &voxel : voxels)
		min_y = std::min(min_y, voxel.position[1]);
	const int dy = ground_y + elevation - min_y;
	if (dy)
		for (auto &voxel : voxels)
			voxel.position[1] += dy;
	place_voxels(editor, voxels);
	++stats.placed;
	stats.voxels += voxels.size();
	return true;
}

bool place_grounded(world_editor::WorldEditor &editor, const ModelAsset &asset,
		const WorldTransform &transform, int ground_y, int elevation,
		ModelPlacementStats &stats, const std::vector<Block> *fallback_palette = nullptr,
		const std::vector<std::pair<float, std::vector<Block>>> *layers = nullptr,
		std::uint64_t palette_seed = 0)
{
	return place_grounded_voxels(editor, voxelize_asset(asset, transform), ground_y,
			elevation, stats, fallback_palette, layers, palette_seed);
}

int ground_for(
		world_editor::WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	return lowest_ground_in_bbox(editor, min_x, min_z, max_x, max_z);
}
}

ModelPlacementStats place_wikidata_prescan(ModelProvider &provider,
		world_editor::WorldEditor &editor, const wikidata::PrescanResult &prescan,
		double blocks_per_meter)
{
	ModelPlacementStats stats;
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0)
		return stats;
	for (const auto &placement : prescan.placements) {
		++stats.attempted;
		auto asset = provider.fetch(placement.qid);
		if (!asset)
			continue;
		const int ground =
				ground_for(editor, placement.footprint.min_x, placement.footprint.min_z,
						placement.footprint.max_x, placement.footprint.max_z);
		if (asset->format == ModelFormat::BinarySTL) {
			const auto triangles = parse_binary_stl(asset->bytes);
			const auto bounds = stl_bbox(triangles);
			auto fit = derive_stl_fit(
					bounds.first, bounds.second, placement, blocks_per_meter);
			if (!fit)
				continue;
			std::array<float, 3> raw{};
			std::array<float, 3> extent{};
			for (std::size_t i = 0; i < raw.size(); ++i)
				raw[i] = bounds.second[i] - bounds.first[i];
			for (std::size_t i = 0; i < extent.size(); ++i)
				extent[i] = std::abs(raw[fit->axes[i].first] * fit->scale[i]) /
							float(blocks_per_meter);
			const float largest = std::max({extent[0], extent[1], extent[2]});
			if (!std::isfinite(largest) || std::max(extent[0], extent[2]) > 225.0f ||
					extent[1] > 600.0f || largest < 2.0f)
				continue;
			WorldTransform transform(0, 1, {0, 0, 0}, {1, 1, 1}, placement.yaw_degrees,
					float(placement.anchor_x), float(ground), float(placement.anchor_z));
			const auto layers = resolve_layers(placement.palette_layers);
			std::uint64_t qid_seed = 0xcbf29ce484222325ULL;
			for (unsigned char c : placement.qid) {
				qid_seed ^= c;
				qid_seed *= 0x100000001b3ULL;
			}
			place_grounded_voxels(editor,
					voxelize_fitted_stl(*asset, placement, blocks_per_meter, transform),
					ground, 0, stats, &placement.palette,
					layers.empty() ? nullptr : &layers, qid_seed);
			continue;
		}
		float scale = 1.0f;
		if (placement.height_m && *placement.height_m > 0)
			scale = wikidata::scale_for_height(
					*asset, float(*placement.height_m * blocks_per_meter));
		else if (placement.xz_extent_m && *placement.xz_extent_m > 0) {
			const float extent = std::max(
					asset->max[0] - asset->min[0], asset->max[2] - asset->min[2]);
			if (extent > 1e-4f)
				scale = float(*placement.xz_extent_m * blocks_per_meter) / extent;
		}
		if (!std::isfinite(scale) || scale <= 0)
			continue;
		// Same final-world safety envelope as Rust's GLB/STL placement code.
		// `scale` is in blocks/model-unit here, so convert the transformed bbox
		// back to metres before expensive voxelisation.
		const float ex =
				std::abs(asset->max[0] - asset->min[0]) * scale / float(blocks_per_meter);
		const float ey =
				std::abs(asset->max[1] - asset->min[1]) * scale / float(blocks_per_meter);
		const float ez =
				std::abs(asset->max[2] - asset->min[2]) * scale / float(blocks_per_meter);
		const float largest = std::max({ex, ey, ez});
		if (!std::isfinite(largest) || std::max(ex, ez) > 225.0f || ey > 600.0f ||
				largest < 2.0f)
			continue;
		const auto offset =
				rotated_center_offset(*asset, scale, float(placement.yaw_degrees));
		WorldTransform transform(0, scale, {0, 0, 0}, {1, 1, 1}, placement.yaw_degrees,
				float(placement.anchor_x) - offset[0],
				float(ground) - asset->min[1] * scale,
				float(placement.anchor_z) - offset[2]);
		const auto layers = resolve_layers(placement.palette_layers);
		std::uint64_t qid_seed = 0xcbf29ce484222325ULL;
		for (unsigned char c : placement.qid) {
			qid_seed ^= c;
			qid_seed *= 0x100000001b3ULL;
		}
		place_grounded(editor, *asset, transform, ground, 0, stats, &placement.palette,
				layers.empty() ? nullptr : &layers, qid_seed);
	}
	return stats;
}

ModelPlacementStats place_three_dmr_prescan(three_dmr::Client &provider,
		world_editor::WorldEditor &editor, const three_dmr::PrescanResult &prescan,
		double blocks_per_meter)
{
	ModelPlacementStats stats;
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0)
		return stats;
	for (const auto &placement : prescan.placements) {
		++stats.attempted;
		auto asset = provider.fetch(std::to_string(placement.model_id));
		auto info = provider.info(placement.model_id);
		if (!asset || !info)
			continue;
		const int ground =
				ground_for(editor, placement.footprint.min_x, placement.footprint.min_z,
						placement.footprint.max_x, placement.footprint.max_z);
		WorldTransform transform(info->rotation, info->scale, info->translation,
				{float(blocks_per_meter), float(blocks_per_meter),
						float(blocks_per_meter)},
				placement.world_yaw_degrees, float(placement.anchor_x), float(ground),
				float(placement.anchor_z));
		place_grounded(editor, *asset, transform, ground, 0, stats);
	}
	return stats;
}

ModelPlacementStats place_plane_prescan(ModelProvider &provider,
		world_editor::WorldEditor &editor,
		const std::vector<custom::plane::Placement> &placements, double blocks_per_meter)
{
	ModelPlacementStats stats;
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0 || placements.empty())
		return stats;
	auto asset = provider.fetch(custom::plane_model_key());
	if (!asset)
		return stats;
	const double intrinsic =
			custom::plane_model_scale(*asset, custom::PLANE_LENGTH_M * blocks_per_meter);
	if (!std::isfinite(intrinsic) || intrinsic <= 0)
		return stats;
	for (const auto &placement : placements) {
		++stats.attempted;
		const int ground =
				ground_for(editor, placement.footprint.min_x, placement.footprint.min_z,
						placement.footprint.max_x, placement.footprint.max_z);
		WorldTransform transform(0, intrinsic, {0, 0, 0}, {1, 1, 1},
				placement.yaw_degrees, float(placement.anchor_x), float(ground),
				float(placement.anchor_z));
		place_grounded(editor, *asset, transform.pitched(placement.pitch_degrees), ground,
				placement.elevation_blocks, stats);
	}
	return stats;
}

ModelPlacementStats place_stadium_prescan(ModelProvider &provider,
		world_editor::WorldEditor &editor,
		const std::vector<custom::stadium::Placement> &placements,
		double blocks_per_meter)
{
	ModelPlacementStats stats;
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0 || placements.empty())
		return stats;
	auto asset = provider.fetch(custom::stadium_model_key());
	if (!asset)
		return stats;
	for (const auto &placement : placements) {
		++stats.attempted;
		const double scale =
				custom::stadium_model_scale(*asset, placement.long_m * blocks_per_meter,
						placement.short_m * blocks_per_meter);
		if (!std::isfinite(scale) || scale <= 0)
			continue;
		const int ground =
				ground_for(editor, placement.footprint.min_x, placement.footprint.min_z,
						placement.footprint.max_x, placement.footprint.max_z);
		const auto offset =
				rotated_center_offset(*asset, float(scale), float(placement.yaw_degrees));
		WorldTransform transform(0, scale, {0, 0, 0}, {1, 1, 1}, placement.yaw_degrees,
				float(placement.anchor_x) - offset[0],
				float(ground) - asset->min[1] * float(scale),
				float(placement.anchor_z) - offset[2]);
		place_grounded(editor, *asset, transform, ground, 0, stats);
	}
	return stats;
}
}
