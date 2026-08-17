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
#include <cctype>
#include <cmath>

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

bool place_grounded(world_editor::WorldEditor &editor, const ModelAsset &asset,
		const WorldTransform &transform, int ground_y, int elevation,
		ModelPlacementStats &stats, const std::vector<Block> *fallback_palette = nullptr,
		const std::vector<std::pair<float, std::vector<Block>>> *layers = nullptr,
		std::uint64_t palette_seed = 0)
{
	auto voxels = voxelize_asset(asset, transform);
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
