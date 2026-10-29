#include "placement_executor.h"
#include "custom/archetypes.h"
#include "pipeline.h"
#include "voxelize.h"
#include "wikidata/placement.h"
#include "wikidata/stl.h"
#include "../land_cover/land_cover.h"
#include "../water_depth.h"
#include "palette.h"
#include "../colors.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <map>
#include <new>
#include <optional>
#include <thread>
#include <unordered_map>
#include <queue>

namespace arnis::models_3d
{
namespace
{
constexpr int TREE_SWEEP_REACH = 10;

struct ModelOverride
{
	std::uint64_t id;
	double yaw_degrees;
	double scale;
	std::vector<std::pair<std::array<float, 3>, Block>> palette;
};

const ModelOverride *model_override(std::uint64_t id)
{
	// Keep per-model corrections as data, matching the Rust 3DMR placement
	// override table. A model can independently correct orientation, size, and
	// material matching without adding another branch to placement execution.
	static const std::array<ModelOverride, 1> overrides{{
			{4, 44.3, 1.035,
					{{{77.0f / 255.0f, 51.0f / 255.0f, 36.0f / 255.0f}, BROWN_TERRACOTTA},
							{{137.0f / 255.0f, 104.0f / 255.0f, 79.0f / 255.0f},
									MUD_BRICKS}}},
	}};
	const auto it = std::find_if(overrides.begin(), overrides.end(),
			[&](const ModelOverride &entry) { return entry.id == id; });
	return it == overrides.end() ? nullptr : &*it;
}

std::uint64_t palette_hash(std::uint64_t seed, const std::array<int, 3> &position)
{
	// Exact counterpart of Rust's splitmix-style pick_voxel_block.  Using the
	// shared land-cover hash here caused visible repeating patterns and made
	// palette selection differ between the two generators.
	std::uint64_t h = seed;
	for (const int coordinate : position) {
		h ^= static_cast<std::uint64_t>(static_cast<std::int64_t>(coordinate));
		h = (h ^ (h >> 30)) * 0xbf58476d1ce4e5b9ULL;
		h = (h ^ (h >> 27)) * 0x94d049bb133111ebULL;
		h ^= h >> 31;
	}
	return h;
}

// Match Rust stamp_voxels: fill enclosed footprint holes before the tree sweep
// so trees beneath legs and platform edges are removed too.
void clear_trees_under_model(
		world_editor::WorldEditor &editor, const std::vector<Voxel> &voxels)
{
	if (voxels.empty())
		return;
	int min_x = voxels.front().position[0], max_x = min_x;
	int min_z = voxels.front().position[2], max_z = min_z;
	int top = voxels.front().position[1];
	for (const auto &voxel : voxels) {
		min_x = std::min(min_x, voxel.position[0]);
		max_x = std::max(max_x, voxel.position[0]);
		min_z = std::min(min_z, voxel.position[2]);
		max_z = std::max(max_z, voxel.position[2]);
		top = std::max(top, voxel.position[1]);
	}
	const auto width = static_cast<std::size_t>(max_x - min_x + 1);
	const auto height = static_cast<std::size_t>(max_z - min_z + 1);
	if (!width || !height || width > 16'000'000 / height)
		return;
	std::vector<std::uint8_t> inside(width * height, 0);
	for (const auto &voxel : voxels)
		inside[static_cast<std::size_t>(voxel.position[2] - min_z) * width +
				static_cast<std::size_t>(voxel.position[0] - min_x)] = 1;
	std::queue<std::pair<int, int>> outside;
	const auto enqueue = [&](int x, int z) {
		auto &value = inside[static_cast<std::size_t>(z) * width + x];
		if (value == 0) {
			value = 2;
			outside.emplace(x, z);
		}
	};
	for (int x = 0; x < static_cast<int>(width); ++x) {
		enqueue(x, 0);
		enqueue(x, static_cast<int>(height) - 1);
	}
	for (int z = 0; z < static_cast<int>(height); ++z) {
		enqueue(0, z);
		enqueue(static_cast<int>(width) - 1, z);
	}
	while (!outside.empty()) {
		const auto [x, z] = outside.front();
		outside.pop();
		for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
			const int nx = x + dx, nz = z + dz;
			if (nx >= 0 && nz >= 0 && nx < static_cast<int>(width) &&
					nz < static_cast<int>(height))
				enqueue(nx, nz);
		}
	}
	constexpr int tree_sweep_height = 64;
	const auto within_model_reach = [min_x, max_x, min_z, max_z](int x, int z) {
		return x >= min_x - TREE_SWEEP_REACH && x <= max_x + TREE_SWEEP_REACH &&
			   z >= min_z - TREE_SWEEP_REACH && z <= max_z + TREE_SWEEP_REACH;
	};
	for (int z = 0; z < static_cast<int>(height); ++z)
		for (int x = 0; x < static_cast<int>(width); ++x) {
			if (inside[static_cast<std::size_t>(z) * width + x] == 2)
				continue;
			const int wx = min_x + x, wz = min_z + z;
			const int ground = editor.get_ground_level(wx, wz);
			for (int y = ground; y <= std::min(ground + tree_sweep_height, top); ++y)
				if (water_depth::is_tree_block(editor.block_name_absolute(wx, y, wz)))
					water_depth::clear_tree_within(editor, wx, y, wz, within_model_reach);
		}
}

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
	// Rust's slice::sort_by is stable, so equal extents keep the original X/Y/Z
	// order. Preserve that tie-breaking when inferring a model's up axis.
	std::stable_sort(sorted.begin(), sorted.end(),
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
	// value_or evaluates its fallback even when the optional is populated.
	// At least one scale is present, but height-only and footprint-only models
	// must never dereference the other, empty optional.
	const float sx = xz ? *xz : *y;
	const float sy = y ? *y : *xz;
	const float sz = sx;
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
	if (!bounds)
		return {};
	auto fit = derive_stl_fit(bounds->first, bounds->second, placement, blocks_per_meter);
	if (!fit)
		return {};
	const auto centre = [&](std::size_t axis) {
		const auto [source, sign] = fit->axes[axis];
		return (bounds->first[source] + bounds->second[source]) * .5f * sign *
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
	return voxelize_solid_triangles(fitted, transform, block_definitions::STONE_BRICKS);
}

Block named_palette_block(const std::string &name)
{
	std::string n = name;
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
	if (n == "CALCITE")
		return CALCITE;
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
	if (n == "DIRT")
		return DIRT;
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
	if (n == "RED_WOOL")
		return RED_WOOL;
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
	if (n == "ORANGE_WOOL")
		return ORANGE_WOOL;
	if (n == "GOLD_BLOCK")
		return GOLD_BLOCK;
	if (n == "IRON_BLOCK")
		return IRON_BLOCK;
	if (n == "SNOW_BLOCK")
		return SNOW_BLOCK;
	if (n == "SMOOTH_QUARTZ")
		return SMOOTH_QUARTZ;
	if (n == "SMOOTH_SANDSTONE")
		return SMOOTH_SANDSTONE;
	// Keep palette-layer names aligned with Rust's block_by_const_name table.
	if (n == "BLACK_TERRACOTTA")
		return BLACK_TERRACOTTA;
	if (n == "BLACK_WOOL")
		return BLACK_WOOL;
	if (n == "BLUE_TERRACOTTA")
		return BLUE_TERRACOTTA;
	if (n == "BLUE_WOOL")
		return BLUE_WOOL;
	if (n == "BROWN_CONCRETE")
		return BROWN_CONCRETE;
	if (n == "COARSE_DIRT")
		return COARSE_DIRT;
	if (n == "CYAN_CONCRETE")
		return CYAN_CONCRETE;
	if (n == "END_STONE_BRICKS")
		return END_STONE_BRICKS;
	if (n == "GRAY_TERRACOTTA")
		return GRAY_TERRACOTTA;
	if (n == "GREEN_CONCRETE")
		return GREEN_CONCRETE;
	if (n == "GREEN_WOOL")
		return GREEN_WOOL;
	if (n == "HAY_BALE")
		return HAY_BALE;
	if (n == "LIGHT_BLUE_CONCRETE")
		return LIGHT_BLUE_CONCRETE;
	if (n == "LIGHT_BLUE_TERRACOTTA")
		return LIGHT_BLUE_TERRACOTTA;
	if (n == "LIGHT_GRAY_TERRACOTTA")
		return LIGHT_GRAY_TERRACOTTA;
	if (n == "LIME_CONCRETE")
		return LIME_CONCRETE;
	if (n == "MAGENTA_CONCRETE")
		return MAGENTA_CONCRETE;
	if (n == "MOSS_BLOCK")
		return MOSS_BLOCK;
	if (n == "MOSSY_COBBLESTONE")
		return MOSSY_COBBLESTONE;
	if (n == "MOSSY_STONE_BRICKS")
		return MOSSY_STONE_BRICKS;
	if (n == "NETHER_BRICK")
		return NETHER_BRICK;
	if (n == "NETHERITE_BLOCK")
		return NETHERITE_BLOCK;
	if (n == "PURPLE_CONCRETE")
		return PURPLE_CONCRETE;
	if (n == "QUARTZ_BRICKS")
		return QUARTZ_BRICKS;
	if (n == "RED_NETHER_BRICKS")
		return RED_NETHER_BRICKS;
	if (n == "WAXED_COPPER_BLOCK")
		return WAXED_COPPER_BLOCK;
	if (n == "WAXED_EXPOSED_COPPER")
		return WAXED_EXPOSED_COPPER;
	if (n == "WAXED_OXIDIZED_COPPER")
		return WAXED_OXIDIZED_COPPER;
	if (n == "WHITE_WOOL")
		return WHITE_WOOL;
	if (n == "YELLOW_TERRACOTTA")
		return YELLOW_TERRACOTTA;
	if (n == "YELLOW_WOOL")
		return YELLOW_WOOL;
	return Block{};
}

bool place_grounded_voxels(world_editor::WorldEditor &editor, std::vector<Voxel> voxels,
		int ground_y, int elevation, ModelPlacementStats &stats,
		const std::vector<Block> *fallback_palette = nullptr,
		const std::optional<wikidata::ResolvedPaletteLayers> *layers = nullptr,
		std::uint64_t palette_seed = 0, bool wikidata_foundation = false)
{
	if (voxels.empty())
		return false;
	if (layers && *layers && !(*layers)->empty()) {
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
				auto it = std::find_if((*layers)->y_max_frac.begin(),
						(*layers)->y_max_frac.end(),
						[frac](float y_max) { return frac <= y_max; });
				const auto layer_index =
						it == (*layers)->y_max_frac.end()
								? (*layers)->blocks.size() - 1
								: static_cast<std::size_t>(
										  it - (*layers)->y_max_frac.begin());
				const auto &pool = (*layers)->blocks[layer_index];
				const auto h = palette_hash(palette_seed, voxel.position);
				voxel.block = pool[h % pool.size()];
			}
	} else if (fallback_palette && !fallback_palette->empty())
		for (auto &voxel : voxels)
			if (voxel.block == block_definitions::STONE_BRICKS) {
				const auto h = palette_hash(palette_seed, voxel.position);
				voxel.block = (*fallback_palette)[h % fallback_palette->size()];
			}
	int min_y = voxels.front().position[1];
	for (const auto &voxel : voxels)
		min_y = std::min(min_y, voxel.position[1]);
	int target_ground = ground_y;
	if (wikidata_foundation) {
		constexpr int load_bearing_band = 2;
		std::map<std::pair<int, int>, int> bottoms;
		for (const auto &voxel : voxels) {
			auto [it, inserted] = bottoms.try_emplace(
					std::pair{voxel.position[0], voxel.position[2]}, voxel.position[1]);
			if (!inserted)
				it->second = std::min(it->second, voxel.position[1]);
		}
		std::vector<int> levels;
		for (const auto &[column, y] : bottoms)
			if (y <= min_y + load_bearing_band)
				levels.push_back(editor.get_ground_level(column.first, column.second));
		if (!levels.empty()) {
			std::sort(levels.begin(), levels.end());
			target_ground = levels[levels.size() / 2];
		}
	}
	const int dy = target_ground + elevation - min_y;
	if (dy)
		for (auto &voxel : voxels)
			voxel.position[1] += dy;
	clear_trees_under_model(editor, voxels);
	place_voxels(editor, voxels);
	if (wikidata_foundation) {
		constexpr int load_bearing_band = 2;
		constexpr int max_foundation_depth = 64;
		std::map<std::pair<int, int>, std::pair<int, Block>> bottoms;
		for (const auto &voxel : voxels) {
			auto [it, inserted] =
					bottoms.try_emplace(std::pair{voxel.position[0], voxel.position[2]},
							std::pair{voxel.position[1], voxel.block});
			if (!inserted && voxel.position[1] < it->second.first)
				it->second = {voxel.position[1], voxel.block};
		}
		const int base_y = min_y + dy;
		for (const auto &[column, bottom] : bottoms) {
			const auto [x, z] = column;
			const auto [y, block] = bottom;
			if (y > base_y + load_bearing_band)
				continue;
			const int floor =
					std::max(y - max_foundation_depth, editor.get_ground_level(x, z) + 1);
			for (int fy = floor; fy < y; ++fy)
				editor.set_block_absolute(block, x, fy, z, std::nullopt,
						std::optional<std::vector<Block>>(std::vector<Block>{}));
		}
	}
	++stats.placed;
	stats.voxels += voxels.size();
	return true;
}

bool place_grounded(world_editor::WorldEditor &editor, const ModelAsset &asset,
		const WorldTransform &transform, int ground_y, int elevation,
		ModelPlacementStats &stats, const std::vector<Block> *fallback_palette = nullptr,
		const std::optional<wikidata::ResolvedPaletteLayers> *layers = nullptr,
		std::uint64_t palette_seed = 0, bool wikidata_foundation = false)
{
	return place_grounded_voxels(editor, voxelize_asset(asset, transform), ground_y,
			elevation, stats, fallback_palette, layers, palette_seed,
			wikidata_foundation);
}

int ground_for(
		world_editor::WorldEditor &editor, int min_x, int min_z, int max_x, int max_z)
{
	return lowest_ground_in_bbox(editor, min_x, min_z, max_x, max_z);
}
}

std::optional<wikidata::ResolvedPaletteLayers> wikidata::resolve_palette_layers(
		const std::optional<std::vector<WikidataEntry::PaletteLayer>> &source)
{
	if (!source)
		return std::nullopt;
	std::vector<std::pair<float, std::vector<Block>>> entries;
	for (const auto &layer : *source) {
		if (!std::isfinite(layer.y_max_frac))
			continue;
		std::vector<Block> pool;
		if (layer.hex) {
			const auto first_hex = layer.hex->find_first_not_of('#');
			const auto hex = first_hex == std::string::npos
									 ? std::string("#")
									 : "#" + layer.hex->substr(first_hex);
			if (auto rgb = color_text_to_rgb_tuple(hex))
				pool = closest_blocks(*rgb, 5);
		} else {
			for (const auto &name : layer.blocks) {
				const auto block = named_palette_block(name);
				if (block != Block{})
					pool.push_back(block);
			}
		}
		if (pool.empty())
			continue;
		entries.emplace_back(layer.y_max_frac, std::move(pool));
	}
	if (entries.empty())
		return std::nullopt;
	const auto total_order_key = [](float value) {
		constexpr std::uint32_t sign = std::uint32_t{1} << 31;
		const auto bits = std::bit_cast<std::uint32_t>(value);
		return bits & sign ? ~bits : bits ^ sign;
	};
	std::stable_sort(entries.begin(), entries.end(), [&](const auto &a, const auto &b) {
		return total_order_key(a.first) < total_order_key(b.first);
	});
	wikidata::ResolvedPaletteLayers resolved;
	resolved.y_max_frac.reserve(entries.size());
	resolved.blocks.reserve(entries.size());
	for (auto &[limit, pool] : entries) {
		resolved.y_max_frac.push_back(limit);
		resolved.blocks.push_back(std::move(pool));
	}
	return resolved.empty()
				   ? std::nullopt
				   : std::optional<wikidata::ResolvedPaletteLayers>(std::move(resolved));
}

ModelPlacementStats place_wikidata_prescan(ModelProvider &provider,
		world_editor::WorldEditor &editor, const wikidata::PrescanResult &prescan,
		double blocks_per_meter, std::optional<std::span<const std::size_t>> indices)
{
	ModelPlacementStats stats;
	const std::size_t placement_count =
			indices ? indices->size() : prescan.placements.size();
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0 || placement_count == 0)
		return stats;
	// Rust's prescan fetches each distinct Commons URL once and shares the
	// resulting bytes across every placement that references it. Keep that
	// behavior here too: providers may be keyed by QID, while multiple QIDs can
	// resolve to the same model URL.
	std::unordered_map<std::string, std::optional<ModelAsset>> assets_by_url;
	for (std::size_t placement_index = 0; placement_index < placement_count;
			++placement_index) {
		const auto &placement = prescan.placements.at(
				indices ? (*indices)[placement_index] : placement_index);
		++stats.attempted;
		// Keep one malformed Commons asset from aborting the remaining
		// Wikidata placements, matching Rust's per-model error handling.
		try {
			const auto *entry = lookup_wikidata(placement.qid);
			const std::string cache_key =
					entry && !entry->url.empty() ? entry->url : placement.qid;
			auto [cached, inserted] = assets_by_url.try_emplace(cache_key);
			if (inserted)
				cached->second = provider.fetch(placement.qid);
			if (!cached->second)
				continue;
			const auto &asset = *cached->second;
			const int ground = ground_for(editor, placement.footprint.min_x,
					placement.footprint.min_z, placement.footprint.max_x,
					placement.footprint.max_z);
			if (asset.format == ModelFormat::BinarySTL) {
				const auto triangles = parse_binary_stl(asset.bytes);
				const auto bounds = stl_bbox(triangles);
				if (!bounds)
					continue;
				auto fit = derive_stl_fit(
						bounds->first, bounds->second, placement, blocks_per_meter);
				if (!fit)
					continue;
				std::array<float, 3> raw{};
				std::array<float, 3> extent{};
				for (std::size_t i = 0; i < raw.size(); ++i)
					raw[i] = bounds->second[i] - bounds->first[i];
				for (std::size_t i = 0; i < extent.size(); ++i)
					extent[i] = std::abs(raw[fit->axes[i].first] * fit->scale[i]) /
								float(blocks_per_meter);
				const float largest = std::max({extent[0], extent[1], extent[2]});
				if (!std::isfinite(largest) || std::max(extent[0], extent[2]) > 225.0f ||
						extent[1] > 600.0f || largest < 2.0f)
					continue;
				WorldTransform transform(0, 1, {0, 0, 0}, {1, 1, 1},
						placement.yaw_degrees, float(placement.anchor_x), float(ground),
						float(placement.anchor_z));
				std::uint64_t qid_seed = 0xcbf29ce484222325ULL;
				for (unsigned char c : placement.qid) {
					qid_seed ^= c;
					qid_seed *= 0x100000001b3ULL;
				}
				place_grounded_voxels(editor,
						voxelize_fitted_stl(
								asset, placement, blocks_per_meter, transform),
						ground, 0, stats, &placement.palette, &placement.palette_layers,
						qid_seed, true);
				continue;
			}
			// Rust derives an intrinsic GLB scale in model/metre units, then applies
			// Args::scale in WorldTransform. Fold that world scale into `scale` here.
			// Keep Rust's guarded match order: a valid height wins; otherwise a valid
			// footprint can still size a model whose vertical bounds are degenerate.
			float intrinsic_scale = 1.0f;
			const float model_y = asset.max[1] - asset.min[1];
			const float model_xz =
					std::max(asset.max[0] - asset.min[0], asset.max[2] - asset.min[2]);
			if (placement.height_m && model_y > 1e-4f)
				intrinsic_scale = float(*placement.height_m) / model_y;
			else if (placement.xz_extent_m && model_xz > 1e-4f)
				intrinsic_scale = float(*placement.xz_extent_m) / model_xz;
			const float scale = intrinsic_scale * float(blocks_per_meter);
			if (!std::isfinite(scale) || scale <= 0)
				continue;
			// Same final-world safety envelope as Rust's GLB/STL placement code.
			// `scale` is in blocks/model-unit here, so convert the transformed bbox
			// back to metres before expensive voxelisation.
			const float ex = std::abs(asset.max[0] - asset.min[0]) * scale /
							 float(blocks_per_meter);
			const float ey = std::abs(asset.max[1] - asset.min[1]) * scale /
							 float(blocks_per_meter);
			const float ez = std::abs(asset.max[2] - asset.min[2]) * scale /
							 float(blocks_per_meter);
			const float largest = std::max({ex, ey, ez});
			if (!std::isfinite(largest) || std::max(ex, ez) > 225.0f || ey > 600.0f ||
					largest < 2.0f)
				continue;
			// Rust's Wikidata GLB path anchors the transformed model origin at the
			// OSM coordinate; it does not recenter the model's XZ bounds. Keep the
			// same origin here so off-center GLB geometry lands at the same place.
			WorldTransform transform(0, scale, {0, 0, 0}, {1, 1, 1},
					placement.yaw_degrees, float(placement.anchor_x),
					float(ground) - asset.min[1] * scale, float(placement.anchor_z));
			// Rust preserves GLB-authored material and vertex colors. Contextual
			// Wikidata palettes are only used for STL, whose triangles carry no
			// reliable material colors; do not recolor the GLB voxelizer's
			// STONE_BRICKS fallback here.
			place_grounded(editor, asset, transform, ground, 0, stats, nullptr, nullptr,
					0, true);
		} catch (const std::bad_alloc &) {
			throw;
		} catch (const std::exception &error) {
			std::cerr << "Warning: Wikidata model voxelization/fetch failed ("
					  << placement.qid << ", OSM " << placement.osm_kind << ' '
					  << placement.osm_id << "): " << error.what() << '\n';
			continue;
		}
	}
	return stats;
}

ModelPlacementStats place_three_dmr_prescan(three_dmr::Client &provider,
		world_editor::WorldEditor &editor, const three_dmr::PrescanResult &prescan,
		double blocks_per_meter, std::optional<std::span<const std::size_t>> indices)
{
	ModelPlacementStats stats;
	const std::size_t placement_count =
			indices ? indices->size() : prescan.placements.size();
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0 || placement_count == 0)
		return stats;

	// Rust fetches metadata and model bytes once per distinct 3DMR ID, then
	// reuses that pair for every OSM placement. Apart from avoiding repeated
	// cache reads and GLB parsing, this keeps duplicate tags from multiplying
	// memory use during a single generation pass.
	std::vector<std::uint64_t> model_ids;
	model_ids.reserve(placement_count);
	for (std::size_t placement_index = 0; placement_index < placement_count;
			++placement_index) {
		const auto &placement = prescan.placements.at(
				indices ? (*indices)[placement_index] : placement_index);
		model_ids.push_back(placement.model_id);
	}
	std::sort(model_ids.begin(), model_ids.end());
	model_ids.erase(std::unique(model_ids.begin(), model_ids.end()), model_ids.end());
	std::unordered_map<std::uint64_t, three_dmr::ModelInfo> infos;
	std::unordered_map<std::uint64_t, ModelAsset> assets;
	infos.reserve(model_ids.size());
	assets.reserve(model_ids.size());
	struct FetchedModel
	{
		three_dmr::ModelInfo info;
		ModelAsset asset;
	};
	std::vector<std::optional<FetchedModel>> fetched(model_ids.size());
	std::atomic_size_t next{0};
	const auto available_workers = std::max(1u, std::thread::hardware_concurrency());
	const auto rayon_like_workers = std::max<std::size_t>(
			1, static_cast<std::size_t>(static_cast<double>(available_workers) * .9));
	const auto worker_count = std::min(model_ids.size(), rayon_like_workers);
	std::vector<std::thread> workers;
	workers.reserve(worker_count);
	for (std::size_t worker = 0; worker < worker_count; ++worker) {
		workers.emplace_back([&] {
			for (;;) {
				const auto index = next.fetch_add(1, std::memory_order_relaxed);
				if (index >= model_ids.size())
					return;
				try {
					const auto id = model_ids[index];
					auto info = provider.info(id);
					if (!info)
						continue;
					auto asset = provider.fetch(std::to_string(id));
					if (asset)
						fetched[index].emplace(
								FetchedModel{std::move(*info), std::move(*asset)});
				} catch (...) {
					// Match Rust's per-ID fetch failure behavior: skip this model while
					// allowing other independent provider requests to complete.
				}
			}
		});
	}
	for (auto &worker : workers)
		worker.join();
	for (std::size_t i = 0; i < fetched.size(); ++i) {
		if (!fetched[i])
			continue;
		infos.emplace(model_ids[i], std::move(fetched[i]->info));
		assets.emplace(model_ids[i], std::move(fetched[i]->asset));
	}

	for (std::size_t placement_index = 0; placement_index < placement_count;
			++placement_index) {
		const auto &placement = prescan.placements.at(
				indices ? (*indices)[placement_index] : placement_index);
		++stats.attempted;
		try {
			const auto asset = assets.find(placement.model_id);
			const auto info = infos.find(placement.model_id);
			if (asset == assets.end() || info == infos.end())
				continue;
			const int ground = ground_for(editor, placement.footprint.min_x,
					placement.footprint.min_z, placement.footprint.max_x,
					placement.footprint.max_z);
			const auto *correction = model_override(placement.model_id);
			const double model_scale =
					info->second.scale * (correction ? correction->scale : 1.0);
			const double extra_yaw = correction ? correction->yaw_degrees : 0.0;
			WorldTransform transform(info->second.rotation, model_scale,
					info->second.translation,
					{float(blocks_per_meter), float(blocks_per_meter),
							float(blocks_per_meter)},
					placement.world_yaw_degrees + extra_yaw, float(placement.anchor_x),
					float(ground), float(placement.anchor_z));
			if (correction && !correction->palette.empty() &&
					asset->second.format == ModelFormat::GLB) {
				place_grounded_voxels(editor,
						voxelize_glb_with_palette(
								asset->second.bytes, transform, correction->palette),
						ground, 0, stats);
			} else {
				place_grounded(editor, asset->second, transform, ground, 0, stats);
			}
		} catch (const std::bad_alloc &) {
			throw;
		} catch (const std::exception &error) {
			std::cerr << "Warning: 3DMR model " << placement.model_id << " (OSM "
					  << placement.osm_id << ") voxelization failed: " << error.what()
					  << '\n';
			continue;
		}
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
		world_editor::WorldEditor &editor, const custom::stadium::PrescanResult &prescan,
		double blocks_per_meter, std::optional<std::span<const std::size_t>> indices)
{
	ModelPlacementStats stats;
	const std::size_t placement_count =
			indices ? indices->size() : prescan.placements.size();
	if (!std::isfinite(blocks_per_meter) || blocks_per_meter <= 0 || placement_count == 0)
		return stats;
	auto asset = provider.fetch(custom::stadium_model_key());
	if (!asset)
		return stats;
	const float model_x = asset->max[0] - asset->min[0];
	const float model_y = asset->max[1] - asset->min[1];
	const float model_z = asset->max[2] - asset->min[2];
	if (!std::isfinite(model_x) || !std::isfinite(model_y) || !std::isfinite(model_z) ||
			model_x < 1e-3f || model_y < 1e-3f || model_z < 1e-3f)
		return stats;
	const bool model_x_is_long = model_x >= model_z;
	const float model_long = std::max(model_x, model_z);
	const float model_short = std::min(model_x, model_z);
	const float center_x = -(asset->min[0] + asset->max[0]) * 0.5f;
	const float center_z = -(asset->min[2] + asset->max[2]) * 0.5f;
	const float block_per_meter = static_cast<float>(blocks_per_meter);
	for (std::size_t placement_index = 0; placement_index < placement_count;
			++placement_index) {
		const auto &placement = prescan.placements.at(
				indices ? (*indices)[placement_index] : placement_index);
		++stats.attempted;
		const float target_long = placement.long_m * block_per_meter;
		const float target_short = placement.short_m * block_per_meter;
		const float target_height =
				custom::stadium_target_height_m(
						placement.osm_height_m, placement.has_osm_height) *
				block_per_meter;
		const float scale_x = target_long / model_long;
		const float scale_y = target_height / model_y;
		const float scale_z = target_short / model_short;
		if (!std::isfinite(scale_x) || !std::isfinite(scale_y) ||
				!std::isfinite(scale_z) || scale_x <= 0 || scale_y <= 0 || scale_z <= 0)
			continue;
		const int ground =
				ground_for(editor, placement.footprint.min_x, placement.footprint.min_z,
						placement.footprint.max_x, placement.footprint.max_z);
		const double intrinsic_yaw = model_x_is_long ? 0.0 : 90.0;
		const double intrinsic_tx = model_x_is_long ? center_x : -center_z;
		const double intrinsic_tz = model_x_is_long ? center_z : center_x;
		const auto transform = WorldTransform::with_world_scale_xyz(intrinsic_yaw, 1.0,
				{intrinsic_tx, 0.0, intrinsic_tz}, {scale_x, scale_y, scale_z},
				placement.yaw_degrees, float(placement.anchor_x), float(ground),
				float(placement.anchor_z));
		try {
			place_grounded(editor, *asset, transform, ground, 0, stats);
		} catch (const std::bad_alloc &) {
			throw;
		} catch (const std::exception &e) {
			std::cerr << "Warning: stadium (OSM " << placement.osm_id
					  << ") voxelization failed: " << e.what() << '\n';
		} catch (...) {
			std::cerr << "Warning: stadium (OSM " << placement.osm_id
					  << ") voxelization failed with an unknown error\n";
		}
	}
	return stats;
}
}
