#include "plantations.h"

#include "area_layout.h"
#include "block_definitions.h"
#include "connected_blocks.h"
#include "world_editor.h"
#include "../ground_generation.h"
#include "../land_cover/land_cover.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace arnis::plantations
{
namespace area_layout = arnis::element_processing::area_layout;
namespace
{
constexpr int VINE_PITCH = 3;
constexpr int VINE_RUN = 36;

int euclid_mod(int value, int divisor)
{
	const int r = value % divisor;
	return r < 0 ? r + divisor : r;
}

bool protected_ground(WorldEditor &editor, int x, int z)
{
	static const std::vector<Block> protected_blocks{STONE, COBBLESTONE, STONE_BRICKS,
			BRICK, SANDSTONE, OAK_PLANKS, SPONGE, WATER};
	return editor.surface_is_sealed(x, z) ||
		   editor.check_for_block(
				   x, 0, z, std::optional<std::vector<Block>>{protected_blocks});
}

bool near_road(const RoadMaskBitmap &roads, int x, int z)
{
	return roads.contains(x, z) || roads.contains(x + 1, z) || roads.contains(x - 1, z) ||
		   roads.contains(x, z + 1) || roads.contains(x, z - 1);
}

std::uint64_t hash_cell(std::uint64_t id, std::int64_t a, std::int64_t b)
{
	return area_layout::cell_hash(id, a, b);
}

struct FruitTree
{
	int trunk = 2;
	double radius = 2.0;
	Block log = OAK_LOG;
	Block leaves = OAK_LEAVES;
	Block accent = AIR;
	bool has_accent = false;

	bool place(WorldEditor &editor, int tx, int tz, int x, int z) const
	{
		const double horizontal = std::hypot(double(x - tx), double(z - tz));
		if (horizontal > radius + 0.5)
			return false;
		const int base = editor.get_absolute_y(tx, 0, tz);
		bool placed = false;
		if (x == tx && z == tz) {
			for (int y = 1; y <= trunk; ++y) {
				editor.set_block_absolute(
						log, x, base + y, z, std::nullopt, std::nullopt);
				placed = true;
			}
		}
		const double crown_y = trunk + radius * .8;
		const int top = static_cast<int>(std::ceil(crown_y + radius * .8));
		for (int y = std::max(trunk + 1, 1); y <= top; ++y) {
			const double dy = (y - crown_y) / .8;
			const auto rough_hash = land_cover::coord_hash(x * 7 + y, z);
			const double rough = (rough_hash % 100) / 250.0;
			if (horizontal * horizontal + dy * dy > std::pow(radius + .3 - rough, 2))
				continue;
			const bool use_accent =
					has_accent && land_cover::coord_hash(x ^ y, z ^ 0x0A99) % 100 < 12;
			editor.set_block_absolute(use_accent ? accent : leaves, x, base + y, z,
					std::nullopt, std::nullopt);
			placed = true;
		}
		return placed;
	}
};

struct OrchardStyle
{
	double along = 5.0, across = 6.0, jitter = .4;
	int planted = 92;
	FruitTree tree{};
	bool strip = false;
};

OrchardStyle style_for(const ProcessedWay &way, double scale)
{
	const auto tag = [&](const char *key) -> std::string {
		auto it = way.tags.find(key);
		return it == way.tags.end() ? std::string{} : it->second;
	};
	const auto trees = tag("trees");
	const bool meadow = tag("orchard") == "meadow_orchard";
	const bool plantation = tag("orchard") == "plantation";
	OrchardStyle s;
	if (meadow) {
		s = {11.0, 11.0, 2.0, 80,
				{3, 3.2, OAK_LOG, OAK_LEAVES, FLOWERING_AZALEA_LEAVES, true}, false};
	} else if (trees == "olive_trees") {
		s = {7, 8, .6, 95, {2, 2.6, OAK_LOG, AZALEA_LEAVES, AIR, false}, false};
	} else if (trees == "orange_trees" || trees == "citrus_trees" ||
			   trees == "lemon_trees") {
		s = {5, 6, 0, 96, {1, 2.1, OAK_LOG, JUNGLE_LEAVES, AIR, false}, false};
	} else if (trees == "oil_palms" || trees == "coconut_palms" ||
			   trees == "date_palms" || trees == "palm_trees") {
		s = {9, 8, 0, 95, {4, 3.2, JUNGLE_LOG, JUNGLE_LEAVES, AIR, false}, false};
	} else if (trees == "walnut_trees" || trees == "almond_trees" ||
			   trees == "chestnut_trees" || trees == "hazel_plants" ||
			   trees == "nut_trees") {
		s = {6, 7, .3, 95, {2, 2.6, OAK_LOG, OAK_LEAVES, AIR, false}, false};
	} else if (trees == "cherry_trees") {
		s = {4, 6, 0, 96, {1, 1.8, OAK_LOG, OAK_LEAVES, CHERRY_LEAVES, true}, true};
	} else if (trees == "apple_trees" || trees == "pear_trees" || trees == "plum_trees" ||
			   trees == "fruit_trees" || plantation) {
		s = {3, 5, 0, 97, {1, 1.5, OAK_LOG, OAK_LEAVES, FLOWERING_AZALEA_LEAVES, true},
				true};
	} else {
		s.tree = {2, 2.0, OAK_LOG, OAK_LEAVES, FLOWERING_AZALEA_LEAVES, true};
	}
	const double factor = std::clamp(scale, .25, 4.0);
	s.along = std::max(2.0, s.along * factor);
	s.across = std::max(3.0, s.across * factor);
	s.tree.radius = std::max(1.0, s.tree.radius * factor);
	s.tree.trunk = std::max(1, static_cast<int>(std::round(s.tree.trunk * factor)));
	return s;
}

std::pair<int, int> row_axis(const WorldEditor &editor, const ProcessedWay &way,
		const area_layout::Frame &frame)
{
	if (way.nodes.size() >= 4) {
		double mx = 0, mz = 0, my = 0;
		for (const auto &n : way.nodes) {
			mx += n.x;
			mz += n.z;
			my += editor.get_ground_level(n.x, n.z);
		}
		const double k = way.nodes.size();
		mx /= k;
		mz /= k;
		my /= k;
		double sxx = 0, szz = 0, sxz = 0, sxy = 0, szy = 0;
		for (const auto &n : way.nodes) {
			const double x = n.x - mx, z = n.z - mz,
						 y = editor.get_ground_level(n.x, n.z) - my;
			sxx += x * x;
			szz += z * z;
			sxz += x * z;
			sxy += x * y;
			szy += z * y;
		}
		const double det = sxx * szz - sxz * sxz;
		if (std::abs(det) > 1e-6) {
			const double gx = (sxy * szz - szy * sxz) / det,
						 gz = (szy * sxx - sxy * sxz) / det;
			if (std::hypot(gx, gz) > .08)
				return std::abs(gx) >= std::abs(gz) ? std::pair{1, 0} : std::pair{0, 1};
		}
	}
	return frame.along_axis();
}
} // namespace

void vineyard_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes)
{
	auto frame = area_layout::Frame::of(way.nodes);
	if (!frame)
		return;
	const auto axis = row_axis(editor, way, *frame);
	const bool along_x = axis.first != 0;
	const auto h = area_layout::mix(way.id ^ 0x007111E5ULL);
	const int phase = h % VINE_PITCH, lane_phase = (h >> 8) % VINE_RUN;
	const bool green = (h >> 16) % 3 != 0;
	const Block canopy = (h >> 20) % 4 == 0 ? JUNGLE_LEAVES : OAK_LEAVES;
	area_layout::AreaMask mask(fill, [&](int x, int z) { return !in_hole(holes, x, z); });
	auto vine = [&](int x, int z) {
		const int along = along_x ? x : z, across = along_x ? z : x;
		return mask.contains(x, z) && euclid_mod(across + phase, VINE_PITCH) == 0 &&
			   euclid_mod(along + lane_phase, VINE_RUN) >= 2 &&
			   !footprints.contains(x, z) && !near_road(roads, x, z) &&
			   !editor.surface_is_sealed(x, z) && !editor.is_lc_water(x, z);
	};
	for (const auto &[x, z] : fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				protected_ground(editor, x, z))
			continue;
		const bool row = euclid_mod((along_x ? z : x) + phase, VINE_PITCH) == 0;
		const double n = ground_generation::value_noise_01(x + 59, z - 13, 5);
		const Block ground = (row || (!green && n > .25) || (green && n > .85))
									 ? COARSE_DIRT
									 : GRASS_BLOCK;
		editor.set_block(ground, x, 0, z, std::nullopt, std::nullopt);
		if (!editor.owns(x, z) || editor.is_lc_water(x, z))
			continue;
		if (!vine(x, z)) {
			if (ground == GRASS_BLOCK && land_cover::coord_hash(x, z ^ 0x51) % 5 == 0)
				editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
			continue;
		}
		const int gy = editor.get_ground_level(x, z);
		auto wired = [&](int nx, int nz) {
			return vine(nx, nz) && editor.get_ground_level(nx, nz) == gy;
		};
		const bool fwd = along_x ? wired(x + 1, z) : wired(x, z + 1),
				   back = along_x ? wired(x - 1, z) : wired(x, z - 1);
		const auto wire = along_x ? connected_blocks::connected_fence(
											OAK_FENCE, false, false, fwd, back)
								  : connected_blocks::connected_fence(
											OAK_FENCE, back, fwd, false, false);
		const int y = editor.get_absolute_y(x, 1, z);
		if (!editor.block_exists_absolute(x, y, z))
			editor.set_block_with_properties_absolute(
					wire, x, y, z, std::nullopt, std::nullopt);
		const int along = along_x ? x : z;
		editor.set_block(euclid_mod(along + lane_phase, 8) == 4 ? OAK_FENCE : canopy, x,
				2, z, std::nullopt, std::nullopt);
	}
}

void orchard_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes)
{
	auto frame = area_layout::Frame::of(way.nodes);
	if (!frame)
		return;
	auto style = style_for(way, editor.scale());
	area_layout::AreaMask mask(fill, [&](int x, int z) { return !in_hole(holes, x, z); });
	const double reach = std::ceil(style.tree.radius) + std::ceil(style.jitter) + 1;
	for (const auto &[x, z] : fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				protected_ground(editor, x, z))
			continue;
		const auto [a, c] = frame->local(x, z);
		const double r = std::floor(c / style.across);
		const bool strip = style.strip && std::abs(c - (r + .5) * style.across) < .75;
		const Block ground = strip ? COARSE_DIRT : GRASS_BLOCK;
		editor.set_block(ground, x, 0, z, std::nullopt, std::nullopt);
		if (!editor.owns(x, z) || editor.is_lc_water(x, z) || footprints.contains(x, z))
			continue;
		const auto s0 = static_cast<std::int64_t>(std::floor((a - reach) / style.along));
		const auto s1 = static_cast<std::int64_t>(std::floor((a + reach) / style.along));
		const auto r0 = static_cast<std::int64_t>(std::floor((c - reach) / style.across));
		const auto r1 = static_cast<std::int64_t>(std::floor((c + reach) / style.across));
		std::vector<std::pair<int, int>> trunks;
		for (auto rr = r0; rr <= r1; ++rr)
			for (auto ss = s0; ss <= s1; ++ss) {
				const auto h = hash_cell(way.id, ss, rr);
				if (h % 100 >= static_cast<unsigned>(style.planted))
					continue;
				const double ja = style.jitter > 0 ? double((h >> 8) % 1000) / 1000 * 2 *
																	 style.jitter -
															 style.jitter
												   : 0;
				const double jc = style.jitter > 0 ? double((h >> 20) % 1000) / 1000 * 2 *
																	 style.jitter -
															 style.jitter
												   : 0;
				const auto [tx, tz] = frame->column(
						(ss + .5) * style.along + ja, (rr + .5) * style.across + jc);
				if (!mask.contains(tx, tz) || roads.contains(tx, tz) ||
						footprints.contains(tx, tz) || editor.surface_is_sealed(tx, tz) ||
						editor.is_lc_water(tx, tz))
					continue;
				trunks.emplace_back(tx, tz);
			}
		std::stable_sort(
				trunks.begin(), trunks.end(), [x, z](const auto &a, const auto &b) {
					return (a.first == x && a.second == z) >
						   (b.first == x && b.second == z);
				});
		bool any = false;
		for (const auto &[tx, tz] : trunks)
			any |= style.tree.place(editor, tx, tz, x, z);
		if (!any && ground == GRASS_BLOCK && land_cover::coord_hash(x ^ 0x0C, z) % 6 == 0)
			editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
	}
}
} // namespace arnis::plantations
