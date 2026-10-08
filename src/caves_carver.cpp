#include "caves_carver.h"
#include "caves_density.h"
#include "caves_theme.h"
#include "caves_water.h"
#include "caves_rng.h"
#include "args.h"
#include "block_definitions.h"
#include "world_editor/floor_state.h"
#include "../../arnis_world_editor.h"
#include <array>
#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arnis::caves
{
namespace
{
constexpr int CAVE_REACH_CHUNKS = 8;

std::uint64_t decor_hash(int x, int y, int z, std::uint64_t seed)
{
	std::uint64_t h = seed ^ (std::uint64_t(std::int64_t(x)) * 0x9E3779B97F4A7C15ULL) ^
					  (std::uint64_t(std::int64_t(y)) * 0xC2B2AE3D27D4EB4FULL) ^
					  (std::uint64_t(std::int64_t(z)) * 0x165667B19E3779F9ULL);
	h ^= h >> 30;
	h *= 0xBF58476D1CE4E5B9ULL;
	h ^= h >> 27;
	return h;
}

bool decor_chance(int x, int y, int z, std::uint64_t seed, std::uint64_t per_thousand)
{
	return decor_hash(x, y, z, seed) % 1000 < per_thousand;
}

void grow_dripstone(world_editor::WorldEditor &editor, int x, int y, int z,
		std::uint64_t seed, bool up)
{
	const int length = 1 + static_cast<int>(decor_hash(x, y, z, seed ^ 0xD21AULL) % 5);
	const int step = up ? 1 : -1;
	const char *direction = up ? "up" : "down";
	for (int i = 0; i < length; ++i) {
		const int yy = y + step * i;
		if (editor.block_exists_absolute(x, yy, z))
			break;
		const char *thickness = length == 1		  ? "tip"
								: i == 0		  ? "base"
								: i == length - 1 ? "tip"
								: i == 1		  ? "frustum"
												  : "middle";
		BlockWithProperties pointed{POINTED_DRIPSTONE,
				{{"thickness", thickness}, {"vertical_direction", direction},
						{"waterlogged", "false"}}};
		editor.set_block_with_properties_absolute(
				pointed, x, yy, z, std::vector<Block>{AIR}, std::nullopt);
	}
}

void hang_cave_vines(world_editor::WorldEditor &editor,
		const std::unordered_set<std::int64_t> &air, int x, int y, int z,
		std::uint64_t seed)
{
	const int length = 1 + static_cast<int>(decor_hash(x, y, z, seed ^ 0x617EULL) % 8);
	for (int i = 0; i < length; ++i) {
		const int yy = y - i;
		if (!air.contains(pack_cave_pos(x, yy, z)) &&
				editor.block_exists_absolute(x, yy, z))
			break;
		const bool berries = decor_hash(x, yy, z, seed ^ 0xBEEFULL) % 5 == 0;
		const auto props =
				i == length - 1
						? std::unordered_map<std::string,
								  std::string>{{"berries", berries ? "true" : "false"},
								  {"age", "25"}}
						: std::unordered_map<std::string, std::string>{
								  {"berries", berries ? "true" : "false"}};
		const Block vine_block =
				i == length - 1 ? (berries ? CAVE_VINES : CAVE_VINES_UNLIT)
								: (berries ? CAVE_VINES_PLANT_LIT : CAVE_VINES_PLANT);
		const BlockWithProperties vine{vine_block, props};
		editor.set_block_with_properties_absolute(
				vine, x, yy, z, std::vector<Block>{AIR}, std::nullopt);
	}
}

BlockWithProperties multiface_block(Block block, const char *face)
{
	std::unordered_map<std::string, std::string> properties;
	for (const char *name : {"down", "up", "north", "south", "east", "west"})
		properties.emplace(name, std::string(name) == face ? "true" : "false");
	return {block, std::move(properties)};
}

void build_giant_mushroom(world_editor::WorldEditor &editor, const CaveRect &region,
		const std::unordered_set<std::int64_t> &air, int x, int y, int z,
		std::uint64_t seed)
{
	const int stem_height =
			3 + static_cast<int>(decor_hash(x, y, z, seed ^ 0x9145ULL) % 4);
	for (int i = 0; i <= stem_height; ++i)
		if (!air.contains(pack_cave_pos(x, y + i, z)) &&
				editor.block_exists_absolute(x, y + i, z))
			return;
	const bool red = decor_hash(x, y, z, seed ^ 0x9146ULL) % 2 == 0;
	const auto air_variants = std::vector<Block>{AIR};
	for (int i = 0; i < stem_height; ++i)
		editor.set_block_absolute(MUSHROOM_STEM, x, y + i, z, air_variants, std::nullopt);
	const int cap_y = y + stem_height;
	auto place_cap = [&](Block block, int px, int py, int pz) {
		if (region.contains(px, pz))
			editor.set_block_absolute(block, px, py, pz, air_variants, std::nullopt);
	};
	if (red) {
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				place_cap(RED_MUSHROOM_BLOCK, x + dx, cap_y, z + dz);
		for (int dx = -2; dx <= 2; ++dx)
			for (int dz = -2; dz <= 2; ++dz)
				if ((std::abs(dx) == 2 || std::abs(dz) == 2) &&
						!(std::abs(dx) == 2 && std::abs(dz) == 2))
					place_cap(RED_MUSHROOM_BLOCK, x + dx, cap_y - 1, z + dz);
	} else {
		for (int dx = -2; dx <= 2; ++dx)
			for (int dz = -2; dz <= 2; ++dz)
				if (!(std::abs(dx) == 2 && std::abs(dz) == 2))
					place_cap(BROWN_MUSHROOM_BLOCK, x + dx, cap_y, z + dz);
		if (decor_hash(x, cap_y, z, seed ^ 0x9147ULL) % 4 == 0)
			editor.set_block_absolute(
					SHROOMLIGHT, x, cap_y - 1, z, air_variants, std::nullopt);
	}
}

void place_clay_pool(world_editor::WorldEditor &editor, int x, int y, int z,
		const std::optional<std::vector<Block>> &rock_options,
		const std::unordered_set<std::int64_t> &air)
{
	for (int dx = -2; dx <= 2; ++dx)
		for (int dz = -2; dz <= 2; ++dz)
			if (!air.contains(pack_cave_pos(x + dx, y, z + dz)) ||
					!editor.check_for_block_absolute(x + dx, y - 1, z + dz, rock_options))
				return;
	for (int dx = -2; dx <= 2; ++dx)
		for (int dz = -2; dz <= 2; ++dz) {
			if (std::abs(dx) == 2 || std::abs(dz) == 2) {
				editor.set_block_absolute(
						CLAY, x + dx, y - 1, z + dz, rock_options, std::nullopt);
			} else {
				editor.set_block_absolute(
						WATER, x + dx, y - 1, z + dz, rock_options, std::nullopt);
				editor.set_block_absolute(
						CLAY, x + dx, y - 2, z + dz, rock_options, std::nullopt);
			}
		}
}

void carve_ellipsoid(world_editor::WorldEditor &editor, const CaveRect &region, double ox,
		double oy, double oz, double radius, double vertical, double floor_level,
		int floor_y, CaveEllipsoids *ellipsoids, std::unordered_set<std::int64_t> *carved)
{
	if (ellipsoids)
		ellipsoids->push_back({ox, oy, oz, radius, vertical, floor_level});
	const int min_x = std::max(region.min_x, int(std::floor(ox - radius)));
	const int max_x = std::min(region.max_x, int(std::floor(ox + radius)));
	const int min_z = std::max(region.min_z, int(std::floor(oz - radius)));
	const int max_z = std::min(region.max_z, int(std::floor(oz + radius)));
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_y =
			std::max({int(std::floor(oy - vertical)), floor_y + 1, write_min_y});
	const int max_y = std::min(int(std::floor(oy + vertical)) + 1, write_max_y);
	if (min_y > max_y || min_x > max_x || min_z > max_z)
		return;
	const std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, DIRT, GRAVEL};
	const auto host_options = std::optional<std::vector<Block>>(hosts);
	for (int x = min_x; x <= max_x; ++x) {
		const double dx = (double(x) + .5 - ox) / radius;
		if (dx * dx >= 1.0)
			continue;
		for (int z = min_z; z <= max_z; ++z) {
			const double dz = (double(z) + .5 - oz) / radius;
			if (dx * dx + dz * dz >= 1.0)
				continue;
			const int roof = editor.get_ground_level(x, z) - 6;
			for (int y = min_y; y <= max_y; ++y) {
				if (y > roof)
					continue;
				const double dy = (double(y) - .5 - oy) / vertical;
				if (dy > floor_level && dx * dx + dy * dy + dz * dz < 1.0) {
					editor.set_block_absolute(AIR, x, y, z, host_options, std::nullopt);
					if (carved)
						carved->insert(pack_cave_pos(x, y, z));
				}
			}
		}
	}
}

constexpr int BRANCH_BUDGET = 112;

bool can_reach(
		int cx, int cz, double x, double z, int step, int branch_count, double width)
{
	const double dx = x - (cx * 16 + 8), dz = z - (cz * 16 + 8);
	const double remaining = branch_count - step;
	const double reach = width + 18.0;
	return dx * dx + dz * dz - remaining * remaining <= reach * reach;
}

double get_thickness(XoroRandom &r)
{
	double f = double(r.next_float()) * 2.0 + double(r.next_float());
	if (r.next_int(10) == 0)
		f *= double(r.next_float()) * double(r.next_float()) * 3.0 + 1.0;
	return std::min(f, 3.0);
}

void create_tunnel(std::int64_t tunnel_seed, double x, double y, double z, double h_mult,
		double v_mult, double thickness, double yaw, double pitch, int branch_index,
		int branch_count, double horizontal_vertical_ratio, double floor_level, int cx,
		int cz, int floor_y, world_editor::WorldEditor &editor, const CaveRect &region,
		CaveEllipsoids *ellipsoids, std::unordered_set<std::int64_t> *carved)
{
	XoroRandom r = XoroRandom::from_seed(tunnel_seed);
	const int branch_point = r.next_int(branch_count / 2) + branch_count / 4;
	const bool steep = r.next_int(6) == 0;
	double yaw_delta = 0.0, pitch_delta = 0.0;
	for (int step = branch_index; step < branch_count; ++step) {
		const double d = 1.5 + std::sin(M_PI * step / branch_count) * thickness;
		const double d1 = d * horizontal_vertical_ratio;
		const double cos_pitch = std::cos(pitch);
		x += std::cos(yaw) * cos_pitch;
		y += std::sin(pitch);
		z += std::sin(yaw) * cos_pitch;
		pitch *= steep ? .92 : .7;
		pitch += pitch_delta * .1;
		yaw += yaw_delta * .1;
		pitch_delta *= .9;
		yaw_delta *= .75;
		pitch_delta += (double(r.next_float()) - double(r.next_float())) *
					   double(r.next_float()) * 2.0;
		yaw_delta += (double(r.next_float()) - double(r.next_float())) *
					 double(r.next_float()) * 4.0;
		if (step == branch_point && thickness > 1.0) {
			const double t1 = double(r.next_float()) * .5 + .5;
			const auto s1 = r.next_long();
			const double t2 = double(r.next_float()) * .5 + .5;
			const auto s2 = r.next_long();
			create_tunnel(s1, x, y, z, h_mult, v_mult, t1, yaw - M_PI / 2.0, pitch / 3.0,
					step, branch_count, 1.0, floor_level, cx, cz, floor_y, editor, region,
					ellipsoids, carved);
			create_tunnel(s2, x, y, z, h_mult, v_mult, t2, yaw + M_PI / 2.0, pitch / 3.0,
					step, branch_count, 1.0, floor_level, cx, cz, floor_y, editor, region,
					ellipsoids, carved);
			return;
		}
		if (r.next_int(4) != 0) {
			if (!can_reach(cx, cz, x, z, step, branch_count, thickness))
				return;
			carve_ellipsoid(editor, region, x, y, z, d * h_mult, d1 * v_mult, floor_level,
					floor_y, ellipsoids, carved);
		}
	}
}

void carve_random_walk_carvers(world_editor::WorldEditor &editor, const CaveRect &region,
		const CaveRect &origin_region, std::int64_t seed, int floor_y,
		CaveEllipsoids *ellipsoids, std::unordered_set<std::int64_t> *carved)
{
	const auto chunk_floor = [](int value) {
		return value >= 0 ? value / 16 : -(((-value) + 15) / 16);
	};
	const int shift = floor_y - VANILLA_FLOOR;
	for (int cx = chunk_floor(origin_region.min_x) - CAVE_REACH_CHUNKS;
			cx <= chunk_floor(origin_region.max_x) + CAVE_REACH_CHUNKS; ++cx) {
		for (int cz = chunk_floor(origin_region.min_z) - CAVE_REACH_CHUNKS;
				cz <= chunk_floor(origin_region.max_z) + CAVE_REACH_CHUNKS; ++cz) {
			for (const auto &cfg : {std::tuple<std::int64_t, float, int, int>{
											0x0CA50CA5LL, .075f, -56, 180},
						 {0xE547E547LL, .025f, -56, 47}}) {
				const auto [salt, probability, y_min, y_max] = cfg;
				XoroRandom r =
						XoroRandom::from_seed(seed ^ std::int64_t(cx) * 341873128712LL ^
											  std::int64_t(cz) * 132897987541LL ^ salt);
				if (r.next_float() > probability)
					continue;
				const int a = r.next_int(15) + 1;
				const int b = r.next_int(a) + 1;
				const int count = r.next_int(b);
				for (int i = 0; i < count; ++i) {
					const int ox = cx * 16 + r.next_int(16);
					const int oy = y_min + r.next_int(y_max - y_min + 1);
					const int oz = cz * 16 + r.next_int(16);
					const double h_mult = .7 + double(r.next_float()) * .7;
					const double v_mult = .8 + double(r.next_float()) * .5;
					const double floor_level = -1.0 + double(r.next_float()) * .6;
					int tunnels = 1;
					if (r.next_int(4) == 0) {
						const double y_scale = .1 + double(r.next_float()) * .8;
						const double room_radius = 1.0 + double(r.next_float()) * 2.0;
						const double d = 1.5 + room_radius;
						carve_ellipsoid(editor, region, ox + 1.0, oy + shift, oz, d,
								d * y_scale, floor_level, floor_y, ellipsoids, carved);
						tunnels += r.next_int(4);
					}
					for (int tunnel = 0; tunnel < tunnels; ++tunnel) {
						const double yaw = double(r.next_float()) * 2.0 * M_PI;
						const double pitch = (double(r.next_float()) - .5) / 4.0;
						const double thickness = get_thickness(r);
						const int branch_count =
								BRANCH_BUDGET - r.next_int(BRANCH_BUDGET / 4);
						const auto tunnel_seed = r.next_long();
						create_tunnel(tunnel_seed, ox, oy + shift, oz, h_mult, v_mult,
								thickness, yaw, pitch, 0, branch_count, 1.0, floor_level,
								cx, cz, floor_y, editor, region, ellipsoids, carved);
					}
				}
			}

			// Vanilla's canyon carver uses a separate origin salt and walk shape.
			XoroRandom r = XoroRandom::from_seed(
					seed ^ std::int64_t(cx) * 341873128712LL ^
					std::int64_t(cz) * 132897987541LL ^ 0x4A1E4A1ELL);
			if (r.next_float() <= .008f) {
				double x = cx * 16 + r.next_int(16);
				double y = -54 + r.next_int(64) + shift;
				double z = cz * 16 + r.next_int(16);
				const double h_mult = .7 + double(r.next_float()) * .7;
				const double v_mult = .8 + double(r.next_float()) * .5;
				const double thickness =
						std::min(2.0 * (double(r.next_float()) * 2.0 +
											   double(r.next_float()) + 1.0),
								5.0);
				double yaw = double(r.next_float()) * 2.0 * M_PI;
				double pitch = (double(r.next_float()) - .5) / 8.0;
				const int branch_count = BRANCH_BUDGET - r.next_int(BRANCH_BUDGET / 4);
				XoroRandom turn = XoroRandom::from_seed(r.next_long());
				std::array<double, BRANCH_BUDGET> widths{};
				double width = 1.0;
				for (int i = 0; i < branch_count; ++i) {
					if (i == 0 || turn.next_int(3) == 0)
						width = 1.0 +
								double(turn.next_float()) * double(turn.next_float());
					widths[i] = std::min(width * width, 1.8);
				}
				const double floor_level = -1.0 + double(r.next_float()) * .6;
				double yaw_delta = 0.0, pitch_delta = 0.0;
				for (int step = 0; step < branch_count; ++step) {
					const double half =
							1.5 + std::sin(M_PI * step / branch_count) * thickness;
					const double cos_pitch = std::cos(pitch);
					x += std::cos(yaw) * cos_pitch;
					y += std::sin(pitch);
					z += std::sin(yaw) * cos_pitch;
					pitch = pitch * .7 + pitch_delta * .05;
					yaw += yaw_delta * .05;
					pitch_delta *= .8;
					yaw_delta *= .5;
					pitch_delta +=
							(double(turn.next_float()) - double(turn.next_float())) *
							double(turn.next_float()) * 2.0;
					yaw_delta += (double(turn.next_float()) - double(turn.next_float())) *
								 double(turn.next_float()) * 4.0;
					if (turn.next_int(4) != 0) {
						if (!can_reach(cx, cz, x, z, step, branch_count, thickness))
							break;
						carve_ellipsoid(editor, region, x, y, z,
								half * h_mult * std::sqrt(widths[step]),
								half * v_mult * 2.2, floor_level, floor_y, ellipsoids,
								carved);
					}
				}
			}
		}
	}
}

void place_geodes(world_editor::WorldEditor &editor, const CaveRect &region, int floor_y,
		const CaveRect &world_bounds, std::uint64_t seed,
		const CaveShapeQuery &cave_shape, const WaterPlan &water_plan,
		const std::unordered_set<std::int64_t> *cave_air)
{
	constexpr double inner_radius = 4.2;
	constexpr double amethyst_radius = 5.0;
	constexpr double calcite_radius = 6.0;
	constexpr double basalt_radius = 6.7;
	const int radius = static_cast<int>(std::ceil(basalt_radius));
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	std::unordered_map<std::int64_t, bool> overlay;
	auto opened = [&](int x, int y, int z) {
		const auto p = pack_cave_pos(x, y, z);
		if (const auto it = overlay.find(p); it != overlay.end())
			return !it->second;
		return (cave_air && cave_air->contains(p)) || water_plan.carved.contains(p) ||
			   water_plan.water.contains(p) || cave_shape.is_cave(x, y, z);
	};
	auto standing = [&](int x, int y, int z) {
		const auto p = pack_cave_pos(x, y, z);
		if (const auto it = overlay.find(p); it != overlay.end())
			return it->second;
		if (water_plan.water.contains(p))
			return true;
		if (cave_air && cave_air->contains(p))
			return false;
		return water_plan.solid(cave_shape, x, y, z);
	};
	const auto expanded = region.grow(radius + 1);
	const auto [cx0, cx1, cz0, cz1] = expanded.chunks();
	const std::vector<Block> bedrock{BEDROCK};
	// Decide every center before writing any geode. Rust tests a candidate
	// against CaveShape/WaterPlan, not against shells or hollows from geodes
	// encountered earlier in this region; the shared shape and water plan keep
	// the result independent of which tile happened to write first.
	std::vector<std::tuple<int, int, int>> centers;
	for (int cx = cx0; cx <= cx1; ++cx)
		for (int cz = cz0; cz <= cz1; ++cz) {
			if (decor_hash(cx, 0, cz, seed ^ 0x006E0DE5ULL) % 80 != 0)
				continue;
			const int gx = cx * 16 + static_cast<int>(decor_hash(cx, 1, cz, seed) % 16);
			const int gz = cz * 16 + static_cast<int>(decor_hash(cx, 2, cz, seed) % 16);
			const int gy =
					floor_y + 10 + static_cast<int>(decor_hash(cx, 3, cz, seed) % 36);
			if (!world_bounds.contains(gx, gz) ||
					gy + radius + 2 >= editor.get_ground_level(gx, gz) ||
					!standing(gx, gy, gz))
				continue;
			centers.emplace_back(gx, gy, gz);
		}
	std::vector<std::tuple<int, int, int, int, int, int>> clusters;
	for (const auto &[gx, gy, gz] : centers) {

		std::set<std::tuple<int, int, int>> hollow;
		std::vector<std::pair<std::tuple<int, int, int>, Block>> shell;
		std::unordered_set<std::int64_t> written_shell;
		std::vector<std::tuple<int, int, int>> buddings;
		for (int dx = -radius; dx <= radius; ++dx)
			for (int dy = -radius; dy <= radius; ++dy)
				for (int dz = -radius; dz <= radius; ++dz) {
					const double distance =
							std::sqrt(double(dx * dx + dy * dy + dz * dz));
					if (distance > basalt_radius)
						continue;
					const int x = gx + dx, y = gy + dy, z = gz + dz;
					const auto pos = std::tuple{x, y, z};
					const auto packed = pack_cave_pos(x, y, z);
					if (distance <= inner_radius) {
						hollow.insert(pos);
						continue;
					}
					if (opened(x, y, z))
						continue;
					Block block;
					if (distance <= amethyst_radius) {
						if (distance <= inner_radius + 1.0 &&
								decor_hash(x, y, z, seed ^ 0xB0DDULL) % 100 < 30) {
							buddings.push_back(pos);
							block = BUDDING_AMETHYST;
						} else {
							block = AMETHYST_BLOCK;
						}
					} else if (distance <= calcite_radius) {
						block = CALCITE;
					} else {
						block = SMOOTH_BASALT;
					}
					shell.emplace_back(pos, block);
					written_shell.insert(packed);
				}

		std::unordered_set<std::int64_t> reverted;
		for (const auto p : written_shell) {
			const auto [x, y, z] = unpack_cave_pos(p);
			bool supported = false;
			for (const auto &[dx, dy, dz] : {std::tuple{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
						 {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}) {
				const int nx = x + dx, ny = y + dy, nz = z + dz;
				const auto np = pack_cave_pos(nx, ny, nz);
				if (written_shell.contains(np) ||
						(!hollow.contains({nx, ny, nz}) && standing(nx, ny, nz))) {
					supported = true;
					break;
				}
			}
			if (!supported)
				reverted.insert(p);
		}

		for (const auto &[x, y, z] : hollow) {
			overlay[pack_cave_pos(x, y, z)] = false;
			if (region.contains(x, z) && y >= write_min_y && y <= write_max_y)
				editor.set_block_absolute(AIR, x, y, z, std::nullopt, bedrock);
		}
		for (const auto &[pos, block] : shell) {
			const auto [x, y, z] = pos;
			const auto p = pack_cave_pos(x, y, z);
			const bool keep = !reverted.contains(p);
			overlay[p] = keep;
			if (region.contains(x, z) && y >= write_min_y && y <= write_max_y)
				editor.set_block_absolute(
						keep ? block : AIR, x, y, z, std::nullopt, bedrock);
		}
		for (const auto &[bx, by, bz] : buddings) {
			if (reverted.contains(pack_cave_pos(bx, by, bz)) ||
					decor_hash(bx, by, bz, seed ^ 0xC1A5ULL) % 100 >= 70)
				continue;
			clusters.emplace_back(gx, gy, gz, bx, by, bz);
		}
	}
	for (const auto &[gx, gy, gz, bx, by, bz] : clusters) {
		const int dx = gx - bx, dy = gy - by, dz = gz - bz;
		const char *face;
		int x = bx, y = by, z = bz;
		if (std::abs(dx) >= std::abs(dy) && std::abs(dx) >= std::abs(dz)) {
			face = dx > 0 ? "east" : "west";
			x += dx > 0 ? 1 : -1;
		} else if (std::abs(dy) >= std::abs(dz)) {
			face = dy > 0 ? "up" : "down";
			y += dy > 0 ? 1 : -1;
		} else {
			face = dz > 0 ? "south" : "north";
			z += dz > 0 ? 1 : -1;
		}
		if (!region.contains(x, z) || y < write_min_y || y > write_max_y)
			continue;
		BlockWithProperties cluster{
				AMETHYST_CLUSTER, {{"facing", face}, {"waterlogged", "false"}}};
		editor.set_block_with_properties_absolute(
				cluster, x, y, z, std::vector<Block>{AIR}, std::nullopt);
	}
}

constexpr std::array<std::tuple<int, int, int>, 6> CAVE_NEIGHBOURS{
		{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};

void despeckle_and_prune(world_editor::WorldEditor &editor, const CaveRect &region,
		std::unordered_set<std::int64_t> &air, int floor_y)
{
	const std::vector<Block> hosts{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRAVEL,
			DIRT, ANDESITE, GRANITE, DIORITE};
	const std::optional<std::vector<Block>> host_options{hosts};
	for (int pass = 0; pass < 2; ++pass) {
		std::unordered_set<std::int64_t> candidates;
		std::vector<std::int64_t> remove;
		for (const auto p : air) {
			const auto [x, y, z] = unpack_cave_pos(p);
			for (const auto &[dx, dy, dz] : CAVE_NEIGHBOURS) {
				const int nx = x + dx, ny = y + dy, nz = z + dz;
				const auto np = pack_cave_pos(nx, ny, nz);
				if (air.contains(np) || !candidates.insert(np).second)
					continue;
				int air_neighbours = 0;
				for (const auto &[ddx, ddy, ddz] : CAVE_NEIGHBOURS)
					air_neighbours +=
							air.contains(pack_cave_pos(nx + ddx, ny + ddy, nz + ddz));
				if (air_neighbours >= 5)
					remove.push_back(np);
			}
		}
		if (remove.empty())
			break;
		for (const auto p : remove) {
			const auto [x, y, z] = unpack_cave_pos(p);
			editor.set_block_absolute(AIR, x, y, z, host_options, std::nullopt);
			air.insert(p);
		}
	}

	std::unordered_set<std::int64_t> seen;
	std::vector<std::int64_t> refill;
	for (const auto start : air) {
		if (!seen.insert(start).second)
			continue;
		std::vector<std::int64_t> component{start};
		std::vector<std::int64_t> stack{start};
		bool touches_edge = false;
		while (!stack.empty()) {
			const auto p = stack.back();
			stack.pop_back();
			const auto [x, y, z] = unpack_cave_pos(p);
			if (x <= region.min_x || x >= region.max_x || z <= region.min_z ||
					z >= region.max_z)
				touches_edge = true;
			for (const auto &[dx, dy, dz] : CAVE_NEIGHBOURS) {
				const auto np = pack_cave_pos(x + dx, y + dy, z + dz);
				if (air.contains(np) && seen.insert(np).second) {
					stack.push_back(np);
					component.push_back(np);
				}
			}
		}
		if (component.size() < 48 && !touches_edge)
			refill.insert(refill.end(), component.begin(), component.end());
	}
	const int vanilla_zero = floor_y + 64;
	const std::vector<Block> air_only{AIR};
	for (const auto p : refill) {
		const auto [x, y, z] = unpack_cave_pos(p);
		editor.set_block_absolute(
				y < vanilla_zero ? DEEPSLATE : STONE, x, y, z, air_only, std::nullopt);
		air.erase(p);
	}
}
}
void carve_region(world_editor::WorldEditor &editor, const CaveRect &region,
		const CaveRect &world_bounds, std::int64_t seed, int floor_y,
		CaveEllipsoids *ellipsoids, std::unordered_set<std::int64_t> *cave_air)
{
	CaveGen density(seed, floor_y, world_editor::world_max_y());
	std::unordered_set<std::int64_t> carved;
	carve_density_region(density, editor, region.min_x, region.max_x, region.min_z,
			region.max_z, floor_y, &carved);
	// Random-walk carvers are origin-chunk features; use Rust's branching cave,
	// extra-underground cave, and canyon walks with the same deterministic salts.
	if (ellipsoids)
		ellipsoids->clear();
	// Feature planning queries the un-pruned cave shape beyond this tile. Collect
	// carvers over the same halo as Rust's CaveShape, but keep voxel writes local.
	const CaveRect shape_region = region.grow(CAVE_SHAPE_MARGIN).clip(world_bounds);
	const CaveRect &carver_origin_region = ellipsoids ? shape_region : region;
	carve_random_walk_carvers(
			editor, region, carver_origin_region, seed, floor_y, ellipsoids, &carved);
	despeckle_and_prune(editor, region, carved, floor_y);
	if (cave_air)
		*cave_air = std::move(carved);
}

void decorate_region(world_editor::WorldEditor &editor, const CaveRect &region,
		const CaveRect &world_bounds, std::int64_t seed, int floor_y, const Args &args,
		const CaveShapeQuery &cave_shape, const WaterPlan &water_plan,
		const std::unordered_set<std::int64_t> *basin_fluid,
		const std::unordered_set<std::int64_t> *water_cells,
		const std::unordered_set<std::int64_t> *cave_air)
{
	const auto amounts = args.cave_biomes
								 ? biome_amounts_for_generation(*args.cave_biomes)
								 : BiomeAmounts::defaults();
	const ThemeSelector themes(seed, amounts, floor_y);
	const auto decoration_seed = static_cast<std::uint64_t>(seed);
	const std::vector<Block> rocks{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, GRAVEL, DIRT};
	const std::optional<std::vector<Block>> rock_options{rocks};
	const std::optional<std::vector<Block>> water_options{std::vector<Block>{WATER}};
	std::unordered_set<std::int64_t> empty_air;
	const auto &air_cells = cave_air ? *cave_air : empty_air;
	// Match Rust: decorate only the explicitly tracked cave-air cells. Fluid
	// cells are consulted separately and unrelated voids are not candidates.
	for (const auto p : air_cells) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (!region.contains(x, z))
			continue;
		const bool floor_rock =
				editor.check_for_block_absolute(x, y - 1, z, rock_options);
		const bool ceiling_rock =
				editor.check_for_block_absolute(x, y + 1, z, rock_options);
		const bool east_rock = editor.check_for_block_absolute(x + 1, y, z, rock_options);
		const bool west_rock = editor.check_for_block_absolute(x - 1, y, z, rock_options);
		const bool south_rock =
				editor.check_for_block_absolute(x, y, z + 1, rock_options);
		const bool north_rock =
				editor.check_for_block_absolute(x, y, z - 1, rock_options);
		const bool adjacent_rock = floor_rock || ceiling_rock || east_rock || west_rock ||
								   south_rock || north_rock;
		if (decor_chance(x, y, z, decoration_seed ^ 0x11C4, 22)) {
			if (adjacent_rock) {
				const char *face = floor_rock	  ? "down"
								   : ceiling_rock ? "up"
								   : east_rock	  ? "east"
								   : west_rock	  ? "west"
								   : south_rock	  ? "south"
												  : "north";
				editor.set_block_with_properties_absolute(
						multiface_block(GLOW_LICHEN, face), x, y, z,
						std::vector<Block>{AIR}, std::nullopt);
			}
			// Rust consumes a lichen roll even when this air cell has no rock face;
			// themed floor/wall decoration is skipped for every successful roll.
			continue;
		}
		const auto theme = themes.at(x, y, z, editor.get_ground_level(x, z));
		if (theme == CaveTheme::Lush) {
			if (floor_rock) {
				bool wet = false;
				if (basin_fluid) {
					for (const auto &[dx, dz] :
							{std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
						if (basin_fluid->contains(pack_cave_pos(x + dx, y, z + dz)) ||
								basin_fluid->contains(
										pack_cave_pos(x + dx, y - 1, z + dz))) {
							wet = true;
							break;
						}
				}
				if (wet && decor_chance(x, 0, z, decoration_seed ^ 0x4D06, 600))
					editor.set_block_absolute(
							MUD, x, y - 1, z, rock_options, std::nullopt);
				else if (decor_chance(x, 0, z, decoration_seed ^ 0x4D05, 780)) {
					editor.set_block_absolute(
							MOSS_BLOCK, x, y - 1, z, rock_options, std::nullopt);
					if (decor_chance(x, y, z, decoration_seed ^ 0x7EE5, 800)) {
						const auto vegetation =
								decor_hash(x, y, z, decoration_seed ^ 0xA21E) % 96;
						if (vegetation < 50)
							editor.set_block_absolute(GRASS, x, y, z,
									std::optional<std::vector<Block>>(
											std::vector<Block>{AIR}),
									std::nullopt);
						else if (vegetation < 75)
							editor.set_block_absolute(MOSS_CARPET, x, y, z,
									std::optional<std::vector<Block>>(
											std::vector<Block>{AIR}),
									std::nullopt);
						else if (vegetation < 85) {
							if (editor.check_for_block_type_absolute(x, y + 1, z, AIR)) {
								editor.set_block_with_properties_absolute(
										BlockWithProperties{
												TALL_GRASS_BOTTOM, {{"half", "lower"}}},
										x, y, z, std::vector<Block>{AIR}, std::nullopt);
								editor.set_block_with_properties_absolute(
										BlockWithProperties{
												TALL_GRASS_TOP, {{"half", "upper"}}},
										x, y + 1, z, std::vector<Block>{AIR},
										std::nullopt);
							}
						} else if (vegetation < 92)
							editor.set_block_absolute(AZALEA, x, y, z,
									std::vector<Block>{AIR}, std::nullopt);
						else
							editor.set_block_absolute(FLOWERING_AZALEA, x, y, z,
									std::vector<Block>{AIR}, std::nullopt);
					}
				} else if (decor_hash(x, y, z, decoration_seed ^ 0xC1A1) % 700 == 0)
					place_clay_pool(editor, x, y, z, rock_options, air_cells);
			} else if (ceiling_rock) {
				if (decor_chance(x, 0, z, decoration_seed ^ 0x4D05, 650))
					editor.set_block_absolute(
							MOSS_BLOCK, x, y + 1, z, rock_options, std::nullopt);
				if (decor_chance(x, y, z, decoration_seed ^ 0x617E, 190))
					hang_cave_vines(editor, air_cells, x, y, z, decoration_seed);
				else if (decor_chance(x, y, z, decoration_seed ^ 0x5B07, 25))
					editor.set_block_absolute(SPORE_BLOSSOM, x, y, z,
							std::vector<Block>{AIR}, std::nullopt);
			}
		} else if (theme == CaveTheme::Dripstone) {
			if (floor_rock && decor_chance(x, 0, z, decoration_seed ^ 0xDB10, 680))
				editor.set_block_absolute(
						DRIPSTONE_BLOCK, x, y - 1, z, rock_options, std::nullopt);
			if (ceiling_rock && decor_chance(x, 0, z, decoration_seed ^ 0xDB11, 680))
				editor.set_block_absolute(
						DRIPSTONE_BLOCK, x, y + 1, z, rock_options, std::nullopt);
			if (ceiling_rock && decor_chance(x, y, z, decoration_seed ^ 0xD21A, 170))
				grow_dripstone(editor, x, y, z, decoration_seed, false);
			else if (floor_rock && decor_chance(x, y, z, decoration_seed ^ 0xD21B, 130))
				grow_dripstone(editor, x, y, z, decoration_seed, true);
		} else if (theme == CaveTheme::Amethyst) {
			if (floor_rock) {
				if (editor.check_for_block_absolute(x, y - 2, z, rock_options) &&
						decor_chance(x, 0, z, decoration_seed ^ 0xA3E1, 620)) {
					const auto palette =
							decor_hash(x, 0, z, decoration_seed ^ 0xA3E2) % 100;
					const Block block = palette < 52   ? AMETHYST_BLOCK
										: palette < 68 ? BUDDING_AMETHYST
										: palette < 86 ? CALCITE
													   : SMOOTH_BASALT;
					editor.set_block_absolute(
							block, x, y - 1, z, rock_options, std::nullopt);
					if (decor_chance(x, y, z, decoration_seed ^ 0xA3E3, 85)) {
						const auto bud_roll =
								decor_hash(x, y, z, decoration_seed ^ 0xA3E4) % 100;
						const Block bud = bud_roll < 35	  ? SMALL_AMETHYST_BUD
										  : bud_roll < 60 ? MEDIUM_AMETHYST_BUD
										  : bud_roll < 82 ? LARGE_AMETHYST_BUD
														  : AMETHYST_CLUSTER;
						editor.set_block_with_properties_absolute(
								BlockWithProperties{bud, {{"facing", "up"}}}, x, y, z,
								std::vector<Block>{AIR}, std::nullopt);
					}
				}
			} else if (ceiling_rock &&
					   editor.check_for_block_absolute(x, y + 2, z, rock_options) &&
					   decor_chance(x, 0, z, decoration_seed ^ 0xA3E5, 560)) {
				const auto palette = decor_hash(x, 1, z, decoration_seed ^ 0xA3E2) % 100;
				const Block block = palette < 52   ? AMETHYST_BLOCK
									: palette < 68 ? BUDDING_AMETHYST
									: palette < 86 ? CALCITE
												   : SMOOTH_BASALT;
				editor.set_block_absolute(block, x, y + 1, z, rock_options, std::nullopt);
				if (decor_chance(x, y, z, decoration_seed ^ 0xA3E6, 65)) {
					const auto bud_roll =
							decor_hash(x, y, z, decoration_seed ^ 0xA3E4) % 100;
					const Block bud = bud_roll < 40	  ? SMALL_AMETHYST_BUD
									  : bud_roll < 70 ? MEDIUM_AMETHYST_BUD
													  : AMETHYST_CLUSTER;
					editor.set_block_with_properties_absolute(
							BlockWithProperties{bud, {{"facing", "down"}}}, x, y, z,
							std::vector<Block>{AIR}, std::nullopt);
				}
			}
		} else if (theme == CaveTheme::Volcanic) {
			if (floor_rock) {
				if (editor.check_for_block_absolute(x, y - 2, z, rock_options) &&
						decor_chance(x, 0, z, decoration_seed ^ 0xBA5A, 720)) {
					const auto palette =
							decor_hash(x, 0, z, decoration_seed ^ 0xBA5B) % 100;
					const Block block = palette < 40   ? BASALT
										: palette < 64 ? SMOOTH_BASALT
										: palette < 78 ? BLACKSTONE
										: palette < 90 ? POLISHED_BLACKSTONE
													   : MAGMA_BLOCK;
					editor.set_block_absolute(
							block, x, y - 1, z, rock_options, std::nullopt);
				}
				if (decor_hash(x, y, z, decoration_seed ^ 0xBA5D) % 170 == 0) {
					const BlockWithProperties pillar{BASALT, {{"axis", "y"}}};
					for (int i = 0; i < 14; ++i) {
						if (editor.block_exists_absolute(x, y + i, z))
							break;
						editor.set_block_with_properties_absolute(pillar, x, y + i, z,
								std::vector<Block>{AIR}, std::nullopt);
					}
				} else if (decor_hash(x, y, z, decoration_seed ^ 0xBA5E) % 520 == 0) {
					bool water_near = false;
					for (int dx = -3; dx <= 3 && !water_near; ++dx)
						for (int dy = -2; dy <= 2 && !water_near; ++dy)
							for (int dz = -3; dz <= 3; ++dz)
								if (editor.check_for_block_absolute(
											x + dx, y + dy, z + dz, water_options)) {
									water_near = true;
									break;
								}
					const bool contained =
							editor.check_for_block_absolute(
									x + 1, y - 1, z, rock_options) &&
							editor.check_for_block_absolute(
									x - 1, y - 1, z, rock_options) &&
							editor.check_for_block_absolute(
									x, y - 1, z + 1, rock_options) &&
							editor.check_for_block_absolute(
									x, y - 1, z - 1, rock_options) &&
							editor.check_for_block_absolute(x, y - 2, z, rock_options);
					if (!water_near && contained) {
						editor.set_block_absolute(
								LAVA, x, y - 1, z, rock_options, std::nullopt);
						editor.schedule_fluid_tick(LAVA, x, y - 1, z);
					}
				}
			} else if (ceiling_rock &&
					   editor.check_for_block_absolute(x, y + 2, z, rock_options) &&
					   decor_chance(x, 0, z, decoration_seed ^ 0xBA5C, 550)) {
				const auto palette = decor_hash(x, 1, z, decoration_seed ^ 0xBA5B) % 100;
				const Block block = palette < 50   ? BASALT
									: palette < 85 ? SMOOTH_BASALT
												   : POLISHED_BLACKSTONE;
				editor.set_block_absolute(block, x, y + 1, z, rock_options, std::nullopt);
			}
		} else if (theme == CaveTheme::DeepDark) {
			if (floor_rock) {
				if (decor_chance(x, 0, z, decoration_seed ^ 0x5CA1, 720)) {
					editor.set_block_absolute(
							SCULK, x, y - 1, z, rock_options, std::nullopt);
					const auto feature =
							decor_hash(x, y, z, decoration_seed ^ 0x5CB2) % 1000;
					if (feature < 12)
						editor.set_block_absolute(SCULK_SENSOR, x, y, z,
								std::vector<Block>{AIR}, std::nullopt);
					else if (feature < 17)
						editor.set_block_absolute(SCULK_SHRIEKER, x, y, z,
								std::vector<Block>{AIR}, std::nullopt);
					else if (feature < 22)
						editor.set_block_absolute(SCULK_CATALYST, x, y - 1, z,
								std::vector<Block>{SCULK}, std::nullopt);
				}
			} else if (ceiling_rock) {
				if (decor_chance(x, 0, z, decoration_seed ^ 0x5CA2, 380))
					editor.set_block_absolute(
							SCULK, x, y + 1, z, rock_options, std::nullopt);
			} else if (decor_chance(x, y, z, decoration_seed ^ 0x5CA3, 50)) {
				const char *face = east_rock	? "east"
								   : west_rock	? "west"
								   : south_rock ? "south"
												: "north";
				editor.set_block_with_properties_absolute(
						multiface_block(SCULK_VEIN, face), x, y, z,
						std::vector<Block>{AIR}, std::nullopt);
			}
		} else if (theme == CaveTheme::Ice) {
			if (floor_rock) {
				if (decor_chance(x, 0, z, decoration_seed ^ 0x1CE1, 700)) {
					const auto ice = decor_hash(x, 0, z, decoration_seed ^ 0x1CE2) % 10;
					const Block floor_block =
							ice < 6 ? ICE : (ice < 9 ? PACKED_ICE : BLUE_ICE);
					editor.set_block_absolute(
							floor_block, x, y - 1, z, rock_options, std::nullopt);
					if (decor_chance(x, y, z, decoration_seed ^ 0x1CE3, 120))
						editor.set_block_absolute(SNOW_LAYER, x, y, z,
								std::optional<std::vector<Block>>(
										std::vector<Block>{AIR}),
								std::nullopt);
				}
			} else if (ceiling_rock &&
					   decor_chance(x, 0, z, decoration_seed ^ 0x1CE4, 620)) {
				const auto ice = decor_hash(x, 1, z, decoration_seed ^ 0x1CE2) % 10;
				const Block ceiling_block =
						ice < 6 ? ICE : (ice < 9 ? PACKED_ICE : BLUE_ICE);
				editor.set_block_absolute(
						ceiling_block, x, y + 1, z, rock_options, std::nullopt);
			}
		} else if (theme == CaveTheme::Mushroom) {
			if (floor_rock) {
				if (decor_chance(x, 0, z, decoration_seed ^ 0x54AD, 750)) {
					editor.set_block_absolute(
							MYCELIUM, x, y - 1, z, rock_options, std::nullopt);
					const auto mushroom =
							decor_hash(x, y, z, decoration_seed ^ 0x54AE) % 1000;
					if (mushroom < 30)
						editor.set_block_absolute(RED_MUSHROOM, x, y, z,
								std::vector<Block>{AIR}, std::nullopt);
					else if (mushroom < 65)
						editor.set_block_absolute(BROWN_MUSHROOM, x, y, z,
								std::vector<Block>{AIR}, std::nullopt);
				}
				if (decor_hash(x, y, z, decoration_seed ^ 0x54AF) % 240 == 0)
					build_giant_mushroom(
							editor, region, air_cells, x, y, z, decoration_seed);
			} else if (ceiling_rock &&
					   decor_chance(x, y, z, decoration_seed ^ 0x54B0, 35))
				hang_cave_vines(editor, air_cells, x, y, z, decoration_seed);
		}
	}
	// Coral decoration is a separate pass over the water feature cells, just as
	// in Rust; it must not be repeated for every cave-air cell.
	if (water_cells) {
		const Block coral_blocks[] = {TUBE_CORAL_BLOCK, BRAIN_CORAL_BLOCK,
				FIRE_CORAL_BLOCK, HORN_CORAL_BLOCK, BUBBLE_CORAL_BLOCK};
		const Block coral_fans[] = {TUBE_CORAL_FAN, BRAIN_CORAL_FAN, FIRE_CORAL_FAN,
				HORN_CORAL_FAN, BUBBLE_CORAL_FAN};
		const Block corals[] = {
				TUBE_CORAL, BRAIN_CORAL, FIRE_CORAL, HORN_CORAL, BUBBLE_CORAL};
		const Block dead_beds[] = {DEAD_TUBE_CORAL_BLOCK, DEAD_BRAIN_CORAL_BLOCK,
				DEAD_FIRE_CORAL_BLOCK, DEAD_HORN_CORAL_BLOCK, DEAD_BUBBLE_CORAL_BLOCK};
		for (const auto packed : *water_cells) {
			const auto [x, y, z] = unpack_cave_pos(packed);
			if (!region.contains(x, z) || !themes.coral_zone(x, z) ||
					!editor.check_for_block_absolute(x, y - 1, z, rock_options))
				continue;
			const auto bed_roll = decor_hash(x, 0, z, decoration_seed ^ 0xC0A1) % 100;
			const Block bed = bed_roll < 18	  ? dead_beds[0]
							  : bed_roll < 36 ? dead_beds[1]
							  : bed_roll < 49 ? dead_beds[2]
							  : bed_roll < 62 ? dead_beds[3]
							  : bed_roll < 75 ? dead_beds[4]
							  : bed_roll < 89 ? SAND
											  : GRAVEL;
			editor.set_block_absolute(bed, x, y - 1, z, rock_options, std::nullopt);
			const auto growth = decor_hash(x, y, z, decoration_seed ^ 0xC0A2) % 1000;
			Block growth_block = AIR;
			if (growth < 90)
				growth_block =
						coral_blocks[decor_hash(x, y, z, decoration_seed ^ 0xC0A3) % 5];
			else if (growth < 210)
				growth_block =
						coral_fans[decor_hash(x, y, z, decoration_seed ^ 0xC0A4) % 5];
			else if (growth < 300)
				growth_block = corals[decor_hash(x, y, z, decoration_seed ^ 0xC0A5) % 5];
			else if (growth < 360)
				growth_block = SEA_PICKLE;
			else if (growth < 450)
				growth_block = SEAGRASS;
			if (growth_block != AIR)
				editor.set_block_absolute(
						growth_block, x, y, z, std::vector<Block>{WATER}, std::nullopt);
		}
	}
	// Geodes are a chunk/region-level feature pass in Rust, independent of the
	// cave-air and water-cell counts. Keep it outside both per-cell passes.
	place_geodes(editor, region, floor_y, world_bounds, decoration_seed, cave_shape,
			water_plan, cave_air);
}
} // namespace arnis::caves
