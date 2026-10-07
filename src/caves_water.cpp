#include "caves_water.h"
#include "args.h"
#include "block_definitions.h"
#include "caves_biomes.h"
#include "caves_density.h"
#include "caves_rng.h"
#include "caves_theme.h"
#include "world_editor/floor_state.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arnis::caves
{
namespace
{
const std::vector<Block> HOSTS{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRAVEL, DIRT,
		ANDESITE, GRANITE, DIORITE};
const std::vector<Block> FLUIDS{WATER, LAVA};

double lerp(double t, double a, double b)
{
	return a + t * (b - a);
}

int chunk_floor(int v)
{
	return v >= 0 ? v / 16 : -(((-v) + 15) / 16);
}

bool host(const world_editor::WorldEditor &e, int x, int y, int z)
{
	return e.check_for_block_absolute(x, y, z, HOSTS);
}

class CaveShapeQuery
{
public:
	CaveShapeQuery(const world_editor::WorldEditor &editor, const CaveRect &world,
			std::int64_t seed, int floor_y, const CaveEllipsoids &ellipsoids) :
			editor_(editor), world_(world), floor_y_(floor_y),
			gen_(seed, floor_y, world_editor::world_max_y()), ellipsoids_(ellipsoids)
	{
		const auto bounds = editor_.writable_y_bounds();
		write_min_y_ = bounds.first;
		write_max_y_ = bounds.second;
		density_cache_.reserve(4096);
		for (std::size_t i = 0; i < ellipsoids_.size(); ++i) {
			const auto &e = ellipsoids_[i];
			const int x0 = CaveRect::floor_div(
					static_cast<int>(std::floor(e.x - e.horizontal_radius)), 16);
			const int x1 = CaveRect::floor_div(
					static_cast<int>(std::floor(e.x + e.horizontal_radius)), 16);
			const int z0 = CaveRect::floor_div(
					static_cast<int>(std::floor(e.z - e.horizontal_radius)), 16);
			const int z1 = CaveRect::floor_div(
					static_cast<int>(std::floor(e.z + e.horizontal_radius)), 16);
			for (int cx = x0; cx <= x1; ++cx)
				for (int cz = z0; cz <= z1; ++cz)
					carvers_by_chunk_[{cx, cz}].push_back(i);
		}
	}

	bool is_cave(int x, int y, int z) const
	{
		if (!world_.contains(x, z) || y <= floor_y_ ||
				y > editor_.get_ground_level(x, z) - 6 || y < write_min_y_ ||
				y > write_max_y_)
			return false;
		if (const auto it = carvers_by_chunk_.find(
					{CaveRect::floor_div(x, 16), CaveRect::floor_div(z, 16)});
				it != carvers_by_chunk_.end())
			for (const auto i : it->second)
				if (ellipsoids_[i].contains(x, y, z))
					return true;
		return noise_carves(x, y, z);
	}

private:
	double density_at(int x, int y, int z) const
	{
		const auto key = pack_cave_pos(x, y, z);
		if (const auto it = density_cache_.find(key); it != density_cache_.end())
			return it->second;
		return density_cache_.emplace(key, gen_.combined_density(x, y, z)).first->second;
	}

	bool noise_carves(int x, int y, int z) const
	{
		const int wx0 = CaveRect::floor_div(x, 4) * 4;
		const int wy0 = CaveRect::floor_div(y, 8) * 8;
		const int wz0 = CaveRect::floor_div(z, 4) * 4;
		const int wx1 = wx0 + 4, wy1 = wy0 + 8, wz1 = wz0 + 4;
		const double n000 = density_at(wx0, wy0, wz0);
		const double n100 = density_at(wx1, wy0, wz0);
		const double n001 = density_at(wx0, wy0, wz1);
		const double n101 = density_at(wx1, wy0, wz1);
		const double n010 = density_at(wx0, wy1, wz0);
		const double n110 = density_at(wx1, wy1, wz0);
		const double n011 = density_at(wx0, wy1, wz1);
		const double n111 = density_at(wx1, wy1, wz1);
		const double fy = double(y - wy0) / 8.0;
		const double xz00 = lerp(fy, n000, n010);
		const double xz10 = lerp(fy, n100, n110);
		const double xz01 = lerp(fy, n001, n011);
		const double xz11 = lerp(fy, n101, n111);
		const double fx = double(x - wx0) / 4.0;
		const double z0 = lerp(fx, xz00, xz10);
		const double z1 = lerp(fx, xz01, xz11);
		return lerp(double(z - wz0) / 4.0, z0, z1) <= 0.0 ||
			   gen_.noodle_density(x, y, z) <= 0.0;
	}

	const world_editor::WorldEditor &editor_;
	CaveRect world_;
	int floor_y_;
	CaveGen gen_;
	const CaveEllipsoids &ellipsoids_;
	int write_min_y_ = 0, write_max_y_ = -1;
	std::map<std::pair<int, int>, std::vector<std::size_t>> carvers_by_chunk_;
	mutable std::unordered_map<std::int64_t, double> density_cache_;
};

bool planned_solid(const world_editor::WorldEditor &editor, const CaveRect &world,
		int floor_y, const std::unordered_set<std::int64_t> &carved,
		const std::unordered_set<std::int64_t> &water,
		const std::function<bool(int, int, int)> &is_cave, int x, int y, int z)
{
	const auto p = pack_cave_pos(x, y, z);
	if (water.contains(p))
		return true;
	return world.contains(x, z) && y >= floor_y && y <= editor.get_ground_level(x, z) &&
		   !is_cave(x, y, z) && !carved.contains(p);
}

void walk_river(world_editor::WorldEditor &editor, const CaveRect &world, int floor_y,
		std::unordered_set<std::int64_t> &carved, std::unordered_set<std::int64_t> &water,
		const std::function<bool(int, int, int)> &is_cave, std::int64_t seed, double x,
		double y, double z, double start_yaw, int steps, int splits_left)
{
	XoroRandom r = XoroRandom::from_seed(seed);
	double yaw = start_yaw;
	const double thickness = 1.6 + double(r.next_float()) * 1.2;
	const int branch_step = steps / 3 + r.next_int(std::max(steps / 3, 1));
	const double sway_phase = double(r.next_float()) * 2.0 * M_PI;
	const double sway_amp = 0.18 + double(r.next_float()) * 0.14;
	std::map<std::pair<int, int>, int> by_column;
	const double level_out = double(floor_y + (-50 - VANILLA_FLOOR));

	for (int step = 0; step < steps; ++step) {
		yaw += (double(r.next_float()) - 0.5) * 0.6 +
			   std::sin(double(step) * 0.35 + sway_phase) * sway_amp;
		x += std::cos(yaw);
		z += std::sin(yaw);
		const bool descended = r.next_int(4) != 0 && y > level_out;
		if (descended)
			y -= 1.0;

		const int cx = static_cast<int>(std::lround(x));
		const int cy = static_cast<int>(std::lround(y));
		const int cz = static_cast<int>(std::lround(z));
		if (!world.contains(cx, cz))
			break;
		const bool contact_here = is_cave(cx, cy, cz);
		const bool contact_below = is_cave(cx, cy - 1, cz) || is_cave(cx, cy - 2, cz);
		if (contact_here && !(descended || contact_below))
			break;
		const bool breach = contact_here || contact_below;
		const int ri = static_cast<int>(std::ceil(thickness));
		for (int dx = -ri; dx <= ri; ++dx)
			for (int dz = -ri; dz <= ri; ++dz) {
				if (double(dx * dx + dz * dz) > thickness * thickness)
					continue;
				const int px = cx + dx, pz = cz + dz;
				if (!world.contains(px, pz))
					continue;
				for (int dy = 0; dy <= 1; ++dy) {
					const int py = cy - dy;
					if (py > editor.get_ground_level(px, pz) - 6 || is_cave(px, py, pz))
						continue;
					carved.insert(pack_cave_pos(px, py, pz));
					auto [it, inserted] = by_column.emplace(std::pair{px, pz}, py);
					if (!inserted)
						it->second = std::min(it->second, py);
				}
			}
		if (breach)
			break;

		if (splits_left > 0 && step == branch_step && steps - step > 8) {
			const int remaining = steps - step;
			for (int i = 0; i < 2; ++i) {
				const double spread = 0.5 + double(r.next_float()) * 0.5;
				const double fork_yaw = i == 0 ? yaw - spread : yaw + spread;
				const int child_splits = i == 0 ? splits_left - 1 : 0;
				const auto fork_seed = r.next_long();
				walk_river(editor, world, floor_y, carved, water, is_cave, fork_seed, x,
						y, z, fork_yaw, remaining, child_splits);
			}
			break;
		}
	}
	if (by_column.size() < 4)
		return;
	for (const auto &[column, py] : by_column) {
		const auto [px, pz] = column;
		if (planned_solid(editor, world, floor_y, carved, water, is_cave, px, py - 1, pz))
			water.insert(pack_cave_pos(px, py, pz));
	}
}

void generate_deep_lava_sea(world_editor::WorldEditor &editor, const CaveRect &region,
		int floor_y, const std::unordered_set<std::int64_t> &water_cells,
		std::unordered_set<std::int64_t> *basin_fluid)
{
	const int lava_level = floor_y + 10;
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_y = std::max(floor_y + 2, write_min_y);
	const int max_y = std::min(lava_level, write_max_y);
	if (min_y > max_y)
		return;
	std::unordered_set<std::int64_t> cave_air;
	std::size_t columns = static_cast<std::size_t>(region.max_x - region.min_x + 1) *
						  static_cast<std::size_t>(region.max_z - region.min_z + 1);
	cave_air.reserve(std::min<std::size_t>(columns * 4, 1'000'000));
	for (int x = region.min_x; x <= region.max_x; ++x)
		for (int z = region.min_z; z <= region.max_z; ++z) {
			const int top =
					std::min({editor.get_ground_level(x, z) - 8, max_y, write_max_y});
			for (int y = min_y; y <= top; ++y)
				if (!editor.block_exists_absolute(x, y, z))
					cave_air.insert(pack_cave_pos(x, y, z));
		}
	if (cave_air.empty())
		return;
	constexpr std::array<std::array<int, 3>, 6> neighbours = {
			{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
	auto is_open = [&](int x, int y, int z) {
		if (!region.contains(x, z))
			return false;
		const auto p = pack_cave_pos(x, y, z);
		return !editor.block_exists_absolute(x, y, z) && !cave_air.contains(p);
	};
	std::vector<std::int64_t> candidates;
	candidates.reserve(cave_air.size());
	for (const auto p : cave_air) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (y >= lava_level)
			continue;
		const bool exposed = std::any_of(neighbours.begin(), neighbours.end(),
				[&, x = x, y = y, z = z](const auto &offset) {
					return is_open(x + offset[0], y + offset[1], z + offset[2]);
				});
		if (!exposed)
			candidates.push_back(p);
	}
	std::sort(candidates.begin(), candidates.end(), [](std::int64_t a, std::int64_t b) {
		return std::get<1>(unpack_cave_pos(a)) < std::get<1>(unpack_cave_pos(b));
	});
	std::unordered_set<std::int64_t> supported;
	supported.reserve(candidates.size());
	for (const auto p : candidates) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (editor.block_exists_absolute(x, y - 1, z) ||
				supported.contains(pack_cave_pos(x, y - 1, z)))
			supported.insert(p);
	}
	const int stone_transition = floor_y + 64;
	const std::vector<Block> rim_rock{
			STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE, DIORITE, ANDESITE};
	for (const auto p : supported) {
		const auto [x, y, z] = unpack_cave_pos(p);
		const bool touches_water = std::any_of(neighbours.begin(), neighbours.end(),
				[&, x = x, y = y, z = z](const auto &offset) {
					return water_cells.contains(
							pack_cave_pos(x + offset[0], y + offset[1], z + offset[2]));
				});
		if (touches_water) {
			const Block seam = y < stone_transition ? DEEPSLATE : STONE;
			editor.set_block_absolute(
					seam, x, y, z, std::vector<Block>{AIR}, std::nullopt);
			continue;
		}
		editor.set_block_absolute(LAVA, x, y, z, std::vector<Block>{AIR}, std::nullopt);
		if (basin_fluid)
			basin_fluid->insert(p);
	}
	for (const auto p : supported) {
		const auto [x, y, z] = unpack_cave_pos(p);
		for (const auto &offset : neighbours) {
			const int nx = x + offset[0], ny = y + offset[1], nz = z + offset[2];
			const auto neighbour = pack_cave_pos(nx, ny, nz);
			if (!region.contains(nx, nz) || supported.contains(neighbour) ||
					cave_air.contains(neighbour))
				continue;
			const auto hash =
					static_cast<std::uint64_t>(neighbour) * 0x9E3779B97F4A7C15ULL;
			const auto roll = (hash >> 40) % 100;
			if (roll < 45)
				editor.set_block_absolute(OBSIDIAN, nx, ny, nz, rim_rock, std::nullopt);
			else if (roll < 62)
				editor.set_block_absolute(
						MAGMA_BLOCK, nx, ny, nz, rim_rock, std::nullopt);
		}
	}
}
}

void generate_water_features(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args,
		const CaveEllipsoids &ellipsoids, std::unordered_set<std::int64_t> *basin_fluid,
		std::unordered_set<std::int64_t> *water_cells_out,
		std::unordered_set<std::int64_t> *cave_air_out)
{
	BiomeAmounts amounts = BiomeAmounts::defaults();
	if (args.cave_biomes)
		amounts = BiomeAmounts::parse(*args.cave_biomes);
	const ThemeSelector decor(seed, amounts, floor_y);
	std::unordered_set<std::int64_t> water_cells;
	std::unordered_set<std::int64_t> carved_cells;
	const CaveRect origins = region.grow(64);
	const int cx0 = chunk_floor(origins.min_x), cx1 = chunk_floor(origins.max_x);
	const int cz0 = chunk_floor(origins.min_z), cz1 = chunk_floor(origins.max_z);
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const CaveShapeQuery cave_shape(editor, region, seed, floor_y, ellipsoids);
	auto dry_cave = [&](int x, int y, int z) { return cave_shape.is_cave(x, y, z); };

	for (int cx = cx0; cx <= cx1; ++cx)
		for (int cz = cz0; cz <= cz1; ++cz) {
			XoroRandom pool_rng = XoroRandom::from_seed(
					seed ^ (std::int64_t(cx) * 341873128712LL) ^
					(std::int64_t(cz) * 132897987541LL) ^ 0xA044A044LL);
			if (pool_rng.next_int(30) == 0) {
				const int gx = cx * 16 + pool_rng.next_int(16);
				const int gz = cz * 16 + pool_rng.next_int(16);
				if (region.contains(gx, gz)) {
					const int top = editor.get_ground_level(gx, gz) - 6;
					const int lo = floor_y + 16;
					const int hi = std::min(top - 10, floor_y + 94);
					if (hi > lo) {
						const int gy = lo + pool_rng.next_int(hi - lo + 1);
						const bool coral = decor.coral_zone(gx, gz);
						const int n_lobes = (coral ? 3 : 2) + pool_rng.next_int(3);
						const double r_lo = coral ? 6.0 : 5.0;
						const double r_span = coral ? 4.0 : 6.0;
						const double v_lo = coral ? 3.0 : 2.5;
						const double v_span = 1.5;
						const int o_span = coral ? 9 : 7;
						const bool grand = pool_rng.next_int(4) == 0;
						const bool stretch_x = pool_rng.next_int(2) == 0;
						struct Lobe
						{
							int ox, oz;
							double rx, rz, rv;
						};
						std::vector<Lobe> lobes;
						lobes.reserve(n_lobes);
						double reach = 0.0, vertical_reach = 0.0;
						for (int i = 0; i < n_lobes; ++i) {
							double rx = r_lo + double(pool_rng.next_float()) * r_span;
							double rz = r_lo + double(pool_rng.next_float()) * r_span;
							if (grand) {
								if (stretch_x)
									rx *= 1.45;
								else
									rz *= 1.45;
							}
							const int ox = pool_rng.next_int(o_span) - o_span / 2;
							const int oz = pool_rng.next_int(o_span) - o_span / 2;
							const double rv =
									v_lo + double(pool_rng.next_float()) * v_span;
							lobes.push_back({ox, oz, rx, rz, rv});
							reach = std::max(reach, std::max(rx, rz));
							vertical_reach = std::max(vertical_reach, rv);
						}
						const int ri_h = static_cast<int>(std::ceil(reach)) + 4;
						const int ri_v = static_cast<int>(std::ceil(vertical_reach));
						std::set<std::tuple<int, int, int>> cells;
						for (int dx = -ri_h; dx <= ri_h; ++dx)
							for (int dz = -ri_h; dz <= ri_h; ++dz) {
								const int px = gx + dx, pz = gz + dz;
								if (!region.contains(px, pz))
									continue;
								const int ptop = editor.get_ground_level(px, pz) - 6;
								for (int dy = -ri_v; dy <= ri_v; ++dy) {
									const int py = gy + dy;
									if (py > ptop || py < write_min_y || py > write_max_y)
										continue;
									const bool inside = std::any_of(lobes.begin(),
											lobes.end(), [&](const Lobe &l) {
												const double ndx =
														double(dx - l.ox) / l.rx;
												const double ndz =
														double(dz - l.oz) / l.rz;
												const double ndy = double(dy) / l.rv;
												return ndx * ndx + ndz * ndz +
															   ndy * ndy <=
													   1.0;
											});
									if (inside)
										cells.emplace(px, py, pz);
								}
							}
						if (cells.size() >= 40 && std::none_of(cells.begin(), cells.end(),
														  [&](const auto &p) {
															  const auto [x, y, z] = p;
															  return dry_cave(x, y, z);
														  })) {
							int y_lo = std::numeric_limits<int>::max();
							int y_hi = std::numeric_limits<int>::min();
							for (const auto &[x, y, z] : cells) {
								carved_cells.insert(pack_cave_pos(x, y, z));
								y_lo = std::min(y_lo, y);
								y_hi = std::max(y_hi, y);
							}
							const int mid = coral ? y_lo + (y_hi - y_lo) * 3 / 4
												  : (y_lo + y_hi) / 2;
							std::vector<std::tuple<int, int, int>> fill;
							for (const auto &p : cells)
								if (std::get<1>(p) <= mid)
									fill.push_back(p);
							std::sort(fill.begin(), fill.end(),
									[](const auto &a, const auto &b) {
										return std::get<1>(a) < std::get<1>(b);
									});
							for (const auto &[x, y, z] : fill)
								if (planned_solid(editor, region, floor_y, carved_cells,
											water_cells, dry_cave, x, y - 1, z))
									water_cells.insert(pack_cave_pos(x, y, z));
						}
					}
				}
			}

			XoroRandom river_rng = XoroRandom::from_seed(
					seed ^ (std::int64_t(cx) * 341873128712LL) ^
					(std::int64_t(cz) * 132897987541LL) ^ 0xA045A045LL);
			if (river_rng.next_int(18) != 0)
				continue;
			const int ox = cx * 16 + river_rng.next_int(16);
			const int oz = cz * 16 + river_rng.next_int(16);
			if (!region.contains(ox, oz))
				continue;
			const int lo = floor_y + 44;
			const int hi = std::min(editor.get_ground_level(ox, oz) - 14, floor_y + 102);
			if (hi <= lo)
				continue;
			const int oy = lo + river_rng.next_int(hi - lo + 1);
			if (dry_cave(ox, lo + (hi - lo) / 2, oz) || dry_cave(ox, oy, oz))
				continue;
			const double yaw = double(river_rng.next_float()) * 2.0 * M_PI;
			const int steps = 26 + river_rng.next_int(27);
			walk_river(editor, region, floor_y, carved_cells, water_cells, dry_cave,
					river_rng.next_long(), ox, oy, oz, yaw, steps, 2);
		}

	// A later river can undercut a source placed by an earlier branch or pool.
	std::vector<std::int64_t> unsupported;
	for (const auto p : water_cells) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (!planned_solid(editor, region, floor_y, carved_cells, water_cells, dry_cave,
					x, y - 1, z))
			unsupported.push_back(p);
	}
	for (const auto p : unsupported)
		water_cells.erase(p);

	std::vector<std::int64_t> ordered_carved(carved_cells.begin(), carved_cells.end());
	std::sort(ordered_carved.begin(), ordered_carved.end());
	for (const auto p : ordered_carved) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (host(editor, x, y, z))
			editor.set_block_absolute(AIR, x, y, z, HOSTS, std::nullopt);
	}
	std::vector<std::int64_t> ordered_water(water_cells.begin(), water_cells.end());
	std::sort(ordered_water.begin(), ordered_water.end());
	for (const auto p : ordered_water) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (editor.check_for_block_absolute(x, y, z, std::vector<Block>{AIR}))
			editor.set_block_absolute(
					WATER, x, y, z, std::vector<Block>{AIR}, std::nullopt);
	}
	if (water_cells_out)
		water_cells_out->insert(water_cells.begin(), water_cells.end());
	if (cave_air_out)
		cave_air_out->insert(carved_cells.begin(), carved_cells.end());
	if (basin_fluid)
		basin_fluid->insert(water_cells.begin(), water_cells.end());
	generate_deep_lava_sea(editor, region, floor_y, water_cells, basin_fluid);
}

void seal_floating_fluid_region(
		world_editor::WorldEditor &editor, const CaveRect &region, int floor_y)
{
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_y = std::max(floor_y + 1, write_min_y);
	if (min_y > write_max_y)
		return;

	// Match Rust's scan-then-write pass: edits are collected before any plug is
	// applied, and each fluid cell over an empty cell gets its own support block.
	std::vector<std::array<int, 3>> plugs;
	for (int x = region.min_x; x <= region.max_x; ++x)
		for (int z = region.min_z; z <= region.max_z; ++z) {
			const int top = std::min(editor.get_ground_level(x, z) + 2, write_max_y);
			for (int y = top; y >= min_y; --y)
				if (editor.check_for_block_absolute(x, y, z, FLUIDS) &&
						!editor.block_exists_absolute(x, y - 1, z))
					plugs.push_back({x, y, z});
		}

	// Rust's vy(1) is terrain_floor_y + 65 (vanilla's floor is -64).
	const int deepslate_transition = floor_y + 65;
	for (const auto &fluid : plugs) {
		const int x = fluid[0], y = fluid[1], z = fluid[2];
		const Block rock = y < deepslate_transition ? DEEPSLATE : STONE;
		editor.set_block_absolute(rock, x, y - 1, z,
				std::optional<std::vector<Block>>({AIR}), std::nullopt);
	}
}

bool WaterPlan::solid(const CaveRect &world, int floor, int x, int y, int z,
		const std::function<bool(int, int, int)> &is_cave) const
{
	const auto p = pack_cave_pos(x, y, z);
	if (water.contains(p))
		return true;
	return world.contains(x, z) && y >= floor && !is_cave(x, y, z) && !carved.contains(p);
}
void WaterPlan::apply(world_editor::WorldEditor &editor, const CaveRect &region,
		const std::vector<Block> &cave_host) const
{
	for (const auto &p : carved) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (region.contains(x, z))
			editor.set_block_absolute(AIR, x, y, z,
					std::optional<std::vector<Block>>(cave_host), std::nullopt);
	}
	for (const auto &p : water) {
		const auto [x, y, z] = unpack_cave_pos(p);
		if (region.contains(x, z)) {
			editor.set_block_absolute(WATER, x, y, z,
					std::optional<std::vector<Block>>({AIR}), std::nullopt);
			// Freeminer's backend updates fluid propagation from the placed source;
			// it has no explicit schedule_fluid_tick API.
		}
	}
}
}
