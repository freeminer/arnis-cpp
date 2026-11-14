#include "ground_scatter.h"

#include "block_definitions.h"
#include "ground_decoration.h"
#include "structures/schem_decoder.h"
#include "../../arnis_world_editor.h"
#include "../../arnis_ground.h"
#include "deterministic_rng.h"
#include "land_cover/land_cover.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <mutex>
#include <string>

namespace arnis::ground_scatter
{
namespace
{
constexpr std::uint64_t SALT = 0x5CA77E12B0C40000ULL;
constexpr int MAX_BASE_STEP = 2;
constexpr int TOWN_REACH = 32;
const std::vector<Block> SOIL{
		GRASS_BLOCK, COARSE_DIRT, DIRT, FARMLAND, MOSS_BLOCK, PODZOL};
const std::vector<Block> ROCK_SOIL{GRASS_BLOCK, COARSE_DIRT, DIRT, MOSS_BLOCK, PODZOL};
const std::vector<Block> CROPS{WHEAT, CARROTS, POTATOES};

using SchemPtr = std::shared_ptr<const structures::StructureSchematic>;
std::mutex pool_mutex;
std::map<std::string, std::vector<SchemPtr>> pools;

std::vector<SchemPtr> load_pool(const std::filesystem::path &root, bool rocks)
{
	const std::string key = root.string() + (rocks ? "/rocks" : "/bushes");
	std::lock_guard lock(pool_mutex);
	if (auto it = pools.find(key); it != pools.end())
		return it->second;
	std::vector<std::string> names;
	if (rocks) {
		for (int i = 1; i <= 8; ++i)
			names.push_back("rock" + std::to_string(i));
	} else {
		static constexpr std::array<const char *, 10> species{"acacia", "azalea", "birch",
				"cherry", "darkoak", "flowering_azalea", "jungle", "mangrove", "oak",
				"spruce"};
		for (const auto *name : species)
			for (int i = 1; i <= 6; ++i)
				names.emplace_back(std::string(name) + "bush" + std::to_string(i));
	}
	std::vector<SchemPtr> loaded;
	loaded.reserve(names.size());
	for (const auto &name : names) {
		auto source = structures::try_load_structure_cached(root / (name + ".schem"));
		if (!source || source->voxels.empty())
			continue;
		loaded.push_back(std::make_shared<const structures::StructureSchematic>(
				source->base_anchored()));
	}
	return pools.emplace(key, std::move(loaded)).first->second;
}

std::pair<int, int> rotate_cell(int x, int z, int width, int length, unsigned rot)
{
	switch (rot & 3u) {
	case 1:
		return {length - 1 - z, x};
	case 2:
		return {width - 1 - x, length - 1 - z};
	case 3:
		return {z, width - 1 - x};
	default:
		return {x, z};
	}
}

struct Offset
{
	int x, y, z;
};
std::vector<Offset> offsets(const structures::StructureSchematic &schem, unsigned rot)
{
	const auto anchor =
			rotate_cell(schem.anchor_x, schem.anchor_z, schem.width, schem.length, rot);
	std::vector<Offset> out;
	out.reserve(schem.voxels.size());
	for (const auto &v : schem.voxels) {
		const auto p = rotate_cell(v.x, v.z, schem.width, schem.length, rot);
		out.push_back({p.first - anchor.first, v.y, p.second - anchor.second});
	}
	return out;
}

int floor_chunk(int v)
{
	return v >= 0 ? v / 16 : -((-v + 15) / 16);
}

void scatter_one(WorldEditor &editor, const Ground &ground, const Args &args,
		const XZBBox &bbox, int min_x, int max_x, int min_z, int max_z, int chunk_x,
		int chunk_z, const std::vector<SchemPtr> &rocks,
		const std::vector<SchemPtr> &bushes)
{
	ChaCha8Rng rng = coord_rng(chunk_x, chunk_z, SALT);
	const bool rock_hit =
			rng.random_bool(args.scatter.rocks ? args.scatter.rock_density : 0.0);
	const bool bush_hit =
			rng.random_bool(args.scatter.bushes ? args.scatter.bush_density : 0.0);
	if (!rock_hit && !bush_hit)
		return;
	const auto &pool = rock_hit ? rocks : bushes;
	if (pool.empty())
		return;
	std::size_t selected = 0;
	if (rock_hit) {
		unsigned total = 0;
		for (const auto &s : pool)
			total += std::max(s->width, s->length) >= 8 ? 1u : 6u;
		unsigned roll = rng.uniform(total);
		for (std::size_t i = 0; i < pool.size(); ++i) {
			const unsigned weight =
					std::max(pool[i]->width, pool[i]->length) >= 8 ? 1u : 6u;
			if (roll < weight) {
				selected = i;
				break;
			}
			roll -= weight;
		}
	} else {
		selected = rng.uniform(static_cast<std::uint32_t>(pool.size()));
	}
	const auto &schem = *pool[selected];
	const unsigned rot = rng.uniform(4);
	const auto cells = offsets(schem, rot);
	if (cells.empty())
		return;
	auto [xmin, xmax] = std::minmax_element(cells.begin(), cells.end(),
			[](const auto &a, const auto &b) { return a.x < b.x; });
	auto [zmin, zmax] = std::minmax_element(cells.begin(), cells.end(),
			[](const auto &a, const auto &b) { return a.z < b.z; });
	const int lo_x = -xmin->x, hi_x = 15 - xmax->x;
	const int lo_z = -zmin->z, hi_z = 15 - zmax->z;
	if (lo_x > hi_x || lo_z > hi_z)
		return;
	// Multiplication is defined for negative chunk coordinates; left-shifting a
	// negative signed integer is undefined in C++ (Rust's signed shift is defined).
	const int ax = chunk_x * 16 + lo_x +
				   rng.uniform(static_cast<std::uint32_t>(hi_x - lo_x + 1));
	const int az = chunk_z * 16 + lo_z +
				   rng.uniform(static_cast<std::uint32_t>(hi_z - lo_z + 1));
	const auto cover = ground.cover_class({ax - bbox.min_x(), az - bbox.min_z()});
	if (cover != land_cover::LC_CROPLAND && cover != land_cover::LC_GRASSLAND)
		return;
	for (int dz = -TOWN_REACH; dz <= TOWN_REACH; dz += 8)
		for (int dx = -TOWN_REACH; dx <= TOWN_REACH; dx += 8)
			if (ground.cover_class({ax + dx - bbox.min_x(), az + dz - bbox.min_z()}) ==
					land_cover::LC_BUILT_UP)
				return;
	auto inside = [&](int x, int z) {
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z &&
			   ground.is_in_rotated_bounds(x, z);
	};
	for (const auto &p : cells)
		if (!inside(ax + p.x, az + p.z))
			return;
	const auto &soil = rock_hit ? ROCK_SOIL : SOIL;
	int low = std::numeric_limits<int>::max(), high = std::numeric_limits<int>::min();
	for (const auto &p : cells) {
		if (p.y != 0)
			continue;
		const int x = ax + p.x, z = az + p.z;
		const int y = ground.elevation_enabled ? editor.get_ground_level(x, z)
											   : args.ground_level;
		if (editor.surface_is_sealed(x, z) ||
				!editor.check_for_block_absolute(x, y, z, soil, std::nullopt))
			return;
		low = std::min(low, y);
		high = std::max(high, y);
	}
	if (low == std::numeric_limits<int>::max() || high - low > MAX_BASE_STEP)
		return;
	const int base_y = low + 1;
	for (const auto &p : cells) {
		const int x = ax + p.x, y = base_y + p.y, z = az + p.z;
		const int gy = ground.elevation_enabled ? editor.get_ground_level(x, z)
												: args.ground_level;
		if (y > gy && editor.block_exists_absolute(x, y, z) &&
				!editor.check_for_block_absolute(
						x, y, z, ground_decoration::loose_plant_blocks(), std::nullopt) &&
				!editor.check_for_block_absolute(x, y, z, CROPS, std::nullopt))
			return;
	}
	structures::place_structure(editor, schem, ax, base_y, az, rot, nullptr);
	for (const auto &p : cells) {
		const int x = ax + p.x, y = base_y + p.y, z = az + p.z;
		editor.set_block_absolute(
				AIR, x, y + 1, z, ground_decoration::stacked_plant_parts(), std::nullopt);
		editor.set_block_absolute(AIR, x, y - 1, z,
				ground_decoration::plant_lower_half_blocks(), std::nullopt);
	}
}
} // namespace

void scatter_region(WorldEditor &editor, const Args &args, const XZBBox &bbox, int min_x,
		int max_x, int min_z, int max_z)
{
	const auto &opts = args.scatter;
	const double rock_density = opts.rocks ? opts.rock_density : 0.0;
	const double bush_density = opts.bushes ? opts.bush_density : 0.0;
	if ((rock_density <= 0 && bush_density <= 0) || !editor.ground ||
			!editor.ground->has_land_cover() || !is_earth(editor.ground->body()) ||
			min_x > max_x || min_z > max_z)
		return;
	const auto root = editor.get_schematic_asset_root();
	if (root.empty())
		return;
	const auto rocks = rock_density > 0 ? load_pool(root, true) : std::vector<SchemPtr>{};
	const auto bushes =
			bush_density > 0 ? load_pool(root, false) : std::vector<SchemPtr>{};
	for (int cx = floor_chunk(min_x); cx <= floor_chunk(max_x); ++cx)
		for (int cz = floor_chunk(min_z); cz <= floor_chunk(max_z); ++cz)
			scatter_one(editor, *editor.ground, args, bbox, min_x, max_x, min_z, max_z,
					cx, cz, rocks, bushes);
}
} // namespace arnis::ground_scatter
