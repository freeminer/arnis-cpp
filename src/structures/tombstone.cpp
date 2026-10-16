#include "tombstone.h"
#include "../assets_root.h"
#include "schem_decoder.h"
#include "../land_cover/land_cover.h"
#include "../floodfill_cache.h"
#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
namespace arnis::structures
{
namespace
{
int wrapping_add(int value, std::uint32_t offset)
{
	return std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(value) + offset);
}
}

namespace tombstone
{
namespace
{
constexpr int SMALL_GRID = 4, LARGE_GRID = 32, LARGE_HALF = 6;

std::filesystem::path asset(const WorldEditor &editor, const std::string &name)
{
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	return root / (name + ".schem");
}
const std::vector<std::shared_ptr<const StructureSchematic>> &variants(
		const WorldEditor &editor, int first, int last)
{
	static std::mutex cache_mutex;
	static std::unordered_map<std::string,
			std::vector<std::shared_ptr<const StructureSchematic>>>
			cache;
	const auto root = editor.get_schematic_asset_root().empty()
							  ? assets::path("structures")
							  : editor.get_schematic_asset_root();
	const auto key = root.lexically_normal().string() + ":" + std::to_string(first) +
					 ":" + std::to_string(last);
	std::lock_guard lock(cache_mutex);
	const auto [it, inserted] = cache.try_emplace(key);
	if (inserted) {
		auto &result = it->second;
		for (int index = first; index <= last; ++index) {
			const auto loaded = try_load_structure_cached(
					root / ("tombstone" + std::to_string(index) + ".schem"));
			if (loaded) {
				// Rust centers every successfully parsed tombstone before choosing from
				// the compacted valid-asset pool.
				result.push_back(
						std::make_shared<const StructureSchematic>(loaded->centered()));
			} else {
				// A broken individual asset is omitted; it must not disable the other
				// variants or change deterministic selection among valid assets.
			}
		}
	}
	return it->second;
}
const StructureSchematic *pick_variant(
		const std::vector<std::shared_ptr<const StructureSchematic>> &models,
		std::uint64_t h)
{
	return models.empty() ? nullptr : models[(h >> 8) % models.size()].get();
}
bool near_large_crypt(int x, int z)
{
	const int gx = static_cast<int>(std::lround(static_cast<double>(x) / LARGE_GRID)) *
				   LARGE_GRID;
	const int gz = static_cast<int>(std::lround(static_cast<double>(z) / LARGE_GRID)) *
				   LARGE_GRID;
	const auto dx = static_cast<long long>(x) - gx;
	const auto dz = static_cast<long long>(z) - gz;
	return land_cover::coord_hash(gx, gz) % 100 < 5 && dx >= -LARGE_HALF &&
		   dx <= LARGE_HALF && dz >= -LARGE_HALF && dz <= LARGE_HALF;
}
}

void maybe_place(WorldEditor &e, int x, int z, const RoadMaskBitmap &road_mask)
{
	if (!e.place_schematics() || x % SMALL_GRID != 0 || z % SMALL_GRID != 0 ||
			e.is_lc_water(x, z) || road_mask.contains(x, z))
		return;
	const auto h = land_cover::coord_hash(x, z);
	if (x % LARGE_GRID == 0 && z % LARGE_GRID == 0 && h % 100 < 5) {
		const auto &models = variants(e, 10, 11);
		if (const auto *schematic = pick_variant(models, h))
			place_structure(
					e, *schematic, x, e.get_absolute_y(x, 0, z), z, (h >> 16) & 3);
		return;
	}
	if (!near_large_crypt(x, z)) {
		// Rust uses wrapping_add for the deterministic offset hash.
		const int hashed_x = wrapping_add(x, 0x5f5fU);
		const int hashed_z = wrapping_add(z, 0x3c3cU);
		const auto small_hash = land_cover::coord_hash(hashed_x, hashed_z);
		if (small_hash % 100 < 28) {
			const auto &models = variants(e, 1, 9);
			if (const auto *schematic = pick_variant(models, small_hash))
				place_structure(e, *schematic, x, e.get_absolute_y(x, 0, z), z,
						(small_hash >> 16) & 3);
		}
	}
}
}

void scatter_tombstones(WorldEditor &e, int minx, int minz, int maxx, int maxz)
{
	if (!e.place_schematics())
		return;
	auto grid_start = [](int value) { return value - ((value % 4) + 4) % 4; };
	for (int z = grid_start(minz); z <= maxz; z += 4)
		for (int x = grid_start(minx); x <= maxx; x += 4) {
			auto h = land_cover::coord_hash(
					wrapping_add(x, 0x5f5fU), wrapping_add(z, 0x3c3cU));
			if (h % 100 >= 28 || e.is_lc_water(x, z))
				continue;
			auto p = tombstone::asset(e, h % 9 ? "tombstone1" : "tombstone10");
			place_schem_file_anchored(e, p, x, e.get_absolute_y(x, 0, z), z,
					(h >> 16) & 3, SchemAnchor::Centered);
		}
}
}
