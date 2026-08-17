#pragma once
#include "../../../arnis_block.h"
#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <optional>
#include <functional>
#include <tuple>
namespace arnis::world_editor
{
inline constexpr int MIN_Y = -64, MIN_SECTION_Y = -4, MAX_Y = 2031, MAX_BLOCK_ID = 512,
					 SECTION_VOLUME = 4096;
struct WorldMetadata
{
	int min_mc_x = 0, max_mc_x = 0, min_mc_z = 0, max_mc_z = 0;
	double min_geo_lat = 0, max_geo_lat = 0, min_geo_lon = 0, max_geo_lon = 0;
	std::string projection = "local";
	double scale = 1.0;
};
struct GenerationBounds
{
	int min_x = 0, min_y = MIN_Y, min_z = 0, max_x = 0, max_y = MAX_Y, max_z = 0;
	bool contains(int x, int y, int z) const
	{
		return x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z &&
			   z <= max_z;
	}
};
inline bool valid_metadata(const WorldMetadata &m)
{
	return m.min_mc_x <= m.max_mc_x && m.min_mc_z <= m.max_mc_z &&
		   m.min_geo_lat <= m.max_geo_lat && m.min_geo_lon <= m.max_geo_lon &&
		   m.scale > 0.0;
}
inline bool intersects(const GenerationBounds &a, const GenerationBounds &b)
{
	return a.max_x >= b.min_x && a.min_x <= b.max_x && a.max_y >= b.min_y &&
		   a.min_y <= b.max_y && a.max_z >= b.min_z && a.min_z <= b.max_z;
}
inline GenerationBounds clamp_bounds(GenerationBounds b)
{
	b.min_y = std::clamp(b.min_y, MIN_Y, MAX_Y);
	b.max_y = std::clamp(b.max_y, MIN_Y, MAX_Y);
	return b;
}
inline WorldMetadata merge_metadata(const WorldMetadata &a, const WorldMetadata &b)
{
	WorldMetadata m = a;
	m.min_mc_x = std::min(a.min_mc_x, b.min_mc_x);
	m.max_mc_x = std::max(a.max_mc_x, b.max_mc_x);
	m.min_mc_z = std::min(a.min_mc_z, b.min_mc_z);
	m.max_mc_z = std::max(a.max_mc_z, b.max_mc_z);
	m.min_geo_lat = std::min(a.min_geo_lat, b.min_geo_lat);
	m.max_geo_lat = std::max(a.max_geo_lat, b.max_geo_lat);
	m.min_geo_lon = std::min(a.min_geo_lon, b.min_geo_lon);
	m.max_geo_lon = std::max(a.max_geo_lon, b.max_geo_lon);
	return m;
}
inline GenerationBounds tile_bounds(
		int tile_x, int tile_z, int tile_size, int min_y = MIN_Y, int max_y = MAX_Y)
{
	return clamp_bounds({tile_x * tile_size, min_y, tile_z * tile_size,
			(tile_x + 1) * tile_size - 1, max_y, (tile_z + 1) * tile_size - 1});
}
inline GenerationBounds expand_bounds(GenerationBounds b, int halo)
{
	b.min_x -= halo;
	b.min_z -= halo;
	b.max_x += halo;
	b.max_z += halo;
	return b;
}
inline bool compatible_metadata(const WorldMetadata &a, const WorldMetadata &b)
{
	return a.projection == b.projection && std::abs(a.scale - b.scale) <= 1e-12;
}
inline WorldMetadata normalize_metadata(WorldMetadata m)
{
	if (m.projection.empty())
		m.projection = "local";
	if (m.scale <= 0.0)
		m.scale = 1.0;
	if (m.min_mc_x > m.max_mc_x)
		std::swap(m.min_mc_x, m.max_mc_x);
	if (m.min_mc_z > m.max_mc_z)
		std::swap(m.min_mc_z, m.max_mc_z);
	if (m.min_geo_lat > m.max_geo_lat)
		std::swap(m.min_geo_lat, m.max_geo_lat);
	if (m.min_geo_lon > m.max_geo_lon)
		std::swap(m.min_geo_lon, m.max_geo_lon);
	return m;
}
inline std::uint64_t bounds_volume(const GenerationBounds &b)
{
	if (!b.contains(b.min_x, b.min_y, b.min_z))
		return 0;
	return std::uint64_t(b.max_x - b.min_x + 1) * std::uint64_t(b.max_y - b.min_y + 1) *
		   std::uint64_t(b.max_z - b.min_z + 1);
}
inline std::pair<int, int> tile_grid_size(const GenerationBounds &b, int tile_size)
{
	if (tile_size <= 0)
		return {0, 0};
	return {(b.max_x - b.min_x + tile_size) / tile_size,
			(b.max_z - b.min_z + tile_size) / tile_size};
}
inline int floor_div(int value, int divisor)
{
	if (divisor <= 0)
		return 0;
	int q = value / divisor, r = value % divisor;
	return r < 0 ? q - 1 : q;
}
inline std::pair<int, int> tile_index_for(
		int x, int z, int origin_x, int origin_z, int tile_size)
{
	if (tile_size <= 0)
		return {0, 0};
	return {floor_div(x - origin_x, tile_size), floor_div(z - origin_z, tile_size)};
}
inline std::pair<int, int> tile_origin(
		int tile_x, int tile_z, int origin_x, int origin_z, int tile_size)
{
	return {origin_x + tile_x * tile_size, origin_z + tile_z * tile_size};
}
inline std::vector<std::pair<int, int>> enumerate_tiles(
		const GenerationBounds &b, int tile_size)
{
	std::vector<std::pair<int, int>> out;
	auto [nx, nz] = tile_grid_size(b, tile_size);
	for (int z = 0; z < nz; ++z)
		for (int x = 0; x < nx; ++x)
			out.emplace_back(x, z);
	return out;
}
inline std::vector<std::pair<int, int>> enumerate_tiles_world(
		const GenerationBounds &b, int tile_size)
{
	std::vector<std::pair<int, int>> out;
	if (tile_size <= 0)
		return out;
	int x0 = floor_div(b.min_x, tile_size), x1 = floor_div(b.max_x, tile_size),
		z0 = floor_div(b.min_z, tile_size), z1 = floor_div(b.max_z, tile_size);
	for (int z = z0; z <= z1; ++z)
		for (int x = x0; x <= x1; ++x)
			out.emplace_back(x, z);
	return out;
}
inline std::pair<int, int> tile_region(int tile_x, int tile_z, int tile_size)
{
	if (tile_size <= 0)
		return {0, 0};
	return {floor_div(tile_x * tile_size, 512), floor_div(tile_z * tile_size, 512)};
}
inline bool tile_authoritative(int x, int z, const GenerationBounds &b, int tile_size)
{
	return x >= b.min_x && x <= b.max_x && z >= b.min_z && z <= b.max_z;
}
inline std::vector<std::pair<int, int>> tiles_in_region(
		int rx, int rz, const GenerationBounds &b, int tile_size)
{
	std::vector<std::pair<int, int>> out;
	if (tile_size <= 0)
		return out;
	for (auto t : enumerate_tiles_world(b, tile_size))
		if (tile_region(t.first, t.second, tile_size) == std::pair<int, int>{rx, rz})
			out.push_back(t);
	return out;
}
inline bool tile_fully_authoritative(
		int tx, int tz, const GenerationBounds &b, int tile_size)
{
	auto t = clipped_tile_bounds(b, tx, tz, tile_size);
	return t.min_x == tx * tile_size && t.max_x == (tx + 1) * tile_size - 1 &&
		   t.min_z == tz * tile_size && t.max_z == (tz + 1) * tile_size - 1;
}
enum class TileOwnership
{
	Outside,
	Partial,
	Full
};
struct TilePlan
{
	int x = 0, z = 0;
	GenerationBounds bounds{};
	TileOwnership ownership = TileOwnership::Outside;
};
struct TileCompletion
{
	int x = 0, z = 0;
	bool complete = false;
	std::size_t blocks = 0;
};
inline TileOwnership classify_tile(
		int tx, int tz, const GenerationBounds &b, int tile_size)
{
	auto raw = tile_bounds(tx, tz, tile_size, b.min_y, b.max_y);
	if (!intersects(raw, b))
		return TileOwnership::Outside;
	return tile_fully_authoritative(tx, tz, b, tile_size) ? TileOwnership::Full
														  : TileOwnership::Partial;
}
inline std::vector<std::pair<int, int>> tiles_for_merge(
		const GenerationBounds &b, int tile_size, bool include_partial = true)
{
	std::vector<std::pair<int, int>> out;
	for (auto t : enumerate_tiles_world(b, tile_size)) {
		auto k = classify_tile(t.first, t.second, b, tile_size);
		if (k == TileOwnership::Full || (include_partial && k == TileOwnership::Partial))
			out.push_back(t);
	}
	return out;
}
inline std::vector<TilePlan> build_tile_plan(
		const GenerationBounds &b, int tile_size, int halo = 0)
{
	std::vector<TilePlan> out;
	for (auto [x, z] : enumerate_tiles_world(b, tile_size)) {
		auto own = classify_tile(x, z, b, tile_size);
		if (own == TileOwnership::Outside)
			continue;
		out.push_back({x, z, halo_tile_bounds(b, x, z, tile_size, halo), own});
	}
	return out;
}
inline std::vector<TilePlan> filter_tile_plan(
		const std::vector<TilePlan> &plan, TileOwnership ownership)
{
	std::vector<TilePlan> out;
	for (const auto &t : plan)
		if (t.ownership == ownership)
			out.push_back(t);
	return out;
}
inline std::uint64_t tile_plan_volume(const std::vector<TilePlan> &plan)
{
	std::uint64_t n = 0;
	for (const auto &t : plan)
		n += bounds_volume(t.bounds);
	return n;
}
inline void sort_tile_plan(std::vector<TilePlan> &plan)
{
	std::sort(plan.begin(), plan.end(), [](const TilePlan &a, const TilePlan &b) {
		return a.z == b.z ? a.x < b.x : a.z < b.z;
	});
}
inline std::vector<TilePlan> tiles_plan_for_region(
		const std::vector<TilePlan> &plan, int rx, int rz, int tile_size)
{
	std::vector<TilePlan> out;
	for (const auto &t : plan)
		if (tile_region(t.x, t.z, tile_size) == std::pair<int, int>{rx, rz})
			out.push_back(t);
	return out;
}
inline void deduplicate_tile_plan(std::vector<TilePlan> &plan)
{
	std::sort(plan.begin(), plan.end(), [](const TilePlan &a, const TilePlan &b) {
		return a.z == b.z ? a.x < b.x : a.z < b.z;
	});
	plan.erase(std::unique(plan.begin(), plan.end(),
					   [](const TilePlan &a, const TilePlan &b) {
						   return a.x == b.x && a.z == b.z;
					   }),
			plan.end());
}
inline bool validate_tile_plan(
		const std::vector<TilePlan> &plan, const GenerationBounds &b, int tile_size)
{
	for (const auto &t : plan) {
		if (t.ownership == TileOwnership::Outside)
			return false;
		if (classify_tile(t.x, t.z, b, tile_size) != t.ownership)
			return false;
	}
	return true;
}
inline std::uint64_t tile_plan_hash(const std::vector<TilePlan> &plan)
{
	std::uint64_t h = 1469598103934665603ULL;
	for (const auto &t : plan) {
		h ^= static_cast<std::uint32_t>(t.x);
		h *= 1099511628211ULL;
		h ^= static_cast<std::uint32_t>(t.z);
		h *= 1099511628211ULL;
		h ^= static_cast<std::uint8_t>(t.ownership);
		h *= 1099511628211ULL;
		h ^= bounds_volume(t.bounds);
		h *= 1099511628211ULL;
	}
	return h;
}
inline std::vector<TilePlan> changed_tile_plan(
		const std::vector<TilePlan> &old_plan, const std::vector<TilePlan> &new_plan)
{
	std::vector<TilePlan> out;
	for (const auto &n : new_plan) {
		auto it = std::find_if(old_plan.begin(), old_plan.end(),
				[&](const TilePlan &o) { return o.x == n.x && o.z == n.z; });
		if (it == old_plan.end() || it->ownership != n.ownership ||
				it->bounds.min_x != n.bounds.min_x ||
				it->bounds.max_x != n.bounds.max_x ||
				it->bounds.min_z != n.bounds.min_z || it->bounds.max_z != n.bounds.max_z)
			out.push_back(n);
	}
	return out;
}
inline std::vector<std::pair<int, int>> removed_tiles(
		const std::vector<TilePlan> &old_plan, const std::vector<TilePlan> &new_plan)
{
	std::vector<std::pair<int, int>> out;
	for (const auto &o : old_plan)
		if (std::none_of(new_plan.begin(), new_plan.end(),
					[&](const TilePlan &n) { return n.x == o.x && n.z == o.z; }))
			out.emplace_back(o.x, o.z);
	return out;
}
inline void reconcile_tile_plan(
		std::vector<TilePlan> &current, const std::vector<TilePlan> &next)
{
	current = next;
	deduplicate_tile_plan(current);
	sort_tile_plan(current);
}
inline GenerationBounds clipped_tile_bounds(
		const GenerationBounds &b, int tile_x, int tile_z, int tile_size)
{
	GenerationBounds t = tile_bounds(tile_x, tile_z, tile_size, b.min_y, b.max_y);
	t.min_x = std::max(t.min_x, b.min_x);
	t.max_x = std::min(t.max_x, b.max_x);
	t.min_z = std::max(t.min_z, b.min_z);
	t.max_z = std::min(t.max_z, b.max_z);
	return t;
}
inline GenerationBounds halo_tile_bounds(
		const GenerationBounds &b, int tile_x, int tile_z, int tile_size, int halo)
{
	return expand_bounds(clipped_tile_bounds(b, tile_x, tile_z, tile_size), halo);
}
inline bool tile_owns(
		int x, int z, const GenerationBounds &b, int tile_x, int tile_z, int tile_size)
{
	return clipped_tile_bounds(b, tile_x, tile_z, tile_size).contains(x, b.min_y, z);
}
class BlockStorage
{
	bool is_uniform_ = true;
	Block uniform_block_{};
	std::vector<std::uint16_t> blocks_;

public:
	BlockStorage() = default;
	explicit BlockStorage(Block b) : uniform_block_(b) {}
	Block get(std::size_t i) const
	{
		return is_uniform_ ? uniform_block_ : Block(static_cast<content_t>(blocks_[i]));
	}
	void set(std::size_t i, Block b)
	{
		if (is_uniform_ && uniform_block_.id() == b.id())
			return;
		if (is_uniform_) {
			blocks_.assign(SECTION_VOLUME, uniform_block_.id());
			is_uniform_ = false;
		}
		blocks_[i] = b.id();
	}
	bool uniform() const { return is_uniform_; }
	std::size_t size() const { return is_uniform_ ? SECTION_VOLUME : blocks_.size(); }
	void fill(Block b)
	{
		is_uniform_ = true;
		uniform_block_ = b;
		blocks_.clear();
	}
	const std::vector<std::uint16_t> &raw() const { return blocks_; }
	std::vector<Block> materialize() const
	{
		std::vector<Block> out;
		out.reserve(SECTION_VOLUME);
		if (is_uniform_) {
			out.assign(SECTION_VOLUME, uniform_block_);
		} else
			for (auto id : blocks_)
				out.emplace_back(static_cast<content_t>(id));
		return out;
	}
	void compact()
	{
		if (is_uniform_ || blocks_.empty())
			return;
		const auto first = blocks_.front();
		for (auto id : blocks_)
			if (id != first)
				return;
		uniform_block_ = Block(static_cast<content_t>(first));
		is_uniform_ = true;
		blocks_.clear();
	}
};
struct PaletteItem
{
	std::string name;
};
struct PackedPalette
{
	std::vector<PaletteItem> palette;
	std::vector<std::uint64_t> data;
	std::size_t bits_per_block = 4;
};
inline std::size_t section_index(int x, int y, int z)
{
	return (static_cast<std::size_t>(y & 15) << 8) |
		   (static_cast<std::size_t>(z & 15) << 4) | static_cast<std::size_t>(x & 15);
}
inline std::array<int, 3> index_coords(std::size_t i)
{
	return {static_cast<int>(i & 15), static_cast<int>((i >> 8) & 15),
			static_cast<int>((i >> 4) & 15)};
}
inline int section_world_y(int sy, int local_y)
{
	return (sy << 4) + (local_y & 15);
}
struct Section
{
	std::int8_t y = 0;
	BlockStorage storage;
};
struct SectionToModify
{
	BlockStorage storage;
	Block get_block(int x, int y, int z) const
	{
		auto b = storage.get(section_index(x, y, z));
		return b.id() == block_definitions::AIR.id() ? Block{} : b;
	}
	void set_block(int x, int y, int z, Block b)
	{
		storage.set(section_index(x, y, z), b);
	}
	Block get_block_at_index(std::size_t i) const { return storage.get(i); }
	void compact() { storage.compact(); }
	void merge_non_air(const SectionToModify &other, bool overwrite)
	{
		auto src = other.storage.materialize();
		for (std::size_t i = 0; i < src.size(); ++i)
			if (src[i].id() != block_definitions::AIR.id() &&
					(overwrite || storage.get(i).id() == block_definitions::AIR.id()))
				storage.set(i, src[i]);
	}
	void merge_air_only(const SectionToModify &other) { merge_non_air(other, false); }
	void fill(Block b) { storage.fill(b); }
	Block get_index(std::size_t i) const { return storage.get(i); }
	void set_index(std::size_t i, Block b)
	{
		if (i < SECTION_VOLUME)
			storage.set(i, b);
	}
	std::size_t replace(Block from, Block to)
	{
		std::size_t n = 0;
		for (std::size_t i = 0; i < SECTION_VOLUME; ++i)
			if (storage.get(i).id() == from.id()) {
				storage.set(i, to);
				++n;
			}
		return n;
	}
	std::size_t replace_if(const std::function<bool(Block)> &predicate, Block to)
	{
		std::size_t n = 0;
		for (std::size_t i = 0; i < SECTION_VOLUME; ++i) {
			auto b = storage.get(i);
			if (predicate(b)) {
				storage.set(i, to);
				++n;
			}
		}
		return n;
	}
	std::size_t fill_air(Block b)
	{
		return replace_if(
				[](Block x) { return x.id() == block_definitions::AIR.id(); }, b);
	}
	bool occupied() const
	{
		for (const auto &b : storage.materialize())
			if (b.id() != block_definitions::AIR.id())
				return true;
		return false;
	}
};
struct ChunkToModify
{
	std::unordered_map<int, SectionToModify> sections;
	SectionToModify &section(int y) { return sections[static_cast<int>(y >> 4)]; }
	const SectionToModify *find_section(int y) const
	{
		auto it = sections.find(static_cast<int>(y >> 4));
		return it == sections.end() ? nullptr : &it->second;
	}
	Block get_block(int x, int y, int z) const
	{
		y = std::clamp(y, MIN_Y, MAX_Y);
		auto *s = find_section(y);
		return s ? s->get_block(x & 15, y & 15, z & 15) : Block{};
	}
	void set_block(int x, int y, int z, Block b)
	{
		y = std::clamp(y, MIN_Y, MAX_Y);
		section(y).set_block(x & 15, y & 15, z & 15, b);
	}
	void prune_empty_sections()
	{
		for (auto it = sections.begin(); it != sections.end();) {
			auto &s = it->second;
			s.compact();
			if (s.storage.uniform() &&
					s.storage.get(0).id() == block_definitions::AIR.id())
				it = sections.erase(it);
			else
				++it;
		}
	}
	bool empty() const { return sections.empty(); }
};
struct RegionToModify
{
	std::unordered_map<std::uint64_t, ChunkToModify> chunks;
	static std::uint64_t key(int x, int z)
	{
		return (std::uint64_t(static_cast<std::uint32_t>(x)) << 32) | std::uint32_t(z);
	}
	ChunkToModify &get_or_create_chunk(int x, int z) { return chunks[key(x, z)]; }
	const ChunkToModify *get_chunk(int x, int z) const
	{
		auto it = chunks.find(key(x, z));
		return it == chunks.end() ? nullptr : &it->second;
	}
	void clear() { chunks.clear(); }
	std::size_t chunk_count() const { return chunks.size(); }
	void prune_empty_chunks()
	{
		for (auto it = chunks.begin(); it != chunks.end();) {
			it->second.prune_empty_sections();
			if (it->second.empty())
				it = chunks.erase(it);
			else
				++it;
		}
	}
	bool empty() const { return chunks.empty(); }
};
struct WorldToModify
{
	enum class PlacementMode
	{
		Overwrite,
		OnlyAir,
		OnlyNonAir
	};
	struct Stats
	{
		std::size_t regions = 0, chunks = 0, sections = 0, non_air = 0;
	};
	std::unordered_map<std::uint64_t, RegionToModify> regions;
	RegionToModify &get_or_create_region(int x, int z)
	{
		return regions[RegionToModify::key(x, z)];
	}
	const RegionToModify *get_region(int x, int z) const
	{
		auto it = regions.find(RegionToModify::key(x, z));
		return it == regions.end() ? nullptr : &it->second;
	}
	std::optional<Block> get_block(int x, int y, int z) const
	{
		int cx = x >> 4, cz = z >> 4;
		auto *r = get_region(cx >> 5, cz >> 5);
		if (!r)
			return std::nullopt;
		auto *c = r->get_chunk(cx & 31, cz & 31);
		if (!c)
			return std::nullopt;
		return c->get_block(x & 15, y, z & 15);
	}
	void set_block(int x, int y, int z, Block b)
	{
		int cx = x >> 4, cz = z >> 4;
		get_or_create_region(cx >> 5, cz >> 5)
				.get_or_create_chunk(cx & 31, cz & 31)
				.set_block(x & 15, y, z & 15, b);
	}
	void set_block_if_absent(int x, int y, int z, Block b)
	{
		auto old = get_block(x, y, z);
		if (!old || old->id() == block_definitions::AIR.id())
			set_block(x, y, z, b);
	}
	void fill_column(
			int x, int z, int y_min, int y_max, Block b, bool skip_existing = false)
	{
		y_min = std::clamp(y_min, MIN_Y, MAX_Y);
		y_max = std::clamp(y_max, MIN_Y, MAX_Y);
		if (y_min > y_max)
			return;
		for (int y = y_min; y <= y_max; ++y) {
			if (skip_existing)
				set_block_if_absent(x, y, z, b);
			else
				set_block(x, y, z, b);
		}
	}
	bool bulk_fill_chunk_sections_below(
			int chunk_x, int chunk_z, int section_y_max, Block b)
	{
		if (section_y_max < MIN_SECTION_Y)
			return true;
		auto &c = get_or_create_region(chunk_x >> 5, chunk_z >> 5)
						  .get_or_create_chunk(chunk_x & 31, chunk_z & 31);
		bool clean = true;
		for (int sy = MIN_SECTION_Y; sy <= section_y_max; ++sy) {
			auto &s = c.sections[sy];
			bool empty = s.storage.uniform() &&
						 s.storage.get(0).id() == block_definitions::AIR.id();
			if (empty)
				s.storage.fill(b);
			else
				clean = false;
		}
		return clean;
	}
	std::uint64_t content_hash() const
	{
		std::uint64_t h = 1469598103934665603ULL;
		for (const auto &rk : regions) {
			h ^= rk.first;
			h *= 1099511628211ULL;
			for (const auto &ck : rk.second.chunks) {
				h ^= ck.first;
				h *= 1099511628211ULL;
				for (const auto &sk : ck.second.sections) {
					h ^= sk.first;
					h *= 1099511628211ULL;
					for (const auto &b : sk.second.storage.materialize()) {
						h ^= b.id();
						h *= 1099511628211ULL;
					}
				}
			}
		}
		return h;
	}
	std::uint64_t region_content_hash(int rx, int rz) const
	{
		auto *r = get_region(rx, rz);
		if (!r)
			return 0;
		std::uint64_t h = 1469598103934665603ULL;
		h ^= static_cast<std::uint32_t>(rx);
		h *= 1099511628211ULL;
		h ^= static_cast<std::uint32_t>(rz);
		h *= 1099511628211ULL;
		for (const auto &ck : r->chunks) {
			h ^= ck.first;
			h *= 1099511628211ULL;
			for (const auto &sk : ck.second.sections) {
				h ^= sk.first;
				h *= 1099511628211ULL;
				for (const auto &b : sk.second.storage.materialize()) {
					h ^= b.id();
					h *= 1099511628211ULL;
				}
			}
		}
		return h;
	}
	void compact()
	{
		for (auto &rk : regions)
			for (auto &ck : rk.second.chunks)
				for (auto &sk : ck.second.sections)
					sk.second.compact();
	}
	void prune_empty()
	{
		for (auto it = regions.begin(); it != regions.end();) {
			it->second.prune_empty_chunks();
			if (it->second.empty())
				it = regions.erase(it);
			else
				++it;
		}
	}
	std::array<int, 6> occupied_bounds() const
	{
		std::array<int, 6> b{0, 0, 0, 0, 0, 0};
		bool found = false;
		for_each_non_air([&](int x, int y, int z, Block) {
			if (!found) {
				b = {x, y, z, x, y, z};
				found = true;
			} else {
				b[0] = std::min(b[0], x);
				b[1] = std::min(b[1], y);
				b[2] = std::min(b[2], z);
				b[3] = std::max(b[3], x);
				b[4] = std::max(b[4], y);
				b[5] = std::max(b[5], z);
			}
		});
		return b;
	}
	void fill_chunk(int chunk_x, int chunk_z, int min_y, int max_y, Block b)
	{
		min_y = std::clamp(min_y, MIN_Y, MAX_Y);
		max_y = std::clamp(max_y, MIN_Y, MAX_Y);
		if (min_y > max_y)
			return;
		for (int z = 0; z < 16; ++z)
			for (int x = 0; x < 16; ++x)
				fill_column(
						(chunk_x << 4) + x, (chunk_z << 4) + z, min_y, max_y, b, false);
	}
	void fill_box(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z,
			Block b, bool skip_existing = false)
	{
		min_y = std::clamp(min_y, MIN_Y, MAX_Y);
		max_y = std::clamp(max_y, MIN_Y, MAX_Y);
		if (min_x > max_x || min_y > max_y || min_z > max_z)
			return;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				fill_column(x, z, min_y, max_y, b, skip_existing);
	}
	void set_box_if_empty(
			int min_x, int min_y, int min_z, int max_x, int max_y, int max_z, Block b)
	{
		fill_box(min_x, min_y, min_z, max_x, max_y, max_z, b, true);
	}
	std::size_t replace_block(Block from, Block to)
	{
		std::size_t n = 0;
		for (auto &rk : regions)
			for (auto &ck : rk.second.chunks)
				for (auto &sk : ck.second.sections) {
					auto vals = sk.second.storage.materialize();
					for (std::size_t i = 0; i < vals.size(); ++i)
						if (vals[i].id() == from.id()) {
							sk.second.storage.set(i, to);
							++n;
						}
				}
		return n;
	}
	std::size_t count_non_air_box(
			int min_x, int min_y, int min_z, int max_x, int max_y, int max_z) const
	{
		std::size_t n = 0;
		min_y = std::clamp(min_y, MIN_Y, MAX_Y);
		max_y = std::clamp(max_y, MIN_Y, MAX_Y);
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = min_y; y <= max_y; ++y) {
					auto b = get_block(x, y, z);
					if (b && b->id() != block_definitions::AIR.id())
						++n;
				}
		return n;
	}
	void clear_box(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z)
	{
		fill_box(min_x, min_y, min_z, max_x, max_y, max_z, block_definitions::AIR, false);
		prune_empty();
	}
	std::vector<int> column_heights(
			int min_x, int min_z, int max_x, int max_z, int min_y, int max_y) const
	{
		std::vector<int> out;
		min_y = std::clamp(min_y, MIN_Y, MAX_Y);
		max_y = std::clamp(max_y, MIN_Y, MAX_Y);
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x) {
				int h = min_y - 1;
				for (int y = max_y; y >= min_y; --y) {
					auto b = get_block(x, y, z);
					if (b && b->id() != block_definitions::AIR.id()) {
						h = y;
						break;
					}
				}
				out.push_back(h);
			}
		return out;
	}
	WorldToModify extract_region(int rx, int rz) const
	{
		WorldToModify out;
		auto *r = get_region(rx, rz);
		if (!r)
			return out;
		out.regions.emplace(RegionToModify::key(rx, rz), *r);
		return out;
	}
	void insert_region(
			int rx, int rz, const RegionToModify &region, bool overwrite = true)
	{
		auto &dst = get_or_create_region(rx, rz);
		for (const auto &ck : region.chunks) {
			auto &dc = dst.chunks[ck.first];
			for (const auto &sk : ck.second.sections) {
				auto &ds = dc.sections[sk.first];
				if (overwrite)
					ds.storage = sk.second.storage;
				else
					ds.merge_air_only(sk.second);
			}
		}
	}
	bool transfer_region_to(WorldToModify &dst, int rx, int rz, bool erase_after = true)
	{
		auto it = regions.find(RegionToModify::key(rx, rz));
		if (it == regions.end())
			return false;
		dst.insert_region(rx, rz, it->second, true);
		if (erase_after)
			regions.erase(it);
		return true;
	}
	std::vector<std::pair<int, int>> occupied_regions(
			int min_x, int min_z, int max_x, int max_z) const
	{
		auto all = region_keys();
		std::vector<std::pair<int, int>> out;
		for (auto [rx, rz] : all) {
			int x0 = rx << 9, z0 = rz << 9;
			if (x0 + 511 >= min_x && x0 <= max_x && z0 + 511 >= min_z && z0 <= max_z)
				out.emplace_back(rx, rz);
		}
		return out;
	}
	bool intersects_region(
			int rx, int rz, int min_x, int min_z, int max_x, int max_z) const
	{
		int x0 = rx << 9, z0 = rz << 9;
		return x0 + 511 >= min_x && x0 <= max_x && z0 + 511 >= min_z && z0 <= max_z;
	}
	bool occupied_at(int x, int y, int z) const
	{
		auto b = get_block(x, y, z);
		return b && b->id() != block_definitions::AIR.id();
	}
	std::vector<std::tuple<int, int, int>> chunk_keys() const
	{
		std::vector<std::tuple<int, int, int>> out;
		for (const auto &rk : regions) {
			int rx = key_x(rk.first), rz = key_z(rk.first);
			for (const auto &ck : rk.second.chunks) {
				int kx = key_x(ck.first), kz = key_z(ck.first);
				out.emplace_back((rx << 5) + kx, (rz << 5) + kz,
						static_cast<int>(ck.second.sections.size()));
			}
		}
		std::sort(out.begin(), out.end());
		return out;
	}
	void merge_intersecting(WorldToModify &&other, int min_x, int min_z, int max_x,
			int max_z, bool erase_outside = false)
	{
		for (auto it = other.regions.begin(); it != other.regions.end();) {
			int rx = key_x(it->first), rz = key_z(it->first);
			int x0 = rx << 9, z0 = rz << 9;
			bool hit =
					x0 + 511 >= min_x && x0 <= max_x && z0 + 511 >= min_z && z0 <= max_z;
			if (hit) {
				insert_region(rx, rz, it->second, true);
				it = other.regions.erase(it);
			} else if (erase_outside)
				it = other.regions.erase(it);
			else
				++it;
		}
	}
	std::size_t block_count() const
	{
		std::size_t n = 0;
		for (const auto &rk : regions)
			for (const auto &ck : rk.second.chunks)
				for (const auto &sk : ck.second.sections)
					n += sk.second.storage.size();
		return n;
	}
	std::size_t estimated_bytes() const
	{
		std::size_t n = 0;
		for (const auto &rk : regions)
			for (const auto &ck : rk.second.chunks)
				for (const auto &sk : ck.second.sections)
					n += sizeof(SectionToModify) +
						 sk.second.storage.size() * sizeof(std::uint16_t);
		return n;
	}
	std::optional<std::pair<int, int>> smallest_region() const
	{
		if (regions.empty())
			return std::nullopt;
		auto it = regions.begin();
		std::size_t best = SIZE_MAX;
		std::pair<int, int> key{key_x(it->first), key_z(it->first)};
		for (const auto &r : regions) {
			std::size_t n = 0;
			for (const auto &c : r.second.chunks)
				for (const auto &s : c.second.sections)
					n += s.second.storage.size();
			if (n < best) {
				best = n;
				key = {key_x(r.first), key_z(r.first)};
			}
		}
		return key;
	}
	std::size_t evict_until(std::size_t target_bytes,
			const std::function<void(int, int, const RegionToModify &)> &flush)
	{
		std::size_t removed = 0;
		while (estimated_bytes() > target_bytes && !regions.empty()) {
			auto key = smallest_region();
			if (!key)
				break;
			auto it = regions.find(RegionToModify::key(key->first, key->second));
			if (it == regions.end())
				break;
			flush(key->first, key->second, it->second);
			removed += it->second.chunk_count();
			regions.erase(it);
		}
		return removed;
	}
	std::size_t evict_farthest(std::size_t target_bytes, int center_rx, int center_rz,
			const std::function<void(int, int, const RegionToModify &)> &flush)
	{
		std::size_t removed = 0;
		while (estimated_bytes() > target_bytes && !regions.empty()) {
			auto it = regions.begin();
			long long best = -1;
			for (auto jt = regions.begin(); jt != regions.end(); ++jt) {
				long long dx = key_x(jt->first) - center_rx,
						  dz = key_z(jt->first) - center_rz, d = dx * dx + dz * dz;
				if (d > best) {
					best = d;
					it = jt;
				}
			}
			int rx = key_x(it->first), rz = key_z(it->first);
			flush(rx, rz, it->second);
			removed += it->second.chunk_count();
			regions.erase(it);
		}
		return removed;
	}
	WorldToModify snapshot() const { return *this; }
	void restore(WorldToModify snapshot_state) { regions.swap(snapshot_state.regions); }
	void swap_state(WorldToModify &other) noexcept { regions.swap(other.regions); }
	std::uint64_t section_hash(const SectionToModify &s) const
	{
		std::uint64_t h = 1469598103934665603ULL;
		for (const auto &b : s.storage.materialize()) {
			h ^= b.id();
			h *= 1099511628211ULL;
		}
		return h;
	}
	std::uint64_t chunk_hash(const ChunkToModify &c) const
	{
		std::uint64_t h = 1469598103934665603ULL;
		for (const auto &s : c.sections) {
			h ^= static_cast<std::uint8_t>(s.first);
			h *= 1099511628211ULL;
			h ^= section_hash(s.second);
			h *= 1099511628211ULL;
		}
		return h;
	}
	std::vector<std::pair<int, int>> changed_regions(const WorldToModify &other) const
	{
		std::vector<std::pair<int, int>> out;
		auto keys = region_keys();
		auto ok = other.region_keys();
		keys.insert(keys.end(), ok.begin(), ok.end());
		std::sort(keys.begin(), keys.end());
		keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
		for (auto [rx, rz] : keys)
			if (region_content_hash(rx, rz) != other.region_content_hash(rx, rz))
				out.emplace_back(rx, rz);
		return out;
	}
	void apply_region_snapshot(const WorldToModify &snapshot_state,
			const std::vector<std::pair<int, int>> &keys)
	{
		for (auto [rx, rz] : keys) {
			erase_region(rx, rz);
			if (auto *r = snapshot_state.get_region(rx, rz))
				insert_region(rx, rz, *r, true);
		}
	}
	void clear_regions(const std::vector<std::pair<int, int>> &keys)
	{
		for (auto [rx, rz] : keys)
			erase_region(rx, rz);
	}
	void merge_changed(const WorldToModify &other)
	{
		for (auto [rx, rz] : changed_regions(other)) {
			if (auto *r = other.get_region(rx, rz))
				insert_region(rx, rz, *r, true);
			else
				erase_region(rx, rz);
		}
	}
	std::vector<std::tuple<int, int, std::size_t>> region_sizes() const
	{
		std::vector<std::tuple<int, int, std::size_t>> out;
		for (const auto &r : regions) {
			std::size_t n = 0;
			for (const auto &c : r.second.chunks)
				for (const auto &s : c.second.sections)
					n += s.second.storage.size() * sizeof(std::uint16_t);
			out.emplace_back(key_x(r.first), key_z(r.first), n);
		}
		std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
			return std::get<2>(a) > std::get<2>(b);
		});
		return out;
	}
	std::vector<std::pair<int, int>> select_regions(std::size_t max_count) const
	{
		auto sizes = region_sizes();
		if (sizes.size() > max_count)
			sizes.resize(max_count);
		std::vector<std::pair<int, int>> out;
		for (auto &e : sizes)
			out.emplace_back(std::get<0>(e), std::get<1>(e));
		return out;
	}
	void for_each_region_mut(const std::function<void(int, int, RegionToModify &)> &fn)
	{
		for (auto &r : regions)
			fn(key_x(r.first), key_z(r.first), r.second);
	}
	void for_each_section_mut(
			const std::function<void(int, int, int, SectionToModify &)> &fn)
	{
		for (auto &r : regions) {
			int rx = key_x(r.first), rz = key_z(r.first);
			for (auto &c : r.second.chunks) {
				int cx = (rx << 5) + key_x(c.first), cz = (rz << 5) + key_z(c.first);
				for (auto &s : c.second.sections)
					fn(cx, cz, s.first, s.second);
			}
		}
	}
	void for_each_section_range_mut(int min_x, int min_z, int max_x, int max_z,
			const std::function<void(int, int, int, SectionToModify &)> &fn)
	{
		for (auto &r : regions) {
			int rx = key_x(r.first), rz = key_z(r.first);
			if (!intersects_region(rx, rz, min_x, min_z, max_x, max_z))
				continue;
			for (auto &c : r.second.chunks) {
				int cx = (rx << 5) + key_x(c.first), cz = (rz << 5) + key_z(c.first);
				if ((cx << 4) + 15 < min_x || (cx << 4) > max_x ||
						(cz << 4) + 15 < min_z || (cz << 4) > max_z)
					continue;
				for (auto &s : c.second.sections)
					fn(cx, cz, s.first, s.second);
			}
		}
	}
	void clear_sections_below(int section_y_max)
	{
		for (auto &r : regions)
			for (auto &c : r.second.chunks)
				for (auto it = c.second.sections.begin(); it != c.second.sections.end();)
					if (it->first <= section_y_max)
						it = c.second.sections.erase(it);
					else
						++it;
		prune_empty();
	}
	void clear_sections_above(int section_y_min)
	{
		for (auto &r : regions)
			for (auto &c : r.second.chunks)
				for (auto it = c.second.sections.begin(); it != c.second.sections.end();)
					if (it->first >= section_y_min)
						it = c.second.sections.erase(it);
					else
						++it;
		prune_empty();
	}
	void normalize_range(int min_x, int min_z, int max_x, int max_z)
	{
		for_each_section_range_mut(min_x, min_z, max_x, max_z,
				[](int, int, int, SectionToModify &s) { s.compact(); });
		prune_empty();
	}
	std::size_t replace_box(int min_x, int min_y, int min_z, int max_x, int max_y,
			int max_z, Block from, Block to)
	{
		std::size_t n = 0;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = get_block(x, y, z);
					if (b && b->id() == from.id()) {
						set_block(x, y, z, to);
						++n;
					}
				}
		return n;
	}
	void transform_box(int min_x, int min_y, int min_z, int max_x, int max_y, int max_z,
			const std::function<Block(Block)> &fn)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = get_block(x, y, z);
					if (b)
						set_block(x, y, z, fn(*b));
				}
		prune_empty();
	}
	void copy_box(const WorldToModify &src, int min_x, int min_y, int min_z, int max_x,
			int max_y, int max_z, int dx, int dy, int dz, bool skip_air = true)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(x + dx, y + dy, z + dz, *b);
				}
	}
	void copy_box_mirror_x(const WorldToModify &src, int min_x, int min_y, int min_z,
			int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			bool skip_air = true)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(dst_x + (max_x - x), dst_y + (y - min_y),
							dst_z + (z - min_z), *b);
				}
	}
	void copy_box_rotate_y(const WorldToModify &src, int min_x, int min_y, int min_z,
			int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			bool skip_air = true)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(dst_x + (z - min_z), dst_y + (y - min_y),
							dst_z + (max_x - x), *b);
				}
	}
	void copy_box_mirror_z(const WorldToModify &src, int min_x, int min_y, int min_z,
			int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			bool skip_air = true)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(dst_x + (x - min_x), dst_y + (y - min_y),
							dst_z + (max_z - z), *b);
				}
	}
	void copy_box_rotate_180(const WorldToModify &src, int min_x, int min_y, int min_z,
			int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			bool skip_air = true)
	{
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(dst_x + (max_x - x), dst_y + (y - min_y),
							dst_z + (max_z - z), *b);
				}
	}
	void copy_box_transform(const WorldToModify &src, int min_x, int min_y, int min_z,
			int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			int quarter_turns, bool mirror_x = false, bool mirror_z = false,
			bool skip_air = true)
	{
		quarter_turns = ((quarter_turns % 4) + 4) % 4;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = std::max(min_y, MIN_Y); y <= std::min(max_y, MAX_Y); ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					int a = x - min_x, c = z - min_z;
					if (mirror_x)
						a = max_x - min_x - a;
					if (mirror_z)
						c = max_z - min_z - c;
					int ox = 0, oz = 0;
					switch (quarter_turns) {
					case 1:
						ox = c;
						oz = max_x - min_x - a;
						break;
					case 2:
						ox = max_x - min_x - a;
						oz = max_z - min_z - c;
						break;
					case 3:
						ox = max_z - min_z - c;
						oz = a;
						break;
					default:
						ox = a;
						oz = c;
					}
					set_block(dst_x + ox, dst_y + (y - min_y), dst_z + oz, *b);
				}
	}
	std::array<int, 6> transformed_bounds(int width, int height, int depth, int dst_x,
			int dst_y, int dst_z, int quarter_turns) const
	{
		quarter_turns = ((quarter_turns % 4) + 4) % 4;
		int w = (quarter_turns % 2) ? depth : width,
			d = (quarter_turns % 2) ? width : depth;
		return {dst_x, dst_y, dst_z, dst_x + w - 1, dst_y + height - 1, dst_z + d - 1};
	}
	bool box_in_world(
			int min_x, int min_y, int min_z, int max_x, int max_y, int max_z) const
	{
		return min_x <= max_x && min_z <= max_z && min_y <= max_y && max_y >= MIN_Y &&
			   min_y <= MAX_Y;
	}
	void copy_box_transform_clipped(const WorldToModify &src, int min_x, int min_y,
			int min_z, int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			int quarter_turns, bool mirror_x = false, bool mirror_z = false,
			bool skip_air = true)
	{
		quarter_turns = ((quarter_turns % 4) + 4) % 4;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = min_y; y <= max_y; ++y) {
					int a = x - min_x, c = z - min_z;
					if (mirror_x)
						a = max_x - min_x - a;
					if (mirror_z)
						c = max_z - min_z - c;
					int ox = 0, oz = 0;
					switch (quarter_turns) {
					case 1:
						ox = c;
						oz = max_x - min_x - a;
						break;
					case 2:
						ox = max_x - min_x - a;
						oz = max_z - min_z - c;
						break;
					case 3:
						ox = max_z - min_z - c;
						oz = a;
						break;
					default:
						ox = a;
						oz = c;
					}
					int wy = dst_y + (y - min_y);
					if (wy < MIN_Y || wy > MAX_Y)
						continue;
					auto b = src.get_block(x, y, z);
					if (!b || (skip_air && b->id() == block_definitions::AIR.id()))
						continue;
					set_block(dst_x + ox, wy, dst_z + oz, *b);
				}
	}
	std::size_t copy_box_transform_count(const WorldToModify &src, int min_x, int min_y,
			int min_z, int max_x, int max_y, int max_z, int dst_x, int dst_y, int dst_z,
			int quarter_turns, bool mirror_x = false, bool mirror_z = false,
			bool skip_air = true)
	{
		std::size_t before = stats().non_air;
		copy_box_transform_clipped(src, min_x, min_y, min_z, max_x, max_y, max_z, dst_x,
				dst_y, dst_z, quarter_turns, mirror_x, mirror_z, skip_air);
		return stats().non_air >= before ? stats().non_air - before : 0;
	}
	std::pair<std::size_t, std::size_t> copy_box_transform_if_empty(
			const WorldToModify &src, int min_x, int min_y, int min_z, int max_x,
			int max_y, int max_z, int dst_x, int dst_y, int dst_z, int quarter_turns,
			bool mirror_x = false, bool mirror_z = false)
	{
		quarter_turns = ((quarter_turns % 4) + 4) % 4;
		std::size_t placed = 0, skipped = 0;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = min_y; y <= max_y; ++y) {
					int a = x - min_x, c = z - min_z;
					if (mirror_x)
						a = max_x - min_x - a;
					if (mirror_z)
						c = max_z - min_z - c;
					int ox = 0, oz = 0;
					switch (quarter_turns) {
					case 1:
						ox = c;
						oz = max_x - min_x - a;
						break;
					case 2:
						ox = max_x - min_x - a;
						oz = max_z - min_z - c;
						break;
					case 3:
						ox = max_z - min_z - c;
						oz = a;
						break;
					default:
						ox = a;
						oz = c;
					}
					int wy = dst_y + y - min_y;
					auto b = src.get_block(x, y, z);
					if (!b || b->id() == block_definitions::AIR.id() || wy < MIN_Y ||
							wy > MAX_Y)
						continue;
					if (occupied_at(dst_x + ox, wy, dst_z + oz)) {
						++skipped;
						continue;
					}
					set_block(dst_x + ox, wy, dst_z + oz, *b);
					++placed;
				}
		return {placed, skipped};
	}
	std::pair<std::size_t, std::size_t> copy_box_mode(const WorldToModify &src, int min_x,
			int min_y, int min_z, int max_x, int max_y, int max_z, int dst_x, int dst_y,
			int dst_z, int quarter_turns, PlacementMode mode)
	{
		quarter_turns = ((quarter_turns % 4) + 4) % 4;
		std::size_t placed = 0, skipped = 0;
		for (int z = min_z; z <= max_z; ++z)
			for (int x = min_x; x <= max_x; ++x)
				for (int y = min_y; y <= max_y; ++y) {
					auto b = src.get_block(x, y, z);
					if (!b || b->id() == block_definitions::AIR.id())
						continue;
					int a = x - min_x, c = z - min_z, ox = 0, oz = 0;
					switch (quarter_turns) {
					case 1:
						ox = c;
						oz = max_x - min_x - a;
						break;
					case 2:
						ox = max_x - min_x - a;
						oz = max_z - min_z - c;
						break;
					case 3:
						ox = max_z - min_z - c;
						oz = a;
						break;
					default:
						ox = a;
						oz = c;
					}
					int wy = dst_y + y - min_y;
					if (wy < MIN_Y || wy > MAX_Y) {
						++skipped;
						continue;
					}
					auto old = get_block(dst_x + ox, wy, dst_z + oz);
					bool occupied = old && old->id() != block_definitions::AIR.id();
					if ((mode == PlacementMode::OnlyAir && occupied) ||
							(mode == PlacementMode::OnlyNonAir && !occupied)) {
						++skipped;
						continue;
					}
					set_block(dst_x + ox, wy, dst_z + oz, *b);
					++placed;
				}
		return {placed, skipped};
	}
	bool validate() const
	{
		for (const auto &r : regions)
			for (const auto &c : r.second.chunks)
				for (const auto &s : c.second.sections) {
					if (s.second.storage.size() != SECTION_VOLUME)
						return false;
					for (auto id : s.second.storage.raw())
						if (id > MAX_BLOCK_ID)
							return false;
				}
		return true;
	}
	void normalize()
	{
		for_each_section_mut([](int, int, int, SectionToModify &s) { s.compact(); });
		prune_empty();
	}
	SectionToModify *section_at(int x, int y, int z)
	{
		int cx = x >> 4, cz = z >> 4;
		auto &r = get_or_create_region(cx >> 5, cz >> 5);
		return &r.get_or_create_chunk(cx & 31, cz & 31).section(y);
	}
	const SectionToModify *section_at(int x, int y, int z) const
	{
		int cx = x >> 4, cz = z >> 4;
		auto *r = get_region(cx >> 5, cz >> 5);
		if (!r)
			return nullptr;
		auto *c = r->get_chunk(cx & 31, cz & 31);
		return c ? c->find_section(y) : nullptr;
	}
	std::array<int, 6> chunk_bounds(
			int cx, int cz, int min_y = MIN_Y, int max_y = MAX_Y) const
	{
		return {cx << 4, min_y, cz << 4, (cx << 4) + 15, std::clamp(max_y, MIN_Y, MAX_Y),
				(cz << 4) + 15};
	}
	void transform_blocks(const std::function<Block(Block)> &fn)
	{
		for (auto &rk : regions)
			for (auto &ck : rk.second.chunks)
				for (auto &sk : ck.second.sections) {
					auto vals = sk.second.storage.materialize();
					for (std::size_t i = 0; i < vals.size(); ++i)
						sk.second.storage.set(i, fn(vals[i]));
				}
		prune_empty();
	}
	std::vector<std::pair<int, int>> region_keys() const
	{
		std::vector<std::pair<int, int>> out;
		out.reserve(regions.size());
		for (const auto &r : regions)
			out.emplace_back(key_x(r.first), key_z(r.first));
		std::sort(out.begin(), out.end());
		return out;
	}
	bool erase_region(int rx, int rz)
	{
		return regions.erase(RegionToModify::key(rx, rz)) != 0;
	}
	std::size_t region_count() const { return regions.size(); }
	Stats stats() const
	{
		Stats s;
		s.regions = regions.size();
		for (const auto &rk : regions) {
			s.chunks += rk.second.chunks.size();
			for (const auto &ck : rk.second.chunks) {
				s.sections += ck.second.sections.size();
				for (const auto &sk : ck.second.sections)
					for (const auto &b : sk.second.storage.materialize())
						if (b.id() != block_definitions::AIR.id())
							++s.non_air;
			}
		}
		return s;
	}
	std::size_t count_block(Block needle) const
	{
		std::size_t n = 0;
		for (const auto &rk : regions)
			for (const auto &ck : rk.second.chunks)
				for (const auto &sk : ck.second.sections)
					for (const auto &b : sk.second.storage.materialize())
						if (b.id() == needle.id())
							++n;
		return n;
	}
	void clear() { regions.clear(); }
	static std::pair<int, int> world_to_region(int x, int z) { return {x >> 9, z >> 9}; }
	static std::pair<int, int> world_to_chunk(int x, int z) { return {x >> 4, z >> 4}; }
	static std::pair<int, int> local_block(int x, int z) { return {x & 15, z & 15}; }
	void for_each_non_air(const std::function<void(int, int, int, Block)> &fn) const
	{
		for (const auto &rk : regions) {
			int rx = key_x(rk.first), rz = key_z(rk.first);
			for (const auto &ck : rk.second.chunks) {
				int lx = key_x(ck.first), lz = key_z(ck.first);
				int bx = ((rx << 5) + lx) << 4, bz = ((rz << 5) + lz) << 4;
				for (const auto &sk : ck.second.sections) {
					auto vals = sk.second.storage.materialize();
					for (int y = 0; y < 16; ++y)
						for (int z = 0; z < 16; ++z)
							for (int x = 0; x < 16; ++x) {
								Block b = vals[(y << 8) | (z << 4) | x];
								if (b.id() != block_definitions::AIR.id())
									fn(bx + x, (sk.first << 4) + y, bz + z, b);
							}
				}
			}
		}
	}
	void for_each_region(
			int rx, int rz, const std::function<void(const RegionToModify &)> &fn) const
	{
		if (auto *r = get_region(rx, rz))
			fn(*r);
	}
	static int key_x(std::uint64_t k)
	{
		return static_cast<int>(static_cast<std::uint32_t>(k >> 32));
	}
	static int key_z(std::uint64_t k)
	{
		return static_cast<int>(static_cast<std::uint32_t>(k));
	}
	void merge(WorldToModify &&other, int min_x, int min_z, int max_x, int max_z)
	{
		for (auto &rk : other.regions) {
			int rx = key_x(rk.first), rz = key_z(rk.first);
			for (auto &ck : rk.second.chunks) {
				int lx = key_x(ck.first), lz = key_z(ck.first);
				int base_x = ((rx << 5) + lx) << 4, base_z = ((rz << 5) + lz) << 4;
				for (auto &sk : ck.second.sections) {
					int sy = sk.first;
					auto vals = sk.second.storage.materialize();
					for (int ly = 0; ly < 16; ++ly)
						for (int lz2 = 0; lz2 < 16; ++lz2)
							for (int lx2 = 0; lx2 < 16; ++lx2) {
								auto b = vals[(ly << 8) | (lz2 << 4) | lx2];
								if (b.id() == block_definitions::AIR.id())
									continue;
								int wx = base_x + lx2, wz = base_z + lz2,
									wy = (sy << 4) + ly;
								bool auth = wx >= min_x && wx <= max_x && wz >= min_z &&
											wz <= max_z;
								if (auth)
									set_block(wx, wy, wz, b);
								else
									set_block_if_absent(wx, wy, wz, b);
							}
				}
			}
		}
	}
	std::optional<int> highest_block_between(int x, int z, int min_y, int max_y) const
	{
		min_y = std::max(min_y, MIN_Y);
		max_y = std::min(max_y, MAX_Y);
		if (min_y > max_y)
			return std::nullopt;
		int cx = x >> 4, cz = z >> 4;
		auto *r = get_region(cx >> 5, cz >> 5);
		if (!r)
			return std::nullopt;
		auto *c = r->get_chunk(cx & 31, cz & 31);
		if (!c)
			return std::nullopt;
		for (int y = max_y; y >= min_y; --y)
			if (c->get_block(x & 15, y, z & 15).id() != block_definitions::AIR.id())
				return y;
		return std::nullopt;
	}
};
inline void invalidate_tiles(WorldToModify &world,
		const std::vector<std::pair<int, int>> &tiles, int tile_size)
{
	for (auto [tx, tz] : tiles) {
		auto [ox, oz] = tile_origin(tx, tz, 0, 0, tile_size);
		auto [rx0, rz0] = WorldToModify::world_to_region(ox, oz);
		auto [rx1, rz1] =
				WorldToModify::world_to_region(ox + tile_size - 1, oz + tile_size - 1);
		for (int rz = rz0; rz <= rz1; ++rz)
			for (int rx = rx0; rx <= rx1; ++rx)
				world.erase_region(rx, rz);
	}
}
inline std::vector<std::pair<int, int>> reconcile_world_plan(WorldToModify &world,
		const std::vector<TilePlan> &old_plan, const std::vector<TilePlan> &new_plan,
		int tile_size)
{
	auto removed = removed_tiles(old_plan, new_plan);
	invalidate_tiles(world, removed, tile_size);
	return removed;
}
inline std::vector<std::pair<int, int>> invalidate_changed_tiles(WorldToModify &world,
		const std::vector<TilePlan> &old_plan, const std::vector<TilePlan> &new_plan,
		int tile_size)
{
	auto changed = changed_tile_plan(old_plan, new_plan);
	std::vector<std::pair<int, int>> keys;
	for (const auto &t : changed)
		keys.emplace_back(t.x, t.z);
	invalidate_tiles(world, keys, tile_size);
	return keys;
}
inline std::vector<std::pair<int, int>> apply_tile_plan_update(WorldToModify &world,
		std::vector<TilePlan> &current, const std::vector<TilePlan> &next, int tile_size)
{
	auto removed = removed_tiles(current, next);
	auto changed = invalidate_changed_tiles(world, current, next, tile_size);
	for (auto t : removed)
		if (std::find(changed.begin(), changed.end(), t) == changed.end())
			changed.push_back(t);
	reconcile_tile_plan(current, next);
	return changed;
}
inline std::vector<std::pair<int, int>> affected_regions_for_tiles(
		const std::vector<std::pair<int, int>> &tiles, int tile_size)
{
	std::vector<std::pair<int, int>> out;
	for (auto [tx, tz] : tiles) {
		auto [ox, oz] = tile_origin(tx, tz, 0, 0, tile_size);
		auto [rx0, rz0] = WorldToModify::world_to_region(ox, oz);
		auto [rx1, rz1] =
				WorldToModify::world_to_region(ox + tile_size - 1, oz + tile_size - 1);
		for (int rz = rz0; rz <= rz1; ++rz)
			for (int rx = rx0; rx <= rx1; ++rx)
				out.emplace_back(rx, rz);
	}
	std::sort(out.begin(), out.end());
	out.erase(std::unique(out.begin(), out.end()), out.end());
	return out;
}
struct RegionUpdatePlan
{
	std::vector<std::pair<int, int>> tiles;
	std::vector<std::pair<int, int>> regions;
};
inline RegionUpdatePlan build_region_update_plan(const std::vector<TilePlan> &old_plan,
		const std::vector<TilePlan> &new_plan, int tile_size)
{
	RegionUpdatePlan p;
	p.tiles = changed_tile_plan(old_plan, new_plan);
	for (auto t : removed_tiles(old_plan, new_plan))
		if (std::find_if(p.tiles.begin(), p.tiles.end(),
					[&](auto x) { return x == t; }) == p.tiles.end())
			p.tiles.push_back(t);
	p.regions = affected_regions_for_tiles(p.tiles, tile_size);
	return p;
}
inline RegionUpdatePlan apply_region_update_plan(WorldToModify &world,
		std::vector<TilePlan> &current, const std::vector<TilePlan> &next, int tile_size)
{
	auto p = build_region_update_plan(current, next, tile_size);
	for (auto [rx, rz] : p.regions)
		world.erase_region(rx, rz);
	reconcile_tile_plan(current, next);
	return p;
}
inline void merge_region_update(WorldToModify &world, const WorldToModify &generated,
		const RegionUpdatePlan &plan)
{
	for (auto [rx, rz] : plan.regions)
		if (auto *r = generated.get_region(rx, rz))
			world.insert_region(rx, rz, *r, true);
}
inline bool update_plan_covers(
		const RegionUpdatePlan &plan, const WorldToModify &generated)
{
	for (auto [rx, rz] : plan.regions)
		if (!generated.get_region(rx, rz))
			return false;
	return true;
}
struct RegionUpdateCheckpoint
{
	WorldToModify before;
	RegionUpdatePlan plan;
};
inline RegionUpdateCheckpoint begin_region_update(
		const WorldToModify &world, const RegionUpdatePlan &plan)
{
	return {world, plan};
}
inline bool commit_region_update(WorldToModify &world, const WorldToModify &generated,
		const RegionUpdateCheckpoint &checkpoint)
{
	if (!update_plan_covers(checkpoint.plan, generated))
		return false;
	merge_region_update(world, generated, checkpoint.plan);
	return true;
}
inline void rollback_region_update(
		WorldToModify &world, RegionUpdateCheckpoint &&checkpoint)
{
	world.restore(std::move(checkpoint.before));
}
inline bool commit_region_update_staged(WorldToModify &world,
		const WorldToModify &generated, const RegionUpdateCheckpoint &checkpoint)
{
	if (!update_plan_covers(checkpoint.plan, generated))
		return false;
	WorldToModify staged = checkpoint.before;
	merge_region_update(staged, generated, checkpoint.plan);
	world.swap_state(staged);
	return true;
}
inline std::vector<std::pair<int, int>> checkpoint_changed_regions(
		const RegionUpdateCheckpoint &checkpoint, const WorldToModify &candidate)
{
	std::vector<std::pair<int, int>> out;
	for (auto [rx, rz] : checkpoint.plan.regions)
		if (checkpoint.before.region_content_hash(rx, rz) !=
				candidate.region_content_hash(rx, rz))
			out.emplace_back(rx, rz);
	return out;
}
struct RegionCommitStats
{
	std::size_t regions = 0, blocks = 0;
};
inline RegionCommitStats region_commit_stats(
		const WorldToModify &world, const std::vector<std::pair<int, int>> &regions)
{
	RegionCommitStats s;
	for (auto [rx, rz] : regions)
		if (auto *r = world.get_region(rx, rz)) {
			++s.regions;
			for (const auto &c : r->chunks)
				for (const auto &sec : c.second.sections)
					s.blocks += non_air_count(sec.second.storage);
		}
	return s;
}
inline std::uint64_t update_plan_volume(
		const RegionUpdatePlan &plan, const GenerationBounds &bounds, int tile_size)
{
	std::uint64_t n = 0;
	for (auto [tx, tz] : plan.tiles)
		n += bounds_volume(clipped_tile_bounds(bounds, tx, tz, tile_size));
	return n;
}
inline std::size_t update_plan_block_count(
		const WorldToModify &world, const RegionUpdatePlan &plan)
{
	std::size_t n = 0;
	for (auto [rx, rz] : plan.regions)
		if (auto *r = world.get_region(rx, rz))
			for (const auto &c : r->chunks)
				for (const auto &s : c.second.sections)
					n += non_air_count(s.second.storage);
	return n;
}
inline double update_progress(std::size_t completed, std::size_t total)
{
	return total ? std::clamp(static_cast<double>(completed) / static_cast<double>(total),
						   0.0, 1.0)
				 : 1.0;
}
inline double update_region_progress(
		const WorldToModify &world, const RegionUpdatePlan &plan)
{
	std::size_t done = 0;
	for (auto [rx, rz] : plan.regions)
		if (world.get_region(rx, rz))
			++done;
	return update_progress(done, plan.regions.size());
}
inline double tile_plan_progress(
		const WorldToModify &world, const std::vector<TilePlan> &plan, int tile_size)
{
	if (plan.empty())
		return 1.0;
	std::size_t done = 0;
	for (const auto &t : plan) {
		auto [rx, rz] = tile_region(t.x, t.z, tile_size);
		if (world.get_region(rx, rz))
			++done;
	}
	return update_progress(done, plan.size());
}
inline bool tile_has_content(const WorldToModify &world, const TilePlan &t, int tile_size)
{
	auto b = clipped_tile_bounds(
			GenerationBounds{t.bounds.min_x, t.bounds.min_y, t.bounds.min_z,
					t.bounds.max_x, t.bounds.max_y, t.bounds.max_z},
			t.x, t.z, tile_size);
	for (auto [rx, rz] : world.occupied_regions(b.min_x, b.min_z, b.max_x, b.max_z))
		if (world.get_region(rx, rz))
			return true;
	return false;
}
inline double tile_content_progress(
		const WorldToModify &world, const std::vector<TilePlan> &plan, int tile_size)
{
	if (plan.empty())
		return 1.0;
	std::size_t done = 0;
	for (const auto &t : plan)
		if (tile_has_content(world, t, tile_size) ||
				t.ownership == TileOwnership::Partial)
			++done;
	return update_progress(done, plan.size());
}
inline double completion_progress(const std::vector<TileCompletion> &c)
{
	std::size_t done = 0;
	for (const auto &e : c)
		if (e.complete)
			++done;
	return update_progress(done, c.size());
}
inline std::vector<TileCompletion> build_completion_plan(
		const std::vector<TilePlan> &plan, const std::vector<TileCompletion> &previous)
{
	std::vector<TileCompletion> out;
	for (const auto &t : plan) {
		auto it = std::find_if(previous.begin(), previous.end(),
				[&](const TileCompletion &e) { return e.x == t.x && e.z == t.z; });
		out.push_back(it == previous.end() ? TileCompletion{t.x, t.z, false, 0} : *it);
	}
	return out;
}
inline void mark_tile_complete(
		std::vector<TileCompletion> &c, int x, int z, std::size_t blocks)
{
	for (auto &e : c)
		if (e.x == x && e.z == z) {
			e.complete = true;
			e.blocks = blocks;
			return;
		}
	c.push_back({x, z, true, blocks});
}
inline void invalidate_completion(
		std::vector<TileCompletion> &c, const std::vector<std::pair<int, int>> &tiles)
{
	for (auto &e : c)
		if (std::find(tiles.begin(), tiles.end(), std::pair<int, int>{e.x, e.z}) !=
				tiles.end()) {
			e.complete = false;
			e.blocks = 0;
		}
}
inline std::vector<TileCompletion> completion_delta(
		const std::vector<TileCompletion> &before,
		const std::vector<TileCompletion> &after)
{
	std::vector<TileCompletion> out;
	for (const auto &e : after) {
		auto it = std::find_if(before.begin(), before.end(),
				[&](const auto &b) { return b.x == e.x && b.z == e.z; });
		if (it == before.end() || it->complete != e.complete || it->blocks != e.blocks)
			out.push_back(e);
	}
	return out;
}
inline std::vector<TileCompletion> apply_completion_plan_update(
		std::vector<TileCompletion> &current, const std::vector<TilePlan> &old_plan,
		const std::vector<TilePlan> &new_plan)
{
	auto next = build_completion_plan(new_plan, current);
	auto affected = changed_tile_plan(old_plan, new_plan);
	std::vector<std::pair<int, int>> keys;
	for (const auto &t : affected)
		keys.emplace_back(t.x, t.z);
	invalidate_completion(next, keys);
	current.swap(next);
	return current;
}
inline std::size_t completed_blocks(const std::vector<TileCompletion> &c)
{
	std::size_t n = 0;
	for (const auto &e : c)
		if (e.complete)
			n += e.blocks;
	return n;
}
inline std::vector<std::pair<int, int>> completed_regions(
		const std::vector<TileCompletion> &c, int tile_size)
{
	std::vector<std::pair<int, int>> out;
	for (const auto &e : c)
		if (e.complete)
			out.push_back(tile_region(e.x, e.z, tile_size));
	std::sort(out.begin(), out.end());
	out.erase(std::unique(out.begin(), out.end()), out.end());
	return out;
}
inline double region_completion_progress(const std::vector<TileCompletion> &c, int)
{
	return completion_progress(c);
}
inline bool completion_matches_plan(
		const std::vector<TileCompletion> &c, const std::vector<TilePlan> &p)
{
	for (const auto &e : c)
		if (std::none_of(p.begin(), p.end(),
					[&](const auto &t) { return t.x == e.x && t.z == e.z; }))
			return false;
	return true;
}
inline void normalize_completion(
		std::vector<TileCompletion> &c, const std::vector<TilePlan> &p)
{
	c.erase(std::remove_if(c.begin(), c.end(),
					[&](const auto &e) {
						return std::none_of(p.begin(), p.end(),
								[&](const auto &t) { return t.x == e.x && t.z == e.z; });
					}),
			c.end());
	std::sort(c.begin(), c.end(), [](const auto &a, const auto &b) {
		return a.z == b.z ? a.x < b.x : a.z < b.z;
	});
	c.erase(std::unique(c.begin(), c.end(),
					[](const auto &a, const auto &b) {
						return a.x == b.x && a.z == b.z;
					}),
			c.end());
}
inline std::uint64_t completion_hash(const std::vector<TileCompletion> &c)
{
	std::uint64_t h = 1469598103934665603ULL;
	for (const auto &e : c) {
		h ^= static_cast<std::uint32_t>(e.x);
		h *= 1099511628211ULL;
		h ^= static_cast<std::uint32_t>(e.z);
		h *= 1099511628211ULL;
		h ^= e.complete;
		h *= 1099511628211ULL;
		h ^= e.blocks;
		h *= 1099511628211ULL;
	}
	return h;
}
inline std::vector<TileCompletion> changed_completions(
		const std::vector<TileCompletion> &a, const std::vector<TileCompletion> &b)
{
	std::vector<TileCompletion> out;
	for (const auto &e : b) {
		auto it = std::find_if(a.begin(), a.end(),
				[&](const auto &o) { return o.x == e.x && o.z == e.z; });
		if (it == a.end() || it->complete != e.complete || it->blocks != e.blocks)
			out.push_back(e);
	}
	return out;
}
struct CompletionCheckpoint
{
	std::vector<TileCompletion> state;
	std::uint64_t hash = 0;
};
inline CompletionCheckpoint checkpoint_completion(const std::vector<TileCompletion> &c)
{
	return {c, completion_hash(c)};
}
inline bool restore_completion(
		std::vector<TileCompletion> &c, const CompletionCheckpoint &cp)
{
	if (completion_hash(cp.state) != cp.hash)
		return false;
	c = cp.state;
	return true;
}
struct GenerationCheckpoint
{
	WorldToModify world;
	CompletionCheckpoint completion;
};
inline GenerationCheckpoint checkpoint_generation(
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	return {world, checkpoint_completion(completion)};
}
inline bool restore_generation(WorldToModify &world,
		std::vector<TileCompletion> &completion, const GenerationCheckpoint &cp)
{
	if (!restore_completion(completion, cp.completion))
		return false;
	world.restore(cp.world);
	return true;
}
struct GenerationCommitReport
{
	std::vector<std::pair<int, int>> changed_regions;
	std::vector<TileCompletion> changed_tiles;
};
inline GenerationCommitReport generation_diff(const GenerationCheckpoint &cp,
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	return {cp.world.changed_regions(world),
			changed_completions(cp.completion.state, completion)};
}
inline bool commit_generation(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp)
{
	(void)cp;
	if (!candidate.validate())
		return false;
	world = candidate;
	completion = candidate_completion;
	return true;
}
inline RegionCommitStats commit_generation_with_stats(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp)
{
	if (!commit_generation(world, completion, candidate, candidate_completion, cp))
		return {};
	world.normalize();
	return {world.region_count(), world.stats().non_air};
}
inline bool within_memory_budget(const WorldToModify &world, std::size_t budget_bytes)
{
	return world.estimated_bytes() <= budget_bytes;
}
inline bool commit_generation_bounded(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, std::size_t budget_bytes)
{
	if (!within_memory_budget(candidate, budget_bytes))
		return false;
	return commit_generation(world, completion, candidate, candidate_completion, cp);
}
inline RegionCommitStats commit_generation_evicted(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, std::size_t budget_bytes,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!commit_generation(world, completion, candidate, candidate_completion, cp))
		return {};
	world.evict_until(budget_bytes, flush);
	return {world.region_count(), world.stats().non_air};
}
inline RegionCommitStats commit_generation_centered(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, std::size_t budget_bytes, int center_rx,
		int center_rz, const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!commit_generation(world, completion, candidate, candidate_completion, cp))
		return {};
	world.evict_farthest(budget_bytes, center_rx, center_rz, flush);
	return {world.region_count(), world.stats().non_air};
}
inline RegionCommitStats commit_generation_protected(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, std::size_t budget_bytes,
		const std::vector<std::pair<int, int>> &protected_regions,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!commit_generation(world, completion, candidate, candidate_completion, cp))
		return {};
	while (world.estimated_bytes() > budget_bytes && !world.regions.empty()) {
		std::optional<std::pair<int, int>> victim;
		std::size_t best = SIZE_MAX;
		for (const auto &r : world.regions) {
			auto key = std::make_pair(
					WorldToModify::key_x(r.first), WorldToModify::key_z(r.first));
			if (std::find(protected_regions.begin(), protected_regions.end(), key) !=
					protected_regions.end())
				continue;
			std::size_t n = 0;
			for (const auto &c : r.second.chunks)
				for (const auto &s : c.second.sections)
					n += s.second.storage.size();
			if (n < best) {
				best = n;
				victim = key;
			}
		}
		if (!victim)
			break;
		auto it = world.regions.find(WorldToModify::key(victim->first, victim->second));
		if (it == world.regions.end())
			break;
		flush(victim->first, victim->second, it->second);
		world.regions.erase(it);
	}
	return {world.region_count(), world.stats().non_air};
}
inline RegionCommitStats commit_generation_centered_protected(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, std::size_t budget_bytes, int center_rx,
		int center_rz, const std::vector<std::pair<int, int>> &protected_regions,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!commit_generation(world, completion, candidate, candidate_completion, cp))
		return {};
	while (world.estimated_bytes() > budget_bytes && !world.regions.empty()) {
		std::optional<std::pair<int, int>> victim;
		long long farthest = -1;
		for (const auto &r : world.regions) {
			auto key = std::make_pair(
					WorldToModify::key_x(r.first), WorldToModify::key_z(r.first));
			if (std::find(protected_regions.begin(), protected_regions.end(), key) !=
					protected_regions.end())
				continue;
			long long dx = key.first - center_rx, dz = key.second - center_rz,
					  d = dx * dx + dz * dz;
			if (d > farthest) {
				farthest = d;
				victim = key;
			}
		}
		if (!victim)
			break;
		auto it = world.regions.find(WorldToModify::key(victim->first, victim->second));
		if (it == world.regions.end())
			break;
		flush(victim->first, victim->second, it->second);
		world.regions.erase(it);
	}
	return {world.region_count(), world.stats().non_air};
}
struct CommitPolicy
{
	std::size_t budget_bytes = 0;
	bool centered = false;
	int center_rx = 0, center_rz = 0;
	std::vector<std::pair<int, int>> protected_regions;
};
inline RegionCommitStats commit_with_policy(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, const CommitPolicy &p,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!p.budget_bytes)
		return commit_generation_with_stats(
				world, completion, candidate, candidate_completion, cp);
	if (!p.protected_regions.empty())
		return p.centered ? commit_generation_centered_protected(world, completion,
									candidate, candidate_completion, cp, p.budget_bytes,
									p.center_rx, p.center_rz, p.protected_regions, flush)
						  : commit_generation_protected(world, completion, candidate,
									candidate_completion, cp, p.budget_bytes,
									p.protected_regions, flush);
	return p.centered ? commit_generation_centered(world, completion, candidate,
								candidate_completion, cp, p.budget_bytes, p.center_rx,
								p.center_rz, flush)
					  : commit_generation_evicted(world, completion, candidate,
								candidate_completion, cp, p.budget_bytes, flush);
}
inline bool valid_commit_policy(const CommitPolicy &p)
{
	return p.budget_bytes == 0 || p.budget_bytes >= sizeof(WorldToModify);
}
inline RegionCommitStats commit_with_policy_checked(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, const CommitPolicy &p,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!valid_commit_policy(p) || !candidate.validate())
		return {};
	return commit_with_policy(
			world, completion, candidate, candidate_completion, cp, p, flush);
}
enum class CommitStatus
{
	Committed,
	InvalidPolicy,
	InvalidCandidate
};
struct CommitResult
{
	CommitStatus status = CommitStatus::InvalidCandidate;
	RegionCommitStats stats{};
};
inline CommitResult commit_with_result(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, const CommitPolicy &p,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	if (!valid_commit_policy(p))
		return {CommitStatus::InvalidPolicy, {}};
	if (!candidate.validate())
		return {CommitStatus::InvalidCandidate, {}};
	return {CommitStatus::Committed, commit_with_policy(world, completion, candidate,
											 candidate_completion, cp, p, flush)};
}
inline bool commit_succeeded(const CommitResult &r)
{
	return r.status == CommitStatus::Committed;
}
inline CommitResult retry_commit_with_budget(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, CommitPolicy p, std::size_t fallback_budget,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	auto r = commit_with_result(
			world, completion, candidate, candidate_completion, cp, p, flush);
	if (!commit_succeeded(r) && fallback_budget) {
		p.budget_bytes = fallback_budget;
		r = commit_with_result(
				world, completion, candidate, candidate_completion, cp, p, flush);
	}
	return r;
}
struct StateFingerprint
{
	std::uint64_t world = 0, completion = 0;
};
inline StateFingerprint fingerprint_state(
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	return {world.content_hash(), completion_hash(completion)};
}
inline bool verify_fingerprint(const WorldToModify &world,
		const std::vector<TileCompletion> &completion, const StateFingerprint &fp)
{
	auto now = fingerprint_state(world, completion);
	return now.world == fp.world && now.completion == fp.completion;
}
struct VerifiedCommitResult
{
	CommitResult result{};
	StateFingerprint fingerprint{};
};
inline VerifiedCommitResult commit_with_fingerprint(WorldToModify &world,
		std::vector<TileCompletion> &completion, const WorldToModify &candidate,
		const std::vector<TileCompletion> &candidate_completion,
		const GenerationCheckpoint &cp, const CommitPolicy &p,
		const std::function<void(int, int, const RegionToModify &)> &flush)
{
	auto r = commit_with_result(
			world, completion, candidate, candidate_completion, cp, p, flush);
	return {r, fingerprint_state(world, completion)};
}
inline bool verify_commit_result(const VerifiedCommitResult &r,
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	return commit_succeeded(r.result) &&
		   verify_fingerprint(world, completion, r.fingerprint);
}
inline bool checkpoint_matches(const GenerationCheckpoint &cp, const WorldToModify &world,
		const std::vector<TileCompletion> &completion)
{
	return cp.world.content_hash() == world.content_hash() &&
		   cp.completion.hash == completion_hash(completion);
}
inline bool rollback_verified(WorldToModify &world,
		std::vector<TileCompletion> &completion, const GenerationCheckpoint &cp)
{
	world.restore(cp.world);
	completion = cp.completion.state;
	return checkpoint_matches(cp, world, completion);
}
enum class TransactionState
{
	Pending,
	Committed,
	RolledBack
};
struct GenerationTransaction
{
	GenerationCheckpoint checkpoint;
	TransactionState state = TransactionState::Pending;
	std::uint64_t sequence = 0;
};
inline GenerationTransaction begin_transaction(
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	static std::uint64_t next = 1;
	return {checkpoint_generation(world, completion), TransactionState::Pending, next++};
}
inline bool finish_transaction(GenerationTransaction &tx, bool committed)
{
	if (tx.state != TransactionState::Pending)
		return false;
	tx.state = committed ? TransactionState::Committed : TransactionState::RolledBack;
	return true;
}
inline bool commit_transaction(GenerationTransaction &tx, WorldToModify &world,
		std::vector<TileCompletion> &completion)
{
	(void)world;
	(void)completion;
	if (tx.state != TransactionState::Pending)
		return false;
	tx.state = TransactionState::Committed;
	return true;
}
inline bool rollback_transaction(GenerationTransaction &tx, WorldToModify &world,
		std::vector<TileCompletion> &completion)
{
	if (tx.state != TransactionState::Pending)
		return false;
	world.restore(tx.checkpoint.world);
	completion = tx.checkpoint.completion.state;
	tx.state = TransactionState::RolledBack;
	return true;
}
struct TransactionReport
{
	TransactionState state = TransactionState::Pending;
	StateFingerprint before{};
	StateFingerprint after{};
};
inline TransactionReport transaction_report(const GenerationTransaction &tx,
		const WorldToModify &world, const std::vector<TileCompletion> &completion)
{
	return {tx.state, {tx.checkpoint.world.content_hash(), tx.checkpoint.completion.hash},
			fingerprint_state(world, completion)};
}
inline bool transaction_valid(const GenerationTransaction &tx, const WorldToModify &world,
		const std::vector<TileCompletion> &completion)
{
	if (tx.state == TransactionState::Pending)
		return true;
	auto r = transaction_report(tx, world, completion);
	return r.before.world != 0 || r.before.completion != 0 ||
		   r.after.world == r.before.world && r.after.completion == r.before.completion;
}
inline bool commit_transaction_checked(GenerationTransaction &tx, WorldToModify &world,
		std::vector<TileCompletion> &completion)
{
	if (!transaction_valid(tx, world, completion))
		return false;
	return commit_transaction(tx, world, completion);
}
inline bool transaction_stale(
		const GenerationTransaction &tx, std::uint64_t current_sequence)
{
	return tx.state == TransactionState::Pending && tx.sequence < current_sequence;
}
inline bool validate_generation_candidate(const WorldToModify &candidate,
		const std::vector<TileCompletion> &completion, const std::vector<TilePlan> &plan)
{
	return candidate.validate() && completion_matches_plan(completion, plan);
}
struct Chunk
{
	std::int32_t x_pos = 0, z_pos = 0;
	std::vector<Section> sections;
	bool is_light_on = false;
};
inline std::int8_t section_y(int y)
{
	return static_cast<std::int8_t>(y >> 4);
}
inline Section &ensure_section(Chunk &chunk, int y)
{
	auto sy = section_y(y);
	for (auto &s : chunk.sections)
		if (s.y == sy)
			return s;
	chunk.sections.push_back({sy, BlockStorage{}});
	return chunk.sections.back();
}
inline Block get_block(const Chunk &chunk, int x, int y, int z)
{
	auto sy = section_y(y);
	for (const auto &s : chunk.sections)
		if (s.y == sy)
			return s.storage.get(section_index(x, y, z));
	return Block{};
}
inline void set_block(Chunk &chunk, int x, int y, int z, Block b)
{
	auto &s = ensure_section(chunk, y);
	s.storage.set(section_index(x, y, z), b);
}
inline void compact_chunk(Chunk &chunk)
{
	for (auto &s : chunk.sections)
		s.storage.compact();
}
inline std::vector<Block> chunk_blocks(const Chunk &chunk, int section_y_value)
{
	for (const auto &s : chunk.sections)
		if (s.y == section_y_value)
			return s.storage.materialize();
	return std::vector<Block>(SECTION_VOLUME, Block{});
}

// Exporter-neutral equivalent of Rust Blockstates::to_section.  The palette is
// keyed by raw block id, while the eventual backend decides how ids become
// names/properties.  Values are packed in the same little-endian 64-bit
// sequence used by Java's block-state arrays (minimum four bits per entry).
inline PackedPalette pack_palette(const BlockStorage &storage)
{
	PackedPalette out;
	std::array<std::uint16_t, MAX_BLOCK_ID> lookup{};
	lookup.fill(UINT16_MAX);
	for (const auto &block : storage.materialize()) {
		auto id = static_cast<std::size_t>(block.id());
		if (id >= MAX_BLOCK_ID)
			continue;
		if (lookup[id] == UINT16_MAX) {
			lookup[id] = static_cast<std::uint16_t>(out.palette.size());
			out.palette.push_back({std::to_string(id)});
		}
	}
	const std::size_t bits = std::max<std::size_t>(4,
			out.palette.size() <= 1 ? 4
									: static_cast<std::size_t>(std::ceil(std::log2(
											  static_cast<double>(out.palette.size())))));
	out.bits_per_block = bits;
	std::uint64_t word = 0;
	std::size_t used = 0;
	for (const auto &block : storage.materialize()) {
		auto id = static_cast<std::size_t>(block.id());
		std::uint64_t p =
				(id < MAX_BLOCK_ID && lookup[id] != UINT16_MAX) ? lookup[id] : 0;
		if (used + bits > 64) {
			out.data.push_back(word);
			word = 0;
			used = 0;
		}
		word |= p << used;
		used += bits;
	}
	if (used)
		out.data.push_back(word);
	return out;
}
inline std::size_t non_air_count(const BlockStorage &storage)
{
	std::size_t n = 0;
	for (const auto &b : storage.materialize())
		if (b.id() != block_definitions::AIR.id())
			++n;
	return n;
}
inline std::vector<std::uint16_t> palette_ids(const BlockStorage &storage)
{
	std::vector<std::uint16_t> ids;
	std::array<bool, MAX_BLOCK_ID> seen{};
	for (const auto &b : storage.materialize()) {
		auto id = static_cast<std::size_t>(b.id());
		if (id < MAX_BLOCK_ID && !seen[id]) {
			seen[id] = true;
			ids.push_back(static_cast<std::uint16_t>(id));
		}
	}
	return ids;
}
}
