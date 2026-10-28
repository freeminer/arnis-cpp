#include "schematic.h"
#include "../../../arnis_adapter.h"
#include "../ground_decoration.h"
#include "../land_cover/land_cover.h"
#include "../block_definitions.h"
#include <array>
#include <fstream>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <bit>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_map>
#include <deque>
namespace arnis::trees
{
namespace
{
// Keep these placement limits aligned with schematic.rs. The first determines
// which low log columns are treated as grounded trunks; the second bounds root
// extension at tile seams.
constexpr int ROOT_BASE_VY = 2;
constexpr int ROOT_MAX = 64;

struct TreeCellHash
{
	std::size_t operator()(const std::pair<int, int> &cell) const noexcept
	{
		const auto packed =
				(std::uint64_t(static_cast<std::uint32_t>(cell.first)) << 32) |
				static_cast<std::uint32_t>(cell.second);
		return std::hash<std::uint64_t>{}(packed);
	}
};

bool is_log_block(Block block)
{
	const auto &logs = ground_decoration::wood_blocks();
	return std::find(logs.begin(), logs.end(), block) != logs.end();
}

bool is_leaf_block(Block block)
{
	using namespace block_definitions;
	static const std::array<Block, 9> leaves = {OAK_LEAVES, BIRCH_LEAVES, SPRUCE_LEAVES,
			DARK_OAK_LEAVES, JUNGLE_LEAVES, ACACIA_LEAVES, CHERRY_LEAVES, MANGROVE_LEAVES,
			AZALEA_LEAVES};
	return std::find(leaves.begin(), leaves.end(), block) != leaves.end();
}

void drop_stray_leaves(Schematic &schematic)
{
	if (schematic.voxels.empty())
		return;
	const auto key = [](int x, int y, int z) {
		return (std::uint64_t(static_cast<std::uint32_t>(y)) << 32) |
			   (std::uint64_t(static_cast<std::uint16_t>(z)) << 16) |
			   static_cast<std::uint16_t>(x);
	};
	// Most tree schematics are compact. A bounded dense index avoids a hash
	// lookup for each of the 125 neighboring cells; sparse or malformed shapes
	// retain the original hash-map behavior.
	constexpr std::size_t max_dense_cells = 1024 * 1024;
	const auto volume =
			schematic.width > 0 && schematic.height > 0 && schematic.length > 0 &&
							std::size_t(schematic.width) <= max_dense_cells &&
							std::size_t(schematic.height) <= max_dense_cells &&
							std::size_t(schematic.length) <= max_dense_cells
					? std::uint64_t(schematic.width) * std::uint64_t(schematic.height) *
							  std::uint64_t(schematic.length)
					: 0;
	const bool dense = volume > 0 && volume <= max_dense_cells &&
					   std::all_of(schematic.voxels.begin(), schematic.voxels.end(),
							   [&](const auto &v) {
								   return v.x >= 0 && v.x < schematic.width && v.y >= 0 &&
										  v.y < schematic.height && v.z >= 0 &&
										  v.z < schematic.length;
							   });
	const auto dense_key = [&](int x, int y, int z) {
		return (std::size_t(y) * schematic.length + z) * schematic.width + x;
	};
	const auto missing = std::numeric_limits<std::size_t>::max();
	std::vector<std::size_t> dense_grid(dense ? volume : 0, missing);
	std::unordered_map<std::uint64_t, std::size_t> grid;
	if (!dense)
		grid.reserve(schematic.voxels.size());
	std::vector<bool> linked(schematic.voxels.size(), false);
	std::deque<std::size_t> todo;
	for (std::size_t i = 0; i < schematic.voxels.size(); ++i) {
		const auto &v = schematic.voxels[i];
		if (dense) {
			auto &entry = dense_grid[dense_key(v.x, v.y, v.z)];
			if (entry == missing)
				entry = i;
		} else {
			grid.emplace(key(v.x, v.y, v.z), i);
		}
		if (is_log_block(v.block)) {
			linked[i] = true;
			todo.push_back(i);
		}
	}
	if (todo.empty())
		return;
	while (!todo.empty()) {
		const auto &v = schematic.voxels[todo.front()];
		const int x = v.x, y = v.y, z = v.z;
		todo.pop_front();
		for (int ny = std::max(0, y - 2); ny <= std::min(schematic.height - 1, y + 2);
				++ny)
			for (int nz = std::max(0, z - 2); nz <= std::min(schematic.length - 1, z + 2);
					++nz)
				for (int nx = std::max(0, x - 2);
						nx <= std::min(schematic.width - 1, x + 2); ++nx) {
					std::size_t neighbor = missing;
					if (dense) {
						neighbor = dense_grid[dense_key(nx, ny, nz)];
					} else if (const auto found = grid.find(key(nx, ny, nz));
							   found != grid.end()) {
						neighbor = found->second;
					}
					if (neighbor == missing || linked[neighbor])
						continue;
					linked[neighbor] = true;
					todo.push_back(neighbor);
				}
	}
	std::size_t i = 0;
	schematic.voxels.erase(
			std::remove_if(schematic.voxels.begin(), schematic.voxels.end(),
					[&](const auto &) { return !linked[i++]; }),
			schematic.voxels.end());
}

void finalize_tree_schematic(Schematic &schematic)
{
	schematic.voxels.shrink_to_fit();
	if (!schematic.voxels.empty()) {
		int min_y = schematic.voxels.front().y;
		for (const auto &voxel : schematic.voxels)
			min_y = std::min(min_y, static_cast<int>(voxel.y));
		if (min_y != 0)
			for (auto &voxel : schematic.voxels)
				voxel.y -= min_y;
		schematic.height = 0;
		std::set<std::pair<int, int>> leaf_columns;
		for (const auto &voxel : schematic.voxels) {
			schematic.height = std::max(schematic.height, voxel.y + 1);
			if (is_leaf_block(voxel.block))
				leaf_columns.emplace(voxel.x, voxel.z);
		}
		schematic.leaf_columns = static_cast<std::uint32_t>(leaf_columns.size());
	}
	schematic.min_log_vy = schematic.height;
	for (const auto &voxel : schematic.voxels)
		if (is_log_block(voxel.block))
			schematic.min_log_vy =
					std::min(schematic.min_log_vy, static_cast<int>(voxel.y));
	if (schematic.min_log_vy == schematic.height)
		schematic.min_log_vy = 0;
}

// Tree assets deliberately collapse decorative states to the Rust tree palette.
std::optional<std::string> tree_block_name(std::string name)
{
	name = name.substr(0, name.find('['));
	while (name.starts_with("minecraft:"))
		name.erase(0, 10);
	// Keep this allowlist in lockstep with Rust's map_block(). In particular,
	// stripped states are supported only for known log/wood blocks; stripping
	// the prefix generically would accidentally admit blocks such as
	// stripped_cherry_leaves or stripped_mangrove_roots.
	if (name.starts_with("stripped_")) {
		const auto base = name.substr(9);
		if (base == "pale_oak_log" || base == "pale_oak_wood")
			return "minecraft:birch_log";
		for (const auto *species : {"oak", "birch", "spruce", "dark_oak", "jungle",
					 "acacia", "cherry", "mangrove"}) {
			const std::string prefix(species);
			if (base == prefix + "_log" || base == prefix + "_wood")
				return "minecraft:" + prefix + "_log";
		}
		if (base == "bamboo_block")
			return "minecraft:jungle_log";
		if (base == "warped_stem" || base == "warped_hyphae")
			return "minecraft:spruce_log";
		return std::nullopt;
	}
	if (name == "vine" || name == "moss_block" || name == "moss_carpet")
		return "minecraft:oak_leaves";
	if (name == "flowering_azalea_leaves" || name == "azalea_leaves")
		return "minecraft:azalea_leaves";
	if (name == "mangrove_roots" || name == "muddy_mangrove_roots")
		return "minecraft:mangrove_log";
	if (name == "mangrove_propagule")
		return "minecraft:mangrove_leaves";
	if (name == "bamboo_block")
		return "minecraft:jungle_log";
	if (name == "warped_stem" || name == "warped_hyphae")
		return "minecraft:spruce_log";
	if (name == "pale_oak_leaves")
		return "minecraft:birch_leaves";
	if (name == "pale_oak_log" || name == "pale_oak_wood")
		return "minecraft:birch_log";
	for (const auto *species : {"oak", "birch", "spruce", "dark_oak", "jungle", "acacia",
				 "cherry", "mangrove"}) {
		const std::string prefix(species);
		if (name == prefix + "_log" || name == prefix + "_wood")
			return "minecraft:" + prefix + "_log";
		if (name == prefix + "_leaves")
			return "minecraft:" + name;
	}
	return std::nullopt;
}
}

std::optional<Block> map_block(const std::string &name)
{
	const auto canonical = tree_block_name(name);
	if (!canonical)
		return std::nullopt;
	return structures::resolve_schem_block(*canonical);
}
Schematic load_schem(const std::filesystem::path &file)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		return {};
	std::vector<std::uint8_t> data((std::istreambuf_iterator<char>(in)), {});
	return load_schem(data);
}
Schematic load_schem(const std::vector<std::uint8_t> &gzip_bytes)
{
	Schematic schematic;
	bool unsupported_coordinate = false;
	structures::SchemDecodeOptions options;
	options.include_entities = false;
	options.block_mapper = map_block;
	options.voxel_sink = [&schematic, &unsupported_coordinate](
								 int x, int y, int z, Block block) {
		constexpr int coordinate_max = std::numeric_limits<std::int16_t>::max();
		if (x < 0 || y < 0 || z < 0 || x > coordinate_max || y > coordinate_max ||
				z > coordinate_max) {
			unsupported_coordinate = true;
			return;
		}
		schematic.voxels.push_back({static_cast<std::int16_t>(x),
				static_cast<std::int16_t>(y), static_cast<std::int16_t>(z), block});
	};
	const auto document = structures::decode_sponge_schem(gzip_bytes, options);
	constexpr int dimension_max = std::numeric_limits<std::int16_t>::max();
	if (unsupported_coordinate || document.width <= 0 || document.height <= 0 ||
			document.length <= 0 || document.width > dimension_max ||
			document.height > dimension_max || document.length > dimension_max)
		return {};
	schematic.width = document.width;
	schematic.height = document.height;
	schematic.length = document.length;
	drop_stray_leaves(schematic);
	finalize_tree_schematic(schematic);
	return schematic;
}
std::shared_ptr<const Schematic> load_schem_shared(const std::filesystem::path &file)
{
	struct Assets
	{
		std::mutex mutex;
		std::unordered_map<std::string, std::weak_ptr<const Schematic>> entries;
	};
	// Detached selector loaders can outlive shutdown of other static objects.
	static auto *assets = new Assets;
	const auto key = std::filesystem::absolute(file).lexically_normal().string();
	{
		std::lock_guard lock(assets->mutex);
		if (const auto it = assets->entries.find(key); it != assets->entries.end())
			if (auto shared = it->second.lock())
				return shared;
	}
	auto shared = std::make_shared<const Schematic>(load_schem(file));
	if (!shared->voxels.empty()) {
		std::lock_guard lock(assets->mutex);
		if (const auto it = assets->entries.find(key); it != assets->entries.end())
			if (auto existing = it->second.lock())
				return existing;
		if (assets->entries.size() >= 1024)
			std::erase_if(assets->entries,
					[](const auto &entry) { return entry.second.expired(); });
		assets->entries.insert_or_assign(key, shared);
	}
	return shared;
}
int min_log_y(const Schematic &s)
{
	return s.min_log_vy;
}
Schematic tree_only(const structures::SchemDocument &in)
{
	// Retain only the data Rust's tree schematic needs; general schematic
	// offsets and block entities are irrelevant after tree decoding.
	Schematic out;
	constexpr int dimension_max = std::numeric_limits<std::int16_t>::max();
	if (in.width <= 0 || in.height <= 0 || in.length <= 0 || in.width > dimension_max ||
			in.height > dimension_max || in.length > dimension_max)
		return out;
	out.width = in.width;
	out.height = in.height;
	out.length = in.length;
	bool unsupported_coordinate = false;
	const auto append = [&](int x, int y, int z, Block block) {
		constexpr int coordinate_max = std::numeric_limits<std::int16_t>::max();
		if (x < 0 || y < 0 || z < 0 || x > coordinate_max || y > coordinate_max ||
				z > coordinate_max) {
			unsupported_coordinate = true;
			return;
		}
		out.voxels.push_back({static_cast<std::int16_t>(x), static_cast<std::int16_t>(y),
				static_cast<std::int16_t>(z), block});
	};
	for (const auto &v : in.voxels) {
		if (v.block.empty()) {
			if (v.resolved_block != Block{})
				append(v.x, v.y, v.z, v.resolved_block);
		} else if (const auto block = map_block(v.block))
			append(v.x, v.y, v.z, *block);
	}
	if (unsupported_coordinate)
		return {};
	drop_stray_leaves(out);
	// Sponge schematics frequently carry an air pad below the root. Rust removes
	// it at load time so every tree has a stable y=0 floor independent of the
	// editor/exporter that produced the asset.
	finalize_tree_schematic(out);
	return out;
}
bool has_leaves(const Schematic &s)
{
	return std::any_of(s.voxels.begin(), s.voxels.end(),
			[&](const auto &v) { return is_leaf_block(v.block); });
}
TreeSize schematic_size(const Schematic &s)
{
	return size_for_height(s.height);
}
std::pair<int, int> trunk_slot_s(int x, int z, int spacing)
{
	const int s = std::max(1, spacing);
	// Preserve Rust's Euclidean cell division and its deliberately tiny (0/1)
	// jitter.  A modulo-spread jitter looks reasonable locally but produces a
	// different global lattice and consequently visible tile-seam divergence.
	auto euclid_div = [s](int v) {
		const auto wide = static_cast<std::int64_t>(v);
		return static_cast<std::int32_t>(wide >= 0 ? wide / s : -((-wide + s - 1) / s));
	};
	const auto cx = euclid_div(x), cz = euclid_div(z);
	// Rust's wrapped seed arithmetic is unsigned arithmetic in C++, then a
	// bit-preserving conversion back to the signed coordinate hash input.
	const auto hx = std::bit_cast<std::int32_t>(std::uint32_t(cx) * 0x1f1fu + 17u);
	const auto hz = std::bit_cast<std::int32_t>(std::uint32_t(cz) * 0x2b2bu + 91u);
	const auto h = land_cover::coord_hash(hx, hz);
	return {std::bit_cast<std::int32_t>(
					std::uint32_t(cx) * std::uint32_t(s) + std::uint32_t(h & 1U)),
			std::bit_cast<std::int32_t>(
					std::uint32_t(cz) * std::uint32_t(s) + std::uint32_t((h >> 1) & 1U))};
}

std::pair<int, int> rotate_xz(int x, int z, int width, int length, unsigned rotation)
{
	switch (rotation & 3u) {
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

bool place_schematic(world_editor::WorldEditor &editor, const Schematic &s, int x, int y,
		int z, unsigned rot)
{
	bool placed = false;
	const bool quarter = (rot & 1u) != 0;
	const int final_w = quarter ? s.length : s.width;
	const int final_l = quarter ? s.width : s.length;
	const int center_x = (final_w - 1) / 2;
	const int center_z = (final_l - 1) / 2;
	for (const auto &v : s.voxels) {
		const auto [px, pz] = rotate_xz(v.x, v.z, s.width, s.length, rot);
		const Block b = v.block;
		if (b != Block{}) {
			editor.set_block_absolute(b, x + px - center_x, y + v.y, z + pz - center_z);
			placed = true;
		}
	}
	return placed;
}
bool place_schematic_tree(world_editor::WorldEditor &editor, const Schematic &s,
		int anchor_x, int anchor_z, int base_y, unsigned rot,
		const std::vector<Block> &blacklist, const BuildingFootprintBitmap *footprints,
		int y_offset)
{
	using namespace block_definitions;
	const int fw = (rot & 1) ? s.length : s.width;
	const int fl = (rot & 1) ? s.width : s.length;
	if (fw <= 0 || fl <= 0)
		return false;
	const int cx = (fw - 1) / 2, cz = (fl - 1) / 2;
	const auto is_log = is_log_block;
	// Snapshot before stamping: tree voxels must never count as their own roof.
	// Match Rust's compact per-column array and skip the whole probe when the
	// footprint does not intersect this crown. Most trees are outside buildings,
	// so avoiding a map allocation and height scan there is significant.
	std::optional<std::vector<int>> roof_tops;
	if (footprints) {
		bool overlaps = false;
		for (int rx = 0; rx < fw && !overlaps; ++rx)
			for (int rz = 0; rz < fl; ++rz)
				if (footprints->contains(anchor_x + rx - cx, anchor_z + rz - cz)) {
					overlaps = true;
					break;
				}
		if (overlaps) {
			roof_tops.emplace(
					static_cast<std::size_t>(fw) * fl, std::numeric_limits<int>::min());
			for (int rx = 0; rx < fw; ++rx)
				for (int rz = 0; rz < fl; ++rz) {
					const int wx = anchor_x + rx - cx, wz = anchor_z + rz - cz;
					if (footprints->contains(wx, wz))
						(*roof_tops)[static_cast<std::size_t>(rx) * fl + rz] =
								editor.highest_block_between(
											  wx, wz, base_y, base_y + s.height)
										.value_or(base_y + s.height);
				}
		}
	}
	struct Root
	{
		int top;
		Block block;
	};
	std::unordered_map<std::pair<int, int>, Root, TreeCellHash> trunk_bottom;
	trunk_bottom.reserve(static_cast<std::size_t>(s.width) * s.length);
	std::unordered_map<std::pair<int, int>, Root, TreeCellHash> base_logs;
	bool placed = false;
	for (const auto &v : s.voxels) {
		const auto [rx, rz] = rotate_xz(v.x, v.z, s.width, s.length, rot);
		const int wx = anchor_x + rx - cx, wz = anchor_z + rz - cz;
		const int wy = base_y + v.y;
		const int roof = roof_tops ? (*roof_tops)[static_cast<std::size_t>(rx) * fl + rz]
								   : std::numeric_limits<int>::min();
		if (roof != std::numeric_limits<int>::min() && wy <= roof)
			continue;
		const bool log = is_log(v.block);
		if (!log && editor.check_for_block_absolute(
							wx, wy - 1, wz, std::optional<std::vector<Block>>({WATER})))
			continue;
		if (log && (editor.check_for_block(
							wx, 0, wz, std::optional<std::vector<Block>>({WATER})) ||
						   (v.y <= s.min_log_vy + ROOT_BASE_VY &&
								   editor.is_lc_water(wx, wz))))
			continue;
		const Block block = v.block;
		placed |=
				editor.try_set_block_absolute(block, wx, wy, wz, std::nullopt, blacklist);
		if (log) {
			ground_decoration::clear_undergrowth_under_trunk(editor, wx, wy, wz);
			const auto cell = std::pair{wx, wz};
			const auto it = trunk_bottom.find(cell);
			if (it == trunk_bottom.end() || wy < it->second.top)
				trunk_bottom[cell] = {wy, block};
			if (v.y >= 0) {
				const auto base_it = base_logs.find(cell);
				if (base_it == base_logs.end() || wy < base_it->second.top)
					base_logs[cell] = {wy, block};
			}
		}
	}
	int lowest = std::numeric_limits<int>::max();
	for (const auto &[cell, root] : trunk_bottom)
		lowest = std::min(lowest, root.top);
	auto root_blacklist = blacklist;
	root_blacklist.insert(
			root_blacklist.end(), {BLACK_CONCRETE, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA,
										  GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, DIRT_PATH});
	for (const auto &[cell, root] : trunk_bottom) {
		const auto [wx, wz] = cell;
		if (root.top > lowest + ROOT_BASE_VY || editor.is_lc_water(wx, wz))
			continue;
		const int from = std::max(
				root.top - 1 - ROOT_MAX, editor.get_absolute_y(wx, y_offset, wz));
		for (int wy = from; wy < root.top; ++wy)
			placed |= editor.try_set_block_absolute(
					root.block, wx, wy, wz, std::nullopt, root_blacklist);
	}
	// Rooted base columns that still start one to three blocks above local ground
	// must bridge the gap; branches and raised root columns stay untouched.
	int lowest_base_y = std::numeric_limits<int>::max();
	for (const auto &[cell, root] : base_logs)
		lowest_base_y = std::min(lowest_base_y, root.top);
	for (const auto &[cell, root] : base_logs) {
		const auto [wx, wz] = cell;
		if (root.top != lowest_base_y || editor.is_lc_water(wx, wz))
			continue;
		const int ground_y = editor.get_absolute_y(wx, y_offset, wz);
		const int gap = root.top - ground_y;
		if (gap < 1 || gap > 3)
			continue;
		for (int wy = ground_y; wy < root.top; ++wy)
			placed |= editor.try_set_block_absolute(
					root.block, wx, wy, wz, std::nullopt, root_blacklist);
	}
	return placed;
}

bool place_schematic_rooted(world_editor::WorldEditor &editor, const Schematic &s, int x,
		int ground_y, int z, unsigned rotation)
{
	// Keep this compatibility entry point on the same water, footprint, and
	// root rules as the active tree pipeline.
	return place_schematic_tree(editor, s, x, z, ground_y, rotation, {}, nullptr, 0);
}
}
