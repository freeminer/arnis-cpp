#include "schematic.h"
#include "../../../arnis_adapter.h"
#include "../land_cover/land_cover.h"
#include "../block_definitions.h"
#include <fstream>
#include <algorithm>
#include <limits>
#include <map>
#include <bit>
namespace arnis::trees
{
namespace
{
// Tree assets deliberately collapse decorative states to the Rust tree palette.
std::optional<std::string> tree_block_name(std::string name)
{
	name = name.substr(0, name.find('['));
	if (name.starts_with("minecraft:"))
		name.erase(0, 10);
	if (name.starts_with("stripped_"))
		name.erase(0, 9);
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
	return tree_only(structures::decode_sponge_schem(data));
}
int min_log_y(const Schematic &s)
{
	int out = s.height;
	for (const auto &v : s.voxels)
		if (v.block.find("_log") != std::string::npos ||
				v.block.find("_wood") != std::string::npos)
			out = std::min(out, v.y);
	return out == s.height ? 0 : out;
}
Schematic tree_only(const Schematic &in)
{
	Schematic out = in;
	out.voxels.clear();
	out.entities.clear();
	for (const auto &v : in.voxels) {
		if (auto name = tree_block_name(v.block))
			out.voxels.push_back({v.x, v.y, v.z, std::move(*name), {}});
	}
	// Sponge schematics frequently carry an air pad below the root. Rust removes
	// it at load time so every tree has a stable y=0 floor independent of the
	// editor/exporter that produced the asset.
	if (!out.voxels.empty()) {
		int min_y = out.voxels.front().y;
		for (const auto &v : out.voxels)
			min_y = std::min(min_y, v.y);
		if (min_y) {
			for (auto &v : out.voxels)
				v.y -= min_y;
			// Rust retains the original asset height for size classification.
		}
	}
	return out;
}
bool has_leaves(const Schematic &s)
{
	for (const auto &v : s.voxels) {
		const auto canonical = tree_block_name(v.block);
		if (canonical && canonical->find("_leaves") != std::string::npos)
			return true;
	}
	return false;
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
		int px = v.x, pz = v.z;
		switch (rot & 3u) {
		case 1:
			px = s.length - 1 - v.z;
			pz = v.x;
			break;
		case 2:
			px = s.width - 1 - v.x;
			pz = s.length - 1 - v.z;
			break;
		case 3:
			px = v.z;
			pz = s.width - 1 - v.x;
			break;
		default:
			break;
		}
		Block b = structures::resolve_schem_block(v.block);
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
	const auto position = [&](const auto &v) {
		int rx = v.x, rz = v.z;
		switch (rot & 3) {
		case 1:
			rx = s.length - 1 - v.z;
			rz = v.x;
			break;
		case 2:
			rx = s.width - 1 - v.x;
			rz = s.length - 1 - v.z;
			break;
		case 3:
			rx = v.z;
			rz = s.width - 1 - v.x;
			break;
		default:
			break;
		}
		return std::pair{anchor_x + rx - cx, anchor_z + rz - cz};
	};
	const auto is_log = [](const std::string &name) {
		const auto base = name.substr(0, name.find('['));
		return base.ends_with("_log") || base.ends_with("_wood") ||
			   base.ends_with("_roots") || base.ends_with("_stem") ||
			   base.ends_with("_hyphae") || base.ends_with("bamboo_block");
	};
	// Snapshot before stamping: tree voxels must never count as their own roof.
	std::map<std::pair<int, int>, int> roof_tops;
	if (footprints)
		for (int rx = 0; rx < fw; ++rx)
			for (int rz = 0; rz < fl; ++rz) {
				const int wx = anchor_x + rx - cx, wz = anchor_z + rz - cz;
				if (footprints->contains(wx, wz))
					roof_tops[{wx, wz}] = editor.highest_block_between(
														wx, wz, base_y, base_y + s.height)
												  .value_or(base_y + s.height);
			}
	int min_log_vy = s.height;
	for (const auto &v : s.voxels)
		if (is_log(v.block))
			min_log_vy = std::min(min_log_vy, v.y);
	struct Root
	{
		int top;
		Block block;
	};
	std::map<std::pair<int, int>, Root> trunk_bottom;
	bool placed = false;
	for (const auto &v : s.voxels) {
		const auto cell = position(v);
		const auto [wx, wz] = cell;
		const int wy = base_y + v.y;
		const auto roof = roof_tops.find(cell);
		if (roof != roof_tops.end() && wy <= roof->second)
			continue;
		const bool log = is_log(v.block);
		if (log && (editor.check_for_block(
							wx, 0, wz, std::optional<std::vector<Block>>({WATER})) ||
						   (v.y <= min_log_vy + 2 && editor.is_lc_water(wx, wz))))
			continue;
		const Block block = structures::resolve_schem_block(v.block);
		if (block == Block{})
			continue;
		placed |=
				editor.try_set_block_absolute(block, wx, wy, wz, std::nullopt, blacklist);
		if (log) {
			const auto it = trunk_bottom.find(cell);
			if (it == trunk_bottom.end() || wy < it->second.top)
				trunk_bottom[cell] = {wy, block};
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
		if (root.top > lowest + 2 || editor.is_lc_water(wx, wz))
			continue;
		const int from = std::max(root.top - 65, editor.get_absolute_y(wx, y_offset, wz));
		for (int wy = from; wy < root.top; ++wy)
			placed |= editor.try_set_block_absolute(
					root.block, wx, wy, wz, std::nullopt, root_blacklist);
	}
	return placed;
}

bool place_schematic_rooted(world_editor::WorldEditor &editor, const Schematic &s, int x,
		int ground_y, int z, unsigned rotation)
{
	// Tree assets are normalized to a floor at y=0. Keeping base_y on ground
	// matches Rust's stamped canopy and leaves the root-extension pass free to
	// follow local slopes instead of shifting the whole model upward.
	const bool placed = place_schematic(editor, s, x, ground_y, z, rotation);
	if (!placed)
		return false;
	const bool quarter = (rotation & 1u) != 0;
	const int final_w = quarter ? s.length : s.width;
	const int final_l = quarter ? s.width : s.length;
	const int center_x = (final_w - 1) / 2, center_z = (final_l - 1) / 2;
	struct Root
	{
		int top;
		Block block;
	};
	std::map<std::pair<int, int>, Root> bottoms;
	for (const auto &v : s.voxels) {
		const auto base = v.block.substr(0, v.block.find('['));
		if (base.find("_log") == std::string::npos &&
				base.find("_wood") == std::string::npos &&
				base.find("_roots") == std::string::npos &&
				base.find("bamboo_block") == std::string::npos &&
				base.find("warped_stem") == std::string::npos &&
				base.find("warped_hyphae") == std::string::npos)
			continue;
		Block log = structures::resolve_schem_block(v.block);
		if (log == Block{})
			continue;
		int px = v.x, pz = v.z;
		switch (rotation & 3u) {
		case 1:
			px = s.length - 1 - v.z;
			pz = v.x;
			break;
		case 2:
			px = s.width - 1 - v.x;
			pz = s.length - 1 - v.z;
			break;
		case 3:
			px = v.z;
			pz = s.width - 1 - v.x;
			break;
		default:
			break;
		}
		const auto key = std::make_pair(x + px - center_x, z + pz - center_z);
		const int top = ground_y + v.y;
		auto it = bottoms.find(key);
		if (it == bottoms.end() || top < it->second.top)
			bottoms[key] = {top, log};
	}
	// Only low trunk columns root into terrain. Branches/canopy logs must not
	// generate vertical pillars down through a slope.
	int lowest = std::numeric_limits<int>::max();
	for (const auto &[key, root] : bottoms)
		lowest = std::min(lowest, root.top);
	for (const auto &[key, root] : bottoms) {
		if (root.top > lowest + 2 || editor.is_lc_water(key.first, key.second))
			continue;
		const int local_ground = editor.get_absolute_y(key.first, 0, key.second);
		const int from = std::max(local_ground, root.top - 1 - 64);
		for (int wy = from; wy < root.top; ++wy)
			editor.set_block_absolute(root.block, key.first, wy, key.second);
	}
	return true;
}
}
