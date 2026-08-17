#include "schematic.h"
#include "../../../arnis_adapter.h"
#include "../land_cover/land_cover.h"
#include <fstream>
#include <algorithm>
#include <limits>
#include <map>
namespace arnis::trees
{
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
	for (const auto &v : in.voxels) {
		const auto &n = v.block;
		if (n.find("_log") != std::string::npos || n.find("_wood") != std::string::npos ||
				n.find("_leaves") != std::string::npos ||
				n.find("_roots") != std::string::npos ||
				n.find("leaves") != std::string::npos ||
				n.find("vine") != std::string::npos || n == "minecraft:moss_block" ||
				n == "minecraft:moss_carpet" ||
				n.find("mangrove_propagule") != std::string::npos ||
				n.find("bamboo_block") != std::string::npos ||
				n.find("warped_stem") != std::string::npos ||
				n.find("warped_hyphae") != std::string::npos)
			out.voxels.push_back(v);
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
			out.height = std::max(0, out.height - min_y);
		}
	}
	return out;
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
	auto euclid_div = [s](int v) { return v >= 0 ? v / s : -(((-v) + s - 1) / s); };
	const int cx = euclid_div(x), cz = euclid_div(z);
	const auto h = land_cover::coord_hash(cx * 0x1f1f + 17, cz * 0x2b2b + 91);
	return {cx * s + int(h & 1U), cz * s + int((h >> 1) & 1U)};
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
