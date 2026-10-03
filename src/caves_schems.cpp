#include "caves_carver.h"

#include "block_definitions.h"
#include "caves_rng.h"
#include "caves_theme.h"
#include "args.h"
#include "structures/schem_decoder.h"
#include "trees/schematic.h"
#include "../../arnis_block.h"
#include "../../arnis_world_editor.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <iterator>

#include <nlohmann/json.hpp>

namespace arnis::caves
{
namespace
{
using namespace block_definitions;
using json = nlohmann::json;

enum class Orientation
{
	Up,
	Down,
	Cover,
	Embed,
	EmbedFloor,
	Any
};

struct PaletteEntry
{
	Block block;
	std::unordered_map<std::string, std::string> properties;
	bool fluid = false;
	bool air = false;
};

struct CaveSchematic
{
	int width = 0, height = 0, length = 0;
	std::vector<structures::SchemVoxel> voxels;
	std::vector<PaletteEntry> palette;
	std::size_t solid_count = 0;
};

struct Family
{
	Orientation orientation = Orientation::Any;
	std::vector<std::string> biomes;
	std::uint32_t weight = 1;
	int sink = 0;
	std::vector<CaveSchematic> schematics;
};

struct CavePack
{
	std::vector<Family> families;
};

std::mutex pack_mutex;
std::unordered_map<std::string, std::shared_ptr<const CavePack>> pack_cache;

std::string base_block_name(std::string name)
{
	if (const auto colon = name.find(':'); colon != std::string::npos)
		name.erase(0, colon + 1);
	if (const auto states = name.find('['); states != std::string::npos)
		name.resize(states);
	return name;
}

std::optional<Block> map_cave_block(const structures::SchemVoxel &voxel)
{
	const auto n = base_block_name(voxel.block);
	if (n == "air" || n == "cave_air" || n == "void_air")
		return AIR;
	if (n == "ice")
		return ICE;
	if (n == "packed_ice")
		return PACKED_ICE;
	if (n == "blue_ice")
		return BLUE_ICE;
	if (n == "snow_block")
		return SNOW_BLOCK;
	if (n == "snow")
		return SNOW_LAYER;
	if (n == "powder_snow")
		return POWDER_SNOW;
	if (n == "cobweb")
		return COBWEB;
	if (n == "glow_lichen")
		return GLOW_LICHEN;
	if (n == "amethyst_block")
		return AMETHYST_BLOCK;
	if (n == "budding_amethyst")
		return BUDDING_AMETHYST;
	if (n == "amethyst_cluster")
		return AMETHYST_CLUSTER;
	if (n == "small_amethyst_bud")
		return SMALL_AMETHYST_BUD;
	if (n == "medium_amethyst_bud")
		return MEDIUM_AMETHYST_BUD;
	if (n == "large_amethyst_bud")
		return LARGE_AMETHYST_BUD;
	if (n == "dripstone_block")
		return DRIPSTONE_BLOCK;
	if (n == "pointed_dripstone")
		return POINTED_DRIPSTONE;
	if (n == "water")
		return WATER;
	if (n == "lava")
		return LAVA;
	if (n == "clay")
		return CLAY;
	if (n == "big_dripleaf")
		return BIG_DRIPLEAF;
	if (n == "big_dripleaf_stem")
		return BIG_DRIPLEAF_STEM;
	if (n == "small_dripleaf") {
		const auto half = voxel.properties.find("half");
		return half != voxel.properties.end() && half->second == "upper"
					   ? SMALL_DRIPLEAF_UPPER
					   : SMALL_DRIPLEAF_LOWER;
	}
	return std::nullopt;
}

std::optional<CaveSchematic> decode_cave_schematic(const std::filesystem::path &file)
{
	std::ifstream in(file, std::ios::binary);
	if (!in)
		return std::nullopt;
	std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)), {});
	structures::SchemDocument document;
	try {
		document = structures::decode_sponge_schem(bytes);
	} catch (...) {
		return std::nullopt;
	}
	if (document.width <= 0 || document.height <= 0 || document.length <= 0)
		return std::nullopt;
	CaveSchematic out;
	out.width = document.width;
	out.height = document.height;
	out.length = document.length;
	int min_solid_y = std::numeric_limits<int>::max();
	for (const auto &voxel : document.voxels) {
		if (base_block_name(voxel.block) == "acacia_leaves")
			continue; // Cave packs use this as a non-voxel marker.
		auto block = map_cave_block(voxel);
		if (!block)
			continue;
		const bool air = *block == AIR;
		out.solid_count += !air;
		if (!air)
			min_solid_y = std::min(min_solid_y, voxel.y);
		out.voxels.push_back(voxel);
		auto properties = voxel.properties;
		for (auto it = properties.begin(); it != properties.end();) {
			if ((it->first == "waterlogged" && it->second == "false") ||
					it->first == "level" || it->first == "distance" ||
					it->first == "persistent")
				it = properties.erase(it);
			else
				++it;
		}
		out.palette.push_back(
				{*block, std::move(properties), *block == WATER || *block == LAVA, air});
	}
	if (out.voxels.empty() || out.solid_count == 0)
		return std::nullopt;
	// The Rust pack loader normalizes padded assets so their lowest solid sits at y=0.
	if (min_solid_y > 0) {
		for (auto &voxel : out.voxels)
			voxel.y -= min_solid_y;
		for (std::size_t i = out.voxels.size(); i-- > 0;)
			if (out.voxels[i].y < 0) {
				out.voxels.erase(out.voxels.begin() + i);
				out.palette.erase(out.palette.begin() + i);
			}
	}
	out.height = 1;
	for (std::size_t i = 0; i < out.voxels.size(); ++i)
		if (!out.palette[i].air)
			out.height = std::max(out.height, out.voxels[i].y + 1);
	std::map<std::tuple<int, int, int>, Block> fluids;
	for (std::size_t i = 0; i < out.voxels.size(); ++i)
		if (out.palette[i].fluid)
			fluids.emplace(std::tuple{out.voxels[i].x, out.voxels[i].y, out.voxels[i].z},
					out.palette[i].block);
	constexpr std::array<std::array<int, 3>, 6> neighbours = {
			{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
	for (const auto &[position, block] : fluids) {
		const auto [x, y, z] = position;
		const Block opposite = block == WATER ? LAVA : WATER;
		for (const auto &offset : neighbours) {
			const auto [dx, dy, dz] = offset;
			const auto it = fluids.find({x + dx, y + dy, z + dz});
			if (it != fluids.end() && it->second == opposite)
				return std::nullopt;
		}
	}
	return out;
}

Orientation parse_orientation(const std::string &value)
{
	if (value == "up")
		return Orientation::Up;
	if (value == "down")
		return Orientation::Down;
	if (value == "cover")
		return Orientation::Cover;
	if (value == "embed")
		return Orientation::Embed;
	if (value == "embed_floor")
		return Orientation::EmbedFloor;
	return Orientation::Any;
}

std::optional<CavePack> load_pack(const std::filesystem::path &root)
{
	std::ifstream in(root / "cave_pack.json");
	if (!in)
		return std::nullopt;
	json manifest;
	try {
		in >> manifest;
	} catch (...) {
		return std::nullopt;
	}
	if (!manifest.is_object() || !manifest.contains("families") ||
			!manifest["families"].is_array())
		return std::nullopt;
	CavePack pack;
	for (const auto &source : manifest["families"]) {
		if (!source.is_object() || !source.contains("files") ||
				!source["files"].is_array())
			continue;
		Family family;
		family.orientation = parse_orientation(source.value("orientation", "any"));
		family.weight = std::max<std::uint32_t>(1, source.value("weight", 1u));
		family.sink = std::max(0, source.value("sink", 0));
		if (source.contains("biomes") && source["biomes"].is_array())
			for (const auto &tag : source["biomes"])
				if (tag.is_string())
					family.biomes.push_back(tag.get<std::string>());
		for (const auto &entry : source["files"]) {
			if (!entry.is_string())
				continue;
			const auto relative = std::filesystem::path(entry.get<std::string>());
			if (relative.is_absolute())
				continue;
			const auto file = root / relative;
			const auto normalized = file.lexically_normal();
			// Pack paths are untrusted; reject traversal outside the configured root.
			const auto rel = normalized.lexically_relative(root.lexically_normal());
			if (rel.empty() || *rel.begin() == "..")
				continue;
			auto schem = decode_cave_schematic(normalized);
			if (schem && schem->solid_count >=
								 (family.orientation == Orientation::Cover ? 3u : 1u))
				family.schematics.push_back(std::move(*schem));
		}
		if (!family.schematics.empty())
			pack.families.push_back(std::move(family));
	}
	return pack;
}

std::shared_ptr<const CavePack> get_pack(const std::filesystem::path &configured_root)
{
	if (configured_root.empty())
		return {};
	const auto root = configured_root.lexically_normal();
	const auto key = root.string();
	std::lock_guard<std::mutex> lock(pack_mutex);
	auto it = pack_cache.find(key);
	if (it == pack_cache.end()) {
		auto loaded = load_pack(root);
		auto pack = loaded ? std::make_shared<CavePack>(std::move(*loaded)) : nullptr;
		it = pack_cache.emplace(key, std::move(pack)).first;
	}
	return it->second;
}

bool is_air(const world_editor::WorldEditor &editor, int x, int y, int z)
{
	return !editor.block_exists_absolute(x, y, z);
}

bool place_formation(world_editor::WorldEditor &editor, const CaveSchematic &schem,
		Orientation orientation, int sink, int ax, int ay, int az, unsigned rotation,
		int floor_y, const CaveRect &region, const std::vector<Block> &rock)
{
	const int fw = (rotation & 1) ? schem.length : schem.width;
	const int fl = (rotation & 1) ? schem.width : schem.length;
	const int cx = (fw - 1) / 2, cz = (fl - 1) / 2;
	const bool embedded =
			orientation == Orientation::Embed || orientation == Orientation::EmbedFloor;
	const int base_y =
			orientation == Orientation::Down || orientation == Orientation::Embed
					? ay - (schem.height - 1)
			: orientation == Orientation::EmbedFloor ? ay - schem.height
													 : ay - sink;
	std::vector<Block> rock_air = rock;
	rock_air.push_back(AIR);
	constexpr std::array<std::array<int, 3>, 6> neighbours = {
			{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}}};
	for (const auto &v : schem.voxels) {
		const auto index = static_cast<std::size_t>(&v - schem.voxels.data());
		const auto &entry = schem.palette[index];
		if (!entry.fluid)
			continue;
		const auto [rx, rz] =
				trees::rotate_xz(v.x, v.z, schem.width, schem.length, rotation);
		const int wx = ax + rx - cx, wy = base_y + v.y, wz = az + rz - cz;
		const Block opposite = entry.block == WATER ? LAVA : WATER;
		for (const auto &offset : neighbours) {
			const auto [dx, dy, dz] = offset;
			if (editor.check_for_block_absolute(
						wx + dx, wy + dy, wz + dz, std::vector<Block>{opposite}))
				return false;
		}
	}
	std::size_t visible = 0, tested = 0;
	for (const auto &v : schem.voxels) {
		const auto &p = schem.palette[&v - schem.voxels.data()];
		if (p.air || embedded)
			continue;
		const auto [rx, rz] =
				trees::rotate_xz(v.x, v.z, schem.width, schem.length, rotation);
		const int wy = base_y + v.y;
		if (wy < ay)
			continue;
		++tested;
		if (is_air(editor, ax + rx - cx, wy, az + rz - cz))
			++visible;
	}
	if (!embedded && (!tested || visible * 2 < tested))
		return false;
	// Embed-floor basins are recessed into the floor. Reject edge anchors where
	// a solid bottom voxel would be left hanging above absent terrain.
	if (orientation == Orientation::EmbedFloor) {
		for (const auto &v : schem.voxels) {
			const auto index = static_cast<std::size_t>(&v - schem.voxels.data());
			if (v.y != 0 || schem.palette[index].air)
				continue;
			const auto [rx, rz] =
					trees::rotate_xz(v.x, v.z, schem.width, schem.length, rotation);
			const int wx = ax + rx - cx, wz = az + rz - cz;
			if (wx < region.min_x || wx > region.max_x || wz < region.min_z ||
					wz > region.max_z ||
					!editor.block_exists_absolute(wx, base_y - 1, wz))
				return false;
		}
	}
	std::vector<std::tuple<int, int, int, Block>> placed_solids;
	std::vector<std::tuple<int, int, int, Block>> top_row;
	for (const auto &v : schem.voxels) {
		const auto index = static_cast<std::size_t>(&v - schem.voxels.data());
		const auto &entry = schem.palette[index];
		const auto [rx, rz] =
				trees::rotate_xz(v.x, v.z, schem.width, schem.length, rotation);
		const int wx = ax + rx - cx, wy = base_y + v.y, wz = az + rz - cz;
		if (wx < region.min_x || wx > region.max_x || wz < region.min_z ||
				wz > region.max_z)
			continue;
		if (wy <= floor_y)
			continue;
		if (wy > editor.get_ground_level(wx, wz) - 8)
			continue;
		if (entry.air) {
			if (orientation == Orientation::Embed)
				editor.set_block_absolute(AIR, wx, wy, wz, rock, std::nullopt);
			continue;
		}
		const auto properties =
				structures::rotate_schem_properties(entry.properties, rotation);
		const std::vector<Block> air_only{AIR};
		const auto &allowed = (embedded || wy < ay) ? rock_air : air_only;
		editor.set_block_with_properties_absolute(
				BlockWithProperties{entry.block, properties}, wx, wy, wz,
				std::optional<std::vector<Block>>{allowed}, std::nullopt);
		if (editor.check_for_block_absolute(
					wx, wy, wz, std::vector<Block>{entry.block})) {
			placed_solids.emplace_back(wx, wy, wz, entry.block);
			if (embedded && v.y == schem.height - 1 && !entry.fluid)
				top_row.emplace_back(wx, wy, wz, entry.block);
		}
	}
	// Remove isolated pieces left behind when the cave roof clipped their support.
	for (const auto &[wx, wy, wz, block] : placed_solids) {
		bool supported = false;
		for (const auto &offset : neighbours) {
			const auto [dx, dy, dz] = offset;
			if (editor.block_exists_absolute(wx + dx, wy + dy, wz + dz)) {
				supported = true;
				break;
			}
		}
		if (!supported)
			editor.set_block_absolute(
					AIR, wx, wy, wz, std::vector<Block>{block}, std::nullopt);
	}
	if (orientation == Orientation::EmbedFloor)
		for (const auto &[wx, wy, wz, block] : top_row)
			for (int depth = 1; depth <= 3; ++depth) {
				if (editor.block_exists_absolute(wx, wy - depth, wz))
					break;
				editor.set_block_absolute(
						block, wx, wy - depth, wz, std::vector<Block>{AIR}, std::nullopt);
			}
	return true;
}

} // namespace

void stamp_schematics_region(world_editor::WorldEditor &editor, const CaveRect &region,
		std::int64_t seed, int floor_y, const Args &args)
{
	const auto pack = get_pack(editor.get_cave_asset_root());
	if (!pack || pack->families.empty())
		return;
	const std::vector<Block> rock{STONE, DEEPSLATE, TUFF, COBBLED_DEEPSLATE, GRANITE,
			DIORITE, ANDESITE, DIRT, GRAVEL};
	auto biome_amounts = BiomeAmounts::defaults();
	if (args.cave_biomes)
		biome_amounts = BiomeAmounts::parse(*args.cave_biomes);
	ThemeSelector themes(seed, biome_amounts, floor_y);
	const auto [write_min_y, write_max_y] = editor.writable_y_bounds();
	const int min_chunk_x =
			region.min_x >= 0 ? region.min_x / 16 : (region.min_x - 15) / 16;
	const int max_chunk_x =
			region.max_x >= 0 ? region.max_x / 16 : (region.max_x - 15) / 16;
	const int min_chunk_z =
			region.min_z >= 0 ? region.min_z / 16 : (region.min_z - 15) / 16;
	const int max_chunk_z =
			region.max_z >= 0 ? region.max_z / 16 : (region.max_z - 15) / 16;
	for (int cx = min_chunk_x; cx <= max_chunk_x; ++cx) {
		for (int cz = min_chunk_z; cz <= max_chunk_z; ++cz) {
			const std::uint64_t chunk_seed =
					std::bit_cast<std::uint64_t>(seed) ^
					static_cast<std::uint64_t>(static_cast<std::int64_t>(cx)) *
							341873128712ULL ^
					static_cast<std::uint64_t>(static_cast<std::int64_t>(cz)) *
							132897987541ULL ^
					0x5CE35CE3ULL;
			auto random = XoroRandom::from_seed(std::bit_cast<std::int64_t>(chunk_seed));
			int placed = 0;
			for (int attempt = 0; attempt < 6 && placed < 2; ++attempt) {
				const int x = cx * 16 + random.next_int(16);
				const int z = cz * 16 + random.next_int(16);
				if (x < region.min_x || x > region.max_x || z < region.min_z ||
						z > region.max_z)
					continue;
				const int surface = editor.get_ground_level(x, z);
				const int roof = std::min(surface - 8, write_max_y);
				const int bottom = std::max(floor_y + 6, write_min_y);
				std::vector<int> floors, ceilings;
				for (int y = bottom; y <= roof; ++y) {
					if (!is_air(editor, x, y, z))
						continue;
					if (!is_air(editor, x, y - 1, z) &&
							editor.check_for_block_absolute(x, y - 1, z, rock))
						floors.push_back(y);
					if (y < roof && !is_air(editor, x, y + 1, z) &&
							editor.check_for_block_absolute(x, y + 1, z, rock))
						ceilings.push_back(y);
				}
				auto open = [&](int y) {
					for (int dx = -1; dx <= 1; ++dx)
						for (int dz = -1; dz <= 1; ++dz)
							if (!is_air(editor, x + dx, y, z + dz))
								return false;
					return true;
				};
				std::erase_if(floors, [&](int y) { return !open(y); });
				std::erase_if(ceilings, [&](int y) { return !open(y); });
				if (floors.empty() && ceilings.empty())
					continue;
				const int probe_y = floors.empty() ? ceilings.front() : floors.front();
				const auto tag = cave_theme_tag(themes.at(x, probe_y, z, surface));
				const bool themed = random.next_int(3) != 0;
				std::vector<const Family *> candidates;
				std::uint64_t total_weight = 0;
				for (const auto &family : pack->families) {
					const bool match =
							std::find(family.biomes.begin(), family.biomes.end(), tag) !=
							family.biomes.end();
					const bool any = std::find(family.biomes.begin(), family.biomes.end(),
											 "any") != family.biomes.end();
					if (match || (any && (!themed || std::string(tag) == "any"))) {
						candidates.push_back(&family);
						total_weight += family.weight;
					}
				}
				if (candidates.empty() || !total_weight)
					continue;
				std::uint64_t roll = static_cast<std::uint32_t>(random.next_int(
						static_cast<std::int32_t>(std::min<std::uint64_t>(total_weight,
								std::numeric_limits<std::int32_t>::max()))));
				const Family *family = candidates.front();
				for (const auto *candidate : candidates) {
					if (roll < candidate->weight) {
						family = candidate;
						break;
					}
					roll -= candidate->weight;
				}
				const auto &schem = family->schematics[random.next_int(
						static_cast<std::int32_t>(family->schematics.size()))];
				const unsigned rotation = static_cast<unsigned>(random.next_int(4));
				int anchor_y = 0;
				switch (family->orientation) {
				case Orientation::Down:
					if (ceilings.empty())
						continue;
					anchor_y = ceilings[random.next_int(
							static_cast<std::int32_t>(ceilings.size()))];
					break;
				case Orientation::Cover:
					if (!floors.empty() && (ceilings.empty() || random.next_int(10) < 7))
						anchor_y = floors[random.next_int(
								static_cast<std::int32_t>(floors.size()))];
					else if (!ceilings.empty())
						anchor_y = ceilings[random.next_int(
								static_cast<std::int32_t>(ceilings.size()))];
					else
						continue;
					break;
				default:
					if (floors.empty())
						continue;
					anchor_y = floors[random.next_int(
							static_cast<std::int32_t>(floors.size()))];
					break;
				}
				if (place_formation(editor, schem, family->orientation, family->sink, x,
							anchor_y, z, rotation, floor_y, region, rock))
					++placed;
			}
		}
	}
}

} // namespace arnis::caves
