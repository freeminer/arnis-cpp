#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <filesystem>
#include <unordered_map>
namespace arnis::world_editor
{
struct WorldEditor;
}
#include "../block_definitions.h"

namespace arnis::structures
{

struct SchemVoxel
{
	int x, y, z;
	std::string block;
	std::unordered_map<std::string, std::string> properties;
};
struct SchemEntity
{
	int x = 0, y = 0, z = 0;
	std::vector<std::uint8_t> nbt;
};
struct SchemDocument
{
	int width = 0, height = 0, length = 0;
	// Sponge schematic origin (the Offset tag), retained so callers can
	// reproduce the source placement rather than silently anchoring at 0,0,0.
	int offset_x = 0, offset_y = 0, offset_z = 0;
	std::vector<SchemVoxel> voxels;
	std::vector<SchemEntity> entities;
};
enum class SchemAnchor
{
	Offset,
	Centered,
	BaseCentroid
};

SchemDocument decode_sponge_schem(const std::vector<std::uint8_t> &gzip_data);
Block resolve_schem_block(const std::string &name);
BlockWithProperties resolve_schem_block_with_properties(const std::string &name);
bool place_schem_file(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz);
bool place_schem_file_rotated(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz, unsigned rotation,
		const Block *ground = nullptr);
bool place_schem_file_anchored(world_editor::WorldEditor &editor,
		const std::filesystem::path &file, int ox, int oy, int oz, unsigned rotation,
		SchemAnchor anchor);
bool place_named_schem(world_editor::WorldEditor &editor, const std::string &name, int ox,
		int oy, int oz, unsigned rotation = 0, const Block *ground = nullptr);

} // namespace arnis::structures
