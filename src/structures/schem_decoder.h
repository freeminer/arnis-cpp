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
// Rust schematic::StructureSchematic equivalent, retaining the source voxel
// list while exposing deterministic anchor/extent calculations.
struct StructureSchematic
{
	int width = 0, length = 0;
	std::vector<SchemVoxel> voxels;
	int anchor_x = 0, anchor_z = 0, max_extent = 0;
	StructureSchematic centered() const;
	StructureSchematic base_anchored() const;
};
enum class SchemAnchor
{
	Offset,
	Centered,
	BaseCentroid
};

SchemDocument decode_sponge_schem(const std::vector<std::uint8_t> &gzip_data);
SchemDocument load_palettized(const std::vector<std::uint8_t> &gzip_data);
StructureSchematic load_structure(const std::vector<std::uint8_t> &gzip_data);
StructureSchematic structure_schematic(const SchemDocument &document);
Block resolve_schem_block(const std::string &name);
BlockWithProperties resolve_schem_block_with_properties(const std::string &name);
// Free-yaw counterpart of the quarter-turn schematic placement path.  It
// samples destination columns so oblique placements remain gap-free.
bool place_schem_document_yaw(world_editor::WorldEditor &, const SchemDocument &,
		int base_x, int base_y, int base_z, double yaw_degrees,
		double pitch_degrees = 0.0);
std::unordered_map<std::string, std::string> rotate_schem_properties(
		const std::unordered_map<std::string, std::string> &input, unsigned rotation);
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
