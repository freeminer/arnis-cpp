#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <functional>
#include <unordered_map>
#include <optional>
#include "../../../arnis_block.h"
namespace arnis::world_editor
{
struct WorldEditor;
}
#include "../block_definitions.h"

namespace arnis::structures
{

using SchemProperties = std::unordered_map<std::string, std::string>;

struct SchemVoxel
{
	int x, y, z;
	std::string block;
	Block resolved_block;
	// Palette states are immutable and shared by all voxels using the same
	// Sponge block state, instead of allocating a string map for every voxel.
	std::shared_ptr<const SchemProperties> properties;

	const SchemProperties &state_properties() const
	{
		static const SchemProperties empty;
		return properties ? *properties : empty;
	}
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
	std::vector<SchemEntity> entities;
	int anchor_x = 0, anchor_z = 0, max_extent = 0;
	StructureSchematic centered() const &;
	StructureSchematic centered() &&;
	StructureSchematic base_anchored() const &;
	StructureSchematic base_anchored() &&;
};

struct ColumnVoxel
{
	std::int16_t x = 0, y = 0, z = 0;
	std::uint8_t palette_slot = 0;
};

// Rust schematic::PalettizedSchematic: one entry per accepted source palette
// ID, with the lowest mapped voxel layer shifted to zero.
struct PalettizedSchematic
{
	int width = 0, length = 0;
	std::vector<BlockWithProperties> palette;
	std::vector<ColumnVoxel> voxels;
};

// Palette-indexed layout for free-yaw models. Column ranges are contiguous,
// matching Rust's ColumnSchematic so placement does not rebuild a map per use.
struct ColumnSchematic
{
	int width = 0, length = 0;
	std::vector<BlockWithProperties> palette;
	std::vector<ColumnVoxel> voxels;
	std::vector<std::pair<std::uint32_t, std::uint32_t>> columns;

	static ColumnSchematic from_document(const SchemDocument &document);
	std::pair<const ColumnVoxel *, std::size_t> column(int x, int z) const;
};

// Decode and derive reusable placement forms once per normalized asset path.
std::shared_ptr<const StructureSchematic> load_structure_cached(
		const std::filesystem::path &file);
// Rust's OnceLock<Option<StructureSchematic>> counterpart: cache both successful
// parses and missing/invalid assets so repeated placements do not retry failures.
std::shared_ptr<const StructureSchematic> try_load_structure_cached(
		const std::filesystem::path &file);
std::shared_ptr<const StructureSchematic> load_centered_structure_cached(
		const std::filesystem::path &file);
std::shared_ptr<const PalettizedSchematic> load_palettized_cached(
		const std::filesystem::path &file);
std::shared_ptr<const ColumnSchematic> load_column_schematic_cached(
		const std::filesystem::path &file);
enum class SchemAnchor
{
	Offset,
	Centered,
	BaseCentroid
};

// General structure schematics use signed Short dimensions; cave packs follow
// the cave loader's unsigned-Short convention while still accepting Int tags.
enum class SchemDimensionMode
{
	Signed,
	CavePack
};

// Optional projection for consumers that need only a subset of schematic
// blocks. Unmapped palette entries and block entities are skipped while
// decoding, avoiding a full general-purpose document allocation.
struct SchemDecodeOptions
{
	SchemDimensionMode dimension_mode = SchemDimensionMode::Signed;
	bool include_entities = true;
	std::function<std::optional<Block>(const std::string &)> block_mapper;
	std::function<void(int, int, int, Block)> voxel_sink;
	// Compact structure consumers retain source palette slots instead of one
	// block-state string and properties pointer per voxel. Supply both callbacks.
	std::function<std::optional<std::uint8_t>(const std::string &)> palette_mapper;
	std::function<void(int, int, int, std::uint8_t)> indexed_voxel_sink;
	// Zero keeps the general decoder's dimension range; compact structures use
	// Rust's i16 coordinate limit for all three dimensions.
	int max_dimension = 0;
};

SchemDocument decode_sponge_schem(const std::vector<std::uint8_t> &gzip_data,
		SchemDimensionMode dimension_mode = SchemDimensionMode::Signed);
SchemDocument decode_sponge_schem(
		const std::vector<std::uint8_t> &gzip_data, const SchemDecodeOptions &options);
// Decode a file-backed schematic once per normalized path. This preserves the
// Rust OnceLock behavior while allowing callers to supply an external asset root.
std::shared_ptr<const SchemDocument> load_schem_cached(const std::filesystem::path &file);
PalettizedSchematic load_palettized(const std::vector<std::uint8_t> &gzip_data);
StructureSchematic load_structure(const std::vector<std::uint8_t> &gzip_data);
ColumnSchematic load_column_schematic(const std::vector<std::uint8_t> &gzip_data);
StructureSchematic structure_schematic(const SchemDocument &document);
Block resolve_schem_block(const std::string &name);
BlockWithProperties resolve_schem_block_with_properties(const std::string &name);
// Free-yaw counterpart of the quarter-turn schematic placement path.  It
// samples destination columns so oblique placements remain gap-free.
bool place_schem_document_yaw(world_editor::WorldEditor &, const SchemDocument &,
		int base_x, int base_y, int base_z, double yaw_degrees,
		double pitch_degrees = 0.0);
// Rust schematic API counterparts for callers that already decoded a
// StructureSchematic instead of retaining the Sponge document wrapper.
bool place_structure(world_editor::WorldEditor &, const StructureSchematic &, int base_x,
		int base_y, int base_z, unsigned rotation = 0, const Block *ground = nullptr);
bool place_structure_yaw(world_editor::WorldEditor &, const StructureSchematic &,
		int base_x, int base_y, int base_z, double yaw_degrees,
		double pitch_degrees = 0.0);
bool place_structure_yaw(world_editor::WorldEditor &, const ColumnSchematic &, int base_x,
		int base_y, int base_z, double yaw_degrees, double pitch_degrees = 0.0);
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
