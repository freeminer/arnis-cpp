#pragma once

#include "facade.h"
#include "../../arnis_block.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace arnis
{
class ProcessedElement;
namespace world_editor
{
struct WorldEditor;
}
}
struct XZBBox;

namespace arnis::mapillary::facades
{
enum class OwnerKind : std::uint8_t
{
	Way,
	Relation
};

struct Owner
{
	OwnerKind kind = OwnerKind::Way;
	std::uint64_t id = 0;
};

struct ExportWall
{
	struct Edge
	{
		std::uint64_t node_a = 0, node_b = 0;
		std::int32_t col0 = 0, col1 = 0;
		std::optional<std::pair<double, double>> span_m;
	};
	Owner owner;
	std::uint32_t columns = 0;
	std::uint32_t rows = 0;
	std::string tier;
	std::filesystem::path color_png;
	std::filesystem::path texture_png;
	std::vector<std::string> views;
	double col0_m = 0.0;
	std::vector<Edge> edges;
	std::vector<std::array<std::uint8_t, 4>> cells;
	std::vector<imgops::Rgb> row_bands;
	std::uint32_t texture_width = 0, texture_height = 0;
	std::vector<std::array<std::uint8_t, 4>> texture_pixels;
};

struct ExportBuilding
{
	Owner owner;
	std::optional<imgops::Rgb> building_color;
	std::vector<ExportWall> walls;
};

struct Export
{
	std::vector<ExportBuilding> buildings;
	std::vector<std::string> image_ids;
};

// Reads the Rust orthofacade export format. Invalid/unrelated JSON files are
// ignored just like the Rust loader; filesystem and parse failures are
// returned so callers can clear a stale facade store rather than reuse it.
std::optional<Export> load_export(
		const std::filesystem::path &, std::string *error = nullptr);
void install_export(Export);
void set_displays_enabled(bool enabled);
void project_export(
		const std::vector<arnis::ProcessedElement> &, const XZBBox &, double scale);
void clear();
bool has_building(std::uint64_t way_id);
std::optional<imgops::Rgb> building_color(std::uint64_t way_id);
bool photo_column(int x, int z, std::uint64_t way_id);
std::size_t collect_displays(arnis::world_editor::WorldEditor &, std::uint64_t way_id,
		int start_y_offset, int abs_terrain_offset, int building_height,
		std::uint32_t pixels_per_block);
std::optional<Block> block_at(int x, int y, int z, int start_y_offset,
		std::uint64_t way_id, Block window_block);
std::optional<Block> band_block_at(
		int x, int y, int z, int start_y_offset, std::uint64_t way_id);
} // namespace arnis::mapillary::facades
