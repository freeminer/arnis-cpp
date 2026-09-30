#pragma once

#include "facade.h"
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

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
	Owner owner;
	std::uint32_t columns = 0;
	std::uint32_t rows = 0;
	std::string tier;
	std::filesystem::path color_png;
	std::filesystem::path texture_png;
	std::vector<std::string> views;
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
void clear();
bool has_building(std::uint64_t way_id);
std::optional<imgops::Rgb> building_color(std::uint64_t way_id);
} // namespace arnis::mapillary::facades
