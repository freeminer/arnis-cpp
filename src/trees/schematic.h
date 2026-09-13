#pragma once
#include <filesystem>
#include <vector>
#include "../structures/schem_decoder.h"
#include "tree_library.h"
#include "../floodfill_cache.h"
namespace arnis::trees
{
using Schematic = structures::SchemDocument;
std::optional<Block> map_block(const std::string &name);
int min_log_y(const Schematic &schem);
bool place_schematic_tree(world_editor::WorldEditor &editor, const Schematic &schem,
		int anchor_x, int anchor_z, int base_y, unsigned rot,
		const std::vector<Block> &blacklist, const BuildingFootprintBitmap *footprints,
		int y_offset);
Schematic tree_only(const Schematic &schem);
Schematic load_schem(const std::filesystem::path &file);
TreeSize schematic_size(const Schematic &schem);
// One deterministic trunk position per lattice cell; shared by regional and
// canopy-driven placement so streamed tiles cannot disagree at their seam.
std::pair<int, int> trunk_slot_s(int x, int z, int spacing);
bool place_schematic(world_editor::WorldEditor &editor, const Schematic &schem, int x,
		int y, int z, unsigned rotation = 0);
bool place_schematic_rooted(world_editor::WorldEditor &editor, const Schematic &schem,
		int x, int ground_y, int z, unsigned rotation = 0);
}
