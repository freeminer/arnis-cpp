#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "../block_definitions.h"
#include "building_region.h"
#include "buildings.h"

namespace arnis::buildings::rooftops
{

struct RoofMembrane
{
	Block base;
	Block patch;
	Block repair;
	std::optional<Block> margin;
	std::uint64_t seed{};

	Block block_at(int x, int z) const;
	Block block_at_edge(int x, int z, int edge) const;
	bool is_green() const;
	static RoofMembrane uniform(Block block) { return {block, block, block, {}, 0}; }
};

// Shared roof-layout state: edge distance is computed once, and every rooftop
// fixture reserves its footprint plus a one-cell service clearance.
struct RoofDistanceGrid
{
	int min_x{}, min_z{}, width{}, height{};
	std::vector<int> dist;

	int get(int x, int z) const;
};

struct RoofDeck
{
	RoofDistanceGrid grid;
	std::vector<bool> taken;
	int deepest{-1};
	int surface_y{};

	static std::optional<RoofDeck> make(
			const std::vector<std::pair<int, int>> &cells, int surface_y);
	int edge(int x, int z) const;
	bool is_taken(int x, int z) const;
	void reserve(int x, int z, int width, int depth);
	bool fits(int x, int z, int width, int depth, int min_edge) const;
	std::optional<std::pair<int, int>> find_near(std::pair<int, int> target, int width,
			int depth, int min_edge, int reach) const;
	std::size_t count_at_least(int min_edge) const;
	std::vector<std::pair<int, int>> cells_at_least(int min_edge) const;
	std::pair<int, int> heart() const;
};

struct RooftopBrief
{
	building_style::BuildingRegion region{building_style::BuildingRegion::Other};
	BuildingCategory category{BuildingCategory::Default};
	ArchEra era{ArchEra::Unknown};
	int height{};
	int floor_cycle{3};
	Block wall;
	double scale{1.0};
	std::uint64_t seed{};
	bool residential{};
	bool whole_building{};
	bool sparse{};
	bool green{};

	int scaled(int value) const;
	int storeys() const;
	bool plant_heavy() const;
};

RoofMembrane pick_membrane(building_style::BuildingRegion region,
		building_style::MaterialRegion materials, BuildingCategory category, ArchEra era,
		int building_height, std::uint64_t style_seed);

Block coping_for(const WorldEditor &editor, Block wall, Block accent, std::uint64_t seed);
BlockWithProperties bottom_slab(Block slab);

void fit_out_terrace(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells,
		int surface_y, std::uint64_t seed, double scale);
void fit_out_terrace(WorldEditor &editor, RoofDeck &deck, const RooftopBrief &brief);
void fit_out_roof(WorldEditor &editor, RoofDeck &deck, const RooftopBrief &brief);

} // namespace arnis::buildings::rooftops
