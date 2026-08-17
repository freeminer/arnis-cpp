#pragma once

#include <array>
#include <vector>
#include <span>
#include <utility>
#include <random>
#include <optional>
#include <functional>
#include <cstdint>

#include "../../../arnis_adapter.h"
#include "../deterministic_rng.h"
#include "../floodfill_cache.h"
#include "bridges.h"

namespace arnis
{

struct Coord
{
	int x;
	int y;
	int z;
};

static constexpr std::array<Coord, 8> ROUND1_PATTERN = {{
		{-2, 0, 0},
		{2, 0, 0},
		{0, 0, -2},
		{0, 0, 2},
		{-1, 0, -1},
		{1, 0, 1},
		{1, 0, -1},
		{-1, 0, 1},
}};

static constexpr std::array<Coord, 12> ROUND2_PATTERN = {{
		{3, 0, 0},
		{2, 0, -1},
		{2, 0, 1},
		{1, 0, -2},
		{1, 0, 2},
		{-3, 0, 0},
		{-2, 0, -1},
		{-2, 0, 1},
		{-1, 0, 2},
		{-1, 0, -2},
		{0, 0, -3},
		{0, 0, 3},
}};

static constexpr std::array<Coord, 12> ROUND3_PATTERN = {{
		{3, 0, -1},
		{3, 0, 1},
		{2, 0, -2},
		{2, 0, 2},
		{1, 0, -3},
		{1, 0, 3},
		{-3, 0, -1},
		{-3, 0, 1},
		{-2, 0, -2},
		{-2, 0, 2},
		{-1, 0, 3},
		{-1, 0, -3},
}};

static const std::array<std::span<const Coord>, 3> ROUND_PATTERNS = {
		std::span<const Coord>(ROUND1_PATTERN), std::span<const Coord>(ROUND2_PATTERN),
		std::span<const Coord>(ROUND3_PATTERN)};

static const std::array<std::pair<Coord, Coord>, 5> OAK_LEAVES_FILL = {{
		{{-1, 3, 0}, {-1, 9, 0}},
		{{1, 3, 0}, {1, 9, 0}},
		{{0, 3, -1}, {0, 9, -1}},
		{{0, 3, 1}, {0, 9, 1}},
		{{0, 9, 0}, {0, 10, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 6> SPRUCE_LEAVES_FILL = {{
		{{-1, 3, 0}, {-1, 10, 0}},
		{{0, 3, -1}, {0, 10, -1}},
		{{1, 3, 0}, {1, 10, 0}},
		{{0, 3, -1}, {0, 10, -1}},
		{{0, 3, 1}, {0, 10, 1}},
		{{0, 11, 0}, {0, 11, 0}},
}};

static const std::array<std::pair<Coord, Coord>, 5> BIRCH_LEAVES_FILL = {{
		{{-1, 2, 0}, {-1, 7, 0}},
		{{1, 2, 0}, {1, 7, 0}},
		{{0, 2, -1}, {0, 7, -1}},
		{{0, 2, 1}, {0, 7, 1}},
		{{0, 7, 0}, {0, 8, 0}},
}};

enum class TreeType
{
	Oak = 0,
	Spruce = 1,
	Birch = 2,
	DarkOak = 3,
	Jungle = 4,
	Acacia = 5,
	Cherry = 6,
	TallOak = 7,
	Pine = 8,
	Bush = 9,
	AzaleaBush = 10,
	Willow = 11,
	FloweringOak = 12,
	Mangrove = 13
};

struct Tree
{
	Block log_block;
	int log_height;
	Block leaves_block;
	std::span<const std::pair<Coord, Coord>> leaves_fill;
	std::array<std::vector<int>, 3> round_ranges;
	float branch_chance = 0.0f;
	std::optional<Block> accent_block;
	std::uint8_t accent_chance = 0;
	bool drooping = false;

	static void round(WorldEditor &editor, Block material, const Coord &center,
			std::span<const Coord> block_pattern)
	{
		for (const Coord &d : block_pattern) {
			editor.set_block(material, center.x + d.x, center.y + d.y, center.z + d.z,
					std::nullopt, std::nullopt);
		}
	}

	static void round_absolute(WorldEditor &editor, Block material, const Coord &center,
			std::span<const Coord> block_pattern)
	{
		for (const Coord &d : block_pattern) {
			editor.set_block_absolute(material, center.x + d.x, center.y + d.y,
					center.z + d.z, std::nullopt, std::nullopt);
		}
	}

	static Tree get_tree(TreeType kind);
	static Tree get_tree_variant(TreeType kind, std::uint32_t variant_idx);

	static void create(WorldEditor &editor, const Coord &pos,
			const BuildingFootprintBitmap *building_footprints = nullptr,
			const bridges::BridgeSurfaceMap *bridge_surface = nullptr);
	// Rust's canopy path has already made the density decision.  The C++
	// procedural fallback does no second density thinning, so it shares the
	// geometry path while retaining this explicit integration entry point.
	static void create_from_canopy(WorldEditor &editor, const Coord &pos,
			const BuildingFootprintBitmap *building_footprints = nullptr,
			const bridges::BridgeSurfaceMap *bridge_surface = nullptr);

	static void create_of_type(WorldEditor &editor, const Coord &pos, TreeType tree_type,
			const BuildingFootprintBitmap *building_footprints = nullptr,
			const bridges::BridgeSurfaceMap *bridge_surface = nullptr,
			bool allow_on_paved = false);

	static std::vector<Block> get_building_wall_blocks();
	static std::vector<Block> get_building_floor_blocks();
	static std::vector<Block> get_structural_blocks();
	static std::vector<Block> get_functional_blocks();
	static bool canopy_might_intersect_building(
			int x, int z, const BuildingFootprintBitmap *building_footprints);
};

}
