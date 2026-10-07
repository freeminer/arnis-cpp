#include "bush.h"

#include "../ecoregion.h"
#include "../land_cover/land_cover.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace arnis::bush
{
const std::array<Block, 8> &leaf_blocks()
{
	static const std::array<Block, 8> blocks = {OAK_LEAVES, BIRCH_LEAVES, SPRUCE_LEAVES,
			DARK_OAK_LEAVES, JUNGLE_LEAVES, ACACIA_LEAVES, AZALEA_LEAVES,
			FLOWERING_AZALEA_LEAVES};
	return blocks;
}

namespace
{
using Offset = std::array<int, 3>;
using Shape = std::vector<Offset>;
using WeightedShape = std::pair<const Shape *, unsigned>;

const Shape SINGLE = {{0, 0, 0}};
const Shape TALL = {{0, 0, 0}, {0, 1, 0}};
const Shape PAIR = {{0, 0, 0}, {1, 0, 0}};
const Shape CLUMP = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 1, 0}};
const Shape MOUND = {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 0, 1}, {0, 0, -1}, {0, 1, 0}};
const Shape WIDE = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}, {0, 1, 0}, {1, 1, 1}};
const Shape SPRAWL = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {2, 0, 1}, {-1, 0, 0}};

const std::array<WeightedShape, 7> WILD_SHAPES = {{{&SINGLE, 25}, {&TALL, 15},
		{&PAIR, 15}, {&CLUMP, 18}, {&MOUND, 12}, {&WIDE, 7}, {&SPRAWL, 8}}};
const std::array<WeightedShape, 5> GARDEN_SHAPES = {
		{{&SINGLE, 10}, {&TALL, 15}, {&CLUMP, 25}, {&MOUND, 30}, {&WIDE, 20}}};
const std::array<WeightedShape, 3> LOW_SHAPES = {
		{{&SINGLE, 55}, {&PAIR, 30}, {&TALL, 15}}};

enum class Flora
{
	Broadleaf,
	Conifer,
	Tropical,
	Dry,
	Mediterranean,
	Grassland
};

constexpr int DRIFT_CELL = 18;
constexpr int32_t SALT_SHAPE = 0x5B0511C3;
constexpr int32_t SALT_DRIFT = 0x0D21F7A9;
constexpr int32_t SALT_FLOWER = 0x7F103A2D;

int32_t wrap_add(int32_t a, int32_t b)
{
	return static_cast<int32_t>(static_cast<uint32_t>(a) + static_cast<uint32_t>(b));
}

int floor_div(int value, int divisor)
{
	int q = value / divisor;
	if (value % divisor < 0)
		--q;
	return q;
}

uint64_t hash(int32_t x, int32_t z, int32_t salt)
{
	return land_cover::coord_hash(x ^ salt, wrap_add(z, salt));
}

Flora flora_for_ecoregion(ecoregion::EcoBiome biome)
{
	using enum ecoregion::EcoBiome;
	switch (biome) {
	case MoistTropical:
	case TropicalConifer:
	case Flooded:
	case Mangroves:
		return Flora::Tropical;
	case DryTropical:
	case TropicalGrassland:
	case Desert:
		return Flora::Dry;
	case TemperateBroadleaf:
		return Flora::Broadleaf;
	case TemperateConifer:
	case Boreal:
	case Tundra:
	case MontaneGrassland:
		return Flora::Conifer;
	case Mediterranean:
		return Flora::Mediterranean;
	case TemperateGrassland:
		return Flora::Grassland;
	}
	return Flora::Broadleaf;
}

Flora flora_for_climate(biome::Climate climate)
{
	switch (climate) {
	case biome::Climate::Temperate:
		return Flora::Broadleaf;
	case biome::Climate::TropicalSavanna:
	case biome::Climate::HotDesert:
	case biome::Climate::HotSteppe:
		return Flora::Dry;
	case biome::Climate::ColdDesert:
	case biome::Climate::ColdSteppe:
	case biome::Climate::DryContinental:
		return Flora::Grassland;
	case biome::Climate::Boreal:
	case biome::Climate::Tundra:
	case biome::Climate::IceCap:
		return Flora::Conifer;
	}
	return Flora::Broadleaf;
}

template <typename T, std::size_t N>
T weighted_pick(const std::array<std::pair<T, unsigned>, N> &options, uint64_t roll)
{
	unsigned total = 0;
	for (const auto &[item, weight] : options) {
		(void)item;
		total += weight;
	}
	unsigned remaining = static_cast<unsigned>(roll % total);
	for (const auto &[item, weight] : options) {
		if (remaining < weight)
			return item;
		remaining -= weight;
	}
	return options.front().first;
}

Block species_pick(Flora flora, uint64_t roll)
{
	switch (flora) {
	case Flora::Broadleaf: {
		const std::array<std::pair<Block, unsigned>, 4> choices = {{{OAK_LEAVES, 45},
				{DARK_OAK_LEAVES, 15}, {BIRCH_LEAVES, 15}, {AZALEA_LEAVES, 25}}};
		return weighted_pick(choices, roll);
	}
	case Flora::Conifer: {
		const std::array<std::pair<Block, unsigned>, 4> choices = {{{SPRUCE_LEAVES, 50},
				{BIRCH_LEAVES, 25}, {OAK_LEAVES, 15}, {AZALEA_LEAVES, 10}}};
		return weighted_pick(choices, roll);
	}
	case Flora::Tropical: {
		const std::array<std::pair<Block, unsigned>, 4> choices = {{{JUNGLE_LEAVES, 45},
				{OAK_LEAVES, 20}, {AZALEA_LEAVES, 20}, {DARK_OAK_LEAVES, 15}}};
		return weighted_pick(choices, roll);
	}
	case Flora::Dry: {
		const std::array<std::pair<Block, unsigned>, 3> choices = {
				{{ACACIA_LEAVES, 50}, {OAK_LEAVES, 35}, {DARK_OAK_LEAVES, 15}}};
		return weighted_pick(choices, roll);
	}
	case Flora::Mediterranean: {
		const std::array<std::pair<Block, unsigned>, 4> choices = {{{OAK_LEAVES, 35},
				{DARK_OAK_LEAVES, 30}, {AZALEA_LEAVES, 25}, {ACACIA_LEAVES, 10}}};
		return weighted_pick(choices, roll);
	}
	case Flora::Grassland: {
		const std::array<std::pair<Block, unsigned>, 4> choices = {{{OAK_LEAVES, 50},
				{BIRCH_LEAVES, 20}, {AZALEA_LEAVES, 15}, {DARK_OAK_LEAVES, 15}}};
		return weighted_pick(choices, roll);
	}
	}
	return OAK_LEAVES;
}

Block species_at(const WorldEditor &editor, int x, int z, Kind kind)
{
	Flora flora = flora_for_climate(editor.climate());
	if (editor.ground)
		if (const auto eco = editor.ground->ecoregion_at(editor.ground_point(x, z)))
			flora = flora_for_ecoregion(eco->biome);
	const uint64_t own = hash(x, z, SALT_DRIFT);
	const uint64_t roll = own % 4 == 0 ? own >> 8
									   : hash(floor_div(x, DRIFT_CELL),
												 floor_div(z, DRIFT_CELL), SALT_DRIFT);
	const Block species = species_pick(flora, roll);
	if (kind == Kind::Garden && species != AZALEA_LEAVES && ((own >> 40) % 5) == 0)
		return AZALEA_LEAVES;
	return species;
}

void set_leaf(WorldEditor &editor, Block leaf, int x, int y, int z)
{
	BlockWithProperties persistent{leaf, {{"persistent", "true"}}};
	editor.set_block_with_properties_absolute(
			persistent, x, editor.get_absolute_y(x, y, z), z, std::nullopt, std::nullopt);
}

bool can_spread(const WorldEditor &editor, int x, int z)
{
	if (editor.surface_is_sealed(x, z) || editor.is_lc_water(x, z))
		return false;
	const int ground_y = editor.get_absolute_y(x, 0, z);
	const auto ground = editor.get_block_absolute(x, ground_y, z);
	if (ground && *ground != GRASS_BLOCK && *ground != DIRT && *ground != COARSE_DIRT &&
			*ground != PODZOL && *ground != MOSS_BLOCK && *ground != SNOWY_GRASS_BLOCK &&
			*ground != SNOWY_PODZOL && *ground != MUD)
		return false;
	return !editor.block_exists_absolute(x, ground_y + 1, z);
}

std::pair<int, int> rotate(int dx, int dz, uint64_t turns)
{
	switch (turns & 3) {
	case 0:
		return {dx, dz};
	case 1:
		return {-dz, dx};
	case 2:
		return {-dx, -dz};
	default:
		return {dz, -dx};
	}
}

template <std::size_t N>
const Shape &select_shape(const std::array<WeightedShape, N> &shapes, uint64_t roll)
{
	unsigned total = 0;
	for (const auto &[shape, weight] : shapes) {
		(void)shape;
		total += weight;
	}
	unsigned remaining = static_cast<unsigned>(roll % total);
	for (const auto &[shape, weight] : shapes) {
		if (remaining < weight)
			return *shape;
		remaining -= weight;
	}
	return *shapes.front().first;
}

} // namespace

void place(WorldEditor &editor, int x, int z, Kind kind)
{
	if (editor.block_exists_absolute(x, editor.get_absolute_y(x, 1, z), z))
		return;
	const uint64_t shape_hash = hash(x, z, SALT_SHAPE);
	const Shape &shape = kind == Kind::Wild		? select_shape(WILD_SHAPES, shape_hash)
						 : kind == Kind::Garden ? select_shape(GARDEN_SHAPES, shape_hash)
												: select_shape(LOW_SHAPES, shape_hash);
	const uint64_t turns = shape_hash >> 32;
	const Block species = species_at(editor, x, z, kind);
	const unsigned flower_share = kind == Kind::Garden ? 45 : 25;
	std::vector<std::pair<int, int>> rooted;
	rooted.reserve(shape.size());
	for (const auto &[dx, dy, dz] : shape) {
		const auto [rx, rz] = rotate(dx, dz, turns);
		const int bx = x + rx;
		const int bz = z + rz;
		if (dy == 0) {
			if ((rx != 0 || rz != 0) && !can_spread(editor, bx, bz))
				continue;
			rooted.emplace_back(bx, bz);
		} else if (std::find(rooted.begin(), rooted.end(), std::pair{bx, bz}) ==
				   rooted.end()) {
			continue;
		}
		const Block leaf =
				species == AZALEA_LEAVES && (hash(bx, wrap_add(bz, dy), SALT_FLOWER) %
													100) < flower_share
						? FLOWERING_AZALEA_LEAVES
						: species;
		set_leaf(editor, leaf, bx, 1 + dy, bz);
	}
}

Block shrubbery_species(const WorldEditor &editor, int x, int z)
{
	return species_at(editor, x, z, Kind::Garden);
}

Block shrubbery_leaf(Block species, int x, int y, int z)
{
	if (species == AZALEA_LEAVES && (hash(x, wrap_add(z, y), SALT_FLOWER) % 100) < 45)
		return FLOWERING_AZALEA_LEAVES;
	return species;
}

} // namespace arnis::bush
