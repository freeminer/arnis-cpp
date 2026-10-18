#include "rooftops.h"

#include "connected_blocks.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../deterministic_rng.h"

namespace arnis::buildings::rooftops
{
namespace
{
using Family = std::vector<std::array<Block, 3>>;

bool is_glass_block(const Block &block)
{
	return block == GLASS || block == WHITE_STAINED_GLASS ||
		   block == GRAY_STAINED_GLASS || block == LIGHT_GRAY_STAINED_GLASS ||
		   block == BLACK_STAINED_GLASS || block == BROWN_STAINED_GLASS ||
		   block == CYAN_STAINED_GLASS || block == BLUE_STAINED_GLASS ||
		   block == LIGHT_BLUE_STAINED_GLASS || block == RED_STAINED_GLASS ||
		   block == YELLOW_STAINED_GLASS || block == PURPLE_STAINED_GLASS ||
		   block == ORANGE_STAINED_GLASS || block == MAGENTA_STAINED_GLASS ||
		   block == TINTED_GLASS || block == GLASS_PANE ||
		   block == BLUE_STAINED_GLASS_PANE || block == GRAY_STAINED_GLASS_PANE;
}

const std::array<Family, 10> &membrane_families()
{
	static const std::array<Family, 10> families = {{
			{{{CALCITE, WHITE_WOOL, SMOOTH_STONE},
					{WHITE_CONCRETE, DIORITE, LIGHT_GRAY_CONCRETE},
					{WHITE_WOOL, CALCITE, POLISHED_ANDESITE}}},
			{{{SMOOTH_STONE, LIGHT_GRAY_CONCRETE, ANDESITE},
					{LIGHT_GRAY_CONCRETE, SMOOTH_STONE, POLISHED_ANDESITE}}},
			{{{GRAY_WOOL, GRAY_CONCRETE, POLISHED_BLACKSTONE},
					{POLISHED_ANDESITE, ANDESITE, GRAY_CONCRETE}}},
			{{{SMOOTH_BASALT, GRAY_WOOL, BLACK_CONCRETE},
					{GRAY_WOOL, SMOOTH_BASALT, POLISHED_BLACKSTONE}}},
			{{{ANDESITE, MOSS_BLOCK, COARSE_DIRT}, {ANDESITE, TUFF, GRAY_CONCRETE},
					{GRAY_CONCRETE, COARSE_DIRT, MOSS_BLOCK}}},
			{{{GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, SMOOTH_STONE},
					{LIGHT_GRAY_CONCRETE, SMOOTH_STONE, GRAY_CONCRETE},
					{SMOOTH_STONE, GRAY_WOOL, LIGHT_GRAY_CONCRETE}}},
			{{{MUSHROOM_STEM, SMOOTH_SANDSTONE, LIGHT_GRAY_CONCRETE},
					{LIGHT_GRAY_CONCRETE, MUSHROOM_STEM, SMOOTH_SANDSTONE},
					{SMOOTH_SANDSTONE, MUSHROOM_STEM, SMOOTH_STONE}}},
			{{{LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, BLUE_WOOL},
					{GRAY_CONCRETE, CYAN_TERRACOTTA, LIGHT_GRAY_CONCRETE},
					{SMOOTH_STONE, GRAY_CONCRETE, CYAN_TERRACOTTA}}},
			{{{TERRACOTTA, RED_TERRACOTTA, LIGHT_GRAY_CONCRETE},
					{RED_TERRACOTTA, TERRACOTTA, SMOOTH_STONE}}},
			{{{WAXED_OXIDIZED_COPPER, CYAN_TERRACOTTA, LIGHT_GRAY_CONCRETE},
					{WAXED_OXIDIZED_COPPER, WAXED_OXIDIZED_COPPER, CYAN_TERRACOTTA}}},
	}};
	return families;
}

std::uint64_t hash2(std::uint64_t seed, std::int32_t a, std::int32_t b)
{
	std::uint64_t h =
			seed ^
			(std::uint64_t(static_cast<std::uint32_t>(a)) * 0x9E3779B97F4A7C15ULL) ^
			(std::uint64_t(static_cast<std::uint32_t>(b)) * 0xC2B2AE3D27D4EB4FULL);
	h ^= h >> 30;
	h *= 0xBF58476D1CE4E5B9ULL;
	h ^= h >> 27;
	h *= 0x94D049BB133111EBULL;
	return h ^ (h >> 31);
}

double value_noise(std::uint64_t seed, std::int32_t x, std::int32_t z, int cell)
{
	const int gx = x >= 0 ? x / cell : (x - cell + 1) / cell;
	const int gz = z >= 0 ? z / cell : (z - cell + 1) / cell;
	const int rx = x - gx * cell;
	const int rz = z - gz * cell;
	const double fx = static_cast<double>(rx) / cell;
	const double fz = static_cast<double>(rz) / cell;
	const auto corner = [&](int i, int j) {
		return static_cast<double>(hash2(seed, gx + i, gz + j) >> 11) /
			   static_cast<double>(std::uint64_t{1} << 53);
	};
	const auto ease = [](double t) { return t * t * (3.0 - 2.0 * t); };
	const double sx = ease(fx), sz = ease(fz);
	const double near = corner(0, 0) + (corner(1, 0) - corner(0, 0)) * sx;
	const double far = corner(0, 1) + (corner(1, 1) - corner(0, 1)) * sx;
	return near + (far - near) * sz;
}

int div_euclid(int value, int divisor)
{
	const int q = value / divisor;
	const int r = value % divisor;
	return r < 0 ? q - 1 : q;
}

struct CellHash
{
	std::size_t operator()(const std::pair<int, int> &cell) const noexcept
	{
		const auto x = static_cast<std::uint32_t>(cell.first);
		const auto z = static_cast<std::uint32_t>(cell.second);
		return static_cast<std::size_t>((std::uint64_t{x} << 32) | z);
	}
};

void place_mast(WorldEditor &editor, int x, int y, int z, int height)
{
	for (int dy = 0; dy < height; ++dy)
		editor.set_block_absolute(IRON_BARS, x, y + dy, z);
	editor.set_block_absolute(LIGHTNING_ROD, x, y + height, z);
}

int family_for_region(building_style::MaterialRegion material)
{
	using R = building_style::MaterialRegion;
	switch (material) {
	case R::EastEurope:
		return 3; // dark tar 45, grey 35, light 20
	case R::CentralAsia:
		return 10;
	case R::MiddleEast:
		return 6;
	case R::SouthAsia:
	case R::SoutheastAsia:
		return 7;
	case R::EastAsia:
		return 1;
	case R::Korea:
		return 9;
	case R::Japan:
		return 1;
	case R::SubSaharanAfrica:
		return 1;
	case R::LatinAmerica:
	case R::SouthernCone:
		return 8;
	case R::Australasia:
		return 1;
	default:
		return -1;
	}
}

void place(WorldEditor &editor, Block block, int x, int y, int z)
{
	editor.set_block_absolute(block, x, y, z, std::nullopt, std::nullopt);
}

const char *facing_for_normal(int nx, int nz)
{
	if (nx == 1)
		return "east";
	if (nx == -1)
		return "west";
	if (nz == 1)
		return "south";
	return "north";
}

BlockWithProperties prop_block(Block block,
		std::initializer_list<std::pair<std::string, std::string>> properties)
{
	return {block, BlockWithProperties::Properties(properties.begin(), properties.end())};
}

void place_props(
		WorldEditor &editor, const BlockWithProperties &block, int x, int y, int z)
{
	editor.set_block_with_properties_absolute(block, x, y, z, std::nullopt, std::nullopt);
}

bool is_glass(Block block)
{
	return is_glass_block(block);
}

void place_bulkhead(WorldEditor &editor, RoofDeck &deck, std::pair<int, int> origin,
		int width, int depth, int height, Block wall, Block cap, Block door,
		std::pair<int, int> door_side)
{
	const auto [x0, z0] = origin;
	const int y1 = deck.surface_y + 1;
	const auto cap_block = bottom_slab(cap);
	for (int z = z0; z < z0 + depth; ++z)
		for (int x = x0; x < x0 + width; ++x) {
			if (x == x0 || x == x0 + width - 1 || z == z0 || z == z0 + depth - 1)
				for (int y = y1; y < y1 + height; ++y)
					place(editor, wall, x, y, z);
			place_props(editor, cap_block, x, y1 + height, z);
		}
	const auto middle_of = [&](std::pair<int, int> normal) {
		if (normal.first == 1)
			return std::pair{x0 + width - 1, z0 + depth / 2};
		if (normal.first == -1)
			return std::pair{x0, z0 + depth / 2};
		if (normal.second == 1)
			return std::pair{x0 + width / 2, z0 + depth - 1};
		return std::pair{x0 + width / 2, z0};
	};
	const auto [dx, dz] = middle_of(door_side);
	const auto facing = facing_for_normal(door_side.first, door_side.second);
	for (const auto &[half, y] : {std::pair{"lower", y1}, {"upper", y1 + 1}})
		place_props(editor,
				prop_block(door,
						{{"half", std::string(half)}, {"facing", std::string(facing)},
								{"hinge", "left"}}),
				dx, y, dz);
	if (height >= 3) {
		const auto [wx, wz] = middle_of({-door_side.first, -door_side.second});
		place(editor, GRAY_STAINED_GLASS, wx, y1 + 1, wz);
	}
	deck.reserve(x0, z0, width, depth);
	deck.reserve(dx + door_side.first, dz + door_side.second, 1, 1);
}

void place_water_tank(WorldEditor &editor, RoofDeck &deck, std::pair<int, int> centre,
		int radius, int frame_height, int barrel_height)
{
	const auto inside = [](int dx, int dz, int r) {
		return dx * dx + dz * dz <= r * r + r;
	};
	const auto rim = [&](int dx, int dz, int r) {
		return inside(dx, dz, r) &&
			   (!inside(dx + 1, dz, r) || !inside(dx - 1, dz, r) ||
					   !inside(dx, dz + 1, r) || !inside(dx, dz - 1, r));
	};
	const auto [cx, cz] = centre;
	const int y1 = deck.surface_y + 1;
	std::vector<std::pair<int, int>> legs = {
			{radius, 0}, {-radius, 0}, {0, radius}, {0, -radius}};
	if (radius == 1)
		legs = {{1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
	else if (radius >= 3) {
		const int k = static_cast<int>(std::lround(radius / std::numbers::sqrt2));
		legs.insert(legs.end(), {{k, k}, {-k, k}, {k, -k}, {-k, -k}});
	}
	for (const auto &[dx, dz] : legs)
		for (int y = y1; y < y1 + frame_height; ++y)
			place(editor, DARK_OAK_FENCE, cx + dx, y, cz + dz);
	const int tub = y1 + frame_height;
	const auto stave = prop_block(BARREL, {{"facing", "up"}, {"open", "false"}});
	for (int dz = -radius; dz <= radius; ++dz)
		for (int dx = -radius; dx <= radius; ++dx) {
			if (rim(dx, dz, radius))
				for (int y = tub; y < tub + barrel_height; ++y)
					place_props(editor, stave, cx + dx, y, cz + dz);
			else if (inside(dx, dz, radius))
				place(editor, SPRUCE_PLANKS, cx + dx, tub, cz + dz);
		}
	const int top = tub + barrel_height;
	const auto slab = bottom_slab(SPRUCE_SLAB);
	for (int course = 0; course <= radius; ++course) {
		const int r = radius - course;
		for (int dz = -r; dz <= r; ++dz)
			for (int dx = -r; dx <= r; ++dx) {
				if (!inside(dx, dz, r))
					continue;
				const int x = cx + dx, y = top + course, z = cz + dz;
				if (r == 0 || (rim(dx, dz, r) && std::abs(dx) == std::abs(dz)))
					place_props(editor, slab, x, y, z);
				else if (!rim(dx, dz, r))
					place(editor, SPRUCE_PLANKS, x, y, z);
				else {
					const char *facing = std::abs(dx) > std::abs(dz)
												 ? (dx > 0 ? "west" : "east")
												 : (dz > 0 ? "north" : "south");
					place_props(editor, prop_block(SPRUCE_STAIRS, {{"facing", facing}}),
							x, y, z);
				}
			}
	}
	deck.reserve(cx - radius, cz - radius, 2 * radius + 1, 2 * radius + 1);
}

void place_condenser(WorldEditor &editor, RoofDeck &deck, std::pair<int, int> origin,
		int width, int depth, Block body, bool tall, bool grille)
{
	const auto [x0, z0] = origin;
	const int y1 = deck.surface_y + 1;
	const int body_height = tall ? 2 : 1;
	const auto trapdoor = prop_block(
			IRON_TRAPDOOR, {{"facing", "north"}, {"half", "bottom"}, {"open", "false"}});
	for (int z = z0; z < z0 + depth; ++z)
		for (int x = x0; x < x0 + width; ++x) {
			for (int y = y1; y < y1 + body_height; ++y)
				place(editor, body, x, y, z);
			if (grille)
				place_props(editor, trapdoor, x, y1 + body_height, z);
			else
				place(editor, DAYLIGHT_DETECTOR, x, y1 + body_height, z);
		}
	deck.reserve(x0, z0, width, depth);
}

void place_small_unit(
		WorldEditor &editor, RoofDeck &deck, std::pair<int, int> cell, ChaCha8Rng &rng)
{
	const auto [x, z] = cell;
	const int y = deck.surface_y + 1;
	static constexpr const char *facings[] = {"north", "south", "east", "west"};
	const char *facing = facings[rng.uniform(4)];
	switch (rng.uniform(6)) {
	case 0:
		place_props(editor, prop_block(BLAST_FURNACE, {{"facing", facing}}), x, y, z);
		break;
	case 1:
		place_props(editor, prop_block(SMOKER, {{"facing", facing}}), x, y, z);
		break;
	case 2:
		place_props(editor,
				prop_block(GRINDSTONE, {{"face", "floor"}, {"facing", facing}}), x, y, z);
		break;
	case 3:
		place_props(editor, prop_block(BARREL, {{"facing", "up"}, {"open", "false"}}), x,
				y, z);
		break;
	case 4:
		place(editor, IRON_BLOCK, x, y, z);
		place_props(editor,
				prop_block(IRON_TRAPDOOR,
						{{"facing", facing}, {"half", "bottom"}, {"open", "false"}}),
				x, y + 1, z);
		break;
	default:
		place(editor, LIGHT_GRAY_CONCRETE, x, y, z);
		place(editor, DAYLIGHT_DETECTOR, x, y + 1, z);
		break;
	}
	deck.reserve(x, z, 1, 1);
}

void place_vent(WorldEditor &editor, RoofDeck &deck, std::pair<int, int> cell,
		bool pipes_only, ChaCha8Rng &rng)
{
	const auto [x, z] = cell;
	const int y = deck.surface_y + 1;
	switch (rng.uniform(pipes_only ? 3 : 6)) {
	case 0: {
		const Block stacks[] = {ANDESITE_WALL, STONE_BRICK_WALL, COBBLESTONE_WALL};
		const Block stack = stacks[rng.uniform(3)];
		const int height = 1 + rng.uniform(2);
		for (int dy = 0; dy < height; ++dy)
			place(editor, stack, x, y + dy, z);
		break;
	}
	case 1:
		place(editor, LIGHTNING_ROD, x, y, z);
		break;
	case 2:
		place(editor, END_ROD, x, y, z);
		break;
	case 3:
		place(editor, IRON_BARS, x, y, z);
		place_props(editor, bottom_slab(SMOOTH_STONE_SLAB), x, y + 1, z);
		break;
	case 4:
		place(editor, CAULDRON, x, y, z);
		break;
	default:
		place(editor, STONE_PRESSURE_PLATE, x, y, z);
		break;
	}
	deck.reserve(x, z, 1, 1);
}

void place_skylight(WorldEditor &editor, RoofDeck &deck, std::pair<int, int> origin,
		int width, int depth)
{
	const auto [x0, z0] = origin;
	const auto curb = bottom_slab(SMOOTH_STONE_SLAB);
	for (int z = z0; z < z0 + depth; ++z)
		for (int x = x0; x < x0 + width; ++x)
			if (x == x0 || x == x0 + width - 1 || z == z0 || z == z0 + depth - 1)
				place_props(editor, curb, x, deck.surface_y + 1, z);
			else
				editor.set_block_absolute(
						GLASS, x, deck.surface_y, z, std::nullopt, std::vector<Block>{});
	deck.reserve(x0, z0, width, depth);
}

bool place_penthouse(
		WorldEditor &editor, RoofDeck &deck, const RooftopBrief &brief, ChaCha8Rng &rng)
{
	const auto total = deck.count_at_least(0);
	int inset = 3;
	while (inset < deck.deepest && deck.count_at_least(inset) * 20 > total * 9)
		++inset;
	const auto cells = deck.cells_at_least(inset);
	if (cells.size() * 100 < total * 20 || cells.size() < 36 ||
			std::any_of(cells.begin(), cells.end(), [&](const auto &p) {
				return !deck.fits(p.first, p.second, 1, 1, inset);
			}))
		return false;
	const int storeys = brief.height >= brief.scaled(120) && cells.size() >= 300 &&
										rng.random_bool(.5)
								? 2
								: 1;
	const int height = std::max(3, brief.floor_cycle) * storeys;
	const Block walls[] = {LIGHT_GRAY_CONCRETE, GRAY_CONCRETE, SMOOTH_STONE};
	const Block wall = is_glass(brief.wall) || rng.random_bool(.5) ? walls[rng.uniform(3)]
																   : brief.wall;
	const Block roofs[] = {GRAY_CONCRETE, POLISHED_ANDESITE, SMOOTH_STONE};
	const Block roof = roofs[rng.uniform(3)];
	const int y1 = deck.surface_y + 1;
	const auto coping = bottom_slab(SMOOTH_STONE_SLAB);
	for (const auto &[x, z] : cells) {
		std::optional<std::pair<int, int>> open;
		for (const auto &[dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}})
			if (deck.edge(x + dx, z + dz) < inset) {
				open = std::pair{dx, dz};
				break;
			}
		if (open) {
			const auto [ox, oz] = *open;
			const int u = ox != 0 ? z : x;
			const int residue = ((u % 6) + 6) % 6;
			const bool louvre = residue >= 2 && residue <= 4;
			for (int k = 0; k < height; ++k) {
				const int y = y1 + k;
				if (louvre && height >= 4 && k >= height - 3 && k < height - 1) {
					arnis::connected_blocks::place_connected(editor, IRON_BARS, x, y, z);
					place(editor, BLACK_CONCRETE, x - ox, y, z - oz);
				} else {
					place(editor, wall, x, y, z);
				}
			}
			place_props(editor, coping, x, y1 + height + 1, z);
		}
		place(editor, roof, x, y1 + height, z);
	}
	for (const auto &[x, z] : cells)
		deck.reserve(x, z, 1, 1);
	if (rng.random_bool(.6)) {
		const auto heart = deck.heart();
		const int mast_height = brief.scaled(6);
		for (int dy = 0; dy < mast_height; ++dy)
			place(editor, IRON_BARS, heart.first, y1 + height + 1 + dy, heart.second);
		place(editor, LIGHTNING_ROD, heart.first, y1 + height + 1 + mast_height,
				heart.second);
	}
	return true;
}

template <typename PlaceOne>
std::size_t lay_row(RoofDeck &deck, std::pair<int, int> start,
		std::pair<int, int> direction, std::size_t count, std::pair<int, int> size,
		int pitch, PlaceOne &&place_one)
{
	std::size_t placed = 0;
	int slot = 0, misses = 0;
	while (placed < count && misses < 4) {
		const auto target = std::pair{start.first + direction.first * pitch * slot,
				start.second + direction.second * pitch * slot};
		++slot;
		if (const auto origin = deck.find_near(target, size.first, size.second, 2, 2)) {
			place_one(deck, *origin);
			++placed;
			misses = 0;
		} else {
			++misses;
		}
	}
	return placed;
}

} // namespace

int RoofDistanceGrid::get(int x, int z) const
{
	const int lx = x - min_x, lz = z - min_z;
	if (lx < 0 || lz < 0 || lx >= width || lz >= height)
		return -1;
	return dist[static_cast<std::size_t>(lz * width + lx)];
}

std::optional<RoofDeck> RoofDeck::make(
		const std::vector<std::pair<int, int>> &cells, int roof_surface_y)
{
	if (cells.empty())
		return std::nullopt;
	RoofDeck deck;
	deck.grid.min_x = deck.grid.min_z = std::numeric_limits<int>::max();
	int max_x = std::numeric_limits<int>::min();
	int max_z = std::numeric_limits<int>::min();
	for (const auto &[x, z] : cells) {
		deck.grid.min_x = std::min(deck.grid.min_x, x);
		deck.grid.min_z = std::min(deck.grid.min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	deck.grid.width = max_x - deck.grid.min_x + 1;
	deck.grid.height = max_z - deck.grid.min_z + 1;
	const auto size = static_cast<std::size_t>(deck.grid.width) * deck.grid.height;
	deck.grid.dist.assign(size, -1);
	deck.taken.assign(size, false);
	deck.surface_y = roof_surface_y;
	const auto index = [&](int x, int z) {
		return static_cast<std::size_t>(z - deck.grid.min_z) * deck.grid.width +
			   static_cast<std::size_t>(x - deck.grid.min_x);
	};
	const std::unordered_set<std::pair<int, int>, CellHash> footprint(
			cells.begin(), cells.end());
	std::deque<std::pair<int, int>> pending;
	constexpr std::array<std::pair<int, int>, 4> directions = {
			{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
	for (const auto &[x, z] : cells) {
		for (const auto &[dx, dz] : directions) {
			if (!footprint.contains({x + dx, z + dz})) {
				deck.grid.dist[index(x, z)] = 0;
				pending.emplace_back(x, z);
				break;
			}
		}
	}
	while (!pending.empty()) {
		const auto [x, z] = pending.front();
		pending.pop_front();
		const int next = deck.grid.dist[index(x, z)] + 1;
		for (const auto &[dx, dz] : directions) {
			const auto cell = std::pair{x + dx, z + dz};
			if (!footprint.contains(cell))
				continue;
			auto &distance = deck.grid.dist[index(cell.first, cell.second)];
			if (distance < 0) {
				distance = next;
				pending.push_back(cell);
			}
		}
	}
	deck.deepest = *std::max_element(deck.grid.dist.begin(), deck.grid.dist.end());
	return deck;
}

int RoofDeck::edge(int x, int z) const
{
	return grid.get(x, z);
}

bool RoofDeck::is_taken(int x, int z) const
{
	const int lx = x - grid.min_x, lz = z - grid.min_z;
	return lx >= 0 && lz >= 0 && lx < grid.width && lz < grid.height &&
		   taken[static_cast<std::size_t>(lz * grid.width + lx)];
}

void RoofDeck::reserve(int x, int z, int width, int depth)
{
	for (int pz = z; pz < z + depth; ++pz)
		for (int px = x; px < x + width; ++px) {
			const int lx = px - grid.min_x, lz = pz - grid.min_z;
			if (lx >= 0 && lz >= 0 && lx < grid.width && lz < grid.height)
				taken[static_cast<std::size_t>(lz * grid.width + lx)] = true;
		}
}

bool RoofDeck::fits(int x, int z, int width, int depth, int min_edge) const
{
	for (int pz = z; pz < z + depth; ++pz)
		for (int px = x; px < x + width; ++px)
			if (edge(px, pz) < min_edge)
				return false;
	for (int pz = z - 1; pz <= z + depth; ++pz)
		for (int px = x - 1; px <= x + width; ++px)
			if (is_taken(px, pz))
				return false;
	return true;
}

std::optional<std::pair<int, int>> RoofDeck::find_near(
		std::pair<int, int> target, int width, int depth, int min_edge, int reach) const
{
	if (width > grid.width - 2 * min_edge || depth > grid.height - 2 * min_edge ||
			deepest < min_edge + (std::min(width, depth) - 1) / 2)
		return std::nullopt;
	const int ox = target.first - width / 2;
	const int oz = target.second - depth / 2;
	for (int r = 0; r <= reach; ++r) {
		for (int d = -r; d <= r; ++d)
			for (const int dz : {-r, r})
				if (fits(ox + d, oz + dz, width, depth, min_edge))
					return std::pair{ox + d, oz + dz};
		for (int d = -r + 1; d < r; ++d)
			for (const int dx : {-r, r})
				if (fits(ox + dx, oz + d, width, depth, min_edge))
					return std::pair{ox + dx, oz + d};
	}
	return std::nullopt;
}

std::size_t RoofDeck::count_at_least(int min_edge) const
{
	return static_cast<std::size_t>(std::count_if(grid.dist.begin(), grid.dist.end(),
			[min_edge](int d) { return d >= min_edge; }));
}

std::vector<std::pair<int, int>> RoofDeck::cells_at_least(int min_edge) const
{
	std::vector<std::pair<int, int>> cells;
	for (int z = 0; z < grid.height; ++z)
		for (int x = 0; x < grid.width; ++x)
			if (grid.dist[static_cast<std::size_t>(z * grid.width + x)] >= min_edge)
				cells.emplace_back(grid.min_x + x, grid.min_z + z);
	return cells;
}

std::pair<int, int> RoofDeck::heart() const
{
	const int cx2 = 2 * grid.min_x + grid.width - 1;
	const int cz2 = 2 * grid.min_z + grid.height - 1;
	auto cells = cells_at_least(deepest);
	if (cells.empty())
		return {cx2 / 2, cz2 / 2};
	return *std::min_element(
			cells.begin(), cells.end(), [&](const auto &a, const auto &b) {
				const auto distance = [&](const auto &p) {
					const std::int64_t dx = 2LL * p.first - cx2;
					const std::int64_t dz = 2LL * p.second - cz2;
					return dx * dx + dz * dz;
				};
				return distance(a) < distance(b);
			});
}

Block coping_for(Block wall, Block accent, std::uint64_t seed)
{
	const auto pick = [seed](std::initializer_list<Block> blocks) {
		const auto i = hash2(seed ^ 0xC0B1460000000001ULL, 0, 0) % blocks.size();
		return *(blocks.begin() + static_cast<std::ptrdiff_t>(i));
	};
	if (wall == BRICK || wall == MUD_BRICKS || wall == TERRACOTTA ||
			wall == RED_TERRACOTTA || wall == ORANGE_TERRACOTTA ||
			wall == BROWN_TERRACOTTA || wall == GRANITE || wall == POLISHED_GRANITE ||
			wall == BROWN_CONCRETE || wall == NETHER_BRICK || wall == RED_NETHER_BRICKS)
		return pick({SMOOTH_STONE_SLAB, SMOOTH_STONE_SLAB, STONE_BRICK_SLAB,
				QUARTZ_SLAB_TOP, CUT_SANDSTONE_SLAB});
	if (wall == QUARTZ_BLOCK || wall == QUARTZ_BRICKS || wall == SMOOTH_QUARTZ ||
			wall == WHITE_CONCRETE || wall == WHITE_TERRACOTTA || wall == CALCITE ||
			wall == DIORITE || wall == POLISHED_DIORITE)
		return pick({SMOOTH_QUARTZ_SLAB, POLISHED_DIORITE_SLAB, QUARTZ_SLAB_TOP});
	if (wall == SANDSTONE || wall == SMOOTH_SANDSTONE || wall == END_STONE_BRICKS ||
			wall == YELLOW_TERRACOTTA)
		return pick({CUT_SANDSTONE_SLAB, SMOOTH_STONE_SLAB});
	if (wall == STONE_BRICKS || wall == STONE || wall == ANDESITE ||
			wall == POLISHED_ANDESITE || wall == SMOOTH_STONE ||
			wall == LIGHT_GRAY_CONCRETE || wall == COBBLESTONE || wall == TUFF ||
			wall == LIGHT_GRAY_TERRACOTTA)
		return pick({SMOOTH_STONE_SLAB, STONE_BRICK_SLAB, POLISHED_ANDESITE_SLAB});
	if (wall == BLACKSTONE || wall == POLISHED_BLACKSTONE ||
			wall == POLISHED_BLACKSTONE_BRICKS || wall == DEEPSLATE ||
			wall == DEEPSLATE_BRICKS || wall == POLISHED_DEEPSLATE ||
			wall == GRAY_CONCRETE || wall == BLACK_CONCRETE || wall == GRAY_TERRACOTTA ||
			wall == CYAN_TERRACOTTA || wall == BLUE_TERRACOTTA)
		return pick({POLISHED_BLACKSTONE_SLAB, SMOOTH_STONE_SLAB, DEEPSLATE_TILE_SLAB});
	if (is_glass_block(wall) && is_glass_block(accent))
		return SMOOTH_STONE_SLAB;
	return is_glass_block(wall) ? get_slab_block_for_material(accent)
								: get_slab_block_for_material(wall);
}

BlockWithProperties bottom_slab(Block slab)
{
	if (slab == OAK_SLAB_TOP || slab == QUARTZ_SLAB_TOP)
		return BlockWithProperties{
				slab, BlockWithProperties::Properties{{"type", "bottom"}}};
	return BlockWithProperties::simple(slab);
}

int RooftopBrief::scaled(int value) const
{
	return std::max(1, static_cast<int>(std::lround(value * scale)));
}

int RooftopBrief::storeys() const
{
	return height / std::max(floor_cycle, 3);
}

bool RooftopBrief::plant_heavy() const
{
	return !residential && category != BuildingCategory::House &&
		   category != BuildingCategory::Historic && category != BuildingCategory::Farm;
}

void fit_out_roof(WorldEditor &editor, RoofDeck &deck, const RooftopBrief &brief)
{
	auto rng = element_rng(brief.seed ^ 0x2F0F70F500000001ULL);
	const auto usable = deck.count_at_least(2);
	if (!usable)
		return;
	const bool na = brief.region == building_style::BuildingRegion::NorthAmerica;
	const bool europe = brief.region == building_style::BuildingRegion::Europe;
	double density = na ? 1.0 : (europe ? .5 : .8);
	if (brief.sparse)
		density *= .5;
	if (brief.green)
		density *= .3;
	const auto count = [&](double per_cells, std::size_t cap) {
		return std::min(cap,
				static_cast<std::size_t>(std::max<long>(0,
						std::lround(static_cast<double>(usable) / per_cells * density))));
	};
	const auto heart = deck.heart();
	const bool along_x = deck.grid.width >= deck.grid.height;
	const std::pair<int, int> axis = along_x ? std::pair{1, 0} : std::pair{0, 1};
	const std::pair<int, int> across{axis.second, axis.first};
	const int long_side = along_x ? deck.grid.width : deck.grid.height;
	const int short_side = along_x ? deck.grid.height : deck.grid.width;
	const int flip = rng.random_bool(.5) ? 1 : -1;
	const auto step = [](std::pair<int, int> p, std::pair<int, int> d, int n) {
		return std::pair{p.first + d.first * n, p.second + d.second * n};
	};
	const auto end_a = step(heart, axis, -flip * long_side / 4);
	const auto end_b = step(heart, axis, flip * long_side / 4);
	const int side = std::max(short_side / 4, 2);
	const auto oriented = [&](int length, int width) {
		return along_x ? std::pair{length, width} : std::pair{width, length};
	};
	const int reach = std::max(long_side, short_side) / 2;

	const bool tower =
			brief.whole_building && brief.height >= brief.scaled(36) && usable >= 80;
	bool penthouse = false;
	if (tower && rng.random_bool(na ? .5 : (europe ? .3 : .4))) {
		auto penthouse_rng = element_rng(brief.seed ^ 0x9E470A5E00000001ULL);
		penthouse = place_penthouse(editor, deck, brief, penthouse_rng);
	}

	std::optional<std::tuple<int, int, int>> bulkhead_top;
	// Stair/lift bulkheads are fitted to roof sections as well as whole
	// buildings. Rust only restricts water tanks and penthouses to the main roof;
	// keep those guards separate from this general rooftop equipment pass.
	if (!penthouse && brief.storeys() >= 4 && usable >= 24 &&
			rng.random_bool(na ? .9 : (europe ? .6 : .8))) {
		const bool pair = usable >= 500 && rng.random_bool(.5);
		const bool big = usable >= 400;
		const auto [w, d] =
				oriented(brief.scaled(big ? 5 : 4), brief.scaled(big ? 4 : 3));
		const int h = std::max(2, brief.scaled(brief.height >= 40 ? 4 : 3));
		const Block wall_choices[] = {LIGHT_GRAY_CONCRETE, SMOOTH_STONE, WHITE_CONCRETE};
		const Block wall = !is_glass(brief.wall) && rng.random_bool(.55)
								   ? brief.wall
								   : wall_choices[rng.uniform(3)];
		const Block cap =
				rng.random_bool(.6) ? POLISHED_BLACKSTONE_SLAB : SMOOTH_STONE_SLAB;
		const Block door = rng.random_bool(.5) ? IRON_DOOR : DARK_OAK_DOOR_LOWER;
		for (int i = 0; i < (pair ? 2 : 1); ++i) {
			const auto target = i == 0 ? end_a : end_b;
			if (const auto origin = deck.find_near(target, w, d, 2, reach)) {
				const auto to_heart = std::pair{heart.first - (origin->first + w / 2),
						heart.second - (origin->second + d / 2)};
				const auto side_to_heart =
						along_x ? std::pair{to_heart.first >= 0 ? 1 : -1, 0}
								: std::pair{0, to_heart.second >= 0 ? 1 : -1};
				place_bulkhead(
						editor, deck, *origin, w, d, h, wall, cap, door, side_to_heart);
				if (!bulkhead_top)
					bulkhead_top = std::tuple{
							origin->first + w / 2, origin->second + d / 2, h + 1};
			}
		}
	} else if (!penthouse && usable >= 6 && rng.random_bool(.6)) {
		if (const auto hatch = deck.find_near(end_a, 1, 1, 2, reach)) {
			place_props(editor,
					prop_block(IRON_TRAPDOOR,
							{{"facing", "north"}, {"half", "bottom"}, {"open", "false"}}),
					hatch->first, deck.surface_y + 1, hatch->second);
			deck.reserve(hatch->first, hatch->second, 1, 1);
		}
	}

	const double tank_share = brief.era == ArchEra::HistoricOrnate ||
											  brief.era == ArchEra::TraditionalPreWar ||
											  brief.era == ArchEra::Unknown
									  ? .45
									  : (brief.era == ArchEra::PostWarPanel ? .25 : .06);
	const auto cat = brief.category;
	const bool tank_building =
			cat != BuildingCategory::GlassySkyscraper &&
			cat != BuildingCategory::GlassCornerSkyscraper &&
			cat != BuildingCategory::GridSkyscraper &&
			cat != BuildingCategory::ContemporarySkyscraper &&
			cat != BuildingCategory::ModernSkyscraper &&
			cat != BuildingCategory::Religious && cat != BuildingCategory::Hospital &&
			cat != BuildingCategory::School && cat != BuildingCategory::House &&
			cat != BuildingCategory::Greenhouse;
	if (na && brief.whole_building && !penthouse && tank_building &&
			brief.storeys() >= 6 && brief.height <= brief.scaled(160) && usable >= 20 &&
			rng.random_bool(tank_share)) {
		const bool large = usable >= 90;
		const int radius = brief.scaled(large ? 2 : 1);
		const auto target =
				bulkhead_top ? end_b : step(heart, axis, flip * long_side / 6);
		if (const auto origin =
						deck.find_near(target, 2 * radius + 1, 2 * radius + 1, 2, reach))
			place_water_tank(editor, deck,
					{origin->first + radius, origin->second + radius}, radius,
					brief.scaled(large ? 3 : 2) + rng.uniform(2),
					brief.scaled(large ? 4 : 3));
	}

	const bool industrial =
			cat == BuildingCategory::Industrial || cat == BuildingCategory::Warehouse;
	const std::size_t skylights =
			industrial ? count(120.0, 8)
					   : (usable >= 60 && rng.random_bool(.4 * std::min(density, 1.0))
										 ? 1 + (usable >= 200 && rng.random_bool(.5) ? 1u
																					 : 0u)
										 : 0u);
	const auto [sw, sd] = oriented(brief.scaled(4), brief.scaled(3));
	for (std::size_t i = 0; i < skylights; ++i) {
		const int offset = i % 2 == 0 ? static_cast<int>(i / 2) * 6
									  : -static_cast<int>(i / 2 + 1) * 6;
		if (const auto origin = deck.find_near(step(heart, axis, offset), sw, sd, 2, 3))
			place_skylight(editor, deck, *origin, sw, sd);
	}

	const std::size_t units = brief.plant_heavy() ? count(110.0, 8) : count(260.0, 3);
	if (units) {
		const Block bodies[] = {IRON_BLOCK, LIGHT_GRAY_CONCRETE, SMOOTH_STONE};
		const Block body = bodies[rng.uniform(3)];
		const bool tall = brief.plant_heavy() && rng.random_bool(.35);
		const int length = brief.scaled(3), width = brief.scaled(2);
		const auto size = oriented(length, width);
		const auto start = step(step(heart, across, side), axis,
				-static_cast<int>(std::min<std::size_t>(units, 4)) * (length + 1) / 2);
		lay_row(deck, start, axis, units, size, length + 1,
				[&](RoofDeck &dck, std::pair<int, int> origin) {
					place_condenser(editor, dck, origin, size.first, size.second, body,
							tall, rng.random_bool(.6));
				});
	}

	double solar_share = 0.0;
	if (europe)
		solar_share = brief.era == ArchEra::Contemporary
							  ? .45
							  : ((brief.era == ArchEra::PostWarPanel ||
										 brief.era == ArchEra::Unknown)
												? .18
												: .05);
	else if (brief.era == ArchEra::Contemporary && brief.storeys() <= 5)
		solar_share = .2;
	if (usable >= 40 && !brief.green && rng.random_bool(solar_share)) {
		const int rows = std::clamp(static_cast<int>(usable / 60), 2, 6);
		const auto size = oriented(std::clamp(long_side / 3, 3, 8), 1);
		const auto anchor = step(heart, across, -side);
		for (int i = 0; i < rows; ++i)
			if (const auto origin = deck.find_near(
						step(anchor, across, -2 * i), size.first, size.second, 2, 2)) {
				for (int z = origin->second; z < origin->second + size.second; ++z)
					for (int x = origin->first; x < origin->first + size.first; ++x)
						place(editor, DAYLIGHT_DETECTOR, x, deck.surface_y + 1, z);
				deck.reserve(origin->first, origin->second, size.first, size.second);
			}
	}

	const auto small = count(na ? 90.0 : 160.0, 6);
	if (small) {
		const auto start =
				step(step(heart, across, -side), axis, static_cast<int>(small));
		lay_row(deck, start, {-axis.first, -axis.second}, small, {1, 1}, 2,
				[&](RoofDeck &dck, std::pair<int, int> cell) {
					place_small_unit(editor, dck, cell, rng);
				});
	}

	const auto vents =
			std::max(count(na ? 35.0 : 60.0, 12), static_cast<std::size_t>(usable >= 8));
	if (vents) {
		const auto line = (vents + 1) / 2;
		const auto start = step(
				step(heart, across, -side - 2), axis, -static_cast<int>(line) * 3 / 2);
		const auto placed = lay_row(deck, start, axis, line, {1, 1}, 3,
				[&](RoofDeck &dck, std::pair<int, int> cell) {
					place_vent(editor, dck, cell, europe, rng);
				});
		for (std::size_t i = placed; i < vents; ++i) {
			const int x = deck.grid.min_x + rng.uniform(deck.grid.width);
			const int z = deck.grid.min_z + rng.uniform(deck.grid.height);
			if (const auto vent = deck.find_near({x, z}, 1, 1, 2, 3))
				place_vent(editor, deck, *vent, europe, rng);
		}
	}

	const bool mast = brief.height >= brief.scaled(40) && !penthouse
							  ? rng.random_bool(.4)
							  : (europe && brief.residential && rng.random_bool(.3));
	if (!mast)
		return;
	int x = heart.first, z = heart.second, y = deck.surface_y + 1;
	if (bulkhead_top) {
		x = std::get<0>(*bulkhead_top);
		z = std::get<1>(*bulkhead_top);
		y = deck.surface_y + std::get<2>(*bulkhead_top) + 1;
	} else if (const auto cell = deck.find_near(heart, 1, 1, 2, reach)) {
		x = cell->first;
		z = cell->second;
		deck.reserve(x, z, 1, 1);
	} else {
		return;
	}
	const int mast_height = brief.scaled(brief.height >= 40 ? 5 : 3);
	for (int dy = 0; dy < mast_height; ++dy)
		place(editor, IRON_BARS, x, y + dy, z);
	place(editor, LIGHTNING_ROD, x, y + mast_height, z);
}

void fit_out_terrace(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells,
		int surface_y, std::uint64_t seed, double scale)
{
	auto deck = RoofDeck::make(cells, surface_y);
	if (!deck)
		return;
	RooftopBrief brief;
	brief.seed = seed;
	brief.scale = scale;
	fit_out_terrace(editor, *deck, brief);
}

void fit_out_terrace(WorldEditor &editor, RoofDeck &deck, const RooftopBrief &brief)
{
	const auto heart = deck.heart();
	const int surface = deck.surface_y;
	for (const auto &[x, z] : deck.cells_at_least(2)) {
		const int edge = deck.edge(x, z);
		if (edge >= 3 && std::abs(x - heart.first) <= deck.grid.width / 4 &&
				std::abs(z - heart.second) <= deck.grid.height / 4) {
			editor.set_block_absolute(
					SPRUCE_PLANKS, x, surface, z, std::nullopt, std::vector<Block>{});
		} else if (edge == 2 &&
				   hash2(brief.seed, div_euclid(x, 3), div_euclid(z, 3)) % 3 == 0) {
			editor.set_block_absolute(MOSS_BLOCK, x, surface + 1, z);
			editor.set_block_absolute(
					(x + z) % 2 == 0 ? AZALEA_LEAVES : OAK_LEAVES, x, surface + 2, z);
			deck.reserve(x, z, 1, 1);
		}
	}
	if (const auto mast = deck.find_near(heart, 1, 1, 2, 4)) {
		place_mast(editor, mast->first, surface + 1, mast->second,
				std::max(1, static_cast<int>(std::lround(5.0 * brief.scale))));
		deck.reserve(mast->first, mast->second, 1, 1);
	}
}

Block RoofMembrane::block_at(int x, int z) const
{
	if (patch == base && repair == base)
		return base;
	if (hash2(seed ^ 0x7E9A12C500000001ULL, div_euclid(x, 4), div_euclid(z, 3)) % 100 < 4)
		return repair;
	return value_noise(seed, x, z, 7) > .64 ? patch : base;
}

Block RoofMembrane::block_at_edge(int x, int z, int edge) const
{
	return margin && edge <= 1 ? *margin : block_at(x, z);
}

bool RoofMembrane::is_green() const
{
	return base == MOSS_BLOCK;
}

RoofMembrane pick_membrane(building_style::BuildingRegion region,
		building_style::MaterialRegion materials, BuildingCategory category, ArchEra era,
		int building_height, std::uint64_t style_seed)
{
	auto rng = element_rng(style_seed ^ 0x6D656D6272616E65ULL);
	const int roll = static_cast<int>(rng.uniform(100));
	const bool industrial = category == BuildingCategory::Industrial ||
							category == BuildingCategory::Warehouse;
	int family = family_for_region(materials);
	if (industrial)
		family = -1;
	if (family == 3) {
		if (materials == building_style::MaterialRegion::EastEurope)
			family = roll < 45 ? 3 : (roll < 80 ? 2 : 1);
	} else if (family == 10) {
		family = roll < 30 ? 3 : (roll < 60 ? 2 : (roll < 85 ? 6 : 1));
	} else if (family == 6) {
		family = roll < 60 ? 6 : (roll < 80 ? 1 : (roll < 95 ? 0 : 2));
	} else if (family == 7) {
		family = materials == building_style::MaterialRegion::SouthAsia
						 ? (roll < 55 ? 7 : (roll < 75 ? 1 : (roll < 92 ? 2 : 8)))
						 : (roll < 35 ? 7 : (roll < 62 ? 2 : (roll < 90 ? 1 : 8)));
	} else if (family == 1) {
		if (materials == building_style::MaterialRegion::EastAsia)
			family = roll < 40 ? 1 : (roll < 75 ? 2 : (roll < 90 ? 7 : 8));
		else if (materials == building_style::MaterialRegion::Japan)
			family = roll < 50 ? 1 : (roll < 90 ? 2 : 0);
		else if (materials == building_style::MaterialRegion::SubSaharanAfrica)
			family = roll < 30 ? 1 : (roll < 60 ? 2 : (roll < 85 ? 6 : 7));
		else
			family = roll < 35 ? 1 : (roll < 70 ? 2 : 0);
	} else if (family == 9) {
		family = roll < 45 ? 9 : (roll < 75 ? 2 : 1);
	} else if (family == 8) {
		family = materials == building_style::MaterialRegion::SouthernCone
						 ? (roll < 20 ? 8 : (roll < 55 ? 1 : (roll < 85 ? 2 : 0)))
						 : (roll < 30 ? 8 : (roll < 55 ? 0 : (roll < 82 ? 1 : 2)));
	}

	if (family < 0 && region == building_style::BuildingRegion::Europe) {
		const bool green_forbidden = building_height > 30 ||
									 category == BuildingCategory::Industrial ||
									 category == BuildingCategory::Warehouse ||
									 category == BuildingCategory::Garage ||
									 category == BuildingCategory::Shed ||
									 category == BuildingCategory::Greenhouse;
		const double green_share =
				green_forbidden											  ? 0.0
				: era == ArchEra::Contemporary							  ? .25
				: era == ArchEra::Unknown || era == ArchEra::PostWarPanel ? .06
																		  : .03;
		if (rng.random_bool(green_share))
			return {MOSS_BLOCK, GRASS_BLOCK, COARSE_DIRT, COARSE_DIRT, style_seed};
		family = roll < 50 ? 5 : (roll < 80 ? 4 : 2);
	}
	if (family < 0) {
		int white = 30, light = 65, grey = 85;
		if (industrial) {
			white = 50;
			light = 85;
			grey = 95;
		} else if (era == ArchEra::HistoricOrnate || era == ArchEra::TraditionalPreWar) {
			white = 20;
			light = 50;
			grey = 75;
		} else if (era == ArchEra::Contemporary) {
			white = 45;
			light = 85;
			grey = 97;
		}
		family = roll < white ? 0 : (roll < light ? 1 : (roll < grey ? 2 : 3));
	}
	const auto &options = membrane_families()[static_cast<std::size_t>(family)];
	const auto index = static_cast<std::size_t>(
			rng.uniform(static_cast<std::uint32_t>(options.size())));
	const auto &selected = options[index];
	return {selected[0], selected[1], selected[2], std::nullopt, style_seed};
}

} // namespace arnis::buildings::rooftops
