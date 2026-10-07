#include "construction_site.h"

#include "../bresenham.h"
#include "connected_blocks.h"
#include "../ground_generation.h"
#include "../land_cover/land_cover.h"
#include "block_definitions.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_set>
#include <utility>

namespace arnis::construction_site
{
namespace
{
constexpr int PLOT = 13;
constexpr int MAX_EXTENT = PLOT - 2;
const std::vector<Block> SITE_GROUND{GRAVEL, COARSE_DIRT, DIRT, MUD};

std::uint64_t key(int x, int z)
{
	return (std::uint64_t(static_cast<std::uint32_t>(x)) << 32) |
		   static_cast<std::uint32_t>(z);
}

int floor_div(int n, int d)
{
	int q = n / d;
	if (n % d < 0)
		--q;
	return q;
}

struct Site
{
	WorldEditor &editor;
	const BuildingFootprintBitmap &footprints;
	std::unordered_set<std::uint64_t> cells;

	bool open(int x, int z) const
	{
		if (!cells.contains(key(x, z)) || footprints.contains(x, z) ||
				editor.surface_is_sealed(x, z) || editor.is_lc_water(x, z))
			return false;
		const int y = editor.get_absolute_y(x, 0, z);
		return editor.check_for_block_absolute(
					   x, y, z, std::optional<std::vector<Block>>(SITE_GROUND)) &&
			   !editor.block_exists_absolute(x, y + 1, z);
	}

	std::optional<int> level_pad(const std::vector<std::pair<int, int>> &area) const
	{
		int lo = INT32_MAX, hi = INT32_MIN;
		for (auto [x, z] : area) {
			if (!open(x, z))
				return std::nullopt;
			const int y = editor.get_absolute_y(x, 1, z);
			lo = std::min(lo, y);
			hi = std::max(hi, y);
		}
		if (hi - lo > 1)
			return std::nullopt;
		for (auto [x, z] : area)
			for (int y = editor.get_absolute_y(x, 1, z); y < hi; ++y)
				editor.set_block_absolute(COARSE_DIRT, x, y, z);
		return hi;
	}
};

struct Rolls
{
	std::uint64_t value;
	std::uint64_t next(std::uint64_t n)
	{
		value = land_cover::coord_hash(static_cast<std::int32_t>(value >> 32),
				static_cast<std::int32_t>(value) ^ 0x517E5D);
		return value % std::max<std::uint64_t>(1, n);
	}
};

enum class Prop
{
	Heap,
	Stockpile,
	Timber,
	Foundation,
	Cabin,
	Container,
	Scaffold,
	Empty
};

Prop pick_prop(std::uint64_t roll)
{
	constexpr std::pair<Prop, std::uint64_t> weights[] = {{Prop::Heap, 22},
			{Prop::Stockpile, 18}, {Prop::Timber, 11}, {Prop::Foundation, 14},
			{Prop::Cabin, 9}, {Prop::Container, 9}, {Prop::Scaffold, 12},
			{Prop::Empty, 9}};
	std::uint64_t value = roll % 104;
	for (const auto &[prop, weight] : weights) {
		if (value < weight)
			return prop;
		value -= weight;
	}
	return Prop::Empty;
}

std::vector<std::tuple<int, int, int, int>> rectangle(
		int x, int z, int along, int across, bool turned)
{
	std::vector<std::tuple<int, int, int, int>> out;
	for (int u = 0; u < along; ++u)
		for (int v = 0; v < across; ++v)
			out.emplace_back(x + (turned ? v : u), z + (turned ? u : v), u, v);
	return out;
}

void fence(WorldEditor &editor, const ProcessedWay &way, const Site &site)
{
	std::vector<std::pair<int, int>> points;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		auto seg = bresenham_line(way.nodes[i - 1].x, 0, way.nodes[i - 1].z,
				way.nodes[i].x, 0, way.nodes[i].z);
		for (const auto &[x, y, z] : seg) {
			(void)y;
			points.emplace_back(x, z);
		}
	}
	for (auto [x, z] : connected_blocks::four_connected_line(points)) {
		if (site.footprints.contains(x, z) || editor.surface_is_sealed(x, z) ||
				editor.is_lc_water(x, z) ||
				editor.check_for_block(
						x, 0, z, std::optional<std::vector<Block>>{{WATER}}))
			continue;
		const int base = editor.get_absolute_y(x, 1, z);
		if (editor.block_exists_absolute(x, base, z))
			continue;
		connected_blocks::place_connected(editor, IRON_BARS, x, base, z);
		connected_blocks::place_connected(editor, IRON_BARS, x, base + 1, z);
	}
}

void prop(WorldEditor &editor, const Site &site, int x, int z, Rolls &r)
{
	Prop type = pick_prop(r.next(std::numeric_limits<std::uint64_t>::max()));
	const bool turned = r.next(2) != 0;
	int along = 6, across = 3;
	int radius = 0;
	switch (type) {
	case Prop::Heap:
		radius = 2 + r.next(2);
		along = across = 2 * radius + 1;
		break;
	case Prop::Stockpile:
		along = 2 + r.next(3);
		across = 2;
		break;
	case Prop::Timber:
		along = 4 + r.next(2);
		across = 2 + r.next(2);
		break;
	case Prop::Foundation:
		along = 5 + r.next(5);
		across = 4 + r.next(4);
		break;
	case Prop::Cabin:
	case Prop::Container:
		along = 6;
		across = 3;
		break;
	case Prop::Scaffold:
		along = 3 + r.next(6);
		across = 1 + r.next(3);
		break;
	case Prop::Empty:
		return;
	}
	const int w = turned ? across : along;
	const int d = turned ? along : across;
	x += r.next(MAX_EXTENT - w + 1);
	z += r.next(MAX_EXTENT - d + 1);
	if (!editor.owns(x, z))
		return;
	const auto cells = rectangle(x, z, along, across, turned);
	std::vector<std::pair<int, int>> coords;
	coords.reserve(cells.size());
	for (const auto &[cx, cz, u, v] : cells) {
		(void)u;
		(void)v;
		coords.emplace_back(cx, cz);
	}
	if (type == Prop::Heap) {
		const int cx = x + radius, cz = z + radius;
		const unsigned material = r.next(3);
		for (int dx = -radius; dx <= radius; ++dx)
			for (int dz = -radius; dz <= radius; ++dz) {
				const int h = std::clamp(
						static_cast<int>(std::floor(
								radius - std::sqrt(double(dx * dx + dz * dz)) + .7)),
						0, radius);
				if (!h || !site.open(cx + dx, cz + dz))
					continue;
				for (int y = 1; y <= h; ++y) {
					Block b = material == 0	  ? SAND
							  : material == 1 ? GRAVEL
							  : land_cover::coord_hash(cx + dx, (cz + dz) ^ y) % 3 == 0
									  ? COARSE_DIRT
									  : DIRT;
					editor.set_block(b, cx + dx, y, cz + dz);
				}
			}
		return;
	}
	auto base = site.level_pad(coords);
	if (!base)
		return;
	if (type == Prop::Stockpile) {
		const std::array<Block, 5> mats{
				BRICK, STONE_BRICKS, LIGHT_GRAY_CONCRETE, SPRUCE_PLANKS, SMOOTH_STONE};
		const Block b = mats[r.next(mats.size())];
		for (auto [cx, cz, u, v] : cells) {
			(void)u;
			(void)v;
			editor.set_block_absolute(b, cx, *base, cz);
			if (r.next(4))
				editor.set_block_absolute(b, cx, *base + 1, cz);
		}
	} else if (type == Prop::Timber) {
		const Block log = r.next(2) == 0 ? SPRUCE_LOG : OAK_LOG;
		const char *axis = turned ? "z" : "x";
		BlockWithProperties state{log, {{"axis", axis}}};
		const bool two_layers = r.next(2) == 0;
		const int max_v = across - 1;
		for (auto [cx, cz, u, v] : cells) {
			(void)u;
			editor.set_block_with_properties_absolute(
					state, cx, *base, cz, std::nullopt, std::nullopt);
			if (two_layers && v < max_v)
				editor.set_block_with_properties_absolute(
						state, cx, *base + 1, cz, std::nullopt, std::nullopt);
		}
	} else if (type == Prop::Foundation) {
		const bool tall_bars = r.next(2) == 0;
		for (auto [cx, cz, u, v] : cells) {
			const bool edge = u == 0 || v == 0 || u == along - 1 || v == across - 1;
			editor.set_block_absolute(LIGHT_GRAY_CONCRETE, cx, *base - 1, cz,
					std::optional<std::vector<Block>>(SITE_GROUND), std::nullopt);
			if (edge)
				editor.set_block_absolute(SPRUCE_SLAB, cx, *base, cz);
			else if (u % 2 && v % 2) {
				editor.set_block_absolute(IRON_BARS, cx, *base, cz);
				if (tall_bars)
					editor.set_block_absolute(IRON_BARS, cx, *base + 1, cz);
			}
		}
	} else if (type == Prop::Cabin || type == Prop::Container) {
		const bool cabin = type == Prop::Cabin;
		const Block wall =
				cabin ? (r.next(4) == 0 ? LIGHT_GRAY_CONCRETE : WHITE_CONCRETE)
					  : std::array<Block, 6>{BLUE_CONCRETE, ORANGE_CONCRETE, RED_CONCRETE,
								GREEN_CONCRETE, CYAN_CONCRETE, GRAY_CONCRETE}[r.next(6)];
		const int door = r.next(2) == 0 ? 0 : along - 1;
		for (auto [cx, cz, u, v] : cells) {
			const bool end = u == 0 || u == along - 1;
			const bool side = v == 0 || v == across - 1;
			for (int dy = 0; dy < 2; ++dy) {
				if (!(end || side) || (cabin && end && u == door && v == across / 2))
					continue;
				const Block b = cabin && side && !end && dy == 1 && u % 2 ? GLASS : wall;
				editor.set_block_absolute(b, cx, *base + dy, cz);
			}
			editor.set_block_absolute(wall, cx, *base + 2, cz);
		}
	} else if (type == Prop::Scaffold) {
		int h = 3 + r.next(4);
		std::vector<int> heights;
		heights.reserve(along);
		for (int u = 0; u < along; ++u) {
			heights.push_back(h);
			h = std::clamp(h + static_cast<int>(r.next(3)) - 1, 2, 7);
		}
		BlockWithProperties scaffold{SCAFFOLDING,
				{{"distance", "0"}, {"bottom", "false"}, {"waterlogged", "false"}}};
		for (const auto &[cx, cz, u, v] : cells) {
			if (u && u != along - 1 && r.next(8) == 0)
				continue;
			const int height = heights[u] - (v > 0 && r.next(3) == 0 ? 1 : 0);
			for (int dy = 0; dy < height; ++dy)
				editor.set_block_with_properties_absolute(
						scaffold, cx, *base + dy, cz, std::nullopt, std::nullopt);
		}
	}
}
} // namespace

bool is_arid(biome::Climate c)
{
	return c == biome::Climate::HotDesert || c == biome::Climate::HotSteppe ||
		   c == biome::Climate::ColdDesert || c == biome::Climate::ColdSteppe;
}

Block ground_block(int x, int z, bool arid)
{
	const double n = ground_generation::value_noise_01(x + 311, z - 173, 11);
	const auto speck = land_cover::coord_hash(x ^ 0x2C51, z) % 100;
	if (n < .22)
		return speck < 4 ? COARSE_DIRT : GRAVEL;
	if (n < .62)
		return speck < 3 ? GRAVEL : COARSE_DIRT;
	if (n < .84 || arid)
		return speck < 6 ? COARSE_DIRT : DIRT;
	return speck < 12 ? DIRT : MUD;
}

void furnish(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &area,
		const BuildingFootprintBitmap &footprints)
{
	if (area.empty())
		return;
	Site site{editor, footprints, {}};
	site.cells.reserve(area.size());
	int min_x = INT32_MAX, min_z = INT32_MAX, max_x = INT32_MIN, max_z = INT32_MIN;
	for (auto [x, z] : area) {
		site.cells.insert(key(x, z));
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
	}
	fence(editor, way, site);
	const auto salt = static_cast<std::int32_t>(way.id ^ (way.id >> 32));
	for (int gx = floor_div(min_x, PLOT); gx <= floor_div(max_x, PLOT); ++gx)
		for (int gz = floor_div(min_z, PLOT); gz <= floor_div(max_z, PLOT); ++gz) {
			const auto salted_z = static_cast<std::int32_t>(
					static_cast<std::uint32_t>(gz) + static_cast<std::uint32_t>(salt));
			Rolls rolls{land_cover::coord_hash(gx ^ salt, salted_z)};
			prop(editor, site, gx * PLOT + 1, gz * PLOT + 1, rolls);
		}
}
} // namespace arnis::construction_site
