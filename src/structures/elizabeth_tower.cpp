#include "elizabeth_tower.h"

#include "../block_definitions.h"
#include "../element_processing/structure_kit.h"

#include <array>
#include <algorithm>
#include <map>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace arnis::structures::elizabeth_tower
{
namespace
{
using namespace block_definitions;
constexpr int SIDE = 17;
constexpr int C = 8;
constexpr int DIAL_Y = 54;
constexpr std::array<int, 6> BANDS{12, 19, 26, 33, 40, 46};
constexpr std::array<std::pair<int, int>, 7> ROOF{
		{{66, 5}, {67, 5}, {68, 4}, {69, 4}, {70, 4}, {71, 3}, {72, 3}}};
constexpr std::array<std::pair<std::pair<int, int>, std::pair<int, int>>, 4> FACES{
		{{{-1, 0}, {0, -1}}, {{0, -1}, {1, 0}}, {{1, 0}, {0, 1}}, {{0, 1}, {-1, 0}}}};

enum class PartKind : std::uint8_t
{
	Block,
	Stair,
	SlabTop,
};
struct Part
{
	Block block;
	PartKind kind = PartKind::Block;
	std::uint8_t face = 0;
	bool top = false;
};
struct PartLess
{
	bool operator()(const Part &a, const Part &b) const
	{
		return std::tuple{a.block.id(), a.kind, a.face, a.top} <
			   std::tuple{b.block.id(), b.kind, b.face, b.top};
	}
};
using Position = std::tuple<int, int, int>;

Part block(Block value)
{
	return {value, PartKind::Block, 0, false};
}
Part stair(Block value, int face, bool top)
{
	return {value, PartKind::Stair, static_cast<std::uint8_t>(face), top};
}

class Tower
{
public:
	std::map<Position, Part> cells;

	void set(int x, int y, int z, Part part) { cells[{x, y, z}] = part; }
	void face(int f, int u, int y, int d, Part part)
	{
		const auto [right, normal] = FACES[f];
		set(C + right.first * u + normal.first * d, y,
				C + right.second * u + normal.second * d, part);
	}
	void clear_face(int f, int u, int y, int d)
	{
		const auto [right, normal] = FACES[f];
		cells.erase({C + right.first * u + normal.first * d, y,
				C + right.second * u + normal.second * d});
	}
	template <typename Pick>
	void ring(int y, int half, Pick pick)
	{
		for (int f = 0; f < 4; ++f)
			for (int u = -half; u <= half; ++u)
				if (auto part = pick(f, u))
					face(f, u, y, half, *part);
	}
	void fill(int y, int half, Part part)
	{
		for (int x = C - half; x <= C + half; ++x)
			for (int z = C - half; z <= C + half; ++z)
				set(x, y, z, part);
	}
	void roof_layer(int y, int half, int next, Block tile, Block stair_block, Part hip)
	{
		ring(y, half, [&](int f, int u) -> std::optional<Part> {
			if (std::abs(u) == half)
				return hip;
			if (next < half)
				return stair(stair_block, f, false);
			return block(tile);
		});
	}
};

BlockWithProperties resolve(const Part &part)
{
	constexpr std::array<StairFacing, 4> inward{
			StairFacing::South, StairFacing::West, StairFacing::North, StairFacing::East};
	switch (part.kind) {
	case PartKind::Block:
		return BlockWithProperties::simple(part.block);
	case PartKind::Stair: {
		auto state = create_stair_with_properties(
				part.block, inward[part.face], StairShape::Straight);
		return part.top ? top_stair(std::move(state)) : state;
	}
	case PartKind::SlabTop:
		return structure_kit::top_slab(part.block);
	}
	return BlockWithProperties::simple(part.block);
}

void plinth(Tower &t)
{
	t.fill(0, 7, block(SMOOTH_SANDSTONE));
	for (int y = 1; y <= 4; ++y) {
		t.ring(y, 7, [&](int, int u) -> std::optional<Part> {
			return block(std::abs(u) == 7 || y == 4
								 ? SANDSTONE
								 : (u % 2 != 0 ? CUT_SANDSTONE : SMOOTH_SANDSTONE));
		});
		t.ring(y, 6,
				[](int, int) -> std::optional<Part> { return block(SMOOTH_SANDSTONE); });
	}
	t.ring(5, 7, [](int f, int u) -> std::optional<Part> {
		return std::abs(u) == 7 ? block(CUT_SANDSTONE_SLAB)
								: stair(SMOOTH_SANDSTONE_STAIRS, f, false);
	});
}

void shaft(Tower &t)
{
	for (int y = 5; y <= 46; ++y) {
		const bool band = std::find(BANDS.begin(), BANDS.end(), y) != BANDS.end();
		int below_band = 0;
		for (const int b : BANDS)
			if (b >= y) {
				below_band = b - y;
				break;
			}
		for (int f = 0; f < 4; ++f)
			for (int u = -6; u <= 6; ++u) {
				const bool panel = u % 2 != 0;
				std::optional<Part> front;
				Block back = SMOOTH_SANDSTONE;
				if (band) {
					front = block(panel ? CHISELED_SANDSTONE : SMOOTH_SANDSTONE);
				} else if (!panel) {
					front = block(std::abs(u) == 6 ? SANDSTONE : SMOOTH_SANDSTONE);
				} else {
					const bool window = (std::abs(u) == 1 || std::abs(u) == 3) &&
										below_band >= 2 && below_band <= 4;
					if (below_band == 1)
						front = stair(SMOOTH_SANDSTONE_STAIRS, f, true);
					back = window ? BLACK_STAINED_GLASS : CUT_SANDSTONE;
				}
				if (front)
					t.face(f, u, y, 6, *front);
				if (std::abs(u) < 6)
					t.face(f, u, y, 5, block(back));
			}
		if (band && y < 46)
			t.ring(y, 7, [](int, int) -> std::optional<Part> {
				return block(CUT_SANDSTONE_SLAB);
			});
	}
	for (int y = 5; y <= 19; ++y)
		t.ring(y, 7, [&](int f, int u) -> std::optional<Part> {
			if (std::abs(u) < 5)
				return std::nullopt;
			if (y < 19)
				return block(SANDSTONE);
			if (std::abs(u) == 7)
				return block(CUT_SANDSTONE_SLAB);
			return stair(SMOOTH_SANDSTONE_STAIRS, f, false);
		});
	t.ring(46, 7, [](int f, int u) -> std::optional<Part> {
		if (std::abs(u) == 7)
			return Part{CUT_SANDSTONE_SLAB, PartKind::SlabTop, 0, true};
		return stair(SMOOTH_SANDSTONE_STAIRS, f, true);
	});
}

std::pair<Part, Block> dial(int u, int dy)
{
	const auto in_disc = [](int x, int y) { return x * x + y * y <= 3.45 * 3.45; };
	if (dy == -5)
		return {block(u % 2 == 0 ? RAW_GOLD_BLOCK : LAPIS_BLOCK), SANDSTONE};
	if (std::abs(u) == 4 || std::abs(dy) == 4 || (std::abs(u) == 3 && std::abs(dy) == 3))
		return {block(RAW_GOLD_BLOCK), SANDSTONE};
	if (!in_disc(u, dy))
		return {block(CHISELED_SANDSTONE), SANDSTONE};
	const bool rim = !(in_disc(u + 1, dy) && in_disc(u - 1, dy) && in_disc(u, dy + 1) &&
					   in_disc(u, dy - 1));
	if (rim)
		return {block(LAPIS_BLOCK), SANDSTONE};
	if ((u == 0 && dy >= 0 && dy <= 2) || (u == -1 && dy == 0))
		return {block(LAPIS_BLOCK), SEA_LANTERN};
	return {block(WHITE_STAINED_GLASS), SEA_LANTERN};
}

void clock_stage(Tower &t)
{
	for (int y = 47; y <= 59; ++y) {
		const int dy = y - DIAL_Y;
		for (int f = 0; f < 4; ++f)
			for (int u = -7; u <= 7; ++u) {
				Part front;
				Block back = SMOOTH_SANDSTONE;
				if (y == 59 && std::abs(u) <= 5)
					front = block(u % 2 != 0 ? WHITE_CONCRETE : RAW_GOLD_BLOCK);
				else if (y == 47 || y == 59)
					front = block(SMOOTH_SANDSTONE);
				else if (std::abs(u) == 7) {
					front = block(CHISELED_SANDSTONE);
					back = SANDSTONE;
				} else if (std::abs(u) >= 5 || dy < -5) {
					front = block(CUT_SANDSTONE);
					back = SANDSTONE;
				} else {
					auto parts = dial(u, dy);
					front = parts.first;
					back = parts.second;
				}
				t.face(f, u, y, 7, front);
				if (std::abs(u) < 7)
					t.face(f, u, y, 6, block(back));
			}
	}
	t.ring(60, 7, [](int, int u) -> std::optional<Part> {
		return block(std::abs(u) == 7
							 ? SANDSTONE_WALL
							 : (u % 2 == 0 ? RAW_GOLD_BLOCK : CUT_SANDSTONE_SLAB));
	});
	for (int f = 0; f < 4; ++f) {
		for (int y = 61; y <= 62; ++y)
			t.face(f, 7, y, 7, block(SANDSTONE_WALL));
		t.face(f, 7, 63, 7, block(GOLD_BLOCK));
		for (int u : {-4, 4}) {
			t.face(f, u, 61, 7, block(SANDSTONE_WALL));
			t.face(f, u, 62, 7, block(LIGHTNING_ROD));
		}
	}
}

void belfry(Tower &t)
{
	for (int y = 60; y <= 65; ++y)
		for (int f = 0; f < 4; ++f)
			for (int u = -6; u <= 6; ++u) {
				const bool lancet = u % 2 == 0 && std::abs(u) <= 4;
				Part part;
				if (y == 65 && u % 2 == 0 && std::abs(u) < 6)
					part = block(RAW_GOLD_BLOCK);
				else if (y == 60 || y == 65)
					part = block(SMOOTH_SANDSTONE);
				else if (y >= 61 && y <= 63 && lancet) {
					t.clear_face(f, u, y, 6);
					continue;
				} else if (y == 64 && lancet)
					part = stair(SMOOTH_SANDSTONE_STAIRS, f, true);
				else if (std::abs(u) == 6)
					part = block(CHISELED_SANDSTONE);
				else
					part = block(SANDSTONE);
				t.face(f, u, y, 6, part);
			}
	t.ring(65, 7,
			[](int, int) -> std::optional<Part> { return block(CUT_SANDSTONE_SLAB); });
	for (int f = 0; f < 4; ++f) {
		for (int y = 66; y <= 69; ++y)
			t.face(f, 6, y, 6, block(SANDSTONE_WALL));
		t.face(f, 6, 70, 6, block(GOLD_BLOCK));
	}
}

void roof(Tower &t)
{
	for (std::size_t i = 0; i < ROOF.size(); ++i) {
		const auto [y, half] = ROOF[i];
		const int next = i + 1 < ROOF.size() ? ROOF[i + 1].second : 3;
		t.roof_layer(y, half, next, POLISHED_ANDESITE, POLISHED_ANDESITE_STAIRS,
				block(ANDESITE));
	}
	for (const auto &[y, half, us] :
			std::array<std::tuple<int, int, std::array<int, 3>>, 2>{
					{{67, 5, {-2, 0, 2}}, {70, 4, {0, 0, 0}}}}) {
		for (int f = 0; f < 4; ++f)
			for (int u : us) {
				if (y == 70 && u != 0)
					continue;
				t.face(f, u, y, half, block(BLACK_STAINED_GLASS));
				t.face(f, u, y + 1, half, block(POLISHED_ANDESITE_SLAB));
			}
	}
	t.ring(72, 3, [](int, int) -> std::optional<Part> { return block(RAW_GOLD_BLOCK); });
	for (int y = 73; y <= 75; ++y)
		t.ring(y, 3, [](int, int u) -> std::optional<Part> {
			return block(u % 2 == 0 ? BLACK_STAINED_GLASS : RAW_GOLD_BLOCK);
		});
	t.fill(76, 3, block(POLISHED_ANDESITE));
	t.ring(76, 3, [](int, int u) -> std::optional<Part> {
		return std::abs(u) == 3
					   ? block(RAW_GOLD_BLOCK)
					   : Part{POLISHED_ANDESITE_SLAB, PartKind::SlabTop, 0, true};
	});
	constexpr std::array<int, 14> halves{3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, 0};
	for (std::size_t i = 0; i < halves.size(); ++i) {
		const int y = 77 + static_cast<int>(i), half = halves[i];
		const int next = i + 1 < halves.size() ? halves[i + 1] : 0;
		if (!half) {
			t.set(C, y, C, block(ANDESITE_WALL));
			continue;
		}
		t.roof_layer(y, half, next, POLISHED_ANDESITE, POLISHED_ANDESITE_STAIRS,
				block(RAW_GOLD_BLOCK));
		t.fill(y, half - 1, block(POLISHED_ANDESITE));
	}
	t.set(C, 91, C, block(ANDESITE_WALL));
	t.set(C, 92, C, block(GOLD_BLOCK));
	for (int y = 93; y <= 95; ++y)
		t.set(C, y, C, block(LIGHTNING_ROD));
}

void interior(Tower &t)
{
	for (int y = 61; y <= 64; ++y)
		t.fill(y, 3, block(DARK_OAK_PLANKS));
	for (int y = 73; y <= 75; ++y)
		t.set(C, y, C, block(GLOWSTONE));
}

} // namespace

PalettizedSchematic build()
{
	Tower tower;
	plinth(tower);
	shaft(tower);
	clock_stage(tower);
	belfry(tower);
	roof(tower);
	interior(tower);

	PalettizedSchematic result;
	result.width = SIDE;
	result.length = SIDE;
	std::map<Part, std::uint8_t, PartLess> slots;
	result.voxels.reserve(tower.cells.size());
	for (const auto &[position, part] : tower.cells) {
		const auto [x, y, z] = position;
		auto found = slots.find(part);
		std::uint8_t slot;
		if (found == slots.end()) {
			if (result.palette.size() >= 256)
				throw std::runtime_error("Elizabeth Tower palette exceeds 256 states");
			slot = static_cast<std::uint8_t>(result.palette.size());
			result.palette.push_back(resolve(part));
			slots.emplace(part, slot);
		} else {
			slot = found->second;
		}
		result.voxels.push_back({static_cast<std::int16_t>(x),
				static_cast<std::int16_t>(y), static_cast<std::int16_t>(z), slot});
	}
	return result;
}

} // namespace arnis::structures::elizabeth_tower
