#include "hollywood_sign.h"

#include "../block_definitions.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <tuple>

namespace arnis::structures::hollywood_sign
{
namespace
{
using Rows = std::array<const char *, 14>;
constexpr Rows H{"###....###", "###....###", "###....###", "###....###", "###....###",
		"##########", "##########", "##########", "###....###", "###....###",
		"###....###", "###....###", "###....###", "###....###"};
constexpr Rows O{"./######\\.", "/########\\", "##########", "###r..7###", "###....###",
		"###....###", "###....###", "###....###", "###....###", "###....###",
		"###\\../###", "##########", "7########r", ".7######r."};
constexpr Rows L{"###......", "###......", "###......", "###......", "###......",
		"###......", "###......", "###......", "###......", "###......", "###......",
		"#########", "#########", "#########"};
constexpr Rows Y{"####...####", "####...####", "7###\\./###r", ".####.####.",
		".####.####.", ".7#######r.", "..#######..", "..7#####r..", "...#####...",
		"...#####...", "...#####...", "...#####...", "...#####...", "...#####..."};
constexpr Rows W{"###..##..###", "###\\.##./###", "####.##.####", "####.##.####",
		"7##########r", ".##########.", ".##########.", ".##########.", ".##########.",
		".##########.", ".####r7####.", ".7###..###r.", "..###..###..", "..###..###.."};
constexpr Rows D{"########\\.", "#########\\", "##########", "###...7###", "###....###",
		"###....###", "###....###", "###....###", "###....###", "###....###",
		"###.../###", "##########", "#########r", "########r."};

enum class Part : std::uint8_t
{
	Panel,
	Cut,
	Bars
};
struct CellPart
{
	Part kind = Part::Panel;
	bool east = false;
	bool top = false;
};
using Position = std::tuple<int, int, int>;
using Letter = std::tuple<const Rows *, int, int, bool>;
constexpr std::array<Letter, 9> LETTERS{{{&H, -53, 8, true}, {&O, -40, 11, true},
		{&L, -27, 9, false}, {&L, -17, 7, false}, {&Y, -7, 6, true}, {&W, 5, 1, true},
		{&O, 18, -1, true}, {&O, 31, -3, true}, {&D, 44, 2, true}}};
constexpr std::array<int, 5> GIRTS{2, 5, 8, 11, 13};
constexpr int LEG_DEPTH = 8;

struct Sign
{
	std::map<Position, CellPart> cells;
	void set(int x, int y, int z, CellPart part)
	{
		cells[{ANCHOR_X + x, GROUND + y, ANCHOR_Z + z}] = part;
	}
};

void letter(Sign &sign, const Rows &rows, int west, int front, bool boxed)
{
	const int width = static_cast<int>(std::char_traits<char>::length(rows[0]));
	for (int i = 0; i < 14; ++i) {
		const int y = 13 - i;
		for (int dx = 0; dx < width; ++dx) {
			const char ch = rows[i][dx];
			CellPart part;
			if (ch == '#') {
				part.kind = Part::Panel;
			} else if (ch == '/' || ch == '\\' || ch == '7' || ch == 'r') {
				part.kind = Part::Cut;
				part.east = ch == '/' || ch == '7';
				part.top = ch == '7' || ch == 'r';
			} else {
				continue;
			}
			sign.set(west + dx, y, front, part);
		}
	}
	const int back = front - 1;
	const std::array<int, 3> posts{1, width / 2, width - 2};
	for (int y = 1; y < 14; ++y) {
		for (int dx = 0; dx < width; ++dx) {
			const char ch = rows[13 - y][dx];
			const bool behind_panel = ch != '.';
			const bool post = std::find(posts.begin(), posts.end(), dx) != posts.end() &&
							  (boxed || behind_panel);
			const bool girt = std::find(GIRTS.begin(), GIRTS.end(), y) != GIRTS.end() &&
							  (boxed || behind_panel);
			if (post || girt)
				sign.set(west + dx, y, back, {Part::Bars});
		}
	}
	std::vector<int> bottom;
	for (int dx = 0; dx < width; ++dx)
		if (rows[13][dx] != '.')
			bottom.push_back(dx);
	if (bottom.size() >= 2)
		for (const int dx : {bottom.front() + 1, bottom.back() - 1})
			for (int y = -LEG_DEPTH; y < 0; ++y)
				sign.set(west + dx, y, front, {Part::Bars});
}

std::array<bool, 4> bar_joins(const Sign &sign, const Position &p)
{
	const auto [x, y, z] = p;
	constexpr std::array<Position, 4> offsets{
			{{0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {-1, 0, 0}}};
	std::array<bool, 4> joins{};
	for (std::size_t i = 0; i < offsets.size(); ++i) {
		const auto [dx, dy, dz] = offsets[i];
		const auto found = sign.cells.find({x + dx, y + dy, z + dz});
		joins[i] = found != sign.cells.end() && found->second.kind != Part::Cut;
	}
	return joins;
}

BlockWithProperties resolve(CellPart part, const std::array<bool, 4> &joins)
{
	using namespace block_definitions;
	if (part.kind == Part::Panel)
		return BlockWithProperties::simple(QUARTZ_BLOCK);
	if (part.kind == Part::Cut)
		return BlockWithProperties(QUARTZ_STAIRS,
				{{"facing", part.east ? "east" : "west"},
						{"half", part.top ? "top" : "bottom"}, {"shape", "straight"}});
	const auto side = [](bool connected) { return connected ? "true" : "false"; };
	return BlockWithProperties(
			IRON_BARS, {{"north", side(joins[0])}, {"south", side(joins[1])},
							   {"east", side(joins[2])}, {"west", side(joins[3])}});
}

} // namespace

PalettizedSchematic build()
{
	Sign sign;
	for (const auto &[rows, west, front, boxed] : LETTERS)
		letter(sign, *rows, west, front, boxed);
	PalettizedSchematic model;
	model.width = 110;
	model.length = 18;
	using PaletteKey = std::tuple<Part, bool, bool, std::array<bool, 4>>;
	std::map<PaletteKey, std::uint8_t> slots;
	for (const auto &[position, part] : sign.cells) {
		const auto sides = part.kind == Part::Bars ? bar_joins(sign, position)
												   : std::array<bool, 4>{};
		const PaletteKey key{part.kind, part.east, part.top, sides};
		auto it = slots.find(key);
		if (it == slots.end()) {
			const auto slot = static_cast<std::uint8_t>(model.palette.size());
			it = slots.emplace(key, slot).first;
			model.palette.push_back(resolve(part, sides));
		}
		const auto [x, y, z] = position;
		model.voxels.push_back({static_cast<std::int16_t>(x),
				static_cast<std::int16_t>(y), static_cast<std::int16_t>(z), it->second});
	}
	return model;
}

} // namespace arnis::structures::hollywood_sign
