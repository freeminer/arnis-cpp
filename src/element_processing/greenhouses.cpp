#include "greenhouses.h"

#include "area_layout.h"
#include "block_definitions.h"
#include "../land_cover/land_cover.h"
#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace arnis::greenhouses
{
namespace
{
constexpr int SPAN = 4;

int across(int x, int z, bool along_x)
{
	return (along_x ? z : x) % SPAN < 0 ? (along_x ? z : x) % SPAN + SPAN
										: (along_x ? z : x) % SPAN;
}

int div_euclid(int value, int divisor)
{
	const int q = value / divisor;
	const int r = value % divisor;
	return r < 0 ? q - 1 : q;
}

enum class Crop
{
	Tomatoes,
	Flowers,
	Seedlings
};

Crop crop_of(std::uint64_t seed)
{
	switch (seed % 3) {
	case 0:
		return Crop::Tomatoes;
	case 1:
		return Crop::Flowers;
	default:
		return Crop::Seedlings;
	}
}

Block flower(std::uint64_t seed)
{
	static const std::array<Block, 8> FLOWERS{RED_TULIP, PINK_TULIP, WHITE_TULIP,
			ORANGE_TULIP, RED_FLOWER, ALLIUM, CORNFLOWER, LILY_OF_THE_VALLEY};
	return FLOWERS[seed % FLOWERS.size()];
}

void plant_column(
		WorldEditor &editor, Crop crop, int x, int floor_y, int z, std::uint64_t row)
{
	const Block soil = crop == Crop::Tomatoes ? DIRT : GRASS_BLOCK;
	editor.set_block_absolute(soil, x, floor_y, z, nullptr, nullptr);
	const auto put = [&](int y, Block block) {
		if (!editor.block_exists_absolute(x, y, z))
			editor.set_block_absolute(block, x, y, z, nullptr, nullptr);
	};
	switch (crop) {
	case Crop::Tomatoes: {
		const bool accent = land_cover::coord_hash(x, z ^ 0x70) % 10 == 0;
		put(floor_y + 1, AZALEA_LEAVES);
		put(floor_y + 2, accent ? FLOWERING_AZALEA_LEAVES : AZALEA_LEAVES);
		break;
	}
	case Crop::Flowers:
		put(floor_y + 1, flower(row));
		break;
	case Crop::Seedlings:
		put(floor_y + 1, row % 2 == 0 ? FERN : GRASS);
		break;
	}
}

bool protected_ground(WorldEditor &editor, int x, int z)
{
	static const std::optional<std::vector<Block>> PROTECTED{std::vector<Block>{
			BLACK_CONCRETE, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA, GRAY_CONCRETE,
			LIGHT_GRAY_CONCRETE, WHITE_CONCRETE, DIRT_PATH, SMOOTH_STONE, WATER}};
	return editor.surface_is_sealed(x, z) || editor.check_for_block(x, 0, z, PROTECTED);
}

} // namespace

bool is_greenhouse(const tags_t &tags)
{
	const auto building =
			tags.contains("building") ? tags.get("building") : tags.get("building:part");
	return building == "greenhouse" || building == "glasshouse";
}

bool wants_venlo_roof(const tags_t &tags)
{
	return is_greenhouse(tags) && !tags.contains("roof:shape");
}

void venlo_roof(WorldEditor &editor, const std::vector<std::pair<int, int>> &area,
		bool along_x, int roof_y)
{
	for (const auto &[x, z] : area) {
		switch (across(x, z, along_x)) {
		case 0:
			editor.set_block_absolute(SMOOTH_STONE_SLAB, x, roof_y, z, nullptr, nullptr);
			break;
		case 2:
			editor.set_block_absolute(AIR, x, roof_y, z, nullptr, nullptr);
			editor.set_block_absolute(GLASS, x, roof_y + 1, z, nullptr, nullptr);
			break;
		default:
			editor.set_block_absolute(GLASS, x, roof_y, z, nullptr, nullptr);
			break;
		}
	}
}

void plant_rows(WorldEditor &editor, const std::vector<std::pair<int, int>> &area,
		bool along_x, int floor_y, std::uint64_t seed)
{
	const element_processing::area_layout::AreaMask mask(
			area, [](int, int) { return true; });
	const auto crop = crop_of(seed);
	for (const auto &[x, z] : area) {
		if (!editor.owns(x, z) || across(x, z, along_x) % 2 != 0)
			continue;
		bool interior = true;
		for (int dx = -2; dx <= 2 && interior; ++dx)
			for (int dz = -2; dz <= 2; ++dz)
				if (!mask.contains(x + dx, z + dz)) {
					interior = false;
					break;
				}
		if (!interior)
			continue;
		const auto row =
				static_cast<std::uint64_t>(div_euclid(along_x ? z : x, 2)) ^ seed;
		plant_column(editor, crop, x, floor_y, z, row);
	}
}

void horticulture_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes)
{
	bool built = false;
	const element_processing::area_layout::AreaMask mask(fill, [&](int x, int z) {
		built |= footprints.contains(x, z);
		return !in_hole(holes, x, z);
	});
	const double scale = std::clamp(editor.scale(), 0.25, 4.0);
	const int hall_x = static_cast<int>(64.0 * scale);
	const int hall_z = static_cast<int>(40.0 * scale);
	const int lane = std::max(3, static_cast<int>(6.0 * scale));
	const int height = std::max(3, static_cast<int>(std::lround(4.0 * scale)));
	const int period_x = hall_x + lane, period_z = hall_z + lane;
	const auto cell = [&](int x, int z) -> std::optional<std::pair<int, int>> {
		const int ix = div_euclid(x, period_x), iz = div_euclid(z, period_z);
		const int lx = x - ix * period_x, lz = z - iz * period_z;
		return lx >= lane && lz >= lane ? std::optional(std::pair{ix, iz}) : std::nullopt;
	};
	const auto hall_at = [&](int x, int z) {
		return mask.contains(x, z) && cell(x, z).has_value() && !roads.contains(x, z) &&
			   !footprints.contains(x, z);
	};
	for (const auto &[x, z] : fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				protected_ground(editor, x, z))
			continue;
		const bool hall = !built && hall_at(x, z);
		const Block ground = built ? GRASS_BLOCK : hall ? GRAVEL : COARSE_DIRT;
		editor.set_block(ground, x, 0, z, {}, {});
		if (!hall || !editor.owns(x, z) || editor.is_lc_water(x, z))
			continue;
		const bool edge = !hall_at(x + 1, z) || !hall_at(x - 1, z) ||
						  !hall_at(x, z + 1) || !hall_at(x, z - 1);
		const int base = editor.get_absolute_y(x, 0, z);
		const int roof_y = base + height + 1;
		if (edge)
			for (int y = 1; y <= height; ++y)
				editor.set_block_absolute(
						WHITE_STAINED_GLASS, x, base + y, z, nullptr, nullptr);
		switch (z % SPAN < 0 ? z % SPAN + SPAN : z % SPAN) {
		case 0:
			editor.set_block_absolute(SMOOTH_STONE_SLAB, x, roof_y, z, nullptr, nullptr);
			break;
		case 2:
			editor.set_block_absolute(
					WHITE_STAINED_GLASS, x, roof_y + 1, z, nullptr, nullptr);
			break;
		default:
			editor.set_block_absolute(
					WHITE_STAINED_GLASS, x, roof_y, z, nullptr, nullptr);
			break;
		}
		if (!edge && z % 2 == 0) {
			const auto [ix, iz] = cell(x, z).value();
			const auto crop =
					crop_of(element_processing::area_layout::cell_hash(way.id, ix, iz));
			plant_column(editor, crop, x, base, z,
					(static_cast<std::uint64_t>(div_euclid(z, 2)) ^ way.id));
		}
	}
}

} // namespace arnis::greenhouses
