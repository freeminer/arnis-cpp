#include "street_furniture.h"

#include "../block_definitions.h"
#include "connected_blocks.h"
#include "surfaces.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace arnis::street_furniture
{
namespace
{
constexpr int ROAD_FRAME_RADIUS = 8;

std::string tag(const tags_t &tags, const char *key)
{
	return tags.get(key);
}

std::pair<bool, std::pair<int, int>> road_frame(
		int x, int z, const RoadMaskBitmap &road_mask, std::uint64_t seed)
{
	double n = 0, sx = 0, sz = 0, sxx = 0, szz = 0;
	for (int dz = -ROAD_FRAME_RADIUS; dz <= ROAD_FRAME_RADIUS; ++dz)
		for (int dx = -ROAD_FRAME_RADIUS; dx <= ROAD_FRAME_RADIUS; ++dx) {
			if (dx * dx + dz * dz > ROAD_FRAME_RADIUS * ROAD_FRAME_RADIUS ||
					!road_mask.contains(x + dx, z + dz))
				continue;
			const double fx = dx, fz = dz;
			++n;
			sx += fx;
			sz += fz;
			sxx += fx * fx;
			szz += fz * fz;
		}
	if (n < 3) {
		const bool along_x = seed % 2 == 0;
		const int side = seed % 4 < 2 ? 1 : -1;
		return {along_x, along_x ? std::pair{0, side} : std::pair{side, 0}};
	}
	const double mx = sx / n, mz = sz / n;
	const bool along_x = sxx / n - mx * mx >= szz / n - mz * mz;
	auto sign = [](double v) { return v < 0 ? -1 : 1; };
	return {along_x, along_x ? std::pair{0, sign(mz)} : std::pair{sign(mx), 0}};
}

std::pair<int, int> frame_cell(
		int x, int z, bool along_x, std::pair<int, int> toward, int a, int d)
{
	const int ax = along_x ? 1 : 0, az = along_x ? 0 : 1;
	return {x + ax * a - toward.first * d, z + az * a - toward.second * d};
}

void post(WorldEditor &editor, const Block &block, int x, int z, int top)
{
	const int ground = editor.get_absolute_y(x, 0, z);
	for (int y = ground + 1; y <= top; ++y)
		editor.set_block_absolute(block, x, y, z);
}

enum class ShelterKind
{
	Transit,
	Pavilion,
	PavilionTable,
	LeanTo
};

std::optional<ShelterKind> shelter_kind(const tags_t &tags)
{
	const auto type = tag(tags, "shelter_type");
	if (type == "public_transport")
		return ShelterKind::Transit;
	if (type == "rock_shelter")
		return std::nullopt;
	if (type == "lean_to" || type == "basic_hut" || type == "weather_shelter" ||
			type == "field_shelter" || type == "changing_rooms")
		return ShelterKind::LeanTo;
	if (type == "gazebo" || type == "sun_shelter" || type == "pavilion")
		return ShelterKind::Pavilion;
	if (tag(tags, "bus") == "yes" || tags.contains("public_transport"))
		return ShelterKind::Transit;
	return ShelterKind::PavilionTable;
}

void transit_shelter(WorldEditor &editor, int x, int z, int base, bool along_x,
		std::pair<int, int> toward)
{
	auto cell = [&](int a, int d) { return frame_cell(x, z, along_x, toward, a, d); };
	for (const auto &[gx, gz] : [&] {
			 std::vector<std::pair<int, int>> v;
			 for (int a = -2; a <= 2; ++a)
				 v.push_back(cell(a, 1));
			 v.push_back(cell(-2, 0));
			 v.push_back(cell(2, 0));
			 return v;
		 }()) {
		const int ground = editor.get_absolute_y(gx, 0, gz);
		for (int y = ground + 1; y <= base + 2; ++y)
			connected_blocks::place_connected(editor, GLASS_PANE, gx, y, gz);
	}
	for (int a = -1; a <= 1; ++a) {
		const auto [bx, bz] = cell(a, 0);
		editor.set_block_absolute(OAK_SLAB_TOP, bx, base + 1, bz);
	}
	for (int a = -2; a <= 2; ++a)
		for (int d = 0; d <= 1; ++d) {
			const auto [rx, rz] = cell(a, d);
			editor.set_block_absolute(SMOOTH_STONE_SLAB, rx, base + 3, rz);
		}
}

BlockWithProperties hip_stair(int dx, int dz, int radius)
{
	using F = StairFacing;
	using S = StairShape;
	F facing;
	S shape = S::Straight;
	const int x_edge = dx == 0 ? 0 : (dx > 0 ? 1 : -1);
	const int z_edge = dz == 0 ? 0 : (dz > 0 ? 1 : -1);
	if (std::abs(dx) == radius && std::abs(dz) == radius) {
		if (x_edge > 0 && z_edge > 0) {
			facing = F::North;
			shape = S::OuterLeft;
		} else if (x_edge < 0 && z_edge > 0) {
			facing = F::North;
			shape = S::OuterRight;
		} else if (x_edge > 0) {
			facing = F::South;
			shape = S::OuterRight;
		} else {
			facing = F::South;
			shape = S::OuterLeft;
		}
	} else if (z_edge > 0)
		facing = F::North;
	else if (z_edge < 0)
		facing = F::South;
	else if (x_edge > 0)
		facing = F::West;
	else
		facing = F::East;
	return create_stair_with_properties(SPRUCE_STAIRS, facing, shape);
}

void pavilion(WorldEditor &editor, int x, int z, int base)
{
	for (const auto &[dx, dz] : {std::pair{-2, -2}, {2, -2}, {-2, 2}, {2, 2}})
		post(editor, SPRUCE_FENCE, x + dx, z + dz, base + 3);
	for (int radius = 1; radius <= 2; ++radius) {
		const int y = base + 6 - radius;
		for (int dz = -radius; dz <= radius; ++dz)
			for (int dx = -radius; dx <= radius; ++dx)
				if (std::max(std::abs(dx), std::abs(dz)) == radius)
					editor.set_block_with_properties_absolute(hip_stair(dx, dz, radius),
							x + dx, y, z + dz, std::nullopt, std::nullopt);
	}
	editor.set_block_absolute(SPRUCE_PLANKS, x, base + 5, z);
	editor.set_block_absolute(SPRUCE_SLAB, x, base + 6, z);
}

void picnic_table(WorldEditor &editor, int x, int z, int base, bool along_x)
{
	for (int a = -1; a <= 1; ++a)
		for (int side = -1; side <= 1; ++side) {
			const auto [cx, cz] =
					along_x ? std::pair{x + a, z + side} : std::pair{x + side, z + a};
			editor.set_block_absolute(
					side == 0 ? OAK_SLAB_TOP : OAK_SLAB, cx, base + 1, cz);
		}
}

void lean_to(WorldEditor &editor, int x, int z, int base, bool along_x,
		std::pair<int, int> toward)
{
	auto cell = [&](int a, int d) { return frame_cell(x, z, along_x, toward, a, d); };
	for (int a = -2; a <= 2; ++a)
		for (int d = 0; d <= 2; ++d)
			if (std::abs(a) == 2 || d == 2) {
				const auto [cx, cz] = cell(a, d);
				post(editor, SPRUCE_PLANKS, cx, cz, base + 2);
			}
	for (int a = -1; a <= 1; ++a) {
		const auto [bx, bz] = cell(a, 1);
		editor.set_block_absolute(SPRUCE_SLAB, bx, base + 1, bz);
	}
	for (int a = -2; a <= 2; ++a)
		for (int d = -1; d <= 2; ++d) {
			const auto [rx, rz] = cell(a, d);
			editor.set_block_absolute(SPRUCE_SLAB, rx, base + 3, rz);
		}
}
} // namespace

void generate_bicycle_parking_area(WorldEditor &editor, const ProcessedElement &element,
		const std::vector<std::pair<int32_t, int32_t>> &area)
{
	if (const auto surface = tag(element.tags(), "surface"); !surface.empty()) {
		if (const auto *blocks = surfaces::get_blocks_for_surface(surface);
				blocks && !blocks->empty())
			for (const auto &[x, z] : area)
				editor.set_block(blocks->front(), x, 0, z);
	}
	const auto parking = tag(element.tags(), "bicycle_parking");
	const bool covered = tag(element.tags(), "covered") == "yes" || parking == "shed" ||
						 parking == "building";
	if (!covered)
		return;
	for (const auto &node : element.nodes())
		for (int y = 1; y <= 3; ++y)
			editor.set_block(SPRUCE_FENCE, node.x, y, node.z);
	for (const auto &[x, z] : area)
		editor.set_block(SMOOTH_STONE_SLAB, x, 4, z);
}

void generate_shelter_node(
		WorldEditor &editor, const ProcessedNode &node, const RoadMaskBitmap &road_mask)
{
	const auto kind = shelter_kind(node.tags);
	if (!kind)
		return;
	const auto [along_x, toward] = road_frame(node.x, node.z, road_mask, node.id);
	const int base = editor.get_absolute_y(node.x, 0, node.z);
	switch (*kind) {
	case ShelterKind::Transit:
		transit_shelter(editor, node.x, node.z, base, along_x, toward);
		break;
	case ShelterKind::Pavilion:
		pavilion(editor, node.x, node.z, base);
		break;
	case ShelterKind::PavilionTable:
		pavilion(editor, node.x, node.z, base);
		picnic_table(editor, node.x, node.z, base, along_x);
		break;
	case ShelterKind::LeanTo:
		lean_to(editor, node.x, node.z, base, along_x, toward);
		break;
	}
}

} // namespace arnis::street_furniture
