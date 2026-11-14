#include "cemetery.h"

#include "area_layout.h"
#include "block_definitions.h"
#include "bush.h"
#include "connected_blocks.h"
#include "tree.h"
#include "../bresenham.h"
#include "../land_cover/land_cover.h"
#include "../structures/structures.h"
#include "../structures/tombstone.h"
#include "../ground_decoration.h"
#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <unordered_set>

namespace arnis::cemetery
{
namespace
{
constexpr std::string_view STYLE_TAG = "arnis:cemetery";
constexpr std::string_view SECTORS_TAG = "arnis:sectors";

bool cemetery_tags(const tags_t &tags)
{
	return tags.get("landuse") == "cemetery" || tags.get("amenity") == "grave_yard";
}

bool war_tags(const tags_t &tags)
{
	const auto value = tags.get("cemetery");
	return value == "war_cemetery" || value == "military";
}

enum class Style
{
	War,
	Lawn,
	Beds
};

Style style_of(const tags_t &tags)
{
	const auto style = tags.get(std::string(STYLE_TAG));
	if (style == "war" || war_tags(tags))
		return Style::War;
	if (style == "lawn")
		return Style::Lawn;
	return Style::Beds;
}

bool protected_ground(WorldEditor &editor, int x, int z)
{
	static const std::vector<Block> protected_blocks{BLACK_CONCRETE, GRAY_CONCRETE_POWDER,
			CYAN_TERRACOTTA, GRAY_CONCRETE, LIGHT_GRAY_CONCRETE, WHITE_CONCRETE,
			DIRT_PATH, SMOOTH_STONE, WATER};
	return editor.surface_is_sealed(x, z) ||
		   editor.check_for_block(
				   x, 0, z, std::optional<std::vector<Block>>(protected_blocks));
}

bool free_cell(const WorldEditor &editor,
		const element_processing::area_layout::AreaMask &mask,
		const RoadMaskBitmap &roads, const BuildingFootprintBitmap &footprints, int x,
		int z, bool avoid_crypt = true)
{
	return mask.contains(x, z) && !roads.contains(x, z) && !footprints.contains(x, z) &&
		   !editor.surface_is_sealed(x, z) && !editor.is_lc_water(x, z) &&
		   !(avoid_crypt && editor.place_prop(structures::Prop::Tombstone) &&
				   structures::tombstone::near_crypt(x, z));
}

Block grave_flower(std::uint64_t hash)
{
	static const std::array<Block, 8> flowers{RED_TULIP, PINK_TULIP, WHITE_TULIP,
			ORANGE_TULIP, RED_FLOWER, ALLIUM, CORNFLOWER, LILY_OF_THE_VALLEY};
	return flowers[hash % flowers.size()];
}

bool clear_around(const WorldEditor &editor, int x, int z)
{
	for (const auto [dx, dz] : {std::pair{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
		const int nx = x + dx, nz = z + dz;
		if (editor.block_exists_absolute(nx, editor.get_absolute_y(nx, 1, nz), nz))
			return false;
	}
	return true;
}

void headstone(WorldEditor &editor, int x, int z, std::pair<int, int> dir,
		std::uint64_t hash, bool low)
{
	if (!editor.owns(x, z))
		return;
	const auto facing = dir.first > 0	 ? StairFacing::East
						: dir.first < 0	 ? StairFacing::West
						: dir.second > 0 ? StairFacing::South
										 : StairFacing::North;
	const int y = editor.get_absolute_y(x, 1, z);
	const auto variant = hash % 20;
	if (variant <= 8) {
		static const std::array<Block, 5> tablets{STONE_BRICK_STAIRS,
				POLISHED_ANDESITE_STAIRS, DIORITE_STAIRS, GRANITE_STAIRS,
				MOSSY_COBBLESTONE_STAIRS};
		const auto stair = create_stair_with_properties(
				tablets[(hash >> 5) % tablets.size()], facing, StairShape::Straight);
		if (!editor.block_exists_absolute(x, y, z))
			editor.set_block_with_properties_absolute(stair, x, y, z);
	} else if (variant <= 14) {
		static const std::array<Block, 4> stones{
				POLISHED_ANDESITE, POLISHED_GRANITE, POLISHED_DIORITE, STONE_BRICKS};
		editor.set_block(stones[(hash >> 5) % stones.size()], x, 1, z);
	} else if (variant <= 16) {
		editor.set_block(POLISHED_ANDESITE, x, 1, z);
		editor.set_block(ANDESITE_WALL, x, 2, z);
	} else {
		editor.set_block(low ? SMOOTH_STONE_SLAB : POLISHED_DIORITE, x, 1, z);
	}
}

struct Grave
{
	std::pair<int, int> head;
	std::uint64_t hash;
};

struct RowPlan
{
	std::uint64_t id;
	Style style;
	element_processing::area_layout::Frame frame;
	element_processing::area_layout::AreaMask &mask;
	const RoadMaskBitmap &roads;
	const BuildingFootprintBitmap &footprints;
	double slot, row;
	int length;
	unsigned occupancy;
	std::pair<int, int> dir;

	std::optional<Grave> candidate(
			const WorldEditor &editor, std::int64_t s, std::int64_t r) const
	{
		const auto hash = element_processing::area_layout::cell_hash(id, s, r);
		if (hash % 100 >= occupancy)
			return std::nullopt;
		const auto head = frame.column(
				(static_cast<double>(s) + .5) * slot, static_cast<double>(r) * row + .5);
		for (int k = 0; k < length; ++k)
			if (!free_cell(editor, mask, roads, footprints, head.first + k * dir.first,
						head.second + k * dir.second, style != Style::War))
				return std::nullopt;
		const int bx = head.first - dir.first, bz = head.second - dir.second;
		if (!mask.contains(bx, bz) || roads.contains(bx, bz))
			return std::nullopt;
		return Grave{head, hash};
	}

	void place(WorldEditor &editor)
	{
		const auto bounds = editor.bounds();
		const auto span = frame.local_span({std::get<0>(bounds), std::get<1>(bounds),
				std::get<2>(bounds), std::get<3>(bounds)});
		const auto a0 = std::max(0.0, span[0].first);
		const auto a1 = std::min(frame.length(), span[0].second);
		const auto c0 = std::max(0.0, span[1].first);
		const auto c1 = std::min(frame.width(), span[1].second);
		if (a0 > a1 || c0 > c1)
			return;
		const auto s0 = static_cast<std::int64_t>(std::floor(a0 / slot)) - 1;
		const auto s1 = static_cast<std::int64_t>(std::ceil(a1 / slot));
		const auto r0 = static_cast<std::int64_t>(std::floor(c0 / row)) - 1;
		const auto r1 = static_cast<std::int64_t>(std::ceil(c1 / row));
		for (auto r = r0; r <= r1; ++r) {
			for (auto s = s0; s <= s1; ++s) {
				auto g = candidate(editor, s, r);
				if (!g)
					continue;
				bool overlaps = false;
				for (const auto [ds, dr] :
						{std::pair{-1, 0}, {-1, -1}, {0, -1}, {1, -1}}) {
					auto prior = candidate(editor, s + ds, r + dr);
					if (!prior)
						continue;
					for (int a = 0; a < length; ++a)
						for (int b = 0; b < length; ++b)
							overlaps |= g->head.first + a * dir.first ==
												prior->head.first + b * dir.first &&
										g->head.second + a * dir.second ==
												prior->head.second + b * dir.second;
				}
				if (overlaps || !editor.owns(g->head.first, g->head.second))
					continue;
				const auto h = g->hash >> 8;
				if (style == Style::War) {
					editor.set_block(SMOOTH_QUARTZ, g->head.first, 1, g->head.second);
				} else if (style == Style::Lawn) {
					headstone(editor, g->head.first, g->head.second, dir, h, true);
					const int fx = g->head.first + dir.first;
					const int fz = g->head.second + dir.second;
					if (h % 7 < 2 && editor.owns(fx, fz))
						editor.set_block(grave_flower(h >> 4), fx, 1, fz);
				} else {
					headstone(editor, g->head.first, g->head.second, dir, h, false);
				}
			}
		}
	}
};

struct BedPlan
{
	std::uint64_t id;
	element_processing::area_layout::AreaMask &mask;
	std::pair<int, int> origin, along, dir;
	bool sections;

	std::pair<int, int> at(int u, int v) const
	{
		return {origin.first + u * along.first + v * dir.first,
				origin.second + u * along.second + v * dir.second};
	}
	std::pair<int, int> uv(int x, int z) const
	{
		const int dx = x - origin.first, dz = z - origin.second;
		return {dx * along.first + dz * along.second, dx * dir.first + dz * dir.second};
	}
	bool on_path(int x, int z) const
	{
		const auto [u, v] = uv(x, z);
		const int k = u >= 0 ? u / 4 : (u - 3) / 4;
		const int r = v >= 0 ? v / 5 : (v - 4) / 5;
		const int um = (u % 4 + 4) % 4, vm = (v % 5 + 5) % 5;
		return sections && (((r + 1) % 8 == 0 && vm >= 1 && vm < 4) ||
								   ((k + 1) % 7 == 0 && um >= 1 && um < 3));
	}
};

void generate_wall(WorldEditor &editor, const ProcessedWay &way,
		const RoadMaskBitmap &roads, const BuildingFootprintBitmap &footprints)
{
	std::vector<std::pair<int, int>> line;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		for (const auto &[x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
					 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
			(void)y;
			line.emplace_back(x, z);
		}
	}
	auto wall_line = connected_blocks::four_connected_line(line);
	std::unordered_set<std::uint64_t> wall;
	const auto key = [](int x, int z) {
		return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
	};
	for (const auto &[x, z] : wall_line)
		if (!roads.contains(x, z) && !footprints.contains(x, z))
			wall.insert(key(x, z));
	for (const auto &[x, z] : wall_line) {
		if (!wall.contains(key(x, z)) || !editor.in_bounds(x, z))
			continue;
		const int ground = editor.get_ground_level(x, z);
		const auto joins = [&](int dx, int dz) {
			return wall.contains(key(x + dx, z + dz)) &&
				   editor.get_ground_level(x + dx, z + dz) == ground;
		};
		const auto block = connected_blocks::connected_wall(
				STONE_BRICK_WALL, joins(0, -1), joins(0, 1), joins(1, 0), joins(-1, 0));
		const int y = editor.get_absolute_y(x, 1, z);
		if (!editor.block_exists_absolute(x, y, z))
			editor.set_block_with_properties_absolute(block, x, y, z);
		editor.set_block(STONE_BRICK_SLAB, x, 2, z);
	}
}

} // namespace

Block flower(std::uint64_t hash)
{
	return grave_flower(hash);
}

bool claims(const tags_t &tags)
{
	return cemetery_tags(tags) || tags.get("cemetery") == "sector";
}

void prepare(std::vector<ProcessedElement> &elements, decals::SignRegion region)
{
	const bool lawn_region = region == decals::SignRegion::NorthAmerica ||
							 region == decals::SignRegion::Canada ||
							 region == decals::SignRegion::Oceania ||
							 region == decals::SignRegion::UkIreland;
	const std::string regional_style = lawn_region ? "lawn" : "beds";
	std::vector<std::pair<std::size_t, std::pair<double, double>>> sectors;
	for (std::size_t i = 0; i < elements.size(); ++i) {
		if (!elements[i].is_way())
			continue;
		const auto &way = elements[i].as_way();
		if (way.tags.get("cemetery") == "sector" && !cemetery_tags(way.tags) &&
				way.nodes.size() >= 3)
			if (const auto point =
							element_processing::area_layout::interior_point(way.nodes))
				sectors.emplace_back(i, *point);
	}
	std::vector<std::size_t> containers, war_sectors;
	if (!sectors.empty()) {
		for (std::size_t i = 0; i < elements.size(); ++i) {
			const tags_t *tags = nullptr;
			std::vector<const std::vector<ProcessedNode> *> rings;
			if (elements[i].is_way() && cemetery_tags(elements[i].as_way().tags)) {
				const auto &way = elements[i].as_way();
				tags = &way.tags;
				rings.push_back(&way.nodes);
			} else if (elements[i].is_relation() &&
					   cemetery_tags(elements[i].as_relation().tags)) {
				const auto &relation = elements[i].as_relation();
				tags = &relation.tags;
				for (const auto &member : relation.members)
					if (member.role == ProcessedMemberRole::Outer && member.way)
						rings.push_back(&member.way->nodes);
			} else {
				continue;
			}
			std::vector<std::size_t> contained;
			for (const auto &[sector_index, point] : sectors) {
				if (sector_index == i)
					continue;
				if (element_processing::area_layout::point_in_rings(rings, point))
					contained.push_back(sector_index);
			}
			if (!contained.empty()) {
				containers.push_back(i);
				if (war_tags(*tags))
					war_sectors.insert(
							war_sectors.end(), contained.begin(), contained.end());
			}
		}
	}
	auto set_tag = [](ProcessedElement &element, const char *key,
						   const std::string &value) {
		if (element.is_way())
			element.as_way().tags.insert_or_assign(key, value);
		else if (element.is_relation())
			std::get<ProcessedRelation>(element).tags.insert_or_assign(key, value);
	};
	for (auto &element : elements) {
		tags_t *tags = element.is_way() ? &element.as_way().tags
					   : element.is_relation()
							   ? &std::get<ProcessedRelation>(element).tags
							   : nullptr;
		if (tags && claims(*tags))
			set_tag(element, STYLE_TAG.data(), war_tags(*tags) ? "war" : regional_style);
	}
	for (auto i : containers)
		set_tag(elements[i], SECTORS_TAG.data(), "yes");
	for (auto i : war_sectors)
		set_tag(elements[i], STYLE_TAG.data(), "war");
}

bool is_mapped_grave(const tags_t &tags)
{
	for (const char *key : {"level", "layer"}) {
		const auto text = tags.get(key);
		if (!text.empty()) {
			try {
				if (std::stod(text) < 0.0)
					return false;
			} catch (...) {
			}
		}
	}
	const auto tomb = tags.get("tomb");
	const bool headstone = tomb.empty() || tomb == "gravestone" || tomb == "headstone" ||
						   tomb == "grave" || tomb == "war_grave";
	return headstone &&
		   (tags.get("cemetery") == "grave" || tags.get("historic") == "tomb");
}

void generate_cemetery(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const bridges::BridgeSurfaceMap &bridges)
{
	cemetery_area(editor, way, args, fills, footprints, roads, bridges, nullptr);
}

void cemetery_area(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const bridges::BridgeSurfaceMap &bridges,
		const Holes *holes)
{
	const auto fill = fills.get_or_compute(way, args.timeout);
	if (!fill || fill->empty())
		return;
	const auto style = style_of(way.tags);
	const bool has_sectors = way.tags.get(std::string(SECTORS_TAG)) == "yes";
	const auto frame = has_sectors
							   ? std::optional<element_processing::area_layout::Frame>{}
							   : element_processing::area_layout::Frame::of(way.nodes);
	bool paths = false;
	int min_x = std::numeric_limits<int>::max(), min_z = min_x;
	int max_x = std::numeric_limits<int>::min(), max_z = max_x;
	element_processing::area_layout::AreaMask mask(*fill, [&](int x, int z) {
		paths |= roads.contains(x, z);
		min_x = std::min(min_x, x);
		min_z = std::min(min_z, z);
		max_x = std::max(max_x, x);
		max_z = std::max(max_z, z);
		return !in_hole(holes, x, z);
	});
	std::optional<BedPlan> beds;
	if (frame && style == Style::Beds && min_x <= max_x) {
		auto axis = frame->along_axis();
		axis = {std::abs(axis.first), std::abs(axis.second)};
		beds.emplace(
				BedPlan{way.id, mask, {min_x, min_z}, axis, {axis.second, axis.first},
						!paths && std::min(frame->length(), frame->width()) >= 48.0});
	}
	for (const auto &[x, z] : *fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				protected_ground(editor, x, z))
			continue;
		const bool path = beds && beds->on_path(x, z);
		editor.set_block(path ? GRAVEL : GRASS_BLOCK, x, 0, z);
	}
	if (frame && !has_sectors && style != Style::Beds) {
		const int slot = style == Style::War ? 2 : 3;
		const int row = style == Style::War ? 3 : 4;
		const int length = style == Style::War ? 1 : 2;
		const unsigned occupancy = style == Style::War ? 94 : 72;
		RowPlan rows{way.id, style, *frame, mask, roads, footprints, double(slot),
				double(row), length, occupancy, frame->across_axis()};
		rows.place(editor);
	} else if (beds) {
		const int u0 = std::max(0, beds->uv(min_x, min_z).first - 2);
		const int u1 = std::max(beds->uv(max_x, max_z).first, u0);
		const int v0 = std::max(0, beds->uv(min_x, min_z).second - 2);
		const int v1 = std::max(beds->uv(max_x, max_z).second, v0);
		for (int r = v0 / 5; r <= v1 / 5 + 1; ++r) {
			if (beds->sections && (r + 1) % 8 == 0)
				continue;
			for (int k = u0 / 4; k <= u1 / 4 + 1; ++k) {
				if (beds->sections && (k + 1) % 7 == 0)
					continue;
				const auto h = element_processing::area_layout::cell_hash(way.id, k, r);
				const int u = k * 4, v = r * 5;
				auto fits = [&](int u0, int u1, int v0, int v1) -> std::optional<bool> {
					const auto [x, z] = beds->at(u0, v0);
					const int base = editor.get_ground_level(x, z);
					bool level = true;
					for (int du = u0; du <= u1; ++du)
						for (int dv = v0; dv <= v1; ++dv) {
							const auto [gx, gz] = beds->at(du, dv);
							if (!free_cell(editor, mask, roads, footprints, gx, gz))
								return std::nullopt;
							level &= editor.get_ground_level(gx, gz) == base;
						}
					const auto [fx, fz] = beds->at(u0, v0 - 1);
					if (!mask.contains(fx, fz) || roads.contains(fx, fz))
						return std::nullopt;
					return level;
				};
				const auto cross_fit = fits(u, u + 2, v, v + 3);
				if (h % 100 < 16 && cross_fit && *cross_fit) {
					const auto [cx, cz] = beds->at(u + 1, v + 2);
					if (editor.owns(cx, cz) &&
							editor.place_prop(structures::Prop::Tombstone)) {
						structures::tombstone::place_model(
								editor, true, {cx, cz}, beds->dir, h >> 8);
						continue;
					}
					if (editor.place_prop(structures::Prop::Tombstone))
						continue;
					if (editor.owns(cx, cz)) {
						editor.set_block(POLISHED_ANDESITE, cx, 1, cz);
						editor.set_block(ANDESITE_WALL, cx, 2, cz);
						editor.set_block(ANDESITE_WALL, cx, 3, cz);
						editor.set_block(ANDESITE_WALL, cx, 4, cz);
						const auto [ax0, az0] = beds->at(u, v + 2);
						const auto [ax1, az1] = beds->at(u + 2, v + 2);
						if (editor.owns(ax0, az0))
							editor.set_block(ANDESITE_WALL, ax0, 3, az0);
						if (editor.owns(ax1, az1))
							editor.set_block(ANDESITE_WALL, ax1, 3, az1);
						continue;
					}
				}
				for (int half = 0; half < 2; ++half) {
					const int hu = u + half * 2;
					const auto grave_hash = element_processing::area_layout::cell_hash(
							way.id ^ 0x6E4E, 2 * k + half, r);
					if (grave_hash % 100 >= 86)
						continue;
					const auto level = fits(hu, hu, v, v + 2);
					if (!level)
						continue;
					const auto [x, z] = beds->at(hu, v + 1);
					if (!editor.owns(x, z))
						continue;
					if (*level && editor.place_prop(structures::Prop::Tombstone)) {
						structures::tombstone::place_grave(
								editor, {x, z}, beds->dir, grave_hash >> 8);
					} else {
						headstone(editor, x, z, beds->dir, grave_hash >> 8, false);
					}
					for (int dv = 1; dv < 3; ++dv) {
						const auto [gx, gz] = beds->at(hu, v + dv);
						if (!editor.owns(gx, gz))
							continue;
						if (((grave_hash >> 12) & 1) == 0)
							editor.set_block(GRAVEL, gx, 0, gz,
									std::optional<std::vector<Block>>{{GRASS_BLOCK}},
									std::nullopt);
						else {
							editor.set_block(PODZOL, gx, 0, gz,
									std::optional<std::vector<Block>>{{GRASS_BLOCK}},
									std::nullopt);
							editor.set_block(
									grave_flower(grave_hash >> (dv * 4)), gx, 1, gz);
						}
					}
				}
			}
		}
	}
	for (const auto &[x, z] : *fill) {
		if (in_hole(holes, x, z) || !editor.owns(x, z) || editor.is_lc_water(x, z))
			continue;
		if (style != Style::War && !has_sectors)
			structures::tombstone::maybe_place_crypt(editor, x, z, roads);
		if (!editor.check_for_block(
					x, 0, z, std::optional<std::vector<Block>>{{GRASS_BLOCK}}) ||
				editor.block_exists_absolute(x, editor.get_absolute_y(x, 1, z), z))
			continue;
		const auto roll = land_cover::coord_hash(x ^ 0x6CE7, z) % 1000;
		const bool on_grid = ((x % 3) + 3) % 3 == 0 && ((z % 3) + 3) % 3 == 0;
		if (on_grid && roll < 30 && editor.land_cover_backs_trees(x, z) &&
				clear_around(editor, x, z)) {
			Tree::create(editor, {x, 1, z}, &footprints, &bridges);
		} else if (style != Style::War && roll >= 30 && roll < 34 &&
				   clear_around(editor, x, z)) {
			bush::place(editor, x, z, bush::Kind::Garden);
		} else if (style != Style::War && roll >= 34 && roll < 60) {
			editor.set_block(GRASS, x, 1, z);
		} else if (style == Style::Lawn && roll >= 60 && roll < 64) {
			ground_decoration::place_scattered_flower(
					editor, x, z, ground_decoration::FlowerSetting::Meadow);
		}
	}
	if (cemetery_tags(way.tags))
		generate_wall(editor, way, roads, footprints);
}

void generate_mapped_grave(WorldEditor &editor, const ProcessedNode &node,
		const BuildingFootprintBitmap &footprints)
{
	const int x = node.x, z = node.z;
	if (!editor.owns(x, z) || footprints.contains(x, z) || editor.surface_is_sealed(x, z))
		return;
	if (node.tags.contains("historic") || node.tags.contains("name")) {
		structures::tombstone::place_cross(editor, x, z);
	} else {
		const auto h = land_cover::coord_hash(x, z) >> 8;
		structures::tombstone::place_grave(editor, {x, z}, {0, 1}, h);
	}
}

} // namespace arnis::cemetery
