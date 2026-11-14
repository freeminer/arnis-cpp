#include "allotments.h"

#include "area_layout.h"
#include "block_definitions.h"
#include "cemetery.h"
#include "connected_blocks.h"
#include "world_editor.h"
#include "../land_cover/land_cover.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace arnis::allotments
{
namespace
{
namespace layout = arnis::element_processing::area_layout;
const std::array<Block, 8> CROPS{WHEAT, CARROTS, POTATOES, BEETROOTS, WHEAT_YOUNG,
		CARROTS_YOUNG, POTATOES_YOUNG, BEETROOTS_YOUNG};

std::uint64_t key_hash(
		std::uint64_t id, std::int64_t band, std::int64_t plot, std::int64_t side)
{
	return layout::cell_hash(id ^ static_cast<std::uint64_t>(side), band, plot);
}

enum class Kind
{
	Path,
	Hedge,
	Fence,
	Gate,
	Walk,
	Lawn,
	Flowers,
	Bed,
	Shed,
	Compost,
	Barrel
};

struct Colony
{
	std::uint64_t id;
	layout::Frame frame;
	layout::AreaMask mask;
	double width, depth, path;
	bool paths, sheds;

	double period() const { return path + depth * 2; }
	struct At
	{
		std::int64_t band, plot, side;
		double along, back;
	};
	std::optional<At> at(int x, int z) const
	{
		const auto [a, c] = frame.local(x, z);
		const auto p = period();
		const auto band = std::floor(c / p);
		const auto cc = c - band * p;
		if (cc < path)
			return std::nullopt;
		const bool first = cc < path + depth;
		const double back = first ? cc - path : p - cc;
		const auto pi = std::floor(a / width);
		return At{static_cast<std::int64_t>(band), static_cast<std::int64_t>(pi),
				first ? 0 : 1, a - pi * width, back};
	}
	std::uint64_t hash(const At &p) const { return key_hash(id, p.band, p.plot, p.side); }
	Block bed_crop(int x, int z) const
	{
		const auto p = at(x, z);
		if (!p)
			return WHEAT;
		const auto k = static_cast<std::int64_t>(std::floor((p->along - 1.0) / 4.0));
		const auto crop = layout::cell_hash(hash(*p), k, 7);
		return CROPS[crop % CROPS.size()];
	}
	bool in_shed(int x, int z) const
	{
		if (!sheds)
			return false;
		auto p = at(x, z);
		if (!p)
			return false;
		const auto h = hash(*p);
		const double w = 4, d = 3 + ((h >> 40) % 2);
		const double l0 = ((h >> 36) % 2) == 0 ? 1.5 : width - 5.5;
		const double b0 = depth - 1.5 - d;
		return p->back >= 1 && p->along >= l0 && p->along < l0 + w && p->back >= b0 &&
			   p->back < depth - 1.5;
	}
	Kind kind(int x, int z) const
	{
		auto p = at(x, z);
		if (!p)
			return paths ? Kind::Path : Kind::Lawn;
		const auto h = hash(*p);
		const bool hedge = h % 3 != 0;
		const auto edge = hedge ? Kind::Hedge : Kind::Fence;
		const double mid = width / 2;
		if (p->back < 1)
			return std::abs(p->along - mid) < .75 ? Kind::Gate : edge;
		if (p->along < 1 || (p->side == 0 && p->back >= depth - 1))
			return edge;
		if (in_shed(x, z))
			return Kind::Shed;
		if (std::abs(p->along - mid) < .75 && p->back < depth * .6)
			return Kind::Walk;
		const bool corner =
				std::abs(p->along - (width - 2)) < .5 && std::abs(p->back - 2) < .5;
		if (corner && ((h >> 44) % 2) == 0)
			return Kind::Compost;
		if (corner && ((h >> 44) % 4) == 1)
			return Kind::Barrel;
		switch ((h >> 8) % 4) {
		case 0:
		case 1:
			if (p->back >= 2 && p->back < depth * .62) {
				const auto k = std::floor((p->along - 1) / 4);
				return p->along - 1 - k * 4 < 3 ? Kind::Bed : Kind::Walk;
			}
			break;
		case 2:
			if (p->along < 2 || p->along >= width - 1)
				return Kind::Flowers;
			break;
		}
		return Kind::Lawn;
	}
};

void place_tree_column(WorldEditor &editor, int tx, int tz, int x, int z)
{
	const int dx = x - tx, dz = z - tz;
	const double radius = 1.8;
	const double horizontal = std::hypot(double(dx), double(dz));
	if (horizontal > radius + .5)
		return;
	const int base = editor.get_absolute_y(tx, 0, tz);
	if (dx == 0 && dz == 0) {
		editor.set_block_absolute(OAK_LOG, x, base + 1, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(OAK_LOG, x, base + 2, z, std::nullopt, std::nullopt);
	}
	const double crown_y = 2 + radius * .8;
	const int top = static_cast<int>(std::ceil(crown_y + radius * .8));
	for (int y = 3; y <= top; ++y) {
		const double dy = (y - crown_y) / .8;
		const double rough = (land_cover::coord_hash(x * 7 + y, z) % 100) / 250.0;
		if (horizontal * horizontal + dy * dy > std::pow(radius + .3 - rough, 2))
			continue;
		const bool accent = land_cover::coord_hash(x ^ y, z ^ 0x0A99) % 100 < 12;
		editor.set_block_absolute(accent ? FLOWERING_AZALEA_LEAVES : OAK_LEAVES, x,
				base + y, z, std::nullopt, std::nullopt);
	}
}
} // namespace

void prepare(std::vector<ProcessedElement> &elements, double scale)
{
	using Ring = std::vector<ProcessedNode>;
	std::vector<Ring> colonies;
	for (const auto &e : elements) {
		auto it = e.tags().find("landuse");
		if (it == e.tags().end() || it->second != "allotments")
			continue;
		if (e.is_way())
			colonies.push_back(e.as_way().nodes);
		else if (e.is_relation())
			for (const auto &m : e.as_relation().members)
				if (m.role == ProcessedMemberRole::Outer && m.way)
					colonies.push_back(m.way->nodes);
	}
	const double max_area = 100.0 * scale * scale;
	for (auto &e : elements) {
		if (!e.is_way())
			continue;
		auto &w = e.as_way();
		auto building = w.tags.find("building");
		if (building == w.tags.end() || w.nodes.size() < 4)
			continue;
		double area = 0;
		for (std::size_t i = 1; i < w.nodes.size(); ++i)
			area += double(w.nodes[i - 1].x) * w.nodes[i].z -
					double(w.nodes[i].x) * w.nodes[i - 1].z;
		if (std::abs(area) * .5 > max_area)
			continue;
		const auto &name = building->second;
		if (name != "yes" && name != "allotment_house" && name != "garden_house" &&
				name != "shed" && name != "hut" && name != "cabin" &&
				name != "summer_house")
			continue;
		const auto center = arnis::element_processing::area_layout::centroid(w.nodes);
		bool inside_colony = false;
		for (const auto &ring : colonies) {
			const auto b = arnis::element_processing::area_layout::bounds(ring);
			if (!b || !arnis::element_processing::area_layout::inside(*b, center))
				continue;
			const std::vector<const Ring *> rings{&ring};
			if (arnis::element_processing::area_layout::point_in_rings(rings, center)) {
				inside_colony = true;
				break;
			}
		}
		if (!inside_colony)
			continue;
		const auto h = arnis::element_processing::area_layout::cell_hash(w.id, 7, 11);
		static const std::array<const char *, 5> roofs{
				"gabled", "gabled", "gabled", "hipped", "skillion"};
		w.tags.try_emplace("building:material", "wood");
		w.tags.try_emplace("roof:shape", roofs[h % roofs.size()]);
		w.tags.try_emplace(
				"roof:material", (h >> 8) % 3 == 0 ? "roof_tiles" : "roofing_felt");
		if (!w.tags.contains("height"))
			w.tags.try_emplace("building:levels", "1");
	}
}

void generate_area(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints, const Holes *holes)
{
	auto frame = layout::Frame::of(way.nodes);
	if (!frame)
		return;
	bool has_paths = false, has_buildings = false;
	layout::AreaMask mask(fill, [&](int x, int z) {
		has_paths |= roads.contains(x, z);
		has_buildings |= footprints.contains(x, z);
		return !in_hole(holes, x, z);
	});
	const double scale = std::clamp(editor.scale(), .25, 4.0);
	const auto h = layout::cell_hash(way.id, 3, 5);
	Colony colony{way.id, *frame, std::move(mask), std::max(6.0, (10.0 + h % 5) * scale),
			std::max(8.0, (15.0 + (h >> 8) % 6) * scale), std::max(1.0, 2.0 * scale),
			!has_paths, !has_buildings};
	for (const auto &[x, z] : fill) {
		if (in_hole(holes, x, z) || !editor.in_bounds(x, z) ||
				editor.surface_is_sealed(x, z) ||
				editor.check_for_block(x, 0, z,
						std::optional<std::vector<Block>>{
								{BLACK_CONCRETE, GRAY_CONCRETE_POWDER, CYAN_TERRACOTTA,
										GRAY_CONCRETE, LIGHT_GRAY_CONCRETE,
										WHITE_CONCRETE, DIRT_PATH, SMOOTH_STONE, WATER}}))
			continue;
		const auto part = colony.kind(x, z);
		Block ground = GRASS_BLOCK;
		if (part == Kind::Path)
			ground = GRAVEL;
		else if (part == Kind::Walk || part == Kind::Gate)
			ground = DIRT_PATH;
		else if (part == Kind::Bed)
			ground = FARMLAND;
		editor.set_block(ground, x, 0, z, std::nullopt, std::nullopt);
		if (!editor.owns(x, z) || editor.is_lc_water(x, z) || footprints.contains(x, z))
			continue;
		if (part == Kind::Lawn || part == Kind::Flowers || part == Kind::Walk) {
			if (const auto p = colony.at(x, z)) {
				const auto tree_hash = key_hash(way.id, p->band, p->plot, p->side);
				if ((tree_hash >> 48) % 5 != 0) {
					const double along =
							(tree_hash >> 36) % 2 == 0 ? colony.width - 3.5 : 3.5;
					const double back = colony.depth * .72;
					const double c =
							p->band * colony.period() +
							(p->side == 0 ? colony.path + back : colony.period() - back);
					const auto tree =
							colony.frame.column(p->plot * colony.width + along, c);
					if (colony.mask.contains(tree.first, tree.second) &&
							!footprints.contains(tree.first, tree.second))
						place_tree_column(editor, tree.first, tree.second, x, z);
				}
			}
		}
		const auto ch = land_cover::coord_hash(x ^ 0x0A11, z);
		switch (part) {
		case Kind::Hedge:
			editor.set_block(OAK_LEAVES, x, 1, z, std::nullopt, std::nullopt);
			break;
		case Kind::Fence: {
			const auto gy = editor.get_ground_level(x, z);
			auto joins = [&](int dx, int dz) {
				const int nx = x + dx, nz = z + dz;
				return colony.kind(nx, nz) == Kind::Fence &&
					   colony.mask.contains(nx, nz) && !footprints.contains(nx, nz) &&
					   !roads.contains(nx, nz) && editor.get_ground_level(nx, nz) == gy;
			};
			auto fence = connected_blocks::connected_fence(
					SPRUCE_FENCE, joins(0, -1), joins(0, 1), joins(1, 0), joins(-1, 0));
			const auto y = editor.get_absolute_y(x, 1, z);
			if (!editor.block_exists_absolute(x, y, z))
				editor.set_block_with_properties_absolute(
						fence, x, y, z, std::nullopt, std::nullopt);
			break;
		}
		case Kind::Bed:
			editor.set_block(colony.bed_crop(x, z), x, 1, z, std::nullopt, std::nullopt);
			break;
		case Kind::Flowers:
			if (ch % 2 == 0)
				editor.set_block(
						cemetery::flower(ch >> 1), x, 1, z, std::nullopt, std::nullopt);
			break;
		case Kind::Compost:
			editor.set_block(COMPOSTER, x, 1, z, std::nullopt, std::nullopt);
			break;
		case Kind::Barrel:
			editor.set_block(BARREL, x, 1, z, std::nullopt, std::nullopt);
			break;
		case Kind::Shed: {
			const auto p = colony.at(x, z);
			const auto sh = p ? colony.hash(*p) : 0;
			static const std::array<Block, 4> walls{
					SPRUCE_PLANKS, OAK_PLANKS, BIRCH_PLANKS, DARK_OAK_PLANKS};
			const Block wall = walls[(sh >> 52) % walls.size()];
			const bool boundary = !colony.in_shed(x + 1, z) ||
								  !colony.in_shed(x - 1, z) ||
								  !colony.in_shed(x, z + 1) || !colony.in_shed(x, z - 1);
			if (boundary) {
				editor.set_block(wall, x, 1, z, std::nullopt, std::nullopt);
				editor.set_block(
						ch % 5 == 0 ? GLASS : wall, x, 2, z, std::nullopt, std::nullopt);
			}
			editor.set_block(DARK_OAK_SLAB, x, 3, z, std::nullopt, std::nullopt);
			break;
		}
		case Kind::Lawn:
			if (ch % 7 == 0)
				editor.set_block(GRASS, x, 1, z, std::nullopt, std::nullopt);
			break;
		default:
			break;
		}
	}
}
} // namespace arnis::allotments
