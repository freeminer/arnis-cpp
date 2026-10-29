#include <string>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <unordered_set>
#include <vector>
#include <string_view>

#include "../bresenham.h"
#include "../strict_parse.h"
#include "../structures/structures.h"
#include "../mapillary/geometry.h"
#include "../unicode_text.h"
#include "../world_editor/floor_state.h"
#include "connected_blocks.h"
#include "waterfront.h"
#include "../../../arnis_adapter.h"
namespace arnis
{

namespace man_made
{

std::string lower_copy(std::string value);

namespace
{
const std::vector<Block> CANOPY_BLOCKS = {OAK_LEAVES, BIRCH_LEAVES, SPRUCE_LEAVES,
		DARK_OAK_LEAVES, JUNGLE_LEAVES, ACACIA_LEAVES, CHERRY_LEAVES, MANGROVE_LEAVES,
		AZALEA_LEAVES};

std::optional<std::pair<int, int>> anchor(const ProcessedElement &element)
{
	if (const auto node = element.first_node(); element.is_node() && node)
		return std::pair{node->x, node->z};
	const auto &nodes = element.nodes_vec();
	if (nodes.empty())
		return std::nullopt;
	const std::size_t count = nodes.size() > 2 && nodes.front().x == nodes.back().x &&
											  nodes.front().z == nodes.back().z
									  ? nodes.size() - 1
									  : nodes.size();
	long long x = 0, z = 0;
	for (std::size_t i = 0; i < count; ++i) {
		const auto &node = nodes[i];
		x += node.x;
		z += node.z;
	}
	return std::pair{static_cast<int>(x / static_cast<long long>(count)),
			static_cast<int>(z / static_cast<long long>(count))};
}

std::optional<double> tag_metres(const ProcessedElement &element, const char *key)
{
	const auto value = element.tag(key);
	if (!value)
		return std::nullopt;
	const auto parsed = mapillary::geodesy::parse_length_m(*value);
	return parsed && *parsed > 0.0 && *parsed < 1000.0 ? parsed : std::nullopt;
}

std::optional<int> roof_top(
		WorldEditor &editor, const CoordinateBitmap *footprints, int x, int z)
{
	if (!footprints || !footprints->contains(x, z))
		return std::nullopt;
	const int ground = editor.get_ground_level(x, z);
	return editor.highest_block_between(x, z, ground + 1, world_editor::world_max_y());
}

int feature_height(double metres, double scale, int ground, int base, int minimum)
{
	const int blocks = static_cast<int>(std::round(metres * scale));
	const int roof_height = base - ground;
	const int own_height =
			roof_height > 0 && blocks > roof_height + 1 ? blocks - roof_height : blocks;
	return std::clamp(own_height, minimum,
			std::max(minimum, world_editor::world_max_y() - base - 1));
}

std::optional<double> mast_metres(const tags_t &tags, bool on_roof)
{
	const std::string kind = tags.get("man_made");
	if (kind.empty())
		return std::nullopt;
	const bool lighting = tags.get("tower:type") == "lighting";
	double default_metres;
	if (kind == "chimney")
		default_metres = on_roof ? 2.0 : 40.0;
	else if (kind == "antenna")
		default_metres = on_roof ? 3.0 : lighting ? 20.0 : 10.0;
	else if (kind == "mast")
		default_metres = on_roof ? 6.0 : lighting ? 20.0 : 30.0;
	else
		return std::nullopt;
	const auto mapped = tags.get("height");
	if (!mapped.empty()) {
		const auto parsed = mapillary::geodesy::parse_length_m(mapped);
		if (parsed && *parsed > 0.0 && *parsed < 1000.0)
			return parsed;
	}
	return default_metres;
}

bool polygon_contains_cell_center(
		int x, int z, const std::vector<std::pair<int, int>> &polygon)
{
	if (polygon.size() < 3)
		return false;
	const double px = static_cast<double>(x) + .5;
	const double pz = static_cast<double>(z) + .5;
	bool inside = false;
	std::size_t j = polygon.size() - 1;
	for (std::size_t i = 0; i < polygon.size(); ++i) {
		const double xi = polygon[i].first, zi = polygon[i].second;
		const double xj = polygon[j].first, zj = polygon[j].second;
		if (((zi > pz) != (zj > pz)) && px < (xj - xi) * (pz - zi) / (zj - zi) + xi)
			inside = !inside;
		j = i;
	}
	return inside;
}

void mast_head(WorldEditor &editor, int x, int z, int y, int reach, bool lighting)
{
	const Block &block = lighting ? SEA_LANTERN : WHITE_CONCRETE;
	const int rows = lighting ? 1 : 2;
	for (const auto &[dx, dz] : {std::pair{0, -reach}, std::pair{0, reach},
				 std::pair{-reach, 0}, std::pair{reach, 0}})
		for (int dy = 0; dy < rows; ++dy)
			editor.set_block_absolute(
					block, x + dx, y + dy, z + dz, CANOPY_BLOCKS, std::nullopt);
}

void mast_cabinet(WorldEditor &editor, int x, int z)
{
	for (int dz = -1; dz <= 0; ++dz)
		for (int y = 1; y <= 2; ++y)
			editor.set_block(
					LIGHT_GRAY_CONCRETE, x + 3, y, z + dz, CANOPY_BLOCKS, std::nullopt);
}

void lattice_ring(WorldEditor &editor, int x, int z, int y, const Block &corner)
{
	const bool walls = corner.id() == ANDESITE_WALL.id();
	for (const auto &[dx, dz] :
			{std::pair{-1, -1}, std::pair{1, -1}, std::pair{-1, 1}, std::pair{1, 1}})
		if (walls) {
			editor.set_block_absolute(
					AIR, x + dx, y, z + dz, CANOPY_BLOCKS, std::nullopt);
			connected_blocks::place_connected(editor, corner, x + dx, y, z + dz);
		} else {
			editor.set_block_absolute(
					corner, x + dx, y, z + dz, CANOPY_BLOCKS, std::nullopt);
		}
	for (const auto &[dx, dz] :
			{std::pair{0, -1}, std::pair{0, 1}, std::pair{-1, 0}, std::pair{1, 0}}) {
		if (walls) {
			editor.set_block_absolute(
					AIR, x + dx, y, z + dz, CANOPY_BLOCKS, std::nullopt);
			connected_blocks::place_connected(editor, IRON_BARS, x + dx, y, z + dz);
		} else {
			const auto bars = connected_blocks::connected_iron_bars(
					dx != 0, dx != 0, dz != 0, dz != 0);
			editor.set_block_with_properties_absolute(bars, x + dx, y, z + dz,
					std::optional<std::vector<Block>>{CANOPY_BLOCKS}, std::nullopt);
		}
	}
}

bool summit_cross_tagged(const tags_t &tags)
{
	const auto natural = tags.get("natural");
	return (natural == "peak" || natural == "volcano" || natural == "hill") &&
		   tags.get("summit:cross") == "yes" && tags.get("man_made") != "cross";
}

void place_cross(
		WorldEditor &editor, const ProcessedNode &node, const Args &args, bool summit)
{
	const int ground = editor.get_ground_level(node.x, node.z);
	const double metres =
			tag_metres(ProcessedElement(node), "height").value_or(summit ? 5.0 : 6.0);
	const int height =
			std::min(40, feature_height(metres, args.scale, ground, ground, 3));
	auto material = node.tags.get("material");
	std::transform(material.begin(), material.end(), material.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	const bool metal = material == "metal" || material == "steel" || material == "iron" ||
					   material == "aluminium";
	const bool solid = height >= 10;
	const Block &block = metal ? (solid ? IRON_BLOCK : IRON_BARS)
						 : (material == "concrete" || material == "stone")
								 ? WHITE_CONCRETE
						 : solid ? DARK_OAK_PLANKS
								 : DARK_OAK_FENCE;
	int base = ground;
	if (summit) {
		editor.set_block_absolute(COBBLESTONE, node.x, base + 1, node.z);
		for (const auto &[dx, dz] :
				{std::pair{1, 0}, std::pair{-1, 0}, std::pair{0, 1}, std::pair{0, -1}})
			editor.set_block_absolute(
					COBBLESTONE_SLAB, node.x + dx, base + 1, node.z + dz);
		++base;
	}
	const int arm_y = base + (height * 2 + 2) / 3;
	const int half = std::max(1, height / 5);
	auto put = [&](int x, int y, int z) {
		if (connected_blocks::is_connectable(block))
			connected_blocks::place_connected(editor, block, x, y, z);
		else
			editor.set_block_absolute(block, x, y, z);
	};
	for (int y = base + 1; y <= base + height; ++y)
		put(node.x, y, node.z);
	for (int k = -half; k <= half; ++k)
		if (node.id % 2 == 0)
			put(node.x + k, arm_y, node.z);
		else
			put(node.x, arm_y, node.z + k);
}
} // namespace

bool is_summit_cross(const tags_t &tags)
{
	return summit_cross_tagged(tags);
}

std::vector<std::pair<int, int>> mapped_crane_points(
		const std::vector<ProcessedElement> &elements)
{
	std::vector<std::pair<int, int>> points;
	for (const auto &element : elements)
		if (element.tag("man_made") == std::optional<std::string>("crane"))
			if (const auto point = anchor(element))
				points.push_back(*point);
	return points;
}

std::size_t drop_doubled_summit_crosses(
		std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<std::pair<int, int>> crosses;
	for (const auto &element : elements)
		if (element.is_node() && element.as_node().tags.get("man_made") == "cross")
			crosses.emplace_back(element.as_node().x, element.as_node().z);
	if (crosses.empty())
		return 0;
	const double reach = std::max(1.0, 30.0 * scale);
	std::size_t dropped = 0;
	for (auto &element : elements) {
		if (!element.is_node() || !summit_cross_tagged(element.as_node().tags))
			continue;
		auto &node = std::get<ProcessedNode>(element);
		const bool near_ =
				std::any_of(crosses.begin(), crosses.end(), [&](const auto &p) {
					return std::hypot(static_cast<double>(p.first) - node.x,
								   static_cast<double>(p.second) - node.z) <= reach;
				});
		if (near_ && node.tags.erase("summit:cross") > 0)
			++dropped;
	}
	return dropped;
}

void generate_summit_cross(
		WorldEditor &editor, const ProcessedNode &node, const Args &args)
{
	place_cross(editor, node, args, true);
}

void generate_antenna(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const CoordinateBitmap *footprints)
{
	const auto point = anchor(element);
	if (!point)
		return;
	const auto &[x, z] = *point;
	const bool lighting = element.tag("tower:type") == "lighting";
	const int ground = editor.get_ground_level(x, z);
	const auto roof = roof_top(editor, footprints, x, z);
	const int base = roof.value_or(ground);
	const auto mapped_metres = mast_metres(element.tags(), roof.has_value());
	if (!mapped_metres)
		return;
	const double metres = *mapped_metres;
	const int height = feature_height(metres, args.scale, ground, base, 2);
	const int top = base + height;
	const auto construction = element.tag("tower:construction");
	const bool tube =
			roof || height <= 12 ||
			(construction &&
					(*construction == "monopole" || *construction == "tube" ||
							*construction == "tubular" || *construction == "guyed_tube" ||
							*construction == "concrete"));
	const bool banded = metres >= 100.0;
	if (tube && (roof || height <= 12)) {
		for (int y = base + 1; y < top; ++y)
			editor.set_block_absolute(IRON_BARS, x, y, z, CANOPY_BLOCKS, std::nullopt);
		editor.set_block_absolute(LIGHTNING_ROD, x, top, z, CANOPY_BLOCKS, std::nullopt);
		if (height >= 6)
			mast_head(editor, x, z, top - 2, 1, lighting);
		return;
	}
	if (tube) {
		const bool thick = height >= 60;
		const std::vector<std::pair<int, int>> section =
				thick ? std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {-1, 0}, {0, 1},
								{0, -1}}
					  : std::vector<std::pair<int, int>>{{0, 0}};
		for (int y = base + 1; y < top - 2; ++y) {
			const int band = (y - base - 1) * 7 / std::max(1, top - 2 - base);
			const Block &block = banded ? (band % 2 == 0 ? RED_CONCRETE : WHITE_CONCRETE)
										: LIGHT_GRAY_CONCRETE;
			for (const auto &[dx, dz] : section)
				editor.set_block_absolute(
						block, x + dx, y, z + dz, CANOPY_BLOCKS, std::nullopt);
		}
		for (int y = top - 2; y < top; ++y)
			editor.set_block_absolute(IRON_BARS, x, y, z, CANOPY_BLOCKS, std::nullopt);
		editor.set_block_absolute(LIGHTNING_ROD, x, top, z, CANOPY_BLOCKS, std::nullopt);
		mast_head(editor, x, z, top - 4, thick ? 2 : 1, lighting);
		mast_cabinet(editor, x, z);
	} else {
		const int lattice_top = top - 4;
		for (int y = base + 1; y <= lattice_top; ++y) {
			const int band = (y - base - 1) * 7 / std::max(1, lattice_top - base);
			const Block &corner = banded ? (band % 2 == 0 ? RED_CONCRETE : WHITE_CONCRETE)
										 : ANDESITE_WALL;
			lattice_ring(editor, x, z, y, corner);
		}
		for (int y = lattice_top + 1; y < top; ++y)
			editor.set_block_absolute(IRON_BARS, x, y, z, CANOPY_BLOCKS, std::nullopt);
		editor.set_block_absolute(LIGHTNING_ROD, x, top, z, CANOPY_BLOCKS, std::nullopt);
		mast_head(editor, x, z, lattice_top - 2, 2, lighting);
		mast_cabinet(editor, x, z);
	}
}

void generate_chimney(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const CoordinateBitmap *footprints)
{
	const auto point = anchor(element);
	if (!point)
		return;
	const auto &[x, z] = *point;
	const int ground = editor.get_ground_level(x, z);
	const int base = roof_top(editor, footprints, x, z).value_or(ground);
	const auto mapped_metres = mast_metres(element.tags(), base > ground);
	if (!mapped_metres)
		return;
	const double metres = *mapped_metres;
	const int height = feature_height(metres, args.scale, ground, base, 2);
	Block body = BRICK;
	const auto material =
			lower_copy(element.tag("building:material")
							   .value_or(element.tag("material").value_or("")));
	if (material == "concrete" || material == "reinforced_concrete" ||
			material == "cement" || material == "metal" || material == "steel" ||
			material == "iron" || (material != "brick" && metres >= 80.0))
		body = LIGHT_GRAY_CONCRETE;
	if (height <= 6) {
		for (int y = base + 1; y <= base + height; ++y)
			editor.set_block_absolute(body, x, y, z, CANOPY_BLOCKS, std::nullopt);
		return;
	}
	const auto diameter = tag_metres(element, "diameter");
	double radius = diameter ? *diameter * args.scale / 2.0
							 : 1.0 + static_cast<double>(height) / 16.0;
	if (element.nodes_vec().size() >= 3) {
		int min_x = x, max_x = x, min_z = z, max_z = z;
		for (const auto &node : element.nodes_vec()) {
			min_x = std::min(min_x, node.x);
			max_x = std::max(max_x, node.x);
			min_z = std::min(min_z, node.z);
			max_z = std::max(max_z, node.z);
		}
		radius = std::min(max_x - min_x, max_z - min_z) / 2.0 - .5;
	}
	const bool outlined = element.nodes_vec().size() >= 3;
	std::set<std::pair<int, int>> outline_cells;
	if (outlined) {
		std::vector<std::pair<int, int>> polygon;
		polygon.reserve(element.nodes_vec().size());
		int min_x = std::numeric_limits<int>::max();
		int max_x = std::numeric_limits<int>::min();
		int min_z = std::numeric_limits<int>::max();
		int max_z = std::numeric_limits<int>::min();
		for (const auto &node : element.nodes_vec()) {
			polygon.emplace_back(node.x, node.z);
			min_x = std::min(min_x, node.x);
			max_x = std::max(max_x, node.x);
			min_z = std::min(min_z, node.z);
			max_z = std::max(max_z, node.z);
		}
		for (int cx = min_x; cx <= max_x; ++cx)
			for (int cz = min_z; cz <= max_z; ++cz)
				if (polygon_contains_cell_center(cx, cz, polygon))
					outline_cells.emplace(cx, cz);
	}
	radius = std::clamp(radius, 1.0, 12.0);
	const double top_radius = std::max(1.0, radius * .6);
	const bool banded = metres >= 100.0 && body != BRICK;
	const int band_height = std::max(1, height / 18);
	const auto on_outline = [&](int cx, int cz) {
		if (!outlined || outline_cells.empty())
			return true;
		for (const auto &[dx, dz] : {std::pair{0, 0}, std::pair{1, 0}, std::pair{-1, 0},
					 std::pair{0, 1}, std::pair{0, -1}})
			if (outline_cells.contains({cx + dx, cz + dz}))
				return true;
		return false;
	};
	for (int i = 0; i < height; ++i) {
		const int y = base + 1 + i;
		const double t = static_cast<double>(i) / height;
		const double r = radius + (top_radius - radius) * t;
		const int from_top = height - 1 - i;
		const Block &layer =
				from_top < 2 ? POLISHED_BLACKSTONE
				: banded && from_top < 2 + 3 * band_height
						? (((from_top - 2) / band_height) % 2 == 0 ? RED_CONCRETE
																   : WHITE_CONCRETE)
						: body;
		const int reach = static_cast<int>(std::ceil(r + .5));
		for (int dz = -reach; dz <= reach; ++dz)
			for (int dx = -reach; dx <= reach; ++dx) {
				const double d2 = static_cast<double>(dx * dx + dz * dz);
				if (d2 <= (r + .5) * (r + .5) && d2 > (r - .5) * (r - .5) &&
						on_outline(x + dx, z + dz))
					editor.set_block_absolute(
							layer, x + dx, y, z + dz, CANOPY_BLOCKS, std::nullopt);
			}
	}
}

void generate_water_well(WorldEditor &editor, const ProcessedElement &element)
{
	const auto point = anchor(element);
	if (!point)
		return;
	const auto &[x, z] = *point;
	const int base = editor.get_ground_level(x, z);
	if (const auto pump = element.tag("pump"); pump && *pump != "no") {
		for (int y = 1; y <= 2; ++y)
			editor.set_block_absolute(BLACKSTONE_WALL, x, base + y, z);
		editor.set_block_with_properties_absolute(
				BlockWithProperties{LEVER,
						{{"face", "floor"}, {"facing", "east"}, {"powered", "false"}}},
				x, base + 3, z, std::nullopt, std::nullopt);
		editor.set_block_with_properties_absolute(
				BlockWithProperties{WATER_CAULDRON, {{"level", "2"}}}, x + 1, base + 1, z,
				std::nullopt, std::nullopt);
		return;
	}

	// Shaft of water inside a two-high cobblestone ring.
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz) {
			if (dx == 0 && dz == 0) {
				for (int y = base - 2; y <= base; ++y)
					editor.set_block_absolute(
							WATER, x, y, z, std::nullopt, std::vector<Block>{});
			} else {
				editor.set_block_absolute(COBBLESTONE, x + dx, base, z + dz, std::nullopt,
						std::vector<Block>{});
				editor.set_block_absolute(COBBLESTONE, x + dx, base + 1, z + dz);
			}
		}
	}

	for (const int dx : {-1, 1})
		for (int y = base + 2; y <= base + 4; ++y)
			editor.set_block_absolute(OAK_FENCE, x + dx, y, z);
	editor.set_block_with_properties_absolute(
			BlockWithProperties{SPRUCE_LOG, {{"axis", "x"}}}, x, base + 5, z,
			std::nullopt, std::nullopt);
	for (const int dx : {-2, -1, 1, 2})
		editor.set_block_absolute(SPRUCE_PLANKS, x + dx, base + 5, z);
	for (int dx = -2; dx <= 2; ++dx) {
		for (const auto &[dz, facing] :
				{std::pair{-1, StairFacing::South}, std::pair{1, StairFacing::North}})
			editor.set_block_with_properties_absolute(
					create_stair_with_properties(
							SPRUCE_STAIRS, facing, StairShape::Straight),
					x + dx, base + 5, z + dz, std::nullopt, std::nullopt);
		editor.set_block_absolute(SPRUCE_SLAB, x + dx, base + 6, z);
	}
	for (int y = base + 3; y <= base + 4; ++y)
		editor.set_block_with_properties_absolute(
				BlockWithProperties{CHAIN_X, {{"axis", "y"}}}, x, y, z, std::nullopt,
				std::nullopt);
}

struct PairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		std::size_t h1 = std::hash<int>()(p.first);
		std::size_t h2 = std::hash<int>()(p.second);
		return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
	}
};

struct TankFootprint
{
	int center_x{0};
	int center_z{0};
	double radius{2.0};
	std::unordered_set<std::pair<int, int>, PairHash> cells;

	static TankFootprint from_element(const ProcessedElement &element)
	{
		std::vector<std::pair<int, int>> nodes;
		for (const auto &node : element.nodes_vec())
			nodes.emplace_back(node.x, node.z);
		if (nodes.empty()) {
			if (auto first = element.first_node(); first.has_value())
				nodes.emplace_back(first->x, first->z);
		}

		TankFootprint footprint;
		if (nodes.empty())
			return footprint;

		if (nodes.size() < 3) {
			footprint.center_x = nodes.front().first;
			footprint.center_z = nodes.front().second;
			footprint.radius = 2.5;
			for (int dx = -2; dx <= 2; ++dx) {
				for (int dz = -2; dz <= 2; ++dz)
					footprint.cells.insert(
							{footprint.center_x + dx, footprint.center_z + dz});
			}
			return footprint;
		}

		int min_x = std::numeric_limits<int>::max();
		int max_x = std::numeric_limits<int>::min();
		int min_z = std::numeric_limits<int>::max();
		int max_z = std::numeric_limits<int>::min();
		for (const auto &[x, z] : nodes) {
			min_x = std::min(min_x, x);
			max_x = std::max(max_x, x);
			min_z = std::min(min_z, z);
			max_z = std::max(max_z, z);
		}
		footprint.center_x = (min_x + max_x) / 2;
		footprint.center_z = (min_z + max_z) / 2;
		const double half_w = std::max(1, max_x - min_x) / 2.0;
		const double half_l = std::max(1, max_z - min_z) / 2.0;
		footprint.radius = (half_w + half_l) / 2.0;

		for (int x = min_x; x <= max_x; ++x) {
			for (int z = min_z; z <= max_z; ++z) {
				if (point_in_polygon(x, z, nodes))
					footprint.cells.insert({x, z});
			}
		}
		return footprint;
	}

	std::vector<std::pair<int, int>> cells_in_disc(double disc_radius) const
	{
		std::vector<std::pair<int, int>> out;
		const double r2 = disc_radius * disc_radius;
		const int r_int = static_cast<int>(std::ceil(disc_radius)) + 1;
		for (int dx = -r_int; dx <= r_int; ++dx) {
			for (int dz = -r_int; dz <= r_int; ++dz) {
				if (static_cast<double>(dx * dx + dz * dz) > r2)
					continue;
				std::pair<int, int> cell{center_x + dx, center_z + dz};
				if (cells.find(cell) != cells.end())
					out.push_back(cell);
			}
		}
		return out;
	}

private:
	static bool point_in_polygon(
			int px_i, int pz_i, const std::vector<std::pair<int, int>> &polygon)
	{
		const double px = static_cast<double>(px_i) + 0.5;
		const double pz = static_cast<double>(pz_i) + 0.5;
		bool inside = false;
		std::size_t j = polygon.size() - 1;
		for (std::size_t i = 0; i < polygon.size(); ++i) {
			const double xi = polygon[i].first;
			const double zi = polygon[i].second;
			const double xj = polygon[j].first;
			const double zj = polygon[j].second;
			const bool intersects = ((zi > pz) != (zj > pz)) &&
									(px < (xj - xi) * (pz - zi) / (zj - zi) + xi);
			if (intersects)
				inside = !inside;
			j = i;
		}
		return inside;
	}
};

std::string lower_copy(std::string value)
{
	std::transform(value.begin(), value.end(), value.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	return value;
}

std::string unicode_lower_copy(std::string_view value)
{
	return unicode_text::lowercase(value);
}

int read_height(const ProcessedElement &element, int default_height, double scale_factor)
{
	auto tag = element.tag("height");
	if (!tag.has_value())
		return std::max(default_height, 3);
	std::string_view value = *tag;
	while (!value.empty() && value.back() == 'm')
		value.remove_suffix(1);
	const auto [begin, end] = unicode_text::trim_whitespace(value);
	value = value.substr(begin, end - begin);
	const auto parsed = strict_parse::f64(value);
	if (!parsed)
		return std::max(default_height, 3);
	const double scaled = *parsed * scale_factor;
	int height = 0; // Rust's float-to-i32 cast maps NaN to zero.
	if (scaled >= std::numeric_limits<int>::max())
		height = std::numeric_limits<int>::max();
	else if (scaled <= std::numeric_limits<int>::min())
		height = std::numeric_limits<int>::min();
	else if (std::isfinite(scaled))
		height = static_cast<int>(std::round(scaled));
	return std::max(height, 3);
}

bool is_tank_value(const std::optional<std::string> &value)
{
	if (!value.has_value())
		return false;
	return value == "water_tower" || value == "silo" || value == "storage_tank";
}

Block tank_material_block(const ProcessedElement &element)
{
	std::optional<std::string> material = element.tag("building:material");
	if (!material.has_value())
		material = element.tag("material");
	if (!material.has_value())
		return SMOOTH_STONE;

	const std::string value = unicode_lower_copy(material.value());
	if (value == "metal" || value == "steel" || value == "aluminium" ||
			value == "aluminum" || value == "iron" || value == "tin")
		return IRON_BLOCK;
	if (value == "concrete" || value == "cement" || value == "reinforced_concrete")
		return GRAY_CONCRETE;
	return SMOOTH_STONE;
}

void generate_water_tower(
		WorldEditor &editor, const ProcessedElement &element, const Args &args)
{
	TankFootprint footprint = TankFootprint::from_element(element);
	if (footprint.cells.empty())
		return;

	const int total_height = read_height(element, 20, args.scale);
	const int support_height = static_cast<int>(std::round(total_height * 0.6));
	const int tank_height = total_height - support_height;
	if (tank_height < 2)
		return;

	const int leg_offset =
			std::max(1, static_cast<int>(std::round(footprint.radius * 0.85)));
	const std::vector<std::pair<int, int>> leg_positions = {
			{-leg_offset, 0}, {leg_offset, 0}, {0, -leg_offset}, {0, leg_offset}};
	for (const auto &[dx, dz] : leg_positions) {
		const int lx = footprint.center_x + dx;
		const int lz = footprint.center_z + dz;
		if (footprint.cells.find({lx, lz}) == footprint.cells.end())
			continue;
		for (int y = 0; y < support_height; ++y)
			editor.set_block(IRON_BLOCK, lx, y, lz, std::nullopt, std::nullopt);
	}

	const auto &nodes = element.nodes_vec();
	if (nodes.size() >= 2) {
		for (int tier_y = 5; tier_y < support_height; tier_y += 5) {
			for (std::size_t i = 1; i < nodes.size(); ++i) {
				auto points = bresenham_line(nodes[i - 1].x, tier_y, nodes[i - 1].z,
						nodes[i].x, tier_y, nodes[i].z);
				for (const auto &[bx, by, bz] : points)
					editor.set_block(
							SMOOTH_STONE, bx, by, bz, std::nullopt, std::nullopt);
			}
		}
	}

	for (int y = 0; y < support_height; ++y) {
		editor.set_block(POLISHED_ANDESITE, footprint.center_x, y, footprint.center_z,
				std::nullopt, std::nullopt);
	}

	const int tank_base =
			editor.get_ground_level(footprint.center_x, footprint.center_z) +
			support_height;
	for (int y = tank_base; y < tank_base + tank_height; ++y) {
		for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius))
			editor.set_block_absolute(
					POLISHED_ANDESITE, cx, y, cz, std::nullopt, std::nullopt);
	}
	for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius)) {
		editor.set_block_absolute(SMOOTH_STONE_SLAB, cx, tank_base + tank_height, cz,
				std::nullopt, std::nullopt);
	}
}

void generate_silo(WorldEditor &editor, const ProcessedElement &element, const Args &args)
{
	TankFootprint footprint = TankFootprint::from_element(element);
	if (footprint.cells.empty())
		return;

	const int height = read_height(element, 25, args.scale);
	const Block body_block = tank_material_block(element);
	const int base = editor.get_ground_level(footprint.center_x, footprint.center_z);
	for (int y = base; y < base + height; ++y) {
		for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius))
			editor.set_block_absolute(body_block, cx, y, cz, std::nullopt, std::nullopt);
	}
	for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius))
		editor.set_block_absolute(
				SMOOTH_STONE_SLAB, cx, base + height, cz, std::nullopt, std::nullopt);
}

void generate_storage_tank(
		WorldEditor &editor, const ProcessedElement &element, const Args &args)
{
	TankFootprint footprint = TankFootprint::from_element(element);
	if (footprint.cells.empty())
		return;

	const int default_height =
			std::max(static_cast<int>(std::round(footprint.radius * 1.2)), 6);
	const int max_height = static_cast<int>(footprint.radius * 1.5) + 4;
	const int height =
			std::min(read_height(element, default_height, args.scale), max_height);

	Block body_block = SMOOTH_STONE;
	if (auto content = element.tag("content"); content.has_value()) {
		const std::string value = unicode_lower_copy(content.value());
		if (value == "oil" || value == "fuel" || value == "diesel" ||
				value == "petroleum" || value == "tar")
			body_block = BLACK_TERRACOTTA;
		else if (value == "gas" || value == "lng" || value == "methane" || value == "lpg")
			body_block = WHITE_CONCRETE;
		else if (value == "water" || value == "wastewater")
			body_block = LIGHT_GRAY_CONCRETE;
	}

	const int base = editor.get_ground_level(footprint.center_x, footprint.center_z);
	for (int y = base; y < base + height; ++y) {
		for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius))
			editor.set_block_absolute(body_block, cx, y, cz, std::nullopt, std::nullopt);
	}
	for (const auto &[cx, cz] : footprint.cells_in_disc(footprint.radius))
		editor.set_block_absolute(
				SMOOTH_STONE_SLAB, cx, base + height, cz, std::nullopt, std::nullopt);
}

void generate_tank_structure(
		WorldEditor &editor, const ProcessedElement &element, const Args &args)
{
	if (!element.first_node().has_value() && element.nodes_vec().empty())
		return;

	std::optional<std::string> kind = element.tag("man_made");
	if (!is_tank_value(kind))
		kind = element.tag("building");

	if (kind == "water_tower")
		generate_water_tower(editor, element, args);
	else if (kind == "silo")
		generate_silo(editor, element, args);
	else if (kind == "storage_tank")
		generate_storage_tank(editor, element, args);
}

bool is_tank_structure(const ProcessedWay &way)
{
	auto man_made = way.tags.get("man_made");
	auto building = way.tags.get("building");
	return is_tank_value(man_made) || is_tank_value(building);
}

void generate_man_made(WorldEditor &editor, const ProcessedElement &element,
		const Args &args, const FloodFillCache &flood_fill_cache,
		const CoordinateBitmap &building_footprints)
{
	for (const char *key : {"layer", "level"}) {
		if (const auto value = element.tag(key);
				value && strict_parse::i32_or_zero(*value) < 0)
			return;
	}

	std::optional<std::string> man_made_type = element.tag("man_made");
	if (!man_made_type.has_value())
		return;
	const std::string &t = man_made_type.value();
	if (t == "pier") {
		waterfront::generate_pier(editor, element, flood_fill_cache, args);
	} else if (t == "breakwater" || t == "groyne") {
		waterfront::generate_breakwater(editor, element, flood_fill_cache, args);
	} else if (t == "antenna" || t == "mast") {
		generate_antenna(editor, element, args, &building_footprints);
	} else if (t == "chimney") {
		generate_chimney(editor, element, args, &building_footprints);
	} else if (t == "water_well") {
		generate_water_well(editor, element);
	} else if (t == "water_tower" || t == "silo" || t == "storage_tank") {
		generate_tank_structure(editor, element, args);
	} else if (t == "lighthouse") {
		if (element.is_way()) {
			const auto &nodes = element.as_way().nodes;
			// Rust's lighthouse way handler averages the complete node list,
			// including a repeated closing node on a footprint ring.
			const std::size_t count = nodes.size();
			if (count == 0)
				return;
			std::int64_t sx = 0;
			std::int64_t sz = 0;
			for (std::size_t i = 0; i < count; ++i) {
				sx += nodes[i].x;
				sz += nodes[i].z;
			}
			const auto n = static_cast<std::int64_t>(count);
			structures::lighthouse::place(
					editor, static_cast<int>(sx / n), static_cast<int>(sz / n));
		} else if (element.is_node()) {
			const auto &node = element.as_node();
			structures::lighthouse::place(editor, node.x, node.z);
		}
	} else if (t == "crane") {
		if (const auto point = anchor(element))
			structures::crane::place_mapped(editor, point->first, point->second,
					element.tags(), building_footprints);
	} else {
		// unknown type -> ignore
	}
}

void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints)
{
	auto it = node.tags.find("man_made");
	if (it == node.tags.end())
		return;

	ProcessedElement element(node);
	//element.single_node = node;
	//element.tags = node.tags;

	const std::string &t = it->second;
	if (t == "antenna" || t == "mast") {
		generate_antenna(editor, element, args, &building_footprints);
	} else if (t == "chimney") {
		generate_chimney(editor, element, args, &building_footprints);
	} else if (t == "water_well") {
		generate_water_well(editor, element);
	} else if (t == "water_tower" || t == "silo" || t == "storage_tank") {
		generate_tank_structure(editor, element, args);
	} else if (t == "lighthouse") {
		structures::lighthouse::place(editor, node.x, node.z);
	} else if (t == "dolphin") {
		waterfront::generate_dolphin(editor, node);
	} else if (t == "cross") {
		const bool summit =
				node.tags.get("summit:cross") == "yes" || node.tags.contains("natural");
		place_cross(editor, node, args, summit);
	} else if (t == "crane") {
		structures::crane::place_mapped(
				editor, node.x, node.z, node.tags, building_footprints);
	} else {
		// unknown -> ignore
	}
}

void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node)
{
	Args args;
	CoordinateBitmap no_footprints;
	generate_man_made_nodes(editor, node, args, no_footprints);
}

void generate_man_made_nodes(
		WorldEditor &editor, const ProcessedNode &node, const Args &args)
{
	CoordinateBitmap no_footprints;
	generate_man_made_nodes(editor, node, args, no_footprints);
}
}
}
