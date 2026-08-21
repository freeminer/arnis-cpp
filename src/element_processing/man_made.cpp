#include <string>
#include <map>
#include <optional>
#include <tuple>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <unordered_set>
#include <vector>

#include "../bresenham.h"
#include "../structures/structures.h"
#include "../../../arnis_adapter.h"
namespace arnis
{

namespace man_made
{


std::optional<int> parse_int(const std::optional<std::string> &s)
{
	if (!s.has_value())
		return std::optional<int>();
	try {
		return std::optional<int>(std::stoi(s.value()));
	} catch (...) {
		return std::optional<int>();
	}
}


void generate_pier(WorldEditor &editor, const ProcessedElement &element)
{
	const std::vector<ProcessedNode> &nodes = element.nodes_vec();
	if (nodes.size() < 2)
		return;

	int pier_width = 3;
	{
		std::optional<int> w = parse_int(element.tag("width"));
		if (w.has_value())
			pier_width = w.value();
	}

	int pier_height = 1;
	int support_spacing = 4;

	for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
		const ProcessedNode &start_node = nodes[i];
		const ProcessedNode &end_node = nodes[i + 1];
		std::vector<std::tuple<int, int, int>> line_points =
				bresenham_line(start_node.x, 0, start_node.z, end_node.x, 0, end_node.z);
		for (std::size_t index = 0; index < line_points.size(); ++index) {
			int center_x = std::get<0>(line_points[index]);
			int center_z = std::get<2>(line_points[index]);

			int half_width = pier_width / 2;
			for (int x = center_x - half_width; x <= center_x + half_width; ++x) {
				for (int z = center_z - half_width; z <= center_z + half_width; ++z) {
					editor.set_block(OAK_SLAB, x, pier_height, z,
							std::optional<std::vector<Block>>(), std::optional<int>());
				}
			}

			if ((index % support_spacing) == 0) {
				int hw = pier_width / 2;
				std::vector<std::pair<int, int>> support_positions = {
						{center_x - hw, center_z}, {center_x + hw, center_z}};
				for (const auto &p : support_positions) {
					int pillar_x = p.first;
					int pillar_z = p.second;
					editor.set_block(OAK_LOG, pillar_x, 0, pillar_z,
							std::optional<std::vector<Block>>(), std::optional<int>());
				}
			}
		}
	}
}

void generate_antenna(WorldEditor &editor, const ProcessedElement &element)
{
	std::optional<ProcessedNode> maybe_node = element.first_node();
	if (!maybe_node.has_value())
		return;
	ProcessedNode node = maybe_node.value();
	int x = node.x;
	int z = node.z;

	int height = 20;
	std::optional<int> htag = parse_int(element.tag("height"));
	if (htag.has_value()) {
		height = std::min(htag.value(), 40);
	} else {
		std::optional<std::string> tower_type = element.tag("tower:type");
		if (tower_type.has_value()) {
			if (tower_type.value() == "communication")
				height = 20;
			else if (tower_type.value() == "cellular")
				height = 15;
			else
				height = 20;
		}
	}

	editor.set_block(IRON_BLOCK, x, 3, z, std::optional<std::vector<Block>>(),
			std::optional<int>());
	for (int y = 4; y < height; ++y) {
		editor.set_block(IRON_BARS, x, y, z, std::optional<std::vector<Block>>(),
				std::optional<int>());
	}

	for (int y = 7; y < height; y += 7) {
		editor.set_block(IRON_BLOCK, x, y, z,
				std::optional<std::vector<Block>>(std::vector<Block>{IRON_BARS}),
				std::optional<int>());
		std::vector<std::pair<int, int>> support_positions = {
				{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
		for (const auto &d : support_positions) {
			editor.set_block(IRON_BLOCK, x + d.first, y, z + d.second,
					std::optional<std::vector<Block>>(), std::optional<int>());
		}
	}

	editor.fill_blocks(GRAY_CONCRETE, x - 1, 1, z - 1, x + 1, 2, z + 1);
}

void generate_chimney(WorldEditor &editor, const ProcessedElement &element)
{
	std::optional<ProcessedNode> maybe_node = element.first_node();
	if (!maybe_node.has_value())
		return;
	ProcessedNode node = maybe_node.value();
	int x = node.x;
	int z = node.z;
	int height = 25;

	for (int y = 0; y < height; ++y) {
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dz = -1; dz <= 1; ++dz) {
				if (dx == 0 && dz == 0)
					continue;
				editor.set_block(BRICK, x + dx, y, z + dz,
						std::optional<std::vector<Block>>(), std::optional<int>());
			}
		}
	}
}

void generate_water_well(WorldEditor &editor, const ProcessedElement &element)
{
	std::optional<ProcessedNode> maybe_node = element.first_node();
	if (!maybe_node.has_value())
		return;
	ProcessedNode node = maybe_node.value();
	int x = node.x;
	int z = node.z;
	if (EARTH_WELL.id() != STONE_BRICKS.id()) {
		editor.set_block(EARTH_WELL, x, 1, z, std::nullopt, std::nullopt);
		return;
	}

	for (int dx = -1; dx <= 1; ++dx) {
		for (int dz = -1; dz <= 1; ++dz) {
			if (dx == 0 && dz == 0) {
				editor.set_block(WATER, x, -1, z, std::optional<std::vector<Block>>(),
						std::optional<int>());
				editor.set_block(WATER, x, 0, z, std::optional<std::vector<Block>>(),
						std::optional<int>());
			} else {
				editor.set_block(STONE_BRICKS, x + dx, 0, z + dz,
						std::optional<std::vector<Block>>(), std::optional<int>());
				editor.set_block(STONE_BRICKS, x + dx, 1, z + dz,
						std::optional<std::vector<Block>>(), std::optional<int>());
			}
		}
	}

	editor.fill_blocks(OAK_LOG, x - 2, 1, z, x - 2, 4, z,
			std::optional<std::vector<Block>>(), std::optional<int>());
	editor.fill_blocks(OAK_LOG, x + 2, 1, z, x + 2, 4, z,
			std::optional<std::vector<Block>>(), std::optional<int>());

	editor.set_block(OAK_SLAB, x - 1, 5, z, std::optional<std::vector<Block>>(),
			std::optional<int>());
	editor.set_block(OAK_FENCE, x, 4, z, std::optional<std::vector<Block>>(),
			std::optional<int>());
	editor.set_block(
			OAK_SLAB, x, 5, z, std::optional<std::vector<Block>>(), std::optional<int>());
	editor.set_block(OAK_SLAB, x + 1, 5, z, std::optional<std::vector<Block>>(),
			std::optional<int>());

	editor.set_block(IRON_BLOCK, x, 3, z, std::optional<std::vector<Block>>(),
			std::optional<int>());
}

void generate_manhole(WorldEditor &editor, const ProcessedElement &element)
{
	if (const auto node = element.first_node())
		editor.set_block(EARTH_GRATING, node->x, 0, node->z,
				std::nullopt, std::nullopt);
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
		if (footprint.cells.empty()) {
			footprint.cells.insert({footprint.center_x, footprint.center_z});
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

int read_height(const ProcessedElement &element, int default_height, double scale_factor)
{
	auto tag = element.tag("height");
	if (!tag.has_value())
		return std::max(default_height, 3);
	std::string value = tag.value();
	value.erase(std::remove_if(value.begin(), value.end(),
						[](unsigned char c) { return std::isspace(c); }),
			value.end());
	if (!value.empty() && (value.back() == 'm' || value.back() == 'M'))
		value.pop_back();
	try {
		return std::max(static_cast<int>(std::round(std::stod(value) * scale_factor)), 3);
	} catch (...) {
		return std::max(default_height, 3);
	}
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

	const std::string value = lower_copy(material.value());
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
		const std::string value = lower_copy(content.value());
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

void generate_man_made(
		WorldEditor &editor, const ProcessedElement &element, const Args &args)
{
	if (element.tag("layer").has_value()) {
		std::optional<int> layer = parse_int(element.tag("layer"));
		if (layer.has_value() && layer.value() < 0)
			return;
	}
	if (element.tag("level").has_value()) {
		std::optional<int> level = parse_int(element.tag("level"));
		if (level.has_value() && level.value() < 0)
			return;
	}

	std::optional<std::string> man_made_type = element.tag("man_made");
	if (!man_made_type.has_value())
		return;
	const std::string &t = man_made_type.value();
	if (t == "pier") {
		generate_pier(editor, element);
	} else if (t == "antenna" || t == "mast") {
		generate_antenna(editor, element);
	} else if (t == "chimney") {
		generate_chimney(editor, element);
	} else if (t == "water_well") {
		generate_water_well(editor, element);
	} else if (t == "manhole") {
		generate_manhole(editor, element);
	} else if (t == "water_tower" || t == "silo" || t == "storage_tank") {
		generate_tank_structure(editor, element, args);
	} else if (t == "lighthouse") {
		if (element.is_way() && !element.as_way().nodes.empty()) {
			long long sx = 0;
			long long sz = 0;
			for (const auto &node : element.as_way().nodes) {
				sx += node.x;
				sz += node.z;
			}
			const auto n = static_cast<long long>(element.as_way().nodes.size());
			structures::lighthouse::place(
					editor, static_cast<int>(sx / n), static_cast<int>(sz / n));
		} else if (auto node = element.first_node(); node.has_value()) {
			structures::lighthouse::place(editor, node->x, node->z);
		}
	} else {
		// unknown type -> ignore
	}
}

void generate_man_made_nodes(
		WorldEditor &editor, const ProcessedNode &node, const Args &args)
{
	auto it = node.tags.find("man_made");
	if (it == node.tags.end())
		return;

	ProcessedElement element(node);
	//element.single_node = node;
	//element.tags = node.tags;

	const std::string &t = it->second;
	if (t == "antenna" || t == "mast") {
		generate_antenna(editor, element);
	} else if (t == "chimney") {
		generate_chimney(editor, element);
	} else if (t == "water_well") {
		generate_water_well(editor, element);
	} else if (t == "manhole") {
		generate_manhole(editor, element);
	} else if (t == "water_tower" || t == "silo" || t == "storage_tank") {
		generate_tank_structure(editor, element, args);
	} else if (t == "lighthouse") {
		structures::lighthouse::place(editor, node.x, node.z);
	} else {
		// unknown -> ignore
	}
}

void generate_man_made_nodes(WorldEditor &editor, const ProcessedNode &node)
{
	Args args;
	generate_man_made_nodes(editor, node, args);
}
}
}
