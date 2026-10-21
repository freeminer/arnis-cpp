#include "block_definitions.h"
#include "bresenham.h"
#include "connected_blocks.h"
#include "world_editor.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <optional>
#include <utility>
#include <algorithm>
#include <limits>

#include "../../../arnis_adapter.h"
#include "bridges.h"
namespace arnis
{

namespace barriers
{

const int BRIDGE_BARRIER_NEARBY_RADIUS = 2;

namespace
{
int rust_round_i32(double value)
{
	// Rust's `f64::round() as i32` rounds ties away from zero, converts NaN to
	// zero, and saturates infinities and out-of-range values. Keep malformed or
	// extreme OSM measurements from invoking undefined C++ float-to-int casts.
	if (std::isnan(value))
		return 0;
	if (value >= static_cast<double>(std::numeric_limits<int>::max()))
		return std::numeric_limits<int>::max();
	if (value <= static_cast<double>(std::numeric_limits<int>::min()))
		return std::numeric_limits<int>::min();
	return static_cast<int>(std::round(value));
}

std::optional<double> parse_min_height(const std::string &raw)
{
	std::string value = raw;
	const auto last = value.find_last_not_of(" \t\r\n");
	if (last == std::string::npos)
		return std::nullopt;
	value.erase(last + 1);
	while (!value.empty() && value.back() == 'm')
		value.pop_back();
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos)
		return std::nullopt;
	value.erase(0, first);
	try {
		std::size_t used = 0;
		const double height = std::stod(value, &used);
		if (used != value.size())
			return std::nullopt;
		return height;
	} catch (...) {
		return std::nullopt;
	}
}

std::vector<std::optional<int>> barrier_bases(world_editor::WorldEditor &editor,
		const bridges::BridgeSurfaceMap &surface,
		const std::vector<std::pair<int, int>> &cells)
{
	const auto ground = [&editor](const std::pair<int, int> &cell) {
		return editor.get_absolute_y(cell.first, 0, cell.second);
	};
	const auto deck = [&surface](const std::pair<int, int> &cell) {
		return surface.nearby_deck_y(
				cell.first, cell.second, BRIDGE_BARRIER_NEARBY_RADIUS);
	};
	std::vector<std::optional<int>> bases;
	bases.reserve(cells.size());
	for (const auto &cell : cells)
		bases.emplace_back(ground(cell));

	std::size_t i = 0;
	while (i < cells.size()) {
		if (!deck(cells[i])) {
			++i;
			continue;
		}
		const auto start = i;
		while (i < cells.size() && deck(cells[i]))
			++i;
		const auto at_grade = [&](std::size_t inside, std::size_t outside) {
			const auto deck_y = deck(cells[inside]);
			return deck_y && *deck_y - ground(cells[outside]) <= 2;
		};
		const bool on_bridge = (start == 0 && i == cells.size()) ||
							   (start > 0 && at_grade(start, start - 1)) ||
							   (i < cells.size() && at_grade(i - 1, i));
		if (!on_bridge)
			continue;
		for (std::size_t n = start; n < i; ++n) {
			const auto [x, z] = cells[n];
			bases[n] = surface.supported_top(editor, x, z, BRIDGE_BARRIER_NEARBY_RADIUS);
			if (!bases[n]) {
				const int y = ground(cells[n]);
				const auto deck_y = deck(cells[n]);
				if (deck_y && *deck_y - y <= 1)
					bases[n] = y;
			}
		}
	}
	return bases;
}
} // namespace

void generate_barriers(world_editor::WorldEditor &editor,
		osm_parser::ProcessedElement const &element,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	block_definitions::Block barrier_material = block_definitions::COBBLESTONE_WALL;
	int barrier_height = 2;

	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto it = tags.find("barrier");
		if (it != tags.end()) {
			const std::string &val = it->second;
			if (val == "bollard") {
				barrier_material = block_definitions::STREETS_BOLLARD;
				barrier_height = 1;
			} else if (val == "guard_rail" || val == "guardrail") {
				barrier_material = block_definitions::STREETS_GUARDRAIL;
				barrier_height = 1;
			} else if (val == "kerb") {
				return;
			} else if (val == "hedge") {
				barrier_material = block_definitions::OAK_LEAVES;
				barrier_height = 2;
			} else if (val == "fence") {
				auto fit = tags.find("fence_type");
				if (fit != tags.end()) {
					const std::string &f = fit->second;
					if (f == "railing" || f == "bars" || f == "krest") {
						barrier_material = block_definitions::EARTH_FENCE_WROUGHT;
						barrier_height = 1;
					} else if (f == "barbed_wire" || f == "electric") {
						barrier_material = block_definitions::EARTH_FENCE_BARBED;
						barrier_height = 2;
					} else if (f == "chain_link" || f == "metal" || f == "wire" ||
							   f == "corrugated_metal" || f == "electric" ||
							   f == "metal_bars") {
						barrier_material = block_definitions::EARTH_FENCE_CHAINLINK;
						barrier_height = 2;
					} else if (f == "slatted" || f == "paling") {
						barrier_material = block_definitions::EARTH_FENCE_PICKET;
						barrier_height = 1;
					} else if (f == "wood" || f == "split_rail" || f == "panel" ||
							   f == "pole") {
						barrier_material = block_definitions::OAK_FENCE;
						barrier_height = 2;
					} else if (f == "concrete" || f == "stone") {
						barrier_material = block_definitions::STONE_BRICK_WALL;
						barrier_height = 2;
					} else if (f == "glass") {
						barrier_material = block_definitions::GLASS;
						barrier_height = 1;
					}
				}
			} else if (val == "wall") {
				barrier_material = block_definitions::STONE_BRICK_WALL;
				barrier_height = 3;
			}
		}
	}

	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto mit = tags.find("material");
		if (mit != tags.end()) {
			const std::string &mat = mit->second;
			if (mat == "brick") {
				barrier_material = block_definitions::BRICK;
			}
			if (mat == "concrete") {
				barrier_material = block_definitions::LIGHT_GRAY_CONCRETE;
			}
			if (mat == "metal") {
				barrier_material = block_definitions::STONE_BRICK_WALL;
			}
			if (mat == "wood") {
				barrier_material = block_definitions::OAK_FENCE;
			}
		}
	}

	std::optional<osm_parser::Way> maybe_way = element.as_way();
	if (!maybe_way.has_value()) {
		return;
	}
	const osm_parser::Way &way = maybe_way.value();

	// Mapped barrier parts can be suspended above the ground (for example,
	// railings attached to a bridge tower). Scale both their lift and tagged
	// height with the world, but keep ordinary ground barriers at their legacy
	// one-block-per-metre height.
	const double scale = editor.scale();
	int lift = 0;
	if (const auto it = element.tags().find("min_height"); it != element.tags().end()) {
		if (const auto metres = parse_min_height(it->second)) {
			const double rounded = std::round(*metres * scale);
			lift = rust_round_i32(std::isnan(rounded) ? 0.0 : std::max(0.0, rounded));
		}
	}
	const double height_scale = lift > 0 ? scale : 1.0;
	int wall_height = barrier_height;
	{
		const std::unordered_map<std::string, std::string> &tags = element.tags();
		auto hit = tags.find("height");
		if (hit != tags.end()) {
			try {
				std::size_t used = 0;
				const double h = std::stod(hit->second, &used);
				if (used == hit->second.size()) {
					const auto height =
							static_cast<std::int64_t>(rust_round_i32(h * height_scale));
					wall_height = static_cast<int>(std::clamp<std::int64_t>(height - lift,
							std::numeric_limits<int>::min(),
							std::numeric_limits<int>::max()));
				}
			} catch (...) {
				wall_height = barrier_height;
			}
		}
		const int minimum_height = lift > 0 ? 1 : 2;
		if (wall_height < minimum_height) {
			wall_height = minimum_height;
		}
	}
	const bool joined = connected_blocks::is_connectable(editor, barrier_material);
	const bool capped = wall_height > 1 &&
						barrier_material != block_definitions::OAK_LEAVES &&
						barrier_material != block_definitions::OAK_FENCE;
	const bool hedge = barrier_material == block_definitions::OAK_LEAVES;
	std::vector<std::pair<int, int>> cells;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		for (const auto &point : bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			const std::pair<int, int> cell{std::get<0>(point), std::get<2>(point)};
			if (cells.empty() || cells.back() != cell)
				cells.push_back(cell);
		}
	}
	if (joined)
		cells = connected_blocks::four_connected_line(cells);
	auto bases = barrier_bases(editor, bridge_surface, cells);
	if (lift > 0 && !cells.empty()) {
		int lowest = std::numeric_limits<int>::max();
		for (const auto &[x, z] : cells)
			lowest = std::min(lowest, editor.get_absolute_y(x, 0, z));
		bases.assign(cells.size(), std::optional<int>(lowest));
	}
	for (std::size_t i = 0; i < cells.size(); ++i) {
		if (!bases[i])
			continue;
		const auto [x, z] = cells[i];
		const int base = *bases[i];
		if (hedge && !editor.surface_is_sealed(x, z) &&
				base == editor.get_absolute_y(x, 0, z))
			editor.set_block_absolute(block_definitions::GRASS_BLOCK, x, base, z,
					std::nullopt, std::nullopt);
		for (int y = lift + 1; y <= lift + wall_height; ++y) {
			if (joined)
				connected_blocks::place_connected(
						editor, barrier_material, x, base + y, z);
			else
				editor.set_block_absolute(
						barrier_material, x, base + y, z, std::nullopt, std::nullopt);
		}
		if (capped)
			editor.set_block_absolute(block_definitions::STONE_BRICK_SLAB, x,
					base + lift + wall_height + 1, z, std::nullopt, std::nullopt);
	}
}

void generate_barriers(
		world_editor::WorldEditor &editor, osm_parser::ProcessedElement const &element)
{
	static const bridges::BridgeSurfaceMap empty_bridge_surface;
	generate_barriers(editor, element, empty_bridge_surface);
}

void place_barrier_node_block(world_editor::WorldEditor &editor,
		osm_parser::ProcessedNode const &node,
		const bridges::BridgeSurfaceMap &bridge_surface, block_definitions::Block block,
		int dy)
{
	const auto deck_y = bridge_surface.contains(node.x, node.z)
								? bridge_surface.supported_top(editor, node.x, node.z,
										  BRIDGE_BARRIER_NEARBY_RADIUS)
								: std::nullopt;
	if (deck_y) {
		editor.set_block_absolute(block, node.x, *deck_y + dy, node.z,
				std::optional<std::vector<block_definitions::Block>>(),
				std::optional<std::vector<block_definitions::Block>>());
	} else {
		editor.set_block(block, node.x, dy, node.z,
				std::optional<std::vector<block_definitions::Block>>(), std::nullopt);
	}
}

void generate_barrier_nodes(world_editor::WorldEditor &editor,
		osm_parser::ProcessedNode const &node,
		const bridges::BridgeSurfaceMap &bridge_surface)
{
	auto it = node.tags.find("barrier");
	if (it == node.tags.end()) {
		return;
	}
	const std::string &val = it->second;
	if (val == "bollard") {
		place_barrier_node_block(
				editor, node, bridge_surface, block_definitions::COBBLESTONE_WALL, 1);
	} else if (val == "stile" || val == "gate" || val == "swing_gate" ||
			   val == "lift_gate") {
		/* Future implementation for gates:
        editor.set_block(
            block_definitions::OAK_TRAPDOOR,
            node.x,
            1,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        editor.set_block(
            block_definitions::AIR,
            node.x,
            2,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        editor.set_block(
            block_definitions::AIR,
            node.x,
            3,
            node.z,
            std::optional<std::vector<block_definitions::Block>>(
                std::vector<block_definitions::Block>{
                    block_definitions::COBBLESTONE_WALL,
                    block_definitions::OAK_FENCE,
                    block_definitions::STONE_BRICK_WALL,
                    block_definitions::OAK_LEAVES,
                    block_definitions::STONE_BRICK_SLAB
                }
            ),
            std::nullopt
        );
        */
	} else if (val == "block") {
		place_barrier_node_block(
				editor, node, bridge_surface, block_definitions::STONE, 1);
	} else if (val == "entrance") {
		place_barrier_node_block(editor, node, bridge_surface, block_definitions::AIR, 1);
	}
}

void generate_barrier_nodes(
		world_editor::WorldEditor &editor, osm_parser::ProcessedNode const &node)
{
	static const bridges::BridgeSurfaceMap empty_bridge_surface;
	generate_barrier_nodes(editor, node, empty_bridge_surface);
}

}
}
