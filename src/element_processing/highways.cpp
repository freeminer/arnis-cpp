#include <vector>
#include <unordered_map>
#include <string>
#include <optional>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstddef>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "bridge_styles.h"
#include "bridges.h"
#include "surfaces.h"
#include "highway_tunnels.h"
#include "signage.h"
#include "../floodfill.h"
#include "../structures/helicopter.h"
namespace arnis
{

namespace highways
{


// Hash for pair<int,int> used in unordered_map
struct PairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
									  static_cast<unsigned long long>(p.second));
	}
};

// Minimum terrain dip (in blocks) below max endpoint elevation to classify a bridge as valley-spanning
const int VALLEY_BRIDGE_THRESHOLD = 7;

std::vector<crate::block_definitions::Block> default_road_mix()
{
	return {crate::block_definitions::ROAD_ASPHALT};
}

std::vector<crate::block_definitions::Block> road_protected_surfaces()
{
	std::vector<crate::block_definitions::Block> result{
			crate::block_definitions::BLACK_CONCRETE,
			crate::block_definitions::ROAD_ASPHALT,
			crate::block_definitions::ROAD_SIDEWALK,
			crate::block_definitions::GRAY_CONCRETE_POWDER,
			crate::block_definitions::CYAN_TERRACOTTA,
			crate::block_definitions::WHITE_CONCRETE,
			crate::block_definitions::WARPED_PLANKS,
			crate::block_definitions::ANDESITE_WALL,
			crate::block_definitions::SEA_LANTERN,
			crate::block_definitions::SMOOTH_SANDSTONE_STAIRS};
	if (crate::block_definitions::STREETS_MARKINGS_AVAILABLE) {
		result.insert(result.end(), {
				crate::block_definitions::ROAD_MARK_DASHED_WHITE,
				crate::block_definitions::ROAD_MARK_DASHED_WHITE_R90,
				crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE,
				crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE_R90});
	}
	return result;
}

#define ROAD_PROTECTED_SURFACES road_protected_surfaces()

int perpendicular_median_raw(const crate::world_editor::WorldEditor &editor, int set_x,
		int set_z, int centerline_x, int centerline_z, int block_range,
		bool dir_horizontal)
{
	std::vector<int> ys;
	ys.reserve(static_cast<std::size_t>(block_range * 2 + 1));
	for (int t = -block_range; t <= block_range; ++t)
		ys.push_back(dir_horizontal ? editor.get_ground_level(set_x, centerline_z + t)
								: editor.get_ground_level(centerline_x + t, set_z));
	std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
	return ys[ys.size() / 2];
}

int perpendicular_median_ground_y(const crate::world_editor::WorldEditor &editor,
		int set_x, int set_z, int centerline_x, int centerline_z, int block_range,
		bool dir_horizontal)
{
	const int prev = perpendicular_median_raw(editor,
			dir_horizontal ? set_x - 1 : set_x, dir_horizontal ? set_z : set_z - 1,
			centerline_x, centerline_z, block_range, dir_horizontal);
	const int current = perpendicular_median_raw(editor, set_x, set_z, centerline_x,
			centerline_z, block_range, dir_horizontal);
	const int next = perpendicular_median_raw(editor,
			dir_horizontal ? set_x + 1 : set_x, dir_horizontal ? set_z : set_z + 1,
			centerline_x, centerline_z, block_range, dir_horizontal);
	return std::max(std::min(prev, current), std::min(std::max(prev, current), next));
}

bool is_pedestrian_way_tags(const crate::tags_t &tags)
{
	const auto highway = tags.get("highway");
	if (highway == "footway" || highway == "pedestrian" || highway == "steps")
		return true;
	const auto footway = tags.get("footway");
	return !footway.empty() && footway != "no";
}

int node_feature_base_y(const crate::world_editor::WorldEditor &editor,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const std::unordered_map<std::string, std::string> &tags,
		int x, int z)
{
	int layer = 0;
	if (tags.find("indoor") == tags.end() || tags.at("indoor") != "yes") {
	if (const auto it = tags.find("layer"); it != tags.end()) {
			try {
				layer = std::max(0, std::stoi(it->second));
			} catch (...) {
			}
		}
	}
	return bridge_surface.nearby_deck_y(x, z, 0).value_or(
			editor.get_ground_level(x, z) + layer * 6);
}

void place_way_lamps(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &way, int block_range,
		const crate::RoadMaskBitmap &road_mask)
{
	constexpr std::size_t interval = 25;
	std::size_t tds = 0;
	int side = 1;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const int dx = b.x - a.x, dz = b.z - a.z;
		const float length = std::sqrt(static_cast<float>(dx * dx + dz * dz));
		if (length == 0.0f)
			continue;
		const int px = static_cast<int>(std::round(-dz / length));
		const int pz = static_cast<int>(std::round(dx / length));
		for (const auto &[x, y, z] : crate::bresenham::bresenham_line(a.x, 0, a.z, b.x, 0,
					 b.z)) {
			(void)y;
			if (tds > 0 && tds % interval == 0) {
				for (const int candidate_side : {side, -side}) {
					const int lx = x + px * (block_range + 2) * candidate_side;
					const int lz = z + pz * (block_range + 2) * candidate_side;
					if (road_mask.contains(lx, lz) || editor.is_lc_water(lx, lz))
						continue;
					editor.set_block(crate::block_definitions::COBBLESTONE_WALL, lx, 1, lz,
								std::nullopt, std::nullopt);
					for (int dy = 2; dy <= 4; ++dy)
					editor.set_block(crate::block_definitions::STREETS_POLE, lx, dy, lz,
									std::nullopt, std::nullopt);
					editor.set_block(crate::block_definitions::EARTH_STREET_LAMP, lx, 5,
							lz, std::nullopt, std::nullopt);
					side = -side;
					break;
				}
			}
			++tds;
		}
	}
}

std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash>
build_highway_connectivity_map(
		const std::vector<crate::osm_parser::ProcessedElement> &elements)
{
	std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash> connectivity_map;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const crate::osm_parser::ProcessedWay &way = element.as_way();
			auto it_highway = way.tags.find("highway");
			if (it_highway != way.tags.end()) {
				int layer_value = 0;
				auto it_layer = way.tags.find("layer");
				if (it_layer != way.tags.end()) {
					try {
						layer_value = std::stoi(it_layer->second);
					} catch (...) {
						layer_value = 0;
					}
				}
				// Treat negative layers as ground level (0) for connectivity
				if (layer_value < 0) {
					layer_value = 0;
				}
				if (!way.nodes.empty()) {
					const crate::osm_parser::ProcessedNode &start_node =
							way.nodes.front();
					const crate::osm_parser::ProcessedNode &end_node = way.nodes.back();
					std::pair<int, int> start_coord = {start_node.x, start_node.z};
					std::pair<int, int> end_coord = {end_node.x, end_node.z};
					connectivity_map[start_coord].push_back(layer_value);
					connectivity_map[end_coord].push_back(layer_value);
				}
			}
		}
	}
	return connectivity_map;
}

void add_highway_support_pillar(crate::world_editor::WorldEditor &editor, int x,
		int highway_y, int z, int dx, int dz, int /*_block_range*/)
{
	using crate::block_definitions::STONE_BRICKS;
	if (dx == 0 && dz == 0 && ((x + z) % 8) == 0) {
		for (int y = 1; y < highway_y; ++y) {
			editor.set_block(STONE_BRICKS, x, y, z,
					std::optional<std::vector<crate::block_definitions::Block>>(),
					std::optional<std::vector<crate::block_definitions::Block>>());
		}
		for (int base_dx = -1; base_dx <= 1; ++base_dx) {
			for (int base_dz = -1; base_dz <= 1; ++base_dz) {
				editor.set_block(STONE_BRICKS, x + base_dx, 0, z + base_dz,
						std::optional<std::vector<crate::block_definitions::Block>>(),
						std::optional<std::vector<crate::block_definitions::Block>>());
			}
		}
	}
}

// Add support pillars for bridges using absolute Y coordinates
// Pillars extend from ground level up to the bridge deck
void add_highway_support_pillar_absolute(crate::world_editor::WorldEditor &editor, int x,
		int bridge_deck_y, int z, int dx, int dz, int /*_block_range*/)
{
	using crate::block_definitions::STONE_BRICKS;
	if (dx == 0 && dz == 0 && ((x + z) % 8) == 0) {
		// Get the actual ground level at this position
		int ground_y = 0;
		auto *ground = editor.get_ground();
		if (ground) {
			ground_y = ground->level(crate::coordinate_system::cartesian::XZPoint(x, z));
		}

		// Add pillar from ground up to bridge deck
		// Only if the bridge is actually above the ground
		if (bridge_deck_y > ground_y) {
			for (int y = ground_y + 1; y < bridge_deck_y; ++y) {
				editor.set_block_absolute(STONE_BRICKS, x, y, z,
						std::optional<std::vector<crate::block_definitions::Block>>(),
						std::optional<std::vector<crate::block_definitions::Block>>());
			}

			// Add pillar base at ground level
			for (int base_dx = -1; base_dx <= 1; ++base_dx) {
				for (int base_dz = -1; base_dz <= 1; ++base_dz) {
					editor.set_block_absolute(STONE_BRICKS, x + base_dx, ground_y,
							z + base_dz,
							std::optional<std::vector<crate::block_definitions::Block>>(),
							std::optional<
									std::vector<crate::block_definitions::Block>>());
				}
			}
		}
	}
}

bool should_add_slope_at_node(const crate::osm_parser::ProcessedNode &node,
		int current_layer,
		const std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash>
				&highway_connectivity)
{
	std::pair<int, int> node_coord = {node.x, node.z};
	if (highway_connectivity.empty()) {
		return current_layer != 0;
	}
	auto it = highway_connectivity.find(node_coord);
	if (it != highway_connectivity.end()) {
		const std::vector<int> &connected_layers = it->second;
		std::size_t same_layer_count = 0;
		for (int layer : connected_layers) {
			if (layer == current_layer) {
				++same_layer_count;
			}
		}
		if (same_layer_count <= 1) {
			return current_layer != 0;
		}
		return false;
	} else {
		return current_layer != 0;
	}
}

std::size_t calculate_way_length(const crate::osm_parser::ProcessedWay &way)
{
	std::size_t total_length = 0;
	const crate::osm_parser::ProcessedNode *prev = nullptr;
	for (const auto &node : way.nodes) {
		if (prev != nullptr) {
			int dx = (node.x - prev->x);
			int dz = (node.z - prev->z);
			double seg = std::sqrt(static_cast<double>(dx * dx + dz * dz));
			total_length += static_cast<std::size_t>(seg);
		}
		prev = &node;
	}
	return total_length;
}

std::size_t calculate_total_bresenham_length(const crate::osm_parser::ProcessedWay &way)
{
	if (way.nodes.size() < 2) {
		return 0;
	}
	std::size_t total_length = 1;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &prev = way.nodes[i - 1];
		const auto &cur = way.nodes[i];
		total_length += static_cast<std::size_t>(
				std::max(std::abs(cur.x - prev.x), std::abs(cur.z - prev.z)));
	}
	return total_length;
}

std::vector<std::pair<int, int>> stair_fill_cells(
		std::pair<int, int> prev, std::pair<int, int> curr)
{
	std::vector<std::pair<int, int>> cells;
	int x = prev.first;
	int z = prev.second;
	while (x != curr.first || z != curr.second) {
		if (x != curr.first) {
			x += (curr.first - x) > 0 ? 1 : -1;
			cells.emplace_back(x, z);
		}
		if (z != curr.second) {
			z += (curr.second - z) > 0 ? 1 : -1;
			cells.emplace_back(x, z);
		}
	}
	if (cells.empty()) {
		cells.push_back(curr);
	}
	return cells;
}

int highway_block_range(const std::string &highway_type,
		const std::unordered_map<std::string, std::string> &tags, double scale)
{
	int block_range = 2;
	if (highway_type == "footway" || highway_type == "pedestrian" ||
			highway_type == "path" || highway_type == "track" ||
			highway_type == "secondary_link" || highway_type == "tertiary_link" ||
			highway_type == "escape" || highway_type == "steps") {
		block_range = 1;
	} else if (highway_type == "motorway" || highway_type == "primary" ||
			   highway_type == "trunk") {
		block_range = 5;
	} else if (highway_type == "secondary") {
		block_range = 4;
	} else if (highway_type == "tertiary") {
		block_range = 2;
	} else if (highway_type == "service") {
		block_range = 2;
	} else {
		auto it_lanes = tags.find("lanes");
		if (it_lanes != tags.end()) {
			if (it_lanes->second == "2")
				block_range = 3;
			else if (it_lanes->second != "1")
				block_range = 4;
		}
	}
	const auto default_lanes = (highway_type == "motorway" ||
				highway_type == "primary" || highway_type == "trunk" ||
				highway_type == "secondary" || highway_type == "tertiary")
					 ? 2
					 : 1;
	int lanes = default_lanes;
	if (const auto it = tags.find("lanes"); it != tags.end()) {
		try {
			lanes = std::clamp(std::stoi(it->second), 1, 16);
		} catch (...) {
		}
	}
	// An explicitly mapped physical width takes precedence. Otherwise retain the
	// historic type width as a minimum and enlarge multi-lane carriageways.
	if (const auto it = tags.find("width"); it != tags.end()) {
		try {
			std::string width = it->second;
			while (!width.empty() && std::isspace(static_cast<unsigned char>(width.back())))
				width.pop_back();
			if (!width.empty() && width.back() == 'm')
				width.pop_back();
			block_range = static_cast<int>(std::round(std::stod(width) / 2.0));
		} catch (...) {
		}
	} else if (highway_type != "footway" && highway_type != "pedestrian" &&
			highway_type != "path" && highway_type != "track" &&
			highway_type != "escape" && highway_type != "steps") {
		block_range = std::max(block_range,
				static_cast<int>(std::round((lanes * 3.5 - 1.0) / 2.0)));
	}
	block_range = std::clamp(block_range, 1, 8);
	if (scale < 1.0)
		block_range = std::max(1, static_cast<int>(std::floor(block_range * scale)));
	return block_range;
}

int calculate_point_elevation(std::size_t segment_index, std::size_t point_index,
		std::size_t segment_length, std::size_t total_segments, int base_elevation,
		bool needs_start_slope, bool needs_end_slope, std::size_t slope_length)
{
	if (!needs_start_slope && !needs_end_slope) {
		return base_elevation;
	}
	std::size_t total_distance_from_start = segment_index * segment_length + point_index;
	std::size_t total_way_length = total_segments * segment_length;
	if (total_way_length == 0 || slope_length == 0) {
		return base_elevation;
	}
	if (needs_start_slope && total_distance_from_start <= slope_length) {
		float slope_progress = static_cast<float>(total_distance_from_start) /
							   static_cast<float>(slope_length);
		int elevation_offset =
				static_cast<int>(static_cast<float>(base_elevation) * slope_progress);
		return elevation_offset;
	}
	if (needs_end_slope &&
			total_distance_from_start >= (total_way_length > slope_length
														 ? total_way_length - slope_length
														 : 0)) {
		std::size_t distance_from_end =
				(total_way_length > total_distance_from_start)
						? (total_way_length - total_distance_from_start)
						: 0;
		float slope_progress =
				static_cast<float>(distance_from_end) / static_cast<float>(slope_length);
		int elevation_offset =
				static_cast<int>(static_cast<float>(base_elevation) * slope_progress);
		return elevation_offset;
	}
	return base_elevation;
}

void generate_highways_internal(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const std::unordered_map<std::pair<int, int>, std::vector<int>, PairHash>
				&highway_connectivity,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const crate::RoadMaskBitmap &road_mask,
		const crate::bridges::BridgeStructureMap &bridge_structures,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals)
{
	using crate::block_definitions::Block;
	using crate::block_definitions::COBBLESTONE_WALL;
	using crate::block_definitions::GLOWSTONE;
	using crate::block_definitions::GREEN_WOOL;
	using crate::block_definitions::OAK_FENCE;
	using crate::block_definitions::RED_WOOL;
	using crate::block_definitions::WHITE_WOOL;
	using crate::block_definitions::YELLOW_WOOL;
	(void)COBBLESTONE_WALL;
	(void)OAK_FENCE;
	(void)GLOWSTONE;
	(void)GREEN_WOOL;
	(void)YELLOW_WOOL;
	(void)RED_WOOL;
	(void)WHITE_WOOL;

	auto it_highway = element.tags().find("highway");
	if (it_highway == element.tags().end()) {
		return;
	}
	const std::string &highway_type = it_highway->second;

	// Check if this is a bridge - bridges need special elevation handling
	// to span across valleys instead of following terrain
	// Accept any bridge tag value except "no" (e.g., "yes", "viaduct", "aqueduct", etc.)
	bool is_bridge = false;
	auto it_bridge = element.tags().find("bridge");
	if (it_bridge != element.tags().end() && it_bridge->second != "no") {
		is_bridge = true;
	}

	if (highway_type == "street_lamp") {
		if (element.is_node()) {
			int x = element.as_node().x;
			int z = element.as_node().z;
			const int base = node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
			editor.set_block_absolute(crate::block_definitions::COBBLESTONE_WALL, x, base + 1, z,
					std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
			for (int dy = 2; dy <= 4; ++dy) {
				editor.set_block_absolute(crate::block_definitions::STREETS_POLE, x, base + dy, z,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
			editor.set_block_absolute(crate::block_definitions::EARTH_STREET_LAMP, x, base + 5, z,
					std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
		}
		return;
	} else if (highway_type == "crossing" || highway_type == "traffic_signals") {
		auto it_crossing = element.tags().find("crossing");
		if (highway_type == "traffic_signals" ||
				(it_crossing != element.tags().end() &&
						it_crossing->second == "traffic_signals")) {
			if (element.is_node()) {
				int x = element.as_node().x;
				int z = element.as_node().z;
				const int base = node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
				auto signal_rng = coord_rng(x, z, 0x54524146464943ULL);
				auto light = crate::block_definitions::STREETS_TRAFFIC_LIGHTS[
						signal_rng.uniform(crate::block_definitions::STREETS_TRAFFIC_LIGHTS.size())];
				if (light.id() != CONTENT_AIR) {
					// Streets traffic-light nodeboxes are deliberately outside their
					// node and attach to a pole in the neighbouring cell.  Keep the
					// support at the OSM point, extend it to head height, and place the
					// head one cell in front of it so the mounting bracket meets it.
					std::uint8_t facing = static_cast<std::uint8_t>(signal_rng.uniform(4));
					constexpr std::array<std::pair<int, int>, 4> head_offsets{{
							{0, -1}, {-1, 0}, {0, 1}, {1, 0}}};
					auto [head_dx, head_dz] = head_offsets[facing];
					if (!editor.owns(x + head_dx, z + head_dz)) {
						facing = static_cast<std::uint8_t>((facing + 2) % 4);
						std::tie(head_dx, head_dz) = head_offsets[facing];
					}
					for (int dy = 1; dy <= 4; ++dy) {
						auto pole = crate::block_definitions::STREETS_POLE;
						pole.setParam2(facing);
						editor.set_block_absolute(pole, x, base + dy, z,
								std::optional<std::vector<Block>>(),
								std::optional<std::vector<Block>>());
					}
					light.setParam2(facing);
					editor.set_block_absolute(light, x + head_dx, base + 4, z + head_dz,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				} else {
					for (int dy = 1; dy <= 3; ++dy) {
						editor.set_block_absolute(crate::block_definitions::STREETS_POLE,
								x, base + dy, z,
								std::optional<std::vector<Block>>(),
								std::optional<std::vector<Block>>());
					}
					editor.set_block_absolute(GREEN_WOOL, x, base + 4, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block_absolute(YELLOW_WOOL, x, base + 5, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
					editor.set_block_absolute(RED_WOOL, x, base + 6, z,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				}
			}
		}
		return;
	} else if (highway_type == "bus_stop") {
		if (element.is_node()) {
			int x = element.as_node().x;
			int z = element.as_node().z;
			const int base = node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
			for (int dy = 1; dy <= 3; ++dy) {
				editor.set_block_absolute(crate::block_definitions::COBBLESTONE_WALL,
						x, base + dy, z,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
			editor.set_block_absolute(crate::block_definitions::WHITE_WOOL, x, base + 4, z,
					std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
			const int neighbour_base = node_feature_base_y(editor, bridge_surface, element.tags(),
					x + 1, z);
				editor.set_block_absolute(crate::block_definitions::WHITE_WOOL, x + 1,
						neighbour_base + 4, z,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
				signage::place_bus_stop_signs(editor, element.as_node().tags,
						x, neighbour_base + 4, z);
			}
		return;
	} else {
		auto it_area = element.tags().find("area");
		if (it_area != element.tags().end() && it_area->second == "yes") {
			if (!element.is_way()) {
				return;
			}
			const crate::osm_parser::ProcessedWay &way = element.as_way();
			auto surface_blocks = crate::surfaces::get_blocks_for_surface_way(
					way, std::vector<crate::block_definitions::Block>{
								 crate::block_definitions::STONE});
			std::vector<std::pair<int, int>> polygon_coords;
			polygon_coords.reserve(way.nodes.size());
			for (const auto &n : way.nodes) {
				polygon_coords.emplace_back(n.x, n.z);
			}
			std::vector<std::pair<int, int>> filled_area =
					crate::floodfill::flood_fill_area(polygon_coords, floodfill_timeout);
			for (const auto &p : filled_area) {
				const auto surface_block = crate::surfaces::semirandom_surface(
						p.first, p.second, surface_blocks);
				editor.set_block(surface_block, p.first, 0, p.second,
						std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
			return;
		}
	}

	// Main highway/walkway processing below
	crate::block_definitions::Block block_type = crate::block_definitions::BLACK_CONCRETE;
	int block_range = 2;
	bool add_stripe = false;
	bool add_outline = false;
	double scale_factor = args.scale;

	int layer_value = 0;
	auto it_layer = element.tags().find("layer");
	if (it_layer != element.tags().end()) {
		try {
			layer_value = std::stoi(it_layer->second);
		} catch (...) {
			layer_value = 0;
		}
	}
	// Treat negative layers as ground level (0)
	if (layer_value < 0) {
		layer_value = 0;
	}
	if (element.tags().find("indoor") != element.tags().end() &&
			element.tags().at("indoor") == "yes")
		layer_value = 0;

	auto it_level = element.tags().find("level");
	if (it_level != element.tags().end()) {
		try {
			int level_val = std::stoi(it_level->second);
			if (level_val < 0)
				return;
		} catch (...) {
			// ignore parse errors
		}
	}

	if (highway_type == "footway" || highway_type == "pedestrian") {
		block_type = crate::block_definitions::ROAD_SIDEWALK;
		block_range = 1;
	} else if (highway_type == "path") {
		block_type = crate::block_definitions::DIRT_PATH;
		block_range = 1;
	} else if (highway_type == "motorway" || highway_type == "primary" ||
			   highway_type == "trunk") {
		block_range = 5;
		add_stripe = true;
	} else if (highway_type == "secondary") {
		block_range = 4;
		add_stripe = true;
	} else if (highway_type == "tertiary") {
		add_stripe = true;
	} else if (highway_type == "track") {
		block_range = 1;
	} else if (highway_type == "service") {
		block_type = crate::block_definitions::ROAD_ASPHALT;
		block_range = 2;
	} else if (highway_type == "secondary_link" || highway_type == "tertiary_link") {
		// Exit ramps, sliproads
		block_type = crate::block_definitions::BLACK_CONCRETE;
		block_range = 1;
	} else if (highway_type == "escape") {
		// Sand trap for vehicles on mountainous roads
		block_type = crate::block_definitions::SAND;
		block_range = 1;
	} else if (highway_type == "steps") {
		// Steps use slab-like paving; elevation interpolation below preserves the
		// tagged ascent while keeping the one-block pedestrian width.
		block_type = crate::block_definitions::STONE_BLOCK_SLAB;
		block_range = 1;
	} else {
		auto it_lanes = element.tags().find("lanes");
		if (it_lanes != element.tags().end()) {
			const std::string &lanes = it_lanes->second;
			if (lanes == "2") {
				block_range = 3;
				add_stripe = true;
				add_outline = true;
			} else if (lanes != "1") {
				block_range = 4;
				add_stripe = true;
				add_outline = true;
			}
		}
	}

	if (!element.is_way()) {
		return;
	}
	const crate::osm_parser::ProcessedWay &way = element.as_way();
	// Keep the renderer's width in sync with the road-mask prescan and the
	// current Rust implementation (lanes=* and width=* both affect it).
	block_range = highway_block_range(highway_type, way.tags, scale_factor);
	int lanes = 1;
	if (highway_type == "motorway" || highway_type == "primary" ||
			highway_type == "trunk" || highway_type == "secondary" ||
			highway_type == "tertiary")
		lanes = 2;
	if (const auto it = way.tags.find("lanes"); it != way.tags.end()) {
		try {
			lanes = std::clamp(std::stoi(it->second), 1, 16);
		} catch (...) {
		}
	}
	if (way.tags.get("lane_markings") == "no")
		lanes = 1;
	const auto *bridge_member = bridge_structures.lookup_member(way.id);
	const auto *bridge_ramp = bridge_structures.lookup_ramp(way.id);
	const bool is_bridge_member = bridge_member != nullptr;
	const bool is_bridge_ramp = bridge_ramp != nullptr;
	const auto tunnel_approach = (is_bridge_member || is_bridge_ramp)
									 ? std::nullopt
									 : tunnel_portals.approach(way.id);
	if (way.tags.get("lit") == "yes" && !is_bridge_member && !is_bridge_ramp &&
				layer_value == 0 && highway_type != "steps")
		place_way_lamps(editor, way, block_range, road_mask);

	std::vector<crate::block_definitions::Block> default_surface = default_road_mix();
	if (highway_type == "footway" || highway_type == "pedestrian" ||
			highway_type == "steps") {
		default_surface = {crate::block_definitions::ROAD_SIDEWALK};
	} else if (highway_type == "path" || highway_type == "track") {
		default_surface = {crate::block_definitions::DIRT_PATH};
	} else if (highway_type == "escape") {
		default_surface = {crate::block_definitions::SAND};
	} else if (block_type != crate::block_definitions::BLACK_CONCRETE) {
		default_surface = {block_type};
	}
	auto block_types = crate::surfaces::get_blocks_for_surface_way(way, default_surface);
	const auto surface_tag = way.tags.get("surface");
	if (is_pedestrian_way_tags(way.tags) &&
			(surface_tag == "concrete" || surface_tag == "paving_stones" ||
					surface_tag == "sett"))
		block_types = {crate::block_definitions::ROAD_SIDEWALK};

	const int LAYER_HEIGHT_STEP = 6;
	int base_elevation = layer_value * LAYER_HEIGHT_STEP;

	bool needs_start_slope = false;
	bool needs_end_slope = false;
	if (!way.nodes.empty()) {
		needs_start_slope = should_add_slope_at_node(
				way.nodes.front(), layer_value, highway_connectivity);
		needs_end_slope = should_add_slope_at_node(
				way.nodes.back(), layer_value, highway_connectivity);
	}

	std::size_t total_way_length = calculate_way_length(way);

	// For bridges: detect if this spans a valley by checking terrain profile
	// A valley bridge has terrain that dips significantly below the endpoints
	// Skip valley detection entirely if terrain is disabled (no valleys in flat terrain)
	// Skip very short bridges (< 25 blocks) as they're unlikely to span significant valleys
	bool terrain_enabled = false;
	auto *ground = editor.get_ground();
	if (ground) {
		terrain_enabled = true; // Assuming elevation is always enabled in this context
	}

	bool is_valley_bridge = false;
	int bridge_deck_y = 0;
	if (is_bridge && terrain_enabled && way.nodes.size() >= 2 && total_way_length >= 25) {
		const crate::osm_parser::ProcessedNode &start_node = way.nodes.front();
		const crate::osm_parser::ProcessedNode &end_node = way.nodes.back();
		// Get ground reference from editor
		auto *ground = editor.get_ground();
		int start_y = 0;
		int end_y = 0;
		if (ground) {
			start_y = ground->level(crate::coordinate_system::cartesian::XZPoint(
					start_node.x, start_node.z));
			end_y = ground->level(
					crate::coordinate_system::cartesian::XZPoint(end_node.x, end_node.z));
		}
		int max_endpoint_y = std::max(start_y, end_y);

		// Sample terrain at middle nodes only (excluding endpoints we already have)
		// This avoids redundant get_ground_level() calls
		const std::vector<crate::osm_parser::ProcessedNode> middle_nodes(
				way.nodes.begin() + 1, way.nodes.end() - 1);
		int sampled_min = max_endpoint_y;
		if (!middle_nodes.empty()) {
			// Sample up to 3 middle points (5 total with endpoints) for performance
			// Valleys are wide terrain features, so sparse sampling is sufficient
			std::size_t sample_count =
					std::min(static_cast<std::size_t>(3), middle_nodes.size());
			std::size_t step = (sample_count > 1)
									   ? (middle_nodes.size() - 1) / (sample_count - 1)
									   : 1;

			for (std::size_t i = 0; i < middle_nodes.size();
					i += std::max(static_cast<std::size_t>(1), step)) {
				const auto &node = middle_nodes[i];
				int node_y = 0;
				if (ground) {
					node_y = ground->level(
							crate::coordinate_system::cartesian::XZPoint(node.x, node.z));
				}
				sampled_min = std::min(sampled_min, node_y);
			}
		}

		// Include endpoint elevations in the minimum calculation
		int min_terrain_y = std::min({sampled_min, start_y, end_y});

		// If ANY sampled point along the bridge is significantly lower than the max endpoint,
		// treat as valley bridge
		is_valley_bridge = (min_terrain_y < max_endpoint_y - VALLEY_BRIDGE_THRESHOLD);

		if (is_valley_bridge) {
			bridge_deck_y = max_endpoint_y;
		}
	}

	const std::size_t total_bresenham_length = calculate_total_bresenham_length(way);
	const std::size_t raw_bridge_ramp_length = static_cast<std::size_t>(
			std::clamp(static_cast<float>(total_bresenham_length) * 0.35f, 15.0f, 50.0f));
	const std::size_t bridge_internal_ramp_length =
			std::clamp<std::size_t>(raw_bridge_ramp_length, 1,
					std::max<std::size_t>(1, total_bresenham_length / 2));

	// Check if this is a short isolated elevated segment (layer > 0), if so, treat as ground level
	bool is_short_isolated_elevated =
			(!is_bridge_member && !is_bridge_ramp && needs_start_slope &&
					needs_end_slope && layer_value > 0 && total_way_length <= 35);

	// Override elevation and slopes for short isolated segments
	int effective_elevation = 0;
	bool effective_start_slope = false;
	bool effective_end_slope = false;
	if (is_bridge_member || is_bridge_ramp || is_short_isolated_elevated) {
		effective_elevation = 0;
		effective_start_slope = false;
		effective_end_slope = false;
	} else {
		effective_elevation = base_elevation;
		effective_start_slope = needs_start_slope;
		effective_end_slope = needs_end_slope;
	}

	std::size_t slope_length = static_cast<std::size_t>(
			std::clamp(static_cast<double>(total_way_length) * 0.35, 15.0, 50.0));

	std::optional<std::pair<int, int>> previous_node;
	std::size_t segment_index = 0;
	std::size_t total_segments = (way.nodes.size() > 0) ? (way.nodes.size() - 1) : 0;
	std::size_t cumulative_distance_from_start = 0;
	std::optional<int> previous_bridge_y;
	std::optional<std::pair<int, int>> previous_rail_left;
	std::optional<std::pair<int, int>> previous_rail_right;
	const auto bridge_style = bridge_member ? bridge_member->style
											: crate::bridge_styles::BridgeStyle::Beam;
	std::vector<crate::bridge_styles::BridgePathSample> bridge_path;

	for (const auto &node : way.nodes) {
		if (previous_node.has_value()) {
			int x1 = previous_node->first;
			int z1 = previous_node->second;
			int x2 = node.x;
			int z2 = node.z;
			const bool dir_horizontal = std::abs(x2 - x1) >= std::abs(z2 - z1);
			const bool flatten_width =
					!is_bridge_member && !is_bridge_ramp && block_range >= 1;
			std::vector<std::tuple<int, int, int>> bresenham_points =
					crate::bresenham::bresenham_line(x1, 0, z1, x2, 0, z2);
			std::size_t segment_length = bresenham_points.size();

			int stripe_length_counter = 0;
			int dash_length = static_cast<int>(std::ceil(5.0 * scale_factor));
			int gap_length = static_cast<int>(std::ceil(5.0 * scale_factor));
			const float dx_segment = static_cast<float>(x2 - x1);
			const float dz_segment = static_cast<float>(z2 - z1);
			const float segment_norm =
					std::sqrt(dx_segment * dx_segment + dz_segment * dz_segment);
			const float perp_x = segment_norm > 0.0f ? -dz_segment / segment_norm : 0.0f;
			const float perp_z = segment_norm > 0.0f ? dx_segment / segment_norm : 0.0f;
			const float lane_width =
					lanes > 0 ? static_cast<float>(2 * block_range + 1) / lanes : 0.0f;
			const bool bridge_like = is_bridge_member || is_bridge_ramp;
			const std::size_t skip_first = bridge_like && segment_index > 0 ? 1 : 0;
			std::optional<std::pair<float, float>> bridge_rail_perp;
			if (bridge_like) {
				const float dx_seg = static_cast<float>(x2 - x1);
				const float dz_seg = static_cast<float>(z2 - z1);
				const float seg_len = std::sqrt(dx_seg * dx_seg + dz_seg * dz_seg);
				if (seg_len > 0.0f) {
					bridge_rail_perp =
							std::make_pair(-dz_seg / seg_len, dx_seg / seg_len);
				}
			}

			for (std::size_t point_index = skip_first;
					point_index < bresenham_points.size(); ++point_index) {
				int x = std::get<0>(bresenham_points[point_index]);
				int z = std::get<2>(bresenham_points[point_index]);
				const std::size_t tds = cumulative_distance_from_start + point_index;

				// Calculate Y elevation for this point
				// Bridge structures drive absolute Y for full bridge groups and their ramps.
				int current_y;
				bool use_absolute_y = false;
				if (bridge_member) {
					current_y = bridge_member->y_at(
							tds, total_bresenham_length, bridge_internal_ramp_length);
					use_absolute_y = true;
				} else if (bridge_ramp) {
					current_y = bridge_ramp->y_at(tds, total_bresenham_length);
					use_absolute_y = true;
				} else if (is_valley_bridge) {
					// Valley bridge deck is level at the maximum endpoint elevation
					// Don't add base_elevation - the layer tag indicates it's above water/road,
					// not that it should be higher than the terrain endpoints
					current_y = bridge_deck_y;
					use_absolute_y = true;
				} else {
					// Regular road or overpass: use terrain-relative calculation with ramps
					current_y = calculate_point_elevation(segment_index, point_index,
							segment_length, total_segments, effective_elevation,
							effective_start_slope, effective_end_slope, slope_length);
					use_absolute_y = false;
				}
				const int tunnel_approach_offset =
						(!use_absolute_y && tunnel_approach)
								? tunnel_approach->offset(static_cast<int>(tds))
								: 0;
				current_y += tunnel_approach_offset;
				if (auto deck_y = bridge_surface.deck_y_at(x, z)) {
					current_y = *deck_y;
					use_absolute_y = true;
				}
				if (is_bridge_member && bridge_rail_perp.has_value()) {
					bridge_path.push_back(
							crate::bridge_styles::BridgePathSample{x, current_y, z,
									bridge_rail_perp->first, bridge_rail_perp->second});
				}

				if (bridge_like) {
					if (previous_bridge_y.has_value()) {
						int fill_lo = 0;
						int fill_hi = -1;
						if (current_y >= *previous_bridge_y + 3) {
							fill_lo = *previous_bridge_y + 1;
							fill_hi = current_y - 2;
						} else if (current_y <= *previous_bridge_y - 3) {
							fill_lo = current_y + 1;
							fill_hi = *previous_bridge_y - 2;
						}
						if (fill_lo <= fill_hi) {
							for (int fill_y = fill_lo; fill_y <= fill_hi; ++fill_y) {
								for (int fdx = -block_range; fdx <= block_range; ++fdx) {
									for (int fdz = -block_range; fdz <= block_range;
											++fdz) {
										editor.set_block_absolute(
												crate::block_definitions::STONE_BRICKS,
												x + fdx, fill_y, z + fdz,
												std::optional<std::vector<
														crate::block_definitions::
																Block>>(),
												std::optional<std::vector<
														crate::block_definitions::Block>>(
														ROAD_PROTECTED_SURFACES));
									}
								}
							}
						}
					}
					previous_bridge_y = current_y;
				}

				// The perpendicular median depends on only one coordinate: X for a
				// horizontal segment and Z for a vertical segment. The old inner loop
				// recomputed the same median once for every cell on the other axis
				// (2r+1 times), including three allocations/nth_element calls each time.
				std::vector<int> flattened_ground_profile;
				if (flatten_width && !use_absolute_y) {
					flattened_ground_profile.reserve(
							static_cast<std::size_t>(block_range * 2 + 1));
					for (int offset = -block_range; offset <= block_range; ++offset) {
						const int sample_x = dir_horizontal ? x + offset : x;
						const int sample_z = dir_horizontal ? z : z + offset;
						flattened_ground_profile.push_back(perpendicular_median_ground_y(
								editor, sample_x, sample_z, x, z, block_range,
								dir_horizontal));
					}
				}
				auto flattened_ground_y_at = [&](int sample_x, int sample_z) {
					const int offset = dir_horizontal ? sample_x - x : sample_z - z;
					if (!flattened_ground_profile.empty() && offset >= -block_range &&
							offset <= block_range)
						return flattened_ground_profile[static_cast<std::size_t>(
								offset + block_range)];
					return perpendicular_median_ground_y(editor, sample_x, sample_z, x, z,
							block_range, dir_horizontal);
				};

				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						int set_x = x + dx;
						int set_z = z + dz;
						const bool surface_absolute = use_absolute_y || flatten_width;
						const int surface_y = surface_absolute
								? (use_absolute_y ? current_y
													  : flattened_ground_y_at(set_x, set_z) +
															current_y)
									: current_y;
						if (flatten_width && current_y == 0 && tunnel_approach_offset == 0)
							editor.register_road_surface_y(set_x, set_z, surface_y);

						bool zebra = false;
						bool crossing_horizontal = false;
						if (is_pedestrian_way_tags(way.tags) &&
								way.tags.get("footway") == "crossing" &&
								way.tags.get("crossing") != "no" &&
								way.tags.get("crossing") != "unmarked" &&
								way.tags.get("crossing:markings") != "no") {
								crossing_horizontal =
										(std::abs(x2 - x1) >= std::abs(z2 - z1));
								if (crossing_horizontal) {
									if ((set_x % 2 + 2) % 2 < 1)
										zebra = true;
								} else {
									if ((set_z % 2 + 2) % 2 < 1)
										zebra = true;
								}
						}

						if (zebra) {
							const bool native_marking =
									crate::block_definitions::STREETS_MARKINGS_AVAILABLE;
							const auto zebra_block = native_marking
									? (crossing_horizontal
											  ? crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE
											  : crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE_R90)
									: crate::block_definitions::WHITE_CONCRETE;
							const std::vector<crate::block_definitions::Block> zebra_hosts{
									crate::block_definitions::BLACK_CONCRETE,
									crate::block_definitions::ROAD_ASPHALT};
							if (surface_absolute) {
								editor.set_block_absolute(
										zebra_block, set_x,
										surface_y, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												zebra_hosts),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							} else {
								editor.set_block(zebra_block,
										set_x, surface_y, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												zebra_hosts),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
						} else {
							const auto road_block = crate::surfaces::semirandom_surface(
									set_x, set_z, block_types);
							if (surface_absolute) {
								editor.set_block_absolute(road_block, set_x, surface_y,
										set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												ROAD_PROTECTED_SURFACES));
							} else {
								editor.set_block(road_block, set_x, surface_y, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												ROAD_PROTECTED_SURFACES));
							}
						}

						// Add stone brick foundation underneath elevated highways/bridges for thickness
						if (((effective_elevation > 0 || use_absolute_y) &&
									current_y > 0) ||
								tunnel_approach_offset < 0) {
							// Add 1 layer of stone bricks underneath the highway surface
							const auto foundation_block =
									is_bridge_member
											? crate::bridge_styles::foundation_block(
													  bridge_style)
											: crate::block_definitions::STONE_BRICKS;
							if (use_absolute_y) {
								editor.set_block_absolute(foundation_block, set_x,
										current_y - 1, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							} else {
								editor.set_block(foundation_block, set_x, current_y - 1,
										set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
						}

						// Add support pillars for elevated highways/bridges
						if ((effective_elevation != 0 || use_absolute_y) &&
								current_y > 0) {
							if (is_bridge_member) {
								const std::size_t interval = std::max<std::size_t>(
										1, crate::bridge_styles::pillar_interval(
												   bridge_style));
								const bool is_centerline = dx == 0 && dz == 0;
								const bool is_pillar_position =
										is_centerline && tds % interval == 0;
								const int centerline_ground_y =
										editor.get_ground_level(x, z);
								crate::bridge_styles::place_bridge_support_below_deck(
										editor, bridge_style, set_x, current_y, set_z,
										centerline_ground_y, tds, total_bresenham_length,
										use_absolute_y, is_centerline,
										is_pillar_position);
							} else if (use_absolute_y) {
								add_highway_support_pillar_absolute(editor, set_x,
										current_y, set_z, dx, dz, block_range);
							} else {
								add_highway_support_pillar(editor, set_x, current_y,
										set_z, dx, dz, block_range);
							}
						}
					}
				}

				if (bridge_like && bridge_rail_perp.has_value() &&
						!(is_bridge_member &&
								!crate::bridge_styles::has_side_railing(bridge_style))) {
					const float perp_x = bridge_rail_perp->first;
					const float perp_z = bridge_rail_perp->second;
					const auto rail_block =
							is_bridge_member
									? crate::bridge_styles::rail_block(bridge_style)
									: crate::block_definitions::LIGHT_GRAY_CONCRETE;
					const auto rail_foundation =
							is_bridge_member
									? crate::bridge_styles::rail_foundation_block(
											  bridge_style)
									: crate::block_definitions::STONE_BRICKS;
					const auto parapet =
							is_bridge_member
									? crate::bridge_styles::parapet_block(bridge_style)
									: std::optional<crate::block_definitions::Block>{
											  crate::block_definitions::BRICK_WALL};
					const float rail_dist =
							static_cast<float>(block_range) *
									(std::abs(perp_x) + std::abs(perp_z)) +
							1.0f;
					for (int side = 0; side < 2; ++side) {
						const float sign = side == 0 ? 1.0f : -1.0f;
						auto &previous_rail =
								side == 0 ? previous_rail_left : previous_rail_right;
						const int rail_x = static_cast<int>(std::round(
								static_cast<float>(x) + perp_x * rail_dist * sign));
						const int rail_z = static_cast<int>(std::round(
								static_cast<float>(z) + perp_z * rail_dist * sign));
						const std::pair<int, int> rail_cell{rail_x, rail_z};
						const auto cells_to_fill =
								previous_rail.has_value()
										? stair_fill_cells(*previous_rail, rail_cell)
										: std::vector<std::pair<int, int>>{rail_cell};
						for (const auto &rail_fill : cells_to_fill) {
							const int rx = rail_fill.first;
							const int rz = rail_fill.second;
							if (bridge_surface.contains(rx, rz)) {
								continue;
							}
							editor.set_block_absolute(rail_block, rx, current_y, rz,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									std::optional<
											std::vector<crate::block_definitions::Block>>(
											ROAD_PROTECTED_SURFACES));
							if (current_y > 0) {
								editor.set_block_absolute(rail_foundation, rx,
										current_y - 1, rz,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
							if (parapet) {
								editor.set_block_absolute(*parapet, rx, current_y + 1, rz,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
						}
						previous_rail = rail_cell;
					}
				}

				if (add_outline) {
					// Left outline
					for (int dz = -block_range; dz <= block_range; ++dz) {
						int outline_x = x - block_range - 1;
						int outline_z = z + dz;
						if (use_absolute_y) {
							editor.set_block_absolute(
									crate::block_definitions::LIGHT_GRAY_CONCRETE,
									outline_x, current_y, outline_z,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									std::optional<std::vector<
											crate::block_definitions::Block>>());
						} else {
							editor.set_block(
									crate::block_definitions::LIGHT_GRAY_CONCRETE,
									outline_x, current_y, outline_z,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									std::optional<std::vector<
											crate::block_definitions::Block>>());
						}
					}
					// Right outline
					for (int dz = -block_range; dz <= block_range; ++dz) {
						int outline_x = x + block_range + 1;
						int outline_z = z + dz;
						if (use_absolute_y) {
							editor.set_block_absolute(
									crate::block_definitions::LIGHT_GRAY_CONCRETE,
									outline_x, current_y, outline_z,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									std::optional<std::vector<
											crate::block_definitions::Block>>());
						} else {
							editor.set_block(
									crate::block_definitions::LIGHT_GRAY_CONCRETE,
									outline_x, current_y, outline_z,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									std::optional<std::vector<
											crate::block_definitions::Block>>());
						}
					}
				}

				if (add_stripe && segment_norm > 0.0f) {
					if (stripe_length_counter < dash_length) {
						for (int lane = 1; lane < lanes; ++lane) {
							const float distance = lane * lane_width -
									static_cast<float>(2 * block_range + 1) / 2.0f;
							const int stripe_x = static_cast<int>(std::round(x + perp_x * distance));
							const int stripe_z = static_cast<int>(std::round(z + perp_z * distance));
							const bool stripe_absolute = use_absolute_y || flatten_width;
							const int stripe_y = stripe_absolute
									? (use_absolute_y ? current_y
													  : flattened_ground_y_at(stripe_x, stripe_z) +
															current_y)
									: current_y;
							const bool native_marking =
									crate::block_definitions::STREETS_MARKINGS_AVAILABLE &&
									std::find(block_types.begin(), block_types.end(),
											crate::block_definitions::ROAD_ASPHALT) != block_types.end();
							const auto stripe_block = native_marking
									? (dir_horizontal
											  ? crate::block_definitions::ROAD_MARK_DASHED_WHITE_R90
											  : crate::block_definitions::ROAD_MARK_DASHED_WHITE)
									: crate::block_definitions::WHITE_CONCRETE;
							const auto stripe_hosts = native_marking
									? std::vector<crate::block_definitions::Block>{
											  crate::block_definitions::ROAD_ASPHALT}
									: block_types;
							if (stripe_absolute) {
								editor.set_block_absolute(stripe_block,
										stripe_x, stripe_y, stripe_z, stripe_hosts, std::nullopt);
							} else {
								editor.set_block(stripe_block,
										stripe_x, stripe_y, stripe_z, stripe_hosts, std::nullopt);
							}
						}
					}
					++stripe_length_counter;
					if (stripe_length_counter >= dash_length + gap_length) {
						stripe_length_counter = 0;
					}
				}
			}

			++segment_index;
			if (!bresenham_points.empty()) {
				cumulative_distance_from_start += bresenham_points.size() - 1;
			}
		}
		previous_node = std::make_optional(std::pair<int, int>{node.x, node.z});
	}
	if (is_bridge_member) {
		const bool start_is_boundary =
				!(bridge_member && bridge_member->start_internal_ramp);
		const bool end_is_boundary = !(bridge_member && bridge_member->end_internal_ramp);
		const bool schematic =
				bridge_style == crate::bridge_styles::BridgeStyle::Beam &&
				crate::bridge_styles::sweep_bridge_schematic(editor, bridge_path,
						block_range, editor.get_schematic_asset_root());
		if (!schematic)
			crate::bridge_styles::decorate_bridge_above_deck(editor, bridge_style,
					bridge_path, block_range, start_is_boundary, end_is_boundary);
	}
}

void generate_highways(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const std::vector<crate::osm_parser::ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const crate::RoadMaskBitmap &road_mask,
		const crate::bridges::BridgeStructureMap &bridge_structures,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals)
{
	auto highway_connectivity = build_highway_connectivity_map(all_elements);
	generate_highways_internal(editor, element, args, highway_connectivity,
			floodfill_timeout, road_mask, bridge_structures, bridge_surface,
			tunnel_portals);
}

void generate_highways(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const std::vector<crate::osm_parser::ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout)
{
	static const crate::RoadMaskBitmap empty_road_mask =
			crate::RoadMaskBitmap::new_empty();
	static const crate::bridges::BridgeStructureMap empty_bridge_structures;
	static const crate::bridges::BridgeSurfaceMap empty_bridge_surface;
	static const TunnelPortalMap empty_tunnel_portals;
	generate_highways(editor, element, args, all_elements, floodfill_timeout,
			empty_road_mask, empty_bridge_structures, empty_bridge_surface,
			empty_tunnel_portals);
}

void generate_siding(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &element,
		const crate::bridges::BridgeSurfaceMap &bridge_surface)
{
	std::optional<crate::coordinate_system::cartesian::XZPoint> previous_node;
	crate::block_definitions::Block siding_block =
			crate::block_definitions::STONE_BRICK_SLAB;
	for (const auto &node : element.nodes) {
		crate::coordinate_system::cartesian::XZPoint current_node = node.xz();
		if (previous_node.has_value()) {
			std::vector<std::tuple<int, int, int>> bresenham_points =
					crate::bresenham::bresenham_line(previous_node->x, 0,
							previous_node->z, current_node.x, 0, current_node.z);
			for (const auto &p : bresenham_points) {
				int bx = std::get<0>(p);
				int bz = std::get<2>(p);
				if (auto deck_y = bridge_surface.deck_y_at(bx, bz)) {
					if (!editor.check_for_block_absolute(bx, *deck_y, bz,
								std::optional<
										std::vector<crate::block_definitions::Block>>(
										ROAD_PROTECTED_SURFACES))) {
						editor.set_block_absolute(siding_block, bx, *deck_y + 1, bz,
								std::optional<
										std::vector<crate::block_definitions::Block>>(),
								std::optional<
										std::vector<crate::block_definitions::Block>>());
					}
				} else if (!editor.check_for_block(bx, 0, bz,
								   std::optional<
										   std::vector<crate::block_definitions::Block>>(
										   ROAD_PROTECTED_SURFACES))) {
					editor.set_block(siding_block, bx, 1, bz,
							std::optional<std::vector<crate::block_definitions::Block>>(),
							std::optional<
									std::vector<crate::block_definitions::Block>>());
				}
			}
		}
		previous_node.emplace(current_node);
	}
}

void generate_siding(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &element)
{
	static const crate::bridges::BridgeSurfaceMap empty_bridge_surface;
	generate_siding(editor, element, empty_bridge_surface);
}

void generate_aeroway(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &way, const crate::args::Args &args)
{
	const bool runway = way.tags.get("aeroway") == "runway";
	const bool taxiway = way.tags.get("aeroway") == "taxiway";
	double half_width_m = 12.0;
	if (auto it = way.tags.find("width"); it != way.tags.end()) {
		try {
			auto text = it->second;
			while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
				text.pop_back();
			if (!text.empty() && text.back() == 'm')
				text.pop_back();
			half_width_m = std::clamp(std::stod(text) * 0.5, 6.0, 40.0);
		} catch (...) {
		}
	}
	const int half_width =
			std::max(1, static_cast<int>(std::round(half_width_m * args.scale)));
	const auto base = runway ? crate::block_definitions::GRAY_CONCRETE
							 : crate::block_definitions::LIGHT_GRAY_CONCRETE;
	struct AerowayPoint { int x, z; float ux, uz, distance; };
	std::vector<AerowayPoint> points;
	float distance_from_start = 0.0f;
	for (size_t i = 1; i < way.nodes.size(); ++i) {
		const int x1 = way.nodes[i - 1].x, z1 = way.nodes[i - 1].z;
		const int x2 = way.nodes[i].x, z2 = way.nodes[i].z;
		const float length = std::max(1.0e-6f, std::hypot(
				static_cast<float>(x2 - x1), static_cast<float>(z2 - z1)));
		const float ux = static_cast<float>(x2 - x1) / length;
		const float uz = static_cast<float>(z2 - z1) / length;
		for (auto [x, y, z] : crate::bresenham::bresenham_line(x1, 0, z1, x2, 0, z2)) {
			if (!points.empty() && points.back().x == x && points.back().z == z)
				continue;
			if (!points.empty())
				distance_from_start += std::hypot(static_cast<float>(x - points.back().x),
						static_cast<float>(z - points.back().z));
			points.push_back({x, z, ux, uz, distance_from_start});
		}
	}
	for (const auto &point : points)
		for (int dx = -half_width; dx <= half_width; ++dx)
			for (int dz = -half_width; dz <= half_width; ++dz)
				editor.set_block(base, point.x + dx, 0, point.z + dz, std::nullopt, std::nullopt);
	const std::vector<crate::block_definitions::Block> over_base = {base};
	for (const auto &point : points) {
		const int x = point.x, z = point.z;
		if (taxiway)
			editor.set_block(crate::block_definitions::YELLOW_CONCRETE, x, 0, z,
					over_base, std::nullopt);
		const float dash_on = std::max(1.0f, 10.0f * static_cast<float>(args.scale));
		const float dash_off = std::max(1.0f, 6.0f * static_cast<float>(args.scale));
		if (runway && std::fmod(point.distance, dash_on + dash_off) < dash_on)
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x, 0, z, over_base,
					std::nullopt);
		if (runway) {
			const float inset = std::max(0.0f, static_cast<float>(half_width) - 1.0f);
			const int ox = static_cast<int>(std::lround(-point.uz * inset));
			const int oz = static_cast<int>(std::lround(point.ux * inset));
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x + ox, 0,
					z + oz, over_base, std::nullopt);
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x - ox, 0,
					z - oz, over_base, std::nullopt);
		}
	}
}

namespace
{
constexpr double HELIPAD_NODE_RADIUS_M = 8.0;
constexpr double HELIPAD_RING_FRACTION = 0.85;

void paint_helipad(WorldEditor &editor, const std::vector<std::pair<int, int>> &cells,
		int cx, int cz, const CoordinateBitmap &building_footprints)
{
	if (cells.empty())
		return;
	const auto covered = std::count_if(cells.begin(), cells.end(), [&](const auto &p) {
		return building_footprints.contains(p.first, p.second);
	});
	// Mostly-rooftop pads are deliberately left to building processing.
	if (covered * 2 > static_cast<std::ptrdiff_t>(cells.size()))
		return;
	const double radius = std::sqrt(static_cast<double>(cells.size()) / M_PI);
	const double ring_radius = std::max(2.5, radius * HELIPAD_RING_FRACTION);
	const int bar_half_h = std::clamp(static_cast<int>(radius * .45), 2, 6);
	const int bar_half_w = std::clamp(static_cast<int>(radius * .30), 1, 4);
	for (const auto &[x, z] : cells) {
		if (!building_footprints.contains(x, z))
			editor.set_block(crate::block_definitions::LIGHT_GRAY_CONCRETE, x, 0, z,
					std::nullopt, std::nullopt);
	}
	const std::vector<crate::block_definitions::Block> over_base = {
			crate::block_definitions::LIGHT_GRAY_CONCRETE};
	for (const auto &[x, z] : cells) {
		if (building_footprints.contains(x, z))
			continue;
		const int dx = x - cx, dz = z - cz;
		const double distance = std::sqrt(static_cast<double>(dx * dx + dz * dz));
		const bool on_ring = distance >= ring_radius - 1.2 && distance < ring_radius;
		const bool on_h = (std::abs(dx) == bar_half_w && std::abs(dz) <= bar_half_h) ||
						  (dz == 0 && std::abs(dx) <= bar_half_w);
		if (on_ring || on_h)
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x, 0, z, over_base,
					std::nullopt);
	}
	if (radius >= 5.0 && !building_footprints.contains(cx, cz))
		structures::maybe_place_helicopter(editor, cx, cz);
}

void paint_helipad_disc(WorldEditor &editor, int cx, int cz, const Args &args,
		const CoordinateBitmap &building_footprints)
{
	const int radius =
			std::max(4, static_cast<int>(std::round(HELIPAD_NODE_RADIUS_M * args.scale)));
	std::vector<std::pair<int, int>> cells;
	for (int dx = -radius; dx <= radius; ++dx)
		for (int dz = -radius; dz <= radius; ++dz)
			if (dx * dx + dz * dz <= radius * radius)
				cells.emplace_back(cx + dx, cz + dz);
	paint_helipad(editor, cells, cx, cz, building_footprints);
}
} // namespace

void generate_helipad_node(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints)
{
	paint_helipad_disc(editor, node.x, node.z, args, building_footprints);
}

void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints)
{
	if (way.tags.get("aeroway") != "helipad") {
		generate_aeroway(editor, way, args);
		return;
	}
	std::vector<std::pair<int, int>> outline;
	outline.reserve(way.nodes.size());
	for (const auto &node : way.nodes)
		outline.emplace_back(node.x, node.z);
	auto cells = floodfill::flood_fill_area(
			outline, std::optional<std::chrono::duration<double>>{});
	if (cells.empty()) {
		if (!way.nodes.empty())
			paint_helipad_disc(editor, way.nodes.front().x, way.nodes.front().z, args,
					building_footprints);
		return;
	}
	std::int64_t sx = 0, sz = 0;
	for (const auto &[x, z] : cells) {
		sx += x;
		sz += z;
	}
	int cx = static_cast<int>(sx / static_cast<std::int64_t>(cells.size()));
	int cz = static_cast<int>(sz / static_cast<std::int64_t>(cells.size()));
	if (std::find(cells.begin(), cells.end(), std::pair<int, int>{cx, cz}) ==
			cells.end()) {
		auto nearest = std::min_element(
				cells.begin(), cells.end(), [&](const auto &a, const auto &b) {
					const auto da = std::int64_t(a.first - cx) * (a.first - cx) +
									std::int64_t(a.second - cz) * (a.second - cz);
					const auto db = std::int64_t(b.first - cx) * (b.first - cx) +
									std::int64_t(b.second - cz) * (b.second - cz);
					return da < db;
				});
		cx = nearest->first;
		cz = nearest->second;
	}
	paint_helipad(editor, cells, cx, cz, building_footprints);
}

crate::CoordinateBitmap collect_road_surface_coords(
		const std::vector<crate::osm_parser::ProcessedElement> &elements,
		const crate::world_editor::WorldEditor &editor, const ::XZBBox &xzbbox, double scale)
{
	crate::CoordinateBitmap bitmap(xzbbox);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		auto it_highway = way.tags.find("highway");
		if (it_highway == way.tags.end())
			continue;
		const auto &highway_type = it_highway->second;
		if (renders_as_highway_tunnel(way) && tunnel_bore_fits(editor, way, scale))
			continue;
		if (highway_type == "street_lamp" || highway_type == "crossing" ||
				highway_type == "bus_stop")
			continue;
		if (way.tags.get("area") == "yes" || way.tags.get("indoor") == "yes")
			continue;
		try {
			auto it_level = way.tags.find("level");
			if (it_level != way.tags.end() && std::stoi(it_level->second) < 0)
				continue;
		} catch (...) {
		}

		const int block_range = highway_block_range(highway_type, way.tags, scale);
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &prev = way.nodes[i - 1];
			const auto &cur = way.nodes[i];
			const auto points =
					crate::bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			for (const auto &p : points) {
				const int bx = std::get<0>(p);
				const int bz = std::get<2>(p);
				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						bitmap.set(bx + dx, bz + dz);
					}
				}
			}
		}
	}
	return bitmap;
}

crate::CoordinateBitmap collect_building_passage_coords(
		const std::vector<crate::osm_parser::ProcessedElement> &elements,
		const ::XZBBox &xzbbox, double scale)
{
	bool has_any = false;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (way.tags.get("tunnel") == "building_passage" &&
					way.tags.contains("highway")) {
				has_any = true;
				break;
			}
		}
	}
	if (!has_any)
		return crate::CoordinateBitmap::new_empty();

	crate::CoordinateBitmap bitmap(xzbbox);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.tags.get("tunnel") != "building_passage")
			continue;
		auto it_highway = way.tags.find("highway");
		if (it_highway == way.tags.end())
			continue;

		const int block_range = highway_block_range(it_highway->second, way.tags, scale);
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &prev = way.nodes[i - 1];
			const auto &cur = way.nodes[i];
			const auto points =
					crate::bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			for (const auto &p : points) {
				const int bx = std::get<0>(p);
				const int bz = std::get<2>(p);
				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						bitmap.set(bx + dx, bz + dz);
					}
				}
			}
		}
	}
	return bitmap;
}

}
}
