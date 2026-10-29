#include <vector>
#include <unordered_map>
#include <string>
#include <optional>
#include <tuple>
#include <cmath>
#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string_view>
#include <unordered_set>
#include <utility>

#include "../../../arnis_adapter.h"
#include "../floodfill_cache.h"
#include "bridge_styles.h"
#include "bridges.h"
#include "surfaces.h"
#include "road_markings.h"
#include "highway_tunnels.h"
#include "highways.h"
#include "connected_blocks.h"
#include "road_mask_search.h"
#include "signage.h"
#include "../floodfill.h"
#include "../structures/helicopter.h"
#include "fm_highway_connectivity.h"
#include "fm_traffic_signals.h"
#include "traffic_lights.h"
#include "cars.h"
#include "../strict_parse.h"
namespace arnis
{

namespace highways
{

constexpr int MAX_BLOCK_RANGE = 8;
constexpr int LAYERED_LAMP_DECK_REACH = 3;

std::optional<double> parse_width_tag_m(
		const std::unordered_map<std::string, std::string> &tags)
{
	const auto it = tags.find("width");
	if (it == tags.end())
		return std::nullopt;
	std::string_view width = it->second;
	while (!width.empty() && std::isspace(static_cast<unsigned char>(width.front())))
		width.remove_prefix(1);
	while (!width.empty() && std::isspace(static_cast<unsigned char>(width.back())))
		width.remove_suffix(1);
	while (!width.empty() && width.back() == 'm')
		width.remove_suffix(1);
	while (!width.empty() && std::isspace(static_cast<unsigned char>(width.back())))
		width.remove_suffix(1);
	const auto metres = crate::strict_parse::f64(width);
	if (!metres || !std::isfinite(*metres) || *metres <= 0.0)
		return std::nullopt;
	return metres;
}

const std::vector<crate::block_definitions::Block> &default_road_mix()
{
	static const std::vector<crate::block_definitions::Block> palette{
			crate::block_definitions::GRAY_CONCRETE_POWDER,
			crate::block_definitions::CYAN_TERRACOTTA};
	return palette;
}

std::vector<crate::block_definitions::Block> palette_copy(
		std::span<const crate::block_definitions::Block> palette)
{
	return {palette.begin(), palette.end()};
}

std::vector<crate::block_definitions::Block> road_protected_surfaces()
{
	std::vector<crate::block_definitions::Block> result{
			crate::block_definitions::BLACK_CONCRETE,
			crate::block_definitions::GRAY_CONCRETE_POWDER,
			crate::block_definitions::CYAN_TERRACOTTA,
			crate::block_definitions::WHITE_CONCRETE,
			crate::block_definitions::YELLOW_CONCRETE,
			crate::block_definitions::RED_CONCRETE,
			crate::block_definitions::RED_TERRACOTTA,
			crate::block_definitions::WARPED_STAIRS,
			crate::block_definitions::WARPED_TRAPDOOR,
			crate::block_definitions::WARPED_SLAB,
			crate::block_definitions::STRIPPED_WARPED_STEM,
			crate::block_definitions::STRIPPED_WARPED_HYPHAE,
			crate::block_definitions::ANDESITE_WALL,
			crate::block_definitions::SEA_LANTERN,
			crate::block_definitions::SMOOTH_SANDSTONE_STAIRS};
	if (crate::block_definitions::STREETS_MARKINGS_AVAILABLE) {
		result.insert(result.end(),
				{crate::block_definitions::ROAD_MARK_DASHED_WHITE,
						crate::block_definitions::ROAD_MARK_DASHED_WHITE_R90,
						crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE,
						crate::block_definitions::ROAD_MARK_SOLID_WHITE_STRIPE_R90});
	}
	return result;
}

std::vector<crate::block_definitions::Block> cycleway_protected_surfaces()
{
	auto result = road_protected_surfaces();
	const auto &gray = crate::block_definitions::GRAY_CONCRETE_POWDER;
	const auto &cyan = crate::block_definitions::CYAN_TERRACOTTA;
	result.erase(
			std::remove_if(result.begin(), result.end(),
					[&](const auto &block) { return block == gray || block == cyan; }),
			result.end());
	return result;
}

double terrain_exact(const crate::world_editor::WorldEditor &editor, int x, int z)
{
	if (const auto height = editor.terrain_level_exact(x, z);
			height && std::isfinite(*height))
		return *height;
	return static_cast<double>(editor.get_ground_level(x, z));
}

constexpr int ROAD_STEP_COST = 5;
constexpr int ROAD_PROFILE_REACH = 2;
constexpr std::size_t ROAD_PROFILE_LEVELS = 2 * ROAD_PROFILE_REACH + 2;
constexpr std::size_t ROAD_HEADING_CELLS = 4;
constexpr int JUNCTION_RADIUS = 3;
constexpr std::array<double, 13> ROAD_SMOOTHING{0.0022, 0.0088, 0.0270, 0.0648, 0.1210,
		0.1760, 0.1995, 0.1760, 0.1210, 0.0648, 0.0270, 0.0088, 0.0022};

std::vector<int> level_profile(
		const std::vector<double> &smooth, const std::vector<std::optional<int>> &pins)
{
	const auto n = smooth.size();
	if (n == 0)
		return {};
	auto low = [&](std::size_t i) {
		return pins[i].value_or(
				static_cast<int>(std::floor(smooth[i])) - ROAD_PROFILE_REACH);
	};
	auto span = [&](std::size_t i) {
		return pins[i] ? std::size_t(1) : ROAD_PROFILE_LEVELS;
	};
	auto own = [&](std::size_t i, int y) {
		const double delta = y - smooth[i];
		return delta * delta;
	};
	std::array<double, ROAD_PROFILE_LEVELS> cost;
	cost.fill(std::numeric_limits<double>::infinity());
	for (std::size_t k = 0; k < span(0); ++k)
		cost[k] = own(0, low(0) + static_cast<int>(k));
	std::vector<std::uint8_t> back(n * ROAD_PROFILE_LEVELS, 0);
	for (std::size_t i = 1; i < n; ++i) {
		const int previous_low = low(i - 1), current_low = low(i);
		std::array<double, ROAD_PROFILE_LEVELS> next;
		next.fill(std::numeric_limits<double>::infinity());
		for (std::size_t k = 0; k < span(i); ++k) {
			const int y = current_low + static_cast<int>(k);
			double best = std::numeric_limits<double>::infinity();
			std::uint8_t best_j = 0;
			for (std::size_t j = 0; j < span(i - 1); ++j) {
				const double candidate =
						cost[j] +
						ROAD_STEP_COST * std::abs(y - previous_low - static_cast<int>(j));
				if (candidate < best) {
					best = candidate;
					best_j = static_cast<std::uint8_t>(j);
				}
			}
			next[k] = best + own(i, y);
			back[i * ROAD_PROFILE_LEVELS + k] = best_j;
		}
		cost = next;
	}
	std::size_t k = 0;
	for (std::size_t candidate = 1; candidate < span(n - 1); ++candidate)
		if (cost[candidate] < cost[k])
			k = candidate;
	std::vector<int> result(n);
	for (std::size_t i = n; i-- > 0;) {
		result[i] = low(i) + static_cast<int>(k);
		k = back[i * ROAD_PROFILE_LEVELS + k];
	}
	return result;
}

std::vector<int> road_profile(const crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &way, int block_range,
		const HighwayConnectivity &connectivity,
		const std::function<std::optional<int>(int, int)> &pin_at)
{
	if (way.nodes.empty())
		return {};
	std::vector<std::pair<int, int>> cells{{way.nodes.front().x, way.nodes.front().z}};
	std::vector<std::size_t> node_cells{0};
	for (std::size_t segment = 1; segment < way.nodes.size(); ++segment) {
		const auto &from = way.nodes[segment - 1];
		const auto &to = way.nodes[segment];
		const auto path =
				crate::bresenham::bresenham_line(from.x, 0, from.z, to.x, 0, to.z);
		for (std::size_t i = 1; i < path.size(); ++i)
			cells.emplace_back(std::get<0>(path[i]), std::get<2>(path[i]));
		node_cells.push_back(cells.size() - 1);
	}
	const auto n = cells.size();
	const double width = 2.0 * block_range + 1.0;
	std::vector<double> across(n);
	for (std::size_t i = 0; i < n; ++i) {
		const auto a = cells[i - std::min(i, ROAD_HEADING_CELLS)];
		const auto b = cells[std::min(n - 1, i + ROAD_HEADING_CELLS)];
		const double tx = b.first - a.first, tz = b.second - a.second;
		const double length = std::hypot(tx, tz);
		const double px = length > 0.0 ? -tz / length : 0.0;
		const double pz = length > 0.0 ? tx / length : 0.0;
		double sum = 0.0;
		for (int k = -block_range; k <= block_range; ++k) {
			const auto x = static_cast<int>(std::llround(cells[i].first + px * k));
			const auto z = static_cast<int>(std::llround(cells[i].second + pz * k));
			sum += terrain_exact(editor, x, z);
		}
		across[i] = sum / width;
	}
	std::vector<double> smooth(n);
	const auto half = ROAD_SMOOTHING.size() / 2;
	for (std::size_t i = 0; i < n; ++i)
		for (std::size_t k = 0; k < ROAD_SMOOTHING.size(); ++k) {
			const auto sample = std::clamp<std::ptrdiff_t>(
					static_cast<std::ptrdiff_t>(i) + static_cast<std::ptrdiff_t>(k) -
							static_cast<std::ptrdiff_t>(half),
					0, static_cast<std::ptrdiff_t>(n - 1));
			smooth[i] += ROAD_SMOOTHING[k] * across[static_cast<std::size_t>(sample)];
		}
	std::vector<std::optional<int>> pins(n);
	for (std::size_t i = 0; i < way.nodes.size(); ++i)
		if (auto y = pin_at(way.nodes[i].x, way.nodes[i].z))
			pins[node_cells[i]] = *y;
	if (connectivity.area) {
		auto inside = [&](std::size_t i) {
			return connectivity.area->contains(
					crate::cartesian::XZPoint{cells[i].first, cells[i].second});
		};
		for (std::size_t i = 1; i < n; ++i) {
			if (inside(i - 1) == inside(i))
				continue;
			const auto lower = cells[i - 1] < cells[i] ? i - 1 : i;
			const int y = static_cast<int>(std::lround(smooth[lower]));
			if (!pins[i - 1])
				pins[i - 1] = y;
			if (!pins[i])
				pins[i] = y;
		}
	}
	return level_profile(smooth, pins);
}

int junction_level(const crate::world_editor::WorldEditor &editor, int x, int z)
{
	double sum = 0.0;
	for (int dz = -JUNCTION_RADIUS; dz <= JUNCTION_RADIUS; ++dz)
		for (int dx = -JUNCTION_RADIUS; dx <= JUNCTION_RADIUS; ++dx)
			sum += terrain_exact(editor, x + dx, z + dz);
	const double side = 2 * JUNCTION_RADIUS + 1;
	return static_cast<int>(std::lround(sum / (side * side)));
}

double perpendicular_median_raw(const crate::world_editor::WorldEditor &editor, int set_x,
		int set_z, int centerline_x, int centerline_z, int block_range,
		bool dir_horizontal)
{
	assert(block_range >= 0 && block_range <= MAX_BLOCK_RANGE);
	std::array<double, 2 * MAX_BLOCK_RANGE + 1> ys{};
	const std::size_t len = static_cast<std::size_t>(block_range * 2 + 1);
	std::size_t i = 0;
	for (int t = -block_range; t <= block_range; ++t)
		ys[i++] = dir_horizontal ? terrain_exact(editor, set_x, centerline_z + t)
								 : terrain_exact(editor, centerline_x + t, set_z);
	std::nth_element(ys.begin(), ys.begin() + len / 2, ys.begin() + len);
	return ys[len / 2];
}

int perpendicular_median_ground_y(const crate::world_editor::WorldEditor &editor,
		int set_x, int set_z, int centerline_x, int centerline_z, int block_range,
		bool dir_horizontal)
{
	const double prev = perpendicular_median_raw(editor,
			dir_horizontal ? set_x - 1 : set_x, dir_horizontal ? set_z : set_z - 1,
			centerline_x, centerline_z, block_range, dir_horizontal);
	const double current = perpendicular_median_raw(editor, set_x, set_z, centerline_x,
			centerline_z, block_range, dir_horizontal);
	const double next = perpendicular_median_raw(editor,
			dir_horizontal ? set_x + 1 : set_x, dir_horizontal ? set_z : set_z + 1,
			centerline_x, centerline_z, block_range, dir_horizontal);
	return static_cast<int>(std::round(
			std::max(std::min(prev, current), std::min(std::max(prev, current), next))));
}

void precompute_row_medians(const crate::world_editor::WorldEditor &editor,
		int centerline_x, int centerline_z, int block_range, bool dir_horizontal,
		std::array<int, 2 * MAX_BLOCK_RANGE + 1> &out)
{
	assert(block_range >= 0 && block_range <= MAX_BLOCK_RANGE);
	std::array<double, 2 * MAX_BLOCK_RANGE + 3> raw{};
	const int len = block_range * 2 + 1;
	for (int i = 0; i < len + 2; ++i) {
		const int axial = -block_range - 1 + i;
		const int sample_x = dir_horizontal ? centerline_x + axial : centerline_x;
		const int sample_z = dir_horizontal ? centerline_z : centerline_z + axial;
		raw[static_cast<std::size_t>(i)] = perpendicular_median_raw(editor, sample_x,
				sample_z, centerline_x, centerline_z, block_range, dir_horizontal);
	}
	for (int i = 0; i < len; ++i) {
		std::array<double, 3> taps{raw[static_cast<std::size_t>(i)],
				raw[static_cast<std::size_t>(i + 1)],
				raw[static_cast<std::size_t>(i + 2)]};
		std::sort(taps.begin(), taps.end());
		out[static_cast<std::size_t>(i)] = taps[1];
	}
}

bool is_carriageway(const std::string &type)
{
	return type != "footway" && type != "path" && type != "steps" &&
		   type != "pedestrian" && type != "cycleway" && type != "bridleway" &&
		   type != "corridor" && type != "track" && type != "elevator" &&
		   type != "platform";
}

std::optional<std::string> surface_highway_type(const ProcessedWay &way)
{
	if (!way.tags.contains("highway"))
		return std::nullopt;
	const auto highway_type = way.tags.get("highway");
	// Piers may be tagged as footways, but their surface is a deck over water,
	// not a road surface that should suppress the water pass below it.
	if (way.tags.get("man_made") == "pier")
		return std::nullopt;
	if (highway_type == "street_lamp" || highway_type == "crossing" ||
			highway_type == "bus_stop" || way.tags.get("area") == "yes" ||
			way.tags.get("indoor") == "yes" || way.tags.get("aeroway") == "jet_bridge")
		return std::nullopt;
	if (const auto level = crate::strict_parse::i32(way.tags.get("level"));
			level && *level < 0)
		return std::nullopt;
	return highway_type;
}

bool tunnel_renders_at_surface(
		const ProcessedWay &way, const world_editor::WorldEditor *editor, double scale)
{
	return !renders_as_highway_tunnel(way) ||
		   (editor && !tunnel_bore_fits(*editor, way, scale));
}

bool is_pedestrian_way_tags(const crate::tags_t &tags)
{
	const auto highway = tags.get("highway");
	if (highway == "footway" || highway == "pedestrian" || highway == "steps")
		return true;
	const auto footway = tags.get("footway");
	// Match Rust's Some(v) check: a present-but-empty footway tag is still a
	// pedestrian-way hint; only the explicit negative value opts out.
	return tags.contains("footway") && footway != "no";
}

std::span<const crate::block_definitions::Block> surface_palette_view(
		const std::string &highway, const crate::tags_t &tags)
{
	if (highway == "construction") {
		static const std::vector<crate::block_definitions::Block> construction_bed{
				crate::block_definitions::COARSE_DIRT, crate::block_definitions::GRAVEL,
				crate::block_definitions::DIRT};
		return construction_bed;
	}
	if (highway == "cycleway")
		if (const auto blocks = crate::surfaces::cycleway_palette(tags); !blocks.empty())
			return blocks;
	const auto surface = tags.get("surface");
	if (is_pedestrian_way_tags(tags) &&
			(surface == "concrete" || surface == "paving_stones" || surface == "sett")) {
		static const std::vector<crate::block_definitions::Block> smooth_stone{
				crate::block_definitions::SMOOTH_STONE};
		return smooth_stone;
	}
	if (const auto blocks = crate::surfaces::get_blocks_for_surface(surface);
			!blocks.empty())
		return blocks;
	if (highway == "footway" || highway == "pedestrian" || highway == "service" ||
			highway == "steps") {
		static const std::vector<crate::block_definitions::Block> gray_concrete{
				crate::block_definitions::GRAY_CONCRETE};
		return gray_concrete;
	}
	if (highway == "path") {
		static const std::vector<crate::block_definitions::Block> dirt_path{
				crate::block_definitions::DIRT_PATH};
		return dirt_path;
	}
	if (highway == "escape") {
		static const std::vector<crate::block_definitions::Block> sand{
				crate::block_definitions::SAND};
		return sand;
	}
	return default_road_mix();
}

struct LaneLine
{
	float offset;
	bool solid;
	crate::block_definitions::Block block;
	bool centre;
};

int highway_default_lanes(const std::string &highway)
{
	return highway == "motorway" || highway == "primary" || highway == "trunk" ||
						   highway == "secondary" || highway == "tertiary"
				   ? 2
				   : 1;
}

int lane_marking_count(const std::string &highway, const crate::tags_t &tags)
{
	if (highway == "construction")
		return 1;
	const auto marked = tags.get("lane_markings");
	if (marked == "no")
		return 1;
	const auto parsed_lanes = crate::strict_parse::i32(tags.get("lanes"));
	int lanes = 1;
	if (parsed_lanes) {
		lanes = std::clamp(*parsed_lanes, 1, 16);
	} else {
		const auto junction = tags.get("junction");
		if (junction == "roundabout" || junction == "circular")
			lanes = 1;
		else
			lanes = highway_default_lanes(highway);
	}
	const bool minor = highway == "residential" || highway == "living_street" ||
					   highway == "service" || highway == "track";
	if (minor && lanes < 3 && marked != "yes")
		return 1;
	return lanes;
}

bool is_roundabout(const crate::tags_t &tags)
{
	const auto junction = tags.get("junction");
	return junction == "roundabout" || junction == "circular";
}

int oneway_sign(const std::string &highway, const crate::tags_t &tags)
{
	const auto value = tags.get("oneway");
	if (value == "yes" || value == "true" || value == "1")
		return 1;
	if (value == "-1" || value == "reverse")
		return -1;
	if (value == "no" || value == "false" || value == "0" || value == "reversible" ||
			value == "alternating")
		return 0;
	return is_roundabout(tags) || highway == "motorway" || highway == "motorway_link" ? 1
																					  : 0;
}

std::vector<LaneLine> lane_lines(const std::string &highway, const crate::tags_t &tags,
		int block_range, const crate::road_markings::RoadMarkingIndex &markings)
{
	const int lanes = lane_marking_count(highway, tags);
	if (lanes < 2)
		return {};
	const float width = static_cast<float>(2 * block_range + 1);
	const float lane_width = width / lanes;
	const bool two_way = oneway_sign(highway, tags) == 0;
	auto parse = [&](const char *key) -> std::optional<int> {
		return crate::strict_parse::i32(tags.get(key));
	};
	const int both = std::max(0, parse("lanes:both_ways").value_or(0));
	std::vector<int> separators;
	if (two_way) {
		if (const auto fw = parse("lanes:forward"), bw = parse("lanes:backward");
				fw && bw) {
			const int left = markings.drives_on_left ? *fw : *bw;
			if (*fw + *bw + both == lanes && left > 0 && left + both < lanes)
				for (int i = left; i <= left + both; ++i)
					separators.push_back(i);
		} else if ((lanes - both) % 2 == 0 && lanes > both + 1) {
			const int left = (lanes - both) / 2;
			for (int i = left; i <= left + both; ++i)
				separators.push_back(i);
		}
	}
	auto at = [&](int lane) { return lane * lane_width - width / 2.0f; };
	const bool double_yellow =
			markings.yellow_centre && separators.size() == 1 && lane_width >= 3.5f;
	std::vector<LaneLine> result;
	auto append_centre = [&](int lane) {
		const bool solid = lanes >= 4 || markings.yellow_centre;
		const auto block = markings.yellow_centre
								   ? crate::block_definitions::YELLOW_CONCRETE
								   : crate::block_definitions::WHITE_CONCRETE;
		if (double_yellow) {
			result.push_back({at(lane) - 1.0f, solid, block, true});
			result.push_back({at(lane) + 1.0f, solid, block, true});
		} else {
			result.push_back({at(lane), solid, block, true});
		}
	};
	if (width + 1.0f < 3.0f * lanes) {
		for (const auto lane : separators)
			append_centre(lane);
		return result;
	}
	for (int lane = 1; lane < lanes; ++lane) {
		if (std::find(separators.begin(), separators.end(), lane) != separators.end()) {
			append_centre(lane);
		} else {
			result.push_back(
					{at(lane), false, crate::block_definitions::WHITE_CONCRETE, false});
		}
	}
	return result;
}

bool fast_road(const std::string &highway, const crate::tags_t &tags)
{
	if (highway == "motorway" || highway == "trunk" || highway == "motorway_link" ||
			highway == "trunk_link")
		return true;
	const auto speed = tags.get("maxspeed");
	std::size_t digits = 0;
	while (digits < speed.size() && static_cast<unsigned char>(speed[digits]) >= '0' &&
			static_cast<unsigned char>(speed[digits]) <= '9')
		++digits;
	if (digits) {
		try {
			double kmh = std::stod(speed.substr(0, digits));
			if (speed.find("mph") != std::string::npos)
				kmh *= 1.609;
			return kmh >= 70.0;
		} catch (...) {
		}
	}
	if (speed == "none")
		return true;
	for (const auto key :
			{"maxspeed", "maxspeed:type", "source:maxspeed", "zone:traffic"}) {
		const auto value = tags.get(key);
		if (value.find("rural") != std::string::npos ||
				value.find("motorway") != std::string::npos ||
				value.find("nsl") != std::string::npos)
			return true;
	}
	return false;
}

int node_feature_base_y(const crate::world_editor::WorldEditor &editor,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const std::unordered_map<std::string, std::string> &tags, int x, int z,
		int bridge_radius = 0)
{
	bool layered = false;
	if (tags.find("indoor") == tags.end() || tags.at("indoor") != "yes") {
		if (const auto it = tags.find("layer"); it != tags.end()) {
			layered = crate::strict_parse::i32(it->second).value_or(0) > 0;
		}
	}
	if (layered) {
		if (const auto deck = bridge_surface.nearest_deck_cell(
					x, z, std::max(1, bridge_radius)))
			return deck->second;
	} else if (const auto deck = bridge_surface.nearby_deck_y(x, z, bridge_radius)) {
		return *deck;
	}
	return editor.get_absolute_y(x, 0, z);
}

void place_way_lamps(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &way, int block_range,
		const crate::RoadMaskBitmap &road_mask)
{
	const auto place_street_lamp = [&editor](int x, int z) {
		const int base = editor.get_absolute_y(x, 0, z);
		editor.register_lamp(x, z);
		editor.set_block_absolute(crate::block_definitions::SMOOTH_STONE, x, base + 1, z);
		for (int dy = 2; dy <= 4; ++dy)
			editor.set_block_absolute(
					crate::block_definitions::STONE_BRICK_WALL, x, base + dy, z);
		editor.set_block_with_properties_absolute(
				BlockWithProperties{crate::block_definitions::REDSTONE_LAMP,
						BlockWithProperties::Properties{{"lit", "true"}}},
				x, base + 5, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(
				crate::block_definitions::IRON_TRAPDOOR, x, base + 6, z);
	};
	constexpr std::size_t interval = 25;
	std::size_t tds = 0;
	int side = 1;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const int dx = b.x - a.x, dz = b.z - a.z;
		const double length =
				std::hypot(static_cast<double>(dx), static_cast<double>(dz));
		if (length == 0.0)
			continue;
		const int px = static_cast<int>(std::round(-static_cast<double>(dz) / length));
		const int pz = static_cast<int>(std::round(static_cast<double>(dx) / length));
		for (const auto &[x, y, z] :
				crate::bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)y;
			if (tds > 0 && tds % interval == 0) {
				for (const int candidate_side : {side, -side}) {
					const int lx = x + px * (block_range + 2) * candidate_side;
					const int lz = z + pz * (block_range + 2) * candidate_side;
					if (road_mask.contains(lx, lz) || editor.is_lc_water(lx, lz))
						continue;
					place_street_lamp(lx, lz);
					side = -side;
					break;
				}
			}
			++tds;
		}
	}
}

void drop_lamps_in_water(crate::world_editor::WorldEditor &editor)
{
	using namespace crate::block_definitions;
	const std::vector<Block> parts{
			SMOOTH_STONE, STONE_BRICK_WALL, REDSTONE_LAMP, IRON_TRAPDOOR};
	for (const auto &[x, z] : editor.take_lamps()) {
		const auto is = [&](int y, std::initializer_list<Block> blocks) {
			return editor.check_for_block_absolute(x, y, z,
					std::optional<std::vector<Block>>{std::vector<Block>(blocks)},
					std::nullopt);
		};
		const int ground = editor.get_ground_level(x, z);
		std::optional<int> head;
		for (int y = ground + 2; y <= ground + 12; ++y)
			if (is(y, {REDSTONE_LAMP}) && is(y - 1, {STONE_BRICK_WALL})) {
				head = y;
				break;
			}
		if (!head)
			continue;
		int foot = *head - 1;
		while (foot > *head - 4 && is(foot - 1, {STONE_BRICK_WALL, SMOOTH_STONE}))
			--foot;
		if (!is(foot - 1, {WATER}))
			continue;
		for (int y = foot; y <= *head + 1; ++y)
			editor.set_block_absolute(AIR, x, y, z, parts, std::nullopt);
	}
}

HighwayConnectivity build_highway_connectivity_map(
		const std::vector<crate::osm_parser::ProcessedElement> &elements)
{
	HighwayConnectivity connectivity;
	std::vector<std::uint64_t> points;
	std::vector<std::uint64_t> own;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const crate::osm_parser::ProcessedWay &way = element.as_way();
			auto it_highway = way.tags.find("highway");
			if (it_highway != way.tags.end()) {
				int layer_value = 0;
				auto it_layer = way.tags.find("layer");
				if (it_layer != way.tags.end())
					layer_value = crate::strict_parse::i32(it_layer->second).value_or(0);
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
					connectivity.ends[start_coord].push_back(layer_value);
					connectivity.ends[end_coord].push_back(layer_value);
				}
				own.clear();
				own.reserve(way.nodes.size());
				for (const auto &node : way.nodes)
					own.push_back((std::uint64_t(std::uint32_t(node.x)) << 32) |
								  std::uint32_t(node.z));
				std::sort(own.begin(), own.end());
				own.erase(std::unique(own.begin(), own.end()), own.end());
				points.insert(points.end(), own.begin(), own.end());
			}
		}
	}
	std::sort(points.begin(), points.end());
	for (std::size_t i = 1; i < points.size(); ++i)
		if (points[i - 1] == points[i])
			connectivity.shared.push_back(points[i]);
	connectivity.shared.erase(
			std::unique(connectivity.shared.begin(), connectivity.shared.end()),
			connectivity.shared.end());
	return connectivity;
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
		int current_layer, const HighwayConnectivity &highway_connectivity)
{
	std::pair<int, int> node_coord = {node.x, node.z};
	if (highway_connectivity.empty()) {
		return current_layer != 0;
	}
	auto it = highway_connectivity.ends.find(node_coord);
	if (it != highway_connectivity.ends.end()) {
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
	if (highway_type == "construction") {
		const auto construction = tags.find("construction");
		if (construction != tags.end() && !construction->second.empty() &&
				construction->second != "construction")
			return highway_block_range(construction->second, tags, scale);
	}
	int block_range = 2;
	bool scales_with_lanes = true;
	if (highway_type == "footway" || highway_type == "pedestrian" ||
			highway_type == "path" || highway_type == "cycleway") {
		block_range = 1;
		scales_with_lanes = false;
	} else if (highway_type == "motorway" || highway_type == "primary" ||
			   highway_type == "trunk") {
		block_range = 5;
	} else if (highway_type == "secondary") {
		block_range = 4;
	} else if (highway_type == "tertiary") {
		block_range = 2;
	} else if (highway_type == "track" || highway_type == "escape" ||
			   highway_type == "steps") {
		block_range = 1;
		scales_with_lanes = false;
	} else if (highway_type == "service") {
		block_range = 2;
	} else if (highway_type == "secondary_link" || highway_type == "tertiary_link") {
		block_range = 1;
	}
	const int default_lanes = highway_default_lanes(highway_type);
	const auto lane_tag = tags.find("lanes");
	const int lanes = std::clamp(
			lane_tag == tags.end()
					? default_lanes
					: crate::strict_parse::i32(lane_tag->second).value_or(default_lanes),
			1, 16);
	// An explicitly mapped physical width takes precedence. Otherwise retain the
	// historic type width as a minimum and enlarge multi-lane carriageways.
	if (const auto metres = parse_width_tag_m(tags)) {
		const double rounded = std::round(*metres / 2.0);
		block_range = static_cast<int>(
				std::clamp(rounded, 1.0, static_cast<double>(MAX_BLOCK_RANGE)));
	} else if (scales_with_lanes) {
		block_range = std::max(
				block_range, static_cast<int>(std::round((lanes * 3.5 - 1.0) / 2.0)));
	}
	block_range = std::clamp(block_range, 1, 8);
	if (scale < 1.0)
		block_range = std::max(1, static_cast<int>(std::floor(block_range * scale)));
	return block_range;
}

int calculate_point_elevation(std::size_t distance, std::size_t total_distance,
		int base_elevation, bool needs_start_slope, bool needs_end_slope,
		std::size_t slope_length)
{
	const auto ramp = [&](std::size_t d) {
		return std::min(static_cast<float>(d) / static_cast<float>(std::max<std::size_t>(
														1, slope_length)),
				1.0f);
	};
	float share = 1.0f;
	if (needs_start_slope)
		share = std::min(share, ramp(distance));
	if (needs_end_slope)
		share = std::min(
				share, ramp(total_distance > distance ? total_distance - distance : 0));
	return static_cast<int>(std::round(static_cast<float>(base_elevation) * share));
}

void generate_highways_internal(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const HighwayConnectivity &highway_connectivity,
		const crate::FloodFillCache *flood_fill_cache,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const crate::RoadMaskBitmap &road_mask,
		const crate::bridges::BridgeStructureMap &bridge_structures,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals,
		const crate::road_markings::RoadMarkingIndex &markings,
		const crate::CoordinateBitmap *tunnel_footprint)
{
	using crate::block_definitions::BLACK_WOOL;
	using crate::block_definitions::Block;
	using crate::block_definitions::COBBLESTONE_WALL;
	using crate::block_definitions::GLOWSTONE;
	using crate::block_definitions::GREEN_WOOL;
	using crate::block_definitions::IRON_BARS;
	using crate::block_definitions::LIGHT_GRAY_WALL_BANNER;
	using crate::block_definitions::OAK_FENCE;
	using crate::block_definitions::RED_WOOL;
	using crate::block_definitions::WHITE_WOOL;
	using crate::block_definitions::YELLOW_WOOL;
	(void)COBBLESTONE_WALL;
	(void)IRON_BARS;
	(void)BLACK_WOOL;
	(void)LIGHT_GRAY_WALL_BANNER;
	(void)OAK_FENCE;
	(void)GLOWSTONE;
	(void)GREEN_WOOL;
	(void)YELLOW_WOOL;
	(void)RED_WOOL;
	(void)WHITE_WOOL;

	auto it_highway = element.tags().find("highway");
	std::string fallback_highway_type;
	if (it_highway == element.tags().end()) {
		const auto crossing = element.tags().find("crossing");
		if (!element.is_node() || crossing == element.tags().end() ||
				crossing->second != "traffic_signals")
			return;
		// Rust sends signalised crossings through the highway processor even
		// when OSM omits highway=crossing on the node.
		fallback_highway_type = "crossing";
	}
	const std::string &highway_type = it_highway == element.tags().end()
											  ? fallback_highway_type
											  : it_highway->second;
	auto protected_surface_blocks = road_protected_surfaces();
	if (highway_type == "cycleway" && element.is_way() &&
			!crate::surfaces::cycleway_palette(element.as_way()).empty()) {
		protected_surface_blocks = cycleway_protected_surfaces();
	}
	const std::optional<std::vector<crate::block_definitions::Block>> protected_surfaces{
			std::move(protected_surface_blocks)};

	if (highway_type == "street_lamp") {
		if (element.is_node()) {
			int x = element.as_node().x;
			int z = element.as_node().z;
			const auto &tags = element.tags();
			const auto indoor = tags.find("indoor");
			const auto layer = tags.find("layer");
			const bool layered = (indoor == tags.end() || indoor->second != "yes") &&
								 layer != tags.end() &&
								 crate::strict_parse::i32(layer->second).value_or(0) > 0;
			if (layered) {
				if (const auto deck = bridge_surface.nearest_deck_cell(
							x, z, LAYERED_LAMP_DECK_REACH)) {
					x = deck->first.first;
					z = deck->first.second;
				}
			}
			const int base =
					node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
			editor.set_block_absolute(crate::block_definitions::COBBLESTONE_WALL, x,
					base + 1, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
			for (int dy = 2; dy <= 4; ++dy) {
				editor.set_block_absolute(crate::block_definitions::STREETS_POLE, x,
						base + dy, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
			editor.set_block_absolute(crate::block_definitions::EARTH_STREET_LAMP, x,
					base + 5, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
		}
		return;
	} else if (highway_type == "crossing" || highway_type == "traffic_signals") {
		auto it_crossing = element.tags().find("crossing");
		if (highway_type == "traffic_signals" ||
				(it_crossing != element.tags().end() &&
						it_crossing->second == "traffic_signals")) {
			if (element.is_node()) {
				const auto &signal_node = element.as_node();
				if (editor.traffic_signal_index) {
					editor.traffic_signal_index->place_node(
							editor, signal_node, [&](int px, int pz) {
								return node_feature_base_y(editor, bridge_surface,
										element.tags(), px, pz, 1);
							});
					return;
				}
				int x = element.as_node().x;
				int z = element.as_node().z;
				const int head_base =
						node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
				auto set_block = [&](const Block &block, int bx, int by, int bz) {
					editor.set_block_absolute(block, bx, by, bz,
							std::optional<std::vector<Block>>(),
							std::optional<std::vector<Block>>());
				};
				const auto anchor =
						road_mask.contains(x, z)
								? crate::element_processing::get_nearest_non_road_block(
										  x, z, 4, road_mask)
								: std::nullopt;
				if (anchor) {
					const auto [ax, az] = *anchor;
					const int pole_base = node_feature_base_y(
							editor, bridge_surface, element.tags(), ax, az, 1);
					set_block(COBBLESTONE_WALL, ax, pole_base + 1, az);
					const int arm_y = head_base + 6;
					for (int y = pole_base + 2; y < arm_y; ++y)
						set_block(IRON_BARS, ax, y, az);

					std::vector<std::pair<int, int>> arm;
					for (const auto &[lx, unused_y, lz] :
							crate::bresenham::bresenham_line(x, 0, z, ax, 0, az)) {
						(void)unused_y;
						arm.emplace_back(lx, lz);
					}
					arm = crate::connected_blocks::four_connected_line(arm);
					const std::unordered_set<std::pair<int, int>, PairHash> arm_cells(
							arm.begin(), arm.end());
					for (const auto &[lx, lz] : arm) {
						auto joins = [&, lx = lx, lz = lz](int dx, int dz) {
							return arm_cells.contains({lx + dx, lz + dz});
						};
						const auto bars = crate::connected_blocks::connected_fence(
								IRON_BARS, joins(0, -1), joins(0, 1), joins(1, 0),
								joins(-1, 0));
						editor.set_block_with_properties_absolute(
								bars, lx, arm_y, lz, std::nullopt, std::nullopt);
					}
				} else {
					set_block(COBBLESTONE_WALL, x, head_base + 1, z);
					set_block(IRON_BARS, x, head_base + 2, z);
					set_block(IRON_BARS, x, head_base + 3, z);
				}

				if (fm_place_native_traffic_signals(editor, x, head_base, z))
					return;

				set_block(BLACK_WOOL, x, head_base + 4, z);
				set_block(BLACK_WOOL, x, head_base + 5, z);
				const std::vector<std::pair<std::string, std::string>> banner_patterns{
						{"red", "minecraft:triangle_top"},
						{"lime", "minecraft:triangle_bottom"},
						{"yellow", "minecraft:circle"},
						{"black", "minecraft:curly_border"},
						{"black", "minecraft:border"}};
				const int banner_y = head_base + 5;
				const std::array<std::tuple<int, int, const char *>, 4> banner_offsets{
						{{0, -1, "north"}, {0, 1, "south"}, {-1, 0, "west"},
								{1, 0, "east"}}};
				for (const auto &[dx, dz, facing] : banner_offsets)
					editor.place_wall_banner(LIGHT_GRAY_WALL_BANNER, x + dx, banner_y,
							z + dz, facing, "light_gray", banner_patterns);
			}
		}
		return;
	} else if (highway_type == "bus_stop") {
		if (element.is_node()) {
			int x = element.as_node().x;
			int z = element.as_node().z;
			const int base =
					node_feature_base_y(editor, bridge_surface, element.tags(), x, z);
			for (int dy = 1; dy <= 3; ++dy) {
				editor.set_block_absolute(crate::block_definitions::COBBLESTONE_WALL, x,
						base + dy, z, std::optional<std::vector<Block>>(),
						std::optional<std::vector<Block>>());
			}
			editor.set_block_absolute(crate::block_definitions::WHITE_WOOL, x, base + 4,
					z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
			const int neighbour_base = node_feature_base_y(
					editor, bridge_surface, element.tags(), x + 1, z, 1);
			editor.set_block_absolute(crate::block_definitions::WHITE_WOOL, x + 1,
					neighbour_base + 4, z, std::optional<std::vector<Block>>(),
					std::optional<std::vector<Block>>());
			signage::place_bus_stop_signs(
					editor, element.as_node().tags, x, neighbour_base + 4, z);
		}
		return;
	} else {
		auto it_area = element.tags().find("area");
		if (it_area != element.tags().end() && it_area->second == "yes") {
			if (!element.is_way()) {
				return;
			}
			const crate::osm_parser::ProcessedWay &way = element.as_way();
			static const std::vector<crate::block_definitions::Block> stone_surface{
					crate::block_definitions::STONE};
			const auto surface_blocks =
					crate::surfaces::get_blocks_for_surface_way(way, stone_surface);
			crate::FloodFillResult filled_area;
			if (flood_fill_cache) {
				filled_area = flood_fill_cache->get_or_compute(way, args.timeout);
			} else {
				std::vector<std::pair<int, int>> polygon_coords;
				polygon_coords.reserve(way.nodes.size());
				for (const auto &n : way.nodes)
					polygon_coords.emplace_back(n.x, n.z);
				auto cells = crate::floodfill::flood_fill_area(
						polygon_coords, floodfill_timeout);
				filled_area =
						std::make_shared<const std::vector<std::pair<int32_t, int32_t>>>(
								std::move(cells));
			}
			for (const auto &p : *filled_area) {
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
	int block_range = 2;
	bool add_stripe = false;
	bool add_outline = false;
	double scale_factor = args.scale;

	int layer_value = 0;
	auto it_layer = element.tags().find("layer");
	if (it_layer != element.tags().end())
		layer_value = crate::strict_parse::i32(it_layer->second).value_or(0);
	// Treat negative layers as ground level (0)
	if (layer_value < 0) {
		layer_value = 0;
	}
	if (element.tags().find("indoor") != element.tags().end() &&
			element.tags().at("indoor") == "yes")
		layer_value = 0;

	auto it_level = element.tags().find("level");
	if (it_level != element.tags().end()) {
		if (const auto level_val = crate::strict_parse::i32(it_level->second);
				level_val && *level_val < 0)
			return;
	}

	if (highway_type == "footway" || highway_type == "pedestrian") {
		block_range = 1;
	} else if (highway_type == "path") {
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
		block_range = 2;
	} else if (highway_type == "secondary_link" || highway_type == "tertiary_link") {
		// Exit ramps, sliproads
		block_range = 1;
	} else if (highway_type == "escape") {
		// Sand trap for vehicles on mountainous roads
		block_range = 1;
	} else if (highway_type == "steps") {
		// Steps use slab-like paving; elevation interpolation below preserves the
		// tagged ascent while keeping the one-block pedestrian width.
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
	add_stripe = lane_marking_count(highway_type, way.tags) >= 2;
	// Rust does not paint lane markings on a tunnel way; preserve pavement width
	// and the tunnel's own interior treatment, but omit all surface stripe blocks.
	const bool tunnel_way = renders_as_highway_tunnel(way);
	const auto painted_lanes =
			tunnel_way ? std::vector<LaneLine>{}
					   : lane_lines(highway_type, way.tags, block_range, markings);
	std::optional<std::pair<float, float>> centre_span;
	for (const auto &line : painted_lanes) {
		if (!line.centre)
			continue;
		if (!centre_span)
			centre_span = std::make_pair(line.offset, line.offset);
		else {
			centre_span->first = std::min(centre_span->first, line.offset);
			centre_span->second = std::max(centre_span->second, line.offset);
		}
	}
	const auto *bridge_member = bridge_structures.lookup_member(way.id);
	const auto *bridge_ramp = bridge_structures.lookup_ramp(way.id);
	if (bridge_member && bridge_member->covered_by_wider)
		return;
	const bool is_bridge_member = bridge_member != nullptr;
	const bool is_bridge_ramp = bridge_ramp != nullptr;
	const auto tunnel_approach = (is_bridge_member || is_bridge_ramp)
										 ? std::nullopt
										 : tunnel_portals.approach(way.id);
	if (way.tags.get("lit") == "yes" && args.mode != GenerationMode::Blueprint &&
			!is_bridge_member && !is_bridge_ramp && layer_value == 0 &&
			highway_type != "steps" && highway_type != "construction")
		place_way_lamps(editor, way, block_range, road_mask);

	const auto block_types = surface_palette_view(highway_type, way.tags);
	const bool pedestrian = is_pedestrian_way_tags(way.tags);
	const auto crossing_paint =
			pedestrian ? markings.crossing_way_paint(way) : std::nullopt;
	const bool marked_crossing =
			crossing_paint &&
			crossing_paint->kind == crate::road_markings::CrossingPaint::Kind::Zebra;

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

	const std::size_t total_bresenham_length = calculate_total_bresenham_length(way);
	const auto bridge_asset_root = editor.get_schematic_asset_root();
	const bool bridge_module_selected =
			bridge_member && bridge_member->module_idx.has_value();
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
	std::vector<int> own_profile;
	if (!is_bridge_member && !is_bridge_ramp)
		own_profile = road_profile(editor, way, block_range, highway_connectivity,
				[&](int x, int z) -> std::optional<int> {
					if (!highway_connectivity.is_shared(x, z))
						return std::nullopt;
					if (tunnel_portals.is_open_end({x, z})) {
						const auto terrain = editor.terrain_level(x, z);
						return terrain.value_or(editor.get_ground_level(x, z));
					}
					double sum = 0.0;
					for (int dz = -3; dz <= 3; ++dz)
						for (int dx = -3; dx <= 3; ++dx)
							sum += terrain_exact(editor, x + dx, z + dz);
					return static_cast<int>(std::lround(sum / 49.0));
				});

	std::optional<std::pair<int, int>> previous_node;
	std::size_t segment_index = 0;
	std::size_t cumulative_distance_from_start = 0;
	std::optional<int> previous_bridge_y;
	std::optional<std::pair<int, int>> previous_rail_left;
	std::optional<std::pair<int, int>> previous_rail_right;
	std::optional<std::pair<std::pair<int, int>, int>> previous_parapet_left;
	std::optional<std::pair<std::pair<int, int>, int>> previous_parapet_right;
	const auto bridge_style = bridge_member ? bridge_member->style
											: crate::bridge_styles::BridgeStyle::Beam;
	const auto *bridge_pylons = bridge_member && bridge_member->cable_pylons
										? &*bridge_member->cable_pylons
										: nullptr;
	const auto *bridge_cable_span = bridge_member && bridge_member->cable_span
											? bridge_member->cable_span.get()
											: nullptr;
	const bool bridge_cables_carry_deck = bridge_pylons != nullptr;
	const auto hangs_at = [&](int x, int z) {
		return bridge_cable_span ? bridge_cable_span->suspends(x, z)
								 : bridge_cables_carry_deck;
	};
	std::vector<crate::bridge_styles::BridgePathSample> bridge_path;
	const auto [min_x, min_z] = editor.get_min_coords();
	const auto [max_x, max_z] = editor.get_max_coords();
	// The host generates one chunk from an extract containing complete ways.
	// Keep a halo for road width, outlines and terrain median samples, but do
	// not populate the ground cache for miles of road outside this chunk.
	// Bridges retain their full path for whole-span decorations and rail joins.
	const auto road_halo = std::int64_t(block_range) * 3 + 4;

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
			const float run =
					static_cast<float>(std::max(std::abs(x2 - x1), std::abs(z2 - z1)));
			const float ex = run > 0.0f ? static_cast<float>(x2 - x1) / run : 0.0f;
			const float ez = run > 0.0f ? static_cast<float>(z2 - z1) / run : 0.0f;
			const float along_inverse =
					1.0f /
					std::max(ex * ex + ez * ez, std::numeric_limits<float>::epsilon());
			auto along_offset = [&](int dx, int dz) {
				return static_cast<int>(std::lround(
						(static_cast<float>(dx) * ex + static_cast<float>(dz) * ez) *
						along_inverse));
			};

			const float dx_segment = static_cast<float>(x2 - x1);
			const float dz_segment = static_cast<float>(z2 - z1);
			const float segment_norm =
					std::sqrt(dx_segment * dx_segment + dz_segment * dz_segment);
			const float perp_x = segment_norm > 0.0f ? -dz_segment / segment_norm : 0.0f;
			const float perp_z = segment_norm > 0.0f ? dx_segment / segment_norm : 0.0f;
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
				if (!bridge_like &&
						(std::int64_t(x) < std::int64_t(min_x) - road_halo ||
								std::int64_t(x) > std::int64_t(max_x) + road_halo ||
								std::int64_t(z) < std::int64_t(min_z) - road_halo ||
								std::int64_t(z) > std::int64_t(max_z) + road_halo)) {
					continue;
				}

				// Calculate Y elevation for this point
				// Bridge structures drive absolute Y for full bridge groups and their ramps.
				int current_y;
				int elevation_offset = 0;
				bool use_absolute_y = false;
				if (bridge_member) {
					current_y = bridge_member->y_at(
							tds, total_bresenham_length, bridge_internal_ramp_length);
					use_absolute_y = true;
				} else if (bridge_ramp) {
					current_y = bridge_ramp->y_at(tds);
					use_absolute_y = true;
				} else {
					// Rust's longitudinal profile is absolute terrain; layer ramps are an
					// offset applied after the profile has been levelled.
					elevation_offset = calculate_point_elevation(tds,
							total_bresenham_length > 0 ? total_bresenham_length - 1 : 0,
							effective_elevation, effective_start_slope,
							effective_end_slope, slope_length);
					current_y =
							own_profile.empty()
									? elevation_offset
									: own_profile[std::min(tds, own_profile.size() - 1)] +
											  elevation_offset;
					use_absolute_y = !own_profile.empty();
				}
				auto approach_at = [&](int dx, int dz) -> std::optional<int> {
					if (!tunnel_approach || own_profile.empty())
						return std::nullopt;
					const auto last = static_cast<int>(own_profile.size() - 1);
					const int path_index = std::clamp(
							static_cast<int>(tds) + along_offset(dx, dz), 0, last);
					const int drop = tunnel_approach->offset(path_index);
					if (drop < 0)
						return own_profile[static_cast<std::size_t>(path_index)] + drop;
					return std::nullopt;
				};
				auto profile_y_at = [&](int sample_x, int sample_z) {
					if (own_profile.empty())
						return current_y;
					const auto last = static_cast<int>(own_profile.size() - 1);
					const int path_index =
							std::clamp(static_cast<int>(tds) +
											   along_offset(sample_x - x, sample_z - z),
									0, last);
					const int own = own_profile[static_cast<std::size_t>(path_index)] +
									elevation_offset;
					if (elevation_offset == 0)
						if (const auto existing =
										editor.surface_override(sample_x, sample_z);
								existing &&
								(!is_bridge_ramp || std::abs(*existing - own) <= 1))
							return *existing;
					return own;
				};
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
												protected_surfaces);
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
				std::array<int, 2 * MAX_BLOCK_RANGE + 1> flattened_ground_profile{};
				const bool has_flattened_ground_profile =
						flatten_width && !use_absolute_y;
				if (has_flattened_ground_profile)
					precompute_row_medians(editor, x, z, block_range, dir_horizontal,
							flattened_ground_profile);
				auto flattened_ground_y_at = [&](int sample_x, int sample_z) {
					if (flatten_width && use_absolute_y)
						return profile_y_at(sample_x, sample_z);
					const int offset = dir_horizontal ? sample_x - x : sample_z - z;
					if (has_flattened_ground_profile && offset >= -block_range &&
							offset <= block_range)
						return flattened_ground_profile[static_cast<std::size_t>(
								offset + block_range)];
					return perpendicular_median_ground_y(editor, sample_x, sample_z, x, z,
							block_range, dir_horizontal);
				};
				if (crossing_paint &&
						crossing_paint->kind ==
								crate::road_markings::CrossingPaint::Kind::Lines &&
						segment_norm > 0.0f &&
						(!crossing_paint->broken ||
								((dir_horizontal ? x : z) % 2 + 2) % 2 == 0)) {
					const auto line_block = crate::block_definitions::WHITE_CONCRETE;
					for (const int side : {-1, 1}) {
						const int offset = side * (block_range + 1);
						const int lx = dir_horizontal ? x + offset : x;
						const int lz = dir_horizontal ? z : z + offset;
						const auto *line_road = markings.carriageway_at(way.id, lx, lz);
						if (!line_road)
							continue;
						const auto line_palette = palette_copy(line_road->palette());
						const bool absolute = use_absolute_y || flatten_width;
						const auto line_approach = approach_at(lx - x, lz - z);
						const int line_y =
								line_approach ? *line_approach
								: absolute
										? (use_absolute_y ? (flatten_width ? profile_y_at(
																					 lx,
																					 lz)
																		   : current_y)
														  : flattened_ground_y_at(
																	lx, lz) +
																	current_y)
										: current_y;
						if (absolute)
							editor.set_block_absolute(line_block, lx, line_y, lz,
									line_palette, std::nullopt);
						else
							editor.set_block(line_block, lx, line_y, lz, line_palette,
									std::nullopt);
					}
				}

				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						int set_x = x + dx;
						int set_z = z + dz;
						const bool surface_absolute = use_absolute_y || flatten_width;
						const auto approach_y = approach_at(dx, dz);
						const int surface_y =
								approach_y ? *approach_y
								: surface_absolute
										? (use_absolute_y ? (flatten_width ? profile_y_at(
																					 set_x,
																					 set_z)
																		   : current_y)
														  : flattened_ground_y_at(
																	set_x, set_z) +
																	current_y)
										: current_y;
						if (flatten_width && elevation_offset == 0 &&
								!(approach_y && tunnel_footprint &&
										tunnel_footprint->contains(set_x, set_z)))
							editor.register_road_surface_y(set_x, set_z, surface_y);

						bool zebra = false;
						bool crossing_horizontal = false;
						if (marked_crossing) {
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
						const auto *crossed_road =
								crossing_paint
										? markings.carriageway_at(way.id, set_x, set_z)
										: nullptr;
						if (marked_crossing && !crossed_road)
							zebra = false;

						// Node-mapped crossings and junction controls are indexed before
						// rendering so that marks remain stable across way boundaries.
						bool transverse_white = false;
						if (segment_norm > 0.0f) {
							if (const auto *way_marks = markings.way(way.id)) {
								const auto t = static_cast<std::uint32_t>(tds);
								for (const auto &mark : way_marks->marks) {
									if (!mark.covers(t))
										continue;
									const float ux = dx_segment / segment_norm;
									const float uz = dz_segment / segment_norm;
									const float px = -uz;
									const float pz = ux;
									const float along = dx * ux + dz * uz;
									const float across = dx * px + dz * pz;
									const float reach =
											block_range * (std::abs(px) + std::abs(pz));
									if (std::abs(along) > 0.5f ||
											std::abs(across) > reach + 0.01f)
										continue;
									const bool on_side =
											mark.side > 0
													? (centre_span ? across > centre_span->second +
																					  0.5f
																   : across > -0.5f)
											: mark.side < 0
													? (centre_span ? across < centre_span->first -
																					  0.5f
																   : across < 0.5f)
													: true;
									if (!on_side)
										continue;
									const float spacing = std::abs(px) + std::abs(pz);
									const int stripe = static_cast<int>(std::floor(
											(set_x * px + set_z * pz) / spacing));
									const bool bar = ((stripe % 2) + 2) % 2 == 0;
									transverse_white =
											mark.kind ==
													crate::road_markings::TransverseMark::
															Kind::Stop ||
											((mark.kind == crate::road_markings::
																	 TransverseMark::Kind::
																			 GiveWay ||
													 mark.kind ==
															 crate::road_markings::
																	 TransverseMark::
																			 Kind::Zebra ||
													 mark.kind ==
															 crate::road_markings::
																	 TransverseMark::Kind::
																			 CrossingLines) &&
													(mark.kind != crate::road_markings::
																			TransverseMark::
																					Kind::CrossingLines ||
															!mark.broken || bar) &&
													bar);
									if (mark.kind ==
													crate::road_markings::TransverseMark::
															Kind::CrossingLines &&
											!mark.broken)
										transverse_white = true;
									if (transverse_white)
										break;
								}
							}
						}

						if (zebra || transverse_white) {
							if (transverse_white)
								crossing_horizontal =
										std::abs(dx_segment) >= std::abs(dz_segment);
							const bool native_marking =
									crate::block_definitions::STREETS_MARKINGS_AVAILABLE;
							const auto zebra_block =
									native_marking
											? (crossing_horizontal
															  ? crate::block_definitions::
																		ROAD_MARK_SOLID_WHITE_STRIPE
															  : crate::block_definitions::
																		ROAD_MARK_SOLID_WHITE_STRIPE_R90)
											: crate::block_definitions::WHITE_CONCRETE;
							const auto &zebra_hosts =
									crossed_road
											? palette_copy(crossed_road->palette())
											: std::vector<
													  crate::block_definitions::Block>(
													  block_types.begin(),
													  block_types.end());
							if (surface_absolute) {
								editor.set_block_absolute(zebra_block, set_x, surface_y,
										set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												zebra_hosts),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							} else {
								editor.set_block(zebra_block, set_x, surface_y, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(
												zebra_hosts),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
						} else {
							const auto road_block = crate::surfaces::semirandom_surface(
									set_x, set_z,
									crossed_road ? crossed_road->palette() : block_types);
							if (surface_absolute) {
								editor.set_block_absolute(road_block, set_x, surface_y,
										set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										protected_surfaces);
							} else {
								editor.set_block(road_block, set_x, surface_y, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										protected_surfaces);
							}
						}

						const bool elevated_deck =
								(is_bridge_member && !bridge_module_selected) ||
								is_bridge_ramp || effective_elevation > 0;
						// Ordinary roads register their ground surface for embankment filling;
						// only bridges and explicit overpasses need a separate foundation.
						if (elevated_deck && (use_absolute_y || surface_y > 0)) {
							// Add 1 layer of stone bricks underneath the highway surface
							const auto foundation_block =
									is_bridge_member
											? (bridge_cable_span
															  ? bridge_cable_span->paint
															  : crate::bridge_styles::
																		foundation_block(
																				bridge_style))
											: crate::block_definitions::STONE_BRICKS;
							if (use_absolute_y) {
								editor.set_block_absolute(foundation_block, set_x,
										surface_y - 1, set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							} else {
								editor.set_block(foundation_block, set_x, surface_y - 1,
										set_z,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
						}

						// Add support pillars for elevated highways/bridges
						if (elevated_deck && (use_absolute_y || surface_y > 0)) {
							if (is_bridge_member) {
								const std::size_t interval =
										crate::bridge_styles::pillar_interval(
												bridge_style, block_range);
								const bool is_centerline = dx == 0 && dz == 0;
								const bool is_pier_step = is_centerline && interval > 0 &&
														  tds % interval == interval / 2;
								const int cell_tds = std::clamp(
										static_cast<int>(tds) + along_offset(dx, dz), 0,
										static_cast<int>(total_bresenham_length - 1));
								const int centerline_ground_y =
										editor.get_ground_level(x, z);
								const bool blocked = use_absolute_y &&
													 bridge_surface.support_blocked(
															 set_x, set_z, surface_y);
								if ((bridge_style == crate::bridge_styles::BridgeStyle::
															 Arch &&
											!blocked) ||
										(bridge_style != crate::bridge_styles::
																 BridgeStyle::Arch &&
												!hangs_at(set_x, set_z)))
									crate::bridge_styles::place_bridge_support_below_deck(
											editor, bridge_style, set_x, surface_y, set_z,
											centerline_ground_y,
											static_cast<std::size_t>(cell_tds),
											total_bresenham_length, use_absolute_y,
											is_centerline, false);
								if (is_pier_step && use_absolute_y &&
										!hangs_at(set_x, set_z) && !blocked &&
										bridge_rail_perp.has_value())
									crate::bridge_styles::place_pier_bent(editor,
											bridge_surface, bridge_style, set_x, set_z,
											surface_y, *bridge_rail_perp, block_range);
							} else if (use_absolute_y) {
								add_highway_support_pillar_absolute(editor, set_x,
										surface_y, set_z, dx, dz, block_range);
							} else {
								add_highway_support_pillar(editor, set_x, surface_y,
										set_z, dx, dz, block_range);
							}
						}
					}
				}

				if (bridge_like && bridge_rail_perp.has_value()) {
					const float perp_x = bridge_rail_perp->first;
					const float perp_z = bridge_rail_perp->second;
					const auto rail_block =
							is_bridge_member
									? (bridge_cable_span
													  ? bridge_cable_span->paint
													  : crate::bridge_styles::rail_block(
																bridge_style))
									: crate::block_definitions::LIGHT_GRAY_CONCRETE;
					const auto rail_foundation =
							is_bridge_member
									? (bridge_cable_span ? bridge_cable_span->paint
														 : crate::bridge_styles::
																   rail_foundation_block(
																		   bridge_style))
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
						auto &previous_parapet = side == 0 ? previous_parapet_left
														   : previous_parapet_right;
						const int rail_x = static_cast<int>(std::round(
								static_cast<float>(x) + perp_x * rail_dist * sign));
						const int rail_z = static_cast<int>(std::round(
								static_cast<float>(z) + perp_z * rail_dist * sign));
						const std::pair<int, int> rail_cell{rail_x, rail_z};
						const auto cells_to_fill =
								previous_rail.has_value()
										? stair_fill_cells(*previous_rail, rail_cell)
										: std::vector<std::pair<int, int>>{rail_cell};
						const bool skip_side_railing =
								is_bridge_member &&
								(!crate::bridge_styles::has_side_railing(bridge_style) ||
										bridge_module_selected);
						for (const auto &rail_fill : cells_to_fill) {
							const int rx = rail_fill.first;
							const int rz = rail_fill.second;
							if (skip_side_railing)
								continue;
							if (bridge_surface.deck_near(rx, rz, current_y, 2)) {
								previous_parapet.reset();
								continue;
							}
							if (is_bridge_ramp &&
									current_y <= editor.get_ground_level(rx, rz)) {
								previous_parapet.reset();
								continue;
							}
							editor.set_block_absolute(rail_block, rx, current_y, rz,
									std::optional<std::vector<
											crate::block_definitions::Block>>(),
									protected_surfaces);
							if (current_y > 0) {
								editor.set_block_absolute(rail_foundation, rx,
										current_y - 1, rz,
										std::optional<std::vector<
												crate::block_definitions::Block>>(),
										std::optional<std::vector<
												crate::block_definitions::Block>>());
							}
							if (parapet) {
								const int top = current_y + 1;
								crate::connected_blocks::place_connected(
										editor, *parapet, rx, top, rz);
								if (previous_parapet.has_value()) {
									const auto &[previous_cell, previous_top] =
											*previous_parapet;
									const int px = previous_cell.first;
									const int pz = previous_cell.second;
									const bool adjacent =
											std::abs(px - rx) + std::abs(pz - rz) == 1;
									if (adjacent && previous_top == top - 1) {
										crate::connected_blocks::place_connected(
												editor, *parapet, px, top, pz);
									} else if (adjacent && previous_top == top + 1) {
										crate::connected_blocks::place_connected(
												editor, *parapet, rx, previous_top, rz);
									}
								}
								previous_parapet = std::make_pair(rail_cell, top);
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
					const auto *way_marks = markings.way(way.id);
					const auto t = static_cast<std::uint32_t>(tds);
					if (!way_marks || !way_marks->in_gap(t)) {
						const bool fast = fast_road(highway_type, way.tags);
						const int dash_on = std::max(1,
								static_cast<int>(
										std::lround((fast ? 6.0 : 3.0) * scale_factor)));
						const int dash_off = std::max(1,
								static_cast<int>(
										std::lround((fast ? 12.0 : 6.0) * scale_factor)));
						const int dash_period = dash_on + dash_off;
						const std::int64_t phase = way_marks
														   ? way_marks->dash_phase(t)
														   : static_cast<std::int64_t>(t);
						const int dash_phase = static_cast<int>(
								((phase % dash_period) + dash_period) % dash_period);
						const bool dash = dash_phase < dash_on;
						for (const auto &line : painted_lanes) {
							if (!line.solid && !dash)
								continue;
							const int stripe_x = static_cast<int>(
									std::round(x + perp_x * line.offset));
							const int stripe_z = static_cast<int>(
									std::round(z + perp_z * line.offset));
							const bool stripe_absolute = use_absolute_y || flatten_width;
							const int stripe_y =
									stripe_absolute
											? (use_absolute_y ? current_y
															  : flattened_ground_y_at(
																		stripe_x,
																		stripe_z) +
																		current_y)
											: current_y;
							const bool native_marking =
									line.block ==
											crate::block_definitions::WHITE_CONCRETE &&
									crate::block_definitions::
											STREETS_MARKINGS_AVAILABLE &&
									std::find(block_types.begin(), block_types.end(),
											crate::block_definitions::ROAD_ASPHALT) !=
											block_types.end();
							const auto stripe_block =
									native_marking
											? (line.solid ? (dir_horizontal ? crate::block_definitions::
																					  ROAD_MARK_SOLID_WHITE_STRIPE_R90
																			: crate::block_definitions::
																					  ROAD_MARK_SOLID_WHITE_STRIPE)
														  : (dir_horizontal ? crate::block_definitions::
																					  ROAD_MARK_DASHED_WHITE_R90
																			: crate::block_definitions::
																					  ROAD_MARK_DASHED_WHITE))
											: line.block;
							const auto stripe_hosts =
									native_marking
											? std::vector<crate::block_definitions::
															  Block>{crate::
															  block_definitions::
																	  ROAD_ASPHALT}
											: std::vector<
													  crate::block_definitions::Block>(
													  block_types.begin(),
													  block_types.end());
							if (stripe_absolute) {
								editor.set_block_absolute(stripe_block, stripe_x,
										stripe_y, stripe_z, stripe_hosts, std::nullopt);
							} else {
								editor.set_block(stripe_block, stripe_x, stripe_y,
										stripe_z, stripe_hosts, std::nullopt);
							}
						}
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
				!bridge_member || bridge_member->start_is_group_boundary;
		const bool end_is_boundary =
				!bridge_member || bridge_member->end_is_group_boundary;
		const bool schematic =
				bridge_module_selected &&
				crate::bridge_styles::sweep_bridge_schematic(editor, bridge_surface,
						bridge_path, bridge_member->module_idx, bridge_asset_root);
		const bool cable_style =
				bridge_style == crate::bridge_styles::BridgeStyle::Suspension ||
				bridge_style == crate::bridge_styles::BridgeStyle::CableStayed;
		if (!schematic && (!cable_style || bridge_pylons))
			crate::bridge_styles::decorate_bridge_above_deck(editor, bridge_style,
					bridge_surface, bridge_path, block_range, start_is_boundary,
					end_is_boundary, bridge_pylons, bridge_cable_span);
	}
}

void generate_highways(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedElement &element, const crate::args::Args &args,
		const std::vector<crate::osm_parser::ProcessedElement> &all_elements,
		const std::optional<std::chrono::duration<double>> &floodfill_timeout,
		const crate::RoadMaskBitmap &road_mask,
		const crate::bridges::BridgeStructureMap &bridge_structures,
		const crate::bridges::BridgeSurfaceMap &bridge_surface,
		const TunnelPortalMap &tunnel_portals,
		const crate::CoordinateBitmap *tunnel_footprint)
{
	auto highway_connectivity = build_highway_connectivity_map(all_elements);
	if (args.one_world_run) {
		const auto [min_x, min_z] = editor.get_min_coords();
		const auto [max_x, max_z] = editor.get_max_coords();
		highway_connectivity.meet_at_edge_of(
				crate::cartesian::XZBBox::rect_from_min_max(min_x, min_z, max_x, max_z));
	}
	const auto geo = editor.geographic_bounds();
	const auto regional_markings = crate::road_markings::RoadMarkingIndex::build(
			all_elements, args.scale,
			crate::decals::detect_region((geo[0] + geo[1]) * .5, (geo[2] + geo[3]) * .5));
	if (editor.car_network && element.is_way())
		editor.car_network->place_way(editor, element.as_way().id);
	generate_highways_internal(editor, element, args, highway_connectivity, nullptr,
			floodfill_timeout, road_mask, bridge_structures, bridge_surface,
			tunnel_portals, regional_markings, tunnel_footprint);
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
			empty_tunnel_portals, nullptr);
}

void generate_siding(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &element,
		const crate::bridges::BridgeSurfaceMap &bridge_surface)
{
	const std::optional<std::vector<crate::block_definitions::Block>> protected_surfaces{
			road_protected_surfaces()};
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
					if (!editor.check_for_block_absolute(
								bx, *deck_y, bz, protected_surfaces)) {
						editor.set_block_absolute(siding_block, bx, *deck_y + 1, bz,
								std::optional<
										std::vector<crate::block_definitions::Block>>(),
								std::optional<
										std::vector<crate::block_definitions::Block>>());
					}
				} else if (!editor.check_for_block(bx, 0, bz, protected_surfaces)) {
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

void generate_aeroway_impl(crate::world_editor::WorldEditor &editor,
		const crate::osm_parser::ProcessedWay &way, const crate::args::Args &args,
		const crate::FloodFillCache *fill_cache)
{
	const bool runway = way.tags.get("aeroway") == "runway";
	const bool taxiway = way.tags.get("aeroway") == "taxiway";
	const auto aeroway = way.tags.get("aeroway");
	const bool stopway = aeroway == "stopway";
	const bool stand_line = aeroway == "parking_position" || aeroway == "taxilane";
	const bool closed = way.nodes.size() >= 4 &&
						way.nodes.front().x == way.nodes.back().x &&
						way.nodes.front().z == way.nodes.back().z;
	const auto area_kind = way.tags.get("area:aeroway");
	const auto ordinary_area_kind =
			aeroway == "apron" || (way.tags.get("area") == "yes" && (runway || taxiway));
	const bool runway_area =
			closed &&
			(area_kind == "runway" || area_kind == "stopway" || area_kind == "blastpad" ||
					area_kind == "blast_pad" ||
					(area_kind.empty() && ordinary_area_kind && (runway || stopway)));
	const bool apron_area =
			closed &&
			(area_kind == "taxiway" || area_kind == "apron" ||
					area_kind == "parking_position" ||
					(area_kind.empty() && ordinary_area_kind && !runway && !stopway));
	if (runway_area || apron_area) {
		std::optional<Block> unpaved;
		const auto surface = way.tags.get("surface");
		static const std::unordered_set<std::string> unpaved_surfaces{"grass", "dirt",
				"ground", "earth", "soil", "unpaved", "gravel", "fine_gravel",
				"compacted", "sand", "clay", "mud"};
		if (unpaved_surfaces.contains(surface))
			if (const auto palette = crate::surfaces::get_blocks_for_surface(surface);
					!palette.empty())
				unpaved = palette.front();
		const Block block = unpaved.value_or(
				runway_area ? crate::block_definitions::GRAY_CONCRETE
							: crate::block_definitions::LIGHT_GRAY_CONCRETE);
		std::vector<std::pair<int, int>> outline;
		std::vector<std::pair<int, int>> uncached_cells;
		if (!fill_cache) {
			outline.reserve(way.nodes.size());
			for (const auto &node : way.nodes)
				outline.emplace_back(node.x, node.z);
			uncached_cells = crate::floodfill::flood_fill_area(outline);
		}
		const auto cached_cells = fill_cache
										  ? fill_cache->get_or_compute(way, args.timeout)
										  : FloodFillResult{};
		const auto &cells = fill_cache ? *cached_cells : uncached_cells;
		const std::vector<Block> runway_overwrites{
				crate::block_definitions::LIGHT_GRAY_CONCRETE,
				crate::block_definitions::YELLOW_CONCRETE};
		const std::optional<std::vector<Block>> over =
				block == crate::block_definitions::GRAY_CONCRETE
						? std::optional<std::vector<Block>>(runway_overwrites)
						: std::nullopt;
		for (const auto &[x, z] : cells)
			if (!editor.nested_area_owns(x, z))
				editor.set_block(block, x, 0, z, over, std::nullopt);
		return;
	}
	if (!(runway || taxiway || stand_line || stopway))
		return;
	std::optional<Block> unpaved;
	const auto surface = way.tags.get("surface");
	static const std::unordered_set<std::string> unpaved_surfaces{"grass", "dirt",
			"ground", "earth", "soil", "unpaved", "gravel", "fine_gravel", "compacted",
			"sand", "clay", "mud"};
	if (unpaved_surfaces.contains(surface))
		if (const auto palette = crate::surfaces::get_blocks_for_surface(surface);
				!palette.empty())
			unpaved = palette.front();
	double half_width_m = stand_line ? 2.5 : 12.0;
	if (!stand_line)
		if (const auto metres = parse_width_tag_m(way.tags))
			half_width_m = std::clamp(*metres * 0.5, 6.0, 40.0);
	const int half_width =
			std::max(1, static_cast<int>(std::round(half_width_m * args.scale)));
	const auto base = unpaved.value_or(
			runway || stopway ? crate::block_definitions::GRAY_CONCRETE
							  : crate::block_definitions::LIGHT_GRAY_CONCRETE);
	const std::vector<crate::block_definitions::Block> runway_overwrites = {
			crate::block_definitions::LIGHT_GRAY_CONCRETE,
			crate::block_definitions::YELLOW_CONCRETE};
	std::optional<std::vector<crate::block_definitions::Block>> base_over;
	if (runway && !unpaved)
		base_over = runway_overwrites;
	struct AerowayPoint
	{
		int x, z;
		float ux, uz, distance;
	};
	std::vector<AerowayPoint> points;
	float distance_from_start = 0.0f;
	for (size_t i = 1; i < way.nodes.size(); ++i) {
		const int x1 = way.nodes[i - 1].x, z1 = way.nodes[i - 1].z;
		const int x2 = way.nodes[i].x, z2 = way.nodes[i].z;
		const float length = std::max(1.0e-6f,
				std::hypot(static_cast<float>(x2 - x1), static_cast<float>(z2 - z1)));
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
				editor.set_block(
						base, point.x + dx, 0, point.z + dz, base_over, std::nullopt);
	if (stopway || unpaved)
		return;
	const std::vector<crate::block_definitions::Block> over_base = {base};
	for (const auto &point : points) {
		const int x = point.x, z = point.z;
		if (taxiway || stand_line)
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
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x + ox, 0, z + oz,
					over_base, std::nullopt);
			editor.set_block(crate::block_definitions::WHITE_CONCRETE, x - ox, 0, z - oz,
					over_base, std::nullopt);
		}
	}
}

namespace
{
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

void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args)
{
	generate_aeroway_impl(editor, way, args, nullptr);
}

std::optional<AerowayAreaKind> aeroway_area_kind(const ProcessedWay &way)
{
	const auto aeroway = way.tags.get("aeroway");
	const bool runway = aeroway == "runway";
	const bool taxiway = aeroway == "taxiway";
	const bool stopway = aeroway == "stopway";
	const bool closed = way.nodes.size() >= 4 &&
						way.nodes.front().x == way.nodes.back().x &&
						way.nodes.front().z == way.nodes.back().z;
	if (!closed)
		return std::nullopt;
	const auto area_kind = way.tags.get("area:aeroway");
	const bool ordinary_area_kind =
			aeroway == "apron" || (way.tags.get("area") == "yes" && (runway || taxiway));
	const bool runway_area =
			area_kind == "runway" || area_kind == "stopway" || area_kind == "blastpad" ||
			area_kind == "blast_pad" ||
			(area_kind.empty() && ordinary_area_kind && (runway || stopway));
	const bool apron_area =
			area_kind == "taxiway" || area_kind == "apron" ||
			area_kind == "parking_position" ||
			(area_kind.empty() && ordinary_area_kind && !runway && !stopway);
	if (runway_area)
		return AerowayAreaKind::Runway;
	if (apron_area)
		return AerowayAreaKind::Apron;
	return std::nullopt;
}

bool is_aeroway_area(const ProcessedWay &way)
{
	return aeroway_area_kind(way).has_value();
}

std::optional<crate::block_definitions::Block> aeroway_area_block(const ProcessedWay &way)
{
	const auto kind = aeroway_area_kind(way);
	if (!kind)
		return std::nullopt;
	const auto surface = way.tags.get("surface");
	static const std::unordered_set<std::string> unpaved_surfaces{"grass", "dirt",
			"ground", "earth", "soil", "unpaved", "gravel", "fine_gravel", "compacted",
			"sand", "clay", "mud"};
	if (unpaved_surfaces.contains(surface))
		return std::nullopt;
	return *kind == AerowayAreaKind::Runway
				   ? crate::block_definitions::GRAY_CONCRETE
				   : crate::block_definitions::LIGHT_GRAY_CONCRETE;
}

void generate_helipad_node(WorldEditor &editor, const ProcessedNode &node,
		const Args &args, const CoordinateBitmap &building_footprints)
{
	paint_helipad_disc(editor, node.x, node.z, args, building_footprints);
}

void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints)
{
	if (way.tags.get("aeroway") != "helipad") {
		generate_aeroway_impl(editor, way, args, nullptr);
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

void generate_aeroway(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const CoordinateBitmap &building_footprints, const FloodFillCache &fill_cache)
{
	if (way.tags.get("aeroway") == "helipad") {
		generate_aeroway(editor, way, args, building_footprints);
		return;
	}
	generate_aeroway_impl(editor, way, args, &fill_cache);
}

crate::CoordinateBitmap collect_road_surface_coords(
		const std::vector<crate::osm_parser::ProcessedElement> &elements,
		const crate::world_editor::WorldEditor &editor, const ::XZBBox &xzbbox,
		double scale)
{
	crate::CoordinateBitmap bitmap(xzbbox);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway_type = surface_highway_type(way);
		if (!highway_type || !tunnel_renders_at_surface(way, &editor, scale))
			continue;

		const int block_range = highway_block_range(*highway_type, way.tags, scale);
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

crate::CoordinateBitmap collect_carriageway_coords(
		const std::vector<crate::osm_parser::ProcessedElement> &elements,
		const ::XZBBox &xzbbox, double scale)
{
	crate::CoordinateBitmap bitmap(xzbbox);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto highway = surface_highway_type(way);
		if (!highway || !is_carriageway(*highway) ||
				!tunnel_renders_at_surface(way, nullptr, scale))
			continue;

		const int block_range = highway_block_range(*highway, way.tags, scale);
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &prev = way.nodes[i - 1];
			const auto &cur = way.nodes[i];
			const auto points =
					crate::bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			for (const auto &point : points) {
				const int x = std::get<0>(point);
				const int z = std::get<2>(point);
				for (int dx = -block_range; dx <= block_range; ++dx)
					for (int dz = -block_range; dz <= block_range; ++dz)
						bitmap.set(x + dx, z + dz);
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
