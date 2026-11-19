#include "mobile_homes.h"

#include "area_layout.h"
#include "../block_definitions.h"
#include "../bresenham.h"
#include "../deterministic_rng.h"
#include "buildings.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <string_view>

namespace arnis::mobile_homes
{
namespace
{
using Cell = std::pair<int, int>;
namespace area_layout = element_processing::area_layout;
using area_layout::Frame;
constexpr std::string_view PARK_TAG = "arnis:mobile_home";
constexpr std::array<std::string_view, 4> MAPPED_TYPES{
		"static_caravan", "mobile_home", "trailer", "caravan"};
constexpr std::array<Cell, 4> SIDES{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

std::optional<std::string> tag(const tags_t &tags, std::string_view key)
{
	const auto found = tags.find(std::string(key));
	if (found == tags.end())
		return std::nullopt;
	return found->second;
}

bool overture_guess(const tags_t &tags)
{
	return tag(tags, "source") == "overture_maps" ||
		   tag(tags, "arnis:height_source") == "overture_maps";
}

bool one_storey(const tags_t &tags)
{
	if (overture_guess(tags))
		return true;
	const auto levels = tag(tags, "building:levels");
	if (!levels)
		return true;
	try {
		const auto first = levels->find_first_not_of(" \t\r\n");
		if (first == std::string::npos)
			return true;
		return std::stod(levels->substr(first)) < 2.0;
	} catch (...) {
		return true;
	}
}

bool named_as_park(std::string name)
{
	std::transform(name.begin(), name.end(), name.begin(),
			[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
	constexpr std::array<std::string_view, 11> phrases{"trailer", "mobile home",
			"mobile park", "mobile estate", "mobile village", "mobile court",
			"mobile manor", "mobile ranch", "mobile villa", "mobile community",
			"manufactured home"};
	for (auto phrase : phrases)
		if (name.find(phrase) != std::string::npos)
			return true;
	std::string word;
	for (unsigned char c : name) {
		if (std::isalnum(c)) {
			word.push_back(static_cast<char>(c));
		} else {
			if (word == "mhp")
				return true;
			word.clear();
		}
	}
	return word == "mhp";
}

bool could_be_one(const tags_t &tags)
{
	const auto building = tag(tags, "building");
	const bool plain =
			building && (*building == "yes" || *building == "house" ||
								*building == "residential" || *building == "detached" ||
								*building == "bungalow" || *building == "cabin" ||
								(*building == "apartments" &&
										tag(tags, "source") == "overture_maps"));
	for (auto key : {"amenity", "shop", "office", "craft", "leisure", "tourism",
				 "man_made", "building:part"})
		if (tags.contains(key))
			return false;
	bool low = overture_guess(tags);
	if (const auto height = tag(tags, "height")) {
		try {
			const auto first = height->find_first_not_of(" \t\r\n");
			std::string value =
					first == std::string::npos ? std::string{} : height->substr(first);
			if (value.ends_with("m"))
				value.pop_back();
			low = low || std::stod(value) <= 6.0;
		} catch (...) {
			low = true;
		}
	} else {
		low = true;
	}
	return plain && low && one_storey(tags);
}

double ring_area(const std::vector<ProcessedNode> &nodes)
{
	double twice = 0.0;
	for (std::size_t i = 1; i < nodes.size(); ++i)
		twice += static_cast<double>(nodes[i - 1].x) * nodes[i].z -
				 static_cast<double>(nodes[i].x) * nodes[i - 1].z;
	return std::abs(twice) * 0.5;
}

bool shaped_like_one(const std::vector<ProcessedNode> &nodes, double scale)
{
	const auto frame = Frame::of(nodes);
	if (!frame || scale <= 0.0)
		return false;
	const double length = frame->length() / scale;
	const double width = frame->width() / scale;
	const double area = ring_area(nodes) / (scale * scale);
	if (width < 2.5 || length > 30.0 || area < 0.75 * length * width)
		return false;
	if (width <= 6.0)
		return length >= 8.0 && length >= 2.4 * width;
	return width <= 12.5 && length <= 26.0 && length >= 1.3 * width && area <= 260.0;
}

struct Park
{
	area_layout::Bounds bounds{};
	std::vector<std::vector<ProcessedNode>> rings;
};

Cell snap(std::pair<double, double> v)
{
	return std::abs(v.first) >= std::abs(v.second) ? Cell{v.first >= 0.0 ? 1 : -1, 0}
												   : Cell{0, v.second >= 0.0 ? 1 : -1};
}

Cell step(Cell p, Cell d, int n = 1)
{
	return {p.first + d.first * n, p.second + d.second * n};
}

int floor_div(int value, int divisor)
{
	int q = value / divisor;
	if (value < 0 && value % divisor)
		--q;
	return q;
}

std::string facing(Cell n)
{
	if (n.first > 0)
		return "east";
	if (n.first < 0)
		return "west";
	if (n.second > 0)
		return "south";
	return "north";
}

BlockWithProperties prop(Block block, BlockProperties properties)
{
	return BlockWithProperties(block, std::move(properties));
}

template <typename T, std::size_t N>
T pick(ChaCha8Rng &rng, const std::array<std::pair<T, std::uint32_t>, N> &options)
{
	std::uint32_t total = 0;
	for (const auto &[value, weight] : options) {
		(void)value;
		total += weight;
	}
	auto roll = rng.uniform(total);
	for (const auto &[value, weight] : options) {
		if (roll < weight)
			return value;
		roll -= weight;
	}
	return options.front().first;
}

} // namespace

bool claims(const tags_t &tags)
{
	const auto building = tag(tags, "building");
	const bool mapped = building && std::find(MAPPED_TYPES.begin(), MAPPED_TYPES.end(),
											*building) != MAPPED_TYPES.end();
	return (mapped || tags.contains(std::string(PARK_TAG))) && one_storey(tags);
}

bool is_trailer_park(const tags_t &tags)
{
	const auto residential = tag(tags, "residential");
	if (residential &&
			(*residential == "trailer_park" || *residential == "mobile_home_park" ||
					*residential == "mobile_home" || *residential == "mobile"))
		return true;
	const bool area = tag(tags, "landuse") == "residential" ||
					  tag(tags, "place") == "neighbourhood";
	const auto name = tag(tags, "name");
	return area && name && named_as_park(*name);
}

void prepare(std::vector<ProcessedElement> &elements, double scale)
{
	constexpr int CELL_SIZE = 128;
	std::vector<Park> parks;
	for (const auto &element : elements) {
		if (!is_trailer_park(element.tags()))
			continue;
		Park park;
		if (element.is_way()) {
			park.rings.push_back(element.as_way().nodes);
		} else if (element.is_relation()) {
			for (const auto &member : element.as_relation().members)
				if (member.role != ProcessedMemberRole::Part && member.way)
					park.rings.push_back(member.way->nodes);
		} else {
			continue;
		}
		std::vector<ProcessedNode> all;
		for (const auto &ring : park.rings)
			all.insert(all.end(), ring.begin(), ring.end());
		const auto bounds = area_layout::bounds(all);
		if (!bounds)
			continue;
		park.bounds = *bounds;
		parks.push_back(std::move(park));
	}
	if (parks.empty())
		return;
	std::map<Cell, std::vector<std::size_t>> grid;
	for (std::size_t i = 0; i < parks.size(); ++i) {
		const auto &b = parks[i].bounds;
		const int x0 = floor_div(static_cast<int>(b[0]), CELL_SIZE);
		const int z0 = floor_div(static_cast<int>(b[1]), CELL_SIZE);
		const int x1 = floor_div(static_cast<int>(b[2]), CELL_SIZE);
		const int z1 = floor_div(static_cast<int>(b[3]), CELL_SIZE);
		for (int x = x0; x <= x1; ++x)
			for (int z = z0; z <= z1; ++z)
				grid[{x, z}].push_back(i);
	}
	for (auto &element : elements) {
		if (!element.is_way())
			continue;
		auto &way = element.as_way();
		if (way.nodes.size() < 4 || claims(way.tags) || !could_be_one(way.tags))
			continue;
		std::vector<ProcessedNode> open(way.nodes.begin(), way.nodes.end() - 1);
		const auto center = area_layout::centroid(open);
		const Cell key{floor_div(static_cast<int>(center.first), CELL_SIZE),
				floor_div(static_cast<int>(center.second), CELL_SIZE)};
		const auto found = grid.find(key);
		if (found == grid.end())
			continue;
		const bool in_park = std::any_of(
				found->second.begin(), found->second.end(), [&](std::size_t i) {
					std::vector<const std::vector<ProcessedNode> *> rings;
					for (const auto &ring : parks[i].rings)
						rings.push_back(&ring);
					return area_layout::inside(parks[i].bounds, center) &&
						   area_layout::point_in_rings(rings, center);
				});
		if (in_park && shaped_like_one(way.nodes, scale))
			way.tags[std::string(PARK_TAG)] = "yes";
	}
}

bool generate(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int32_t, int32_t>> &fill, int ground,
		const Args &args, const RoadMaskBitmap &roads,
		const BuildingFootprintBitmap &footprints)
{
	if (!claims(way.tags) || fill.empty())
		return false;
	const auto frame = Frame::of(way.nodes);
	if (!frame)
		return false;
	std::set<Cell> cells(fill.begin(), fill.end());
	for (std::size_t i = 1; i < way.nodes.size(); ++i)
		for (const auto &[x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
					 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
			(void)y;
			cells.emplace(x, z);
		}
	const Cell along = snap(frame->along);
	Cell front = frame->across_axis();
	if (front.first == along.first && front.second == along.second)
		front = {-front.first, -front.second};
	int length = static_cast<int>(std::lround(frame->length()));
	int width = static_cast<int>(std::lround(frame->width()));
	if (width < 3 || length < 5 || cells.size() < 15)
		return false;
	const double scale = std::max(0.1, args.scale);
	auto rng = element_rng(way.id ^ 0x7A11'E400'0000'0001ULL);
	const int floor = ground + std::max(1, static_cast<int>(std::lround(.7 * scale)));
	const int wall_top = floor + std::max(2, static_cast<int>(std::lround(2.4 * scale)));
	const int eave = wall_top + 1;
	auto has = [&](Cell p) { return cells.contains(p); };
	auto wall = [&](Cell p) {
		return std::any_of(
				SIDES.begin(), SIDES.end(), [&](Cell d) { return !has(step(p, d)); });
	};
	const Cell back_direction{-front.first, -front.second};
	const int road_reach = width + static_cast<int>(std::lround(12.0 * scale));
	auto road_distance = [&](Cell direction) {
		int best = std::numeric_limits<int>::max();
		for (const auto c : cells) {
			if (!wall(c) || has(step(c, direction)))
				continue;
			for (int k = 1; k <= road_reach; ++k)
				if (roads.contains(c.first + direction.first * k,
							c.second + direction.second * k)) {
					best = std::min(best, k);
					break;
				}
		}
		return best;
	};
	const int front_road = road_distance(front);
	const int back_road = road_distance(back_direction);
	if (back_road < front_road || (back_road == front_road && rng.random_bool(.5))) {
		front = back_direction;
	}
	constexpr std::array<double, 4> door_shares{.3, .35, .65, .7};
	const int door_at = std::clamp(
			static_cast<int>(std::lround(
					length * door_shares[rng.uniform(
									 static_cast<std::uint32_t>(door_shares.size()))])),
			2, std::max(2, length - 3));
	std::optional<Cell> door_cell;
	int door_distance = std::numeric_limits<int>::max();
	for (const Cell c : cells) {
		if (!wall(c) || has(step(c, front)))
			continue;
		const int at =
				static_cast<int>(std::lround(frame->local(c.first, c.second).first));
		const int distance = std::abs(at - door_at);
		if (distance < door_distance) {
			door_distance = distance;
			door_cell = c;
		}
	}
	const Block siding = pick(
			rng, std::array<std::pair<Block, std::uint32_t>, 9>{
						 {{WHITE_CONCRETE, 20}, {SMOOTH_QUARTZ, 8}, {BIRCH_PLANKS, 14},
								 {SMOOTH_SANDSTONE, 12}, {WHITE_TERRACOTTA, 8},
								 {END_STONE_BRICKS, 6}, {LIGHT_BLUE_TERRACOTTA, 9},
								 {LIGHT_GRAY_CONCRETE, 10}, {CALCITE, 5}}});
	std::optional<Block> band;
	if (rng.random_bool(.15)) {
		const Block selected = pick(
				rng, std::array<std::pair<Block, std::uint32_t>, 4>{
							 {{LIGHT_BLUE_TERRACOTTA, 3}, {GRAY_TERRACOTTA, 2},
									 {BROWN_TERRACOTTA, 2}, {LIGHT_GRAY_CONCRETE, 2}}});
		if (selected != siding)
			band = selected;
	}
	Block skirting;
	do {
		skirting = pick(rng,
				std::array<std::pair<Block, std::uint32_t>, 7>{
						{{STONE_BRICKS, 5}, {LIGHT_GRAY_TERRACOTTA, 4},
								{GRAY_TERRACOTTA, 3}, {POLISHED_ANDESITE, 3},
								{WHITE_TERRACOTTA, 2}, {BRICK, 2}, {COBBLESTONE, 2}}});
	} while (skirting == siding);
	const auto roof = pick(
			rng, std::array<std::pair<std::pair<Block, Block>, std::uint32_t>, 4>{
						 {{{SMOOTH_STONE, SMOOTH_STONE_SLAB}, 35},
								 {{SMOOTH_QUARTZ, SMOOTH_QUARTZ_SLAB}, 20},
								 {{POLISHED_ANDESITE, POLISHED_ANDESITE_SLAB}, 25},
								 {{COBBLED_DEEPSLATE, COBBLED_DEEPSLATE_SLAB}, 20}}});
	const Block roof_block = roof.first, roof_slab = roof.second;
	const Block door = pick(rng, std::array<std::pair<Block, std::uint32_t>, 2>{
										 {{BIRCH_DOOR, 5}, {DARK_OAK_DOOR_LOWER, 4}}});
	const Block glass = pick(rng, std::array<std::pair<Block, std::uint32_t>, 2>{
										  {{GLASS, 4}, {LIGHT_GRAY_STAINED_GLASS, 1}}});
	std::optional<Block> shutters;
	if (rng.random_bool(.35))
		shutters = pick(rng,
				std::array<std::pair<Block, std::uint32_t>, 3>{{{DARK_OAK_TRAPDOOR, 3},
						{SPRUCE_TRAPDOOR, 2}, {IRON_TRAPDOOR, 1}}});
	const auto deck_material =
			pick(rng, std::array<std::pair<std::pair<Block, Block>, std::uint32_t>, 2>{
							  {{{OAK_PLANKS, OAK_STAIRS}, 3},
									  {{SPRUCE_PLANKS, SPRUCE_STAIRS}, 2}}});
	const Block planks = deck_material.first, stair_block = deck_material.second;
	const bool peaked = rng.random_bool(.35);
	const Cell back{-front.first, -front.second};
	const int period = std::max(3, static_cast<int>(std::lround(3.5 * scale)));
	const int pane = rng.random_bool(.5) ? 1 : 2;
	const int front_phase =
			static_cast<int>(rng.uniform(static_cast<std::uint32_t>(period)));
	const int back_phase =
			static_cast<int>(rng.uniform(static_cast<std::uint32_t>(period)));
	const bool rear_door = rng.random_bool(.4);
	const int rear_at = length - door_at;
	std::optional<Cell> rear_door_cell;
	if (rear_door) {
		int best = std::numeric_limits<int>::max();
		for (const Cell c : cells) {
			if (!wall(c) || has(step(c, back)))
				continue;
			const int at =
					static_cast<int>(std::lround(frame->local(c.first, c.second).first));
			if (std::abs(at - rear_at) < best) {
				best = std::abs(at - rear_at);
				rear_door_cell = c;
			}
		}
	}
	const int window_low = floor + std::max(1, static_cast<int>(std::lround(scale))) + 1;
	const int window_high = wall_top - static_cast<int>(std::lround(0.4 * scale));
	std::set<Cell> glazed;
	for (const Cell c : cells) {
		if (!wall(c)) {
			editor.set_block_absolute(
					planks, c.first, floor, c.second, std::nullopt, std::nullopt);
		} else {
			const int base =
					args.terrain_enabled()
							? editor.terrain_level(c.first, c.second).value_or(ground)
							: ground;
			for (int y = std::min(base, floor); y <= floor; ++y)
				editor.set_block_absolute(
						skirting, c.first, y, c.second, std::nullopt, std::nullopt);
			const auto local = frame->local(c.first, c.second);
			const int a = static_cast<int>(std::lround(local.first));
			const int across = static_cast<int>(std::lround(local.second));
			const bool front_wall = !has(step(c, front));
			const bool back_wall = !has(step(c, back));
			const bool end_wall =
					!has(step(c, along)) || !has(step(c, {-along.first, -along.second}));
			const bool front_door = door_cell && c == *door_cell;
			const bool rear_entry = rear_door_cell && c == *rear_door_cell;
			const bool door_here = front_door || rear_entry;
			const int nearest_door = front_door ? door_at : rear_at;
			const bool long_window =
					(front_wall || back_wall) && a > 1 && a < length - 1 &&
					((a - (front_wall ? front_phase : back_phase) + period) % period <
							pane) &&
					std::abs(a - nearest_door) > 1;
			const bool is_window =
					!door_here && (long_window || (end_wall && (across == width / 2)));
			if (is_window)
				glazed.insert(c);
			for (int y = floor + 1; y <= wall_top; ++y) {
				Block block = is_window && y >= window_low && y <= window_high
									  ? glass
									  : (y == floor + 1 && band ? *band : siding);
				if (door_here && (y == floor + 1 || y == floor + 2)) {
					const bool lower = y == floor + 1;
					const Cell outward = rear_entry ? back : front;
					const BlockWithProperties state = prop(door,
							{{"half", lower ? "lower" : "upper"},
									{"facing", facing(outward)}, {"hinge", "left"}});
					editor.set_block_with_properties_absolute(
							state, c.first, y, c.second, std::nullopt, std::nullopt);
				} else {
					editor.set_block_absolute(
							block, c.first, y, c.second, std::nullopt, std::nullopt);
				}
			}
		}
		const auto local = frame->local(c.first, c.second);
		const double depth =
				std::max(0.0, std::min(local.second, frame->width() - local.second)) /
				scale;
		const double middle = frame->width() / (2.0 * scale);
		const bool double_wide = frame->width() / scale > 6.5;
		const double steps = double_wide
									 ? std::clamp(std::floor((middle - .5) / 2.0), 0.0,
											   std::max(2.0, std::round(2.0 * scale)))
									 : (peaked && middle >= 1.75 ? 1.0 : 0.0);
		const double run = double_wide ? 2.0 : (peaked && middle >= 1.75 ? .5 : 1.0);
		const int roof_rows =
				depth < .5 ? 1
						   : 2 + static_cast<int>(std::max(0.0,
										 steps - std::floor((middle - depth) / run)));
		for (int k = 0; k < roof_rows / 2; ++k)
			editor.set_block_absolute(
					roof_block, c.first, eave + k, c.second, std::nullopt, std::nullopt);
		if (roof_rows % 2)
			editor.set_block_absolute(roof_slab, c.first, eave + roof_rows / 2, c.second,
					std::nullopt, std::nullopt);
	}
	if (shutters) {
		const BlockWithProperties shutter(prop(*shutters,
				{{"facing", facing(front)}, {"open", "true"}, {"half", "top"}}));
		const Cell along_direction = along;
		for (const Cell window : glazed) {
			if (has(step(window, front)))
				continue;
			for (const Cell tangent : {along_direction,
						 Cell{-along_direction.first, -along_direction.second}}) {
				const Cell side = step(window, tangent);
				if (!has(side) || !wall(side) || has(step(side, front)) ||
						(door_cell && side == *door_cell) ||
						(rear_door_cell && side == *rear_door_cell) ||
						!footprints.contains(side.first, side.second))
					continue;
				const Cell outside = step(side, front);
				if (roads.contains(outside.first, outside.second) ||
						footprints.contains(outside.first, outside.second) ||
						editor.is_mapped_water(outside.first, outside.second))
					continue;
				for (int y = window_low; y <= window_high; ++y)
					editor.set_block_with_properties_absolute(shutter, outside.first, y,
							outside.second, std::nullopt, std::nullopt);
			}
		}
	}
	// Provide a short wooden approach from the front door only when it is clear
	// of mapped footprints, roads and water. This keeps the unit distinct while
	// avoiding the original building footprint and utility masks.
	for (const Cell c : cells) {
		if (!wall(c) || has(step(c, front)))
			continue;
		if (!door_cell || c != *door_cell)
			continue;
		for (int k = 1; k <= 3; ++k) {
			const Cell p = step(c, front, k);
			if (cells.contains(p) || footprints.contains(p.first, p.second) ||
					roads.contains(p.first, p.second) ||
					editor.is_mapped_water(p.first, p.second))
				break;
			const int y = floor - k;
			if (editor.terrain_level(p.first, p.second).value_or(ground) >= y)
				break;
			const BlockWithProperties stair = create_stair_with_properties(stair_block,
					front == Cell{1, 0}	   ? StairFacing::West
					: front == Cell{-1, 0} ? StairFacing::East
					: front == Cell{0, 1}  ? StairFacing::North
										   : StairFacing::South,
					StairShape::Straight);
			editor.set_block_with_properties_absolute(
					stair, p.first, y, p.second, std::nullopt, std::nullopt);
		}
		break;
	}
	return true;
}

} // namespace arnis::mobile_homes
