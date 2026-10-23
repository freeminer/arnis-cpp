#include "waterfront.h"

#include "../block_definitions.h"
#include "../bresenham.h"
#include "../mapillary/geometry.h"
#include "../world_editor/floor_state.h"
#include "connected_blocks.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace arnis::waterfront
{
namespace
{
using Cell = std::pair<int, int>;
constexpr double PIER_WIDTH_M = 3.0;
constexpr double PATH_ON_PIER_SHARE = 0.75;
constexpr int PIER_MAX_RISE = 3;
constexpr int PIER_RAILING_RISE = 3;
constexpr double PILE_SPACING_M = 4.0;
constexpr int PIER_LAND_END_REACH = 3;
constexpr std::size_t LANTERN_EVERY = 3;
constexpr double MOORING_POST_MAX_WIDTH = 4.0;
constexpr double BREAKWATER_WIDTH_M = 5.0;
constexpr double GROYNE_WIDTH_M = 3.0;
constexpr int BREAKWATER_MAX_HEIGHT = 6;

struct CellHash
{
	std::size_t operator()(const Cell &p) const noexcept
	{
		const auto key = (std::uint64_t(static_cast<std::uint32_t>(p.first)) << 32) |
						 static_cast<std::uint32_t>(p.second);
		return std::hash<std::uint64_t>{}(key);
	}
};

struct FootCell
{
	int x;
	int z;
	bool edge;
};

std::optional<std::string> tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? std::nullopt : std::optional<std::string>(it->second);
}

double parsed_width(const tags_t &tags, double default_m, double scale)
{
	const auto raw = tag(tags, "width");
	const auto metres = raw ? mapillary::geodesy::parse_length_m(*raw) : std::nullopt;
	const double width_m =
			metres && *metres > 0.0 && *metres < 200.0 ? *metres : default_m;
	return std::max(1.0, width_m * scale);
}

bool is_area(const ProcessedWay &way)
{
	return way.nodes.size() > 3 && way.nodes.front().x == way.nodes.back().x &&
		   way.nodes.front().z == way.nodes.back().z && tag(way.tags, "area") != "no";
}

void sort_cells(std::vector<FootCell> &cells)
{
	std::sort(cells.begin(), cells.end(), [](const auto &a, const auto &b) {
		return std::tie(a.x, a.z) < std::tie(b.x, b.z);
	});
}

std::vector<FootCell> line_footprint(const ProcessedWay &way, double width)
{
	std::unordered_map<Cell, double, CellHash> nearest;
	const double half = width / 2.0;
	const int reach = static_cast<int>(std::ceil(half)) + 1;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const double dx = b.x - a.x, dz = b.z - a.z;
		const double len2 = std::max(1e-9, dx * dx + dz * dz);
		const bool x_major = std::abs(dx) >= std::abs(dz);
		const double major_a = x_major ? a.x : a.z;
		const double minor_a = x_major ? a.z : a.x;
		const double d_major = x_major ? dx : dz;
		const double d_minor = x_major ? dz : dx;
		const double slope = std::abs(d_major) > 1e-9 ? d_minor / d_major : 0.0;
		const int band = static_cast<int>(std::ceil(half * (1.0 + std::abs(slope)))) + 1;
		const int m0 = static_cast<int>(std::min(major_a, major_a + d_major)) - reach;
		const int m1 = static_cast<int>(std::max(major_a, major_a + d_major)) + reach;
		for (int m = m0; m <= m1; ++m) {
			const double along = std::abs(d_major) > 1e-9
										 ? std::clamp((m - major_a) / d_major, 0.0, 1.0)
										 : 0.0;
			const int centre = static_cast<int>(std::round(minor_a + along * d_minor));
			for (int n = centre - band; n <= centre + band; ++n) {
				const int x = x_major ? m : n, z = x_major ? n : m;
				const double t =
						std::clamp(((x - a.x) * dx + (z - a.z) * dz) / len2, 0.0, 1.0);
				const double ex = x - a.x - t * dx, ez = z - a.z - t * dz;
				const double d = std::sqrt(ex * ex + ez * ez);
				if (d <= half) {
					auto [it, inserted] = nearest.emplace(Cell{x, z}, d);
					if (!inserted)
						it->second = std::min(it->second, d);
				}
			}
		}
	}
	std::vector<FootCell> cells;
	cells.reserve(nearest.size());
	for (const auto &[p, distance] : nearest)
		cells.push_back({p.first, p.second, distance > half - 1.0});
	sort_cells(cells);
	return cells;
}

std::unordered_set<Cell, CellHash> area_cells(
		const ProcessedElement &element, const FloodFillCache &cache, const Args &args)
{
	std::unordered_set<Cell, CellHash> result;
	auto add_way = [&](const ProcessedWay &way, bool subtract) {
		const auto fill = cache.get_or_compute(way, args.timeout);
		for (const auto &p : *fill) {
			if (subtract)
				result.erase({p.first, p.second});
			else
				result.insert({p.first, p.second});
		}
		if (!subtract && !fill->empty()) {
			for (std::size_t i = 1; i < way.nodes.size(); ++i)
				for (const auto &[x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
							 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
					(void)y;
					result.insert({x, z});
				}
		}
	};
	if (element.is_way()) {
		add_way(element.as_way(), false);
	} else if (element.is_relation()) {
		for (const auto &member : element.as_relation().members)
			if (member.role == ProcessedMemberRole::Outer && member.way)
				add_way(*member.way, false);
		for (const auto &member : element.as_relation().members)
			if (member.role == ProcessedMemberRole::Inner && member.way)
				add_way(*member.way, true);
	}
	return result;
}

std::vector<FootCell> area_footprint(
		const ProcessedElement &element, const FloodFillCache &cache, const Args &args)
{
	auto inside = area_cells(element, cache, args);
	std::vector<FootCell> cells;
	cells.reserve(inside.size());
	for (const auto &[x, z] : inside) {
		const bool edge = !inside.contains({x + 1, z}) || !inside.contains({x - 1, z}) ||
						  !inside.contains({x, z + 1}) || !inside.contains({x, z - 1});
		cells.push_back({x, z, edge});
	}
	sort_cells(cells);
	return cells;
}

std::vector<FootCell> footprint(const ProcessedElement &element,
		const FloodFillCache &cache, const Args &args, double default_width)
{
	if (element.is_way()) {
		const auto &way = element.as_way();
		if (way.nodes.size() < 2)
			return {};
		if (!is_area(way))
			return line_footprint(way, parsed_width(way.tags, default_width, args.scale));
	} else if (!element.is_relation()) {
		return {};
	}
	return area_footprint(element, cache, args);
}

void drive_pile(
		world_editor::WorldEditor &editor, int x, int z, int top, const Block &block)
{
	const int ground = editor.get_ground_level(x, z);
	if (top >= ground + 1)
		editor.fill_column_absolute(block, x, z, ground + 1, top, false);
	editor.register_support_column(x, z, block);
	for (int y = std::min(ground, top); y > world_editor::min_y(); --y) {
		if (!editor.check_for_block_absolute(x, y, z,
					std::optional<std::vector<Block>>{
							std::vector<Block>{block_definitions::WATER}}))
			break;
		editor.set_block_absolute(block, x, y, z,
				std::optional<std::vector<Block>>{
						std::vector<Block>{block_definitions::WATER}},
				std::nullopt);
	}
}

struct PierMaterials
{
	Block deck;
	Block pile;
	Block railing;
};

std::vector<std::pair<double, double>> stations(
		const std::vector<const ProcessedWay *> &lines, double spacing)
{
	std::vector<std::pair<double, double>> out;
	for (const auto *way : lines) {
		if (!way)
			continue;
		double carry = 0.0;
		for (std::size_t i = 1; i < way->nodes.size(); ++i) {
			const double ax = way->nodes[i - 1].x, az = way->nodes[i - 1].z;
			const double dx = double(way->nodes[i].x) - ax;
			const double dz = double(way->nodes[i].z) - az;
			const double length = std::hypot(dx, dz);
			if (length <= 0.0)
				continue;
			double distance = carry;
			for (; distance < length; distance += spacing)
				out.emplace_back(
						ax + dx * distance / length, az + dz * distance / length);
			// Preserve the distance from this segment's end to the next station.
			// This matches Rust's `s - len`; `(carry + length) % spacing`
			// shifts station placement whenever the last sample overshoots an end.
			carry = distance - length;
		}
		if (!way->nodes.empty())
			out.emplace_back(way->nodes.back().x, way->nodes.back().z);
	}
	return out;
}

std::pair<double, double> nearest_heading(const ProcessedWay &way, double x, double z)
{
	double best_distance = std::numeric_limits<double>::max();
	std::pair<double, double> best{1.0, 0.0};
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const double ax = way.nodes[i - 1].x, az = way.nodes[i - 1].z;
		const double dx = double(way.nodes[i].x) - ax;
		const double dz = double(way.nodes[i].z) - az;
		const double length_squared = dx * dx + dz * dz;
		if (length_squared <= 0.0)
			continue;
		const double t =
				std::clamp(((x - ax) * dx + (z - az) * dz) / length_squared, 0.0, 1.0);
		const double ex = x - ax - t * dx, ez = z - az - t * dz;
		const double distance = ex * ex + ez * ez;
		if (distance < best_distance) {
			const double length = std::sqrt(length_squared);
			best_distance = distance;
			best = {dx / length, dz / length};
		}
	}
	return best;
}

PierMaterials pier_materials(const tags_t &tags)
{
	const auto solid = [](const std::optional<std::string> &value) {
		if (!value)
			return false;
		return *value == "concrete" || *value == "asphalt" || *value == "paved" ||
			   *value == "paving_stones" || *value == "sett" || *value == "stone" ||
			   *value == "reinforced_concrete";
	};
	const bool stone = solid(tag(tags, "surface")) || solid(tag(tags, "material"));
	return stone ? PierMaterials{block_definitions::SMOOTH_STONE,
						   block_definitions::POLISHED_ANDESITE,
						   block_definitions::IRON_BARS}
				 : PierMaterials{block_definitions::SPRUCE_PLANKS,
						   block_definitions::SPRUCE_LOG,
						   block_definitions::SPRUCE_FENCE};
}

int rem_euclid(int value, int modulus)
{
	const int remainder = value % modulus;
	return remainder < 0 ? remainder + modulus : remainder;
}

Block rubble(int x, int z)
{
	const auto h = static_cast<std::uint32_t>(x) * 73856093u ^
				   static_cast<std::uint32_t>(z) * 19349663u;
	switch (h % 10) {
	case 0:
	case 1:
	case 2:
	case 3:
		return block_definitions::STONE;
	case 4:
	case 5:
	case 6:
		return block_definitions::COBBLESTONE;
	case 7:
	case 8:
		return block_definitions::ANDESITE;
	default:
		return block_definitions::MOSSY_COBBLESTONE;
	}
}
} // namespace

void generate_pier(world_editor::WorldEditor &editor, const ProcessedElement &element,
		const FloodFillCache &cache, const Args &args)
{
	std::vector<const ProcessedWay *> outline;
	if (element.is_way() && element.as_way().nodes.size() >= 2)
		outline.push_back(&element.as_way());
	else if (element.is_relation())
		for (const auto &member : element.as_relation().members)
			if (member.role == ProcessedMemberRole::Outer && member.way)
				outline.push_back(member.way.get());
	const auto cells = footprint(element, cache, args, PIER_WIDTH_M);
	if (cells.empty())
		return;
	const bool line = element.is_way() && !is_area(element.as_way());
	const bool floating = tag(element.tags(), "floating") == "yes";
	const bool lit = tag(element.tags(), "lit") == "yes";
	const auto materials = pier_materials(element.tags());
	std::vector<int> grounds;
	grounds.reserve(cells.size());
	int base = std::numeric_limits<int>::max(), high = std::numeric_limits<int>::min();
	for (const auto &cell : cells) {
		const int ground = editor.get_ground_level(cell.x, cell.z);
		grounds.push_back(ground);
		base = std::min(base, ground);
		high = std::max(high, ground);
	}
	const int deck_y =
			floating ? base + 1 : base + 1 + std::clamp(high - base, 1, PIER_MAX_RISE);
	const bool railing = !floating && deck_y - base >= PIER_RAILING_RISE;
	const auto deck_at = [deck_y](int ground) { return std::max(deck_y, ground + 1); };
	for (std::size_t i = 0; i < cells.size(); ++i)
		editor.set_block_absolute(
				materials.deck, cells[i].x, deck_at(grounds[i]), cells[i].z);
	if (floating)
		return;

	std::unordered_map<Cell, std::pair<bool, int>, CellHash> footprint_by_cell;
	footprint_by_cell.reserve(cells.size());
	for (std::size_t i = 0; i < cells.size(); ++i)
		footprint_by_cell.emplace(
				Cell{cells[i].x, cells[i].z}, std::pair{cells[i].edge, grounds[i]});
	const double width = parsed_width(element.tags(), PIER_WIDTH_M, args.scale);
	const bool posts_above = line && width <= MOORING_POST_MAX_WIDTH;
	const double spacing = PILE_SPACING_M * std::max(0.25, args.scale);
	const double edge_inset = line ? std::max(width / 2.0 - 0.5, 0.0) : 0.0;
	std::vector<Cell> edge_piles;
	for (const auto &[sx, sz] : stations(outline, spacing)) {
		if (line && !outline.empty()) {
			const auto [dx, dz] = nearest_heading(*outline.front(), sx, sz);
			for (const double sign : {1.0, -1.0})
				edge_piles.emplace_back(
						static_cast<int>(std::lround(sx - dz * edge_inset * sign)),
						static_cast<int>(std::lround(sz + dx * edge_inset * sign)));
		} else {
			const int x = static_cast<int>(std::lround(sx));
			const int z = static_cast<int>(std::lround(sz));
			const std::array<Cell, 5> candidates = {Cell{x, z}, Cell{x + 1, z},
					Cell{x - 1, z}, Cell{x, z + 1}, Cell{x, z - 1}};
			for (const auto &candidate : candidates) {
				const auto it = footprint_by_cell.find(candidate);
				if (it != footprint_by_cell.end() && it->second.first) {
					edge_piles.push_back(candidate);
					break;
				}
			}
		}
	}
	edge_piles.erase(std::unique(edge_piles.begin(), edge_piles.end()), edge_piles.end());
	std::size_t lanterns = 0;
	for (const auto &[x, z] : edge_piles) {
		const auto it = footprint_by_cell.find({x, z});
		if (it == footprint_by_cell.end())
			continue;
		const int y = deck_at(it->second.second);
		if (y - it->second.second < 2)
			continue;
		drive_pile(editor, x, z, posts_above ? y + 1 : y - 1, materials.pile);
		if (lit && posts_above) {
			if (lanterns % LANTERN_EVERY == 0)
				editor.set_block_absolute(block_definitions::LANTERN, x, y + 2, z);
			++lanterns;
		}
	}
	if (!line) {
		const int grid = std::max(2, static_cast<int>(std::lround(spacing)));
		for (std::size_t i = 0; i < cells.size(); ++i) {
			const auto &cell = cells[i];
			if (!cell.edge && rem_euclid(cell.x, grid) == 0 &&
					rem_euclid(cell.z, grid) == 0 &&
					deck_at(grounds[i]) - grounds[i] >= 2)
				drive_pile(
						editor, cell.x, cell.z, deck_at(grounds[i]) - 1, materials.pile);
		}
	}

	if (!railing)
		return;
	std::vector<Cell> land_ends;
	if (line && !outline.empty() && !outline.front()->nodes.empty()) {
		const auto &way = *outline.front();
		for (const auto *endpoint : {&way.nodes.front(), &way.nodes.back()}) {
			bool meets_land = false;
			for (int dx = -PIER_LAND_END_REACH; dx <= PIER_LAND_END_REACH && !meets_land;
					dx++)
				for (int dz = -PIER_LAND_END_REACH; dz <= PIER_LAND_END_REACH; dz++)
					if (editor.get_ground_level(endpoint->x + dx, endpoint->z + dz) >=
							deck_y - 1) {
						meets_land = true;
						break;
					}
			if (meets_land)
				land_ends.emplace_back(endpoint->x, endpoint->z);
		}
	}
	const double gate_distance = std::pow(width / 2.0 + 1.0, 2.0);
	lanterns = 0;
	for (std::size_t i = 0; i < cells.size(); ++i) {
		const auto &cell = cells[i];
		const int ground = grounds[i];
		const int y = deck_at(ground);
		const bool at_land_end =
				std::any_of(land_ends.begin(), land_ends.end(), [&](const Cell &end) {
					const auto dx = std::int64_t(cell.x) - end.first;
					const auto dz = std::int64_t(cell.z) - end.second;
					return double(dx * dx + dz * dz) <= gate_distance;
				});
		if (!cell.edge || y - ground < 2 || at_land_end)
			continue;
		connected_blocks::place_connected(
				editor, materials.railing, cell.x, y + 1, cell.z);
		if (lit && std::find(edge_piles.begin(), edge_piles.end(),
						   Cell{cell.x, cell.z}) != edge_piles.end()) {
			if (lanterns % LANTERN_EVERY == 0)
				editor.set_block_absolute(
						block_definitions::LANTERN, cell.x, y + 2, cell.z);
			++lanterns;
		}
	}
}

void generate_breakwater(world_editor::WorldEditor &editor,
		const ProcessedElement &element, const FloodFillCache &cache, const Args &args)
{
	const auto type = tag(element.tags(), "man_made");
	const bool groyne = type == "groyne";
	const bool wood = tag(element.tags(), "material") == "wood";
	const double default_width = wood	  ? 1.0
								 : groyne ? GROYNE_WIDTH_M
										  : BREAKWATER_WIDTH_M;
	const auto cells = footprint(element, cache, args, default_width);
	if (cells.empty())
		return;
	int base = std::numeric_limits<int>::max();
	for (const auto &cell : cells)
		base = std::min(base, editor.get_ground_level(cell.x, cell.z));
	int rise = 0;
	if (tag(element.tags(), "submerged") != "yes") {
		if (const auto h = tag(element.tags(), "height")) {
			const auto metres = mapillary::geodesy::parse_length_m(*h);
			if (metres)
				rise = std::clamp(static_cast<int>(std::round(*metres * args.scale)), 1,
						BREAKWATER_MAX_HEIGHT);
			else
				rise = groyne ? 1 : 2;
		} else {
			rise = groyne ? 1 : 2;
		}
	}
	for (const auto &cell : cells) {
		const int ground = editor.get_ground_level(cell.x, cell.z);
		// Rust uses rem_euclid here. Keep the same stagger west/north of the
		// origin too; C++'s signed remainder is negative for negative coordinates.
		const auto coordinate_sum = static_cast<std::int64_t>(cell.x) + cell.z;
		const bool staggered = ((coordinate_sum % 3) + 3) % 3 == 0;
		const int uneven = rise > 0 && staggered ? 1 : 0;
		if (wood) {
			drive_pile(editor, cell.x, cell.z, base + rise + uneven,
					block_definitions::SPRUCE_LOG);
		} else {
			const int top = base + rise - (cell.edge && rise > 1 ? 1 : 0);
			if (top > ground || ground <= base)
				drive_pile(editor, cell.x, cell.z, std::max(top, ground),
						rubble(cell.x, cell.z));
		}
	}
}

void generate_dolphin(world_editor::WorldEditor &editor, const ProcessedNode &node)
{
	const int top = editor.get_ground_level(node.x, node.z) + 2;
	for (const auto &[dx, dz] : {Cell{0, 0}, Cell{1, 0}, Cell{0, 1}, Cell{1, 1}})
		drive_pile(editor, node.x + dx, node.z + dz, top, block_definitions::SPRUCE_LOG);
}

std::size_t drop_paths_on_piers(std::vector<ProcessedElement> &elements, double scale,
		const FloodFillCache &cache,
		const std::optional<std::chrono::milliseconds> &timeout)
{
	std::unordered_set<Cell, CellHash> deck;
	Args fill_args;
	fill_args.scale = scale;
	fill_args.timeout = timeout;
	for (const auto &element : elements) {
		if (tag(element.tags(), "man_made") != "pier")
			continue;
		if (element.is_way() && !is_area(element.as_way())) {
			const auto cells = line_footprint(
					element.as_way(), parsed_width(element.tags(), PIER_WIDTH_M, scale));
			for (const auto &cell : cells)
				deck.insert({cell.x, cell.z});
		} else if (element.is_way() || element.is_relation()) {
			const auto cells = area_cells(element, cache, fill_args);
			deck.insert(cells.begin(), cells.end());
		}
	}
	if (deck.empty())
		return 0;
	const auto walkway = [](const ProcessedWay &way) {
		const auto highway = tag(way.tags, "highway");
		return highway &&
			   (*highway == "footway" || *highway == "path" || *highway == "pedestrian" ||
					   *highway == "cycleway" || *highway == "steps" ||
					   *highway == "bridleway" || *highway == "corridor") &&
			   tag(way.tags, "man_made") != "pier";
	};
	auto it = std::remove_if(elements.begin(), elements.end(), [&](const auto &element) {
		if (!element.is_way() || !walkway(element.as_way()))
			return false;
		const auto &way = element.as_way();
		std::size_t total = 0, on_deck = 0;
		for (std::size_t i = 1; i < way.nodes.size(); ++i)
			for (const auto &[x, y, z] : bresenham_line(way.nodes[i - 1].x, 0,
						 way.nodes[i - 1].z, way.nodes[i].x, 0, way.nodes[i].z)) {
				(void)y;
				++total;
				on_deck += deck.contains({x, z});
			}
		return total > 0 && static_cast<double>(on_deck) >= total * PATH_ON_PIER_SHARE;
	});
	const auto removed = static_cast<std::size_t>(std::distance(it, elements.end()));
	elements.erase(it, elements.end());
	return removed;
}

} // namespace arnis::waterfront
