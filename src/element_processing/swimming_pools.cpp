#include "swimming_pools.h"

#include "sport_pitches.h"
#include "../bresenham.h"
#include "../ground_decoration.h"
#include "../mapillary/geometry.h"
#include "../water_depth.h"
#include "world_editor.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>
#include <unordered_set>

namespace arnis::swimming_pools
{
const Block COPING = SMOOTH_QUARTZ;
const Block LINING = LIGHT_BLUE_CONCRETE;
namespace
{
const Block LANE_LINE = BLUE_CONCRETE;
const std::array<Block, 7> NATURAL_GROUND = {
		GRASS_BLOCK, DIRT, COARSE_DIRT, PODZOL, MUD, SAND, GRAVEL};
constexpr double DEFAULT_DEPTH_M = 2.0;
constexpr double PLUNGE_POOL_M2 = 12.0;
constexpr int MAX_DEPTH = 5;
using Cell = std::pair<int, int>;
using Cells = std::unordered_set<Cell, world_editor::WorldEditor::XZCellHash>;

bool tree_part(const WorldEditor &editor, int x, int y, int z)
{
	return water_depth::is_tree_block(editor.block_name_absolute(x, y, z));
}

bool under_floor(const WorldEditor &editor, const BuildingFootprintBitmap &footprints,
		int x, int z)
{
	if (!footprints.contains(x, z))
		return false;
	const auto block = editor.get_block_absolute(x, editor.get_ground_level(x, z), z);
	return block && std::find(NATURAL_GROUND.begin(), NATURAL_GROUND.end(), *block) ==
							NATURAL_GROUND.end();
}

void place_pool_block(WorldEditor &editor, Block block, int x, int y, int z, int ground_y)
{
	if (y <= ground_y) {
		editor.set_block_absolute(block, x, y, z, std::nullopt,
				std::optional<const std::vector<Block>>(std::vector<Block>{}));
		return;
	}
	const auto existing = editor.get_block_absolute(x, y, z);
	if (!existing) {
		editor.set_block_absolute(block, x, y, z);
		return;
	}
	if (tree_part(editor, x, y, z)) {
		water_depth::clear_tree_from(editor, x, y, z);
		editor.set_block_absolute(block, x, y, z);
		return;
	}
	const auto &loose = ground_decoration::loose_plant_blocks();
	const auto &stacked = ground_decoration::stacked_plant_parts();
	if (std::find(loose.begin(), loose.end(), *existing) != loose.end() ||
			std::find(stacked.begin(), stacked.end(), *existing) != stacked.end() ||
			*existing == COPING || *existing == LINING || *existing == LANE_LINE)
		editor.set_block_absolute(block, x, y, z, std::nullopt,
				std::optional<const std::vector<Block>>(std::vector<Block>{}));
}

int pool_depth(const tags_t &tags, std::size_t cell_count, double scale)
{
	std::optional<double> tagged;
	const auto value = tags.get("depth");
	if (!value.empty())
		tagged = mapillary::geodesy::parse_length_m(value);
	if (tagged && (!std::isfinite(*tagged) || *tagged <= 0.0))
		tagged.reset();
	double metres = tagged.value_or(DEFAULT_DEPTH_M);
	const auto kind = tags.get("swimming_pool");
	if (!tagged) {
		if (kind == "wading" || kind == "paddling" || kind == "kiddie" ||
				kind == "toddler" || kind == "children")
			metres = 0.5;
		else if (kind == "diving")
			metres = 4.0;
		else if (static_cast<double>(cell_count) < PLUNGE_POOL_M2 * scale * scale)
			metres = 1.0;
	}
	// Rust rounds the positive product and then casts to i32 (saturating for
	// overflow) before clamping. Avoid lround's out-of-range domain case for
	// extreme but parseable depth/scale values while preserving half-up rounding.
	const double scaled = metres * scale;
	if (!(scaled > 0.5))
		return 1; // also matches Rust's NaN-to-zero cast followed by clamp
	if (scaled >= MAX_DEPTH - 0.5)
		return MAX_DEPTH;
	return static_cast<int>(std::floor(scaled + 0.5));
}

std::optional<std::pair<double, double>> lap_pool_size(const tags_t &tags)
{
	const auto kind = tags.get("swimming_pool");
	if (kind == "wading" || kind == "paddling" || kind == "kiddie" || kind == "toddler" ||
			kind == "children" || kind == "diving" || kind == "plunge" ||
			kind == "plunge_pool" || kind == "hot_tub" || kind == "whirlpool" ||
			kind == "spa" || kind == "wave" || kind == "ornamental" ||
			kind == "reflecting")
		return std::nullopt;
	bool sport_swimming = false;
	const auto sport = tags.get("sport");
	if (!sport.empty()) {
		std::size_t begin = 0;
		while (begin <= sport.size()) {
			const auto end = sport.find(';', begin);
			std::string_view item(sport.data() + begin,
					(end == std::string::npos ? sport.size() : end) - begin);
			while (!item.empty() &&
					std::isspace(static_cast<unsigned char>(item.front())))
				item.remove_prefix(1);
			while (!item.empty() && std::isspace(static_cast<unsigned char>(item.back())))
				item.remove_suffix(1);
			if (item == "swimming") {
				sport_swimming = true;
				break;
			}
			if (end == std::string::npos)
				break;
			begin = end + 1;
		}
	}
	return kind == "swimming" || kind == "lap" || kind == "competition" || sport_swimming
				   ? std::pair{20.0, 7.0}
				   : std::pair{25.0, 10.0};
}

Cells lane_cells(const ProcessedWay &way, const Cells &cells, double scale)
{
	Cells out;
	const auto minimum = lap_pool_size(way.tags);
	const auto direction = sport_pitches::longest_edge_dir(way);
	if (!minimum || !direction || cells.empty())
		return out;
	double ux = direction->first, uz = direction->second;
	double vx = -uz, vz = ux;
	const double n = static_cast<double>(cells.size());
	double cx = 0.0, cz = 0.0;
	for (const auto &[x, z] : cells) {
		cx += x;
		cz += z;
	}
	cx /= n;
	cz /= n;
	auto extent = [&](double ax, double az) {
		double lo = std::numeric_limits<double>::max();
		double hi = std::numeric_limits<double>::lowest();
		for (const auto &[x, z] : cells) {
			const double u = (x - cx) * ax + (z - cz) * az;
			lo = std::min(lo, u);
			hi = std::max(hi, u);
		}
		return std::pair{lo, hi};
	};
	auto [umin, umax] = extent(ux, uz);
	auto [vmin, vmax] = extent(vx, vz);
	if (vmax - vmin > umax - umin) {
		std::swap(ux, vx);
		std::swap(uz, vz);
		std::swap(umin, vmin);
		std::swap(umax, vmax);
	}
	const double length = umax - umin + 1.0, width = vmax - vmin + 1.0;
	if (length < minimum->first * scale || width < minimum->second * scale ||
			n < 0.85 * length * width)
		return out;
	const double lane = 2.5 * std::max(1.0, scale);
	const int count = static_cast<int>(std::floor(width / lane));
	if (count < 2)
		return out;
	const double first = (vmin + vmax) * 0.5 - count * lane * 0.5 + lane * 0.5;
	const double umid = (umin + umax) * 0.5;
	const double line_end = length * 0.5 - std::max(2.0 * scale, 1.5);
	auto on = [](double d, double half) { return d > -half && d <= half; };
	for (const auto &[x, z] : cells) {
		const double dx = x - cx, dz = z - cz;
		const double u = dx * ux + dz * uz - umid;
		const double v = dx * vx + dz * vz;
		bool marked = false;
		for (int k = 0; k < count; ++k) {
			const double dv = v - (first + k * lane);
			if (on(dv, 0.5) && std::abs(u) <= line_end) {
				marked = true;
				break;
			}
			if (lane >= 4.0 && on(dv, 1.5) && on(std::abs(u) - line_end, 0.5)) {
				marked = true;
				break;
			}
		}
		if (marked)
			out.emplace(x, z);
	}
	return out;
}

bool is_closed(const ProcessedWay &way)
{
	return way.nodes.size() >= 4 && way.nodes.front().x == way.nodes.back().x &&
		   way.nodes.front().z == way.nodes.back().z;
}

void generate_ring(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &cache, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads, const Holes *holes)
{
	if (!is_closed(way))
		return;
	const auto fill = cache.get_or_compute(way, args.timeout);
	if (fill->empty() && is_oversized_ring(way))
		return;
	Cells cells;
	for (const auto &[x, z] : *fill)
		if (!in_hole(holes, x, z))
			cells.emplace(x, z);
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1], &b = way.nodes[i];
		for (const auto &[x, y, z] :
				bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)y;
			if (!edge_in_hole(holes, x, z))
				cells.emplace(x, z);
		}
	}
	for (auto it = cells.begin(); it != cells.end();) {
		if (under_floor(editor, footprints, it->first, it->second))
			it = cells.erase(it);
		else
			++it;
	}
	if (cells.empty())
		return;
	int water_y = std::numeric_limits<int>::min();
	for (const auto &[x, z] : cells)
		water_y = std::max(water_y, editor.get_ground_level(x, z));
	const int floor_y = water_y - pool_depth(way.tags, cells.size(), editor.scale());
	const auto lanes = lane_cells(way, cells, editor.scale());
	for (const auto &[x, z] : cells) {
		const int ground_y = editor.get_ground_level(x, z);
		place_pool_block(editor, lanes.contains({x, z}) ? LANE_LINE : LINING, x, floor_y,
				z, ground_y);
		for (int y = floor_y + 1; y <= water_y; ++y)
			place_pool_block(editor, WATER, x, y, z, ground_y);
		editor.fill_column_absolute(
				LINING, x, z, std::min(ground_y, floor_y - 1), floor_y - 1, true);
		if (tree_part(editor, x, water_y + 1, z))
			water_depth::clear_tree_from(editor, x, water_y + 1, z);
		editor.set_block_absolute(AIR, x, water_y + 1, z,
				std::optional<std::vector<Block>>(
						ground_decoration::loose_plant_blocks()),
				std::nullopt);
		editor.set_block_absolute(AIR, x, water_y + 2, z,
				std::optional<std::vector<Block>>(
						ground_decoration::stacked_plant_parts()),
				std::nullopt);
	}
	Cells rim;
	for (const auto &[x, z] : cells)
		for (int dx = -1; dx <= 1; ++dx)
			for (int dz = -1; dz <= 1; ++dz)
				if (!cells.contains({x + dx, z + dz}) &&
						!under_floor(editor, footprints, x + dx, z + dz))
					rim.emplace(x + dx, z + dz);
	for (const auto &[x, z] : rim) {
		const int ground_y = editor.get_ground_level(x, z);
		const int top = std::max(water_y, ground_y);
		if (top > ground_y)
			editor.fill_column_absolute(COPING, x, z, ground_y + 1, top, true);
		else if (!roads.contains(x, z))
			editor.set_block_absolute(COPING, x, top, z,
					std::optional<std::vector<Block>>(std::vector<Block>(
							NATURAL_GROUND.begin(), NATURAL_GROUND.end())),
					std::nullopt);
		editor.fill_column_absolute(LINING, x, z, std::min(floor_y - 1, ground_y),
				std::min(ground_y, top - 1), true);
	}
}
} // namespace

bool is_swimming_pool(const tags_t &tags)
{
	return tags.get("leisure") == "swimming_pool";
}

void generate(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &cache, const BuildingFootprintBitmap &footprints,
		const RoadMaskBitmap &roads)
{
	generate_ring(editor, way, args, cache, footprints, roads, nullptr);
}

void generate_relation(WorldEditor &editor, const ProcessedRelation &rel,
		const Args &args, const FloodFillCache &cache,
		const BuildingFootprintBitmap &footprints, const RoadMaskBitmap &roads)
{
	const InnerRings inners(rel);
	for (const auto &member : rel.members) {
		if (member.role != ProcessedMemberRole::Outer || !member.way)
			continue;
		const auto holes = cache.holes_in(*member.way, inners, args.timeout);
		ProcessedWay way = *member.way;
		way.tags = rel.tags;
		generate_ring(
				editor, way, args, cache, footprints, roads, holes ? &*holes : nullptr);
	}
}
} // namespace arnis::swimming_pools
