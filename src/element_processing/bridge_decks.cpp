#include "bridge_decks.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>

#include "../block_definitions.h"
#include "../bresenham.h"
#include "../strict_parse.h"
#include "connected_blocks.h"
#include "highways.h"

namespace arnis::bridge_decks
{
namespace
{
std::uint64_t variety(std::uint64_t seed)
{
	return (seed * UINT64_C(0x9E3779B97F4A7C15)) >> 61;
}

std::string first_tag(const std::vector<const ProcessedWay *> &ways, const char *key)
{
	for (const auto *way : ways) {
		if (!way)
			continue;
		auto value = way->tags.get(key);
		if (!value.empty())
			return value;
	}
	return {};
}

bool major_road(const std::vector<const ProcessedWay *> &ways)
{
	for (const auto *way : ways) {
		if (!way)
			continue;
		const auto highway = way->tags.get("highway");
		if (highway == "motorway" || highway == "motorway_link" || highway == "trunk" ||
				highway == "trunk_link" || way->tags.get("motorroad") == "yes")
			return true;
	}
	return false;
}

bool vehicular(const std::vector<const ProcessedWay *> &ways)
{
	return std::any_of(ways.begin(), ways.end(), [](const ProcessedWay *way) {
		if (!way)
			return false;
		const auto highway = way->tags.get("highway");
		return highway != "footway" && highway != "pedestrian" && highway != "path" &&
			   highway != "cycleway" && highway != "bridleway" && highway != "steps";
	});
}

DeckDesign road_design(const std::vector<const ProcessedWay *> &ways)
{
	if (major_road(ways))
		return DeckDesign::Motorway;
	return vehicular(ways) ? DeckDesign::Road : DeckDesign::Foot;
}

bool minor_road(const std::vector<const ProcessedWay *> &ways)
{
	return std::all_of(ways.begin(), ways.end(), [](const ProcessedWay *way) {
		if (!way)
			return true;
		const auto highway = way->tags.get("highway");
		return highway == "residential" || highway == "unclassified" ||
			   highway == "service" || highway == "living_street" || highway == "track" ||
			   highway == "pedestrian" || highway == "footway" || highway == "path" ||
			   highway == "cycleway" || highway == "bridleway";
	});
}

bool short_structure(const std::vector<const ProcessedWay *> &ways)
{
	for (const auto *way : ways) {
		if (!way)
			continue;
		double length = 0.0;
		for (std::size_t i = 1; i < way->nodes.size(); ++i) {
			const double dx = way->nodes[i].x - way->nodes[i - 1].x;
			const double dz = way->nodes[i].z - way->nodes[i - 1].z;
			length += std::hypot(dx, dz);
		}
		if (length > 60.0)
			return false;
	}
	return true;
}

bool supports_lamps(DeckDesign design)
{
	return design == DeckDesign::Motorway || design == DeckDesign::Road ||
		   design == DeckDesign::Foot || design == DeckDesign::Glass ||
		   design == DeckDesign::Stone || design == DeckDesign::Brick;
}

Block lamp_pole(DeckDesign design)
{
	const auto wall = parapet(design);
	if (wall && (wall->id() == block_definitions::IRON_BARS.id() ||
						wall->id() == block_definitions::GLASS_PANE.id()))
		return block_definitions::IRON_BARS;
	return wall.value_or(block_definitions::STONE_BRICK_WALL);
}

struct Cell
{
	int y = 0;
	std::uint64_t way = 0;
};

struct PairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		const auto a = std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(p.first));
		const auto b = std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(p.second));
		return a ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2));
	}
};

struct MemberPath
{
	const ProcessedWay *way = nullptr;
	const BridgeMemberInfo *info = nullptr;
	std::vector<std::pair<int, int>> cells;
	int half = 0;
	std::size_t total = 0;
	std::size_t ramp = 1;

	int y(std::size_t i) const { return info->y_at(i, total, ramp); }
	std::pair<float, float> heading(std::size_t i) const
	{
		const auto n = cells.size();
		const auto a = cells[i > 3 ? i - 3 : 0];
		const auto b = cells[std::min(i + 3, n - 1)];
		const float dx = static_cast<float>(b.first - a.first);
		const float dz = static_cast<float>(b.second - a.second);
		const float length = std::hypot(dx, dz);
		return length > 0.0f ? std::pair{dx / length, dz / length}
							 : std::pair{1.0f, 0.0f};
	}
};

struct OpenEnd
{
	float x = 0.0f;
	float z = 0.0f;
	float dx = 0.0f;
	float dz = 0.0f;
	float reach = 0.0f;
};

struct GradeRoads
{
	struct Segment
	{
		float ax = 0.0f;
		float az = 0.0f;
		float bx = 0.0f;
		float bz = 0.0f;
		float reach = 0.0f;
	};
	std::vector<Segment> segments;
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, PairHash> buckets;

	static int cell(int value) { return value >= 0 ? value / 32 : (value - 31) / 32; }

	static GradeRoads build(const std::vector<ProcessedElement> &elements,
			const BridgeStructureMap &structures, double scale)
	{
		GradeRoads result;
		for (const auto &element : elements) {
			if (!element.is_way())
				continue;
			const auto &way = element.as_way();
			if (way.nodes.size() < 2 || structures.lookup_ramp(way.id) ||
					!way.tags.contains("highway") || bridges::is_bridge_way(way) ||
					way.tags.get("area") == "yes" || way.tags.get("indoor") == "yes")
				continue;
			const auto highway = way.tags.get("highway");
			const auto tunnel = way.tags.get("tunnel");
			const auto level = strict_parse::i32(way.tags.get("level"));
			const int layer = strict_parse::i32(way.tags.get("layer")).value_or(0);
			if ((!tunnel.empty() && tunnel != "no") || (level && *level < 0) ||
					layer > 0 || highway == "street_lamp" || highway == "crossing" ||
					highway == "bus_stop" || highway == "proposed" ||
					highway == "construction" || highway == "razed" ||
					highway == "abandoned" || highway == "elevator" ||
					highway == "platform" || highway == "corridor")
				continue;
			const int half = highways::highway_block_range(highway, way.tags, scale);
			const float reach = static_cast<float>(half) + 0.5f;
			for (std::size_t i = 1; i < way.nodes.size(); ++i) {
				const auto &a = way.nodes[i - 1];
				const auto &b = way.nodes[i];
				const std::size_t index = result.segments.size();
				result.segments.push_back(
						{static_cast<float>(a.x), static_cast<float>(a.z),
								static_cast<float>(b.x), static_cast<float>(b.z), reach});
				const int r = half + 1;
				for (int bx = cell(std::min(a.x, b.x) - r);
						bx <= cell(std::max(a.x, b.x) + r); ++bx)
					for (int bz = cell(std::min(a.z, b.z) - r);
							bz <= cell(std::max(a.z, b.z) + r); ++bz)
						result.buckets[{bx, bz}].push_back(index);
			}
		}
		return result;
	}

	bool covers(int x, int z) const
	{
		const auto it = buckets.find({cell(x), cell(z)});
		if (it == buckets.end())
			return false;
		for (const auto index : it->second) {
			const auto &segment = segments[index];
			const float dx = segment.bx - segment.ax;
			const float dz = segment.bz - segment.az;
			const float len2 = dx * dx + dz * dz;
			const float t =
					len2 > 0.0f
							? std::clamp(((x - segment.ax) * dx + (z - segment.az) * dz) /
												 len2,
									  0.0f, 1.0f)
							: 0.0f;
			const float ox = x - (segment.ax + t * dx);
			const float oz = z - (segment.az + t * dz);
			if (std::max(std::abs(ox), std::abs(oz)) <= segment.reach)
				return true;
		}
		return false;
	}
};

std::size_t bridge_total_cells(const ProcessedWay &way)
{
	std::size_t total = 1;
	for (std::size_t i = 1; i < way.nodes.size(); ++i)
		total += static_cast<std::size_t>(
				std::max(std::abs(way.nodes[i].x - way.nodes[i - 1].x),
						std::abs(way.nodes[i].z - way.nodes[i - 1].z)));
	return total;
}

bool design_has_girders(DeckDesign design)
{
	return design == DeckDesign::Motorway || design == DeckDesign::Road ||
		   design == DeckDesign::Stone || design == DeckDesign::Brick;
}

bool design_has_lamps(DeckDesign design)
{
	return supports_lamps(design);
}

Block design_beam(DeckDesign design)
{
	return slab(design);
}

bool slender(DeckDesign design)
{
	return design == DeckDesign::Foot || design == DeckDesign::Glass ||
		   design == DeckDesign::Timber;
}

float pier_spacing(DeckDesign design, bridge_styles::BridgeStyle style, int height)
{
	if (style == bridge_styles::BridgeStyle::Truss)
		return 32.0f;
	if (style == bridge_styles::BridgeStyle::Suspension ||
			style == bridge_styles::BridgeStyle::CableStayed)
		return 24.0f;
	if (design == DeckDesign::Foot || design == DeckDesign::Glass)
		return std::clamp(8.0f + height * 0.5f, 8.0f, 20.0f);
	if (design == DeckDesign::Timber)
		return 6.0f;
	return std::clamp(12.0f + height * 0.6f, 12.0f, 40.0f);
}

std::vector<std::pair<int, int>> station_line(const MemberPath &path, std::size_t i,
		const std::unordered_map<std::pair<int, int>, Cell, PairHash> &all)
{
	const auto [hx, hz] = path.heading(i);
	const float px = -hz;
	const float pz = hx;
	const auto [cx, cz] = path.cells[i];
	const auto at = [&](int offset) {
		return std::pair{static_cast<int>(std::lround(cx + px * offset)),
				static_cast<int>(std::lround(cz + pz * offset))};
	};
	std::vector<std::pair<int, int>> left, right;
	for (int offset = 1; offset < 64; ++offset) {
		const auto cell = at(offset);
		if (!all.contains(cell))
			break;
		left.push_back(cell);
	}
	for (int offset = 1; offset < 64; ++offset) {
		const auto cell = at(-offset);
		if (!all.contains(cell))
			break;
		right.push_back(cell);
	}
	std::vector<std::pair<int, int>> line(right.rbegin(), right.rend());
	line.emplace_back(cx, cz);
	line.insert(line.end(), left.begin(), left.end());
	return connected_blocks::four_connected_line(line);
}

void plan_piers(const std::vector<MemberPath> &paths, DeckDesign design,
		bridge_styles::BridgeStyle style,
		const std::unordered_map<std::pair<int, int>, Cell, PairHash> &all,
		const BridgeSurfaceMap &surface, const WorldEditor &editor,
		std::unordered_map<std::uint64_t, DeckParts> &out)
{
	if (style == bridge_styles::BridgeStyle::Arch || design == DeckDesign::Boardwalk)
		return;
	// Rust plans one pier grid along the shared structure axis, rather than
	// restarting the spacing at every member. That matters at connected spans:
	// independently rounded per-way stations otherwise bunch at junctions.
	const auto &axis = paths.empty() ? std::optional<std::pair<std::pair<float, float>,
											   std::pair<float, float>>>{}
									 : paths.front().info->structure_axis;
	const auto along = [&](const MemberPath &path, std::size_t i) {
		if (!axis)
			return static_cast<float>(i);
		return (path.cells[i].first - axis->first.first) * axis->second.first +
			   (path.cells[i].second - axis->first.second) * axis->second.second;
	};
	float structure_min = std::numeric_limits<float>::max();
	float structure_max = std::numeric_limits<float>::lowest();
	if (axis) {
		for (const auto &path : paths)
			for (std::size_t i = 0; i < path.cells.size(); ++i) {
				const float u = along(path, i);
				structure_min = std::min(structure_min, u);
				structure_max = std::max(structure_max, u);
			}
	}
	int structure_height = 0;
	if (axis)
		for (const auto &path : paths)
			for (std::size_t i = 0; i < path.cells.size(); ++i)
				structure_height = std::max(structure_height,
						path.y(i) - editor.get_ground_level(
											path.cells[i].first, path.cells[i].second));
	std::unordered_set<std::pair<int, int>, PairHash> placed;
	for (const auto &path : paths) {
		if (path.cells.size() < 5)
			continue;
		float path_min = along(path, 0);
		float path_max = path_min;
		for (std::size_t i = 1; i < path.cells.size(); ++i) {
			const float u = along(path, i);
			path_min = std::min(path_min, u);
			path_max = std::max(path_max, u);
		}
		const float span = axis ? structure_max - structure_min
								: static_cast<float>(path.cells.size() - 1);
		int max_height = 0;
		for (std::size_t i = 0; i < path.cells.size(); ++i)
			max_height = std::max(
					max_height, path.y(i) - editor.get_ground_level(path.cells[i].first,
													path.cells[i].second));
		const float spacing =
				pier_spacing(design, style, axis ? structure_height : max_height);
		const int spans =
				span < 2.0f * spacing
						? 1
						: std::max(1, static_cast<int>(std::lround(span / spacing)));
		const float step = span / spans;
		const int first_station =
				axis ? std::max(1, static_cast<int>(
										   std::ceil((path_min - structure_min) / step)))
					 : 1;
		const int last_station =
				axis ? std::min(spans - 1, static_cast<int>(std::floor(
												   (path_max - structure_min) / step)))
					 : spans - 1;
		for (int station = first_station; station <= last_station; ++station) {
			const float target = axis ? structure_min + step * station : step * station;
			std::size_t nearest = 0;
			float distance = std::numeric_limits<float>::max();
			for (std::size_t i = 0; i < path.cells.size(); ++i) {
				const float d = std::abs(along(path, i) - target);
				if (d < distance) {
					distance = d;
					nearest = i;
				}
			}
			if (distance > 1.0f || placed.contains(path.cells[nearest]))
				continue;
			const auto shift = static_cast<std::size_t>(step / 3.0f);
			bool laid = false;
			for (std::size_t offset = 0; offset <= shift && !laid; ++offset) {
				std::array<std::optional<std::size_t>, 2> candidates{
						nearest + offset < path.cells.size()
								? std::optional<std::size_t>(nearest + offset)
								: std::nullopt,
						offset > 0 && nearest >= offset
								? std::optional<std::size_t>(nearest - offset)
								: std::nullopt};
				for (const auto candidate : candidates) {
					if (!candidate)
						continue;
					const auto [cx, cz] = path.cells[*candidate];
					const int y = path.y(*candidate);
					const int ground = editor.get_ground_level(cx, cz);
					const int height = y - ground;
					if (height < 3 ||
							(path.info->cable_span &&
									path.info->cable_span->suspends(cx, cz)) ||
							(!path.info->cable_span && path.info->cable_pylons))
						break;
					const bool water = editor.is_mapped_water(cx, cz);
					if (water && y - editor.get_water_level(cx, cz) < 6)
						continue;
					auto line = station_line(path, *candidate, all);
					if (line.empty())
						continue;
					const auto free =
							std::count_if(line.begin(), line.end(), [&](const auto &p) {
								return !surface.support_blocked(p.first, p.second, y);
							});
					if (free * 2 < static_cast<int>(line.size()))
						continue;
					const int width = static_cast<int>(line.size());
					const int thick = slender(design) && height < 16 ? 1
									  : slender(design)				 ? 2
									  : height < 10					 ? 1
									  : height < 25					 ? 2
									  : height < 50					 ? 3
																	 : 4;
					const bool wall =
							!slender(design) && width >= 5 && (water || height >= 30);
					std::vector<std::size_t> picks;
					if (wall) {
						for (std::size_t k = 1; k + 1 < line.size(); ++k)
							picks.push_back(k);
					} else if (design == DeckDesign::Timber && width >= 3) {
						picks = {0, line.size() - 1};
					} else if (width <= 4) {
						picks = {line.size() / 2};
					} else {
						const std::size_t a = 1, b = line.size() - 2;
						const auto gaps = std::max<std::size_t>(
								1, static_cast<std::size_t>(std::ceil((b - a) / 8.0)));
						for (std::size_t k = 0; k <= gaps; ++k)
							picks.push_back(a + (b - a) * k / gaps);
					}
					const auto [hx, hz] = path.heading(*candidate);
					auto &parts = out[path.way->id];
					std::unordered_set<std::pair<int, int>, PairHash> column_cells;
					for (const auto k : picks) {
						const auto [lx, lz] = line[k];
						for (int t = 0; t < thick; ++t) {
							const float a = static_cast<float>(t - (thick - 1) / 2);
							for (int w = 0; w < (wall ? 1 : thick); ++w) {
								const float b = static_cast<float>(
										w - ((wall ? 1 : thick) - 1) / 2);
								const auto cell = std::pair{
										static_cast<int>(
												std::lround(lx + hx * a - hz * b)),
										static_cast<int>(
												std::lround(lz + hz * a + hx * b))};
								if (column_cells.insert(cell).second &&
										!surface.support_blocked(
												cell.first, cell.second, y)) {
									parts.piers.emplace_back(
											cell.first, cell.second, y - 2);
									if (!wall && height >= 5)
										parts.footed.push_back(cell);
								}
							}
						}
					}
					if (!wall && picks.size() > 1 && height >= 5)
						for (const auto &[x, z] : line)
							parts.caps.emplace_back(x, z, y - 2);
					placed.insert(line.begin(), line.end());
					laid = true;
					break;
				}
			}
		}
	}
}

} // namespace

DeckDesign pick(
		bridge_styles::BridgeStyle style, const std::vector<const ProcessedWay *> &ways)
{
	using bridge_styles::BridgeStyle;
	if (style == BridgeStyle::Boardwalk)
		return DeckDesign::Boardwalk;
	if (style == BridgeStyle::Covered)
		return DeckDesign::Covered;
	const bool drives = vehicular(ways);
	if (style == BridgeStyle::Suspension || style == BridgeStyle::CableStayed ||
			style == BridgeStyle::Truss)
		return drives ? DeckDesign::Road : DeckDesign::Foot;

	const auto material = first_tag(ways, "bridge:material");
	const auto mat = material.empty() ? first_tag(ways, "material") : material;
	if (mat == "wood" || mat == "timber")
		return DeckDesign::Timber;
	if (mat == "stone" || mat == "masonry" || mat == "sandstone" || mat == "granite" ||
			mat == "limestone")
		return DeckDesign::Stone;
	if (mat == "brick" || mat == "bricks")
		return DeckDesign::Brick;
	if (mat == "concrete" || mat == "reinforced_concrete" || mat == "steel" ||
			mat == "metal" || mat == "iron")
		return road_design(ways);
	if (style == BridgeStyle::Arch)
		return DeckDesign::Stone;

	if (!drives) {
		bool wooden = first_tag(ways, "surface") == "wood";
		if (!wooden) {
			wooden = std::all_of(ways.begin(), ways.end(), [](const ProcessedWay *way) {
				if (!way)
					return true;
				const auto highway = way->tags.get("highway");
				const auto surface = way->tags.get("surface");
				return (highway == "path" || highway == "bridleway") &&
					   (surface.empty() || surface == "ground" || surface == "dirt" ||
							   surface == "unpaved" || surface == "gravel" ||
							   surface == "fine_gravel" || surface == "compacted" ||
							   surface == "wood");
			});
		}
		if (wooden)
			return DeckDesign::Timber;
	}

	const auto year_tag = first_tag(ways, "start_date");
	int year = 0;
	if (year_tag.size() >= 4) {
		try {
			year = std::stoi(year_tag.substr(0, 4));
		} catch (...) {
			year = 0;
		}
	}
	const bool old = !first_tag(ways, "historic").empty() ||
					 !first_tag(ways, "heritage").empty() || (year != 0 && year < 1900);
	const bool masonry =
			std::any_of(ways.begin(), ways.end(), [](const ProcessedWay *way) {
				if (!way)
					return false;
				const auto highway = way->tags.get("highway");
				const auto surface = way->tags.get("surface");
				return highway == "steps" || surface == "sett" ||
					   surface == "cobblestone" || surface == "unhewn_cobblestone" ||
					   surface == "stone";
			});
	if (old || masonry)
		return DeckDesign::Stone;

	const auto roll =
			ways.empty()
					? 7
					: variety((*std::min_element(ways.begin(), ways.end(),
									   [](const ProcessedWay *a, const ProcessedWay *b) {
										   return (a ? a->id : 0) < (b ? b->id : 0);
									   }))
									  ->id);
	const auto road = road_design(ways);
	if (road == DeckDesign::Road && minor_road(ways) && short_structure(ways) && roll < 3)
		return DeckDesign::Stone;
	if (road == DeckDesign::Foot && roll < 2)
		return DeckDesign::Glass;
	return road;
}

Block slab(DeckDesign design)
{
	switch (design) {
	case DeckDesign::Stone:
		return block_definitions::STONE_BRICKS;
	case DeckDesign::Brick:
		return block_definitions::BRICK;
	case DeckDesign::Timber:
		return block_definitions::SPRUCE_PLANKS;
	case DeckDesign::Boardwalk:
		return block_definitions::OAK_PLANKS;
	case DeckDesign::Covered:
		return block_definitions::DARK_OAK_PLANKS;
	default:
		return block_definitions::SMOOTH_STONE;
	}
}

std::optional<Block> parapet(DeckDesign design)
{
	switch (design) {
	case DeckDesign::Motorway:
		return block_definitions::ANDESITE_WALL;
	case DeckDesign::Road:
	case DeckDesign::Foot:
		return block_definitions::IRON_BARS;
	case DeckDesign::Glass:
		return block_definitions::GLASS_PANE;
	case DeckDesign::Stone:
		return block_definitions::STONE_BRICK_WALL;
	case DeckDesign::Brick:
		return block_definitions::BRICK_WALL;
	case DeckDesign::Timber:
		return block_definitions::SPRUCE_FENCE;
	case DeckDesign::Boardwalk:
	case DeckDesign::Covered:
		return std::nullopt;
	}
	return std::nullopt;
}

std::unordered_map<std::uint64_t, DeckParts>
plan(const std::vector<ProcessedElement> &elements, const BridgeStructureMap &structures,
		const BridgeSurfaceMap &surface, const WorldEditor &editor, double scale)
{
	std::unordered_map<std::size_t, std::vector<MemberPath>> grouped;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto *info = structures.lookup_member(way.id);
		if (!info || info->covered_by_wider)
			continue;
		auto cells = bridges::way_cells(way);
		if (cells.empty())
			continue;
		const auto total = bridge_total_cells(way);
		const auto raw_ramp = static_cast<std::size_t>(
				std::clamp(static_cast<float>(total) * 0.35f, 15.0f, 50.0f));
		const auto ramp =
				std::clamp<std::size_t>(raw_ramp, 1, std::max<std::size_t>(1, total / 2));
		const auto highway = way.tags.get("highway");
		const int half = highways::highway_block_range(highway, way.tags, scale);
		grouped[info->structure_id].push_back(
				MemberPath{&way, info, std::move(cells), half, total, ramp});
	}
	std::unordered_map<std::uint64_t, DeckParts> result;
	if (grouped.empty())
		return result;

	std::vector<std::pair<int, int>> mapped_lamps;
	for (const auto &element : elements)
		if (element.is_node() && element.tags().get("highway") == "street_lamp")
			mapped_lamps.emplace_back(element.as_node().x, element.as_node().z);
	const auto roads = GradeRoads::build(elements, structures, scale);

	std::vector<std::size_t> structure_ids;
	structure_ids.reserve(grouped.size());
	for (const auto &[id, _] : grouped)
		structure_ids.push_back(id);
	std::sort(structure_ids.begin(), structure_ids.end());
	for (const auto structure_id : structure_ids) {
		auto &paths = grouped.at(structure_id);
		std::sort(paths.begin(), paths.end(),
				[](const auto &a, const auto &b) { return a.way->id < b.way->id; });
		const auto design = paths.front().info->deck_design;
		for (const auto &path : paths)
			result.try_emplace(path.way->id);
		std::unordered_map<std::pair<int, int>, Cell, PairHash> deck;
		for (const auto &path : paths) {
			for (std::size_t i = 0; i < path.cells.size(); ++i) {
				const auto [cx, cz] = path.cells[i];
				const auto y = path.y(i);
				for (int dx = -path.half; dx <= path.half; ++dx)
					for (int dz = -path.half; dz <= path.half; ++dz) {
						const auto cell = std::pair{cx + dx, cz + dz};
						auto [it, inserted] =
								deck.try_emplace(cell, Cell{y, path.way->id});
						if (!inserted && y > it->second.y)
							it->second = {y, path.way->id};
					}
			}
		}
		int min_x = std::numeric_limits<int>::max();
		int min_z = std::numeric_limits<int>::max();
		int max_x = std::numeric_limits<int>::lowest();
		int max_z = std::numeric_limits<int>::lowest();
		for (const auto &[coord, _] : deck) {
			min_x = std::min(min_x, coord.first);
			min_z = std::min(min_z, coord.second);
			max_x = std::max(max_x, coord.first);
			max_z = std::max(max_z, coord.second);
		}
		std::vector<std::pair<int, int>> nearby_mapped_lamps;
		for (const auto &lamp : mapped_lamps)
			if (lamp.first >= min_x - 8 && lamp.first <= max_x + 8 &&
					lamp.second >= min_z - 8 && lamp.second <= max_z + 8)
				nearby_mapped_lamps.push_back(lamp);

		// Fill narrow, level gaps between side-by-side bridge ways.
		std::unordered_map<std::pair<int, int>, Cell, PairHash> fills;
		for (const bool along_x : {true, false}) {
			std::map<int, std::vector<std::pair<int, Cell>>> lines;
			for (const auto &[coord, cell] : deck) {
				const auto [x, z] = coord;
				lines[along_x ? z : x].emplace_back(along_x ? x : z, cell);
			}
			for (auto &[line, cells] : lines) {
				std::sort(cells.begin(), cells.end(),
						[](const auto &a, const auto &b) { return a.first < b.first; });
				for (std::size_t i = 1; i < cells.size(); ++i) {
					const auto &[a, ca] = cells[i - 1];
					const auto &[b, cb] = cells[i];
					const int gap = b - a - 1;
					if (gap < 1 || gap > 4 || std::abs(ca.y - cb.y) > 1)
						continue;
					const int y = std::max(ca.y, cb.y);
					for (int pos = a + 1; pos < b; ++pos) {
						const auto cell =
								along_x ? std::pair{pos, line} : std::pair{line, pos};
						if (surface.deck_near(cell.first, cell.second, y, 2) ||
								surface.rail_deck_near(cell.first, cell.second, y, 2))
							continue;
						const auto owner = pos - a <= b - pos ? ca.way : cb.way;
						fills.try_emplace(cell, Cell{y, owner});
					}
				}
			}
		}
		for (const auto &[coord, cell] : fills)
			result[cell.way].fills.emplace_back(coord.first, coord.second, cell.y);
		auto all = deck;
		all.insert(fills.begin(), fills.end());

		std::vector<OpenEnd> ends;
		for (const auto &path : paths) {
			if (path.cells.size() < 2)
				continue;
			const auto last = path.cells.size() - 1;
			const auto add_end = [&](bool open, std::size_t i, std::size_t j) {
				if (!open || i == j)
					return;
				const auto [x, z] = path.cells[i];
				const auto [jx, jz] = path.cells[j];
				const float dx = static_cast<float>(x - jx);
				const float dz = static_cast<float>(z - jz);
				const float len = std::hypot(dx, dz);
				if (len > 0.0f)
					ends.push_back({static_cast<float>(x), static_cast<float>(z),
							dx / len, dz / len, static_cast<float>(path.half + 2)});
			};
			add_end(path.info->start_is_group_boundary, 0,
					std::min<std::size_t>(3, last));
			add_end(path.info->end_is_group_boundary, last, last > 3 ? last - 3 : 0);
		}
		const auto beyond_end = [&](int x, int z) {
			return std::any_of(ends.begin(), ends.end(), [&](const OpenEnd &end) {
				const float dx = x - end.x;
				const float dz = z - end.z;
				const float along = dx * end.dx + dz * end.dz;
				const float across = std::abs(dx * end.dz - dz * end.dx);
				return along > 0.5f && along <= end.reach && across <= end.reach;
			});
		};

		std::unordered_map<std::pair<int, int>, Cell, PairHash> ring;
		for (const auto &[coord, cell] : all) {
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz) {
					if (dx == 0 && dz == 0)
						continue;
					const auto neighbour = std::pair{coord.first + dx, coord.second + dz};
					if (all.contains(neighbour))
						continue;
					auto [it, inserted] = ring.try_emplace(neighbour, cell);
					if (!inserted && cell.y > it->second.y)
						it->second = cell;
				}
		}
		std::unordered_map<std::pair<int, int>, Cell, PairHash> rails;
		if (design != DeckDesign::Boardwalk) {
			for (const auto &[coord, cell] : ring) {
				const auto [x, z] = coord;
				const int ground = editor.get_ground_level(x, z);
				if (ground >= cell.y || surface.deck_near(x, z, cell.y, 2) ||
						beyond_end(x, z) ||
						(cell.y - ground <= 2 &&
								(roads.covers(x, z) || surface.over_grade_way(x, z))))
					continue;
				rails.try_emplace(coord, cell);
			}
		}

		std::vector<std::pair<int, int>> lamps;
		if (paths.front().info->lit && design_has_lamps(design)) {
			for (const auto &path : paths) {
				for (std::size_t i = 8, n = 0; i < path.cells.size(); i += 16, ++n) {
					const auto [hx, hz] = path.heading(i);
					const float px = -hz;
					const float pz = hx;
					const float reach = path.half * (std::abs(px) + std::abs(pz)) + 1.0f;
					const auto [cx, cz] = path.cells[i];
					const int first_sign = n % 2 == 0 ? 1 : -1;
					for (const int sign : {first_sign, -first_sign}) {
						const auto cell = std::pair{
								static_cast<int>(std::lround(cx + px * reach * sign)),
								static_cast<int>(std::lround(cz + pz * reach * sign))};
						const bool clear =
								std::all_of(lamps.begin(), lamps.end(),
										[&](const auto &other) {
											const auto dx = cell.first - other.first;
											const auto dz = cell.second - other.second;
											return dx * dx + dz * dz >= 64;
										}) &&
								std::all_of(nearby_mapped_lamps.begin(),
										nearby_mapped_lamps.end(),
										[&](const auto &other) {
											const auto dx = cell.first - other.first;
											const auto dz = cell.second - other.second;
											return dx * dx + dz * dz >= 64;
										});
						if (!rails.contains(cell) || !clear)
							continue;
						lamps.push_back(cell);
						break;
					}
				}
			}
		}
		const std::unordered_set<std::pair<int, int>, PairHash> lamp_set(
				lamps.begin(), lamps.end());
		for (const auto &[coord, cell] : rails) {
			auto &parts = result[cell.way];
			if (lamp_set.contains(coord)) {
				parts.lamps.emplace_back(coord.first, coord.second, cell.y);
				continue;
			}
			parts.rails.emplace_back(coord.first, coord.second, cell.y);
			static constexpr std::array<std::pair<int, int>, 4> adjacent{
					{{0, -1}, {0, 1}, {1, 0}, {-1, 0}}};
			const bool step =
					std::any_of(adjacent.begin(), adjacent.end(), [&](const auto &d) {
						const auto it = rails.find(
								{coord.first + d.first, coord.second + d.second});
						return it != rails.end() && it->second.y == cell.y + 1;
					});
			if (step)
				parts.steps.emplace_back(coord.first, coord.second, cell.y + 2);
		}

		if (design_has_girders(design)) {
			for (const auto &[coord, cell] : all) {
				const int height =
						cell.y - editor.get_ground_level(coord.first, coord.second);
				const int depth = height <= 9 ? 0 : height <= 19 ? 1 : 2;
				for (int d = 0; d < depth; ++d)
					result[cell.way].girders.emplace_back(
							coord.first, coord.second, cell.y - 2 - d);
			}
			for (const auto &[coord, cell] : rails) {
				const int height =
						cell.y - editor.get_ground_level(coord.first, coord.second);
				const int depth = height <= 9 ? 0 : height <= 19 ? 1 : 2;
				for (int d = 0; d < depth; ++d)
					result[cell.way].girders.emplace_back(
							coord.first, coord.second, cell.y - 2 - d);
			}
		}
		plan_piers(
				paths, design, paths.front().info->style, all, surface, editor, result);
	}
	return result;
}

void render(WorldEditor &editor, DeckDesign design, const DeckParts &parts)
{
	const Block base = slab(design);
	const Block beam = design_beam(design);
	const auto room = [&](int x, int z, int y) {
		return y > editor.get_ground_level(x, z) +
						   static_cast<int>(editor.is_mapped_water(x, z));
	};
	for (const auto &[x, z, y] : parts.fills) {
		editor.set_block_absolute(base, x, y, z);
		editor.set_block_absolute(base, x, y - 1, z);
	}
	for (const auto &[x, z, y] : parts.rails) {
		editor.set_block_absolute(base, x, y, z);
		editor.set_block_absolute(base, x, y - 1, z);
		if (room(x, z, y - 2))
			editor.set_block_absolute(beam, x, y - 2, z);
	}
	for (const auto &[x, z, y] : parts.lamps) {
		editor.set_block_absolute(base, x, y, z);
		editor.set_block_absolute(base, x, y - 1, z);
		if (room(x, z, y - 2))
			editor.set_block_absolute(beam, x, y - 2, z);
	}
	if (const auto wall = parapet(design)) {
		for (const auto &[x, z, y] : parts.rails)
			connected_blocks::place_connected(editor, *wall, x, y + 1, z);
		for (const auto &[x, z, y] : parts.steps)
			connected_blocks::place_connected(editor, *wall, x, y, z);
	}
	const auto pole = lamp_pole(design);
	for (const auto &[x, z, y] : parts.lamps) {
		editor.register_lamp(x, z);
		for (int dy = 1; dy <= 4; ++dy)
			connected_blocks::place_connected(editor, pole, x, y + dy, z);
		editor.set_block_with_properties_absolute(
				BlockWithProperties{block_definitions::REDSTONE_LAMP,
						BlockWithProperties::Properties{{"lit", "true"}}},
				x, y + 5, z, std::nullopt, std::nullopt);
		editor.set_block_absolute(block_definitions::IRON_TRAPDOOR, x, y + 6, z);
	}
	for (const auto &[x, z, y] : parts.caps)
		if (room(x, z, y))
			editor.set_block_absolute(beam, x, y, z);
	for (const auto &[x, z, y] : parts.girders)
		if (room(x, z, y))
			editor.set_block_absolute(beam, x, y, z);
	const std::unordered_set<std::pair<int, int>, PairHash> footed(
			parts.footed.begin(), parts.footed.end());
	const Block pier = design == DeckDesign::Stone	 ? block_definitions::STONE_BRICKS
					   : design == DeckDesign::Brick ? block_definitions::BRICK
					   : design == DeckDesign::Timber ||
									   design == DeckDesign::Boardwalk ||
									   design == DeckDesign::Covered
							   ? block_definitions::SPRUCE_LOG
							   : block_definitions::LIGHT_GRAY_CONCRETE;
	for (const auto &[x, z, top] : parts.piers) {
		const int ground = editor.get_ground_level(x, z);
		if (top <= ground)
			continue;
		for (int y = ground + 1; y <= top; ++y)
			editor.set_block_absolute(pier, x, y, z);
		editor.register_support_column(x, z, pier);
		if (footed.contains({x, z}))
			for (int dx = -1; dx <= 1; ++dx)
				for (int dz = -1; dz <= 1; ++dz)
					editor.set_block_absolute(pier, x + dx, ground, z + dz);
	}
}

void render(WorldEditor &editor, const BridgeSurfaceMap &surface, DeckDesign design,
		const std::vector<bridge_styles::BridgePathSample> &path, int block_range,
		bool lit)
{
	if (path.size() < 2 || block_range < 0)
		return;
	const auto base = slab(design);
	const auto rail = parapet(design);
	std::unordered_set<std::uint64_t> drawn;
	std::vector<std::pair<int, int>> placed_lamps;
	for (const auto &sample : path) {
		const float norm = std::hypot(sample.perp_x, sample.perp_z);
		if (norm <= 0.0f)
			continue;
		for (int offset = -block_range; offset <= block_range; ++offset) {
			const int x = static_cast<int>(
					std::lround(sample.x + sample.perp_x / norm * offset));
			const int z = static_cast<int>(
					std::lround(sample.z + sample.perp_z / norm * offset));
			const auto deck_y = surface.deck_y_at(x, z);
			if (!deck_y)
				continue;
			const auto key =
					(static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) ^
					static_cast<std::uint32_t>(z);
			if (!drawn.insert(key).second)
				continue;
			editor.set_block_absolute(base, x, *deck_y - 1, z, nullptr, nullptr);
		}
		if (!rail)
			continue;
		for (const int sign : {-1, 1}) {
			const int x = static_cast<int>(
					std::lround(sample.x + sample.perp_x / norm * block_range * sign));
			const int z = static_cast<int>(
					std::lround(sample.z + sample.perp_z / norm * block_range * sign));
			const auto deck_y = surface.deck_y_at(x, z);
			if (deck_y) {
				editor.set_block_absolute(base, x, *deck_y, z, nullptr, nullptr);
				editor.set_block_absolute(base, x, *deck_y - 1, z, nullptr, nullptr);
				if (!rail)
					continue;
				connected_blocks::place_connected(editor, *rail, x, *deck_y + 1, z);
			}
		}
		if (!lit || !supports_lamps(design) || path.size() <= 8 ||
				(&sample - path.data()) < 8 || ((&sample - path.data() - 8) % 16) != 0)
			continue;
		const auto index = static_cast<std::size_t>(&sample - path.data());
		const int sign_order = ((index - 8) / 16) % 2 == 0 ? 1 : -1;
		for (const int sign : {sign_order, -sign_order}) {
			const int x = static_cast<int>(std::lround(
					sample.x + sample.perp_x / norm * (block_range + 1) * sign));
			const int z = static_cast<int>(std::lround(
					sample.z + sample.perp_z / norm * (block_range + 1) * sign));
			if (surface.deck_near(x, z, sample.y, 2) ||
					editor.get_ground_level(x, z) >= sample.y)
				continue;
			const bool spaced = std::all_of(
					placed_lamps.begin(), placed_lamps.end(), [&](const auto &other) {
						const auto dx = x - other.first;
						const auto dz = z - other.second;
						return dx * dx + dz * dz >= 64;
					});
			if (!spaced)
				continue;
			placed_lamps.emplace_back(x, z);
			editor.register_lamp(x, z);
			editor.set_block_absolute(base, x, sample.y, z, nullptr, nullptr);
			editor.set_block_absolute(base, x, sample.y - 1, z, nullptr, nullptr);
			for (int dy = 1; dy <= 4; ++dy)
				connected_blocks::place_connected(
						editor, lamp_pole(design), x, sample.y + dy, z);
			editor.set_block_with_properties_absolute(
					BlockWithProperties{block_definitions::REDSTONE_LAMP,
							BlockWithProperties::Properties{{"lit", "true"}}},
					x, sample.y + 5, z, std::nullopt, std::nullopt);
			editor.set_block_absolute(
					block_definitions::IRON_TRAPDOOR, x, sample.y + 6, z);
			break;
		}
	}
}

} // namespace arnis::bridge_decks
