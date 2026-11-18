#include "elements.h"

#include "../element_processing/highways.h"
#include "../element_processing/way_segments.h"
#include "../element_processing/waterways.h"
#include "../floodfill_cache.h"
#include "../../arnis_ground.h"
#include "grid.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arnis::survey::elements
{
namespace
{
constexpr std::uint64_t ID_BASE = 0x7A00000000000000ULL;
constexpr int PAVED_CELL = 32;
constexpr double OVER_WATER_REACH = 2.5;
constexpr std::array<const char *, 8> TRACKS = {"rail", "light_rail", "tram", "subway",
		"narrow_gauge", "preserved", "monorail", "funicular"};
using Point = std::pair<double, double>;
using Ring = std::vector<Point>;
using Line = std::tuple<Point, Point, double>;

struct WaterRegion
{
	std::vector<Ring> outers;
	std::vector<Ring> inners;
};

const std::string *tag(const tags_t &tags, const char *key)
{
	const auto it = tags.find(key);
	return it == tags.end() ? nullptr : &it->second;
}

Point node_point(const ProcessedNode &node)
{
	return {double(node.x) + .5, double(node.z) + .5};
}

bool closed(const ProcessedWay &way)
{
	return way.nodes.size() >= 4 && way.nodes.front().x == way.nodes.back().x &&
		   way.nodes.front().z == way.nodes.back().z;
}

bool ring_contains(const Ring &ring, double x, double z)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto [xi, zi] = ring[i];
		const auto [xj, zj] = ring[j];
		if ((zi > z) != (zj > z) && x < (xj - xi) * (z - zi) / (zj - zi) + xi)
			inside = !inside;
		j = i;
	}
	return inside;
}

bool in_convex(const Ring &ring, double x, double z)
{
	bool positive = false, negative = false;
	for (std::size_t i = 1; i < ring.size(); ++i) {
		const auto [ax, az] = ring[i - 1];
		const auto [bx, bz] = ring[i];
		const double cross = (bx - ax) * (z - az) - (bz - az) * (x - ax);
		positive |= cross > 1e-9;
		negative |= cross < -1e-9;
	}
	return !(positive && negative);
}

bool not_a_garden(const tags_t &tags)
{
	if (tags.contains("building") || tags.contains("highway"))
		return false;
	if (const auto *leisure = tag(tags, "leisure"))
		return *leisure != "garden" && *leisure != "swimming_pool" &&
			   *leisure != "resort";
	if (const auto *natural = tag(tags, "natural"))
		return *natural != "tree" && *natural != "tree_row";
	if (const auto *landuse = tag(tags, "landuse")) {
		static const std::array<const char *, 9> gardens = {"residential", "commercial",
				"retail", "industrial", "education", "religious", "allotments",
				"farmyard", "garages"};
		return std::none_of(gardens.begin(), gardens.end(),
				[&](const char *v) { return *landuse == v; });
	}
	const auto *amenity = tag(tags, "amenity");
	return (amenity &&
				   (*amenity == "school" || *amenity == "kindergarten" ||
						   *amenity == "waste_disposal" || *amenity == "recycling")) ||
		   (tag(tags, "place") && *tag(tags, "place") == "square");
}

struct Paved
{
	std::vector<Line> lines;
	std::vector<Line> water_channels;
	std::vector<bool> driveway;
	std::vector<Ring> areas, not_gardens, no_pools;
	std::vector<WaterRegion> water_regions;
	std::unordered_map<std::uint64_t, std::vector<std::size_t>> buckets;

	static std::uint64_t key(int x, int z)
	{
		return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
	}

	static std::vector<Ring> outers(const ProcessedElement &element)
	{
		std::vector<Ring> result;
		if (element.is_way() && closed(element.as_way())) {
			Ring ring;
			for (const auto &n : element.as_way().nodes)
				ring.push_back(node_point(n));
			result.push_back(std::move(ring));
		} else if (element.is_relation()) {
			for (const auto &member : element.as_relation().members) {
				if (member.role != ProcessedMemberRole::Outer || !closed(*member.way))
					continue;
				Ring ring;
				for (const auto &n : member.way->nodes)
					ring.push_back(node_point(n));
				result.push_back(std::move(ring));
			}
		}
		return result;
	}

	static bool on_ground(const tags_t &tags)
	{
		for (const char *key_name : {"tunnel", "bridge", "indoor"})
			if (const auto *value = tag(tags, key_name); value && *value != "no")
				return false;
		if (const auto *layer = tag(tags, "layer")) {
			int parsed = 0;
			const auto [end, error] =
					std::from_chars(layer->data(), layer->data() + layer->size(), parsed);
			if (error == std::errc{} && end == layer->data() + layer->size() &&
					parsed != 0)
				return false;
		}
		return true;
	}

	static bool is_area(const tags_t &tags)
	{
		const auto *parking = tag(tags, "parking");
		const bool garages =
				parking && (*parking == "garage_boxes" || *parking == "carports");
		const auto *highway = tag(tags, "highway");
		const auto *area = tag(tags, "area");
		const auto *amenity = tag(tags, "amenity");
		const auto *landuse = tag(tags, "landuse");
		return (highway && area && *area == "yes") ||
			   (amenity && (*amenity == "parking" || *amenity == "bus_station") &&
					   !tags.contains("building") && !garages) ||
			   (landuse && *landuse == "railway");
	}

	static bool is_water_area(const tags_t &tags)
	{
		const auto *natural = tag(tags, "natural");
		const auto *waterway = tag(tags, "waterway");
		const auto *landuse = tag(tags, "landuse");
		const auto *man_made = tag(tags, "man_made");
		const auto *leisure = tag(tags, "leisure");
		return tags.contains("water") ||
			   (natural && (*natural == "water" || *natural == "bay")) ||
			   (waterway && (*waterway == "riverbank" || *waterway == "dock")) ||
			   (landuse && *landuse == "reservoir") ||
			   (man_made && *man_made == "reservoir_covered") ||
			   (leisure && *leisure == "swimming_pool");
	}

	static Ring ring_of(const std::vector<ProcessedNode> &nodes)
	{
		Ring ring;
		ring.reserve(nodes.size());
		for (const auto &node : nodes)
			ring.push_back(node_point(node));
		return ring;
	}

	static Ring ring_of(const ProcessedWay &way) { return ring_of(way.nodes); }

	static WaterRegion water_region(const ProcessedElement &element)
	{
		WaterRegion region;
		if (element.is_way()) {
			if (closed(element.as_way()))
				region.outers.push_back(ring_of(element.as_way()));
		} else if (element.is_relation()) {
			std::vector<std::vector<ProcessedNode>> outers, inners;
			for (const auto &member : element.as_relation().members) {
				if (member.role == ProcessedMemberRole::Outer)
					outers.push_back(member.way->nodes);
				else if (member.role == ProcessedMemberRole::Inner)
					inners.push_back(member.way->nodes);
			}
			merge_way_segments(outers);
			merge_way_segments(inners);
			for (const auto &ring : outers)
				if (ring.size() >= 4 && ring.front().x == ring.back().x &&
						ring.front().z == ring.back().z)
					region.outers.push_back(ring_of(ring));
			for (const auto &ring : inners)
				if (ring.size() >= 4 && ring.front().x == ring.back().x &&
						ring.front().z == ring.back().z)
					region.inners.push_back(ring_of(ring));
		}
		return region;
	}

	static void append_lines(
			Paved &out, const ProcessedWay &way, double reach, bool driveway)
	{
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			out.lines.emplace_back(
					node_point(way.nodes[i - 1]), node_point(way.nodes[i]), reach);
			out.driveway.push_back(driveway);
		}
	}

	static void append_area(Paved &out, const ProcessedWay &way)
	{
		if (!closed(way))
			return;
		Ring ring;
		for (const auto &n : way.nodes)
			ring.push_back(node_point(n));
		out.areas.push_back(std::move(ring));
	}

	static Paved build(const std::vector<ProcessedElement> &existing, double scale)
	{
		Paved out;
		const double track_reach = std::max(1.5 * scale, 1.0) + .5;
		for (const auto &element : existing) {
			const auto &tags = element.tags();
			if (is_water_area(tags)) {
				auto region = water_region(element);
				if (!region.outers.empty())
					out.water_regions.push_back(std::move(region));
			}
			if (element.is_way()) {
				const auto *waterway = tag(tags, "waterway");
				if (waterway && waterways::is_channel_waterway(*waterway) &&
						!waterways::is_underground_waterway(tags)) {
					const double reach = waterways::capped_waterway_width(
												 *waterway, tags, std::nullopt) /
												 2 +
										 1;
					const auto &way = element.as_way();
					for (std::size_t i = 1; i < way.nodes.size(); ++i)
						out.water_channels.emplace_back(node_point(way.nodes[i - 1]),
								node_point(way.nodes[i]), reach);
				}
			}
			const auto *waterway = tag(tags, "waterway");
			const auto *man_made = tag(tags, "man_made");
			const auto *bridge = tag(tags, "bridge");
			const bool over_water =
					(waterway && (*waterway == "weir" || *waterway == "dam" ||
										 *waterway == "lock_gate")) ||
					(man_made && *man_made == "pier") || (bridge && *bridge != "no");
			if (over_water) {
				if (element.is_way()) {
					double reach = OVER_WATER_REACH;
					if (const auto *highway = tag(tags, "highway"))
						reach = highways::highway_block_range(*highway, tags, scale) + .5;
					append_lines(out, element.as_way(), reach, false);
				}
				auto rings = outers(element);
				out.areas.insert(out.areas.end(), rings.begin(), rings.end());
				continue;
			}
			const auto *leisure = tag(tags, "leisure");
			const auto *landuse = tag(tags, "landuse");
			if ((leisure && *leisure == "marina") || (landuse && *landuse == "harbour")) {
				auto rings = outers(element);
				out.areas.insert(out.areas.end(), rings.begin(), rings.end());
				continue;
			}
			if (!on_ground(tags))
				continue;
			if (not_a_garden(tags)) {
				auto rings = outers(element);
				out.not_gardens.insert(out.not_gardens.end(), rings.begin(), rings.end());
			} else if (landuse && (*landuse == "retail" || *landuse == "commercial" ||
										  *landuse == "industrial")) {
				auto rings = outers(element);
				out.no_pools.insert(out.no_pools.end(), rings.begin(), rings.end());
			}
			if (element.is_way()) {
				const auto &way = element.as_way();
				if (is_area(tags) && closed(way)) {
					append_area(out, way);
					continue;
				}
				const auto *railway = tag(tags, "railway");
				if (railway && std::find(TRACKS.begin(), TRACKS.end(), *railway) !=
									   TRACKS.end()) {
					append_lines(out, way, track_reach, false);
					continue;
				}
				const auto *highway = tag(tags, "highway");
				if (!highway || *highway == "proposed" || *highway == "abandoned" ||
						*highway == "razed" || *highway == "disused" ||
						*highway == "platform")
					continue;
				const auto *service = tag(tags, "service");
				const bool is_driveway =
						*highway == "service" && service && *service == "driveway";
				append_lines(out, way,
						highways::highway_block_range(*highway, tags, scale) + .5,
						is_driveway);
			} else if (element.is_relation() && is_area(tags)) {
				for (const auto &member : element.as_relation().members)
					if (member.role == ProcessedMemberRole::Outer)
						append_area(out, *member.way);
			}
		}
		for (std::size_t i = 0; i < out.lines.size(); ++i) {
			const auto &[a, b, reach] = out.lines[i];
			const auto cell = [](double v) {
				return static_cast<int>(std::floor(v / PAVED_CELL));
			};
			const int x0 = cell(std::min(a.first, b.first) - reach);
			const int x1 = cell(std::max(a.first, b.first) + reach);
			const int z0 = cell(std::min(a.second, b.second) - reach);
			const int z1 = cell(std::max(a.second, b.second) + reach);
			for (int x = x0; x <= x1; ++x)
				for (int z = z0; z <= z1; ++z)
					out.buckets[key(x, z)].push_back(i);
		}
		return out;
	}

	template <typename F>
	void visit_lines(double x, double z, F &&visit) const
	{
		const auto it = buckets.find(key(static_cast<int>(std::floor(x / PAVED_CELL)),
				static_cast<int>(std::floor(z / PAVED_CELL))));
		if (it == buckets.end())
			return;
		for (const auto index : it->second) {
			const auto &[a, b, reach] = lines[index];
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double length2 = dx * dx + dz * dz;
			const double t =
					length2 > 0 ? std::clamp(((x - a.first) * dx + (z - a.second) * dz) /
													 length2,
										  0.0, 1.0)
								: 0.0;
			if (std::hypot(x - a.first - t * dx, z - a.second - t * dz) <= reach)
				visit(index);
		}
	}

	bool covers(double x, double z) const
	{
		bool road = false;
		visit_lines(x, z, [&](std::size_t i) { road |= !driveway[i]; });
		return road || std::any_of(areas.begin(), areas.end(), [&](const Ring &ring) {
			return ring_contains(ring, x, z);
		});
	}

	bool off_gardens(double x, double z, bool pool) const
	{
		return std::any_of(not_gardens.begin(), not_gardens.end(),
					   [&](const Ring &ring) { return ring_contains(ring, x, z); }) ||
			   (pool && std::any_of(
								no_pools.begin(), no_pools.end(), [&](const Ring &ring) {
									return ring_contains(ring, x, z);
								}));
	}

	bool mapped_water(double x, double z) const
	{
		for (const auto &region : water_regions) {
			const bool inside_outer =
					std::any_of(region.outers.begin(), region.outers.end(),
							[&](const Ring &ring) { return ring_contains(ring, x, z); });
			if (inside_outer &&
					!std::any_of(region.inners.begin(), region.inners.end(),
							[&](const Ring &ring) { return ring_contains(ring, x, z); }))
				return true;
		}
		for (const auto &[a, b, reach] : water_channels) {
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double length2 = dx * dx + dz * dz;
			const double t =
					length2 > 0 ? std::clamp(((x - a.first) * dx + (z - a.second) * dz) /
													 length2,
										  0.0, 1.0)
								: 0.0;
			if (std::hypot(x - a.first - t * dx, z - a.second - t * dz) <= reach + .71)
				return true;
		}
		return false;
	}

	bool mostly_on_driveway(const std::vector<ProcessedNode> &nodes) const
	{
		if (nodes.empty())
			return false;
		Ring points;
		int x0 = nodes.front().x, x1 = x0, z0 = nodes.front().z, z1 = z0;
		for (const auto &node : nodes) {
			points.emplace_back(node.x, node.z);
			x0 = std::min(x0, node.x);
			x1 = std::max(x1, node.x);
			z0 = std::min(z0, node.z);
			z1 = std::max(z1, node.z);
		}
		std::size_t total = 0, on = 0;
		for (int z = z0; z <= z1; ++z)
			for (int x = x0; x <= x1; ++x) {
				if (!in_convex(points, x, z))
					continue;
				++total;
				bool driveway_here = false;
				visit_lines(x + .5, z + .5,
						[&](std::size_t i) { driveway_here |= driveway[i]; });
				on += driveway_here;
			}
		return 2 * on > total;
	}
};

std::array<int, 4> bbox_of(const ProcessedWay &way)
{
	if (way.nodes.empty())
		return {};
	std::array<int, 4> box{
			way.nodes[0].x, way.nodes[0].z, way.nodes[0].x, way.nodes[0].z};
	for (const auto &node : way.nodes) {
		box[0] = std::min(box[0], node.x);
		box[1] = std::min(box[1], node.z);
		box[2] = std::max(box[2], node.x);
		box[3] = std::max(box[3], node.z);
	}
	return box;
}
} // namespace

std::vector<ProcessedElement> candidates(const Ground &ground, const ::XZBBox &bbox)
{
	if (!ground.has_survey())
		return {};
	const int x0 = bbox.min_x(), z0 = bbox.min_z();
	const auto width64 = std::int64_t(bbox.max_x()) - x0 + 1;
	const auto height64 = std::int64_t(bbox.max_z()) - z0 + 1;
	if (width64 <= 0 || height64 <= 0 ||
			std::uint64_t(width64) >
					std::numeric_limits<std::size_t>::max() / std::uint64_t(height64))
		return {};
	const auto width = static_cast<std::size_t>(width64);
	const auto height = static_cast<std::size_t>(height64);
	std::vector<ProcessedElement> out;
	std::uint64_t next_id = ID_BASE;
	for (const auto kind : {Kind::Shed, Kind::Pool}) {
		std::vector<std::uint8_t> mask(width * height, 0);
		std::vector<float> heights(width * height, 0);
		for (std::size_t z = 0; z < height; ++z)
			for (std::size_t x = 0; x < width; ++x)
				if (bbox.contains(x0 + static_cast<int>(x), z0 + static_cast<int>(z)))
					if (const auto cell = ground.survey_cell(
								XZPoint(static_cast<int>(x), static_cast<int>(z)));
							cell && cell->kind == kind) {
						const auto i = z * width + x;
						mask[i] = 1;
						heights[i] = cell->height;
					}
		auto [labels, count] = grid::label(mask, width, height, false);
		if (!count)
			continue;
		const auto shape = grid::shapes(labels, count, width);
		std::vector<std::vector<std::pair<int, int>>> cells(count + 1);
		std::vector<std::vector<float>> component_heights(count + 1);
		for (std::size_t i = 0; i < labels.size(); ++i)
			if (labels[i]) {
				cells[labels[i]].emplace_back(
						static_cast<int>(i % width), static_cast<int>(i / width));
				component_heights[labels[i]].push_back(heights[i]);
			}
		for (std::uint32_t label = 1; label <= count; ++label) {
			if (shape[label].area < 4.0)
				continue;
			const auto ring = grid::hull(cells[label]);
			if (ring.size() < 3)
				continue;
			auto &hs = component_heights[label];
			std::sort(hs.begin(), hs.end());
			const float tall = std::clamp(hs[hs.size() / 2], 2.0f, 5.0f);
			ProcessedWay way{};
			way.id = next_id;
			if (kind == Kind::Shed) {
				way.tags.emplace("building",
						shape[label].area >= 15 && tall >= 2.2f ? "garage" : "shed");
				char height[16];
				std::snprintf(height, sizeof(height), "%.1f", tall);
				way.tags.emplace("height", height);
			} else {
				way.tags.emplace("leisure", "swimming_pool");
				way.tags.emplace("access", "private");
			}
			way.tags.emplace("source", "garden_detail");
			for (std::size_t i = 0; i < ring.size(); ++i) {
				const auto [x, z] = ring[i];
				way.nodes.push_back({next_id + 1 + i, {}, x0 + x, z0 + z});
			}
			auto first = way.nodes.front();
			way.nodes.push_back(first);
			out.emplace_back(way);
			next_id += 1 << 8;
		}
	}
	return out;
}

std::pair<std::vector<ProcessedElement>, std::size_t> merge(
		std::vector<ProcessedElement> generated,
		const std::vector<ProcessedElement> &existing, double scale,
		const std::function<bool(int, int)> &is_water)
{
	const auto paved = Paved::build(existing, scale);
	struct Blocker
	{
		std::array<int, 4> box;
		bool building;
	};
	std::vector<Blocker> blockers;
	for (const auto &element : existing) {
		const auto &tags = element.tags();
		const bool building = tags.contains("building") || tags.contains("building:part");
		const auto *leisure = tag(tags, "leisure");
		if (!building && (!leisure || *leisure != "swimming_pool"))
			continue;
		auto add = [&](const ProcessedWay &way) {
			if (way.nodes.size() >= 3)
				blockers.push_back({bbox_of(way), building});
		};
		if (element.is_way())
			add(element.as_way());
		else if (element.is_relation())
			for (const auto &member : element.as_relation().members)
				add(*member.way);
	}
	std::size_t on_paved = 0;
	std::vector<ProcessedElement> kept;
	kept.reserve(generated.size());
	for (auto &element : generated) {
		if (!element.is_way())
			continue;
		auto &way = std::get<ProcessedWay>(static_cast<variant_t &>(element));
		const bool pool = way.tags.contains("leisure");
		const auto box = bbox_of(way);
		Point centre{};
		for (const auto &node : way.nodes) {
			centre.first += node.x;
			centre.second += node.z;
		}
		const double n = std::max<std::size_t>(way.nodes.size(), 1);
		centre.first = centre.first / n + .5;
		centre.second = centre.second / n + .5;
		std::vector<Point> probes;
		for (const auto &node : way.nodes)
			probes.emplace_back(double(node.x) + .5, double(node.z) + .5);
		probes.push_back(centre);
		bool on_paving = std::any_of(probes.begin(), probes.end(), [&](const Point &p) {
			return paved.covers(p.first, p.second);
		}) || paved.mostly_on_driveway(way.nodes);
		const bool mapped =
				std::any_of(blockers.begin(), blockers.end(), [&](const Blocker &b) {
					const bool overlaps = box[0] <= b.box[2] && box[2] >= b.box[0] &&
										  box[1] <= b.box[3] && box[3] >= b.box[1];
					return overlaps && (b.building || pool);
				});
		if (on_paving && !mapped)
			++on_paved;
		const bool wet = std::any_of(probes.begin(), probes.end(), [&](const Point &p) {
			return paved.mapped_water(p.first, p.second) ||
				   (is_water && is_water(static_cast<int>(std::floor(p.first)),
										static_cast<int>(std::floor(p.second))));
		});
		if (!on_paving && !mapped && !wet &&
				!paved.off_gardens(centre.first, centre.second, pool))
			kept.emplace_back(std::move(element));
	}
	return {std::move(kept), on_paved};
}

} // namespace arnis::survey::elements
