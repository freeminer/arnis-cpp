#include <vector>

#include "../structures/structures.h"
#include "../water_depth.h"
#include <string>
#include <map>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

#include "../../../arnis_adapter.h"
#include "../bresenham.h"
#include "args.h"
namespace arnis
{

namespace water_areas
{

using BgPoint = boost::geometry::model::d2::point_xy<double>;
using BgLinestring = boost::geometry::model::linestring<BgPoint>;
using BgPolygon = boost::geometry::model::polygon<BgPoint>;
using BgBox = boost::geometry::model::box<BgPoint>;

// Forward declarations
static void merge_loopy_loops(std::vector<std::vector<ProcessedNode>> &loops);
static bool verify_closed_rings(const std::vector<std::vector<ProcessedNode>> &rings);

static void rect_fill(
		int min_x, int max_x, int min_z, int max_z, int ground_level, WorldEditor &editor)
{
	for (int x = min_x; x < max_x; ++x) {
		for (int z = min_z; z < max_z; ++z) {
			editor.set_block(block_definitions::WATER, x, ground_level, z, std::nullopt,
					std::nullopt);
		}
	}
}

static void inverse_floodfill_iterative(const std::pair<int, int> &min,
		const std::pair<int, int> &max, int ground_level,
		const std::vector<BgPolygon> &outers, const std::vector<BgPolygon> &inners,
		WorldEditor &editor)
{
	for (int x = min.first; x < max.first; ++x) {
		for (int z = min.second; z < max.second; ++z) {
			BgPoint p(static_cast<double>(x), static_cast<double>(z));

			bool in_outer = false;
			for (const auto &poly : outers) {
				if (boost::geometry::within(p, poly)) {
					in_outer = true;
					break;
				}
			}

			if (!in_outer) {
				continue;
			}

			bool in_inner = false;
			for (const auto &poly : inners) {
				if (boost::geometry::within(p, poly)) {
					in_inner = true;
					break;
				}
			}

			if (!in_inner) {
				editor.set_block(block_definitions::WATER, x, ground_level, z,
						std::nullopt, std::nullopt);
			}
		}
	}
}

static BgPolygon make_rectangle_polygon(int min_x, int max_x, int min_z, int max_z)
{
	BgPolygon poly;
	boost::geometry::append(poly.outer(), BgPoint(min_x, min_z));
	boost::geometry::append(poly.outer(), BgPoint(max_x, min_z));
	boost::geometry::append(poly.outer(), BgPoint(max_x, max_z));
	boost::geometry::append(poly.outer(), BgPoint(min_x, max_z));
	boost::geometry::append(poly.outer(), BgPoint(min_x, min_z)); // close polygon
	boost::geometry::correct(poly);
	return poly;
}

void inverse_floodfill_recursive(std::pair<int32_t, int32_t> min,
		std::pair<int32_t, int32_t> max, const std::vector<BgPolygon> &outers,
		const std::vector<BgPolygon> &inners, WorldEditor &editor)
{
	using namespace std::chrono;
	constexpr int64_t ITERATIVE_THRES = 10'000;

	if (min.first > max.first || min.second > max.second) {
		return;
	}

	int64_t width = static_cast<int64_t>(max.first) - static_cast<int64_t>(min.first);
	int64_t height = static_cast<int64_t>(max.second) - static_cast<int64_t>(min.second);

	if (width * height < ITERATIVE_THRES) {
		inverse_floodfill_iterative(min, max, 0, outers, inners, editor);
		return;
	}

	int32_t center_x = (min.first + max.first) / 2;
	int32_t center_z = (min.second + max.second) / 2;

	std::array<std::tuple<int32_t, int32_t, int32_t, int32_t>, 4> quadrants = {
			{{min.first, center_x, min.second, center_z},
					{center_x, max.first, min.second, center_z},
					{min.first, center_x, center_z, max.second},
					{center_x, max.first, center_z, max.second}}};

	for (const auto &[min_x, max_x, min_z, max_z] : quadrants) {
		auto rect = make_rectangle_polygon(min_x, max_x, min_z, max_z);

		bool any_outer_contains = std::any_of(
				outers.begin(), outers.end(), [&rect](const BgPolygon &outer) {
					return boost::geometry::within(rect, outer);
				});

		bool any_inner_intersects = std::any_of(
				inners.begin(), inners.end(), [&rect](const BgPolygon &inner) {
					return boost::geometry::intersects(inner, rect);
				});

		if (any_outer_contains && !any_inner_intersects) {
			rect_fill(min_x, max_x, min_z, max_z, 0, editor);
			continue;
		}

		std::vector<BgPolygon> outers_intersects;
		std::copy_if(outers.begin(), outers.end(), std::back_inserter(outers_intersects),
				[&rect](const BgPolygon &poly) {
					return boost::geometry::intersects(poly, rect);
				});

		std::vector<BgPolygon> inners_intersects;
		std::copy_if(inners.begin(), inners.end(), std::back_inserter(inners_intersects),
				[&rect](const BgPolygon &poly) {
					return boost::geometry::intersects(poly, rect);
				});

		if (!outers_intersects.empty()) {
			inverse_floodfill_recursive({min_x, min_z}, {max_x, max_z}, outers_intersects,
					inners_intersects, editor);
		}
	}
}

[[maybe_unused]] static void inverse_floodfill(int min_x, int min_z, int max_x, int max_z,
		const std::vector<std::vector<XZPoint>> &outers,
		const std::vector<std::vector<XZPoint>> &inners, WorldEditor &editor)
{
	std::vector<BgPolygon> inners_bg;
	inners_bg.reserve(inners.size());
	for (const auto &poly_pts : inners) {
		BgLinestring ls;
		ls.reserve(poly_pts.size());
		for (const auto &pt : poly_pts) {
			ls.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.z));
		}
		// ensure closed
		if (!ls.empty() &&
				(ls.front().x() != ls.back().x() || ls.front().y() != ls.back().y())) {
			ls.push_back(ls.front());
		}
		BgPolygon poly;
		boost::geometry::assign_points(poly, ls);
		boost::geometry::correct(poly);
		inners_bg.push_back(std::move(poly));
	}

	std::vector<BgPolygon> outers_bg;
	outers_bg.reserve(outers.size());
	for (const auto &poly_pts : outers) {
		BgLinestring ls;
		ls.reserve(poly_pts.size());
		for (const auto &pt : poly_pts) {
			ls.emplace_back(static_cast<double>(pt.x), static_cast<double>(pt.z));
		}
		if (!ls.empty() &&
				(ls.front().x() != ls.back().x() || ls.front().y() != ls.back().y())) {
			ls.push_back(ls.front());
		}
		BgPolygon poly;
		boost::geometry::assign_points(poly, ls);
		boost::geometry::correct(poly);
		outers_bg.push_back(std::move(poly));
	}

	inverse_floodfill_recursive(std::make_pair(min_x, min_z),
			std::make_pair(max_x, max_z), outers_bg, inners_bg, editor);
}

struct ScanlineEdge
{
	double x1;
	double z1;
	double x2;
	double z2;
};

static std::vector<ScanlineEdge> collect_ring_edges(const std::vector<XZPoint> &ring)
{
	std::vector<ScanlineEdge> edges;
	if (ring.size() < 2)
		return edges;
	for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
		const auto &a = ring[i];
		const auto &b = ring[i + 1];
		if (a.z != b.z) {
			edges.push_back({static_cast<double>(a.x), static_cast<double>(a.z),
					static_cast<double>(b.x), static_cast<double>(b.z)});
		}
	}
	const auto &first = ring.front();
	const auto &last = ring.back();
	if (first.z != last.z) {
		edges.push_back({static_cast<double>(last.x), static_cast<double>(last.z),
				static_cast<double>(first.x), static_cast<double>(first.z)});
	}
	return edges;
}

static std::vector<ScanlineEdge> collect_all_ring_edges(
		const std::vector<std::vector<XZPoint>> &rings)
{
	std::vector<ScanlineEdge> edges;
	for (const auto &ring : rings) {
		auto ring_edges = collect_ring_edges(ring);
		edges.insert(edges.end(), ring_edges.begin(), ring_edges.end());
	}
	return edges;
}

static std::vector<std::pair<int, int>> compute_scanline_spans(
		const std::vector<ScanlineEdge> &edges, double z, int min_x, int max_x)
{
	std::vector<double> xs;
	for (const auto &edge : edges) {
		if ((edge.z1 > z) != (edge.z2 > z)) {
			double t = (z - edge.z1) / (edge.z2 - edge.z1);
			xs.push_back(edge.x1 + t * (edge.x2 - edge.x1));
		}
	}
	if (xs.empty())
		return {};

	std::sort(xs.begin(), xs.end());
	std::vector<std::pair<int, int>> spans;
	for (std::size_t i = 0; i + 1 < xs.size(); i += 2) {
		int start = std::max(static_cast<int>(std::ceil(xs[i])), min_x);
		int end = std::min(static_cast<int>(std::floor(xs[i + 1])), max_x);
		if (start <= end)
			spans.emplace_back(start, end);
	}
	return spans;
}

static std::vector<std::pair<int, int>> union_spans(
		const std::vector<std::pair<int, int>> &a,
		const std::vector<std::pair<int, int>> &b)
{
	if (a.empty())
		return b;
	if (b.empty())
		return a;
	std::vector<std::pair<int, int>> all;
	all.reserve(a.size() + b.size());
	all.insert(all.end(), a.begin(), a.end());
	all.insert(all.end(), b.begin(), b.end());
	std::sort(all.begin(), all.end());

	std::vector<std::pair<int, int>> out;
	auto current = all.front();
	for (std::size_t i = 1; i < all.size(); ++i) {
		if (all[i].first <= current.second + 1) {
			current.second = std::max(current.second, all[i].second);
		} else {
			out.push_back(current);
			current = all[i];
		}
	}
	out.push_back(current);
	return out;
}

static std::vector<std::pair<int, int>> subtract_spans(
		const std::vector<std::pair<int, int>> &a,
		const std::vector<std::pair<int, int>> &b)
{
	if (b.empty())
		return a;
	std::vector<std::pair<int, int>> out;
	std::size_t bi = 0;
	for (const auto &[a_start, a_end] : a) {
		int pos = a_start;
		while (bi < b.size() && b[bi].second < a_start)
			++bi;
		for (std::size_t j = bi; j < b.size() && b[j].first <= a_end; ++j) {
			if (b[j].first > pos)
				out.emplace_back(pos, std::min(b[j].first - 1, a_end));
			pos = std::max(pos, b[j].second + 1);
		}
		if (pos <= a_end)
			out.emplace_back(pos, a_end);
	}
	return out;
}

static bool spans_contain(const std::vector<std::pair<int, int>> &spans, int x)
{
	return std::any_of(spans.begin(), spans.end(),
			[x](const auto &span) { return x >= span.first && x <= span.second; });
}

// Rust parity: water_areas.rs::still_surface_level.  ESA land-cover data often
// sees a reservoir at one level while the OSM outline includes its exposed
// banks.  Resolve one surface for a sufficiently large, consistently-levelled
// water body instead of following each terrain cell independently.
static std::optional<int> still_surface_level(WorldEditor &editor, int min_x, int min_z,
		int max_x, int max_z, const std::vector<std::vector<XZPoint>> &outers,
		const std::vector<std::vector<XZPoint>> &inners)
{
	if (!editor.ground || !editor.ground->has_land_cover())
		return std::nullopt;
	constexpr std::size_t max_samples = 250000;
	constexpr std::size_t min_lc_columns = 64;
	constexpr double min_lc_share = .3;
	const auto width = static_cast<std::int64_t>(max_x) - min_x + 1;
	const auto height = static_cast<std::int64_t>(max_z) - min_z + 1;
	if (width <= 0 || height <= 0)
		return std::nullopt;
	const int stride = std::max(
			1, static_cast<int>(std::ceil(
					   std::sqrt(static_cast<double>(width * height) / max_samples))));

	std::vector<std::vector<ScanlineEdge>> outer_edge_groups;
	outer_edge_groups.reserve(outers.size());
	for (const auto &outer : outers)
		outer_edge_groups.push_back(collect_ring_edges(outer));
	const auto inner_edges = collect_all_ring_edges(inners);
	std::size_t total = 0;
	std::vector<int> levels;
	for (int z = min_z; z <= max_z; z += stride) {
		std::vector<std::pair<int, int>> outer_spans;
		for (const auto &edges : outer_edge_groups)
			outer_spans = union_spans(outer_spans,
					compute_scanline_spans(edges, static_cast<double>(z), min_x, max_x));
		if (outer_spans.empty())
			continue;
		auto spans = inner_edges.empty()
							 ? outer_spans
							 : subtract_spans(outer_spans,
									   compute_scanline_spans(inner_edges,
											   static_cast<double>(z), min_x, max_x));
		for (const auto &[start, end] : spans) {
			const int remainder = ((start % stride) + stride) % stride;
			int x = start + (stride - remainder) % stride;
			for (; x <= end; x += stride) {
				++total;
				const XZPoint relative{
						x - editor.mg->node_min.X, z - editor.mg->node_min.Z};
				if (editor.ground->cover_class(relative) == land_cover::LC_WATER)
					levels.push_back(editor.get_ground_level(x, z));
			}
		}
	}
	if (levels.size() < min_lc_columns || levels.size() < min_lc_share * total)
		return std::nullopt;
	std::sort(levels.begin(), levels.end());
	const auto q1 = levels.size() / 4;
	const auto q3 = levels.size() * 3 / 4;
	if (levels[q1] != levels[q3])
		return std::nullopt;
	return levels[levels.size() / 2];
}

static void scanline_fill_water(int min_x, int min_z, int max_x, int max_z,
		const std::vector<std::vector<XZPoint>> &outers,
		const std::vector<std::vector<XZPoint>> &inners, WorldEditor &editor,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint, std::optional<int> still_surface)
{
	std::vector<std::vector<ScanlineEdge>> outer_edge_groups;
	outer_edge_groups.reserve(outers.size());
	for (const auto &outer : outers)
		outer_edge_groups.push_back(collect_ring_edges(outer));
	auto inner_edges = collect_all_ring_edges(inners);
	auto filled_at = [&](int x, int z) {
		std::vector<std::pair<int, int>> outer_spans;
		for (const auto &edges : outer_edge_groups)
			outer_spans = union_spans(
					outer_spans, compute_scanline_spans(edges, static_cast<double>(z),
										 min_x - 4, max_x + 4));
		if (outer_spans.empty() || !spans_contain(outer_spans, x))
			return false;
		if (inner_edges.empty())
			return true;
		return !spans_contain(compute_scanline_spans(inner_edges, static_cast<double>(z),
									  min_x - 4, max_x + 4),
				x);
	};

	for (int z = min_z; z <= max_z; ++z) {
		std::vector<std::pair<int, int>> outer_spans;
		for (const auto &edges : outer_edge_groups) {
			auto spans =
					compute_scanline_spans(edges, static_cast<double>(z), min_x, max_x);
			if (!spans.empty())
				outer_spans = union_spans(outer_spans, spans);
		}
		if (outer_spans.empty())
			continue;

		std::vector<std::pair<int, int>> fill_spans = outer_spans;
		if (!inner_edges.empty()) {
			auto inner_spans = compute_scanline_spans(
					inner_edges, static_cast<double>(z), min_x, max_x);
			if (!inner_spans.empty())
				fill_spans = subtract_spans(outer_spans, inner_spans);
		}

		for (const auto &[start, end] : fill_spans) {
			for (int x = start; x <= end; ++x) {
				if (road_mask.contains(x, z))
					continue;
				int ground_y = editor.get_ground_level(x, z);
				int water_y = still_surface.value_or(editor.get_water_level(x, z));
				if (still_surface && (ground_y > water_y || water_y - ground_y > 20))
					continue;
				if (!still_surface && ground_y > water_y) {
					// A DEM step fully inside the water polygon remains water; a
					// bank near its edge is left dry.  This is Rust's four-cell
					// interior-margin test, evaluated from the same scanline edges.
					if (!filled_at(x - 4, z) || !filled_at(x + 4, z) ||
							!filled_at(x, z - 4) || !filled_at(x, z + 4))
						continue;
					water_y = ground_y;
				}
				if (ground_y <= water_y) {
					// Rust fills down to terrain over a tunnel bore, but never
					// excavates or replaces the bore below the terrain surface.
					if (tunnel_footprint && tunnel_footprint->contains(x, z)) {
						for (int y = ground_y + 1; y <= water_y; ++y)
							editor.set_block_absolute(
									WATER, x, y, z, std::nullopt, std::nullopt);
						continue;
					}
					water_depth::carve_water_column(
							editor, x, z, water_y, bwf.depth_at(x, z), road_mask, bwf);
				}
			}
		}
	}
}

static void generate_water_areas(WorldEditor &editor,
		const std::vector<std::vector<ProcessedNode>> &outers,
		const std::vector<std::vector<ProcessedNode>> &inners,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint)
{
	// Calculate polygon bounding box to limit fill area
	int32_t poly_min_x = std::numeric_limits<int32_t>::max();
	int32_t poly_min_z = std::numeric_limits<int32_t>::max();
	int32_t poly_max_x = std::numeric_limits<int32_t>::min();
	int32_t poly_max_z = std::numeric_limits<int32_t>::min();

	for (const auto &outer : outers) {
		for (const auto &node : outer) {
			poly_min_x = std::min(poly_min_x, node.x);
			poly_min_z = std::min(poly_min_z, node.z);
			poly_max_x = std::max(poly_max_x, node.x);
			poly_max_z = std::max(poly_max_z, node.z);
		}
	}

	// If no valid bounds, nothing to fill
	if (poly_min_x == std::numeric_limits<int32_t>::max() ||
			poly_max_x == std::numeric_limits<int32_t>::min()) {
		return;
	}

	// Clamp to world bounds just in case
	auto [world_min_x, world_min_z] = editor.get_min_coords();
	auto [world_max_x, world_max_z] = editor.get_max_coords();
	int32_t min_x = std::max(poly_min_x, world_min_x);
	int32_t min_z = std::max(poly_min_z, world_min_z);
	int32_t max_x = std::min(poly_max_x, world_max_x);
	int32_t max_z = std::min(poly_max_z, world_max_z);

	std::vector<std::vector<XZPoint>> outers_xz;
	outers_xz.reserve(outers.size());
	for (const auto &outer : outers) {
		std::vector<XZPoint> v;
		v.reserve(outer.size());
		for (const auto &node : outer) {
			v.push_back(node.xz());
		}
		outers_xz.push_back(std::move(v));
	}

	std::vector<std::vector<XZPoint>> inners_xz;
	inners_xz.reserve(inners.size());
	for (const auto &inner : inners) {
		std::vector<XZPoint> v;
		v.reserve(inner.size());
		for (const auto &node : inner) {
			v.push_back(node.xz());
		}
		inners_xz.push_back(std::move(v));
	}

	const auto still_surface =
			still_surface_level(editor, min_x, min_z, max_x, max_z, outers_xz, inners_xz);
	scanline_fill_water(min_x, min_z, max_x, max_z, outers_xz, inners_xz, editor, bwf,
			road_mask, tunnel_footprint, still_surface);
	structures::boat::scatter_boats(editor, min_x, min_z, max_x, max_z);
}

static bool verify_closed_rings(const std::vector<std::vector<ProcessedNode>> &rings)
{
	bool valid = true;
	for (const auto &ring : rings) {
		if (ring.empty()) {
			continue;
		}

		const ProcessedNode &first = ring.front();
		const ProcessedNode &last = ring.back();

		// Check if ring is closed (by ID or proximity)
		bool is_closed = (first.id == last.id) || ([&]() {
			int32_t dx = std::abs(first.x - last.x);
			int32_t dz = std::abs(first.z - last.z);
			return dx <= 1 && dz <= 1;
		})();

		if (!is_closed) {
			std::cerr << "WARN: Disconnected ring" << std::endl;
			valid = false;
		}
	}
	return valid;
}

static void merge_loopy_loops(std::vector<std::vector<ProcessedNode>> &loops)
{
	std::vector<std::size_t> removed;
	std::vector<std::vector<ProcessedNode>> merged;

	for (std::size_t i = 0; i < loops.size(); ++i) {
		for (std::size_t j = 0; j < loops.size(); ++j) {
			if (i == j) {
				continue;
			}
			if (std::find(removed.begin(), removed.end(), i) != removed.end() ||
					std::find(removed.begin(), removed.end(), j) != removed.end()) {
				continue;
			}

			const std::vector<ProcessedNode> &x = loops[i];
			const std::vector<ProcessedNode> &y = loops[j];

			if (x.empty() || y.empty()) {
				continue;
			}

			if (x.front().id == x.back().id) {
				continue;
			}
			if (y.front().id == y.back().id) {
				continue;
			}

			if (x.front().id == y.front().id) {
				removed.push_back(i);
				removed.push_back(j);

				std::vector<ProcessedNode> r = x;
				std::reverse(r.begin(), r.end());
				r.insert(r.end(), y.begin() + 1, y.end());
				merged.push_back(std::move(r));
			} else if (x.back().id == y.back().id) {
				removed.push_back(i);
				removed.push_back(j);

				std::vector<ProcessedNode> r = x;
				r.insert(r.end(), y.rbegin() + 1, y.rend());
				std::reverse(r.begin() + x.size(),
						r.end()); // correct ordering after insert from reverse iterator
				merged.push_back(std::move(r));
			} else if (x.front().id == y.back().id) {
				removed.push_back(i);
				removed.push_back(j);

				std::vector<ProcessedNode> r = y;
				r.insert(r.end(), x.begin() + 1, x.end());
				merged.push_back(std::move(r));
			} else if (x.back().id == y.front().id) {
				removed.push_back(i);
				removed.push_back(j);

				std::vector<ProcessedNode> r = x;
				r.insert(r.end(), y.begin() + 1, y.end());
				merged.push_back(std::move(r));
			}
		}
	}

	std::sort(removed.begin(), removed.end());
	for (auto it = removed.rbegin(); it != removed.rend(); ++it) {
		if (*it < loops.size()) {
			loops.erase(loops.begin() + *it);
		}
	}

	std::size_t merged_len = merged.size();
	for (auto &m : merged) {
		loops.push_back(std::move(m));
	}

	if (merged_len > 0) {
		merge_loopy_loops(loops);
	}
}

void generate_water_area_from_way(WorldEditor &editor, const ProcessedWay &element,
		const water_depth::BigWaterField &bwf, const RoadMaskBitmap &road_mask,
		const RoadMaskBitmap *tunnel_footprint)
{
	std::vector<std::vector<ProcessedNode>> outers = {{element.nodes}};

	if (!verify_closed_rings(outers)) {
		std::cout << "Skipping way " << element.id << " due to invalid polygon"
				  << std::endl;
		return;
	}

	generate_water_areas(editor, outers, {}, bwf, road_mask, tunnel_footprint);
}

void generate_water_areas_from_relation(WorldEditor &editor,
		const ProcessedRelation &element, const water_depth::BigWaterField &bwf,
		const RoadMaskBitmap &road_mask, const RoadMaskBitmap *tunnel_footprint)
{
	// Check if this is a water relation (either with water tag or natural=water or natural=bay)
	bool is_water = element.tags.find("water") != element.tags.end() || ([&]() {
		auto it_nat = element.tags.find("natural");
		return it_nat != element.tags.end() &&
			   (it_nat->second == "water" || it_nat->second == "bay");
	})();

	if (!is_water) {
		return;
	}

	// Don't handle water below layer 0
	auto it_layer = element.tags.find("layer");
	if (it_layer != element.tags.end()) {
		try {
			int layer = std::stoi(it_layer->second);
			if (layer < 0) {
				return;
			}
		} catch (...) {
			// ignore parse errors
		}
	}

	std::vector<std::vector<ProcessedNode>> outers;
	std::vector<std::vector<ProcessedNode>> inners;

	for (const auto &mem : element.members) {
		if (mem.role == ProcessedMemberRole::Outer) {
			outers.push_back(mem.way.nodes);
		} else if (mem.role == ProcessedMemberRole::Inner) {
			inners.push_back(mem.way.nodes);
		}
	}

	// Preserve OSM-defined outer/inner roles without modification
	merge_loopy_loops(outers);

	// Filter: Keep only loops that are already closed OR can be closed within 1 block
	outers.erase(std::remove_if(outers.begin(), outers.end(),
						 [](const std::vector<ProcessedNode> &loop_nodes) {
							 if (loop_nodes.size() < 3) {
								 return true;
							 }
							 const ProcessedNode &first = loop_nodes.front();
							 const ProcessedNode &last = loop_nodes.back();
							 if (first.id == last.id) {
								 return false; // Already closed by ID
							 }
							 int32_t dx = std::abs(first.x - last.x);
							 int32_t dz = std::abs(first.z - last.z);
							 return !(dx <= 1 &&
									  dz <= 1); // Remove if not closable within 1 block
						 }),
			outers.end());

	// Now close the remaining loops that are within 1 block tolerance
	for (auto &loop_nodes : outers) {
		if (loop_nodes.size() >= 2) {
			const ProcessedNode &first = loop_nodes.front();
			const ProcessedNode &last = loop_nodes.back();
			if (first.id != last.id) {
				// Endpoints are close (within tolerance), close the loop
				loop_nodes.push_back(first);
			}
		}
	}

	// If no valid outer loops remain, skip the relation
	if (outers.empty()) {
		return;
	}

	// Verify again after filtering and closing
	if (!verify_closed_rings(outers)) {
		std::cout << "Skipping relation " << element.id << " due to invalid polygon"
				  << std::endl;
		return;
	}

	merge_loopy_loops(inners);
	if (!verify_closed_rings(inners)) {
		std::cout << "Skipping relation " << element.id << " due to invalid polygon"
				  << std::endl;
		return;
	}

	generate_water_areas(editor, outers, inners, bwf, road_mask, tunnel_footprint);
}

}
}
