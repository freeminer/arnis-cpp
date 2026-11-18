#include "parking_lots.h"

#include "../block_definitions.h"
#include "../bresenham.h"
#include "../element_processing/surfaces.h"
#include "../floodfill_cache.h"
#include "../structures/structures.h"
#include "relation_area.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace arnis::element_processing::parking_lots
{
namespace
{
constexpr int STALL_WIDTH = 4;
constexpr int STALL_LENGTH = 8;
constexpr int AISLE_WIDTH = 5;
constexpr int ROW_PERIOD = STALL_LENGTH + AISLE_WIDTH;

struct Grid
{
	double ox = 0.0;
	double oz = 0.0;
	// Rows run along u; v is the direction a parked car points toward.
	double ux = 1.0;
	double uz = 0.0;
	double vx = 0.0;
	double vz = 1.0;

	std::pair<double, double> local(int x, int z) const
	{
		const double dx = double(x) + .5 - ox;
		const double dz = double(z) + .5 - oz;
		return {dx * ux + dz * uz, dx * vx + dz * vz};
	}
	std::pair<int, int> cell(double a, double b) const
	{
		return {static_cast<int>(std::lround(ox + a * ux + b * vx - .5)),
				static_cast<int>(std::lround(oz + a * uz + b * vz - .5))};
	}
	static Grid for_way(const ProcessedWay &way)
	{
		Grid grid;
		if (way.nodes.empty())
			return grid;
		using Point = std::pair<double, double>;
		std::vector<Point> points;
		points.reserve(way.nodes.size());
		for (const auto &n : way.nodes)
			points.emplace_back(double(n.x) + .5, double(n.z) + .5);
		std::sort(points.begin(), points.end());
		points.erase(std::unique(points.begin(), points.end()), points.end());
		const auto cross = [](Point o, Point a, Point b) {
			return (a.first - o.first) * (b.second - o.second) -
				   (a.second - o.second) * (b.first - o.first);
		};
		std::vector<Point> hull;
		for (const auto &p : points) {
			while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0)
				hull.pop_back();
			hull.push_back(p);
		}
		const auto lower = hull.size();
		for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
			while (hull.size() > lower &&
					cross(hull[hull.size() - 2], hull.back(), *it) <= 0)
				hull.pop_back();
			hull.push_back(*it);
		}
		if (hull.size() > 1)
			hull.pop_back();
		// Match the minimum-area rectangle direction used by Rust, then snap only
		// near-axis lots so stall stripes remain continuous rather than stepped.
		double best_area = std::numeric_limits<double>::infinity();
		Point long_axis{1.0, 0.0};
		for (std::size_t i = 0; i < hull.size(); ++i) {
			const auto &a = hull[i], &b = hull[(i + 1) % hull.size()];
			const double dx = b.first - a.first, dz = b.second - a.second;
			const double len = std::hypot(dx, dz);
			if (len < 1e-6)
				continue;
			const Point e{dx / len, dz / len}, n{-e.second, e.first};
			double e0 = std::numeric_limits<double>::infinity(), e1 = -e0;
			double n0 = e0, n1 = -e0;
			for (const auto &p : hull) {
				const double ep = p.first * e.first + p.second * e.second;
				const double np = p.first * n.first + p.second * n.second;
				e0 = std::min(e0, ep);
				e1 = std::max(e1, ep);
				n0 = std::min(n0, np);
				n1 = std::max(n1, np);
			}
			const double area = (e1 - e0) * (n1 - n0);
			if (area < best_area) {
				best_area = area;
				long_axis = e1 - e0 >= n1 - n0 ? e : n;
			}
		}
		if (std::abs(long_axis.first) >= std::abs(long_axis.second)) {
			const double alignment = std::abs(long_axis.first) /
									 std::hypot(long_axis.first, long_axis.second);
			if (alignment >= std::cos(15.0 * 3.14159265358979323846 / 180.0))
				grid.ux = std::copysign(1.0, long_axis.first), grid.uz = 0.0;
		} else {
			const double alignment = std::abs(long_axis.second) /
									 std::hypot(long_axis.first, long_axis.second);
			if (alignment >= std::cos(15.0 * 3.14159265358979323846 / 180.0))
				grid.ux = 0.0, grid.uz = std::copysign(1.0, long_axis.second);
		}
		grid.vx = -grid.uz;
		grid.vz = grid.ux;
		double min_u = std::numeric_limits<double>::infinity();
		double min_v = min_u;
		for (const auto &n : way.nodes) {
			const double x = double(n.x) + .5, z = double(n.z) + .5;
			min_u = std::min(min_u, x * grid.ux + z * grid.uz);
			min_v = std::min(min_v, x * grid.vx + z * grid.vz);
		}
		grid.ox = grid.ux * min_u + grid.vx * min_v;
		grid.oz = grid.uz * min_u + grid.vz * min_v;
		return grid;
	}
};

bool is_closed(const ProcessedWay &way)
{
	return way.nodes.size() >= 4 && way.nodes.front().x == way.nodes.back().x &&
		   way.nodes.front().z == way.nodes.back().z;
}
} // namespace

bool is_surface_lot(const tags_t &tags)
{
	return parking_lot_surface(tags);
}

void generate_parking_lot(WorldEditor &editor, const ProcessedWay &way, const Args &args,
		const FloodFillCache &fills, const Holes *holes)
{
	if (!is_surface_lot(way.tags) || !is_closed(way))
		return;
	static const std::array<Block, 2> default_surface{
			block_definitions::GRAY_CONCRETE_POWDER, block_definitions::CYAN_TERRACOTTA};
	static const std::array<Block, 3> outline_surface{block_definitions::BLACK_CONCRETE,
			block_definitions::GRAY_CONCRETE_POWDER, block_definitions::CYAN_TERRACOTTA};
	static const std::array<Block, 5> lot_surface{block_definitions::BLACK_CONCRETE,
			block_definitions::GRAY_CONCRETE_POWDER, block_definitions::CYAN_TERRACOTTA,
			block_definitions::GRAY_CONCRETE, block_definitions::POLISHED_ANDESITE};
	std::span<const Block> palette(default_surface);
	if (const auto surface = way.tags.get("surface"); !surface.empty()) {
		const auto mapped = surfaces::get_blocks_for_surface(surface);
		if (!mapped.empty())
			palette = mapped;
	}
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		for (const auto &[x, y, z] : bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)y;
			if (!in_hole(holes, x, z))
				editor.set_block(surfaces::semirandom_surface(x, z, palette), x, 0, z,
						std::vector<Block>(
								outline_surface.begin(), outline_surface.end()),
						std::nullopt);
		}
	}
	const auto area = fills.get_or_compute(way, args.timeout);
	std::vector<std::pair<int, int>> lot;
	lot.reserve(area->size());
	for (const auto &[x, z] : *area) {
		if (editor.nested_area_owns(x, z) || in_hole(holes, x, z))
			continue;
		lot.emplace_back(x, z);
		editor.set_block(surfaces::semirandom_surface(x, z, palette), x, 0, z,
				std::vector<Block>(lot_surface.begin(), lot_surface.end()), std::nullopt);
	}
	if (lot.empty())
		return;
	const Grid grid = Grid::for_way(way);
	for (const auto &[x, z] : lot) {
		const auto [a, b] = grid.local(x, z);
		const auto floor_mod = [](int value, int modulus) {
			const int rem = value % modulus;
			return rem < 0 ? rem + modulus : rem;
		};
		const int across = floor_mod(static_cast<int>(std::floor(a)), STALL_WIDTH);
		const int along = floor_mod(static_cast<int>(std::floor(b)), ROW_PERIOD);
		const bool line = along < STALL_LENGTH ? (across == 0 || along == 0)
											   : along == STALL_LENGTH;
		if (line && !editor.on_carriageway(x, z))
			editor.set_block(block_definitions::WHITE_CONCRETE, x, 0, z,
					std::vector<Block>(lot_surface.begin(), lot_surface.end()),
					std::nullopt);
	}
	std::sort(lot.begin(), lot.end());
	const auto contains = [&](int x, int z) {
		return std::binary_search(lot.begin(), lot.end(), std::pair{x, z});
	};
	int min_x = lot.front().first, max_x = min_x;
	int min_z = lot.front().second, max_z = min_z;
	for (const auto &[x, z] : lot) {
		min_x = std::min(min_x, x);
		max_x = std::max(max_x, x);
		min_z = std::min(min_z, z);
		max_z = std::max(max_z, z);
	}
	double u0 = std::numeric_limits<double>::infinity();
	double u1 = -u0;
	double v0 = u0, v1 = -u0;
	for (const auto &[x, z] : lot) {
		const auto [a, b] = grid.local(x, z);
		u0 = std::min(u0, a);
		u1 = std::max(u1, a);
		v0 = std::min(v0, b);
		v1 = std::max(v1, b);
	}
	for (int i = static_cast<int>(std::floor(u0 / STALL_WIDTH));
			i <= static_cast<int>(std::ceil(u1 / STALL_WIDTH)); ++i) {
		for (int j = static_cast<int>(std::floor(v0 / ROW_PERIOD));
				j <= static_cast<int>(std::ceil(v1 / ROW_PERIOD)); ++j) {
			const double a = double(i * STALL_WIDTH), b = double(j * ROW_PERIOD);
			const auto corner = grid.cell(a + .5, b + .5);
			if (i % 3 == 0 && j % 2 == 0 && contains(corner.first, corner.second) &&
					!editor.on_carriageway(corner.first, corner.second)) {
				const int x = corner.first, z = corner.second;
				editor.set_block(block_definitions::SMOOTH_STONE, x, 1, z, std::nullopt,
						std::nullopt);
				editor.set_block(block_definitions::ANDESITE_WALL, x, 2, z, std::nullopt,
						std::nullopt);
				for (int dy = 3; dy <= 5; ++dy)
					editor.set_block(block_definitions::IRON_BARS, x, dy, z, std::nullopt,
							std::nullopt);
				editor.set_block(block_definitions::SEA_LANTERN, x, 6, z, std::nullopt,
						std::nullopt);
				editor.set_block(block_definitions::SMOOTH_STONE_SLAB, x, 7, z,
						std::nullopt, std::nullopt);
			}
			bool bay_inside = true;
			for (int da = 0; bay_inside && da <= STALL_WIDTH; ++da)
				for (int db = 0; db <= STALL_LENGTH; ++db) {
					const auto cell = grid.cell(a + da, b + db);
					if (!contains(cell.first, cell.second) ||
							((da != 0 && da != STALL_WIDTH && db != 0 &&
									 db != STALL_LENGTH) &&
									editor.on_carriageway(cell.first, cell.second))) {
						bay_inside = false;
						break;
					}
				}
			if (bay_inside) {
				const auto center =
						grid.cell(a + STALL_WIDTH / 2.0, b + STALL_LENGTH / 2.0);
				structures::car::maybe_place_car(
						editor, center.first, center.second, grid.ux == 1.0 ? 0 : 1);
			}
		}
	}
}

} // namespace arnis::element_processing::parking_lots
