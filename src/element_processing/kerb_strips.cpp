#include "kerb_strips.h"

#include "../bresenham.h"
#include "../land_cover/land_cover.h"
#include "bridges.h"
#include "highways.h"
#include "road_width.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

namespace arnis::element_processing::kerb_strips
{
namespace
{
constexpr double MAX_GAP_METRES = 3.0;
constexpr double MIN_RUN_METRES = 6.0;

using Cell = std::pair<int, int>;

std::vector<Cell> centreline(const ProcessedWay &way)
{
	std::vector<Cell> points;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		for (const auto &p : bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			const Cell c{std::get<0>(p), std::get<2>(p)};
			if (points.empty() || points.back() != c)
				points.push_back(c);
		}
	}
	return points;
}

bool runs_as_sidewalk(const ProcessedWay &way, const std::string &type)
{
	const auto &tags = way.tags;
	const auto tag = [&](const char *key) { return tags.get(key); };
	const bool kind = type == "footway" ? tag("footway") != "crossing"
					  : type == "pedestrian"
							  ? tag("area") != "yes"
							  : type == "path" && tag("footway") == "sidewalk" &&
										tag("bicycle") != "designated";
	if (!kind || bridges::is_bridge_way(way))
		return false;
	for (const char *key : {"tunnel", "indoor"}) {
		const auto value = tag(key);
		if (!value.empty() && value != "no")
			return false;
	}
	for (const char *key : {"layer", "level"}) {
		const auto value = tag(key);
		if (!value.empty()) {
			try {
				if (std::stod(value) != 0.0)
					return false;
			} catch (...) {
				return false;
			}
		}
	}
	return true;
}

void stamp_way(CoordinateBitmap &bitmap, const ProcessedWay &way, const std::string &type,
		double scale)
{
	const int half = highways::highway_block_range(type, way.tags, scale);
	const auto sides = road_width::kerb_sides(type, way.tags, scale);
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const double vx = double(b.x - a.x), vz = double(b.z - a.z);
		const double len = std::hypot(vx, vz);
		for (const auto &p : bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			const int x = std::get<0>(p), z = std::get<2>(p);
			for (int dx = -half; dx <= half; ++dx)
				for (int dz = -half; dz <= half; ++dz) {
					if (len > 0.0) {
						const double across = dx * (-vz / len) + dz * (vx / len);
						if (across < -sides.first || across > sides.second)
							continue;
					}
					bitmap.set(x + dx, z + dz);
				}
		}
	}
}

struct Reach
{
	float edge{};
	float road{};
	std::size_t gap{};
	Cell from{};
};

std::optional<Reach> reach(Cell p, std::pair<float, float> n, float limit,
		std::size_t max_gap, const auto &road, const auto &walkway, const auto &open)
{
	bool own = true;
	Cell from = p, last = p;
	float edge = 0.0f;
	std::vector<Cell> gap;
	for (float t = .25f; t < limit; t += .25f) {
		const Cell c{static_cast<int>(std::lround(p.first + n.first * t)),
				static_cast<int>(std::lround(p.second + n.second * t))};
		if (c == last)
			continue;
		last = c;
		if (own && walkway(c.first, c.second)) {
			from = c;
			continue;
		}
		if (own) {
			own = false;
			edge = t;
		}
		if (road(c.first, c.second))
			return Reach{edge, t, gap.size(), from};
		if (walkway(c.first, c.second) || !open(c.first, c.second))
			return std::nullopt;
		if (std::find(gap.begin(), gap.end(), c) == gap.end()) {
			gap.push_back(c);
			if (gap.size() > max_gap)
				return std::nullopt;
		}
	}
	return std::nullopt;
}

bool green_cover(std::uint8_t cls)
{
	return cls == land_cover::LC_SHRUBLAND || cls == land_cover::LC_GRASSLAND ||
		   cls == land_cover::LC_CROPLAND || cls == land_cover::LC_WETLAND ||
		   cls == land_cover::LC_MANGROVES || cls == land_cover::LC_MOSS;
}

} // namespace

const std::vector<StripCell> &KerbStrips::of(std::uint64_t way) const
{
	static const std::vector<StripCell> empty;
	const auto it = by_way_.find(way);
	return it == by_way_.end() ? empty : it->second;
}

void KerbStrips::add(std::uint64_t way, StripCell cell)
{
	by_way_[way].push_back(cell);
}

std::vector<Cell> KerbStrips::cells() const
{
	std::vector<Cell> out;
	for (const auto &[id, cells] : by_way_) {
		(void)id;
		for (const auto &cell : cells)
			out.push_back(cell.cell);
	}
	return out;
}

KerbWays KerbWays::stamp(const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor, const ::XZBBox &bbox, double scale)
{
	KerbWays result(bbox);
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		const auto type = highways::surface_highway_type(way);
		if (!type)
			continue;
		if (highways::is_carriageway(*type)) {
			const auto tunnel = way.tags.get("tunnel");
			const bool at_grade =
					!bridges::is_bridge_way(way) && (tunnel.empty() || tunnel == "no");
			const bool ground_level =
					way.tags.get("layer").empty() || way.tags.get("layer") == "0";
			if (at_grade && ground_level &&
					highways::tunnel_renders_at_surface(way, &editor, scale))
				stamp_way(result.vehicles_, way, *type, scale);
			continue;
		}
		stamp_way(result.walkways_, way, *type, scale);
		if (runs_as_sidewalk(way, *type))
			result.sidewalks_.push_back(
					{&way, highways::highway_block_range(*type, way.tags, scale)});
	}
	return result;
}

KerbStrips KerbWays::strips(const WorldEditor &editor, double scale,
		const CoordinateBitmap &roads, const CoordinateBitmap &footprints,
		const CoordinateBitmap &blocked) const
{
	KerbStrips out;
	if (sidewalks_.empty())
		return out;
	const auto max_gap =
			static_cast<std::size_t>(std::max(1.0, std::round(MAX_GAP_METRES * scale)));
	const auto min_run =
			static_cast<std::size_t>(std::max(4.0, std::round(MIN_RUN_METRES * scale)));
	const auto road = [&](int x, int z) {
		return vehicles_.contains(x, z) && roads.contains(x, z);
	};
	const auto walkway = [&](int x, int z) { return walkways_.contains(x, z); };
	const auto open = [&](int x, int z) {
		return !roads.contains(x, z) && !footprints.contains(x, z) &&
			   !blocked.contains(x, z) && !editor.is_mapped_water(x, z) &&
			   !editor.under_mapped_crown(x, z);
	};
	for (const auto &sidewalk : sidewalks_) {
		const auto points = centreline(*sidewalk.way);
		if (points.size() < 2)
			continue;
		const float limit = float(sidewalk.half_width + static_cast<int>(max_gap) + 2);
		std::vector<StripCell> cells;
		for (const float side : {1.0f, -1.0f}) {
			struct Found
			{
				Reach r;
				std::pair<float, float> d, n;
			};
			std::vector<std::optional<Found>> found(points.size());
			for (std::size_t i = 0; i < points.size(); ++i) {
				const auto a = points[i > 3 ? i - 3 : 0];
				const auto b = points[std::min(i + 3, points.size() - 1)];
				const float dx = float(b.first - a.first),
							dz = float(b.second - a.second);
				const float len = std::hypot(dx, dz);
				if (len < 1e-3f)
					continue;
				const std::pair<float, float> d{dx / len, dz / len};
				const std::pair<float, float> n{-d.second * side, d.first * side};
				if (auto r = reach(points[i], n, limit, max_gap, road, walkway, open))
					found[i] = Found{*r, d, n};
			}
			std::size_t i = 0;
			while (i < found.size()) {
				const std::size_t start = i;
				while (i < found.size() && found[i])
					++i;
				const std::size_t run_size = i - start;
				if (run_size >= min_run) {
					std::size_t meets = 0;
					for (std::size_t k = start; k < i; ++k)
						meets += found[k]->r.gap == 0;
					const bool ragged = meets * 4 >= run_size;
					for (std::size_t k = start; k < i; ++k) {
						const auto &f = *found[k];
						const auto p = points[k];
						const int reach_out = static_cast<int>(std::ceil(f.r.road)) + 1;
						for (int dx = -reach_out; dx <= reach_out; ++dx)
							for (int dz = -reach_out; dz <= reach_out; ++dz) {
								const int x = p.first + dx, z = p.second + dz;
								const float across = dx * f.n.first + dz * f.n.second;
								const float along = dx * f.d.first + dz * f.d.second;
								if (std::abs(along) <= .75f && across > 0.0f &&
										across < f.r.road - .25f &&
										across >= f.r.edge - .75f && !walkway(x, z) &&
										!road(x, z) && open(x, z) &&
										(ragged ||
												!green_cover(editor.cover_class(x, z))))
									cells.push_back({{x, z}, f.r.from});
							}
					}
				}
				i = std::max(i, start + 1);
			}
		}
		std::sort(cells.begin(), cells.end(),
				[](const auto &a, const auto &b) { return a.cell < b.cell; });
		cells.erase(
				std::unique(cells.begin(), cells.end(),
						[](const auto &a, const auto &b) { return a.cell == b.cell; }),
				cells.end());
		for (const auto &cell : cells)
			out.add(sidewalk.way->id, cell);
	}
	return out;
}

} // namespace arnis::element_processing::kerb_strips
