#include "roofs.h"

#include "../floodfill_cache.h"
#include "survey.h"
#include "../../../arnis_ground.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace arnis::survey::roofs
{
namespace
{
constexpr int REGISTER = 2;
constexpr double MIN_COVER = 0.6;
constexpr double MIN_AREA_M2 = 30.0;
constexpr std::size_t MAX_BLOCKS = 40000;
constexpr float SATURATED_M = 51.0f;
constexpr double PLANE_RMS_M = 0.3;
constexpr double MIN_PITCH = 0.25;
constexpr double MAX_PITCH = 1.8;
constexpr double FLAT_SHARE = 0.15;
constexpr double FLAT_RISE_M = 0.5;
constexpr double RIM_M = 1.5;
constexpr double MIDDLE_M = 3.0;
constexpr double FLAT_SPAN_M = 1.5;
constexpr double PITCHED_SHARE = 0.35;
constexpr std::size_t MIN_SLOPED = 12;
constexpr double ONE_WAY = 0.8;
constexpr double TURN_DEG = 35.0;
constexpr double BAND_M = 2.0;
constexpr double WALL_MIN_M = 3.0;
constexpr std::size_t WALL_CELLS = 3;
constexpr double WALL_SHARE = 0.6;
constexpr double HIP_READ = 0.85;
constexpr double HIP_EAVES = 0.8;
constexpr double MAX_INWARD = 0.15;
constexpr double ONE_AXIS = 0.6;

using Point = std::pair<double, double>;

struct Wall
{
	Point start, dir, out;
	double len = 0.0;
};

struct Slope
{
	Point falls;
	std::size_t wall = 0;
	double along = 0.0;
	double off = 0.0;
};

struct Roof
{
	const char *shape;
	std::optional<double> direction;
};

bool inside(const std::vector<Point> &ring, double x, double z)
{
	if (ring.size() < 3)
		return false;
	bool odd = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto [xi, zi] = ring[i];
		const auto [xj, zj] = ring[j];
		if ((zi > z) != (zj > z) && x < (xj - xi) * (z - zi) / (zj - zi) + xi)
			odd = !odd;
		j = i;
	}
	return odd;
}

std::optional<Wall> make_wall(const std::vector<Point> &ring, std::size_t i)
{
	const Point a = ring[i], b = ring[(i + 1) % ring.size()];
	const double dx = b.first - a.first, dz = b.second - a.second;
	const double len = std::hypot(dx, dz);
	if (len < 1e-9)
		return std::nullopt;
	const Point dir{dx / len, dz / len};
	const Point mid{a.first + dx * .5, a.second + dz * .5};
	const double probe = std::min(.3, len * .25);
	const Point normal{dir.second, -dir.first};
	const Point out = inside(ring, mid.first + normal.first * probe,
							  mid.second + normal.second * probe)
							  ? Point{-normal.first, -normal.second}
							  : normal;
	return Wall{a, dir, out, len};
}

std::optional<std::tuple<std::size_t, double, double>> nearest_wall(
		const std::vector<Wall> &walls, double x, double z)
{
	std::optional<std::tuple<std::size_t, double, double>> best;
	for (std::size_t i = 0; i < walls.size(); ++i) {
		const auto &w = walls[i];
		const double rx = x - w.start.first, rz = z - w.start.second;
		const double t =
				std::clamp((rx * w.dir.first + rz * w.dir.second) / w.len, 0.0, 1.0);
		const double qx = w.start.first + w.dir.first * w.len * t;
		const double qz = w.start.second + w.dir.second * w.len * t;
		const double d = std::hypot(x - qx, z - qz);
		if (!best || d < std::get<2>(*best))
			best = std::make_tuple(i, t, d);
	}
	return best;
}

double det3(const std::array<std::array<double, 3>, 3> &m)
{
	return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
		   m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

std::optional<Point> clean_plane(const std::vector<std::array<double, 3>> &pts)
{
	const double n = static_cast<double>(pts.size());
	double su = 0, sw = 0, suu = 0, sww = 0, suw = 0;
	double sv = 0, suv = 0, swv = 0;
	for (const auto &[u, w, v] : pts) {
		su += u;
		sw += w;
		suu += u * u;
		sww += w * w;
		suw += u * w;
		sv += v;
		suv += u * v;
		swv += w * v;
	}
	const std::array<std::array<double, 3>, 3> m{
			{{{n, su, sw}}, {{su, suu, suw}}, {{sw, suw, sww}}}};
	const double det = det3(m);
	if (std::abs(det) < 1e-9)
		return std::nullopt;
	const std::array<double, 3> rhs{sv, suv, swv};
	auto solve = [&](std::size_t col) {
		auto mm = m;
		for (std::size_t row = 0; row < 3; ++row)
			mm[row][col] = rhs[row];
		return det3(mm) / det;
	};
	const double a = solve(0), b = solve(1), c = solve(2);
	double mse = 0.0;
	for (const auto &[u, w, v] : pts)
		mse += std::pow(v - (a + b * u + c * w), 2);
	mse /= n;
	return std::sqrt(mse) <= PLANE_RMS_M ? std::optional<Point>{Point{b, c}}
										 : std::nullopt;
}

double median(std::vector<double> &v)
{
	std::sort(v.begin(), v.end());
	return v[v.size() / 2];
}

double spread(std::vector<double> &v)
{
	if (v.empty())
		return 0.0;
	std::sort(v.begin(), v.end());
	auto at = [&](double q) {
		return v[static_cast<std::size_t>(std::llround((v.size() - 1) * q))];
	};
	return at(.9) - at(.1);
}

double dot(Point a, Point b)
{
	return a.first * b.first + a.second * b.second;
}

double bearing(Point v)
{
	constexpr double pi = 3.14159265358979323846;
	return std::fmod(std::atan2(v.first, -v.second) * 180.0 / pi + 360.0, 360.0);
}

std::optional<Roof> decide(const std::vector<Wall> &walls,
		const std::vector<Slope> &slopes, std::size_t cells, bool levelled, double scale)
{
	if (cells < 10)
		return std::nullopt;
	const double sloping = static_cast<double>(slopes.size()) / cells;
	if (sloping < FLAT_SHARE)
		return levelled ? std::optional<Roof>(Roof{"flat", std::nullopt}) : std::nullopt;
	if (sloping < PITCHED_SHARE || slopes.size() < MIN_SLOPED)
		return std::nullopt;
	const double n = static_cast<double>(slopes.size());
	Point mean{};
	for (const auto &s : slopes) {
		mean.first += s.falls.first;
		mean.second += s.falls.second;
	}
	if (std::hypot(mean.first / n, mean.second / n) >= ONE_WAY)
		return std::nullopt;
	constexpr double pi = 3.14159265358979323846;
	const double cos_turn = std::cos(TURN_DEG * pi / 180.0);
	const double sin_turn = std::sin(TURN_DEG * pi / 180.0);
	auto towards = [&](const Slope &s) { return dot(s.falls, walls[s.wall].out); };
	const auto inward_count = std::count_if(slopes.begin(), slopes.end(),
			[&](const Slope &s) { return towards(s) <= -cos_turn; });
	if (static_cast<double>(inward_count) / n > MAX_INWARD)
		return std::nullopt;
	struct Band
	{
		std::size_t all = 0, eaves = 0, gables = 0;
	};
	std::vector<Band> bands(walls.size());
	for (const auto &s : slopes) {
		if (s.off / scale > BAND_M || s.along < .25 || s.along > .75)
			continue;
		auto &b = bands[s.wall];
		++b.all;
		const double t = towards(s);
		if (t >= cos_turn)
			++b.eaves;
		else if (std::abs(t) <= sin_turn)
			++b.gables;
	}
	double long_m = 0, read_m = 0, eaves_m = 0, gables_m = 0;
	for (std::size_t i = 0; i < walls.size(); ++i) {
		const double len_m = walls[i].len / scale;
		if (len_m < WALL_MIN_M)
			continue;
		long_m += len_m;
		const auto &b = bands[i];
		if (b.all < WALL_CELLS)
			continue;
		read_m += len_m;
		if (static_cast<double>(b.eaves) >= WALL_SHARE * b.all)
			eaves_m += len_m;
		else if (static_cast<double>(b.gables) >= WALL_SHARE * b.all)
			gables_m += len_m;
	}
	if (read_m > 0 && read_m >= HIP_READ * long_m && gables_m == 0 &&
			eaves_m >= HIP_EAVES * read_m)
		return Roof{"hipped", std::nullopt};
	if (gables_m > 0 && eaves_m > 0) {
		double c2 = 0, s2 = 0;
		for (const auto &s : slopes) {
			const double a = 2.0 * std::atan2(s.falls.second, s.falls.first);
			c2 += std::cos(a);
			s2 += std::sin(a);
		}
		c2 /= n;
		s2 /= n;
		const double axis = .5 * std::atan2(s2, c2);
		const auto direction =
				std::hypot(c2, s2) >= ONE_AXIS
						? std::optional<double>(bearing({std::cos(axis), std::sin(axis)}))
						: std::nullopt;
		return Roof{"gabled", direction};
	}
	return std::nullopt;
}

std::optional<Roof> measure(
		const ProcessedWay &way, const Ground &ground, const ::XZBBox &bbox, double scale)
{
	if (!(scale > 0.0) || !std::isfinite(scale))
		return std::nullopt;
	std::vector<Point> ring;
	for (const auto &node : way.nodes)
		ring.emplace_back(node.x, node.z);
	if (ring.size() > 1 && ring.front() == ring.back())
		ring.pop_back();
	if (ring.size() < 3)
		return std::nullopt;
	double x0 = ring[0].first, x1 = x0, z0 = ring[0].second, z1 = z0;
	for (const auto &p : ring) {
		x0 = std::min(x0, p.first);
		x1 = std::max(x1, p.first);
		z0 = std::min(z0, p.second);
		z1 = std::max(z1, p.second);
	}
	constexpr int margin = REGISTER + 2;
	const int gx0 = static_cast<int>(std::floor(x0)) - margin;
	const int gz0 = static_cast<int>(std::floor(z0)) - margin;
	const auto w = static_cast<std::size_t>(
			std::max(1, static_cast<int>(std::ceil(x1)) + margin - gx0 + 1));
	const auto h = static_cast<std::size_t>(
			std::max(1, static_cast<int>(std::ceil(z1)) + margin - gz0 + 1));
	if (w > 4 * MAX_BLOCKS / h)
		return std::nullopt;
	auto at = [](int x, int z) { return Point{x - .5, z - .5}; };
	std::vector<std::pair<std::int64_t, std::int64_t>> inside_cells;
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto p = at(gx0 + static_cast<int>(x), gz0 + static_cast<int>(z));
			if (inside(ring, p.first, p.second))
				inside_cells.emplace_back(
						static_cast<std::int64_t>(x), static_cast<std::int64_t>(z));
		}
	const auto blocks = inside_cells.size();
	if (blocks > MAX_BLOCKS ||
			static_cast<double>(blocks) / (scale * scale) < MIN_AREA_M2)
		return std::nullopt;
	const double bpm = ground.blocks_per_meter();
	if (!(bpm > 0.0))
		return std::nullopt;
	const auto to_world = [&](int x, int z) {
		return XZPoint{gx0 + x - bbox.min_x(), gz0 + z - bbox.min_z()};
	};
	const double terrain_ref = ground.level_exact(
			to_world(static_cast<int>(w / 2), static_cast<int>(h / 2)));
	std::vector<double> heights(w * h, std::numeric_limits<double>::quiet_NaN());
	for (std::size_t z = 0; z < h; ++z)
		for (std::size_t x = 0; x < w; ++x) {
			const auto coord = to_world(static_cast<int>(x), static_cast<int>(z));
			const auto c = ground.laser_cell(coord);
			if (c && c->kind == Kind::Building && c->height > 1.5f &&
					c->height < SATURATED_M)
				heights[z * w + x] =
						c->height + (ground.level_exact(coord) - terrain_ref) / bpm;
		}
	auto roof_at = [&](std::int64_t x, std::int64_t z) {
		if (x < 0 || z < 0 || static_cast<std::size_t>(x) >= w ||
				static_cast<std::size_t>(z) >= h)
			return std::numeric_limits<double>::quiet_NaN();
		return heights[static_cast<std::size_t>(z) * w + static_cast<std::size_t>(x)];
	};
	std::size_t best_covered = 0;
	std::int64_t best_move = std::numeric_limits<std::int64_t>::max(), sx = 0, sz = 0;
	for (int dz = -REGISTER; dz <= REGISTER; ++dz)
		for (int dx = -REGISTER; dx <= REGISTER; ++dx) {
			std::size_t covered = 0;
			for (const auto &[x, z] : inside_cells)
				if (!std::isnan(roof_at(x + dx, z + dz)))
					++covered;
			const auto moved = std::abs(dx) + std::abs(dz);
			if (covered > best_covered ||
					(covered == best_covered && moved < best_move)) {
				best_covered = covered;
				best_move = moved;
				sx = dx;
				sz = dz;
			}
		}
	if (static_cast<double>(best_covered) < MIN_COVER * blocks)
		return std::nullopt;
	auto laid = [&](std::int64_t x, std::int64_t z) { return roof_at(x + sx, z + sz); };
	std::vector<Wall> walls;
	for (std::size_t i = 0; i < ring.size(); ++i)
		if (auto w = make_wall(ring, i))
			walls.push_back(*w);
	if (walls.empty())
		return std::nullopt;
	std::size_t cells = 0;
	std::vector<Slope> slopes;
	std::vector<double> rim, middle, all;
	for (const auto &[gx, gz] : inside_cells) {
		const auto here = laid(gx, gz);
		if (std::isnan(here))
			continue;
		const auto p = at(gx0 + static_cast<int>(gx), gz0 + static_cast<int>(gz));
		const auto nearest = nearest_wall(walls, p.first, p.second);
		all.push_back(here);
		if (nearest) {
			const double off = std::get<2>(*nearest) / scale;
			if (off <= RIM_M)
				rim.push_back(here);
			else if (off >= MIDDLE_M)
				middle.push_back(here);
		}
		std::vector<std::array<double, 3>> pts;
		for (int oz = -1; oz <= 1; ++oz)
			for (int ox = -1; ox <= 1; ++ox) {
				const auto v = laid(gx + ox, gz + oz);
				if (!std::isnan(v))
					pts.push_back({static_cast<double>(ox), static_cast<double>(oz), v});
			}
		if (pts.size() < 6)
			continue;
		++cells;
		const auto plane = clean_plane(pts);
		if (!plane)
			continue;
		const double rx = plane->first * scale, rz = plane->second * scale;
		const double pitch = std::hypot(rx, rz);
		if (pitch < MIN_PITCH || pitch > MAX_PITCH || !nearest)
			continue;
		slopes.push_back({{-rx / pitch, -rz / pitch}, std::get<0>(*nearest),
				std::get<1>(*nearest), std::get<2>(*nearest)});
	}
	const bool levelled = !rim.empty() && !middle.empty()
								  ? median(middle) - median(rim) <= FLAT_RISE_M
								  : spread(all) <= FLAT_SPAN_M;
	return decide(walls, slopes, cells, levelled, scale);
}

bool wants_roof(const ProcessedWay &way)
{
	return !way.tags.get("building").empty() && way.tags.get("building") != "no" &&
		   !way.tags.contains("building:part") && !way.tags.contains("roof:shape") &&
		   !way.tags.contains(SHAPE_HINT);
}
} // namespace

std::size_t apply(std::vector<ProcessedElement> &elements, const Ground &ground,
		const ::XZBBox &bbox, double scale)
{
	if (!ground.has_laser() || !ground.laser_roofs_wanted())
		return 0;
	std::size_t count = 0;
	for (auto &element : elements) {
		if (!element.is_way())
			continue;
		auto &way = element.as_way();
		if (!wants_roof(way))
			continue;
		const auto roof = measure(way, ground, bbox, scale);
		if (!roof)
			continue;
		way.tags[SHAPE_HINT] = roof->shape;
		// The C++ building pipeline's existing roof selectors consume the OSM
		// tags directly. Keep the private hint as the Rust-side source record,
		// and mirror its resolved value into the selectors only when unmapped.
		way.tags["roof:shape"] = roof->shape;
		if (roof->direction) {
			char degrees[16];
			std::snprintf(degrees, sizeof(degrees), "%.0f", *roof->direction);
			way.tags[DIRECTION_HINT] = degrees;
			way.tags["roof:direction"] = degrees;
		}
		++count;
	}
	return count;
}
} // namespace arnis::survey::roofs
