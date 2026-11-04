#include "sport_pitches.h"
#include "../unicode_text.h"
#include "../land_cover/land_cover.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <numeric>
#include <optional>
#include <set>
#include <string_view>

namespace arnis::sport_pitches
{
namespace
{
enum class PitchKind
{
	Soccer,
	AmericanFootball,
	Rugby,
	Baseball,
	Oval,
	Cricket,
	Basketball,
	Tennis,
	Generic
};

std::optional<PitchKind> pitch_kind(const ProcessedWay &way)
{
	auto it = way.tags.find("sport");
	if (it == way.tags.end())
		return std::nullopt;
	const std::string_view sports(it->second);
	bool bare_football = false;
	bool american_football = false;
	bool rugby = false;
	for (std::size_t begin = 0; begin <= sports.size();) {
		const auto separator = sports.find(';', begin);
		const auto end = separator == std::string_view::npos ? sports.size() : separator;
		const auto item = sports.substr(begin, end - begin);
		const auto [trim_begin, trim_end] = unicode_text::trim_whitespace(item);
		const auto sport = item.substr(trim_begin, trim_end - trim_begin);
		bare_football |= sport == "football";
		american_football |= sport == "american_football" || sport == "canadian_football";
		rugby |= sport == "rugby" || sport == "rugby_union" || sport == "rugby_league";
		if (separator == std::string_view::npos)
			break;
		begin = separator + 1;
	}
	if (bare_football && american_football)
		return PitchKind::AmericanFootball;
	if (bare_football && rugby)
		return PitchKind::Rugby;
	for (std::size_t begin = 0; begin <= sports.size();) {
		const auto separator = sports.find(';', begin);
		const auto end = separator == std::string_view::npos ? sports.size() : separator;
		const auto item = sports.substr(begin, end - begin);
		const auto [trim_begin, trim_end] = unicode_text::trim_whitespace(item);
		const auto sport = item.substr(trim_begin, trim_end - trim_begin);
		if (sport == "soccer" || sport == "football")
			return PitchKind::Soccer;
		if (sport == "american_football" || sport == "canadian_football")
			return PitchKind::AmericanFootball;
		if (sport == "rugby" || sport == "rugby_union" || sport == "rugby_league")
			return PitchKind::Rugby;
		if (sport == "baseball" || sport == "softball")
			return PitchKind::Baseball;
		if (sport == "australian_football" || sport == "gaelic_games" ||
				sport == "gaelic_football" || sport == "hurling")
			return PitchKind::Oval;
		if (sport == "cricket")
			return PitchKind::Cricket;
		if (sport == "basketball")
			return PitchKind::Basketball;
		if (sport == "tennis" || sport == "paddle_tennis" || sport == "padel")
			return PitchKind::Tennis;
		if (sport == "handball" || sport == "futsal" || sport == "hockey" ||
				sport == "field_hockey" || sport == "ice_hockey" ||
				sport == "volleyball" || sport == "beachvolleyball" ||
				sport == "badminton" || sport == "netball" || sport == "korfball" ||
				sport == "team_handball" || sport == "multi")
			return PitchKind::Generic;
		if (separator == std::string_view::npos)
			break;
		begin = separator + 1;
	}
	return std::nullopt;
}

using Point = std::pair<double, double>;

std::vector<Point> hull_ccw_impl(const std::vector<std::pair<int, int>> &points)
{
	std::vector<Point> sorted;
	sorted.reserve(points.size());
	for (const auto &[x, z] : points)
		sorted.emplace_back(x, z);
	std::sort(sorted.begin(), sorted.end());
	sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
	if (sorted.size() < 3)
		return sorted;
	auto cross = [](Point o, Point a, Point b) {
		return (a.first - o.first) * (b.second - o.second) -
			   (a.second - o.second) * (b.first - o.first);
	};
	std::vector<Point> hull;
	for (const auto &p : sorted) {
		while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), p) <= 0.0)
			hull.pop_back();
		hull.push_back(p);
	}
	const auto lower = hull.size() + 1;
	for (auto it = sorted.rbegin() + 1; it != sorted.rend(); ++it) {
		while (hull.size() >= lower &&
				cross(hull[hull.size() - 2], hull.back(), *it) <= 0.0)
			hull.pop_back();
		hull.push_back(*it);
	}
	hull.pop_back();
	return hull;
}

std::optional<std::pair<Point, Point>> home_plate(
		const ProcessedWay &way, const std::vector<std::pair<int, int>> &area)
{
	std::vector<std::pair<int, int>> outline;
	for (const auto &node : way.nodes)
		outline.emplace_back(node.x, node.z);
	const auto hull = hull_ccw_impl(outline);
	if (hull.size() < 3 || area.empty())
		return std::nullopt;
	struct Run
	{
		Point direction, start;
		double length;
	};
	std::vector<Run> runs;
	for (std::size_t i = 0; i < hull.size(); ++i) {
		const auto p = hull[i], q = hull[(i + 1) % hull.size()];
		const double dx = q.first - p.first, dz = q.second - p.second;
		const double length = std::hypot(dx, dz);
		if (length < 1e-6)
			continue;
		const Point direction{dx / length, dz / length};
		if (!runs.empty() &&
				runs.back().direction.first * direction.first +
								runs.back().direction.second * direction.second >
						.985) {
			auto &last = runs.back();
			const double total = last.length + length;
			Point blended{
					(last.direction.first * last.length + direction.first * length) /
							total,
					(last.direction.second * last.length + direction.second * length) /
							total};
			const double norm = std::hypot(blended.first, blended.second);
			last.direction = {blended.first / norm, blended.second / norm};
			last.length = total;
		} else {
			runs.push_back({direction, p, length});
		}
	}
	const double n = static_cast<double>(area.size());
	const Point centre{std::accumulate(area.begin(), area.end(), 0.0,
							   [](double sum, const auto &p) { return sum + p.first; }) /
							   n,
			std::accumulate(area.begin(), area.end(), 0.0, [](double sum, const auto &p) {
				return sum + p.second;
			}) / n};
	double longest = 0.0;
	for (const auto &run : runs)
		longest = std::max(longest, run.length);
	std::optional<std::pair<double, Point>> best;
	for (std::size_t i = 0; i < runs.size(); ++i) {
		for (std::size_t step = 1; step <= 2; ++step) {
			const std::size_t j = (i + step) % runs.size();
			if (i == j)
				continue;
			const auto &a = runs[i];
			const auto &b = runs[j];
			if (a.length < longest * .4 || b.length < longest * .4 ||
					std::abs(a.direction.first * b.direction.first +
							 a.direction.second * b.direction.second) > .35)
				continue;
			const double det = a.direction.first * b.direction.second -
							   a.direction.second * b.direction.first;
			if (std::abs(det) < 1e-9)
				continue;
			const double t =
					((b.start.first - a.start.first) * b.direction.second -
							(b.start.second - a.start.second) * b.direction.first) /
					det;
			const Point apex{a.start.first + a.direction.first * t,
					a.start.second + a.direction.second * t};
			const double score = std::min(a.length, b.length);
			if (!best || score > best->first)
				best = std::pair{score, apex};
		}
	}
	const Point apex = best ? best->second : hull[way.id % hull.size()];
	const double dx = centre.first - apex.first, dz = centre.second - apex.second;
	const double length = std::hypot(dx, dz);
	if (length < 5.0)
		return std::nullopt;
	return std::pair{apex, Point{dx / length, dz / length}};
}

void draw_diamond(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &area, Block surface)
{
	const auto plate = home_plate(way, area);
	if (!plate)
		return;
	const auto [home, direction] = *plate;
	const auto [hx, hz] = home;
	const auto [ux, uz] = direction;
	const double wx = -uz, wz = ux;
	const std::set<std::pair<int, int>> cells(area.begin(), area.end());
	const double scale = editor.scale();
	double reach = 0.0;
	for (const auto &[x, z] : area)
		reach = std::max(reach, std::hypot(x - hx, z - hz));
	if (reach < 30.0 * scale)
		return;
	const double base = std::clamp(reach * .27, 12.0 * scale, 27.4 * scale);
	const double s2 = std::sqrt(.5), mound = base * .672;
	const Block clay = TERRACOTTA;
	const std::optional<Block> stripes =
			(surface == GRASS_BLOCK || surface == GREEN_STAINED_HARDENED_CLAY)
					? std::optional<Block>(MOSS_BLOCK)
					: std::nullopt;
	const double band = std::max(3.0, std::round(base / 5.0));
	const int track = std::max(2, static_cast<int>(std::lround(base * .16)));
	auto near_edge = [&](int x, int z, int radius) {
		for (int dz = -radius; dz <= radius; ++dz)
			for (int dx = -radius; dx <= radius; ++dx)
				if (dx * dx + dz * dz <= radius * radius &&
						!cells.contains({x + dx, z + dz}))
					return true;
		return false;
	};
	for (const auto &[x, z] : area) {
		const double px = x - hx, pz = z - hz;
		const double along = px * ux + pz * uz, across = px * wx + pz * wz;
		const double s = (along - across) * s2, t = (along + across) * s2;
		const double mound_d = std::hypot(along - mound, across),
					 home_r = std::hypot(along, across);
		const bool fair = s >= -.5 && t >= -.5;
		const bool foul = fair && (std::abs(s) <= .5 || std::abs(t) <= .5);
		const bool inner_grass = s > 1.5 && t > 1.5 && s < base - 1.5 && t < base - 1.5;
		const bool skin = fair && mound_d <= base * 1.055 && along > 0.0;
		const bool path = (std::abs(s) <= 1.5 || std::abs(t) <= 1.5) && s <= base + 1.5 &&
						  t <= base + 1.5 && s > -2.0 && t > -2.0;
		const bool dirt = home_r <= base * .145 || mound_d <= base * .1 ||
						  (skin && !inner_grass) || path;
		const bool warning = fair && home_r > base * 1.6 && near_edge(x, z, track);
		std::optional<Block> block;
		if (foul && home_r > base * .145)
			block = WHITE_CONCRETE;
		else if (dirt || warning)
			block = clay;
		else if (stripes && static_cast<long long>(std::floor(along / band)) % 2 != 0)
			block = stripes;
		if (block)
			editor.set_block(*block, x, 0, z, std::vector<Block>{surface}, std::nullopt);
	}
	auto at = [&](double a, double c) {
		return std::pair{static_cast<int>(std::lround(hx + a * ux + c * wx)),
				static_cast<int>(std::lround(hz + a * uz + c * wz))};
	};
	for (const auto &[x, z] : std::array{at(0, 0), at(base * s2, -base * s2),
				 at(base * 2 * s2, 0), at(base * s2, base * s2), at(mound, 0)})
		if (cells.contains({x, z}))
			editor.set_block(WHITE_CONCRETE, x, 0, z, std::nullopt, std::vector<Block>{});
	if (reach >= 85.0 * scale)
		for (const auto &[x, z] : area) {
			const double px = x - hx, pz = z - hz;
			const double along = px * ux + pz * uz, across = px * wx + pz * wz;
			if (along - std::abs(across) >= -.5 &&
					std::hypot(along, across) > base * 2.0 && near_edge(x, z, 1))
				for (int y = 1; y <= 3; ++y)
					editor.set_block(GREEN_CONCRETE, x, y, z, std::nullopt, std::nullopt);
		}
}

std::optional<Block> default_surface_impl(const tags_t &tags)
{
	const auto sport = tags.get("sport");
	for (std::size_t begin = 0; begin <= sport.size();) {
		const auto sep = sport.find(';', begin);
		const auto end = sep == std::string::npos ? sport.size() : sep;
		const std::string_view item(sport.data() + begin, end - begin);
		const auto [a, b] = unicode_text::trim_whitespace(item);
		const auto name = item.substr(a, b - a);
		if (name == "beachvolleyball" || name == "beach_volleyball" ||
				name == "beachhandball" || name == "beach_soccer")
			return SAND;
		if (name == "equestrian" || name == "horse_racing")
			return COARSE_DIRT;
		if (name == "athletics" || name == "running")
			return RED_TERRACOTTA;
		if (name == "boules" || name == "petanque" || name == "bocce")
			return GRAVEL;
		if (name == "skateboard")
			return SMOOTH_STONE;
		if (sep == std::string::npos)
			break;
		begin = sep + 1;
	}
	return std::nullopt;
}

std::optional<std::pair<double, double>> longest_edge_dir_impl(const ProcessedWay &way)
{
	double best = 0.0, bx = 0.0, bz = 0.0;
	for (size_t i = 1; i < way.nodes.size(); ++i) {
		const double dx = way.nodes[i].x - way.nodes[i - 1].x;
		const double dz = way.nodes[i].z - way.nodes[i - 1].z;
		const double len2 = dx * dx + dz * dz;
		if (len2 > best) {
			best = len2;
			bx = dx;
			bz = dz;
		}
	}
	if (best < 1.0)
		return std::nullopt;
	const double len = std::sqrt(best);
	return std::pair{bx / len, bz / len};
}
}

std::vector<std::pair<double, double>> hull_ccw(
		const std::vector<std::pair<int, int>> &points)
{
	return hull_ccw_impl(points);
}

std::optional<Block> default_surface(const tags_t &tags)
{
	return default_surface_impl(tags);
}

std::optional<std::pair<double, double>> longest_edge_dir(const ProcessedWay &way)
{
	return longest_edge_dir_impl(way);
}

void draw_pitch_markings(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &area, Block surface)
{
	const auto kind = pitch_kind(way);
	if (!kind || area.size() < 40 || way.tags.get("indoor") == "yes")
		return;
	if (*kind == PitchKind::Baseball) {
		draw_diamond(editor, way, area, surface);
		return;
	}
	auto direction = longest_edge_dir(way);
	if (!direction)
		return;
	double ux = direction->first, uz = direction->second;
	double vx = -uz, vz = ux;
	double cx = 0.0, cz = 0.0;
	for (auto [x, z] : area) {
		cx += x;
		cz += z;
	}
	cx /= area.size();
	cz /= area.size();
	double umin = 1e30, umax = -1e30, vmin = 1e30, vmax = -1e30;
	for (auto [x, z] : area) {
		const double dx = x - cx, dz = z - cz;
		const double u = dx * ux + dz * uz, v = dx * vx + dz * vz;
		umin = std::min(umin, u);
		umax = std::max(umax, u);
		vmin = std::min(vmin, v);
		vmax = std::max(vmax, v);
	}
	double a = (umax - umin) / 2.0, b = (vmax - vmin) / 2.0;
	if (b > a) {
		std::swap(a, b);
		std::swap(ux, vx);
		std::swap(uz, vz);
	}
	if (a < 6.0 || b < 4.0)
		return;
	const double umid = (umin + umax) / 2.0, vmid = (vmin + vmax) / 2.0;
	const std::set<std::pair<int, int>> cells(area.begin(), area.end());
	auto line = [](double d) { return std::abs(d) <= 0.6; };
	const auto stripe =
			(*kind == PitchKind::Soccer || *kind == PitchKind::AmericanFootball ||
					*kind == PitchKind::Rugby || *kind == PitchKind::Oval ||
					*kind == PitchKind::Cricket) &&
							a >= 20.0 &&
							(surface == GRASS_BLOCK ||
									surface == GREEN_STAINED_HARDENED_CLAY)
					? std::optional<Block>(MOSS_BLOCK)
					: std::nullopt;
	const double stripe_band = std::max(3.0, std::round(a / 9.0));
	const Block end_zone[]{BLUE_CONCRETE, RED_CONCRETE, GREEN_CONCRETE};
	const auto team_color = end_zone[land_cover::coord_hash(static_cast<int>(way.id),
											 static_cast<int>(way.id >> 32)) %
									 3];
	for (auto [x, z] : area) {
		const double dx = x - cx, dz = z - cz;
		const double u = dx * ux + dz * uz - umid;
		const double v = dx * vx + dz * vz - vmid;
		const bool boundary = !cells.contains({x + 1, z}) ||
							  !cells.contains({x - 1, z}) ||
							  !cells.contains({x, z + 1}) || !cells.contains({x, z - 1});
		bool marked = boundary;
		const double r = std::hypot(u, v);
		if (!marked &&
				(*kind == PitchKind::Soccer || *kind == PitchKind::AmericanFootball ||
						*kind == PitchKind::Rugby || *kind == PitchKind::Oval ||
						*kind == PitchKind::Cricket)) {
			const double cr = std::clamp(b * .3, 3.0, 9.0),
						 depth = std::min(a * .22, 12.0), hw = std::min(b * .65, 15.0);
			if (*kind == PitchKind::AmericanFootball) {
				const double goal = a * 5.0 / 6.0, step = goal / 10.0;
				marked = std::abs(u) <= goal + .6 &&
						 line(std::abs(u) - std::round(std::abs(u) / step) * step);
			} else if (*kind == PitchKind::Rugby) {
				const double goal = a * 5.0 / 6.0;
				const auto dash_phase =
						(static_cast<long long>(std::lround(v)) % 4 + 4) % 4;
				marked = line(u) || line(std::abs(u) - goal) ||
						 line(std::abs(u) - goal * .56) ||
						 (line(std::abs(u) - goal * .2) && dash_phase < 2);
			} else if (*kind == PitchKind::Oval) {
				const double k = a / 80.0, from_goal = std::hypot(a - std::abs(u), v);
				marked = (line(std::abs(u) - 25.0 * k) && std::abs(v) <= 25.0 * k) ||
						 (line(std::abs(v) - 25.0 * k) && std::abs(u) <= 25.0 * k) ||
						 line(r - 5.0 * k) || line(from_goal - 50.0 * k);
			} else if (*kind == PitchKind::Cricket) {
				const double k = std::max(a / 70.0, .3);
				marked = line(std::abs(u) - 8.8 * k) &&
						 std::abs(v) <= 2.0 * std::max(k, 1.0);
			} else {
				marked = line(u) || line(r - cr) ||
						 (line(std::abs(u) - (a - depth)) && std::abs(v) <= hw) ||
						 (line(std::abs(v) - hw) && std::abs(u) >= a - depth);
			}
		} else if (!marked && *kind == PitchKind::Basketball) {
			const double cr = std::clamp(b * .25, 2.0, 5.0),
						 depth = std::min(a * .28, 9.0), hw = std::min(b * .35, 4.0);
			const double fu = a - depth, du = std::abs(u) - fu;
			marked = line(u) || line(r - cr) || (line(du) && std::abs(v) <= hw) ||
					 (line(std::abs(v) - hw) && std::abs(u) >= fu) ||
					 (std::abs(std::hypot(du, v) - cr) <= .6 && std::abs(u) <= fu);
		} else if (!marked && *kind == PitchKind::Tennis) {
			const double singles = b * .75, service = a * .54;
			marked = line(u) || line(std::abs(v) - singles) ||
					 (line(std::abs(u) - service) && std::abs(v) <= singles) ||
					 (line(v) && std::abs(u) <= service);
		} else if (!marked && *kind == PitchKind::Generic) {
			marked = line(u) || line(r - std::clamp(b * .3, 2.0, 8.0));
		}
		std::optional<Block> paint;
		if (marked)
			paint = WHITE_CONCRETE;
		else if (*kind == PitchKind::Cricket) {
			const double k = std::max(a / 70.0, .3);
			if (std::abs(u) <= 10.0 * k && std::abs(v) <= 1.5 * std::max(k, 1.0))
				paint = DIRT_PATH;
		} else if (*kind == PitchKind::AmericanFootball &&
				   std::abs(u) > a * 5.0 / 6.0 + .6)
			paint = team_color;
		else if (stripe &&
				 static_cast<long long>(std::floor((u + a) / stripe_band)) % 2 != 0)
			paint = stripe;
		if (paint)
			editor.set_block(*paint, x, 0, z, std::vector<Block>{surface}, std::nullopt);
	}
	auto place = [&](double u, double v, int y, Block block) {
		const int x = std::lround(cx + (u + umid) * ux + (v + vmid) * vx);
		const int z = std::lround(cz + (u + umid) * uz + (v + vmid) * vz);
		if (cells.contains({x, z}))
			editor.set_block(block, x, y, z, std::nullopt, std::nullopt);
	};
	if (*kind == PitchKind::Soccer && a >= 10.0 && b >= 5.0) {
		for (double end : {-1.0, 1.0}) {
			const double u = end * (a - 0.5);
			for (int dv = -2; dv <= 2; ++dv) {
				place(u, dv, 2, IRON_BARS);
				if (std::abs(dv) == 2)
					place(u, dv, 1, IRON_BARS);
			}
		}
	} else if (*kind == PitchKind::AmericanFootball && a >= 20.0 && b >= 10.0) {
		const int half = std::max(2, static_cast<int>(std::lround(b * .115)));
		const int bar = std::max(2, static_cast<int>(std::lround(b * .125)));
		const int top = bar + std::max(4, static_cast<int>(std::lround(b * .37)));
		for (double end : {-1.0, 1.0}) {
			const double u = end * (a - .5);
			for (int y = 1; y < bar; ++y)
				place(u, 0.0, y, YELLOW_CONCRETE);
			for (int dv = -half; dv <= half; ++dv)
				place(u, dv, bar, YELLOW_CONCRETE);
			for (int y = bar + 1; y <= top; ++y) {
				place(u, -half, y, YELLOW_CONCRETE);
				place(u, half, y, YELLOW_CONCRETE);
			}
		}
	} else if (*kind == PitchKind::Rugby && a >= 20.0 && b >= 10.0) {
		const int half = std::max(2, static_cast<int>(std::lround(b * .08)));
		const int bar = std::max(2, static_cast<int>(std::lround(b * .086)));
		const int top = std::max(bar + 4, static_cast<int>(std::lround(b * .45)));
		const double try_line = a * 5.0 / 6.0;
		for (double end : {-1.0, 1.0}) {
			const double u = end * try_line;
			for (int dv = -half; dv <= half; ++dv)
				place(u, dv, bar, WHITE_CONCRETE);
			for (int y = 1; y <= top; ++y) {
				place(u, -half, y, WHITE_CONCRETE);
				place(u, half, y, WHITE_CONCRETE);
			}
		}
	} else if (*kind == PitchKind::Oval && a >= 30.0) {
		const double k = a / 80.0;
		const int gap = std::max(2, static_cast<int>(std::lround(3.2 * k)));
		const int tall = std::max(5, static_cast<int>(std::lround(b * .1)));
		for (double end : {-1.0, 1.0})
			for (int side : {-1, 1}) {
				for (int y = 1; y <= tall; ++y)
					place(end * (a - .5), side * gap, y, WHITE_CONCRETE);
				for (int y = 1; y <= tall * 2 / 3; ++y)
					place(end * (a - .5), side * gap * 3, y, WHITE_CONCRETE);
			}
	} else if (*kind == PitchKind::Soccer && a >= 10.0 && b >= 5.0) {
		for (double end : {-1.0, 1.0})
			for (int dv = -2; dv <= 2; ++dv) {
				place(end * (a - .5), dv, 2, IRON_BARS);
				if (std::abs(dv) == 2)
					place(end * (a - .5), dv, 1, IRON_BARS);
			}
	} else if (*kind == PitchKind::Tennis && b >= 4.0) {
		for (int dv = -static_cast<int>(b * .8); dv <= static_cast<int>(b * .8); ++dv)
			place(0.0, dv, 1, IRON_BARS);
	}
}
}
