#include "sport_pitches.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <sstream>

namespace arnis::sport_pitches
{
namespace
{
enum class PitchKind
{
	Soccer,
	Basketball,
	Tennis,
	Generic
};

std::optional<PitchKind> pitch_kind(const ProcessedWay &way)
{
	auto it = way.tags.find("sport");
	if (it == way.tags.end())
		return std::nullopt;
	std::istringstream values(it->second);
	std::string sport;
	while (std::getline(values, sport, ';')) {
		sport.erase(0, sport.find_first_not_of(" \t"));
		sport.erase(sport.find_last_not_of(" \t") + 1);
		if (sport == "soccer" || sport == "football")
			return PitchKind::Soccer;
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
	}
	return std::nullopt;
}

std::optional<std::pair<double, double>> longest_edge_dir(const ProcessedWay &way)
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

void draw_pitch_markings(WorldEditor &editor, const ProcessedWay &way,
		const std::vector<std::pair<int, int>> &area, Block surface)
{
	const auto kind = pitch_kind(way);
	if (!kind || area.size() < 40 || way.tags.get("indoor") == "yes")
		return;
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
	for (auto [x, z] : area) {
		const double dx = x - cx, dz = z - cz;
		const double u = dx * ux + dz * uz - umid;
		const double v = dx * vx + dz * vz - vmid;
		const bool boundary = !cells.contains({x + 1, z}) ||
							  !cells.contains({x - 1, z}) ||
							  !cells.contains({x, z + 1}) || !cells.contains({x, z - 1});
		bool marked = boundary;
		const double r = std::hypot(u, v);
		if (!marked && *kind == PitchKind::Soccer) {
			const double cr = std::clamp(b * .3, 3.0, 9.0),
						 depth = std::min(a * .22, 12.0), hw = std::min(b * .65, 15.0);
			marked = line(u) || line(r - cr) ||
					 (line(std::abs(u) - (a - depth)) && std::abs(v) <= hw) ||
					 (line(std::abs(v) - hw) && std::abs(u) >= a - depth);
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
		if (marked)
			editor.set_block(
					WHITE_CONCRETE, x, 0, z, std::vector<Block>{surface}, std::nullopt);
	}
	auto place = [&](double u, double v, int y, Block block) {
		const int x = std::lround(cx + (u + umid) * ux + (v + vmid) * vx);
		const int z = std::lround(cz + (u + umid) * uz + (v + vmid) * vz);
		if (cells.contains({x, z}))
			editor.set_block(block, x, y, z, std::nullopt, std::nullopt);
	};
	if (*kind == PitchKind::Soccer && a >= 10.0 && b >= 5.0) {
		for (double end : {-1.0, 1.0})
			for (int dv = -2; dv <= 2; ++dv) {
				place(end * (a - .5), dv, 2, IRON_BARS);
				if (std::abs(dv) == 2)
					place(end * (a - .5), dv, 1, IRON_BARS);
			}
	} else if (*kind == PitchKind::Tennis) {
		for (int dv = -static_cast<int>(b * .8); dv <= static_cast<int>(b * .8); ++dv)
			place(0.0, dv, 1, IRON_BARS);
	}
}
}
