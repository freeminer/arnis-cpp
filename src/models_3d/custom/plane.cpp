#include "plane.h"
#include "../../../../arnis_adapter.h"
#include "../../deterministic_rng.h"
#include <algorithm>
#include <cmath>
#include <map>

namespace arnis::models_3d::custom::plane
{
namespace
{
constexpr double plane_length_m = 90, climb_pitch = 12, climb_min_m = 1500,
				 parked_min_m = 120;
struct Seg
{
	std::int64_t id;
	bool runway;
	std::int64_t first, last;
	std::vector<std::pair<double, double>> points;
	double angle;
};
struct Strip
{
	std::int64_t id;
	bool runway;
	double cx, cz, dx, dz, length, perp, min_a, max_a;
};
struct UF
{
	std::vector<std::size_t> p;
	explicit UF(std::size_t n) : p(n)
	{
		for (std::size_t i = 0; i < n; ++i)
			p[i] = i;
	}
	std::size_t find(std::size_t x) { return p[x] == x ? x : p[x] = find(p[x]); }
	void join(std::size_t a, std::size_t b)
	{
		a = find(a);
		b = find(b);
		if (a != b)
			p[a] = b;
	}
};
bool close(double a, double b)
{
	double d = std::abs(a - b);
	return std::min(d, M_PI - d) < .349;
}
}

std::vector<Placement> prescan(
		const std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<Seg> segs;
	for (const auto &e : elements) {
		if (!e.is_way())
			continue;
		const auto &w = e.as_way();
		auto it = w.tags.find("aeroway");
		if (it == w.tags.end() || (it->second != "runway" && it->second != "taxiway") ||
				w.tags.get("area") == "yes" || w.nodes.size() < 2)
			continue;
		const auto &a = w.nodes.front(), &b = w.nodes.back();
		if (a.id == b.id)
			continue;
		double dx = b.x - a.x, dz = b.z - a.z;
		if (dx == 0 && dz == 0)
			continue;
		Seg s{w.id, it->second == "runway", a.id, b.id, {}, std::atan2(dz, dx)};
		s.angle = std::fmod(s.angle + M_PI, M_PI);
		for (auto &n : w.nodes)
			s.points.push_back({double(n.x), double(n.z)});
		segs.push_back(std::move(s));
	}
	UF uf(segs.size());
	std::map<std::int64_t, std::vector<std::size_t>> ends;
	for (std::size_t i = 0; i < segs.size(); ++i) {
		ends[segs[i].first].push_back(i);
		ends[segs[i].last].push_back(i);
	}
	for (auto &[_, v] : ends)
		for (std::size_t a = 0; a < v.size(); ++a)
			for (std::size_t b = a + 1; b < v.size(); ++b)
				if (segs[v[a]].runway == segs[v[b]].runway &&
						close(segs[v[a]].angle, segs[v[b]].angle))
					uf.join(v[a], v[b]);
	std::map<std::size_t, std::vector<std::size_t>> groups;
	for (std::size_t i = 0; i < segs.size(); ++i)
		groups[uf.find(i)].push_back(i);
	std::vector<Placement> out;
	if (scale <= 0)
		return out;
	const double plane = plane_length_m * scale;
	for (auto &[_, g] : groups) {
		std::vector<std::pair<double, double>> p;
		std::int64_t id = segs[g[0]].id;
		bool runway = segs[g[0]].runway;
		for (auto i : g) {
			id = std::min(id, segs[i].id);
			p.insert(p.end(), segs[i].points.begin(), segs[i].points.end());
		}
		double cx = 0, cz = 0;
		for (auto [q, r] : p) {
			cx += q;
			cz += r;
		}
		cx /= p.size();
		cz /= p.size();
		double xx = 0, xz = 0, zz = 0;
		for (auto [q, r] : p) {
			q -= cx;
			r -= cz;
			xx += q * q;
			xz += q * r;
			zz += r * r;
		}
		double th = .5 * std::atan2(2 * xz, xx - zz), dx = std::cos(th),
			   dz = std::sin(th);
		double lo = 1e99, hi = -1e99, plo = 1e99, phi = -1e99;
		for (auto [q, r] : p) {
			double a = (q - cx) * dx + (r - cz) * dz, b = (q - cx) * dz - (r - cz) * dx;
			lo = std::min(lo, a);
			hi = std::max(hi, a);
			plo = std::min(plo, b);
			phi = std::max(phi, b);
		}
		// PCA can produce the short axis. Rust flips it so the direction always
		// means nose-to-tail, which matters for take-off endpoint and yaw.
		if (phi - plo > hi - lo) {
			const double old_dx = dx;
			dx = dz;
			dz = -old_dx;
			std::swap(lo, plo);
			std::swap(hi, phi);
		}
		double len = hi - lo;
		if (len <= 0 || len / scale > 8000 || (phi - plo) / len > (runway ? .5 : .12))
			continue;
		double yaw = std::atan2(-dx, dz) * 180 / M_PI;
		auto make = [&](Kind k, double a, double pitch, int elev) {
			int x = std::lround(cx + a * dx), z = std::lround(cz + a * dz),
				r = int(std::ceil(plane * .5)) + 4;
			out.push_back({id, k, x, z, elev, yaw, pitch, {x - r, z - r, x + r, z + r}});
		};
		if (runway && len / scale >= climb_min_m)
			make(Kind::Ascending, hi, climb_pitch,
					std::max(1, int(std::lround(plane * .45 + 20 * scale))));
		double prob = runway ? .4 : .15;
		auto rng = element_rng(static_cast<std::uint64_t>(id));
		if (len / scale >= parked_min_m && rng.random_bool(prob) &&
				hi - plane * .5 > lo + plane * .5) {
			const double unit = double(rng()) / 4294967296.0;
			make(Kind::Parked, lo + plane * .5 + (hi - lo - plane) * unit, 0, 0);
		}
	}
	return out;
}
std::vector<std::pair<int, int>> deferred_regions(
		const std::vector<Placement> &p, double scale)
{
	std::vector<std::pair<int, int>> o;
	int r = std::ceil(plane_length_m * scale);
	for (auto &a : p)
		for (int z = (a.anchor_z - r) >> 9; z <= (a.anchor_z + r) >> 9; ++z)
			for (int x = (a.anchor_x - r) >> 9; x <= (a.anchor_x + r) >> 9; ++x)
				o.push_back({x, z});
	std::sort(o.begin(), o.end());
	o.erase(std::unique(o.begin(), o.end()), o.end());
	return o;
}
}
