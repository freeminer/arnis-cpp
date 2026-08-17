#include "stadium.h"
#include "../../../../arnis_adapter.h"
#include <algorithm>
#include <cmath>

namespace arnis::models_3d::custom::stadium
{
namespace
{
using P = std::pair<double, double>;
bool has(const std::vector<std::pair<std::string, std::int64_t>> &v,
		const std::pair<std::string, std::int64_t> &k)
{
	return std::find(v.begin(), v.end(), k) != v.end();
}
std::string tag(const ProcessedElement &e, const char *name)
{
	auto it = e.tags().find(name);
	return it == e.tags().end() ? std::string{} : it->second;
}
std::vector<P> points(const ProcessedElement &e)
{
	std::vector<P> o;
	if (e.is_way())
		for (auto &n : e.as_way().nodes)
			o.push_back({n.x, n.z});
	else if (e.is_relation())
		for (auto &m : e.as_relation().members)
			for (auto &n : m.way.nodes)
				o.push_back({n.x, n.z});
	return o;
}
std::pair<int, int> anchor(const ProcessedElement &e)
{
	auto p = points(e);
	double x = 0, z = 0;
	for (auto &q : p) {
		x += q.first;
		z += q.second;
	}
	return p.empty() ? std::pair{0, 0} : std::pair{int(x / p.size()), int(z / p.size())};
}
double cross(P a, P b, P c)
{
	return (b.first - a.first) * (c.second - a.second) -
		   (b.second - a.second) * (c.first - a.first);
}
std::vector<P> hull(std::vector<P> p)
{
	std::sort(p.begin(), p.end());
	p.erase(std::unique(p.begin(), p.end()), p.end());
	if (p.size() < 3)
		return p;
	std::vector<P> h;
	for (int pass = 0; pass < 2; ++pass) {
		auto q = p;
		if (pass)
			std::reverse(q.begin(), q.end());
		for (auto x : q) {
			while (h.size() > 1 && cross(h[h.size() - 2], h.back(), x) <= 0)
				h.pop_back();
			h.push_back(x);
		}
	}
	h.pop_back();
	return h;
}
bool placement(const ProcessedElement &e, double scale, Placement &o)
{
	auto p = points(e);
	if (p.size() < 3 || scale <= 0)
		return false;
	auto h = hull(p);
	if (h.size() < 3)
		return false;
	double area = 1e300, cx = 0, cz = 0, longv = 0, shortv = 0, theta = 0;
	for (std::size_t i = 0; i < h.size(); ++i) {
		auto a = h[i], b = h[(i + 1) % h.size()];
		double dx = b.first - a.first, dz = b.second - a.second, len = std::hypot(dx, dz);
		if (len < 1e-9)
			continue;
		double ux = dx / len, uz = dz / len, vx = -uz, vz = ux, lo = 1e300, hi = -1e300,
			   plo = 1e300, phi = -1e300;
		for (auto q : h) {
			double x = q.first * ux + q.second * uz, z = q.first * vx + q.second * vz;
			lo = std::min(lo, x);
			hi = std::max(hi, x);
			plo = std::min(plo, z);
			phi = std::max(phi, z);
		}
		if ((hi - lo) * (phi - plo) < area) {
			area = (hi - lo) * (phi - plo);
			double ca = (lo + hi) / 2, cb = (plo + phi) / 2;
			cx = ca * ux + cb * vx;
			cz = ca * uz + cb * vz;
			longv = hi - lo;
			shortv = phi - plo;
			theta = std::atan2(uz, ux);
		}
	}
	if (shortv > longv) {
		std::swap(shortv, longv);
		theta += M_PI / 2;
	}
	float lm = longv / scale, sm = shortv / scale;
	if (sm < 10 || lm > 500 || sm > 400)
		return false;
	int minx = p[0].first, maxx = minx, minz = p[0].second, maxz = minz;
	for (auto q : p) {
		minx = std::min(minx, int(q.first));
		maxx = std::max(maxx, int(q.first));
		minz = std::min(minz, int(q.second));
		maxz = std::max(maxz, int(q.second));
	}
	o = {e.id(), int(std::lround(cx)), int(std::lround(cz)), {minx, minz, maxx, maxz}, lm,
			sm, theta * 180 / M_PI, 0, false};
	auto hs = tag(e, "height");
	if (!hs.empty())
		try {
			o.osm_height_m = std::min(200.f, float(std::stod(hs)));
			o.has_osm_height = o.osm_height_m > 0;
		} catch (...) {
		}
	return true;
}

// The voxelized stadium may protrude past the OSM polygon's axis-aligned
// bounds when it is rotated.  Rust uses this oriented rectangle to decide
// which inner pitch/track/building features the model owns.
bool in_model_rect(const Placement &p, double scale, int x, int z, double margin = 2.)
{
	const double theta = p.yaw_degrees * M_PI / 180.;
	const double dx = x - p.anchor_x, dz = z - p.anchor_z;
	const double a = dx * std::cos(theta) + dz * std::sin(theta);
	const double b = -dx * std::sin(theta) + dz * std::cos(theta);
	return std::abs(a) <= p.long_m * scale * .5 + margin &&
		   std::abs(b) <= p.short_m * scale * .5 + margin;
}
}
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double scale,
		const std::vector<std::pair<std::string, std::int64_t>> &already)
{
	PrescanResult r;
	std::vector<std::pair<int, int>> building_anchors;
	for (auto &e : elements)
		if (tag(e, "building") == "stadium")
			building_anchors.push_back(anchor(e));
	auto add = [&](const ProcessedElement &e, bool leisure) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key) || has(r.suppressed, key))
			return;
		Placement p;
		if (!placement(e, scale, p))
			return;
		double a = double(p.long_m) * p.short_m;
		bool inner = std::any_of(building_anchors.begin(), building_anchors.end(),
				[&](auto x) { return p.footprint.contains(x.first, x.second); });
		bool q = leisure ? (a >= 20000 || (a >= 10000 && inner)) : a >= 20000;
		if (q) {
			r.suppressed.push_back(key);
			r.placements.push_back(p);
		}
	};
	for (auto &e : elements)
		if (tag(e, "leisure") == "stadium")
			add(e, true);
	for (auto &e : elements)
		if (tag(e, "building") == "stadium")
			add(e, false);
	for (auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key) || has(r.suppressed, key))
			continue;
		auto tags = e.tags();
		if (!(tags.find("building") != tags.end() ||
					tags.find("building:part") != tags.end() ||
					tag(e, "leisure") == "pitch" || tag(e, "leisure") == "track"))
			continue;
		auto [x, z] = anchor(e);
		for (auto &p : r.placements)
			if (p.footprint.contains(x, z) || in_model_rect(p, scale, x, z)) {
				r.suppressed.push_back(key);
				break;
			}
	}
	return r;
}
void retain_fetchable(PrescanResult &r, bool model_fetchable)
{
	if (model_fetchable)
		return;
	// No placement means no interior claim: procedural features must receive
	// the original OSM elements, exactly as Rust's fetch-failure return does.
	r.placements.clear();
	r.suppressed.clear();
}
std::vector<std::pair<int, int>> deferred_regions(
		const std::vector<Placement> &p, double scale)
{
	std::vector<std::pair<int, int>> o;
	for (auto &a : p) {
		int d = std::ceil(.5 * std::hypot(a.long_m, a.short_m) * scale);
		for (int z = (a.anchor_z - d) >> 9; z <= (a.anchor_z + d) >> 9; ++z)
			for (int x = (a.anchor_x - d) >> 9; x <= (a.anchor_x + d) >> 9; ++x)
				o.push_back({x, z});
	}
	std::sort(o.begin(), o.end());
	o.erase(std::unique(o.begin(), o.end()), o.end());
	return o;
}
}
