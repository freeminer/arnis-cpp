#include "prescan.h"
#include "../../../../arnis_adapter.h"
#include <algorithm>
#include <cmath>
namespace arnis::models_3d::three_dmr
{
namespace
{
std::string tag(const ProcessedElement &e, const char *n)
{
	auto i = e.tags().find(n);
	return i == e.tags().end() ? std::string{} : i->second;
}
bool has(const std::vector<std::pair<std::string, std::int64_t>> &v,
		const std::pair<std::string, std::int64_t> &k)
{
	return std::find(v.begin(), v.end(), k) != v.end();
}
std::vector<std::pair<int, int>> points(const ProcessedElement &e)
{
	std::vector<std::pair<int, int>> o;
	if (e.is_node())
		o.push_back({e.as_node().x, e.as_node().z});
	else if (e.is_way())
		for (auto &n : e.as_way().nodes)
			o.push_back({n.x, n.z});
	else
		for (auto &m : e.as_relation().members)
			for (auto &n : m.way.nodes)
				o.push_back({n.x, n.z});
	return o;
}
bool direction(const std::string &s, double &o)
{
	try {
		std::size_t i = 0;
		o = std::stod(s, &i);
		if (i == s.size()) {
			o = std::fmod(std::fmod(o, 360) + 360, 360);
			return true;
		}
	} catch (...) {
	}
	static const std::pair<const char *, double> names[] = {{"N", 0}, {"NORTH", 0},
			{"NNE", 22.5}, {"NE", 45}, {"ENE", 67.5}, {"E", 90}, {"EAST", 90},
			{"ESE", 112.5}, {"SE", 135}, {"SSE", 157.5}, {"S", 180}, {"SOUTH", 180},
			{"SSW", 202.5}, {"SW", 225}, {"WSW", 247.5}, {"W", 270}, {"WEST", 270},
			{"WNW", 292.5}, {"NW", 315}, {"NNW", 337.5}};
	std::string u = s;
	std::transform(u.begin(), u.end(), u.begin(), ::toupper);
	for (auto [n, d] : names)
		if (u == n) {
			o = d;
			return true;
		}
	return false;
}
}
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double rotation,
		const std::vector<std::pair<std::string, std::int64_t>> &already)
{
	PrescanResult r;
	std::vector<Bounds> footprints;
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key))
			continue;
		std::int64_t model = 0;
		try {
			std::size_t n = 0;
			auto raw = tag(e, "3dmr");
			model = std::stoll(raw, &n);
			if (n != raw.size() || model < 0)
				continue;
		} catch (...) {
			continue;
		}
		auto pts = points(e);
		if (pts.empty())
			continue;
		long long sx = 0, sz = 0;
		int x0 = pts[0].first, x1 = x0, z0 = pts[0].second, z1 = z0;
		for (auto [x, z] : pts) {
			sx += x;
			sz += z;
			x0 = std::min(x0, x);
			x1 = std::max(x1, x);
			z0 = std::min(z0, z);
			z1 = std::max(z1, z);
		}
		int ax = sx / pts.size(), az = sz / pts.size();
		Bounds raw{x0, z0, x1, z1},
				fp = pts.size() == 1 ? Bounds{ax - 8, az - 8, ax + 8, az + 8} : raw;
		double yaw = 0;
		direction(tag(e, "direction"), yaw);
		r.placements.push_back({e.id(), model, ax, az, fp, yaw + rotation});
		r.suppressed.push_back(key);
		if (pts.size() > 1)
			footprints.push_back(raw);
	}
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key) || has(r.suppressed, key) ||
				(tag(e, "building").empty() && tag(e, "building:part").empty()))
			continue;
		auto p = points(e);
		if (p.empty())
			continue;
		long long sx = 0, sz = 0;
		for (auto [x, z] : p) {
			sx += x;
			sz += z;
		}
		int x = sx / p.size(), z = sz / p.size();
		if (std::any_of(footprints.begin(), footprints.end(),
					[&](const Bounds &b) { return b.contains(x, z); }))
			r.suppressed.push_back(key);
	}
	return r;
}
std::vector<std::pair<int, int>> deferred_regions(const PrescanResult &r, double scale)
{
	std::vector<std::pair<int, int>> o;
	int d = std::ceil(ASSUMED_HALF_EXTENT_M * scale);
	for (auto &p : r.placements)
		for (int z = (p.anchor_z - d) >> 9; z <= (p.anchor_z + d) >> 9; ++z)
			for (int x = (p.anchor_x - d) >> 9; x <= (p.anchor_x + d) >> 9; ++x)
				o.push_back({x, z});
	std::sort(o.begin(), o.end());
	o.erase(std::unique(o.begin(), o.end()), o.end());
	return o;
}
}
