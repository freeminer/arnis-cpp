#include "prescan.h"
#include "../../../../arnis_adapter.h"
#include "../../strict_parse.h"
#include "../../tile.h"
#include "../../unicode_text.h"
#include <algorithm>
#include <cmath>
#include <string_view>
namespace arnis::models_3d::three_dmr
{
namespace
{
std::string tag(const ProcessedElement &e, const char *n)
{
	auto i = e.tags().find(n);
	return i == e.tags().end() ? std::string{} : i->second;
}
bool has(const std::vector<std::pair<std::string, std::uint64_t>> &v,
		const std::pair<std::string, std::uint64_t> &k)
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
			for (auto &n : m.way->nodes)
				o.push_back({n.x, n.z});
	return o;
}
bool direction(const std::string &s, double &o)
{
	const auto [begin, end] = unicode_text::trim_whitespace(s);
	const std::string_view value(s.data() + begin, end - begin);
	if (const auto degrees = strict_parse::f64(value);
			degrees && std::isfinite(*degrees)) {
		o = std::fmod(std::fmod(*degrees, 360.0) + 360.0, 360.0);
		return true;
	}
	static const std::pair<const char *, double> names[] = {{"N", 0}, {"NORTH", 0},
			{"NNE", 22.5}, {"NE", 45}, {"ENE", 67.5}, {"E", 90}, {"EAST", 90},
			{"ESE", 112.5}, {"SE", 135}, {"SSE", 157.5}, {"S", 180}, {"SOUTH", 180},
			{"SSW", 202.5}, {"SW", 225}, {"WSW", 247.5}, {"W", 270}, {"WEST", 270},
			{"WNW", 292.5}, {"NW", 315}, {"NNW", 337.5}};
	std::string u = value;
	std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c) {
		return static_cast<char>(c >= 'a' && c <= 'z' ? c - ('a' - 'A') : c);
	});
	for (auto [n, d] : names)
		if (u == n) {
			o = d;
			return true;
		}
	return false;
}
}
PrescanResult prescan(const std::vector<ProcessedElement> &elements, double rotation,
		const std::vector<std::pair<std::string, std::uint64_t>> &already)
{
	PrescanResult r;
	std::vector<Bounds> footprints;
	for (const auto &e : elements) {
		auto key = std::make_pair(std::string(e.kind()), e.id());
		if (has(already, key))
			continue;
		std::uint64_t model = 0;
		try {
			const auto raw = tag(e, "3dmr");
			const auto [first, last] = unicode_text::trim_whitespace(raw);
			std::string_view value(raw.data() + first, last - first);
			if (value.empty())
				continue;
			const auto digits_begin =
					value.starts_with('+') ? std::size_t{1} : std::size_t{0};
			if (digits_begin == value.size() ||
					!std::all_of(
							value.begin() + static_cast<std::ptrdiff_t>(digits_begin),
							value.end(),
							[](unsigned char c) { return c >= '0' && c <= '9'; }))
				continue;
			std::size_t n = 0;
			model = std::stoull(std::string(value.substr(digits_begin)), &n);
			if (n != value.size() - digits_begin)
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
		// Keep the divisor signed, as in Rust's i64 centroid, for negative X/Z.
		const auto count = static_cast<long long>(pts.size());
		int ax = static_cast<int>(sx / count), az = static_cast<int>(sz / count);
		// Match Rust's osm_bbox: a node (or degenerate one-point geometry)
		// retains its actual point-sized footprint. Do not synthesize an area.
		Bounds fp{x0, z0, x1, z1};
		double yaw = 0;
		direction(tag(e, "direction"), yaw);
		r.placements.push_back({e.id(), model, ax, az, fp, yaw + rotation});
		r.suppressed.push_back(key);
		// Rust also uses this raw OSM bbox for building suppression.
		footprints.push_back(fp);
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
		const auto count = static_cast<long long>(p.size());
		int x = static_cast<int>(sx / count), z = static_cast<int>(sz / count);
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
		// Match Rust's region_keys_around safety ring. A conservative full-region
		// margin keeps streamed output intact if voxelization reaches the edge.
		for (int z = tiles::region_floor(p.anchor_z - d) - 1;
				z <= tiles::region_floor(p.anchor_z + d) + 1; ++z)
			for (int x = tiles::region_floor(p.anchor_x - d) - 1;
					x <= tiles::region_floor(p.anchor_x + d) + 1; ++x)
				o.push_back({x, z});
	std::sort(o.begin(), o.end());
	o.erase(std::unique(o.begin(), o.end()), o.end());
	return o;
}
}
