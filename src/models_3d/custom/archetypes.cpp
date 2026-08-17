#include "archetypes.h"
#include <cmath>
#include <algorithm>
namespace arnis::models_3d::custom
{
std::optional<ModelAsset> fetch_plane_model(ModelProvider &p, const std::string &key)
{
	return p.fetch(key);
}
std::optional<ModelAsset> fetch_stadium_model(ModelProvider &p, const std::string &key)
{
	return p.fetch(key);
}
ArchetypeModels fetch_archetype_models(ModelProvider &p, bool a, bool b)
{
	ArchetypeModels r;
	if (a)
		r.plane = p.fetch(PLANE_CACHE_FILE);
	if (b)
		r.stadium = p.fetch(STADIUM_CACHE_FILE);
	return r;
}
void apply_model_availability(ArchetypePrescan &p, const ArchetypeModels &m,
		const std::vector<ProcessedElement> &e, double scale)
{
	filter_unavailable(p, !p.planes.empty() && m.plane.has_value(),
			!p.stadiums.empty() && m.stadium.has_value());
	finalize_prescan(p, e, scale);
}
bool suppressed_at(const ArchetypePrescan &p, int x, int z)
{
	for (const auto &v : p.planes)
		if (v.footprint.contains(x, z))
			return true;
	for (const auto &v : p.stadiums)
		if (v.footprint.contains(x, z))
			return true;
	return false;
}
std::vector<std::pair<int, int>> placement_anchors(const ArchetypePrescan &p)
{
	std::vector<std::pair<int, int>> out;
	out.reserve(placement_count(p));
	for (const auto &v : p.planes)
		out.emplace_back(v.anchor_x, v.anchor_z);
	for (const auto &v : p.stadiums)
		out.emplace_back(v.anchor_x, v.anchor_z);
	return out;
}
std::vector<PlacementRef> placement_refs(const ArchetypePrescan &p)
{
	std::vector<PlacementRef> out;
	out.reserve(placement_count(p));
	for (std::size_t i = 0; i < p.planes.size(); ++i)
		out.push_back({ArchetypeKind::Plane, i});
	for (std::size_t i = 0; i < p.stadiums.size(); ++i)
		out.push_back({ArchetypeKind::Stadium, i});
	return out;
}
PlacementTransform transform_for(const ArchetypePrescan &p, PlacementRef r)
{
	if (r.kind == ArchetypeKind::Plane) {
		const auto &v = p.planes.at(r.index);
		return {v.anchor_x, v.anchor_z, v.elevation_blocks, v.yaw_degrees,
				v.pitch_degrees};
	}
	const auto &v = p.stadiums.at(r.index);
	return {v.anchor_x, v.anchor_z, 0, v.yaw_degrees, 0};
}
const char *model_key_for(PlacementRef r)
{
	return r.kind == ArchetypeKind::Plane ? PLANE_CACHE_FILE : STADIUM_CACHE_FILE;
}
const ModelFootprint &footprint_for(const ArchetypePrescan &p, PlacementRef r)
{
	return r.kind == ArchetypeKind::Plane ? p.planes.at(r.index).footprint
										  : p.stadiums.at(r.index).footprint;
}
bool valid_placement(const ArchetypePrescan &p, PlacementRef r)
{
	const auto t = transform_for(p, r);
	return valid_footprint(footprint_for(p, r)) && std::isfinite(t.yaw) &&
		   std::isfinite(t.pitch);
}
std::vector<PlacementRef> valid_placement_refs(const ArchetypePrescan &p)
{
	std::vector<PlacementRef> out;
	for (const auto r : placement_refs(p))
		if (valid_placement(p, r))
			out.push_back(r);
	return out;
}
double model_scale_for(
		const ArchetypePrescan &p, PlacementRef r, const ArchetypeModels &m, double s)
{
	if (r.kind == ArchetypeKind::Plane)
		return m.plane ? plane_model_scale(*m.plane, PLANE_LENGTH_M * s) : 0;
	const auto &v = p.stadiums.at(r.index);
	return m.stadium ? stadium_model_scale(*m.stadium, v.long_m * s, v.short_m * s) : 0;
}
std::vector<PlacementPlan> build_placement_plans(
		const ArchetypePrescan &p, const ArchetypeModels &m, double s)
{
	std::vector<PlacementPlan> out;
	for (const auto r : valid_placement_refs(p)) {
		const auto k = model_key_for(r);
		const auto sc = model_scale_for(p, r, m, s);
		if (sc <= 0)
			continue;
		out.push_back({r, transform_for(p, r), k, sc, footprint_for(p, r)});
	}
	std::sort(out.begin(), out.end(), [&](const auto &a, const auto &b) {
		if (a.ref.kind != b.ref.kind)
			return int(a.ref.kind) < int(b.ref.kind);
		return a.ref.index < b.ref.index;
	});
	return out;
}
std::vector<std::pair<int, int>> plan_regions(const PlacementPlan &p, double s, int n)
{
	const int r =
			p.ref.kind == ArchetypeKind::Plane
					? int(std::ceil(PLANE_LENGTH_M * s))
					: int(std::ceil(std::hypot(p.footprint.max_x - p.footprint.min_x,
											p.footprint.max_z - p.footprint.min_z) *
									0.5));
	return deferred_region_keys(p.transform.anchor_x, p.transform.anchor_z, r, n);
}
bool plan_contains(const PlacementPlan &p, int x, int z)
{
	return p.footprint.contains(x, z);
}
bool plane_strip_eligible(double length_m, bool runway)
{
	return length_m >= 120.0 && length_m <= 8000.0 &&
		   (runway ? length_m >= 120.0 : length_m >= 120.0);
}
bool stadium_footprint_eligible(double area, double long_m, double short_m, bool inner)
{
	return short_m >= 10.0 && long_m <= 500.0 && short_m <= 400.0 &&
		   ((area >= 20000.0) || (area >= 10000.0 && inner));
}
double normalize_yaw_degrees(double yaw)
{
	double v = std::fmod(yaw, 360.0);
	return v < 0 ? v + 360.0 : v;
}
ModelFootprint rotated_footprint(int x, int z, double l, double s, double yaw)
{
	const double a = normalize_yaw_degrees(yaw) * 3.141592653589793 / 180.0,
				 c = std::abs(std::cos(a)), q = std::abs(std::sin(a));
	const int hx = int(std::ceil((c * l + q * s) * 0.5)),
			  hz = int(std::ceil((q * l + c * s) * 0.5));
	return {x - hx, z - hz, x + hx, z + hz};
}
int ascending_elevation_blocks(double length, double scale, double ground)
{
	if (length < ASCENDING_MIN_LENGTH_M)
		return int(std::lround(ground));
	return int(std::lround(
			ground + (length * ASCENDING_ELEV_FACTOR + ASCENDING_EXTRA_ELEV_M) * scale));
}
float stadium_target_height_m(float h, bool has)
{
	return (has && h > 0 ? std::min(h, 200.0f) : STADIUM_DEFAULT_HEIGHT_M) *
		   STADIUM_HEIGHT_MULTIPLIER;
}
double plane_model_scale(const ModelAsset &a, double target)
{
	const double len = double(a.max[2]) - a.min[2];
	return len > 1e-6 ? target / len : 0.0;
}
double stadium_model_scale(const ModelAsset &a, double l, double s)
{
	const double x = double(a.max[0]) - a.min[0], z = double(a.max[2]) - a.min[2];
	const double sx = x > 1e-6 ? l / x : 0, sz = z > 1e-6 ? s / z : 0;
	return std::min(sx, sz);
}
double way_length_blocks(const ProcessedWay &w, double scale)
{
	double n = 0;
	for (std::size_t i = 1; i < w.nodes.size(); ++i)
		n += std::hypot(w.nodes[i].x - w.nodes[i - 1].x, w.nodes[i].z - w.nodes[i - 1].z);
	return n / scale;
}
double way_bearing_degrees(const ProcessedWay &w)
{
	if (w.nodes.size() < 2)
		return 0;
	const auto &a = w.nodes.front(), &b = w.nodes.back();
	return normalize_yaw_degrees(
			std::atan2(double(b.x - a.x), double(b.z - a.z)) * 180.0 / 3.141592653589793);
}
float parse_height_m(const std::string &v, bool &valid)
{
	try {
		const float h = std::stof(v);
		valid = h > 0 && h <= 200;
		return valid ? h : 0.0f;
	} catch (...) {
		valid = false;
		return 0.0f;
	}
}
double stadium_yaw_from_extents(double x, double z)
{
	return x >= z ? 0.0 : 90.0;
}
bool valid_footprint(const ModelFootprint &f)
{
	return f.min_x <= f.max_x && f.min_z <= f.max_z;
}
bool deterministic_model_chance(std::uint64_t id, double probability, std::uint64_t salt)
{
	if (probability <= 0)
		return false;
	if (probability >= 1)
		return true;
	std::uint64_t x = id + salt + 0x9e3779b97f4a7c15ULL;
	x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
	x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
	x ^= x >> 31;
	return double(x >> 11) / double(1ULL << 53) < probability;
}
ArchetypePrescan prescan_archetypes(const std::vector<ProcessedElement> &e, double scale)
{
	ArchetypePrescan r;
	r.planes = prescan_planes(e, scale);
	r.stadiums = prescan_stadiums(e, scale);
	std::sort(r.planes.begin(), r.planes.end(), [](const auto &a, const auto &b) {
		return a.representative_id < b.representative_id;
	});
	std::sort(r.stadiums.begin(), r.stadiums.end(),
			[](const auto &a, const auto &b) { return a.osm_id < b.osm_id; });
	r.suppressed_ids = suppressed_element_ids(r.stadiums, e);
	for (const auto &p : r.planes) {
		auto v = deferred_region_keys(p.anchor_x, p.anchor_z, deferred_radius(p, scale));
		r.deferred_regions.insert(r.deferred_regions.end(), v.begin(), v.end());
	}
	for (const auto &p : r.stadiums) {
		auto v = deferred_region_keys(p.anchor_x, p.anchor_z, deferred_radius(p, scale));
		r.deferred_regions.insert(r.deferred_regions.end(), v.begin(), v.end());
	}
	std::sort(r.deferred_regions.begin(), r.deferred_regions.end());
	r.deferred_regions.erase(
			std::unique(r.deferred_regions.begin(), r.deferred_regions.end()),
			r.deferred_regions.end());
	std::sort(r.suppressed_ids.begin(), r.suppressed_ids.end());
	r.suppressed_ids.erase(std::unique(r.suppressed_ids.begin(), r.suppressed_ids.end()),
			r.suppressed_ids.end());
	return r;
}
void filter_unavailable(ArchetypePrescan &p, bool plane, bool stadium)
{
	if (!plane)
		p.planes.clear();
	if (!stadium)
		p.stadiums.clear();
}
void rebuild_deferred_regions(ArchetypePrescan &p, double scale)
{
	p.deferred_regions.clear();
	for (const auto &v : p.planes) {
		auto r = deferred_region_keys(v.anchor_x, v.anchor_z, deferred_radius(v, scale));
		p.deferred_regions.insert(p.deferred_regions.end(), r.begin(), r.end());
	}
	for (const auto &v : p.stadiums) {
		auto r = deferred_region_keys(v.anchor_x, v.anchor_z, deferred_radius(v, scale));
		p.deferred_regions.insert(p.deferred_regions.end(), r.begin(), r.end());
	}
	std::sort(p.deferred_regions.begin(), p.deferred_regions.end());
	p.deferred_regions.erase(
			std::unique(p.deferred_regions.begin(), p.deferred_regions.end()),
			p.deferred_regions.end());
}
void rebuild_suppression(ArchetypePrescan &p, const std::vector<ProcessedElement> &e)
{
	p.suppressed_ids = suppressed_element_ids(p.stadiums, e);
	std::sort(p.suppressed_ids.begin(), p.suppressed_ids.end());
	p.suppressed_ids.erase(std::unique(p.suppressed_ids.begin(), p.suppressed_ids.end()),
			p.suppressed_ids.end());
}
void finalize_prescan(
		ArchetypePrescan &p, const std::vector<ProcessedElement> &e, double scale)
{
	rebuild_suppression(p, e);
	rebuild_deferred_regions(p, scale);
}
int deferred_radius(const PlanePlacement &, double scale)
{
	return int(std::ceil(90.0 * scale));
}
int deferred_radius(const StadiumPlacement &p, double scale)
{
	return int(std::ceil(std::hypot(p.long_m, p.short_m) * 0.5 * scale));
}
std::vector<std::pair<int, int>> deferred_region_keys(int x, int z, int radius, int size)
{
	std::vector<std::pair<int, int>> out;
	if (size <= 0)
		return out;
	int x0 = (x - radius) / size, x1 = (x + radius) / size, z0 = (z - radius) / size,
		z1 = (z + radius) / size;
	for (int rz = z0; rz <= z1; ++rz)
		for (int rx = x0; rx <= x1; ++rx)
			out.emplace_back(rx, rz);
	return out;
}
std::vector<StadiumPlacement> prescan_stadiums(
		const std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<StadiumPlacement> out;
	for (const auto &e : elements) {
		if (!e.is_way())
			continue;
		const auto &t = e.tags();
		auto tag = [&](const char *k) {
			auto i = t.find(k);
			return i == t.end() ? std::string() : i->second;
		};
		if (tag("leisure") != "stadium" && tag("building") != "stadium")
			continue;
		const auto &nodes = e.as_way().nodes;
		if (nodes.empty())
			continue;
		int minx = nodes.front().x, maxx = minx, minz = nodes.front().z, maxz = minz;
		for (const auto &n : nodes) {
			minx = std::min(minx, n.x);
			maxx = std::max(maxx, n.x);
			minz = std::min(minz, n.z);
			maxz = std::max(maxz, n.z);
		}
		const double l = (maxx - minx) / scale, s = (maxz - minz) / scale;
		if (!stadium_footprint_eligible(l * s, l, s, tag("building") == "stadium"))
			continue;
		StadiumPlacement p;
		p.osm_id = std::uint64_t(e.id());
		p.anchor_x = (minx + maxx) / 2;
		p.anchor_z = (minz + maxz) / 2;
		p.long_m = float(std::max(l, s));
		p.short_m = float(std::min(l, s));
		p.footprint = {minx, minz, maxx, maxz};
		out.push_back(p);
	}
	return out;
}
std::vector<PlanePlacement> prescan_planes(
		const std::vector<ProcessedElement> &elements, double scale)
{
	std::vector<PlanePlacement> out;
	for (const auto &e : elements) {
		if (!e.is_way())
			continue;
		const auto &t = e.tags();
		auto i = t.find("aeroway");
		if (i == t.end() || (i->second != "runway" && i->second != "taxiway"))
			continue;
		const auto &w = e.as_way();
		if (w.nodes.size() < 2)
			continue;
		int minx = w.nodes.front().x, maxx = minx, minz = w.nodes.front().z, maxz = minz;
		for (const auto &q : w.nodes) {
			minx = std::min(minx, q.x);
			maxx = std::max(maxx, q.x);
			minz = std::min(minz, q.z);
			maxz = std::max(maxz, q.z);
		}
		double len = way_length_blocks(w, scale);
		if (!plane_strip_eligible(len, i->second == "runway"))
			continue;
		PlanePlacement p;
		p.representative_id = std::uint64_t(e.id());
		p.anchor_x = (minx + maxx) / 2;
		p.anchor_z = (minz + maxz) / 2;
		p.yaw_degrees = way_bearing_degrees(w);
		p.ascending = i->second == "runway" && len >= ASCENDING_MIN_LENGTH_M;
		p.pitch_degrees = p.ascending ? ASCENDING_PITCH_DEG : 0;
		p.footprint = rotated_footprint(p.anchor_x, p.anchor_z, PLANE_LENGTH_M * scale,
				20.0 * scale, p.yaw_degrees);
		out.push_back(p);
	}
	return out;
}
std::vector<std::uint64_t> suppressed_element_ids(
		const std::vector<StadiumPlacement> &placements,
		const std::vector<ProcessedElement> &elements)
{
	std::vector<std::uint64_t> out;
	for (const auto &e : elements) {
		if (!e.is_way())
			continue;
		for (const auto &p : placements) {
			const auto &n = e.as_way().nodes;
			if (!n.empty() && p.footprint.contains(n.front().x, n.front().z)) {
				out.push_back(std::uint64_t(e.id()));
				break;
			}
		}
	}
	return out;
}
}
