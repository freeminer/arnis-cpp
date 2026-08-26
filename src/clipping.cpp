#include "clipping.h"
#include <algorithm>
#include <cmath>
namespace arnis::clipping
{
namespace
{
struct P
{
	double x, z;
};
enum Edge
{
	Left,
	Right,
	Top,
	Bottom
};
bool in(P p, Edge e, double a, double b, double c, double d)
{
	return e == Left ? p.x >= a : e == Right ? p.x <= c : e == Top ? p.z >= b : p.z <= d;
}
P cross(P s, P q, Edge e, double a, double b, double c, double d)
{
	double t = e == Left || e == Right ? ((e == Left ? a : c) - s.x) / (q.x - s.x)
									   : ((e == Top ? b : d) - s.z) / (q.z - s.z);
	return {s.x + t * (q.x - s.x), s.z + t * (q.z - s.z)};
}
bool inside_box(P p, const XZBBox &box)
{
	return p.x >= box.min_x() && p.x <= box.max_x() && p.z >= box.min_z() &&
			p.z <= box.max_z();
}
bool closed(const std::vector<ProcessedNode> &nodes)
{
	return nodes.size() >= 3 && (nodes.front().id == nodes.back().id ||
			(nodes.front().x == nodes.back().x && nodes.front().z == nodes.back().z));
}
std::vector<P> clip_polygon(std::vector<P> points, const XZBBox &box)
{
	if (points.size() > 1 && points.front().x == points.back().x &&
			points.front().z == points.back().z)
		points.pop_back();
	for (const Edge edge : {Left, Right, Top, Bottom}) {
		std::vector<P> out;
		if (points.empty()) break;
		P previous = points.back();
		bool previous_inside = in(previous, edge, box.min_x(), box.min_z(), box.max_x(), box.max_z());
		for (const P current : points) {
			const bool current_inside = in(current, edge, box.min_x(), box.min_z(), box.max_x(), box.max_z());
			if (current_inside != previous_inside)
				out.push_back(cross(previous, current, edge, box.min_x(), box.min_z(), box.max_x(), box.max_z()));
			if (current_inside) out.push_back(current);
			previous = current;
			previous_inside = current_inside;
		}
		points.swap(out);
	}
	std::vector<P> unique;
	for (const P p : points)
		if (unique.empty() || std::abs(unique.back().x - p.x) >= .1 || std::abs(unique.back().z - p.z) >= .1)
			unique.push_back({std::clamp(p.x, double(box.min_x()), double(box.max_x())),
					std::clamp(p.z, double(box.min_z()), double(box.max_z()))});
	if (unique.size() > 1 && std::abs(unique.front().x - unique.back().x) < .1 &&
			std::abs(unique.front().z - unique.back().z) < .1)
		unique.pop_back();
	return unique;
}
std::optional<std::pair<P, P>> clip_segment(P a, P b, const XZBBox &box)
{
	const double dx = b.x - a.x, dz = b.z - a.z;
	double low = 0., high = 1.;
	auto limit = [&](double p, double q) {
		if (std::abs(p) < 1e-12) return q >= 0.;
		const double r = q / p;
		if (p < 0.) { if (r > high) return false; low = std::max(low, r); }
		else { if (r < low) return false; high = std::min(high, r); }
		return true;
	};
	if (!limit(-dx, a.x - box.min_x()) || !limit(dx, box.max_x() - a.x) ||
			!limit(-dz, a.z - box.min_z()) || !limit(dz, box.max_z() - a.z) || low > high)
		return std::nullopt;
	return std::pair<P, P>{{a.x + low * dx, a.z + low * dz}, {a.x + high * dx, a.z + high * dz}};
}
ProcessedNode synthetic(std::uint64_t way_id, std::size_t index, P point)
{
	const auto id = way_id * 10000000ULL + index;
	return {id, {}, int(std::lround(point.x)), int(std::lround(point.z))};
}
}
std::vector<ProcessedNode> clip_way_to_bbox(const std::vector<ProcessedNode> &nodes,
		const XZBBox &box)
{
	if (nodes.empty()) return {};
	if (closed(nodes)) {
		if (std::all_of(nodes.begin(), nodes.end(), [&](const auto &n) { return inside_box({double(n.x), double(n.z)}, box); }))
			return nodes;
		std::vector<P> polygon;
		polygon.reserve(nodes.size());
		for (const auto &n : nodes) polygon.push_back({double(n.x), double(n.z)});
		auto clipped = clip_polygon(std::move(polygon), box);
		if (clipped.size() < 3) return {};
		std::vector<ProcessedNode> out;
		out.reserve(clipped.size() + 1);
		for (std::size_t i = 0; i < clipped.size(); ++i) out.push_back(synthetic(nodes.front().id, i, clipped[i]));
		out.push_back(out.front());
		return out;
	}
	std::vector<ProcessedNode> out;
	auto append = [&](P p, bool original, const ProcessedNode &node) {
		const int x = int(std::lround(p.x)), z = int(std::lround(p.z));
		if (!out.empty() && out.back().x == x && out.back().z == z) return;
		out.push_back(original ? node : synthetic(nodes.front().id, out.size(), p));
	};
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		const P a{double(nodes[i - 1].x), double(nodes[i - 1].z)}, b{double(nodes[i].x), double(nodes[i].z)};
		auto segment = clip_segment(a, b, box);
		if (!segment) continue;
		append(segment->first, inside_box(a, box), nodes[i - 1]);
		append(segment->second, inside_box(b, box), nodes[i]);
	}
	return out;
}
std::optional<std::vector<ProcessedNode>> clip_water_ring_to_bbox(
		const std::vector<ProcessedNode> &ring, const XZBBox &box)
{
	if (ring.size() < 3 || !closed(ring))
		return std::nullopt;
	std::vector<P> input;
	input.reserve(ring.size());
	for (const auto &node : ring) input.push_back({double(node.x), double(node.z)});
	auto p = clip_polygon(std::move(input), box);
	if (p.size() < 3)
		return std::nullopt;
	std::vector<ProcessedNode> out;
	out.reserve(p.size() + 1);
	for (std::size_t i = 0; i < p.size(); ++i)
		out.push_back(synthetic(ring.front().id, i, p[i]));
	out.push_back(out.front());
	return out;
}
}
