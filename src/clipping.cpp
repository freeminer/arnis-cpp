#include "clipping.h"
#include "rust_numeric.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
#include <utility>
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
										(nodes.front().x == nodes.back().x &&
												nodes.front().z == nodes.back().z));
}
bool ring_outside_box(const std::vector<ProcessedNode> &nodes, const XZBBox &box)
{
	return std::all_of(nodes.begin(), nodes.end(), [&](const auto &node) {
		return node.x < box.min_x();
	}) || std::all_of(nodes.begin(), nodes.end(), [&](const auto &node) {
		return node.x > box.max_x();
	}) || std::all_of(nodes.begin(), nodes.end(), [&](const auto &node) {
		return node.z < box.min_z();
	}) || std::all_of(nodes.begin(), nodes.end(), [&](const auto &node) {
		return node.z > box.max_z();
	});
}
std::vector<P> clip_polygon(std::vector<P> points, const XZBBox &box)
{
	if (points.size() > 1 && points.front().x == points.back().x &&
			points.front().z == points.back().z)
		points.pop_back();
	for (const Edge edge : {Left, Right, Top, Bottom}) {
		std::vector<P> out;
		if (points.empty())
			break;
		P previous = points.back();
		bool previous_inside =
				in(previous, edge, box.min_x(), box.min_z(), box.max_x(), box.max_z());
		for (const P current : points) {
			const bool current_inside =
					in(current, edge, box.min_x(), box.min_z(), box.max_x(), box.max_z());
			if (current_inside != previous_inside)
				out.push_back(cross(previous, current, edge, box.min_x(), box.min_z(),
						box.max_x(), box.max_z()));
			if (current_inside)
				out.push_back(current);
			previous = current;
			previous_inside = current_inside;
		}
		points.swap(out);
	}
	std::vector<P> unique;
	for (const P p : points)
		if (unique.empty() || std::abs(unique.back().x - p.x) >= .1 ||
				std::abs(unique.back().z - p.z) >= .1)
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
		if (std::abs(p) < 1e-12)
			return q >= 0.;
		const double r = q / p;
		if (p < 0.) {
			if (r > high)
				return false;
			low = std::max(low, r);
		} else {
			if (r < low)
				return false;
			high = std::min(high, r);
		}
		return true;
	};
	if (!limit(-dx, a.x - box.min_x()) || !limit(dx, box.max_x() - a.x) ||
			!limit(-dz, a.z - box.min_z()) || !limit(dz, box.max_z() - a.z) || low > high)
		return std::nullopt;
	return std::pair<P, P>{
			{a.x + low * dx, a.z + low * dz}, {a.x + high * dx, a.z + high * dz}};
}
ProcessedNode synthetic(std::uint64_t way_id, std::size_t index, P point)
{
	const auto id = INVENTED_NODE_BASE |
					((way_id * 10000000ULL + static_cast<std::uint64_t>(index)) &
							(INVENTED_NODE_BASE - 1));
	return {id, {}, int(std::lround(point.x)), int(std::lround(point.z))};
}
}
bool is_invented_node_id(std::uint64_t id)
{
	return id >= INVENTED_NODE_BASE;
}
void mark_cut_ends(std::vector<ProcessedNode> &nodes,
		const std::array<std::pair<int, int>, 2> &ends, std::uint64_t way_id)
{
	if (nodes.empty())
		return;
	const auto last = nodes.size() - 1;
	const auto mark = [&](std::size_t index, const std::pair<int, int> &original) {
		auto &node = nodes[index];
		if (std::pair{node.x, node.z} == original)
			return;
		node.id = INVENTED_NODE_BASE |
				  ((way_id * 10000000ULL + static_cast<std::uint64_t>(index)) &
						  (INVENTED_NODE_BASE - 1));
	};
	mark(0, ends[0]);
	mark(last, ends[1]);
}
std::vector<ProcessedNode> clip_way_to_bbox(
		const std::vector<ProcessedNode> &nodes, const XZBBox &box)
{
	if (nodes.empty())
		return {};
	if (closed(nodes)) {
		if (std::all_of(nodes.begin(), nodes.end(), [&](const auto &n) {
				return inside_box({double(n.x), double(n.z)}, box);
			}))
			return nodes;
		std::vector<P> polygon;
		polygon.reserve(nodes.size());
		for (const auto &n : nodes)
			polygon.push_back({double(n.x), double(n.z)});
		auto clipped = clip_polygon(std::move(polygon), box);
		if (clipped.size() < 3)
			return {};
		std::map<std::pair<int, int>, const ProcessedNode *> original_nodes;
		for (const auto &node : nodes)
			original_nodes.emplace(std::pair{node.x, node.z}, &node);
		std::vector<ProcessedNode> out;
		out.reserve(clipped.size() + 1);
		for (std::size_t i = 0; i < clipped.size(); ++i) {
			const int x = int(std::lround(clipped[i].x));
			const int z = int(std::lround(clipped[i].z));
			const auto original = original_nodes.find({x, z});
			if (original != original_nodes.end()) {
				auto retained = *original->second;
				retained.x = x;
				retained.z = z;
				out.push_back(std::move(retained));
			} else {
				out.push_back(synthetic(nodes.front().id, i, clipped[i]));
			}
		}
		if (out.size() >= 2 && out.front().x == out.back().x &&
				out.front().z == out.back().z)
			out.back().id = out.front().id;
		out.push_back(out.front());
		return out;
	}
	std::vector<ProcessedNode> out;
	auto append = [&](P p, bool original, const ProcessedNode &node) {
		const int x = int(std::lround(p.x)), z = int(std::lround(p.z));
		if (!out.empty() && out.back().x == x && out.back().z == z)
			return;
		out.push_back(original ? node : synthetic(nodes.front().id, out.size(), p));
	};
	for (std::size_t i = 1; i < nodes.size(); ++i) {
		const P a{double(nodes[i - 1].x), double(nodes[i - 1].z)},
				b{double(nodes[i].x), double(nodes[i].z)};
		auto segment = clip_segment(a, b, box);
		if (!segment)
			continue;
		append(segment->first, inside_box(a, box), nodes[i - 1]);
		append(segment->second, inside_box(b, box), nodes[i]);
	}
	if (out.size() >= 2) {
		constexpr double endpoint_tolerance_squared = 50.0 * 50.0;
		auto matches_endpoint = [=](const ProcessedNode &clipped,
										const ProcessedNode &endpoint) {
			const double dx = double(clipped.x) - endpoint.x;
			const double dz = double(clipped.z) - endpoint.z;
			return dx * dx + dz * dz < endpoint_tolerance_squared;
		};
		if (matches_endpoint(out.front(), nodes.front()))
			out.front().id = nodes.front().id;
		if (matches_endpoint(out.back(), nodes.back()))
			out.back().id = nodes.back().id;
	}
	return out;
}

std::vector<ProcessedNode> clip_way_to_bbox_owned(
		std::vector<ProcessedNode> nodes, const XZBBox &box)
{
	if (std::all_of(nodes.begin(), nodes.end(), [&](const auto &node) {
			return inside_box({double(node.x), double(node.z)}, box);
		}))
		return nodes;
	return clip_way_to_bbox(nodes, box);
}

std::optional<std::vector<ProcessedNode>> clip_water_ring_to_bbox(
		const std::vector<ProcessedNode> &ring, const XZBBox &box)
{
	if (ring.empty())
		return std::nullopt;
	if (std::all_of(ring.begin(), ring.end(), [&](const auto &node) {
			return inside_box({double(node.x), double(node.z)}, box);
		}))
		return ring;
	// Match Rust's fast rejection before polygon clipping. Besides avoiding work
	// for distant OSM water, this prevents a degenerate all-outside ring from
	// producing a spurious clipped sliver after coordinate rounding.
	if (ring_outside_box(ring, box))
		return std::nullopt;
	// Match Rust's water-ring closure tolerance. OSM rings can end at adjacent
	// projected cells after coordinate quantization even when their node IDs
	// differ; rejecting those here drops otherwise valid water polygons.
	const bool water_ring_closed =
			ring.size() >= 3 &&
			(ring.front().id == ring.back().id ||
					(std::abs(ring.front().x - ring.back().x) <= 1 &&
							std::abs(ring.front().z - ring.back().z) <= 1));
	if (!water_ring_closed)
		return std::nullopt;
	std::vector<P> input;
	input.reserve(ring.size());
	for (const auto &node : ring)
		input.push_back({double(node.x), double(node.z)});
	auto p = clip_polygon(std::move(input), box);
	if (p.size() < 3)
		return std::nullopt;
	if (!std::all_of(p.begin(), p.end(),
				[&](const P point) { return inside_box(point, box); }))
		return std::nullopt;
	std::vector<ProcessedNode> out;
	out.reserve(p.size() + 1);
	for (std::size_t i = 0; i < p.size(); ++i)
		out.push_back({1'000'000'000ULL + i, {}, int(std::lround(p[i].x)),
				int(std::lround(p[i].z))});
	out.push_back(out.front());
	return out;
}

std::optional<std::vector<OutlineRun>> cut_line_to_outline(std::uint64_t way_id,
		const std::vector<OutlinePoint> &points, const XZBBox &world, std::uint64_t &cuts)
{
	if (!world.outline())
		return std::nullopt;
	constexpr int slack = 2;
	constexpr double min_cut_run = 12.0;
	constexpr std::uint64_t cut_base = 9'000'000;
	const auto block = [](double x, double z) {
		return std::pair{rust_numeric::saturating_i32(std::floor(x)),
				rust_numeric::saturating_i32(std::floor(z))};
	};
	const auto inside = [&](double x, double z) {
		const auto [bx, bz] = block(x, z);
		return world.contains(bx, bz);
	};
	const auto between = [&](std::int32_t bx, std::int32_t bz) {
		const auto hit = [&](int dx, int dz) {
			for (int k = 1; k <= 2 * slack; ++k)
				if (world.contains(bx + dx * k, bz + dz * k))
					return true;
			return false;
		};
		return (hit(1, 0) && hit(-1, 0)) || (hit(0, 1) && hit(0, -1));
	};
	const auto near_ = [&](double x, double z) {
		const auto [bx, bz] = block(x, z);
		return world.touches(bx - slack, bz - slack, bx + slack, bz + slack) &&
			   !between(bx, bz);
	};
	const auto initial_cuts = cuts;
	const std::uint64_t base = cut_base + (way_id >> 62) * 250'000;
	const auto cut_at = [&](double x, double z) {
		++cuts;
		const auto id = INVENTED_NODE_BASE | ((way_id * 10'000'000ULL + base + cuts) &
													 (INVENTED_NODE_BASE - 1));
		return OutlinePoint{id, x, z};
	};
	std::vector<OutlineRun> runs;
	OutlineRun run;
	bool starts_cut = false;
	const auto first = points.empty() ? OutlinePoint{} : points.front();
	bool is_in = !points.empty() && inside(std::get<1>(first), std::get<2>(first));
	std::tuple<double, double, std::size_t> last_in{
			points.empty() ? 0.0 : std::get<1>(first),
			points.empty() ? 0.0 : std::get<2>(first), 1};
	std::optional<std::pair<double, double>> after_in;
	if (is_in)
		run.push_back(first);
	const auto boundary = [&](double ax, double az, double bx, double bz) {
		double lo = 0.0, hi = 1.0;
		for (int i = 0; i < 16; ++i) {
			const double mid = (lo + hi) * 0.5;
			if (inside(ax + (bx - ax) * mid, az + (bz - az) * mid))
				lo = mid;
			else
				hi = mid;
		}
		return std::pair{ax + (bx - ax) * lo, az + (bz - az) * lo};
	};
	const auto close = [&](bool began_cut, bool ended_cut) {
		double length = 0.0;
		for (std::size_t i = 1; i < run.size(); ++i)
			length += std::hypot(std::get<1>(run[i]) - std::get<1>(run[i - 1]),
					std::get<2>(run[i]) - std::get<2>(run[i - 1]));
		if (run.size() >= 2 && !(began_cut && ended_cut && length < min_cut_run))
			runs.push_back(std::move(run));
		run.clear();
	};
	for (std::size_t pair_index = 1; pair_index < points.size(); ++pair_index) {
		const auto a = points[pair_index - 1], b = points[pair_index];
		const double ax = std::get<1>(a), az = std::get<2>(a);
		const double bx = std::get<1>(b), bz = std::get<2>(b);
		const auto at = [&](double t) {
			return std::pair{ax + (bx - ax) * t, az + (bz - az) * t};
		};
		const auto steps = static_cast<std::size_t>(
				std::max(1.0, std::ceil(std::hypot(bx - ax, bz - az) * 2.0)));
		double t0 = 0.0;
		for (std::size_t sample = 1; sample <= steps; ++sample) {
			const double t1 = static_cast<double>(sample) / steps;
			const auto [x, z] = at(t1);
			if (is_in) {
				if (inside(x, z)) {
					if (sample < steps)
						last_in = {x, z, run.size()};
					after_in.reset();
				} else {
					if (!after_in)
						after_in = {x, z};
					if (!near_(x, z)) {
						const auto [lx, lz, length] = last_in;
						const auto [cx, cz] =
								boundary(lx, lz, after_in->first, after_in->second);
						if (run.size() > length)
							run.resize(length);
						run.push_back(cut_at(cx, cz));
						close(starts_cut, true);
						is_in = false;
					}
				}
			} else if (inside(x, z)) {
				const auto [cx, cz] = boundary(x, z, at(t0).first, at(t0).second);
				run.push_back(cut_at(cx, cz));
				starts_cut = true;
				last_in = {x, z, run.size()};
				after_in.reset();
				is_in = true;
			}
			t0 = t1;
		}
		if (is_in) {
			run.push_back(b);
			if (inside(bx, bz)) {
				last_in = {bx, bz, run.size()};
				after_in.reset();
			}
		}
	}
	if (is_in) {
		if (after_in) {
			const auto [lx, lz, length] = last_in;
			const auto [cx, cz] = boundary(lx, lz, after_in->first, after_in->second);
			if (run.size() > length)
				run.resize(length);
			run.push_back(cut_at(cx, cz));
			close(starts_cut, true);
		} else {
			close(starts_cut, false);
		}
	}
	if (cuts == initial_cuts && runs.size() == 1 && runs.front().size() == points.size())
		return std::nullopt;
	return runs;
}

std::optional<std::vector<ProcessedWay>> cut_way_to_outline(
		const ProcessedWay &way, const XZBBox &world)
{
	std::vector<OutlinePoint> points;
	points.reserve(way.nodes.size());
	for (const auto &node : way.nodes)
		points.emplace_back(node.id, node.x + 0.5, node.z + 0.5);
	std::uint64_t cuts = 0;
	auto runs = cut_line_to_outline(way.id, points, world, cuts);
	if (!runs)
		return std::nullopt;
	std::unordered_map<std::uint64_t, const ProcessedNode *> by_id;
	for (const auto &node : way.nodes)
		by_id.emplace(node.id, &node);
	std::vector<ProcessedWay> result;
	result.reserve(runs->size());
	for (const auto &line : *runs) {
		ProcessedWay clipped;
		clipped.id = way.id;
		clipped.tags = way.tags;
		clipped.nodes.reserve(line.size());
		for (const auto &[id, x, z] : line) {
			const auto it = by_id.find(id);
			if (it != by_id.end() && !is_invented_node_id(id)) {
				clipped.nodes.push_back(*it->second);
			} else {
				// Rust's float-to-i32 `as` cast saturates (and maps NaN to zero).
				// Keep invented outline-cut nodes safe for coordinates outside the
				// representable world-coordinate range as well.
				clipped.nodes.push_back(
						{id, {}, rust_numeric::saturating_i32(std::floor(x)),
								rust_numeric::saturating_i32(std::floor(z))});
			}
		}
		result.push_back(std::move(clipped));
	}
	return result;
}

std::vector<ProcessedElement> cut_elements_to_outline(
		const std::vector<ProcessedElement> &elements, const XZBBox &world)
{
	if (!world.outline())
		return elements;
	std::vector<ProcessedElement> result;
	result.reserve(elements.size());
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (auto clipped = cut_way_to_outline(way, world)) {
				for (auto &piece : *clipped)
					result.emplace_back(std::move(piece));
			} else {
				result.emplace_back(way);
			}
		} else if (element.is_node()) {
			const auto &node = element.as_node();
			if (world.contains(node.x, node.z))
				result.emplace_back(node);
		} else {
			result.emplace_back(element.as_relation());
		}
	}
	return result;
}
}
