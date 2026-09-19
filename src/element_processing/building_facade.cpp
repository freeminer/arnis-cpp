#include "building_facade.h"

#include "../bresenham.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arnis::building_facade
{
namespace
{
int sign(int value)
{
	return (value > 0) - (value < 0);
}

int outward_side(const std::vector<ProcessedNode> &nodes)
{
	std::int64_t area2 = 0;
	if (nodes.empty())
		return 1;
	for (std::size_t i = 0; i < nodes.size(); ++i) {
		const auto &a = nodes[i];
		const auto &b = nodes[(i + 1) % nodes.size()];
		area2 += std::int64_t(a.x) * b.z - std::int64_t(b.x) * a.z;
	}
	return area2 > 0 ? -1 : 1;
}

std::pair<int, int> outward_normal(int x1, int z1, int x2, int z2, int outward)
{
	const int dx = x2 - x1, dz = z2 - z1;
	const int raw_x = -dz * outward, raw_z = dx * outward;
	if (!raw_x && !raw_z)
		return {};
	return std::abs(raw_x) >= std::abs(raw_z) ? std::pair{sign(raw_x), 0}
											  : std::pair{0, sign(raw_z)};
}

int setback(double scale)
{
	return std::clamp(int(std::lround(8.0 * scale)), 8, 24);
}

std::optional<CornerPlan> detect_corner(const ProcessedWay &element,
		const std::vector<std::optional<SegmentFacade>> &segments,
		const PointSet &own_cells, const BuildingContext &context, double scale)
{
	if (segments.empty())
		return std::nullopt;
	const int min_len = std::max(6, int(6.0 * scale));
	const bool closed = element.nodes.size() > 1 &&
						element.nodes.front().x == element.nodes.back().x &&
						element.nodes.front().z == element.nodes.back().z;
	std::optional<std::pair<int, CornerPlan>> best;
	for (std::size_t i = 0; i < segments.size(); ++i) {
		const std::size_t j = (i + 1) % segments.size();
		if (j < i && !closed)
			continue;
		if (!segments[i] || !segments[j])
			continue;
		const auto &a = *segments[i], &b = *segments[j];
		if (a.facade_class != FacadeClass::Street ||
				b.facade_class != FacadeClass::Street || a.len < min_len ||
				b.len < min_len ||
				a.normal.first * b.normal.first + a.normal.second * b.normal.second !=
						0 ||
				j >= element.nodes.size())
			continue;
		const auto &vertex = element.nodes[j];
		const int dx = vertex.x + a.normal.first + b.normal.first;
		const int dz = vertex.z + a.normal.second + b.normal.second;
		if (own_cells.contains({dx, dz}) || context.building_footprints.contains(dx, dz))
			continue;
		const int score = a.road_dist.value_or(std::numeric_limits<int>::max() / 2) +
						  b.road_dist.value_or(std::numeric_limits<int>::max() / 2);
		if (!best || score < best->first)
			best = std::pair{score, CornerPlan{{vertex.x, vertex.z}, i, j}};
	}
	return best ? std::optional{best->second} : std::nullopt;
}
}

std::size_t PointHash::operator()(const std::pair<int, int> &point) const noexcept
{
	const auto x = std::uint64_t(std::uint32_t(point.first));
	const auto z = std::uint64_t(std::uint32_t(point.second));
	return std::hash<std::uint64_t>{}((x << 32) | z);
}

FacadePlan FacadePlan::empty()
{
	return {};
}
bool FacadePlan::is_party(int x, int z) const
{
	return !party_columns_.empty() && party_columns_.contains({x, z});
}
bool FacadePlan::is_street(int x, int z) const
{
	return street_columns_.contains({x, z});
}
bool FacadePlan::is_door(int x, int z) const
{
	return !door_columns.empty() && door_columns.contains({x, z});
}
void FacadePlan::mark_door_column(int x, int z)
{
	door_columns.insert({x, z});
}
void FacadePlan::add_party_column(int x, int z)
{
	party_columns_.insert({x, z});
}
void FacadePlan::add_street_column(int x, int z)
{
	street_columns_.insert({x, z});
}

FacadePlan compute_facade_plan(const ProcessedWay &element,
		const BuildingContext &context, double scale, const PointSet &own_cells)
{
	if (element.nodes.size() < 3)
		return FacadePlan::empty();
	const int outward = outward_side(element.nodes);
	FacadePlan plan;
	std::vector<std::vector<std::pair<int, int>>> segment_columns;
	for (std::size_t i = 1; i < element.nodes.size(); ++i) {
		const auto &a = element.nodes[i - 1], &b = element.nodes[i];
		const auto normal = outward_normal(a.x, a.z, b.x, b.z, outward);
		if (normal == std::pair<int, int>{}) {
			plan.segments.emplace_back();
			segment_columns.emplace_back();
			continue;
		}
		std::vector<std::pair<int, int>> points;
		for (const auto &[x, y, z] :
				bresenham::bresenham_line(a.x, 0, a.z, b.x, 0, b.z)) {
			(void)y;
			points.emplace_back(x, z);
		}
		std::size_t party_count = 0;
		for (const auto &[x, z] : points) {
			bool party = false;
			for (int depth = 1; depth <= 2; ++depth) {
				const std::pair candidate{
						x + normal.first * depth, z + normal.second * depth};
				party |= context.building_footprints.contains(
								 candidate.first, candidate.second) &&
						 !own_cells.contains(candidate);
			}
			if (party) {
				plan.add_party_column(x, z);
				++party_count;
			}
		}
		std::vector<std::size_t> samples;
		if (points.size() > 24)
			for (std::size_t sample = 0; sample < points.size(); sample += 8)
				samples.push_back(sample);
		else if (!points.empty()) {
			samples = {points.size() / 4, points.size() / 2, points.size() * 3 / 4};
			std::sort(samples.begin(), samples.end());
			samples.erase(std::unique(samples.begin(), samples.end()), samples.end());
		}
		std::size_t hits = 0;
		std::optional<int> road_dist;
		for (const auto sample : samples) {
			const auto [x, z] = points[std::min(sample, points.size() - 1)];
			for (int distance = 1; distance <= setback(scale); ++distance) {
				const int px = x + normal.first * distance;
				const int pz = z + normal.second * distance;
				if (context.road_mask.contains(px, pz)) {
					++hits;
					road_dist = std::min(road_dist.value_or(distance), distance);
					break;
				}
				if (context.building_footprints.contains(px, pz) &&
						!own_cells.contains({px, pz}))
					break;
			}
		}
		const int len = std::max(std::abs(b.x - a.x), std::abs(b.z - a.z));
		const bool party = !points.empty() && double(party_count) / points.size() >= 0.5;
		const bool street = road_dist && hits >= (len < 6 ? 1 : (samples.size() + 1) / 2);
		plan.segments.push_back(SegmentFacade{party	   ? FacadeClass::Party
											  : street ? FacadeClass::Street
													   : FacadeClass::Open,
				street ? road_dist : std::nullopt, normal,
				{sign(b.x - a.x), sign(b.z - a.z)}, len});
		segment_columns.push_back(std::move(points));
	}
	plan.has_any_street = std::any_of(
			plan.segments.begin(), plan.segments.end(), [](const auto &segment) {
				return segment && segment->facade_class == FacadeClass::Street;
			});
	if (plan.has_any_street)
		for (auto &segment : plan.segments)
			if (segment && segment->facade_class == FacadeClass::Open)
				segment->facade_class = FacadeClass::Rear;
	for (std::size_t i = 0; i < plan.segments.size(); ++i)
		if (plan.segments[i] && plan.segments[i]->facade_class == FacadeClass::Street)
			for (const auto &[x, z] : segment_columns[i])
				if (!plan.is_party(x, z))
					plan.add_street_column(x, z);
	for (std::size_t i = 0; i < plan.segments.size(); ++i) {
		if (!plan.segments[i] || plan.segments[i]->facade_class != FacadeClass::Street)
			continue;
		if (!plan.front_segment) {
			plan.front_segment = i;
			continue;
		}
		const auto &current = *plan.segments[i],
				   &best = *plan.segments[*plan.front_segment];
		if (current.road_dist < best.road_dist ||
				(current.road_dist == best.road_dist && current.len > best.len))
			plan.front_segment = i;
	}
	plan.corner = detect_corner(element, plan.segments, own_cells, context, scale);
	return plan;
}
}
