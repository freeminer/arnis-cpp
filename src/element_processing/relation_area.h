#pragma once

#include "../osm_parser.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace arnis::element_processing
{

enum class RelationArea
{
	Building,
	Water,
	Natural,
	Landuse,
	Park,
	ManMade,
};

inline bool landuse_paints_ground(const tags_t &tags)
{
	const auto it = tags.find("landuse");
	return it == tags.end() ||
		   (it->second != "residential" && it->second != "commercial");
}

inline std::optional<RelationArea> relation_area(const tags_t &tags)
{
	if (tags.contains("building") || tags.contains("building:part") ||
			(tags.contains("type") && tags.at("type") == "building"))
		return RelationArea::Building;
	if (tags.contains("water") ||
			(tags.contains("natural") &&
					(tags.at("natural") == "water" || tags.at("natural") == "bay")))
		return RelationArea::Water;
	if (tags.contains("natural"))
		return RelationArea::Natural;
	if (tags.contains("landuse"))
		return RelationArea::Landuse;
	if (tags.contains("leisure") && tags.at("leisure") == "park")
		return RelationArea::Park;
	if (tags.contains("man_made"))
		return RelationArea::ManMade;
	return std::nullopt;
}

inline bool relation_area_fills_members(RelationArea area, const tags_t &tags)
{
	switch (area) {
	case RelationArea::Natural:
	case RelationArea::Park:
		return true;
	case RelationArea::Landuse:
		return landuse_paints_ground(tags);
	case RelationArea::ManMade:
		return tags.contains("man_made") &&
			   (tags.at("man_made") == "pier" || tags.at("man_made") == "breakwater" ||
					   tags.at("man_made") == "groyne");
	case RelationArea::Building:
	case RelationArea::Water:
		return false;
	}
	return false;
}

// Whether the relation is rendered as an area, either by water processing or by
// painting its outer members. Keep this alongside Rust's RelationArea::draws_area
// so parser suppression and generation agree on which multipolygons own their rings.
inline bool relation_area_draws_area(RelationArea area, const tags_t &tags)
{
	return area == RelationArea::Water || relation_area_fills_members(area, tags);
}

using RingGroup = std::pair<std::vector<std::size_t>, std::vector<std::size_t>>;

// Assign each inner ring only to the outer ring(s) it cuts. This preserves islands
// nested inside holes instead of subtracting a lake's hole from the island itself.
inline std::vector<RingGroup> hole_groups(
		const std::vector<std::vector<ProcessedNode>> &outers,
		const std::vector<std::vector<ProcessedNode>> &inners)
{
	using Bounds = std::array<std::int32_t, 4>; // min x, min z, max x, max z
	const auto bounds =
			[](const std::vector<ProcessedNode> &ring) -> std::optional<Bounds> {
		if (ring.empty())
			return std::nullopt;
		Bounds out{ring.front().x, ring.front().z, ring.front().x, ring.front().z};
		for (const auto &p : ring) {
			out[0] = std::min(out[0], p.x);
			out[1] = std::min(out[1], p.z);
			out[2] = std::max(out[2], p.x);
			out[3] = std::max(out[3], p.z);
		}
		return out;
	};
	const auto mostly_inside = [&](const std::vector<ProcessedNode> &ring,
									   const Bounds &ring_bounds,
									   const std::vector<ProcessedNode> &around,
									   const Bounds &around_bounds) {
		if (around.size() < 3 ||
				std::int64_t(ring_bounds[0]) < std::int64_t(around_bounds[0]) - 1 ||
				std::int64_t(ring_bounds[1]) < std::int64_t(around_bounds[1]) - 1 ||
				std::int64_t(ring_bounds[2]) > std::int64_t(around_bounds[2]) + 1 ||
				std::int64_t(ring_bounds[3]) > std::int64_t(around_bounds[3]) + 1)
			return false;
		std::size_t inside = 0, outside = 0;
		for (const auto &p : ring) {
			bool hit = false, on_edge = false;
			for (std::size_t i = 0, j = around.size() - 1; i < around.size(); j = i++) {
				const auto &a = around[j];
				const auto &b = around[i];
				// Widen before subtraction as well as multiplication. Map coordinates
				// can span a large world; casting the already-subtracted int32 values
				// would still overflow before reaching the 64-bit cross product.
				const std::int64_t cross =
						(std::int64_t(b.x) - a.x) * (std::int64_t(p.z) - a.z) -
						(std::int64_t(b.z) - a.z) * (std::int64_t(p.x) - a.x);
				if (cross == 0 && p.x >= std::min(a.x, b.x) &&
						p.x <= std::max(a.x, b.x) && p.z >= std::min(a.z, b.z) &&
						p.z <= std::max(a.z, b.z)) {
					on_edge = true;
					break;
				}
				if ((a.z > p.z) != (b.z > p.z) &&
						double(p.x) < (double(b.x) - a.x) * (double(p.z) - a.z) /
													  (double(b.z) - a.z) +
											  a.x)
					hit = !hit;
			}
			if (!on_edge) {
				if (hit)
					++inside;
				else
					++outside;
			}
		}
		return inside > outside;
	};
	std::vector<std::optional<Bounds>> inner_bounds;
	inner_bounds.reserve(inners.size());
	for (const auto &inner : inners)
		inner_bounds.push_back(bounds(inner));
	std::vector<RingGroup> groups;
	for (std::size_t o = 0; o < outers.size(); ++o) {
		const auto outer_bounds = bounds(outers[o]);
		std::vector<std::size_t> cutting;
		for (std::size_t i = 0; i < inners.size(); ++i) {
			const auto &inner_bound = inner_bounds[i];
			if (!outer_bounds || !inner_bound || (*inner_bound)[0] > (*outer_bounds)[2] ||
					(*inner_bound)[2] < (*outer_bounds)[0] ||
					(*inner_bound)[1] > (*outer_bounds)[3] ||
					(*inner_bound)[3] < (*outer_bounds)[1] ||
					!mostly_inside(outers[o], *outer_bounds, inners[i], *inner_bound))
				cutting.push_back(i);
		}
		auto found = std::find_if(groups.begin(), groups.end(),
				[&](const auto &group) { return group.second == cutting; });
		if (found == groups.end())
			groups.emplace_back(std::vector<std::size_t>{o}, std::move(cutting));
		else
			found->first.push_back(o);
	}
	return groups;
}

inline std::vector<std::vector<std::size_t>> holes_by_outer(
		const std::vector<std::vector<ProcessedNode>> &outers,
		const std::vector<std::vector<ProcessedNode>> &inners)
{
	std::vector<std::vector<std::size_t>> result(outers.size());
	for (auto &group : hole_groups(outers, inners))
		for (const auto outer : group.first)
			result[outer] = group.second;
	return result;
}

} // namespace arnis::element_processing
