#pragma once

#include "../osm_parser.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
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

namespace detail
{
using RingBounds = std::array<std::int32_t, 4>; // min x, min z, max x, max z

template <typename Ring>
inline std::optional<RingBounds> ring_bounds(const Ring &ring)
{
	if (ring.empty())
		return std::nullopt;
	RingBounds out{ring.front().x, ring.front().z, ring.front().x, ring.front().z};
	for (const auto &point : ring) {
		out[0] = std::min(out[0], point.x);
		out[1] = std::min(out[1], point.z);
		out[2] = std::max(out[2], point.x);
		out[3] = std::max(out[3], point.z);
	}
	return out;
}

template <typename Ring, typename Around>
inline bool vertices_inside(const Ring &ring, const RingBounds &ring_bounds,
		const Around &around, const RingBounds &around_bounds)
{
	if (around.size() < 3 ||
			std::int64_t(ring_bounds[0]) < std::int64_t(around_bounds[0]) - 1 ||
			std::int64_t(ring_bounds[1]) < std::int64_t(around_bounds[1]) - 1 ||
			std::int64_t(ring_bounds[2]) > std::int64_t(around_bounds[2]) + 1 ||
			std::int64_t(ring_bounds[3]) > std::int64_t(around_bounds[3]) + 1)
		return false;
	std::size_t inside = 0, outside = 0;
	for (const auto &point : ring) {
		bool hit = false;
		bool on_edge = false;
		for (std::size_t i = 0, j = around.size() - 1; i < around.size(); j = i++) {
			const auto &a = around[j];
			const auto &b = around[i];
			// Widen before subtraction as well as multiplication. World coordinates
			// may exceed the safe range for a 32-bit cross product.
			const std::int64_t cross =
					(std::int64_t(b.x) - a.x) * (std::int64_t(point.z) - a.z) -
					(std::int64_t(b.z) - a.z) * (std::int64_t(point.x) - a.x);
			if (cross == 0 && point.x >= std::min(a.x, b.x) &&
					point.x <= std::max(a.x, b.x) && point.z >= std::min(a.z, b.z) &&
					point.z <= std::max(a.z, b.z)) {
				on_edge = true;
				break;
			}
			if ((a.z > point.z) != (b.z > point.z) &&
					double(point.x) < (double(b.x) - a.x) * (double(point.z) - a.z) /
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
}
} // namespace detail

template <typename Ring>
inline bool mostly_inside(const Ring &ring, const Ring &around)
{
	const auto ring_bounds = detail::ring_bounds(ring);
	const auto around_bounds = detail::ring_bounds(around);
	return ring_bounds && around_bounds &&
		   detail::vertices_inside(ring, *ring_bounds, around, *around_bounds);
}

// Assign each inner ring only to the outer ring(s) it cuts. This preserves islands
// nested inside holes instead of subtracting a lake's hole from the island itself.
template <typename OuterRing, typename InnerRing>
inline std::vector<RingGroup> hole_groups(
		const std::vector<OuterRing> &outers, const std::vector<InnerRing> &inners)
{
	std::vector<std::optional<detail::RingBounds>> inner_bounds;
	inner_bounds.reserve(inners.size());
	for (const auto &inner : inners)
		inner_bounds.push_back(detail::ring_bounds(inner));
	std::vector<RingGroup> groups;
	for (std::size_t o = 0; o < outers.size(); ++o) {
		const auto outer_bounds = detail::ring_bounds(outers[o]);
		std::vector<std::size_t> cutting;
		for (std::size_t i = 0; i < inners.size(); ++i) {
			const auto &inner_bound = inner_bounds[i];
			if (!outer_bounds || !inner_bound ||
					!detail::vertices_inside(
							outers[o], *outer_bounds, inners[i], *inner_bound))
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

template <typename OuterRing, typename InnerRing>
inline std::vector<std::vector<std::size_t>> holes_by_outer(
		const std::vector<OuterRing> &outers, const std::vector<InnerRing> &inners)
{
	std::vector<std::vector<std::size_t>> result(outers.size());
	for (auto &group : hole_groups(outers, inners))
		for (const auto outer : group.first)
			result[outer] = group.second;
	return result;
}

} // namespace arnis::element_processing
