#include "tile.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <unordered_map>

namespace arnis
{
namespace
{
struct Aabb
{
	int32_t min_x, max_x, min_z, max_z;
};

struct Region
{
	int32_t x, z;
	bool operator==(const Region &other) const noexcept
	{
		return x == other.x && z == other.z;
	}
};

struct RegionHash
{
	std::size_t operator()(const Region &r) const noexcept
	{
		const auto x = std::hash<int32_t>{}(r.x);
		const auto z = std::hash<int32_t>{}(r.z);
		return x ^ (z + 0x9e3779b9U + (x << 6) + (x >> 2));
	}
};

struct RegionRange
{
	int32_t x0, x1, z0, z1;
};

std::optional<Aabb> way_aabb(const ProcessedWay &way)
{
	if (way.nodes.empty())
		return std::nullopt;
	Aabb box{std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::min(),
			std::numeric_limits<int32_t>::max(), std::numeric_limits<int32_t>::min()};
	for (const auto &node : way.nodes) {
		box.min_x = std::min(box.min_x, node.x);
		box.max_x = std::max(box.max_x, node.x);
		box.min_z = std::min(box.min_z, node.z);
		box.max_z = std::max(box.max_z, node.z);
	}
	return box;
}

bool intersects(const Aabb &box, const TileBounds &bounds)
{
	return box.min_x < bounds.max_x && box.max_x >= bounds.min_x &&
		   box.min_z < bounds.max_z && box.max_z >= bounds.min_z;
}

RegionRange region_range(const Aabb &box, int32_t halo)
{
	return {(box.min_x - halo) >> 9, (box.max_x + halo) >> 9, (box.min_z - halo) >> 9,
			(box.max_z + halo) >> 9};
}

bool is_linear_element(const ProcessedWay &way)
{
	static constexpr const char *keys[] = {
			"highway", "railway", "barrier", "waterway", "power", "man_made", "aeroway"};
	return std::any_of(std::begin(keys), std::end(keys),
			[&](const char *key) { return way.tags.find(key) != way.tags.end(); });
}

void union_aabb(Aabb &into, const Aabb &other)
{
	into.min_x = std::min(into.min_x, other.min_x);
	into.max_x = std::max(into.max_x, other.max_x);
	into.min_z = std::min(into.min_z, other.min_z);
	into.max_z = std::max(into.max_z, other.max_z);
}
} // namespace

std::vector<TileBounds> create_tiles(const ::XZBBox &bbox, int32_t tile_size)
{
	std::vector<TileBounds> tiles;
	if (tile_size <= 0)
		return tiles;
	const int32_t aligned_min_x = (bbox.min_x() >> 9) << 9;
	const int32_t aligned_min_z = (bbox.min_z() >> 9) << 9;
	const int32_t aligned_max_x = ((bbox.max_x() + 512) >> 9) << 9;
	const int32_t aligned_max_z = ((bbox.max_z() + 512) >> 9) << 9;
	for (int64_t z = aligned_min_z; z < aligned_max_z; z += tile_size) {
		for (int64_t x = aligned_min_x; x < aligned_max_x; x += tile_size) {
			const int32_t max_x =
					static_cast<int32_t>(std::min<int64_t>(x + tile_size, aligned_max_x));
			const int32_t max_z =
					static_cast<int32_t>(std::min<int64_t>(z + tile_size, aligned_max_z));
			if (max_x > bbox.min_x() && x <= bbox.max_x() && max_z > bbox.min_z() &&
					z <= bbox.max_z())
				tiles.push_back(
						{static_cast<int32_t>(x), static_cast<int32_t>(z), max_x, max_z});
		}
	}
	return tiles;
}

std::vector<std::vector<std::size_t>> assign_elements_to_tiles(
		const std::vector<ProcessedElement> &elements,
		const std::vector<TileBounds> &tiles, double scale)
{
	std::vector<std::vector<std::size_t>> assigned(tiles.size());
	const int32_t linear_halo = std::max<int32_t>(
			TILE_EDITOR_HALO, static_cast<int32_t>(std::ceil(40.0 * scale)));
	std::unordered_map<Region, std::size_t, RegionHash> tile_grid;
	tile_grid.reserve(tiles.size());
	for (std::size_t i = 0; i < tiles.size(); ++i)
		tile_grid[{tiles[i].min_x >> 9, tiles[i].min_z >> 9}] = i;

	for (std::size_t element_index = 0; element_index < elements.size();
			++element_index) {
		const auto &element = elements[element_index];
		if (element.is_node()) {
			const auto &node = element.as_node();
			const auto aeroway = node.tags.find("aeroway");
			if (aeroway != node.tags.end() && aeroway->second == "helipad") {
				const int32_t reach =
						std::max<int32_t>(
								4, static_cast<int32_t>(std::round(8.0 * scale))) +
						12;
				const Aabb box{
						node.x - reach, node.x + reach, node.z - reach, node.z + reach};
				const auto range = region_range(box, 0);
				for (int64_t rx = range.x0; rx <= range.x1; ++rx) {
					for (int64_t rz = range.z0; rz <= range.z1; ++rz) {
						auto it = tile_grid.find(
								{static_cast<int32_t>(rx), static_cast<int32_t>(rz)});
						if (it != tile_grid.end() && intersects(box, tiles[it->second]))
							assigned[it->second].push_back(element_index);
					}
				}
				continue;
			}
			auto it = tile_grid.find({node.x >> 9, node.z >> 9});
			if (it != tile_grid.end() && tiles[it->second].contains(node.x, node.z))
				assigned[it->second].push_back(element_index);
			continue;
		}

		if (element.is_way()) {
			const auto &way = element.as_way();
			const auto box = way_aabb(way);
			if (!box)
				continue;
			const int32_t halo = is_linear_element(way) ? linear_halo : TILE_EDITOR_HALO;
			const auto range = region_range(*box, halo);
			for (int64_t rx = range.x0; rx <= range.x1; ++rx) {
				for (int64_t rz = range.z0; rz <= range.z1; ++rz) {
					auto it = tile_grid.find(
							{static_cast<int32_t>(rx), static_cast<int32_t>(rz)});
					if (it != tile_grid.end() &&
							intersects(*box, tiles[it->second].expanded(halo)))
						assigned[it->second].push_back(element_index);
				}
			}
			continue;
		}

		const auto &relation = element.as_relation();
		std::vector<Aabb> member_boxes;
		member_boxes.reserve(relation.members.size());
		for (const auto &member : relation.members) {
			if (auto box = way_aabb(member.way))
				member_boxes.push_back(*box);
		}
		if (member_boxes.empty())
			continue;
		Aabb union_box = member_boxes.front();
		for (std::size_t i = 1; i < member_boxes.size(); ++i)
			union_aabb(union_box, member_boxes[i]);
		const auto range = region_range(union_box, TILE_EDITOR_HALO);
		for (int64_t rx = range.x0; rx <= range.x1; ++rx) {
			for (int64_t rz = range.z0; rz <= range.z1; ++rz) {
				auto it = tile_grid.find(
						{static_cast<int32_t>(rx), static_cast<int32_t>(rz)});
				if (it == tile_grid.end())
					continue;
				const auto expanded = tiles[it->second].expanded(TILE_EDITOR_HALO);
				if (std::any_of(member_boxes.begin(), member_boxes.end(),
							[&](const Aabb &box) { return intersects(box, expanded); }))
					assigned[it->second].push_back(element_index);
			}
		}
	}
	return assigned;
}

} // namespace arnis
