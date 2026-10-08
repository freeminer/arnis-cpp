#include "index.h"

#include "../../buildings.h"
#include "../../way_segments.h"
#include "../../../strict_parse.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <limits>
#include <string_view>

namespace arnis::interior_uses
{
namespace
{
constexpr int BUCKET = 32;

bool point_in_ring(int x, int z, const std::vector<std::pair<int, int>> &ring);

struct Outline
{
	std::uint64_t id{};
	std::vector<std::pair<int, int>> ring;
	int min_x{}, min_z{}, max_x{}, max_z{};
	bool ground{true};
	std::int64_t area{};
	double top_m{4.0};

	static std::optional<Outline> make(
			std::uint64_t id, const std::vector<ProcessedNode> &nodes, const tags_t &tags)
	{
		if (nodes.size() < 4)
			return std::nullopt;
		Outline result;
		result.id = id;
		result.ring.reserve(nodes.size());
		for (const auto &node : nodes)
			result.ring.emplace_back(node.x, node.z);
		if (result.ring.empty())
			return std::nullopt;
		result.min_x = result.max_x = result.ring.front().first;
		result.min_z = result.max_z = result.ring.front().second;
		std::int64_t twice_area = 0;
		for (std::size_t i = 0; i < result.ring.size(); ++i) {
			const auto [x, z] = result.ring[i];
			result.min_x = std::min(result.min_x, x);
			result.max_x = std::max(result.max_x, x);
			result.min_z = std::min(result.min_z, z);
			result.max_z = std::max(result.max_z, z);
			if (i + 1 < result.ring.size()) {
				const auto [nx, nz] = result.ring[i + 1];
				twice_area += static_cast<std::int64_t>(x) * nz -
							  static_cast<std::int64_t>(nx) * z;
			}
		}
		result.area = std::llabs(twice_area);
		auto positive = [&](const char *key) -> std::optional<double> {
			const auto it = tags.find(key);
			if (it == tags.end())
				return std::nullopt;
			std::string_view value = it->second;
			auto trim = [&]() {
				while (!value.empty() &&
						std::isspace(static_cast<unsigned char>(value.front())))
					value.remove_prefix(1);
				while (!value.empty() &&
						std::isspace(static_cast<unsigned char>(value.back())))
					value.remove_suffix(1);
			};
			trim();
			while (!value.empty() && value.back() == 'm')
				value.remove_suffix(1);
			trim();
			const auto parsed = strict_parse::f64(value);
			if (!parsed || !(*parsed > 0.0))
				return std::nullopt;
			return parsed;
		};
		if (const auto height = positive("height"))
			result.top_m = *height;
		else if (const auto levels = positive("building:levels"))
			result.top_m = *levels * 3.0 + 2.0;
		if (positive("building:min_level") || positive("min_height"))
			result.ground = false;
		return result;
	}

	std::pair<int, int> center() const
	{
		const auto n = std::max<std::size_t>(1, ring.size() - 1);
		std::int64_t x = 0, z = 0;
		for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
			x += ring[i].first;
			z += ring[i].second;
		}
		return {static_cast<int>(x / static_cast<std::int64_t>(n)),
				static_cast<int>(z / static_cast<std::int64_t>(n))};
	}
	std::int64_t bbox_area() const
	{
		return static_cast<std::int64_t>(max_x - min_x + 1) * (max_z - min_z + 1);
	}
	bool contains(int x, int z) const
	{
		return x >= min_x && x <= max_x && z >= min_z && z <= max_z &&
			   point_in_ring(x, z, ring);
	}
	bool overlaps(const Outline &other) const;
};

bool point_in_ring(int x, int z, const std::vector<std::pair<int, int>> &ring)
{
	if (ring.size() < 3)
		return false;
	bool inside = false;
	std::size_t j = ring.size() - 1;
	for (std::size_t i = 0; i < ring.size(); ++i) {
		const auto [xi, zi] = ring[i];
		const auto [xj, zj] = ring[j];
		if ((zi > z) != (zj > z) && x < ((static_cast<double>(xj) - xi) * (z - zi) /
												(static_cast<double>(zj) - zi)) +
													xi)
			inside = !inside;
		j = i;
	}
	return inside;
}

bool segments_cross(std::pair<int, int> a, std::pair<int, int> b, std::pair<int, int> c,
		std::pair<int, int> d)
{
	auto side = [](auto p, auto q, auto r) {
		const auto cross =
				static_cast<std::int64_t>(q.first - p.first) * (r.second - p.second) -
				static_cast<std::int64_t>(q.second - p.second) * (r.first - p.first);
		return cross > 0 ? 1 : cross < 0 ? -1 : 0;
	};
	return side(a, b, c) * side(a, b, d) < 0 && side(c, d, a) * side(c, d, b) < 0;
}

bool Outline::overlaps(const Outline &other) const
{
	if (min_x > other.max_x || other.min_x > max_x || min_z > other.max_z ||
			other.min_z > max_z)
		return false;
	for (const auto &[x, z] : other.ring)
		if (point_in_ring(x, z, ring))
			return true;
	for (const auto &[x, z] : ring)
		if (point_in_ring(x, z, other.ring))
			return true;
	for (std::size_t i = 1; i < ring.size(); ++i)
		for (std::size_t j = 1; j < other.ring.size(); ++j)
			if (segments_cross(ring[i - 1], ring[i], other.ring[j - 1], other.ring[j]))
				return true;
	return false;
}

struct Grid
{
	std::vector<Outline> outlines;
	struct PairHash
	{
		std::size_t operator()(const std::pair<int, int> &p) const noexcept
		{
			return std::hash<int>{}(p.first) ^
				   (static_cast<std::size_t>(std::hash<int>{}(p.second)) << 1);
		}
	};
	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, PairHash> buckets;

	void add(Outline outline)
	{
		const auto index = outlines.size();
		for (int x = static_cast<int>(
					 std::floor(static_cast<double>(outline.min_x) / BUCKET));
				x <=
				static_cast<int>(std::floor(static_cast<double>(outline.max_x) / BUCKET));
				++x)
			for (int z = static_cast<int>(
						 std::floor(static_cast<double>(outline.min_z) / BUCKET));
					z <= static_cast<int>(
								 std::floor(static_cast<double>(outline.max_z) / BUCKET));
					++z)
				buckets[{x, z}].push_back(index);
		outlines.push_back(std::move(outline));
	}
	std::vector<const Outline *> within(int min_x, int min_z, int max_x, int max_z) const
	{
		std::vector<std::size_t> indices;
		for (int x = static_cast<int>(std::floor(static_cast<double>(min_x) / BUCKET));
				x <= static_cast<int>(std::floor(static_cast<double>(max_x) / BUCKET));
				++x)
			for (int z = static_cast<int>(
						 std::floor(static_cast<double>(min_z) / BUCKET));
					z <=
					static_cast<int>(std::floor(static_cast<double>(max_z) / BUCKET));
					++z) {
				auto it = buckets.find({x, z});
				if (it != buckets.end())
					indices.insert(indices.end(), it->second.begin(), it->second.end());
			}
		std::sort(indices.begin(), indices.end());
		indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
		std::vector<const Outline *> result;
		result.reserve(indices.size());
		for (const auto i : indices)
			result.push_back(&outlines[i]);
		return result;
	}
	std::vector<const Outline *> containing(int x, int z) const
	{
		const int bx = static_cast<int>(std::floor(static_cast<double>(x) / BUCKET));
		const int bz = static_cast<int>(std::floor(static_cast<double>(z) / BUCKET));
		std::vector<const Outline *> result;
		const auto it = buckets.find({bx, bz});
		if (it != buckets.end())
			for (const auto i : it->second)
				if (outlines[i].contains(x, z))
					result.push_back(&outlines[i]);
		return result;
	}
};

bool is_building(const tags_t &tags)
{
	for (const char *key : {"building", "building:part"}) {
		const auto it = tags.find(key);
		if (it != tags.end() && it->second != "no")
			return true;
	}
	return false;
}

bool closed(const std::vector<ProcessedNode> &nodes)
{
	return nodes.size() >= 4 && nodes.front().x == nodes.back().x &&
		   nodes.front().z == nodes.back().z;
}

unsigned area_rank(Use use)
{
	return use.kind == UseKind::Factory || use.kind == UseKind::Barn ||
						   use.kind == UseKind::Shop
				   ? 1
				   : 2;
}
} // namespace

bool covers(const std::vector<std::pair<int, int>> &ring, int x, int z)
{
	if (ring.size() < 3)
		return false;
	int min_x = ring.front().first, max_x = min_x;
	int min_z = ring.front().second, max_z = min_z;
	for (const auto &[px, pz] : ring) {
		min_x = std::min(min_x, px);
		max_x = std::max(max_x, px);
		min_z = std::min(min_z, pz);
		max_z = std::max(max_z, pz);
	}
	if (x < min_x - 1 || x > max_x + 1 || z < min_z - 1 || z > max_z + 1)
		return false;
	if (point_in_ring(x, z, ring))
		return true;
	for (std::size_t i = 1; i < ring.size(); ++i) {
		const auto [ax, az] = ring[i - 1];
		const auto [bx, bz] = ring[i];
		const double dx = bx - ax, dz = bz - az;
		const double length2 = dx * dx + dz * dz;
		const double t =
				length2 > 0.0
						? std::clamp(((x - ax) * dx + (z - az) * dz) / length2, 0.0, 1.0)
						: 0.0;
		const double cx = ax + t * dx - x, cz = az + t * dz - z;
		if (cx * cx + cz * cz <= 0.5625)
			return true;
	}
	return false;
}

const std::vector<Tenant> &InteriorUseIndex::tenants(std::uint64_t id) const
{
	static const std::vector<Tenant> empty;
	const auto it = tenants_.find(id);
	return it == tenants_.end() ? empty : it->second;
}
std::optional<Use> InteriorUseIndex::area(std::uint64_t id) const
{
	const auto it = areas_.find(id);
	return it == areas_.end() ? std::nullopt : std::optional<Use>(it->second);
}
const std::vector<Claim> &InteriorUseIndex::claims(std::uint64_t id) const
{
	static const std::vector<Claim> empty;
	const auto it = claims_.find(id);
	return it == claims_.end() ? empty : it->second;
}

InteriorUseIndex InteriorUseIndex::build(
		const std::vector<ProcessedElement> &elements, const ::XZBBox &bbox)
{
	Grid grid;
	for (const auto &element : elements) {
		if (element.is_way()) {
			const auto &way = element.as_way();
			if (is_building(way.tags))
				if (auto outline = Outline::make(way.id, way.nodes, way.tags))
					grid.add(std::move(*outline));
		} else if (element.is_relation()) {
			const auto &relation = element.as_relation();
			if (is_building(relation.tags))
				for (const auto &way : buildings::facade_outer_rings(relation, bbox))
					if (auto outline = Outline::make(way.id, way.nodes, way.tags))
						grid.add(std::move(*outline));
		}
	}
	InteriorUseIndex index;
	if (grid.outlines.empty())
		return index;
	for (const auto &outline : grid.outlines) {
		if (!outline.ground)
			continue;
		for (const auto *other :
				grid.within(outline.min_x, outline.min_z, outline.max_x, outline.max_z)) {
			if (other->id == outline.id || !other->ground ||
					std::pair(other->area, other->id) >=
							std::pair(outline.area, outline.id) ||
					!outline.overlaps(*other))
				continue;
			index.claims_[outline.id].push_back({other->ring, other->top_m});
		}
	}
	auto add_tenant = [&](Use use, int x, int z, std::optional<int> level) {
		auto around = grid.containing(x, z);
		const Outline *ground = nullptr;
		for (const auto *o : around)
			if (o->ground && (!ground || std::pair(o->area, o->id) <
												 std::pair(ground->area, ground->id)))
				ground = o;
		for (const auto *o : around)
			if (!o->ground || o == ground)
				index.tenants_[o->id].push_back({use, x, z, level});
	};
	struct AreaChoice
	{
		unsigned rank;
		std::int64_t neg_area;
		Use use;
	};
	std::unordered_map<std::uint64_t, AreaChoice> best_area;
	auto add_area = [&](const std::vector<Outline> &areas, Use use) {
		for (const auto &area : areas)
			for (const auto *o :
					grid.within(area.min_x, area.min_z, area.max_x, area.max_z)) {
				const auto [cx, cz] = o->center();
				if (std::none_of(areas.begin(), areas.end(),
							[&](const auto &r) { return r.contains(cx, cz); }))
					continue;
				const AreaChoice candidate{area_rank(use), -area.area, use};
				auto it = best_area.find(o->id);
				if (it == best_area.end() ||
						std::pair(candidate.rank, candidate.neg_area) >
								std::pair(it->second.rank, it->second.neg_area))
					best_area[o->id] = candidate;
			}
	};
	for (const auto &element : elements) {
		if (element.is_node()) {
			const auto &node = element.as_node();
			if (const auto use = use_from_tags(node.tags)) {
				std::optional<int> level;
				if (const auto it = node.tags.find("level"); it != node.tags.end())
					level = parse_level(it->second);
				add_tenant(*use, node.x, node.z, level);
			}
		} else if (element.is_way()) {
			const auto &way = element.as_way();
			if (is_building(way.tags) || !closed(way.nodes))
				continue;
			auto outline = Outline::make(way.id, way.nodes, way.tags);
			if (!outline)
				continue;
			if (const auto use = use_from_tags(way.tags)) {
				const auto [cx, cz] = outline->center();
				const auto around = grid.containing(cx, cz);
				const bool fits_inside =
						std::any_of(around.begin(), around.end(), [&](const auto *b) {
							return b->bbox_area() > outline->bbox_area();
						});
				if (fits_inside) {
					std::optional<int> level;
					if (const auto it = way.tags.find("level"); it != way.tags.end())
						level = parse_level(it->second);
					add_tenant(*use, cx, cz, level);
					continue;
				}
			}
			if (const auto use = use_from_area(way.tags))
				add_area(std::vector<Outline>{std::move(*outline)}, *use);
		} else {
			const auto &relation = element.as_relation();
			if (is_building(relation.tags))
				continue;
			const auto use = use_from_area(relation.tags);
			if (!use)
				continue;
			std::vector<std::vector<ProcessedNode>> rings;
			for (const auto &member : relation.members)
				if (member.role == ProcessedMemberRole::Outer)
					rings.push_back(member.way.nodes);
			arnis::merge_way_segments(rings);
			std::vector<Outline> outlines;
			for (const auto &ring : rings)
				if (auto outline = Outline::make(relation.id, ring, relation.tags))
					outlines.push_back(std::move(*outline));
			add_area(outlines, *use);
		}
	}
	for (const auto &[id, choice] : best_area)
		index.areas_[id] = choice.use;
	return index;
}
} // namespace arnis::interior_uses
