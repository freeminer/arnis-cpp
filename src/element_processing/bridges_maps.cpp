#include "bridges.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "../bresenham.h"

namespace arnis::bridges
{

namespace
{
constexpr int LAYER_HEIGHT_STEP = 6;
constexpr int FLAT_TERRAIN_DIP_THRESHOLD = 4;
constexpr std::size_t SHORT_BRIDGE_LENGTH_BLOCKS = 30;
constexpr int BRIDGE_NAME_FUSE_DISTANCE_BLOCKS = 200;
constexpr float DUAL_CARRIAGEWAY_MAX_DISTANCE_BLOCKS = 12.0f;
constexpr float DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG = 20.0f;
constexpr std::size_t CENTERLINE_SAMPLE_LIMIT = 5;

struct XZPairHash
{
	std::size_t operator()(const std::pair<int, int> &p) const noexcept
	{
		return std::hash<long long>()((static_cast<long long>(p.first) << 32) ^
									  static_cast<unsigned long long>(p.second));
	}
};

class UnionFind
{
public:
	explicit UnionFind(std::size_t n) : parent_(n), rank_(n, 0)
	{
		for (std::size_t i = 0; i < n; ++i)
			parent_[i] = i;
	}

	std::size_t find(std::size_t i)
	{
		while (parent_[i] != i) {
			parent_[i] = parent_[parent_[i]];
			i = parent_[i];
		}
		return i;
	}

	void unite(std::size_t a, std::size_t b)
	{
		const auto ra = find(a);
		const auto rb = find(b);
		if (ra == rb)
			return;
		if (rank_[ra] < rank_[rb]) {
			parent_[ra] = rb;
		} else if (rank_[ra] > rank_[rb]) {
			parent_[rb] = ra;
		} else {
			parent_[rb] = ra;
			++rank_[ra];
		}
	}

private:
	std::vector<std::size_t> parent_;
	std::vector<unsigned char> rank_;
};

int effective_layer(const ProcessedWay &way)
{
	const auto it = way.tags.find("layer");
	if (it == way.tags.end())
		return is_bridge_way(way) ? 1 : 0;
	try {
		return std::max(0, std::stoi(it->second));
	} catch (...) {
		return is_bridge_way(way) ? 1 : 0;
	}
}

int highway_block_range(const ProcessedWay &way, double scale)
{
	const auto h = way.tags.get("highway");
	int block_range = 2;
	if (h == "footway" || h == "pedestrian" || h == "path" || h == "track" ||
			h == "secondary_link" || h == "tertiary_link" || h == "escape" ||
			h == "steps") {
		block_range = 1;
	} else if (h == "motorway" || h == "primary" || h == "trunk") {
		block_range = 5;
	} else if (h == "secondary") {
		block_range = 4;
	} else if (h == "tertiary") {
		block_range = 2;
	} else if (h == "service") {
		block_range = 2;
	} else if (const auto it = way.tags.find("lanes"); it != way.tags.end()) {
		if (it->second == "2")
			block_range = 3;
		else if (it->second != "1")
			block_range = 4;
	}
	if (scale < 1.0)
		block_range = static_cast<int>(std::floor(block_range * scale));
	return std::max(0, block_range);
}

bool is_ramp_candidate(const ProcessedWay &way)
{
	if (is_bridge_way(way))
		return false;
	if (way.tags.get("indoor") == "yes")
		return false;
	if (const auto it = way.tags.find("embankment");
			it != way.tags.end() && it->second != "no")
		return true;
	if (way.tags.get("man_made") == "embankment")
		return true;
	if (const auto it = way.tags.find("layer"); it != way.tags.end()) {
		try {
			return std::stoi(it->second) >= 1;
		} catch (...) {
			return false;
		}
	}
	return false;
}

bool is_oneway(const ProcessedWay &way)
{
	const auto oneway = way.tags.get("oneway");
	return oneway == "yes" || oneway == "-1" || oneway == "true";
}

std::size_t way_length_blocks(const ProcessedWay &way)
{
	std::size_t total = 0;
	for (std::size_t i = 1; i < way.nodes.size(); ++i) {
		const auto &a = way.nodes[i - 1];
		const auto &b = way.nodes[i];
		const float dx = static_cast<float>(b.x - a.x);
		const float dz = static_cast<float>(b.z - a.z);
		total += static_cast<std::size_t>(std::sqrt(dx * dx + dz * dz));
	}
	return total;
}

std::pair<int, int> centroid(const ProcessedWay &way)
{
	if (way.nodes.empty())
		return {0, 0};
	long long sx = 0;
	long long sz = 0;
	for (const auto &node : way.nodes) {
		sx += node.x;
		sz += node.z;
	}
	const auto n = static_cast<long long>(way.nodes.size());
	return {static_cast<int>(sx / n), static_cast<int>(sz / n)};
}

std::vector<std::pair<int, int>> centerline_samples(const ProcessedWay &way)
{
	std::vector<std::pair<int, int>> out;
	if (way.nodes.empty())
		return out;
	const std::size_t last = way.nodes.size() - 1;
	out.emplace_back(way.nodes.front().x, way.nodes.front().z);
	if (last > 0)
		out.emplace_back(way.nodes.back().x, way.nodes.back().z);
	if (last >= 2) {
		const std::size_t interior = last - 1;
		const std::size_t take = std::min(interior, CENTERLINE_SAMPLE_LIMIT);
		if (take > 0) {
			const std::size_t step =
					std::max<std::size_t>(1, std::max<std::size_t>(1, interior) / take);
			for (std::size_t idx = 1;
					idx < last && out.size() < CENTERLINE_SAMPLE_LIMIT + 2; idx += step) {
				out.emplace_back(way.nodes[idx].x, way.nodes[idx].z);
			}
		}
	}
	return out;
}

std::optional<float> heading_deg(const ProcessedWay &way)
{
	if (way.nodes.size() < 2)
		return std::nullopt;
	const auto &s = way.nodes.front();
	const auto &e = way.nodes.back();
	const float dx = static_cast<float>(e.x - s.x);
	const float dz = static_cast<float>(e.z - s.z);
	if (dx == 0.0f && dz == 0.0f)
		return std::nullopt;
	constexpr float pi = 3.14159265358979323846f;
	return std::atan2(dz, dx) * 180.0f / pi;
}

std::pair<int, int> midpoint(const ProcessedWay &way)
{
	const auto &n = way.nodes[way.nodes.size() / 2];
	return {n.x, n.z};
}

bool are_dual_carriageway_pair(const ProcessedWay &a, const ProcessedWay &b)
{
	const auto ha = heading_deg(a);
	const auto hb = heading_deg(b);
	if (!ha || !hb)
		return false;
	float diff = std::fmod(std::abs(*ha - *hb), 360.0f);
	if (diff > 180.0f)
		diff = 360.0f - diff;
	const bool parallel =
			diff <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG ||
			std::abs(180.0f - diff) <= DUAL_CARRIAGEWAY_HEADING_TOLERANCE_DEG;
	if (!parallel)
		return false;
	const auto ma = midpoint(a);
	const auto mb = midpoint(b);
	const float dx = static_cast<float>(ma.first - mb.first);
	const float dz = static_cast<float>(ma.second - mb.second);
	return std::sqrt(dx * dx + dz * dz) <= DUAL_CARRIAGEWAY_MAX_DISTANCE_BLOCKS;
}

std::optional<int> decide_internal_ramp(const std::pair<int, int> &xz, int deck_y,
		const std::unordered_map<std::pair<int, int>, std::size_t, XZPairHash>
				&endpoint_counts,
		const std::unordered_map<std::pair<int, int>, bool, XZPairHash>
				&boundary_with_external_ramp,
		const WorldEditor &editor)
{
	const auto count_it = endpoint_counts.find(xz);
	if (count_it != endpoint_counts.end() && count_it->second > 1)
		return std::nullopt;
	const auto ramp_it = boundary_with_external_ramp.find(xz);
	if (ramp_it != boundary_with_external_ramp.end() && ramp_it->second)
		return std::nullopt;
	const int ground_y = editor.get_ground_level(xz.first, xz.second);
	return deck_y > ground_y ? std::optional<int>(ground_y) : std::nullopt;
}

bridge_styles::BridgeStyle majority_style(const std::vector<std::size_t> &group_indices,
		const std::vector<const ProcessedWay *> &bridge_ways,
		const bridge_styles::BridgeOutlineIndex &outlines)
{
	std::unordered_map<bridge_styles::BridgeStyle, std::size_t> counts;
	for (const auto idx : group_indices) {
		const auto style = bridge_styles::resolve_bridge_style_with_outline(
				*bridge_ways[idx], outlines);
		++counts[style];
	}
	const bridge_styles::BridgeStyle priority[] = {
			bridge_styles::BridgeStyle::Suspension,
			bridge_styles::BridgeStyle::CableStayed,
			bridge_styles::BridgeStyle::Arch,
			bridge_styles::BridgeStyle::Truss,
			bridge_styles::BridgeStyle::Covered,
			bridge_styles::BridgeStyle::Boardwalk,
			bridge_styles::BridgeStyle::Beam,
	};
	bridge_styles::BridgeStyle best = bridge_styles::BridgeStyle::Beam;
	std::size_t best_count = 0;
	for (const auto style : priority) {
		const auto count = counts[style];
		if (count > best_count) {
			best = style;
			best_count = count;
		}
	}
	return best;
}
}

bool is_bridge_way(const ProcessedWay &way)
{
	if (way.tags.get("indoor") == "yes")
		return false;
	const auto it = way.tags.find("bridge");
	return it != way.tags.end() && it->second != "no";
}

int BridgeMemberInfo::y_at(
		std::size_t tds, std::size_t total_bresenham, std::size_t ramp_length) const
{
	if (total_bresenham == 0)
		return deck_y;
	const auto last_idx = total_bresenham - 1;
	const float denom = static_cast<float>(std::max<std::size_t>(1, ramp_length - 1));
	if (start_internal_ramp && tds < ramp_length) {
		const float t = std::min(1.0f, static_cast<float>(tds) / denom);
		return static_cast<int>(
				std::round(*start_internal_ramp + (deck_y - *start_internal_ramp) * t));
	}
	const auto dist_from_end = last_idx > tds ? last_idx - tds : 0;
	if (end_internal_ramp && dist_from_end < ramp_length) {
		const float t = std::min(1.0f, static_cast<float>(dist_from_end) / denom);
		return static_cast<int>(
				std::round(*end_internal_ramp + (deck_y - *end_internal_ramp) * t));
	}
	return deck_y;
}

int BridgeRampInfo::y_at(std::size_t tds, std::size_t total_bresenham) const
{
	if (total_bresenham == 0)
		return deck_y;
	const float denom = static_cast<float>(std::max<std::size_t>(1, total_bresenham - 1));
	const int start_y = bridge_side_at_start ? deck_y : ground_y;
	const int end_y = bridge_side_at_start ? ground_y : deck_y;
	const float t = std::min(1.0f, static_cast<float>(tds) / denom);
	return static_cast<int>(std::round(start_y + (end_y - start_y) * t));
}

BridgeStructureMap BridgeStructureMap::build(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor)
{
	const auto outlines = bridge_styles::BridgeOutlineIndex::build(elements);
	return build(elements, editor, outlines);
}

BridgeStructureMap BridgeStructureMap::build(
		const std::vector<ProcessedElement> &elements, const WorldEditor &editor,
		const bridge_styles::BridgeOutlineIndex &outlines)
{
	BridgeStructureMap result;
	std::vector<const ProcessedWay *> bridge_ways;
	std::vector<const ProcessedWay *> other_highway_ways;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !way.tags.contains("highway"))
			continue;
		if (is_bridge_way(way))
			bridge_ways.push_back(&way);
		else
			other_highway_ways.push_back(&way);
	}
	if (bridge_ways.empty())
		return result;

	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash>
			node_to_bridge_indices;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		const auto &way = *bridge_ways[i];
		const auto &start = way.nodes.front();
		const auto &end = way.nodes.back();
		node_to_bridge_indices[{start.x, start.z}].push_back(i);
		if (end.x != start.x || end.z != start.z)
			node_to_bridge_indices[{end.x, end.z}].push_back(i);
	}

	UnionFind uf(bridge_ways.size());

	for (const auto &entry : node_to_bridge_indices) {
		const auto &indices = entry.second;
		if (indices.size() < 2)
			continue;
		std::unordered_map<int, std::vector<std::size_t>> by_layer;
		for (const auto idx : indices)
			by_layer[effective_layer(*bridge_ways[idx])].push_back(idx);
		for (const auto &layer_entry : by_layer) {
			const auto &group = layer_entry.second;
			if (group.size() < 2)
				continue;
			for (std::size_t i = 1; i < group.size(); ++i)
				uf.unite(group.front(), group[i]);
		}
	}

	std::unordered_map<std::string, std::vector<std::size_t>> by_name;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		const auto name = bridge_ways[i]->tags.get("bridge:name");
		if (!name.empty())
			by_name[name].push_back(i);
	}
	for (const auto &entry : by_name) {
		const auto &group = entry.second;
		if (group.size() < 2)
			continue;
		std::vector<std::pair<int, int>> centroids;
		centroids.reserve(group.size());
		for (const auto idx : group)
			centroids.push_back(centroid(*bridge_ways[idx]));
		for (std::size_t i = 0; i < group.size(); ++i) {
			for (std::size_t j = i + 1; j < group.size(); ++j) {
				const auto a = group[i];
				const auto b = group[j];
				if (effective_layer(*bridge_ways[a]) != effective_layer(*bridge_ways[b]))
					continue;
				if (std::abs(centroids[i].first - centroids[j].first) <=
								BRIDGE_NAME_FUSE_DISTANCE_BLOCKS &&
						std::abs(centroids[i].second - centroids[j].second) <=
								BRIDGE_NAME_FUSE_DISTANCE_BLOCKS) {
					uf.unite(a, b);
				}
			}
		}
	}

	std::vector<std::size_t> oneway_indices;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i) {
		if (is_oneway(*bridge_ways[i]))
			oneway_indices.push_back(i);
	}
	for (std::size_t ai = 0; ai < oneway_indices.size(); ++ai) {
		for (std::size_t bi = ai + 1; bi < oneway_indices.size(); ++bi) {
			const auto a = oneway_indices[ai];
			const auto b = oneway_indices[bi];
			if (uf.find(a) == uf.find(b))
				continue;
			if (effective_layer(*bridge_ways[a]) != effective_layer(*bridge_ways[b]))
				continue;
			if (are_dual_carriageway_pair(*bridge_ways[a], *bridge_ways[b]))
				uf.unite(a, b);
		}
	}

	std::unordered_map<std::size_t, std::vector<std::size_t>> groups;
	for (std::size_t i = 0; i < bridge_ways.size(); ++i)
		groups[uf.find(i)].push_back(i);

	std::unordered_map<std::pair<int, int>, std::vector<std::size_t>, XZPairHash>
			node_to_other_highways;
	for (std::size_t i = 0; i < other_highway_ways.size(); ++i) {
		const auto &way = *other_highway_ways[i];
		const auto &start = way.nodes.front();
		const auto &end = way.nodes.back();
		node_to_other_highways[{start.x, start.z}].push_back(i);
		if (end.x != start.x || end.z != start.z)
			node_to_other_highways[{end.x, end.z}].push_back(i);
	}

	std::unordered_set<std::uint64_t> claimed_ramp_ways;

	for (const auto &group_entry : groups) {
		const auto &group_indices = group_entry.second;
		const auto group_style = majority_style(group_indices, bridge_ways, outlines);
		std::unordered_map<std::pair<int, int>, std::size_t, XZPairHash> endpoint_counts;
		for (const auto idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto &s = way.nodes.front();
			const auto &e = way.nodes.back();
			++endpoint_counts[{s.x, s.z}];
			if (e.x != s.x || e.z != s.z)
				++endpoint_counts[{e.x, e.z}];
		}

		int max_layer = 0;
		bool had_unlabelled = false;
		for (const auto idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto it = way.tags.find("layer");
			if (it == way.tags.end()) {
				had_unlabelled = true;
				continue;
			}
			try {
				max_layer = std::max(max_layer, std::max(0, std::stoi(it->second)));
			} catch (...) {
				had_unlabelled = true;
			}
		}
		if (max_layer == 0 && had_unlabelled)
			max_layer = 1;

		std::vector<int> terrain_samples;
		for (const auto idx : group_indices) {
			for (const auto sample : centerline_samples(*bridge_ways[idx]))
				terrain_samples.push_back(
						editor.get_ground_level(sample.first, sample.second));
		}
		if (terrain_samples.empty())
			continue;
		const auto [terrain_min_it, terrain_max_it] =
				std::minmax_element(terrain_samples.begin(), terrain_samples.end());
		const int terrain_min = *terrain_min_it;
		const int terrain_max = *terrain_max_it;
		const int dip = terrain_max - terrain_min;
		std::size_t total_length = 0;
		for (const auto idx : group_indices)
			total_length += way_length_blocks(*bridge_ways[idx]);
		const int style_clearance =
				group_style == bridge_styles::BridgeStyle::Arch ? 8 : 0;
		const int clearance =
				(dip < FLAT_TERRAIN_DIP_THRESHOLD &&
						total_length >= SHORT_BRIDGE_LENGTH_BLOCKS)
						? std::max(max_layer * LAYER_HEIGHT_STEP, style_clearance)
						: 0;
		const int deck_y = terrain_max + clearance;

		std::unordered_map<std::pair<int, int>, bool, XZPairHash>
				boundary_with_external_ramp;
		for (const auto &entry : endpoint_counts) {
			const auto xz = entry.first;
			const auto count = entry.second;
			if (count > 1)
				continue;
			const auto other_it = node_to_other_highways.find(xz);
			if (other_it == node_to_other_highways.end()) {
				boundary_with_external_ramp[xz] = false;
				continue;
			}

			bool found_ramp = false;
			for (const auto oi : other_it->second) {
				const auto &candidate = *other_highway_ways[oi];
				if (!is_ramp_candidate(candidate))
					continue;
				if (claimed_ramp_ways.contains(candidate.id))
					continue;
				const bool bridge_side_at_start = candidate.nodes.front().x == xz.first &&
												  candidate.nodes.front().z == xz.second;
				const auto &far_node = bridge_side_at_start ? candidate.nodes.back()
															: candidate.nodes.front();
				if (endpoint_counts.contains({far_node.x, far_node.z}))
					continue;

				BridgeRampInfo info;
				info.bridge_side_at_start = bridge_side_at_start;
				info.deck_y = deck_y;
				info.ground_y = editor.get_ground_level(far_node.x, far_node.z);
				result.ramps_.emplace(candidate.id, info);
				claimed_ramp_ways.insert(candidate.id);
				found_ramp = true;
			}
			boundary_with_external_ramp[xz] = found_ramp;
		}

		for (const auto idx : group_indices) {
			const auto &way = *bridge_ways[idx];
			const auto &s = way.nodes.front();
			const auto &e = way.nodes.back();
			BridgeMemberInfo info;
			info.deck_y = deck_y;
			info.style = group_style;
			info.start_internal_ramp = decide_internal_ramp({s.x, s.z}, deck_y,
					endpoint_counts, boundary_with_external_ramp, editor);
			info.end_internal_ramp = decide_internal_ramp({e.x, e.z}, deck_y,
					endpoint_counts, boundary_with_external_ramp, editor);
			result.members_.emplace(way.id, info);
		}
	}
	return result;
}

const BridgeMemberInfo *BridgeStructureMap::lookup_member(std::uint64_t way_id) const
{
	auto it = members_.find(way_id);
	return it == members_.end() ? nullptr : &it->second;
}

const BridgeRampInfo *BridgeStructureMap::lookup_ramp(std::uint64_t way_id) const
{
	auto it = ramps_.find(way_id);
	return it == ramps_.end() ? nullptr : &it->second;
}

BridgeSurfaceMap BridgeSurfaceMap::build(const std::vector<ProcessedElement> &elements,
		const BridgeStructureMap &structures, double scale)
{
	BridgeSurfaceMap result;
	for (const auto &element : elements) {
		if (!element.is_way())
			continue;
		const auto &way = element.as_way();
		if (way.nodes.size() < 2 || !way.tags.contains("highway"))
			continue;
		const auto *info = structures.lookup_member(way.id);
		const auto *ramp = structures.lookup_ramp(way.id);
		if (!info && !ramp)
			continue;
		const int block_range = highway_block_range(way, scale);

		std::size_t total_bresenham = 1;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &a = way.nodes[i - 1];
			const auto &b = way.nodes[i];
			total_bresenham += static_cast<std::size_t>(
					std::max(std::abs(b.x - a.x), std::abs(b.z - a.z)));
		}
		const std::size_t raw_ramp = static_cast<std::size_t>(
				std::clamp(static_cast<float>(total_bresenham) * 0.35f, 15.0f, 50.0f));
		const std::size_t internal_ramp_length = std::clamp<std::size_t>(
				raw_ramp, 1, std::max<std::size_t>(1, total_bresenham / 2));

		std::size_t tds = 0;
		for (std::size_t i = 1; i < way.nodes.size(); ++i) {
			const auto &prev = way.nodes[i - 1];
			const auto &cur = way.nodes[i];
			const auto points = bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);
			const std::size_t skip_first = i == 1 ? 0 : 1;
			for (std::size_t point_idx = skip_first; point_idx < points.size();
					++point_idx) {
				const auto &p = points[point_idx];
				const int x = std::get<0>(p);
				const int z = std::get<2>(p);
				const int cell_y =
						info ? info->y_at(tds, total_bresenham, internal_ramp_length)
							 : ramp->y_at(tds, total_bresenham);
				for (int dx = -block_range; dx <= block_range; ++dx) {
					for (int dz = -block_range; dz <= block_range; ++dz) {
						auto &existing = result.deck_y_[{x + dx, z + dz}];
						existing = std::max(existing, cell_y);
					}
				}
				++tds;
			}
		}
	}
	return result;
}

std::optional<int> BridgeSurfaceMap::deck_y_at(int x, int z) const
{
	auto it = deck_y_.find({x, z});
	if (it == deck_y_.end())
		return std::nullopt;
	return it->second;
}

std::optional<int> BridgeSurfaceMap::nearby_deck_y(int x, int z, int radius) const
{
	if (auto direct = deck_y_at(x, z))
		return direct;
	std::optional<int> found;
	for (int r = 1; r <= radius; ++r) {
		for (int dx = -r; dx <= r; ++dx) {
			for (int dz = -r; dz <= r; ++dz) {
				if (std::abs(dx) != r && std::abs(dz) != r)
					continue;
				if (auto y = deck_y_at(x + dx, z + dz))
					found = found ? std::max(*found, *y) : *y;
			}
		}
	}
	return found;
}

}
