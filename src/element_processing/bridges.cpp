#include <vector>
#include <tuple>
#include <cstddef>
#include <optional>
#include <string>
#include <algorithm>
#include <cmath>

#include "block_definitions.hpp"
#include "bresenham.hpp"
#include "../osm_parser.hpp"
#include "world_editor.hpp"

#include "../../../arnis_adapter.h"

namespace arnis
{

void generate_bridges(
		world_editor::WorldEditor &editor, osm_parser::ProcessedWay const &element)
{
	auto it = element.tags.find(std::string("bridge"));
	if (it != element.tags.end()) {
		int bridge_height = 3; // Height above the ground level

		// Need at least 2 nodes for a bridge
		if (element.nodes.size() < 2) {
			return;
		}

		// Get start and end node elevations and use MAX for level bridge deck
		// Using MAX ensures bridges don't dip when multiple bridge ways meet in a valley
		int bridge_deck_ground_y = 0;
		if (!element.nodes.empty()) {
			const auto &start_node = element.nodes.front();
			const auto &end_node = element.nodes.back();
			// Get ground reference from editor
			auto *ground = editor.get_ground();
			if (ground) {
				int start_y = ground->level(XZPoint(start_node.x, start_node.z));
				int end_y = ground->level(XZPoint(end_node.x, end_node.z));
				bridge_deck_ground_y = std::max(start_y, end_y);
			}
		}

		// Calculate total bridge length for ramp positioning
		double total_length = 0.0;
		for (std::size_t i = 1; i < element.nodes.size(); ++i) {
			const auto &prev = element.nodes[i - 1];
			const auto &cur = element.nodes[i];
			double dx = static_cast<double>(cur.x - prev.x);
			double dz = static_cast<double>(cur.z - prev.z);
			total_length += std::sqrt(dx * dx + dz * dz);
		}

		if (total_length == 0.0) {
			return;
		}

		double accumulated_length = 0.0;

		for (std::size_t i = 1; i < element.nodes.size(); ++i) {
			const auto &prev = element.nodes[i - 1];
			const auto &cur = element.nodes[i];

			double segment_dx = static_cast<double>(cur.x - prev.x);
			double segment_dz = static_cast<double>(cur.z - prev.z);
			double segment_length =
					std::sqrt(segment_dx * segment_dx + segment_dz * segment_dz);

			std::vector<std::tuple<int, int, int>> points =
					bresenham::bresenham_line(prev.x, 0, prev.z, cur.x, 0, cur.z);

			// 15% of bridge, min 6, max 20 blocks
			int ramp_length =
					static_cast<int>(std::clamp(total_length * 0.15, 6.0, 20.0));

			for (std::size_t idx = 0; idx < points.size(); ++idx) {
				int x, y, z;
				std::tie(x, y, z) = points[idx];

				// Calculate progress along this segment
				double segment_progress =
						(points.size() > 1)
								? static_cast<double>(idx) /
										  static_cast<double>(points.size() - 1)
								: 0.0;

				// Calculate overall progress along the entire bridge
				double point_distance =
						accumulated_length + segment_progress * segment_length;
				double overall_progress =
						std::clamp(point_distance / total_length, 0.0, 1.0);
				int total_len_int = static_cast<int>(total_length);
				int overall_idx = static_cast<int>(
						overall_progress * static_cast<double>(total_len_int));

				// Calculate ramp height offset
				int ramp_offset;
				if (overall_idx < ramp_length) {
					// Start ramp (rising)
					ramp_offset = static_cast<int>(static_cast<double>(overall_idx) *
												   static_cast<double>(bridge_height) /
												   static_cast<double>(ramp_length));
				} else if (overall_idx >= total_len_int - ramp_length) {
					// End ramp (descending)
					int dist_from_end = total_len_int - overall_idx;
					ramp_offset = static_cast<int>(static_cast<double>(dist_from_end) *
												   static_cast<double>(bridge_height) /
												   static_cast<double>(ramp_length));
				} else {
					// Middle section (constant height)
					ramp_offset = bridge_height;
				}

				// Use fixed bridge deck height (max of endpoints) plus ramp offset
				int bridge_y = bridge_deck_ground_y + ramp_offset;

				// Place bridge blocks
				for (int dx = -2; dx <= 2; ++dx) {
					editor.set_block_absolute(block_definitions::LIGHT_GRAY_CONCRETE,
							x + dx, bridge_y, z, std::optional<std::vector<Block>>{},
							std::optional<std::vector<Block>>{});
				}
			}

			accumulated_length += segment_length;
		}
	}
}

}

#if 0 // Duplicate legacy map implementation; canonical code lives in bridges_maps.cpp.
// BridgeStructureMap implementation
BridgeStructureMap BridgeStructureMap::build(
		const std::vector<ProcessedElement> &elements,
		const WorldEditor &editor,
		const bridge_styles::BridgeOutlineIndex &outlines) {
	BridgeStructureMap result;

	std::vector<const ProcessedWay*> bridge_ways;
	std::vector<const ProcessedWay*> other_highway_ways;

	for (const auto &elem : elements) {
		if (elem.is_way()) {
			const auto &way = elem.as_way();
			if (!way.tags.contains("highway") || way.nodes.size() < 2) continue;
			if (is_bridge_way(way)) {
				bridge_ways.push_back(&way);
			} else {
				other_highway_ways.push_back(&way);
			}
		}
	}

	if (bridge_ways.empty()) return result;

	// Build bridge member info for each bridge way
	for (const auto *way : bridge_ways) {
		if (way->nodes.size() < 2) continue;

		const auto &start = way->nodes.front();
		const auto &end = way->nodes.back();
		auto *ground = const_cast<WorldEditor*>(&editor)->get_ground();

		BridgeMemberInfo info;
		if (ground) {
			info.deck_y = std::max(
				ground->level(XZPoint(start.x, start.z)),
				ground->level(XZPoint(end.x, end.z))
			);
		}

		// Determine bridge style
		info.style = bridge_styles::resolve_bridge_style(way->tags);

		auto way_id = way->id;
		result.members_.insert({way_id, info});
	}

	return result;
}

BridgeMemberInfo* BridgeStructureMap::lookup_member(std::uint64_t way_id) {
	auto it = members_.find(way_id);
	return it != members_.end() ? &it->second : nullptr;
}

const BridgeMemberInfo* BridgeStructureMap::lookup_member(std::uint64_t way_id) const {
	auto it = members_.find(way_id);
	return it != members_.end() ? &it->second : nullptr;
}

BridgeRampInfo* BridgeStructureMap::lookup_ramp(std::uint64_t way_id) {
	auto it = ramps_.find(way_id);
	return it != ramps_.end() ? &it->second : nullptr;
}

const BridgeRampInfo* BridgeStructureMap::lookup_ramp(std::uint64_t way_id) const {
	auto it = ramps_.find(way_id);
	return it != ramps_.end() ? &it->second : nullptr;
}

// BridgeSurfaceMap implementation
BridgeSurfaceMap BridgeSurfaceMap::build(
		const std::vector<ProcessedElement> &elements,
		const BridgeStructureMap &structures,
		double scale) {
	BridgeSurfaceMap result;

	for (const auto &elem : elements) {
		if (elem.is_way()) {
			const auto &way = elem.as_way();
			auto way_id = way->id;
			auto *member = const_cast<BridgeStructureMap*>(&structures)->lookup_member(way_id);
			if (!member) continue;

			int deck_y = member->deck_y;
			if (deck_y == 0) continue;

			// Mark deck surface
			for (size_t i = 1; i < way.nodes.size(); ++i) {
				const auto &prev = way.nodes[i - 1];
				const auto &cur = way.nodes[i];

				double dx = cur.x - prev.x;
				double dz = cur.z - prev.z;
				double dist = std::sqrt(dx * dx + dz * dz);
				int steps = static_cast<int>(std::max(1.0, dist * scale));

				for (int t = 0; t <= steps; ++t) {
					int x = static_cast<int>(std::round(prev.x + (dx * t / steps)));
					int z = static_cast<int>(std::round(prev.z + (dz * t / steps)));

					// 2 blocks on either side
					for (int w = -2; w <= 2; ++w) {
						float px = static_cast<float>(dz) / (dist + 0.01f);
						float pz = static_cast<float>(-dx) / (dist + 0.01f);
						int wx = static_cast<int>(std::round(x + px * w));
						int wz = static_cast<int>(std::round(z + pz * w));
						result.deck_y_.try_emplace({wx, wz}, deck_y);
					}
				}
			}
		}
	}

	return result;
}

std::optional<int> BridgeSurfaceMap::deck_y_at(int x, int z) const {
	auto it = deck_y_.find({x, z});
	return it != deck_y_.end() ? std::optional<int>(it->second) : std::nullopt;
}

std::optional<int> BridgeSurfaceMap::nearby_deck_y(int x, int z, int radius) const {
	// Simple nearest-neighbor search
	for (int r = 0; r <= radius; ++r) {
		for (int dz = -r; dz <= r; ++dz) {
			for (int dx = -r; dx <= r; ++dx) {
				if (std::max(std::abs(dx), std::abs(dz)) != r) continue;
				auto it = deck_y_.find({x + dx, z + dz});
				if (it != deck_y_.end()) {
					return it->second;
				}
			}
		}
	}
	return std::nullopt;
}

bool BridgeSurfaceMap::contains(int x, int z) const {
	return deck_y_at(x, z).has_value();
}

bool is_bridge_way(const ProcessedWay &way) {
	return way.tags.contains("bridge") || way.tags.contains("layer");
}

} // namespace bridges
#endif
