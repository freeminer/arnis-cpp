#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../../../arnis_adapter.h"

namespace arnis::bridge_styles
{

enum class BridgeStyle {
	Beam,
	Arch,
	Truss,
	Suspension,
	CableStayed,
	Covered,
	Boardwalk,
};

Block foundation_block(BridgeStyle style);
Block rail_block(BridgeStyle style);
std::size_t pillar_interval(BridgeStyle style);
bool has_side_railing(BridgeStyle style);
std::optional<Block> parapet_block(BridgeStyle style);
Block rail_foundation_block(BridgeStyle style);

struct BridgePathSample {
	int x = 0;
	int y = 0;
	int z = 0;
	float perp_x = 0.0f;
	float perp_z = 0.0f;
};

class BridgeOutlineIndex {
public:
	static BridgeOutlineIndex build(const std::vector<ProcessedElement> &elements);

	std::optional<BridgeStyle> style_for_way(const ProcessedWay &way) const;

private:
	struct OutlineEntry {
		std::vector<std::pair<int, int>> nodes;
		int bbox_min_x = 0;
		int bbox_max_x = 0;
		int bbox_min_z = 0;
		int bbox_max_z = 0;
		std::optional<std::string> structure;
		std::optional<std::string> bridge;
	};

	std::vector<OutlineEntry> entries_;
};

BridgeStyle resolve_bridge_style(const std::unordered_map<std::string, std::string> &tags);
BridgeStyle resolve_bridge_style_with_outline(
		const ProcessedWay &way, const BridgeOutlineIndex &outlines);

void place_bridge_support_below_deck(WorldEditor &editor, BridgeStyle style, int set_x,
		int cell_y, int set_z, int centerline_ground_y, std::size_t tds,
		std::size_t total, bool use_absolute_y, bool is_centerline,
		bool is_pillar_position);

void decorate_bridge_above_deck(WorldEditor &editor, BridgeStyle style,
		const std::vector<BridgePathSample> &path, int block_range,
		bool start_is_boundary, bool end_is_boundary);

}
