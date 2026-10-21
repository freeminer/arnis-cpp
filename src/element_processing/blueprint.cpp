#include "blueprint.h"

#include "../block_definitions.h"
#include "../bresenham.h"
#include "../strict_parse.h"

namespace arnis::blueprint
{
namespace
{
bool underground_building(const tags_t &tags)
{
	if (const auto location = tags.get("location"); !location.empty()) {
		if (location == "underground" || location == "subway")
			return true;
		if (location == "surface" || location == "overground" || location == "roof")
			return false;
	}
	for (const char *key : {"layer", "level"})
		if (const auto value = tags.get(key);
				!value.empty() && strict_parse::i32_or_zero(value) < 0)
			return true;
	return tags.contains("building:levels:underground") &&
		   !tags.contains("building:levels");
}

void draw_ring(WorldEditor &editor, const std::vector<ProcessedNode> &nodes)
{
	for (std::size_t i = 1; i < nodes.size(); ++i)
		for (const auto &[x, y, z] : bresenham_line(
					 nodes[i - 1].x, 0, nodes[i - 1].z, nodes[i].x, 0, nodes[i].z))
			editor.set_block(block_definitions::LIGHT_BLUE_CONCRETE, x, 1, z,
					std::nullopt, std::nullopt);
}
}

void generate_outline(WorldEditor &editor, const ProcessedWay &way)
{
	if (!underground_building(way.tags))
		draw_ring(editor, way.nodes);
}

void generate_relation_outline(WorldEditor &editor, const ProcessedRelation &relation)
{
	if (underground_building(relation.tags))
		return;
	for (const auto &member : relation.members)
		if (member.role != ProcessedMemberRole::Part && member.way)
			draw_ring(editor, member.way->nodes);
}
}
