#include "blueprint.h"

#include "../block_definitions.h"
#include "../bresenham.h"
#include "buildings.h"

namespace arnis::blueprint
{
namespace
{
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
	if (!buildings::is_underground_building(way.tags))
		draw_ring(editor, way.nodes);
}

void generate_relation_outline(WorldEditor &editor, const ProcessedRelation &relation)
{
	if (buildings::is_underground_building(relation.tags))
		return;
	for (const auto &member : relation.members)
		if (member.role != ProcessedMemberRole::Part && member.way)
			draw_ring(editor, member.way->nodes);
}
}
