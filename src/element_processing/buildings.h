#pragma once
#include <optional>
#include "../../../arnis_adapter.h"

namespace arnis
{

namespace buildings
{

void generate_building_from_relation(
		WorldEditor &editor, const ProcessedRelation &relation, const Args &args);
void generate_buildings(WorldEditor *editor, const ProcessedWay &element,
		const Args &args, const std::optional<int> &relation_levels);
}
}