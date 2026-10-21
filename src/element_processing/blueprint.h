#pragma once

#include "../../../arnis_adapter.h"

namespace arnis::blueprint
{
void generate_outline(WorldEditor &editor, const ProcessedWay &way);
void generate_relation_outline(WorldEditor &editor, const ProcessedRelation &relation);
}
