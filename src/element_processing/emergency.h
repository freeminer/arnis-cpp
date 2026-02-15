#pragma once

#include "../../../arnis_adapter.h"

namespace arnis {
namespace emergency {
void generate_emergency(WorldEditor &editor, const ProcessedNode &node);
void generate_fire_hydrant(WorldEditor &editor, const ProcessedNode &node);

}
}