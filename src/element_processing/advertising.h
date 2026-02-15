#pragma once

#include "../../../arnis_adapter.h"

namespace arnis {
namespace advertising {
void generate_advertising(WorldEditor &editor, const ProcessedNode &node);
void generate_advertising_column(WorldEditor &editor, const ProcessedNode &node);
void generate_advertising_flag(WorldEditor &editor, const ProcessedNode &node);
void generate_poster_box(WorldEditor &editor, const ProcessedNode &node);

}
}