#pragma once
#include "../../../../arnis_adapter.h"
#include "../../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include "../../coordinate_system/cartesian/xzvector.h"
namespace arnis::map_transformation
{
void translate_by_vector(
		cartesian::XZVector, std::vector<ProcessedElement> &, cartesian::XZBBox &);
}
