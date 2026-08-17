#pragma once
#include "operator.h"
namespace arnis::map_transformation
{
void transform_map(std::vector<ProcessedElement> &, cartesian::XZBBox &, Ground &,
		const std::vector<Operator *> &);
}
