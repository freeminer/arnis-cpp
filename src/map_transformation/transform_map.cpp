#include "transform_map.h"
namespace arnis::map_transformation
{
void transform_map(std::vector<ProcessedElement> &e, cartesian::XZBBox &b, Ground &g,
		const std::vector<Operator *> &ops)
{
	for (auto *op : ops)
		if (op)
			op->operate(e, b, g);
}
}
