#pragma once
#include "../../coordinate_system/cartesian/xzvector.h"
#include <string>
#include "../operator.h"
#include "translator.h"
namespace arnis::map_transformation
{
struct VectorTranslator : Operator
{
	cartesian::XZVector vector;
	std::string repr() const { return "translate diaplacement " + vector.to_string(); }
	void operate(std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox,
			Ground &) override
	{
		translate_by_vector(vector, elements, bbox);
	}
};
}
