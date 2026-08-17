#pragma once
#include "../../coordinate_system/cartesian/xzpoint.h"
#include "../../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include <string>
#include "../operator.h"
namespace arnis::map_transformation
{
struct StartEndTranslator : Operator
{
	cartesian::XZPoint start, end;
	cartesian::XZVector displacement() const { return end - start; }
	std::string repr() const
	{
		return "translate " + start.to_string() + " to " + end.to_string();
	}
	void operate(std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox,
			Ground &) override;
};
}
