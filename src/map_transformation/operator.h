#pragma once
#include "../../arnis_adapter.h"
#include "../coordinate_system/cartesian/xzbbox/xzbbox_enum.h"
#include <string>
#include <vector>
#include <memory>
#include <nlohmann/json.hpp>
namespace arnis::map_transformation
{
class Operator
{
public:
	virtual ~Operator() = default;
	virtual void operate(
			std::vector<ProcessedElement> &, cartesian::XZBBox &, Ground &) = 0;
	virtual std::string repr() const = 0;
};

std::unique_ptr<Operator> operator_from_json(const nlohmann::json &config);
std::vector<std::unique_ptr<Operator>> operator_vec_from_json(const nlohmann::json &list);
void operate_all(const std::vector<std::unique_ptr<Operator>> &,
		std::vector<ProcessedElement> &, cartesian::XZBBox &, Ground &);
}
