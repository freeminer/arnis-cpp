#include "operator.h"
#include "rotate/rotator.h"
#include "translate/translator_factory.h"
#include <stdexcept>

namespace arnis::map_transformation
{
std::unique_ptr<Operator> operator_from_json(const nlohmann::json &entry)
{
	if (!entry.is_object() || !entry.contains("operation") ||
			!entry.at("operation").is_string())
		throw std::runtime_error("Expected string field 'operation' in an operator dict");
	if (!entry.contains("config") || !entry.at("config").is_object())
		throw std::runtime_error("Expected object field 'config' in an operator dict");
	const auto operation = entry.at("operation").get<std::string>();
	const auto &config = entry.at("config");
	if (operation == "rotate") {
		if (!config.contains("angle_degrees") || !config.at("angle_degrees").is_number())
			throw std::runtime_error("Rotator config requires numeric 'angle_degrees'");
		auto out = std::make_unique<rotate::Rotator>();
		out->angle_degrees = config.at("angle_degrees").get<double>();
		return out;
	}
	if (operation != "translate")
		throw std::runtime_error("Unrecognized operation type '" + operation + "'");
	return translator_from_json(config);
}

std::vector<std::unique_ptr<Operator>> operator_vec_from_json(const nlohmann::json &list)
{
	if (!list.is_array())
		throw std::runtime_error("Expected a list of operator dicts");
	std::vector<std::unique_ptr<Operator>> out;
	out.reserve(list.size());
	for (std::size_t i = 0; i < list.size(); ++i)
		try {
			out.push_back(operator_from_json(list[i]));
		} catch (const std::exception &error) {
			throw std::runtime_error("Operator dict at index " + std::to_string(i) +
									 " format error:\n" + error.what());
		}
	return out;
}

void operate_all(const std::vector<std::unique_ptr<Operator>> &operators,
		std::vector<ProcessedElement> &elements, cartesian::XZBBox &bbox, Ground &ground)
{
	for (const auto &op : operators)
		if (op)
			op->operate(elements, bbox, ground);
}
}
