#include "operator.h"
#include "rotate/rotator.h"
#include "translate/startend_translator.h"
#include "translate/vector_translator.h"
#include <stdexcept>

namespace arnis::map_transformation
{
namespace
{
int integer_field(const nlohmann::json &object, const char *key)
{
	if (!object.contains(key) || !object.at(key).is_number_integer())
		throw std::runtime_error(std::string("Expected integer field '") + key + "'");
	return object.at(key).get<int>();
}
cartesian::XZPoint point_field(const nlohmann::json &object, const char *key)
{
	if (!object.contains(key) || !object.at(key).is_object())
		throw std::runtime_error(std::string("Expected object field '") + key + "'");
	const auto &point = object.at(key);
	return {integer_field(point, "x"), integer_field(point, "z")};
}
}

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
	if (!config.contains("type") || !config.at("type").is_string() ||
			!config.contains("config") || !config.at("config").is_object())
		throw std::runtime_error(
				"Translator config requires string 'type' and object 'config'");
	const auto type = config.at("type").get<std::string>();
	const auto &translate = config.at("config");
	if (type == "vector") {
		const auto vector =
				translate.contains("vector") ? translate.at("vector") : translate;
		auto out = std::make_unique<VectorTranslator>();
		out->vector = {integer_field(vector, "dx"), integer_field(vector, "dz")};
		return out;
	}
	if (type == "startend") {
		auto out = std::make_unique<StartEndTranslator>();
		out->start = point_field(translate, "start");
		out->end = point_field(translate, "end");
		return out;
	}
	throw std::runtime_error("Unrecognized translator type '" + type + "'");
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
