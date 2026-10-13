#include "translator_factory.h"
#include "vector_translator.h"
#include "startend_translator.h"
#include <cstdint>
#include <limits>
#include <stdexcept>
namespace arnis::map_transformation
{
namespace
{
int integer_field(const nlohmann::json &object, const char *key)
{
	if (!object.is_object() || !object.contains(key) ||
			!object.at(key).is_number_integer())
		throw std::runtime_error(std::string("Expected integer field '") + key + "'");
	const auto &value = object.at(key);
	if (value.is_number_unsigned()) {
		const auto number = value.get<std::uint64_t>();
		if (number > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
			throw std::runtime_error(
					std::string("Integer field '") + key + "' is outside the i32 range");
		return static_cast<int>(number);
	}
	const auto number = value.get<std::int64_t>();
	if (number < std::numeric_limits<int>::min() ||
			number > std::numeric_limits<int>::max())
		throw std::runtime_error(
				std::string("Integer field '") + key + "' is outside the i32 range");
	return static_cast<int>(number);
}

cartesian::XZPoint point_field(const nlohmann::json &object, const char *key)
{
	if (!object.is_object() || !object.contains(key) || !object.at(key).is_object())
		throw std::runtime_error(std::string("Expected object field '") + key + "'");
	const auto &point = object.at(key);
	return {integer_field(point, "x"), integer_field(point, "z")};
}
}

std::unique_ptr<Operator> translator_from_json(const nlohmann::json &config)
{
	if (!config.is_object() || !config.contains("type") || !config.at("type").is_string())
		throw std::runtime_error("Expected string field 'type' in a translator dict");
	if (!config.contains("config") || !config.at("config").is_object())
		throw std::runtime_error("Expected object field 'config' in a translator dict");
	const auto type = config.at("type").get<std::string>();
	const auto &values = config.at("config");
	try {
		if (type == "vector") {
			if (!values.contains("vector") || !values.at("vector").is_object())
				throw std::runtime_error("Expected object field 'vector'");
			const auto &vector = values.at("vector");
			auto result = std::make_unique<VectorTranslator>();
			result->vector = {integer_field(vector, "dx"), integer_field(vector, "dz")};
			return result;
		}
		if (type == "startend") {
			auto result = std::make_unique<StartEndTranslator>();
			result->start = point_field(values, "start");
			result->end = point_field(values, "end");
			return result;
		}
		throw std::runtime_error("Unrecognized translator type '" + type + "'");
	} catch (const std::exception &error) {
		throw std::runtime_error(
				"Translator config format error:\n" + std::string(error.what()));
	}
}
}
