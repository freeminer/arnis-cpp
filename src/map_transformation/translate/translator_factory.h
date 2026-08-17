#pragma once
#include "../operator.h"
#include <memory>
#include <string>
namespace arnis::map_transformation
{
std::unique_ptr<Operator> translator_from_json(
		const std::string &type, const std::string &config);
}
