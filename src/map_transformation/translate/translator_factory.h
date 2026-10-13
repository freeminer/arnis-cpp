#pragma once
#include "../operator.h"
#include <memory>
namespace arnis::map_transformation
{
// Parses the Rust-compatible {"type": ..., "config": ...} translator object.
// Throws std::runtime_error with a field-specific message when it is invalid.
std::unique_ptr<Operator> translator_from_json(const nlohmann::json &config);
}
