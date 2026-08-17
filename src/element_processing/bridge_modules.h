#pragma once
#include <cstddef>
#include <optional>
namespace arnis::bridge_modules
{
constexpr std::size_t MIN_MODULE_BRIDGE_LEN = 12;
std::optional<std::size_t> pick_module_index(int, std::size_t);
int module_half_width(std::size_t);
bool module_has_pillars(std::size_t);
}
