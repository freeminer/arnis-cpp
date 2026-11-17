#pragma once

#include <cstdint>

namespace arnis::element_processing::plane_bodies
{
enum class Airframe : std::uint8_t
{
	Light,
	Bizjet,
	Regional,
	Narrow,
	Wide
};
} // namespace arnis::element_processing::plane_bodies
