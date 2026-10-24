#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace arnis::trains
{
struct Coach;
struct Livery;
enum class TrainKind : std::uint8_t;
} // namespace arnis::trains

namespace arnis::train_bodies
{
using Kind = trains::TrainKind;

/// One axis-aligned carriage component in a coordinate frame centred on the rail.
/// The frame's +Z axis points towards the front of the carriage.
struct Part
{
	std::string block;
	std::array<float, 3> size{};
	std::array<float, 3> corner{};
	float tilt{};
	bool glow{};
};

double length_m(const trains::Coach &coach);
std::vector<Part> parts(const trains::Coach &coach, const trains::Livery &livery);
std::string key(const trains::Coach &coach, const trains::Livery &livery);
} // namespace arnis::train_bodies
