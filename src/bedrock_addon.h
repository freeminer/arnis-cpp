#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace arnis::highways::cars
{
class CarNetwork;
}
namespace arnis::trains
{
class TrainRouteNetwork;
}

namespace arnis::bedrock_addon
{

enum class Fleet : std::uint8_t
{
	Car,
	Train
};

constexpr std::string_view family(Fleet fleet) noexcept
{
	switch (fleet) {
	case Fleet::Car:
		return "arnis_car";
	case Fleet::Train:
		return "arnis_train";
	}
	return "arnis_car";
}

enum class Lamp : std::uint8_t
{
	Off,
	On,
	Brake
};

struct Shape
{
	std::string block;
	std::array<float, 3> size{};
	std::array<float, 3> corner{};
	float tilt{};
	Lamp lamp{Lamp::Off};
};

struct Body
{
	std::string name;
	Fleet fleet{Fleet::Car};
	std::vector<Shape> shapes;
};

struct Pack
{
	std::string id;
	std::string folder;
	std::vector<std::pair<std::string, std::vector<std::uint8_t>>> files;
};

struct Addon
{
	Pack behavior;
	Pack resources;
};

// Backend-neutral vehicle data and geometry. The host can package it into a
// Bedrock behavior/resource pack or consume the entity descriptions itself.
struct VehiclePayload
{
	nlohmann::json data;
	std::vector<Body> bodies;
};

std::optional<VehiclePayload> vehicles(const ::arnis::highways::cars::CarNetwork *cars,
		const ::arnis::trains::TrainRouteNetwork *trains,
		const std::function<std::optional<int>(std::uint64_t, int, int)> &rail_y);

// The behavior-pack script keeps large numeric lists compact until a vehicle
// needs them, matching the Rust add-on's wire representation.
template <typename Range>
std::string list(const Range &values)
{
	std::ostringstream out;
	bool first = true;
	for (const auto &value : values) {
		if (!first)
			out << ',';
		first = false;
		out << value;
	}
	return out.str();
}

} // namespace arnis::bedrock_addon
