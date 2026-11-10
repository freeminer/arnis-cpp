#pragma once

#include <array>
#include <cstdint>
#include <string_view>
#include <utility>
#include <vector>

namespace arnis::train_bodies
{
struct Livery;
struct Part;
}

namespace arnis::ship_bodies
{
/// Vessel archetypes used by mapped ferries, navigable waterways and leisure routes.
enum class Vessel : std::uint8_t
{
	RiverFerry,
	WaterBus,
	CarFerry,
	SeaFerry,
	Barge,
	ContainerBarge,
	TankBarge,
	RiverCruise,
	Peniche,
	TourBoat,
	Sailboat,
	Motorboat
};

std::string_view name(Vessel vessel);
std::pair<double, double> size_m(Vessel vessel);
bool lowers(Vessel vessel);
double cruise_kmh(Vessel vessel);
std::array<double, 3> acceleration(Vessel vessel);
std::pair<double, double> dwell_seconds(Vessel vessel);
std::pair<std::vector<float>, double> seats_m(Vessel vessel);
std::vector<train_bodies::Part> parts(
		Vessel vessel, const train_bodies::Livery &livery, bool low = false);
double air_draft_m(Vessel vessel, bool low = false);
std::vector<train_bodies::Livery> liveries(Vessel vessel);
} // namespace arnis::ship_bodies
