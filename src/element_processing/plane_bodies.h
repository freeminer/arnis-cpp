#pragma once

#include "train_bodies.h"

#include <array>
#include <span>
#include <string_view>
#include <utility>

namespace arnis::element_processing::plane_bodies
{
/// Aircraft sizes and performance, matching Rust's `plane_bodies::Airframe`.
enum class Airframe
{
	Light,
	Bizjet,
	Regional,
	Narrow,
	Wide
};

inline constexpr std::array<Airframe, 5> ALL{Airframe::Light, Airframe::Bizjet,
		Airframe::Regional, Airframe::Narrow, Airframe::Wide};

struct SeatLayout
{
	std::span<const float> positions_m;
	double eye_height_m = 0.0;
};

std::string_view name(Airframe frame);
std::string_view model(Airframe frame);
std::pair<double, double> size_m(Airframe frame);
std::pair<double, double> reach_m(Airframe frame);
bool retracts(Airframe frame);
std::pair<double, bool> runway_m(Airframe frame);
std::pair<double, double> speeds_mps(Airframe frame);
std::pair<double, double> accel_mps2(Airframe frame);
/// Climb angle, climb attitude, and touchdown attitude in degrees.
std::array<double, 3> angles_degrees(Airframe frame);
bool slow(Airframe frame);
SeatLayout seats(Airframe frame);
double cabin_m(Airframe frame);
std::span<const train_bodies::Livery> liveries(Airframe frame);

} // namespace arnis::element_processing::plane_bodies
