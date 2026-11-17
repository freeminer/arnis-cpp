#include "plane_bodies.h"

#include <array>

namespace arnis::element_processing::plane_bodies
{
namespace
{
using train_bodies::Livery;

constexpr std::string_view WHITE = "minecraft:white_concrete";
constexpr std::string_view LIGHT = "minecraft:light_gray_concrete";
constexpr std::string_view GREY = "minecraft:gray_concrete";
constexpr std::string_view BLACK = "minecraft:black_concrete";
constexpr std::string_view RED = "minecraft:red_concrete";
constexpr std::string_view ORANGE = "minecraft:orange_concrete";
constexpr std::string_view YELLOW = "minecraft:yellow_concrete";
constexpr std::string_view GREEN = "minecraft:green_concrete";
constexpr std::string_view CYAN = "minecraft:cyan_concrete";
constexpr std::string_view SKY = "minecraft:light_blue_concrete";
constexpr std::string_view BLUE = "minecraft:blue_concrete";
constexpr std::string_view PURPLE = "minecraft:purple_concrete";

Livery livery(std::string_view id, std::string_view body, std::string_view lower,
		std::string_view stripe, std::string_view door, std::string_view roof)
{
	return {std::string(body), std::string(lower), std::string(stripe), std::string(door),
			std::string(roof), std::string(id), 0};
}

const std::array<Livery, 15> LIVERIES{livery("wb", WHITE, LIGHT, WHITE, BLUE, WHITE),
		livery("wr", WHITE, WHITE, WHITE, RED, RED),
		livery("wy", WHITE, BLUE, YELLOW, YELLOW, BLUE),
		livery("wo", WHITE, ORANGE, WHITE, ORANGE, ORANGE),
		livery("wg", WHITE, LIGHT, GREEN, GREEN, WHITE),
		livery("ws", WHITE, SKY, WHITE, SKY, WHITE),
		livery("wp", WHITE, PURPLE, WHITE, PURPLE, PURPLE),
		livery("wc", WHITE, LIGHT, CYAN, CYAN, CYAN),
		livery("jw", WHITE, WHITE, GREY, WHITE, WHITE),
		livery("jk", WHITE, BLACK, RED, BLACK, BLACK),
		livery("jb", WHITE, BLUE, LIGHT, BLUE, WHITE),
		livery("lr", WHITE, WHITE, RED, RED, WHITE),
		livery("lb", WHITE, WHITE, BLUE, BLUE, WHITE),
		livery("ly", YELLOW, YELLOW, BLACK, YELLOW, YELLOW),
		livery("lg", WHITE, GREEN, GREEN, GREEN, WHITE)};

constexpr std::array<float, 1> LIGHT_SEATS{0.44f};
constexpr std::array<float, 2> BIZJET_SEATS{2.5f, 0.9f};
constexpr std::array<float, 4> REGIONAL_SEATS{8.0f, 4.25f, -0.25f, -4.0f};
constexpr std::array<float, 5> NARROW_SEATS{12.0f, 7.0f, 1.0f, -5.0f, -11.0f};
constexpr std::array<float, 6> WIDE_SEATS{24.0f, 16.0f, 8.0f, 0.0f, -8.0f, -16.0f};
} // namespace

std::string_view name(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return "light";
	case Airframe::Bizjet:
		return "bizjet";
	case Airframe::Regional:
		return "regional";
	case Airframe::Narrow:
		return "narrow";
	case Airframe::Wide:
		return "wide";
	}
	return "light";
}

std::string_view model(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return "plane_light";
	case Airframe::Bizjet:
		return "plane_bizjet";
	case Airframe::Regional:
		return "plane_regional";
	case Airframe::Narrow:
		return "plane_narrow";
	case Airframe::Wide:
		return "plane_wide";
	}
	return "plane_light";
}

std::pair<double, double> size_m(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {8.3, 11.0};
	case Airframe::Bizjet:
		return {15.6, 16.3};
	case Airframe::Regional:
		return {27.2, 27.05};
	case Airframe::Narrow:
		return {37.6, 35.8};
	case Airframe::Wide:
		return {65.0, 63.0};
	}
	return {8.3, 11.0};
}

std::pair<double, double> reach_m(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {2.9, 5.4};
	case Airframe::Bizjet:
		return {8.2, 7.4};
	case Airframe::Regional:
		return {13.2, 14.0};
	case Airframe::Narrow:
		return {17.7, 19.9};
	case Airframe::Wide:
		return {33.5, 31.5};
	}
	return {2.9, 5.4};
}

bool retracts(Airframe frame)
{
	return frame != Airframe::Light;
}

std::pair<double, bool> runway_m(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {320.0, false};
	case Airframe::Bizjet:
		return {900.0, true};
	case Airframe::Regional:
		return {1100.0, true};
	case Airframe::Narrow:
		return {1700.0, true};
	case Airframe::Wide:
		return {2400.0, true};
	}
	return {320.0, false};
}

std::pair<double, double> speeds_mps(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {28.0, 28.0};
	case Airframe::Bizjet:
		return {56.0, 54.0};
	case Airframe::Regional:
		return {58.0, 56.0};
	case Airframe::Narrow:
		return {72.0, 68.0};
	case Airframe::Wide:
		return {79.0, 72.0};
	}
	return {28.0, 28.0};
}

std::pair<double, double> accel_mps2(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {1.8, 1.6};
	case Airframe::Bizjet:
		return {2.8, 2.2};
	case Airframe::Regional:
		return {2.3, 2.0};
	case Airframe::Narrow:
		return {2.2, 1.9};
	case Airframe::Wide:
		return {1.9, 1.7};
	}
	return {1.8, 1.6};
}

std::array<double, 3> angles_degrees(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {6.0, 9.0, 5.0};
	case Airframe::Bizjet:
		return {12.0, 15.0, 5.0};
	case Airframe::Regional:
		return {9.0, 12.0, 4.0};
	case Airframe::Narrow:
		return {10.0, 14.0, 5.0};
	case Airframe::Wide:
		return {8.0, 12.0, 5.0};
	}
	return {6.0, 9.0, 5.0};
}

bool slow(Airframe frame)
{
	return frame == Airframe::Light;
}

SeatLayout seats(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return {LIGHT_SEATS, 1.72};
	case Airframe::Bizjet:
		return {BIZJET_SEATS, 1.75};
	case Airframe::Regional:
		return {REGIONAL_SEATS, 2.72};
	case Airframe::Narrow:
		return {NARROW_SEATS, 4.5};
	case Airframe::Wide:
		return {WIDE_SEATS, 5.81};
	}
	return {LIGHT_SEATS, 1.72};
}

double cabin_m(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return 1.6;
	case Airframe::Bizjet:
		return 2.0;
	case Airframe::Regional:
		return 3.0;
	case Airframe::Narrow:
		return 4.0;
	case Airframe::Wide:
		return 6.0;
	}
	return 1.6;
}

std::span<const train_bodies::Livery> liveries(Airframe frame)
{
	switch (frame) {
	case Airframe::Light:
		return std::span(LIVERIES).subspan(11, 4);
	case Airframe::Bizjet:
		return std::span(LIVERIES).subspan(8, 3);
	case Airframe::Regional:
		return std::span(LIVERIES).subspan(4, 4);
	case Airframe::Narrow:
		return std::span(LIVERIES).first(6);
	case Airframe::Wide:
		return std::span(LIVERIES).first(3);
	}
	return std::span(LIVERIES).subspan(11, 4);
}

} // namespace arnis::element_processing::plane_bodies
