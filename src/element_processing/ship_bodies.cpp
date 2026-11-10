#include "ship_bodies.h"

#include "train_bodies.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace arnis::ship_bodies
{
namespace
{
using train_bodies::Livery;
using train_bodies::Part;
using namespace std::literals;
constexpr std::string_view GLASS = "minecraft:tinted_glass";
constexpr std::string_view WHITE = "minecraft:white_concrete";
constexpr std::string_view BLACK = "minecraft:black_concrete";
constexpr std::string_view GREY = "minecraft:gray_concrete";
constexpr std::string_view LIGHT = "minecraft:light_gray_concrete";
constexpr std::string_view RED = "minecraft:red_concrete";
constexpr std::string_view BLUE = "minecraft:blue_concrete";
constexpr std::string_view GREEN = "minecraft:green_concrete";
constexpr std::string_view YELLOW = "minecraft:yellow_concrete";
constexpr std::string_view ORANGE = "minecraft:orange_concrete";
constexpr std::string_view CYAN = "minecraft:cyan_concrete";
constexpr std::string_view BROWN = "minecraft:brown_terracotta";
constexpr std::string_view TEAK = "minecraft:light_gray_terracotta";
constexpr std::string_view RUST = "minecraft:red_terracotta";

Part box(std::string_view block, std::pair<float, float> x, std::pair<float, float> y,
		std::pair<float, float> z)
{
	return {std::string(block),
			{x.second - x.first, y.second - y.first, z.second - z.first},
			{x.first, y.first, z.first}, 0.0f, false};
}

std::vector<Part> hull(
		float len, float beam, float down, float top, std::string_view block, bool both)
{
	const float half = len / 2.0f, hw = beam / 2.0f;
	const float taper = std::min(beam * 0.9f, len * 0.2f);
	std::vector<Part> out;
	const float aft = both ? -half + taper : -half;
	out.push_back(box(block, {-hw, hw}, {-down, top}, {aft, half - taper}));
	constexpr std::array<std::pair<float, float>, 3> steps{
			{{0.4f, 0.82f}, {0.75f, 0.55f}, {1.0f, 0.28f}}};
	float from = 0.0f;
	for (const auto [to, width] : steps) {
		const float z0 = half - taper * (1.0f - from);
		const float z1 = half - taper * (1.0f - to);
		const float w = hw * width;
		const float y0 = -down * (1.0f - to * 0.6f), y1 = top + 0.25f * to;
		out.push_back(box(block, {-w, w}, {y0, y1}, {z0, z1}));
		if (both)
			out.push_back(box(block, {-w, w}, {y0, y1}, {-z1, -z0}));
		from = to;
	}
	return out;
}

Part band(float len, float beam, std::pair<float, float> y, std::string_view block,
		bool both)
{
	const float taper = std::min(beam * 0.9f, len * 0.2f);
	const float aft = both ? -len / 2.0f + taper : -len / 2.0f;
	return box(block, {-beam / 2.0f - 0.04f, beam / 2.0f + 0.04f}, y,
			{aft, len / 2.0f - taper});
}

std::vector<Part> car(float x, float z, std::string_view block)
{
	return {box(block, {x - .9f, x + .9f}, {.85f, 1.55f}, {z - 2.2f, z + 2.2f}),
			box(GLASS, {x - .8f, x + .8f}, {1.55f, 2.05f}, {z - 1.2f, z + 1.0f}),
			box(BLACK, {x - .95f, x + .95f}, {.7f, 1.1f}, {z - 1.5f, z - .9f}),
			box(BLACK, {x - .95f, x + .95f}, {.7f, 1.1f}, {z + .9f, z + 1.5f})};
}

std::vector<Part> build(Vessel v, float len, float beam, const Livery &l, bool low)
{
	const float half = len / 2.0f, hw = beam / 2.0f;
	std::vector<Part> out;
	switch (v) {
	case Vessel::RiverFerry: {
		out = {box(l.lower, {-hw, hw}, {-.8f, .6f}, {-half + 1.2f, half - 1.2f}),
				box(l.stripe, {-hw - .04f, hw + .04f}, {.3f, .6f},
						{-half + 1.2f, half - 1.2f}),
				box(GREY, {-hw + .3f, hw - .3f}, {.6f, .7f},
						{-half + 1.2f, half - 1.2f})};
		for (float end : {-1.0f, 1.0f}) {
			float a = end * (half - 1.2f), b = end * half;
			out.push_back(box(GREY, {-hw + .8f, hw - .8f}, {.3f, .55f},
					{std::min(a, b), std::max(a, b)}));
		}
		for (float side : {-1.0f, 1.0f}) {
			const float x = side * (hw - .1f);
			out.push_back(
					box(WHITE, {x - .06f, x + .06f}, {.7f, 1.7f}, {-half + 2, half - 2}));
		}
		out.push_back(box(l.body, {hw - 2.2f, hw - .2f}, {.7f, 3.0f}, {-1.4f, 1.4f}));
		out.push_back(box(GLASS, {hw - 2.25f, hw - .15f}, {1.9f, 2.7f}, {-1.3f, 1.3f}));
		out.push_back(box(l.roof, {hw - 2.4f, hw}, {3.0f, 3.2f}, {-1.6f, 1.6f}));
		auto a = car(-hw + 2.2f, -4.5f, BLUE), b = car(-hw + 2.2f, 2.0f, WHITE);
		out.insert(out.end(), a.begin(), a.end());
		out.insert(out.end(), b.begin(), b.end());
		break;
	}
	case Vessel::WaterBus: {
		out = hull(len, beam, 1.2f, 1.0f, l.lower, true);
		out.push_back(band(len, beam, {.65f, 1.0f}, l.stripe, true));
		const float cabin = half * .62f;
		out.insert(out.end(),
				{box(l.door, {-hw + .25f, hw - .25f}, {1.0f, 1.1f},
						 {-half + 2, half - 2}),
						box(l.body, {-hw + .45f, hw - .45f}, {1.1f, 3.2f},
								{-cabin, cabin}),
						box(GLASS, {-hw + .4f, hw - .4f}, {1.9f, 2.8f},
								{-cabin + .4f, cabin - .4f}),
						box(l.roof, {-hw + .35f, hw - .35f}, {3.2f, 3.4f},
								{-cabin - .3f, cabin + .3f}),
						box(l.body, {-1.3f, 1.3f}, {3.4f, 4.9f}, {-1.3f, 1.3f}),
						box(GLASS, {-1.35f, 1.35f}, {4.0f, 4.7f}, {-1.2f, 1.2f}),
						box(l.roof, {-1.5f, 1.5f}, {4.9f, 5.1f}, {-1.5f, 1.5f})});
		for (float side : {-1.0f, 1.0f}) {
			const float x = side * (hw - .3f);
			out.push_back(
					box(WHITE, {x - .05f, x + .05f}, {3.4f, 4.3f}, {-cabin, cabin}));
		}
		break;
	}
	case Vessel::CarFerry:
	case Vessel::SeaFerry: {
		const bool sea = v == Vessel::SeaFerry;
		const float deck = sea ? 4.0f : 2.0f, upper = sea ? 8.5f : 8.5f;
		out = hull(len, beam, sea ? 5.5f : 2.5f, deck, l.lower, true);
		out.push_back(band(len, beam, {deck - .45f, deck}, l.stripe, true));
		if (!sea) {
			out.push_back(box(GREY, {-hw + 1.4f, hw - 1.4f}, {deck, deck + .1f},
					{-half + 2, half - 2}));
			const float side = half * .62f;
			for (float s : {-1.0f, 1.0f}) {
				const float a = s * (hw - 1.6f), b = s * hw;
				const std::pair<float, float> x{std::min(a, b), std::max(a, b)};
				out.push_back(box(l.body, x, {deck, upper}, {-side, side}));
				out.push_back(box(GLASS, {x.first - .04f, x.second + .04f},
						{deck + 1.4f, deck + 2.4f}, {-side + 1, side - 1}));
				out.push_back(box(GLASS, {x.first - .04f, x.second + .04f},
						{deck + 4, deck + 5}, {-side + 1, side - 1}));
			}
			out.push_back(box(
					l.body, {-hw, hw}, {upper, upper + .6f}, {-side * .7f, side * .7f}));
			out.push_back(box(
					l.body, {-hw * .6f, hw * .6f}, {upper + .6f, upper + 3.2f}, {-4, 4}));
			out.push_back(box(GLASS, {-hw * .6f - .05f, hw * .6f + .05f},
					{upper + 1.6f, upper + 2.8f}, {-3.8f, 3.8f}));
			out.push_back(box(l.roof, {-hw * .6f - .2f, hw * .6f + .2f},
					{upper + 3.2f, upper + 3.5f}, {-4.2f, 4.2f}));
			for (float x : {-2.6f, 2.6f})
				for (float z = -half + 9; z < half - 9; z += 11) {
					auto c = car(x, z, BLUE);
					for (auto &p : c)
						p.corner[1] += deck - .6f;
					out.insert(out.end(), c.begin(), c.end());
				}
		} else {
			const float body = half * .78f;
			out.push_back(box(l.body, {-hw, hw}, {4, 14}, {-body, body}));
			out.push_back(box(l.body, {-hw + 1.5f, hw - 1.5f}, {14, 17},
					{-body * .6f, body * .6f}));
			out.push_back(box(l.roof, {-hw + 1.2f, hw - 1.2f}, {17, 17.3f},
					{-body * .62f, body * .62f}));
			out.push_back(box(l.body, {-hw * .7f, hw * .7f}, {17.3f, 20.5f}, {-5, 5}));
			out.push_back(box(GLASS, {-hw * .7f - .05f, hw * .7f + .05f}, {18.4f, 19.8f},
					{-4.8f, 4.8f}));
			for (auto [a, b] : std::array<std::pair<float, float>, 4>{
						 {{5.5f, 6.8f}, {8, 9.3f}, {10.5f, 11.8f}, {14.8f, 16}}}) {
				const float w = a > 14 ? hw - 1.5f : hw,
							reach = a > 14 ? body * .6f : body;
				out.push_back(box(GLASS, {-w - .05f, w + .05f}, {a, b},
						{-reach + 1.5f, reach - 1.5f}));
			}
		}
		break;
	}
	case Vessel::Barge:
	case Vessel::ContainerBarge:
	case Vessel::TankBarge: {
		out = hull(len, beam, 2.8f, 1.2f, l.lower, false);
		out.push_back(band(len, beam, {.9f, 1.2f}, l.stripe, false));
		const float hf = -half + 13, ht = half - 9;
		if (v == Vessel::ContainerBarge) {
			const std::array<std::string_view, 8> colors{
					RED, BLUE, ORANGE, GREEN, LIGHT, BROWN, CYAN, WHITE};
			const int tiers = low ? 1 : 2;
			const float bay = (ht - hf) / 7;
			for (int tier = 0; tier < tiers; tier++)
				for (int row = 0; row < 2; row++)
					for (int b = 0; b < 7; b++) {
						if (tier == 1 && (b + row) % 3 == 0)
							continue;
						const float x0 = row == 0 ? -hw + .7f : .1f,
									x1 = row == 0 ? -.1f : hw - .7f, z = hf + bay * b;
						out.push_back(box(
								colors[(b * 3 + row * 5 + tier * 2) % colors.size()],
								{x0, x1}, {1.8f + tier * 2.6f, 4.35f + tier * 2.6f},
								{z + .15f, z + bay - .15f}));
					}
		} else if (v == Vessel::TankBarge) {
			out.push_back(box(RUST, {-hw + .3f, hw - .3f}, {1.2f, 1.35f}, {hf, ht}));
			out.push_back(box(LIGHT, {-.3f, .3f}, {1.35f, 1.85f}, {hf + 2, ht - 2}));
			for (float z = hf + 6; z < ht - 4; z += 14)
				out.push_back(box(GREY, {-.9f, .9f}, {1.35f, 2.3f}, {z - .9f, z + .9f}));
		} else {
			out.push_back(box(l.lower, {-hw + .5f, hw - .5f}, {1.2f, 2.3f}, {hf, ht}));
			const float lid = (ht - hf) / 8;
			for (int k = 0; k < 8; k++) {
				float z = hf + lid * k;
				out.push_back(box(l.door, {-hw + .6f, hw - .6f}, {2.3f, 2.6f},
						{z + .08f, z + lid - .08f}));
			}
		}
		out.push_back(
				box(l.body, {-hw + .4f, hw - .4f}, {1.2f, 4}, {-half + 1, -half + 11}));
		out.push_back(box(GLASS, {-hw + .35f, hw - .35f}, {2.2f, 3.2f},
				{-half + 1.6f, -half + 10.4f}));
		const float wheel = low ? 4.2f : 5.6f;
		out.push_back(box(l.body, {-2.4f, 2.4f}, {wheel, wheel + 1.8f},
				{-half + 7, -half + 10.6f}));
		out.push_back(box(GLASS, {-2.45f, 2.45f}, {wheel + .6f, wheel + 1.6f},
				{-half + 7.2f, -half + 10.65f}));
		out.push_back(box(l.roof, {-2.6f, 2.6f}, {wheel + 1.8f, wheel + 2},
				{-half + 6.8f, -half + 10.8f}));
		if (!low)
			out.push_back(
					box(GREY, {-.25f, .25f}, {4.2f, wheel}, {-half + 8.5f, -half + 9}));
		out.push_back(
				box(l.roof, {-.15f, .15f}, {1.2f, 3.2f}, {half - 3.5f, half - 3.2f}));
		break;
	}
	case Vessel::RiverCruise:
	case Vessel::Peniche:
	case Vessel::TourBoat:
	case Vessel::Sailboat:
	case Vessel::Motorboat: {
		const bool sail = v == Vessel::Sailboat, motor = v == Vessel::Motorboat;
		out = hull(len, beam,
				sail	? .5f
				: motor ? .45f
						: (v == Vessel::Peniche ? 1.8f : 1.4f),
				sail	? .8f
				: motor ? .75f
						: 1.0f,
				l.lower, false);
		out.push_back(band(len, beam, sail ? std::pair{.55f, .8f} : std::pair{.65f, 1.0f},
				l.stripe, false));
		if (v == Vessel::RiverCruise) {
			const float from = -half + 3, to = half - 12;
			out.push_back(box(l.body, {-hw + .2f, hw - .2f}, {1, 7.6f}, {from, to}));
			for (auto [a, b] : std::array<std::pair<float, float>, 3>{
						 {{1.7f, 2.7f}, {3.9f, 5}, {6.1f, 7.2f}}})
				out.push_back(
						box(GLASS, {-hw + .15f, hw - .15f}, {a, b}, {from + 1, to - 1}));
			out.push_back(box(l.door, {-hw + .3f, hw - .3f}, {7.6f, 7.75f}, {from, to}));
			const float wh = low ? 7.75f : 8.6f;
			out.push_back(
					box(l.body, {-hw + 1, hw - 1}, {wh, wh + 1.8f}, {to - 7, to - 2}));
			out.push_back(box(GLASS, {-hw + .95f, hw - .95f}, {wh + .7f, wh + 1.6f},
					{to - 6.8f, to - 1.95f}));
		} else if (v == Vessel::Peniche) {
			out.push_back(
					box(l.door, {-hw + .35f, hw - .35f}, {1, 2}, {-half + 8, half - 4}));
			out.push_back(box(
					l.body, {-hw + .4f, hw - .4f}, {1, 3}, {-half + .6f, -half + 6.5f}));
			out.push_back(box(GLASS, {-hw + .35f, hw - .35f}, {1.9f, 2.6f},
					{-half + 1, -half + 6.1f}));
			out.push_back(box(l.roof, {-hw + .3f, hw - .3f}, {3, 3.15f},
					{-half + .4f, -half + 6.7f}));
			if (!low) {
				out.push_back(box(
						l.body, {-1.3f, 1.3f}, {3.15f, 4.6f}, {-half + 2, -half + 4.5f}));
				out.push_back(box(GLASS, {-1.35f, 1.35f}, {3.6f, 4.4f},
						{-half + 2.1f, -half + 4.55f}));
			}
		} else if (v == Vessel::TourBoat) {
			const float from = -half + 3, to = half - 7;
			out.push_back(box(l.body, {-hw + .3f, hw - .3f}, {1, 1.6f}, {from, to}));
			out.push_back(box(
					GLASS, {-hw + .3f, hw - .3f}, {1.6f, 3.2f}, {from + .3f, to - .3f}));
			out.push_back(box(l.roof, {-hw + .25f, hw - .25f}, {3.2f, 3.4f}, {from, to}));
			if (!low)
				out.push_back(box(l.body, {-1.4f, 1.4f}, {3.4f, 5}, {to - 4, to - 1}));
		} else if (sail) {
			out.push_back(box(l.body, {-.9f, .9f}, {.85f, 1.45f}, {-1.6f, 1.4f}));
			out.push_back(box(GLASS, {-.95f, .95f}, {1.05f, 1.3f}, {-1.4f, 1.2f}));
			out.push_back(box(WHITE, {-.07f, .07f}, {.85f, 12.5f}, {1.3f, 1.45f}));
			for (auto [a, b, r] : std::array<std::array<float, 3>, 4>{{{1.8f, 4.4f, 4.8f},
						 {4.4f, 7.2f, 3.6f}, {7.2f, 9.8f, 2.3f}, {9.8f, 12.2f, 1}}})
				out.push_back(box(WHITE, {-.03f, .03f}, {a, b}, {1.25f - r, 1.25f}));
		} else {
			out.push_back(box(l.body, {-beam / 2 + .2f, beam / 2 - .2f}, {.8f, 1.15f},
					{-.2f, 1.6f}));
			out.push_back(box(GLASS, {-beam / 2 + .25f, beam / 2 - .25f}, {1.15f, 1.6f},
					{.9f, 1.15f}));
		}
		break;
	}
	}
	train_bodies::apart(out);
	return out;
}

std::vector<Livery> all_liveries()
{
	const auto make = [](std::string name, std::string_view body, std::string_view lower,
							  std::string_view stripe, std::string_view door,
							  std::string_view roof) {
		return Livery{std::string(body), std::string(lower), std::string(stripe),
				std::string(door), std::string(roof), std::move(name)};
	};
	return {make("fwb", WHITE, BLUE, BLUE, GREY, BLUE),
			make("fwr", WHITE, RED, RED, GREY, RED),
			make("fwg", WHITE, GREEN, GREEN, GREY, YELLOW),
			make("fwk", WHITE, BLACK, YELLOW, GREY, BLACK),
			make("fyb", YELLOW, BLUE, WHITE, GREY, BLUE),
			make("bk", WHITE, BLACK, RED, GREY, GREY),
			make("bb", WHITE, BLUE, WHITE, RUST, BLUE),
			make("br", WHITE, RED, BLACK, GREY, RED),
			make("bg", WHITE, GREEN, WHITE, BROWN, GREEN),
			make("bn", LIGHT, BLACK, ORANGE, GREEN, BLACK),
			make("cw", WHITE, BLUE, CYAN, LIGHT, BLUE),
			make("ck", WHITE, BLACK, WHITE, LIGHT, YELLOW),
			make("cr", WHITE, RED, WHITE, LIGHT, WHITE),
			make("sw", WHITE, WHITE, BLUE, TEAK, WHITE),
			make("sb", WHITE, BLUE, WHITE, TEAK, WHITE),
			make("sr", WHITE, RED, WHITE, TEAK, WHITE),
			make("sk", WHITE, BLACK, YELLOW, TEAK, WHITE)};
}
} // namespace

std::string_view name(Vessel v)
{
	constexpr std::array names{"river_ferry"sv, "water_bus"sv, "car_ferry"sv,
			"sea_ferry"sv, "barge"sv, "container_barge"sv, "tank_barge"sv,
			"river_cruise"sv, "peniche"sv, "tour_boat"sv, "sailboat"sv, "motorboat"sv};
	return names[static_cast<std::size_t>(v)];
}

std::pair<double, double> size_m(Vessel v)
{
	switch (v) {
	case Vessel::RiverFerry:
		return {24, 9};
	case Vessel::WaterBus:
		return {28, 7};
	case Vessel::CarFerry:
		return {72, 16};
	case Vessel::SeaFerry:
		return {135, 24};
	case Vessel::Barge:
	case Vessel::ContainerBarge:
	case Vessel::TankBarge:
		return {105, 11.4};
	case Vessel::RiverCruise:
		return {135, 11.4};
	case Vessel::Peniche:
		return {38.5, 5.05};
	case Vessel::TourBoat:
		return {40, 8};
	case Vessel::Sailboat:
		return {10, 3.4};
	case Vessel::Motorboat:
		return {7, 2.6};
	}
	return {};
}
bool lowers(Vessel v)
{
	return v == Vessel::Barge || v == Vessel::ContainerBarge || v == Vessel::TankBarge ||
		   v == Vessel::RiverCruise || v == Vessel::Peniche || v == Vessel::TourBoat;
}
double cruise_kmh(Vessel v)
{
	switch (v) {
	case Vessel::RiverFerry:
		return 8;
	case Vessel::WaterBus:
		return 20;
	case Vessel::CarFerry:
		return 22;
	case Vessel::SeaFerry:
		return 30;
	case Vessel::Barge:
	case Vessel::ContainerBarge:
	case Vessel::TankBarge:
		return 13;
	case Vessel::RiverCruise:
		return 15;
	case Vessel::Peniche:
		return 9;
	case Vessel::TourBoat:
		return 12;
	case Vessel::Sailboat:
		return 9;
	case Vessel::Motorboat:
		return 18;
	}
	return 0;
}
std::array<double, 3> acceleration(Vessel v)
{
	if (v == Vessel::Sailboat || v == Vessel::Motorboat)
		return {.25, .3, .4};
	if (v == Vessel::RiverFerry || v == Vessel::WaterBus || v == Vessel::TourBoat)
		return {.15, .2, .25};
	return {.06, .08, .12};
}
std::pair<double, double> dwell_seconds(Vessel v)
{
	switch (v) {
	case Vessel::RiverFerry:
		return {20, 40};
	case Vessel::WaterBus:
		return {25, 45};
	case Vessel::TourBoat:
		return {40, 60};
	case Vessel::CarFerry:
		return {60, 150};
	case Vessel::SeaFerry:
		return {60, 240};
	default:
		return {40, 60};
	}
}
std::pair<std::vector<float>, double> seats_m(Vessel v)
{
	switch (v) {
	case Vessel::RiverFerry:
		return {{-6, 0, 6}, 2.3};
	case Vessel::WaterBus:
		return {{-6, 0, 6}, 2.7};
	case Vessel::CarFerry:
		return {{-24, -12, 0, 12, 24}, 3.6};
	case Vessel::SeaFerry:
		return {{-30, -15, 0, 15, 30}, 18.9};
	case Vessel::Barge:
	case Vessel::ContainerBarge:
	case Vessel::TankBarge:
		return {{-49.5f}, 5.8};
	case Vessel::RiverCruise:
		return {{-45, -20, 5, 30}, 9.35};
	case Vessel::Peniche:
		return {{0}, 3.6};
	case Vessel::TourBoat:
		return {{-10, 0, 8}, 2.6};
	case Vessel::Sailboat:
		return {{-2.5f}, 2.2};
	case Vessel::Motorboat:
		return {{-1.9f}, 1.7};
	}
	return {};
}
std::vector<train_bodies::Part> parts(Vessel v, const Livery &l, bool low)
{
	auto [len, beam] = size_m(v);
	return build(v, static_cast<float>(len), static_cast<float>(beam), l, low);
}
double air_draft_m(Vessel v, bool low)
{
	auto ls = liveries(v);
	if (ls.empty())
		return 0;
	double top = 0;
	for (const auto &p : parts(v, ls.front(), low))
		top = std::max(top, double(p.corner[1] + p.size[1]));
	return top;
}
std::vector<Livery> liveries(Vessel v)
{
	auto all = all_liveries();
	if (v == Vessel::RiverFerry || v == Vessel::WaterBus || v == Vessel::CarFerry ||
			v == Vessel::SeaFerry)
		return {all.begin(), all.begin() + 5};
	if (v == Vessel::Barge || v == Vessel::ContainerBarge || v == Vessel::TankBarge ||
			v == Vessel::Peniche)
		return {all.begin() + 5, all.begin() + 10};
	if (v == Vessel::RiverCruise || v == Vessel::TourBoat)
		return {all.begin() + 10, all.begin() + 13};
	return {all.begin() + 13, all.end()};
}
} // namespace arnis::ship_bodies
