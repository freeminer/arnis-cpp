#include "train_bodies.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <utility>
#include <string_view>
#include <type_traits>

namespace arnis::train_bodies
{
namespace
{
constexpr std::string_view GLASS = "minecraft:tinted_glass";
constexpr std::string_view BOGIE = "minecraft:black_concrete";
constexpr std::string_view UNDER = "minecraft:gray_concrete";
constexpr std::string_view BLACK = "minecraft:black_concrete";
constexpr std::string_view HEAD = "minecraft:white_concrete";
constexpr std::string_view TAIL = "minecraft:red_concrete";
constexpr std::string_view SIGN = "minecraft:orange_concrete";
constexpr std::string_view STEEL = "minecraft:gray_concrete";
constexpr std::array<std::string_view, 10> CONTAINERS{"minecraft:blue_concrete",
		"minecraft:red_terracotta", "minecraft:green_terracotta",
		"minecraft:orange_concrete", "minecraft:white_concrete",
		"minecraft:gray_concrete", "minecraft:brown_concrete",
		"minecraft:light_blue_terracotta", "minecraft:cyan_terracotta",
		"minecraft:yellow_terracotta"};
constexpr std::array<std::string_view, 4> TANKS{"minecraft:black_concrete",
		"minecraft:white_concrete", "minecraft:light_gray_concrete",
		"minecraft:gray_concrete"};
constexpr std::array<std::string_view, 4> BOXCARS{"minecraft:brown_terracotta",
		"minecraft:red_terracotta", "minecraft:gray_terracotta",
		"minecraft:brown_concrete"};

std::uint64_t roll(std::uint64_t seed, std::uint64_t k)
{
	std::uint64_t h = seed ^ (k * 0x9E3779B97F4A7C15ULL);
	h ^= h >> 31;
	h *= 0xBF58476D1CE4E5B9ULL;
	return h ^ (h >> 29);
}

std::size_t wagon(std::uint8_t variant, std::uint8_t seed)
{
	return static_cast<std::size_t>(roll(variant, seed) % 1000);
}

struct Spec
{
	float len{}, width{}, floor{}, top{};
	std::vector<std::pair<float, float>> windows;
	std::vector<float> doors;
	float door_width{};
	std::vector<float> bogies;
	std::pair<float, float> bogie{};
	float band{};
	std::optional<std::pair<float, float>> stripe;
	std::optional<float> pantograph;
	std::optional<float> cab;
	bool tram_nose{};
	bool streamlined{};
};

Part box(std::string_view block, std::pair<float, float> x, std::pair<float, float> y,
		std::pair<float, float> z)
{
	return {std::string(block),
			{x.second - x.first, y.second - y.first,
					std::max(z.first, z.second) - std::min(z.first, z.second)},
			{x.first, y.first, std::min(z.first, z.second)}, 0.0f, false};
}

Part plate(std::string_view block, std::pair<float, float> x,
		std::pair<float, float> from, std::pair<float, float> to, float thickness)
{
	const float dy = to.first - from.first;
	const float dz = to.second - from.second;
	return {std::string(block), {x.second - x.first, thickness, std::hypot(dy, dz)},
			{x.first, from.first, from.second}, std::atan2(-dy, dz), false};
}

void append(std::vector<Part> &out, Part part, bool glow = false)
{
	part.glow = glow;
	out.push_back(std::move(part));
}

std::pair<float, float> directed(float face, float direction, float a, float b)
{
	return {face + direction * a, face + direction * b};
}

void pantograph(std::vector<Part> &out, float roof, float z)
{
	append(out, box(STEEL, {-0.5f, 0.5f}, {roof, roof + 0.16f}, {z - 0.7f, z + 0.7f}));
	append(out, plate(STEEL, {-0.05f, 0.05f}, {roof + 0.16f, z - 0.55f},
						{roof + 0.95f, z + 0.3f}, 0.08f));
	append(out, box(BLACK, {-0.85f, 0.85f}, {roof + 0.93f, roof + 1.0f},
						{z + 0.22f, z + 0.42f}));
}

void cab(std::vector<Part> &out, const Spec &s, const Livery &livery, float direction,
		float face)
{
	const float hw = s.width / 2.0f;
	const auto at = [&](float a, float b) { return directed(face, direction, a, b); };
	const float lamp_base = s.floor + (s.tram_nose ? 0.45f : 0.35f);
	const auto lamps = std::pair{lamp_base, lamp_base + 0.2f};
	if (!s.streamlined) {
		const float low = s.tram_nose		  ? s.floor + 0.95f
						  : s.windows.empty() ? s.floor + 1.2f
											  : s.windows.front().first - 0.1f;
		append(out, box(GLASS, {-hw + 0.22f, hw - 0.22f}, {low, s.top - 0.36f},
							at(-0.02f, 0.04f)));
		append(out,
				box(SIGN, {-hw + 0.55f, hw - 0.55f}, {s.top - 0.31f, s.top - 0.1f},
						at(-0.02f, 0.04f)),
				true);
		if (s.tram_nose)
			append(out,
					box(livery.lower, {-hw + 0.08f, hw - 0.08f},
							{std::max(s.floor, 0.3f), s.floor + 0.8f}, at(0.0f, 0.14f)));
		else
			append(out, box(BLACK, {-hw + 0.3f, hw - 0.3f}, {0.55f, s.floor + 0.15f},
								at(0.0f, 0.22f)));
		const auto lamp_z = s.tram_nose ? at(0.1f, 0.17f) : at(0.0f, 0.05f);
		append(out, box(HEAD, {hw - 0.62f, hw - 0.28f}, lamps, lamp_z), true);
		append(out, box(HEAD, {-hw + 0.28f, -hw + 0.62f}, lamps, lamp_z), true);
		append(out, box(TAIL, {hw - 0.9f, hw - 0.7f}, lamps, lamp_z), true);
		append(out, box(TAIL, {-hw + 0.7f, -hw + 0.9f}, lamps, lamp_z), true);
		return;
	}
	const float half = s.len / 2.0f;
	const float tip = direction * (half - 0.15f);
	const auto span = [&](float reach) {
		return std::pair{face, direction * (half - reach)};
	};
	append(out, box(livery.body, {-hw, hw}, {s.floor, s.floor + 1.05f}, {face, tip}));
	append(out, box(livery.body, {-hw + 0.08f, hw - 0.08f},
						{s.floor + 1.05f, s.floor + 1.75f}, span(1.3f)));
	append(out, box(livery.body, {-hw + 0.25f, hw - 0.25f},
						{s.floor + 1.75f, s.top - 0.12f}, span(2.7f)));
	append(out, plate(GLASS, {-hw + 0.3f, hw - 0.3f},
						{s.floor + 1.0f, direction * (half - 0.95f)},
						{s.top - 0.08f, direction * (half - 2.85f)}, 0.06f));
	if (s.stripe)
		append(out, box(livery.stripe, {-hw - 0.02f, hw + 0.02f}, *s.stripe,
							{face, direction * (half - 0.7f)}));
	append(out, box(UNDER, {-hw + 0.2f, hw - 0.2f}, {0.45f, s.floor},
						{face, direction * (half - 0.4f)}));
	const auto nose_lamps = std::pair{s.floor + 0.42f, s.floor + 0.62f};
	const auto nose_z = std::pair{tip - direction * 0.02f, tip + direction * 0.03f};
	append(out, box(HEAD, {hw - 0.75f, hw - 0.4f}, nose_lamps, nose_z), true);
	append(out, box(HEAD, {-hw + 0.4f, -hw + 0.75f}, nose_lamps, nose_z), true);
	append(out, box(TAIL, {hw - 1.0f, hw - 0.82f}, nose_lamps, nose_z), true);
	append(out, box(TAIL, {-hw + 0.82f, -hw + 1.0f}, nose_lamps, nose_z), true);
}

std::vector<Part> passenger(const Spec &s, const Livery &livery)
{
	std::vector<Part> out;
	const float hw = s.width / 2.0f, half = s.len / 2.0f;
	const float nose = s.streamlined ? 3.6f : 0.0f;
	const float z0 = s.cab && *s.cab > 0.0f ? -half + 0.15f
					 : s.cab				? -half + nose
											: -half + 0.15f;
	const float z1 = s.cab && *s.cab > 0.0f ? half - nose
					 : s.cab				? half - 0.15f
											: half - 0.15f;
	for (float z : s.bogies)
		append(out, box(BOGIE, {-hw + 0.35f, hw - 0.35f}, {0.04f, s.bogie.second},
							{z - s.bogie.first / 2.0f, z + s.bogie.first / 2.0f}));
	if (s.floor > 0.7f)
		append(out, box(UNDER, {-hw + 0.15f, hw - 0.15f}, {0.55f, s.floor},
							{z0 + 0.4f, z1 - 0.4f}));
	append(out, box(livery.body, {-hw, hw}, {s.floor, s.top}, {z0, z1}));
	if (s.band > 0.0f)
		append(out, box(livery.lower, {-hw - 0.01f, hw + 0.01f},
							{s.floor, s.floor + s.band}, {z0, z1}));
	if (s.stripe)
		append(out, box(livery.stripe, {-hw - 0.02f, hw + 0.02f}, *s.stripe, {z0, z1}));
	for (const auto &[a, b] : s.windows)
		append(out,
				box(GLASS, {-hw - 0.03f, hw + 0.03f}, {a, b}, {z0 + 0.7f, z1 - 0.7f}));
	const float door_top =
			s.windows.empty() ? s.top - 0.3f : s.windows.front().second + 0.12f;
	for (float z : s.doors)
		append(out,
				box(livery.door, {-hw - 0.045f, hw + 0.045f}, {s.floor + 0.05f, door_top},
						{z - s.door_width / 2.0f, z + s.door_width / 2.0f}));
	append(out, box(livery.roof, {-hw + 0.12f, hw - 0.12f}, {s.top, s.top + 0.14f},
						{z0 + 0.25f, z1 - 0.25f}));
	append(out, box(livery.roof, {-hw + 0.6f, hw - 0.6f}, {s.top + 0.14f, s.top + 0.26f},
						{z0 + 0.5f, z1 - 0.5f}));
	for (float direction : {-1.0f, 1.0f}) {
		if (s.cab && *s.cab == direction)
			continue;
		const float face = direction * half;
		append(out, box(BLACK, {-hw + 0.5f, hw - 0.5f}, {s.floor + 0.15f, s.top - 0.2f},
							{face - direction * 0.3f, face + direction * 0.22f}));
	}
	if (s.cab)
		cab(out, s, livery, *s.cab, *s.cab > 0.0f ? z1 : z0);
	if (s.pantograph)
		pantograph(out, s.top + 0.26f, *s.pantograph);
	if (s.len > 12.0f) {
		const float az =
				s.pantograph && *s.pantograph > 0.0f ? -s.len / 4.0f : half - 0.5f;
		append(out, box(UNDER, {-0.75f, 0.75f}, {s.top + 0.26f, s.top + 0.55f},
							{az - 1.3f, az + 1.3f}));
	}
	return out;
}

std::vector<Part> loco(const Livery &livery)
{
	const float half = 9.75f, hw = 1.5f, floor = 1.15f, top = 4.1f;
	std::vector<Part> out;
	append(out, box(BOGIE, {-hw + 0.35f, hw - 0.35f}, {0.04f, 1.0f}, {-7.95f, -4.35f}));
	append(out, box(BOGIE, {-hw + 0.35f, hw - 0.35f}, {0.04f, 1.0f}, {4.35f, 7.95f}));
	append(out, box(UNDER, {-hw + 0.15f, hw - 0.15f}, {0.55f, floor}, {-9.25f, 9.25f}));
	append(out, box(livery.body, {-hw, hw}, {floor, top}, {-9.6f, 9.6f}));
	append(out, box(livery.lower, {-hw - 0.01f, hw + 0.01f}, {floor, floor + 0.35f},
						{-9.6f, 9.6f}));
	append(out, box(livery.stripe, {-hw - 0.02f, hw + 0.02f},
						{floor + 0.55f, floor + 0.75f}, {-9.6f, 9.6f}));
	append(out, box(UNDER, {-hw - 0.02f, hw + 0.02f}, {floor + 1.2f, floor + 2.3f},
						{-6.35f, 6.35f}));
	append(out, box(GLASS, {-hw - 0.03f, hw + 0.03f}, {floor + 1.35f, floor + 2.25f},
						{7.15f, 8.55f}));
	append(out, box(GLASS, {-hw - 0.03f, hw + 0.03f}, {floor + 1.35f, floor + 2.25f},
						{-8.55f, -7.15f}));
	append(out, box(livery.roof, {-hw + 0.12f, hw - 0.12f}, {top, top + 0.14f},
						{-9.35f, 9.35f}));
	Spec s{19.5f, 3.0f, floor, top, {{floor + 1.35f, top - 0.4f}}, {}, 0.0f,
			{-6.15f, 6.15f}, {3.6f, 1.0f}, 0.0f, {{floor + 0.55f, floor + 0.75f}},
			std::nullopt, std::nullopt, false, false};
	cab(out, s, livery, 1.0f, half - 0.15f);
	cab(out, s, livery, -1.0f, -half + 0.15f);
	pantograph(out, top + 0.14f, half - 4.6f);
	pantograph(out, top + 0.14f, -half + 4.6f);
	return out;
}

using LiverySet = std::vector<Livery>;

Livery make_livery(const char *name, const char *body, const char *lower,
		const char *stripe, const char *door, const char *roof)
{
	return {body, lower, stripe, door, roof, name, 0};
}

const LiverySet &liveries(Kind kind)
{
	static const std::string light_gray = "minecraft:light_gray_concrete";
	static const std::string white = "minecraft:white_concrete";
	static const LiverySet trams{make_livery("ty", "minecraft:yellow_concrete",
										 light_gray.c_str(), "minecraft:yellow_concrete",
										 "minecraft:yellow_concrete", light_gray.c_str()),
			make_livery("twr", white.c_str(), "minecraft:red_concrete",
					"minecraft:red_concrete", "minecraft:red_concrete",
					light_gray.c_str()),
			make_livery("trw", "minecraft:red_concrete", white.c_str(), white.c_str(),
					white.c_str(), light_gray.c_str()),
			make_livery("tbw", white.c_str(), "minecraft:blue_concrete",
					"minecraft:blue_concrete", "minecraft:blue_concrete",
					light_gray.c_str()),
			make_livery("tgw", white.c_str(), "minecraft:green_concrete",
					"minecraft:green_concrete", "minecraft:green_concrete",
					light_gray.c_str())};
	static const LiverySet metros{
			make_livery("my", "minecraft:yellow_concrete", "minecraft:gray_concrete",
					"minecraft:yellow_concrete", "minecraft:yellow_concrete",
					light_gray.c_str()),
			make_livery("mb", light_gray.c_str(), light_gray.c_str(),
					"minecraft:blue_concrete", "minecraft:blue_concrete",
					"minecraft:gray_concrete"),
			make_livery("mr", white.c_str(), white.c_str(), "minecraft:red_concrete",
					"minecraft:red_concrete", light_gray.c_str()),
			make_livery("mo", light_gray.c_str(), light_gray.c_str(),
					"minecraft:orange_concrete", "minecraft:orange_concrete",
					"minecraft:gray_concrete")};
	static const LiverySet sbahns{
			make_livery("sb", "minecraft:yellow_terracotta", "minecraft:red_concrete",
					"minecraft:yellow_terracotta", "minecraft:red_concrete",
					light_gray.c_str()),
			make_livery("sd", white.c_str(), light_gray.c_str(), "minecraft:red_concrete",
					"minecraft:red_concrete", light_gray.c_str()),
			make_livery("sg", white.c_str(), "minecraft:green_concrete",
					"minecraft:green_concrete", "minecraft:green_concrete",
					light_gray.c_str())};
	static const LiverySet regionals{
			make_livery("rr", "minecraft:red_concrete", light_gray.c_str(), white.c_str(),
					light_gray.c_str(), light_gray.c_str()),
			make_livery("rb", white.c_str(), "minecraft:blue_concrete",
					"minecraft:blue_concrete", "minecraft:blue_concrete",
					light_gray.c_str()),
			make_livery("rg", white.c_str(), "minecraft:lime_concrete",
					"minecraft:green_concrete", "minecraft:lime_concrete",
					light_gray.c_str()),
			make_livery("rk", "minecraft:yellow_concrete", "minecraft:gray_concrete",
					"minecraft:gray_concrete", "minecraft:gray_concrete",
					light_gray.c_str())};
	static const LiverySet decks{
			make_livery("dr", "minecraft:red_concrete", light_gray.c_str(), white.c_str(),
					light_gray.c_str(), light_gray.c_str()),
			make_livery("db", white.c_str(), "minecraft:blue_concrete",
					"minecraft:blue_concrete", "minecraft:blue_concrete",
					light_gray.c_str())};
	static const LiverySet intercity{
			make_livery("ice", white.c_str(), white.c_str(), "minecraft:red_concrete",
					light_gray.c_str(), light_gray.c_str()),
			make_livery("icb", white.c_str(), white.c_str(), "minecraft:blue_concrete",
					light_gray.c_str(), light_gray.c_str())};
	static const LiverySet locos{
			make_livery("fr", "minecraft:red_concrete", "minecraft:gray_concrete",
					white.c_str(), "minecraft:red_concrete", light_gray.c_str()),
			make_livery("fb", "minecraft:blue_concrete", "minecraft:gray_concrete",
					white.c_str(), "minecraft:blue_concrete", light_gray.c_str()),
			make_livery("fs", light_gray.c_str(), "minecraft:gray_concrete",
					"minecraft:red_concrete", light_gray.c_str(),
					"minecraft:gray_concrete"),
			make_livery("fg", "minecraft:green_concrete", "minecraft:gray_concrete",
					"minecraft:yellow_concrete", "minecraft:green_concrete",
					light_gray.c_str()),
			make_livery("fk", "minecraft:black_concrete", "minecraft:gray_concrete",
					"minecraft:yellow_concrete", "minecraft:black_concrete",
					"minecraft:gray_concrete")};
	switch (kind) {
	case Kind::Tram:
		return trams;
	case Kind::Metro:
		return metros;
	case Kind::SBahn:
		return sbahns;
	case Kind::Regional:
		return regionals;
	case Kind::DoubleDeck:
		return decks;
	case Kind::Intercity:
		return intercity;
	case Kind::Freight:
		return locos;
	}
	return regionals;
}

CoachKind coach_kind(const Coach &coach)
{
	return std::visit(
			[](const auto &variant) -> CoachKind {
				using T = std::decay_t<decltype(variant)>;
				if constexpr (std::is_same_v<T, TramCab>)
					return CoachKind::TramCab;
				else if constexpr (std::is_same_v<T, TramMid>)
					return CoachKind::TramMid;
				else if constexpr (std::is_same_v<T, TramShort>)
					return CoachKind::TramShort;
				else if constexpr (std::is_same_v<T, MetroCab>)
					return CoachKind::MetroCab;
				else if constexpr (std::is_same_v<T, MetroMid>)
					return CoachKind::MetroMid;
				else if constexpr (std::is_same_v<T, SBahnCab>)
					return CoachKind::SBahnCab;
				else if constexpr (std::is_same_v<T, SBahnMid>)
					return CoachKind::SBahnMid;
				else if constexpr (std::is_same_v<T, RegioCab>)
					return CoachKind::RegioCab;
				else if constexpr (std::is_same_v<T, RegioMid>)
					return CoachKind::RegioMid;
				else if constexpr (std::is_same_v<T, DeckCab>)
					return CoachKind::DeckCab;
				else if constexpr (std::is_same_v<T, DeckMid>)
					return CoachKind::DeckMid;
				else if constexpr (std::is_same_v<T, Loco>)
					return CoachKind::Loco;
				else if constexpr (std::is_same_v<T, IceCab>)
					return CoachKind::IceCab;
				else if constexpr (std::is_same_v<T, IceMid>)
					return CoachKind::IceMid;
				else if constexpr (std::is_same_v<T, Container>)
					return CoachKind::Container;
				else if constexpr (std::is_same_v<T, Tank>)
					return CoachKind::Tank;
				else
					return CoachKind::Boxcar;
			},
			coach);
}

bool coach_front(const Coach &coach)
{
	return std::visit(
			[](const auto &variant) {
				using T = std::decay_t<decltype(variant)>;
				if constexpr (std::is_same_v<T, TramCab> || std::is_same_v<T, MetroCab> ||
							  std::is_same_v<T, SBahnCab> ||
							  std::is_same_v<T, RegioCab> || std::is_same_v<T, DeckCab> ||
							  std::is_same_v<T, IceCab>)
					return variant.front;
				return false;
			},
			coach);
}

std::uint8_t coach_variant(const Coach &coach)
{
	return std::visit(
			[](const auto &variant) {
				using T = std::decay_t<decltype(variant)>;
				if constexpr (std::is_same_v<T, Container> || std::is_same_v<T, Tank> ||
							  std::is_same_v<T, Boxcar>)
					return variant.variant;
				return std::uint8_t{0};
			},
			coach);
}

Coach make_coach(CoachKind kind, bool front = false, std::uint8_t variant = 0)
{
	switch (kind) {
	case CoachKind::TramCab:
		return TramCab{front};
	case CoachKind::TramMid:
		return TramMid{};
	case CoachKind::TramShort:
		return TramShort{};
	case CoachKind::MetroCab:
		return MetroCab{front};
	case CoachKind::MetroMid:
		return MetroMid{};
	case CoachKind::SBahnCab:
		return SBahnCab{front};
	case CoachKind::SBahnMid:
		return SBahnMid{};
	case CoachKind::RegioCab:
		return RegioCab{front};
	case CoachKind::RegioMid:
		return RegioMid{};
	case CoachKind::DeckCab:
		return DeckCab{front};
	case CoachKind::DeckMid:
		return DeckMid{};
	case CoachKind::Loco:
		return Loco{};
	case CoachKind::IceCab:
		return IceCab{front};
	case CoachKind::IceMid:
		return IceMid{};
	case CoachKind::Container:
		return Container{variant};
	case CoachKind::Tank:
		return Tank{variant};
	case CoachKind::Boxcar:
		return Boxcar{variant};
	}
	return Loco{};
}

std::pair<std::vector<Coach>, std::function<Livery(std::size_t)>> compose_impl(
		Kind kind, std::uint64_t seed, bool short_train)
{
	const auto &palette = liveries(kind);
	Livery base = palette[roll(seed, 1) % palette.size()];
	std::vector<Coach> coaches;
	const auto add = [&](CoachKind coach_kind, bool front = false,
							 std::uint8_t variant = 0) {
		coaches.push_back(make_coach(coach_kind, front, variant));
	};
	const auto repeated = [&](CoachKind coach_kind, std::size_t count) {
		for (std::size_t i = 0; i < count; ++i)
			add(coach_kind);
	};
	switch (kind) {
	case Kind::Tram:
		add(CoachKind::TramCab, true);
		add(CoachKind::TramShort);
		if (!short_train) {
			add(CoachKind::TramMid);
			add(CoachKind::TramShort);
		}
		add(CoachKind::TramCab, false);
		break;
	case Kind::Metro:
		add(CoachKind::MetroCab, true);
		repeated(CoachKind::MetroMid, short_train ? 2 : 4);
		add(CoachKind::MetroCab, false);
		break;
	case Kind::SBahn:
		add(CoachKind::SBahnCab, true);
		repeated(CoachKind::SBahnMid, short_train ? 2 : 4);
		add(CoachKind::SBahnCab, false);
		break;
	case Kind::Regional:
		add(CoachKind::RegioCab, true);
		repeated(CoachKind::RegioMid, short_train ? 1 : 2);
		add(CoachKind::RegioCab, false);
		break;
	case Kind::Intercity:
		add(CoachKind::IceCab, true);
		repeated(CoachKind::IceMid, short_train ? 2 : 5);
		add(CoachKind::IceCab, false);
		break;
	case Kind::DoubleDeck:
		add(CoachKind::Loco);
		repeated(CoachKind::DeckMid, short_train ? 1 : 3);
		add(CoachKind::DeckCab, false);
		break;
	case Kind::Freight:
		add(CoachKind::Loco);
		for (std::uint64_t k = 0; k < (short_train ? 4U : 10U); ++k) {
			const auto r = roll(seed, 100 + k);
			const auto variant = static_cast<std::uint8_t>(r >> 8);
			const auto choice = r % 100;
			add(choice <= 44	   ? CoachKind::Container
					: choice <= 74 ? CoachKind::Tank
								   : CoachKind::Boxcar,
					false, variant);
		}
		break;
	}
	auto livery_for_departure = [kind, seed, base = std::move(base)](
										std::size_t departure) {
		if (kind != Kind::Freight)
			return base;
		const auto &freight_liveries = liveries(Kind::Freight);
		Livery livery =
				freight_liveries[roll(seed, 50 + static_cast<std::uint64_t>(departure)) %
								 freight_liveries.size()];
		livery.seed = static_cast<std::uint8_t>(
				roll(seed, 60 + static_cast<std::uint64_t>(departure)) & 0xff);
		return livery;
	};
	return {std::move(coaches), std::move(livery_for_departure)};
}
} // namespace

std::pair<std::vector<Coach>, std::function<Livery(std::size_t)>> compose(
		Kind kind, std::uint64_t seed, bool short_train)
{
	return compose_impl(kind, seed, short_train);
}

double length_m(const Coach &coach)
{
	switch (coach_kind(coach)) {
	case CoachKind::TramCab:
		return 7.4;
	case CoachKind::TramMid:
		return 7.0;
	case CoachKind::TramShort:
		return 4.0;
	case CoachKind::MetroCab:
	case CoachKind::MetroMid:
		return 16.0;
	case CoachKind::SBahnCab:
	case CoachKind::SBahnMid:
		return 18.4;
	case CoachKind::RegioCab:
	case CoachKind::RegioMid:
		return 20.0;
	case CoachKind::DeckCab:
	case CoachKind::DeckMid:
		return 26.8;
	case CoachKind::Loco:
		return 19.5;
	case CoachKind::IceCab:
		return 25.8;
	case CoachKind::IceMid:
		return 24.8;
	case CoachKind::Container:
		return 18.4;
	case CoachKind::Tank:
		return 15.0;
	case CoachKind::Boxcar:
		return 16.5;
	}
	return 0.0;
}

std::vector<Part> parts(const Coach &coach, const Livery &livery)
{
	const auto kind = coach_kind(coach);
	const float len = static_cast<float>(length_m(coach));
	const float half = len / 2.0f;
	const auto cab_dir = coach_front(coach) ? 1.0f : -1.0f;
	Spec s;
	s.len = len;
	switch (kind) {
	case CoachKind::TramCab:
		s = {len, 2.4f, 0.35f, 3.3f, {{1.25f, 2.75f}}, {-cab_dir * 1.6f}, 1.3f,
				{-cab_dir * 0.6f}, {2.0f, 0.55f}, 0.45f, std::nullopt, std::nullopt,
				cab_dir, true, false};
		break;
	case CoachKind::TramMid:
	case CoachKind::TramShort: {
		const bool short_body = kind == CoachKind::TramShort;
		s = {len, 2.4f, 0.35f, 3.3f, {{1.25f, 2.75f}},
				short_body ? std::vector<float>{} : std::vector<float>{0.0f}, 1.3f,
				short_body ? std::vector<float>{} : std::vector<float>{0.0f},
				{2.0f, 0.55f}, 0.45f, std::nullopt,
				short_body ? std::nullopt : std::optional<float>(-1.8f), std::nullopt,
				true, false};
		break;
	}
	case CoachKind::MetroCab:
	case CoachKind::MetroMid:
		s = {len, 2.65f, 0.95f, 3.5f, {{1.85f, 2.85f}}, {-5.0f, 0.0f, 5.0f}, 1.3f,
				{-half + 2.4f, half - 2.4f}, {2.4f, 0.85f}, 0.3f, {{1.55f, 1.7f}},
				std::nullopt,
				kind == CoachKind::MetroCab ? std::optional(cab_dir) : std::nullopt,
				false, false};
		break;
	case CoachKind::SBahnCab:
	case CoachKind::SBahnMid:
		s = {len, 3.0f, 1.0f, 3.95f, {{2.0f, 3.05f}}, {-5.6f, 0.0f, 5.6f}, 1.4f,
				{-half + 2.5f, half - 2.5f}, {2.6f, 0.9f}, 1.0f, {{1.82f, 1.96f}},
				kind == CoachKind::SBahnCab && livery.name != "sb"
						? std::optional(cab_dir * (half - 4.0f))
						: std::nullopt,
				kind == CoachKind::SBahnCab ? std::optional(cab_dir) : std::nullopt,
				false, false};
		break;
	case CoachKind::RegioCab:
	case CoachKind::RegioMid:
		s = {len, 2.9f, 0.75f, 3.9f, {{1.75f, 2.85f}}, {-half + 3.2f, half - 3.2f}, 1.3f,
				{-half + 2.4f, half - 2.4f}, {2.5f, 0.8f}, 0.35f, {{1.5f, 1.62f}},
				kind == CoachKind::RegioCab ? std::optional(cab_dir * (half - 5.0f))
											: std::nullopt,
				kind == CoachKind::RegioCab ? std::optional(cab_dir) : std::nullopt,
				false, false};
		break;
	case CoachKind::DeckCab:
	case CoachKind::DeckMid:
		s = {len, 2.8f, 0.6f, 4.6f, {{1.35f, 2.2f}, {2.95f, 3.85f}},
				{-half + 2.6f, half - 2.6f}, 1.4f, {-half + 2.6f, half - 2.6f},
				{2.6f, 0.75f}, 0.45f, {{2.45f, 2.62f}}, std::nullopt,
				kind == CoachKind::DeckCab ? std::optional(cab_dir) : std::nullopt, false,
				false};
		break;
	case CoachKind::IceCab:
	case CoachKind::IceMid: {
		const std::vector<float> ends{-half + 1.6f, half - 1.6f};
		s = {len, 2.95f, 1.15f, 3.85f, {{2.15f, 3.15f}},
				kind == CoachKind::IceMid ? ends : std::vector<float>{0.0f}, 1.0f,
				{-half + 3.0f, half - 3.0f}, {3.0f, 1.0f}, 0.0f, {{1.9f, 2.05f}},
				kind == CoachKind::IceCab ? std::optional(-cab_dir * (half - 6.0f))
										  : std::nullopt,
				kind == CoachKind::IceCab ? std::optional(cab_dir) : std::nullopt, false,
				true};
		break;
	}
	case CoachKind::Loco:
		return loco(livery);
	case CoachKind::Container:
	case CoachKind::Tank:
	case CoachKind::Boxcar: {
		std::vector<Part> out{
				box(BLACK, {-1.15f, 1.15f}, {0.04f, 0.85f}, {-half + 1.6f, -half + 4.2f}),
				box(BLACK, {-1.15f, 1.15f}, {0.04f, 0.85f}, {half - 4.2f, half - 1.6f}),
				box(UNDER, {-1.4f, 1.4f}, {0.85f, 1.25f}, {-half + 0.3f, half - 0.3f}),
				box(BLACK, {-1.1f, 1.1f}, {0.82f, 1.15f}, {half - 0.3f, half + 0.18f}),
				box(BLACK, {-1.1f, 1.1f}, {0.82f, 1.15f}, {-half - 0.18f, -half + 0.3f})};
		const auto choice = wagon(coach_variant(coach), livery.seed);
		if (kind == CoachKind::Container) {
			const auto colour = [&](std::size_t k) {
				return CONTAINERS[(choice / (k + 1)) % CONTAINERS.size()];
			};
			if (choice % 10 <= 3)
				append(out,
						box(colour(0), {-1.22f, 1.22f}, {1.25f, 3.85f}, {-6.1f, 6.1f}));
			else if (choice % 10 != 4) {
				append(out, box(colour(0), {-1.22f, 1.22f}, {1.25f, 3.85f},
									{-6.35f, -0.25f}));
				append(out,
						box(colour(1), {-1.22f, 1.22f}, {1.25f, 3.85f}, {0.25f, 6.35f}));
			}
		} else if (kind == CoachKind::Tank) {
			const auto colour = TANKS[choice % TANKS.size()];
			append(out, box(colour, {-1.0f, 1.0f}, {1.35f, 3.65f},
								{-half + 0.7f, half - 0.7f}));
			append(out, box(colour, {-1.35f, 1.35f}, {1.7f, 3.3f},
								{-half + 0.95f, half - 0.95f}));
			append(out, box(colour, {-0.4f, 0.4f}, {3.65f, 3.9f}, {-0.5f, 0.5f}));
		} else {
			const auto colour = BOXCARS[choice % BOXCARS.size()];
			append(out, box(colour, {-1.4f, 1.4f}, {1.25f, 3.95f},
								{-half + 0.35f, half - 0.35f}));
			append(out, box(colour, {-1.25f, 1.25f}, {3.95f, 4.12f},
								{-half + 0.45f, half - 0.45f}));
			append(out, box("minecraft:gray_terracotta", {-1.43f, 1.43f}, {1.4f, 3.7f},
								{-1.6f, 1.6f}));
		}
		return out;
	}
	}
	return passenger(s, livery);
}

std::string key(const Coach &coach, const Livery &livery)
{
	const auto kind = coach_kind(coach);
	std::string base;
	switch (kind) {
	case CoachKind::TramCab:
		base = coach_front(coach) ? "tram_cab_f" : "tram_cab_b";
		break;
	case CoachKind::TramMid:
		base = "tram_mid";
		break;
	case CoachKind::TramShort:
		base = "tram_short";
		break;
	case CoachKind::MetroCab:
		base = coach_front(coach) ? "metro_cab_f" : "metro_cab_b";
		break;
	case CoachKind::MetroMid:
		base = "metro_mid";
		break;
	case CoachKind::SBahnCab:
		base = coach_front(coach) ? "sbahn_cab_f" : "sbahn_cab_b";
		break;
	case CoachKind::SBahnMid:
		base = "sbahn_mid";
		break;
	case CoachKind::RegioCab:
		base = coach_front(coach) ? "regio_cab_f" : "regio_cab_b";
		break;
	case CoachKind::RegioMid:
		base = "regio_mid";
		break;
	case CoachKind::DeckCab:
		base = coach_front(coach) ? "deck_cab_f" : "deck_cab_b";
		break;
	case CoachKind::DeckMid:
		base = "deck_mid";
		break;
	case CoachKind::Loco:
		base = "loco";
		break;
	case CoachKind::IceCab:
		base = coach_front(coach) ? "ice_cab_f" : "ice_cab_b";
		break;
	case CoachKind::IceMid:
		base = "ice_mid";
		break;
	case CoachKind::Container:
		base = "container_" +
			   std::to_string(wagon(coach_variant(coach), livery.seed) % 100);
		break;
	case CoachKind::Tank:
		base = "tank_" +
			   std::to_string(wagon(coach_variant(coach), livery.seed) % TANKS.size());
		break;
	case CoachKind::Boxcar:
		base = "boxcar_" +
			   std::to_string(wagon(coach_variant(coach), livery.seed) % BOXCARS.size());
		break;
	}
	if (kind != CoachKind::Container && kind != CoachKind::Tank &&
			kind != CoachKind::Boxcar)
		base += "_" + livery.name;
	return base;
}

std::string summon_function(const std::vector<Part> &parts, double size, bool bare)
{
	const float scale = static_cast<float>(size);
	const auto vector = [scale](const std::array<float, 3> &value) {
		std::ostringstream out;
		out << std::fixed << std::setprecision(3) << '[' << value[0] * scale << "f,"
			<< value[1] * scale << "f," << value[2] * scale << "f]";
		return out.str();
	};
	std::ostringstream out;
	for (const auto &part : parts) {
		const float half = part.tilt / 2.0f;
		const std::string block =
				bare ? "\"" + part.block + "\"" : "{Name:\"" + part.block + "\"}";
		out << "$summon block_display ~ ~ ~ {Tags:[\"arnis.tp\",\"arnis.tc$(c)\"],Rotation:[$(h)f,0f],teleport_duration:2,view_range:3f,block_state:"
			<< block << ",transformation:{left_rotation:[" << std::fixed
			<< std::setprecision(4) << std::sin(half) << "f,0f,0f," << std::cos(half)
			<< "f],right_rotation:[0f,0f,0f,1f],translation:" << vector(part.corner)
			<< ",scale:" << vector(part.size) << '}';
		if (part.glow)
			out << ",brightness:{sky:15,block:15}";
		out << "}\n";
	}
	return out.str();
}
} // namespace arnis::train_bodies
