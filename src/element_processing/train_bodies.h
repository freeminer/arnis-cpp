#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace arnis::train_bodies
{
/// What a mapped rail route's trains are.
enum class Kind : std::uint8_t
{
	Tram,
	Metro,
	SBahn,
	Regional,
	DoubleDeck,
	Intercity,
	Freight
};

enum class CoachKind : std::uint8_t
{
	TramCab,
	TramMid,
	TramShort,
	MetroCab,
	MetroMid,
	SBahnCab,
	SBahnMid,
	RegioCab,
	RegioMid,
	DeckCab,
	DeckMid,
	Loco,
	IceCab,
	IceMid,
	Container,
	Tank,
	Boxcar
};

struct TramCab
{
	bool front{};
};
struct TramMid
{
};
struct TramShort
{
};
struct MetroCab
{
	bool front{};
};
struct MetroMid
{
};
struct SBahnCab
{
	bool front{};
};
struct SBahnMid
{
};
struct RegioCab
{
	bool front{};
};
struct RegioMid
{
};
struct DeckCab
{
	bool front{};
};
struct DeckMid
{
};
struct Loco
{
};
struct IceCab
{
	bool front{};
};
struct IceMid
{
};
struct Container
{
	std::uint8_t variant{};
};
struct Tank
{
	std::uint8_t variant{};
};
struct Boxcar
{
	std::uint8_t variant{};
};

/// One carriage, mirroring the Rust enum: payloads exist only on variants that use them.
using Coach = std::variant<TramCab, TramMid, TramShort, MetroCab, MetroMid, SBahnCab,
		SBahnMid, RegioCab, RegioMid, DeckCab, DeckMid, Loco, IceCab, IceMid, Container,
		Tank, Boxcar>;

/// A train's colours and deterministic freight palette seed.
struct Livery
{
	std::string body;
	std::string lower;
	std::string stripe;
	std::string door;
	std::string roof;
	std::string name;
	std::uint8_t seed{};
};

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

std::pair<std::vector<Coach>, std::function<Livery(std::size_t)>> compose(
		Kind kind, std::uint64_t seed, bool short_train);
double length_m(const Coach &coach);
std::vector<Part> parts(const Coach &coach, const Livery &livery);
std::string key(const Coach &coach, const Livery &livery);
/// Emit the macro body that summons a carriage's block displays at its centre.
std::string summon_function(const std::vector<Part> &parts, double size, bool bare);
} // namespace arnis::train_bodies
