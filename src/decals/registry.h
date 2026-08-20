#pragma once

#include "region.h"

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace arnis::decals
{
enum class TextStyleKind
{
	Fascia,
	StreetName,
	HouseNumber,
	StationBoard,
	StopName,
	Plaque
};
struct TextStyle
{
	TextStyleKind kind{TextStyleKind::Fascia};
	BladeStyle blade{BladeStyle::Blue};
	bool operator==(const TextStyle &) const = default;
	bool operator<(const TextStyle &other) const
	{
		return std::tie(kind, blade) < std::tie(other.kind, other.blade);
	}
};
enum class TrafficSign
{
	Stop,
	GiveWay,
	NoEntry,
	PriorityRoad,
	Crossing,
	OneWay,
	NoParking,
	DeadEnd,
	LevelCrossing,
	HighVoltage,
	Bicycle,
	Motorway,
	MotorwayEnd
};
enum class ShieldStyle
{
	Blue,
	Yellow,
	Green,
	Interstate,
	White
};

struct PictogramKey
{
	std::string name;
	bool operator==(const PictogramKey &) const = default;
	bool operator<(const PictogramKey &other) const { return name < other.name; }
};
struct TextKey
{
	TextStyle style;
	std::string text;
	std::uint8_t cols{1};
	bool operator==(const TextKey &) const = default;
	bool operator<(const TextKey &other) const
	{
		return std::tie(style, text, cols) <
			   std::tie(other.style, other.text, other.cols);
	}
};
struct TrafficKey
{
	TrafficSign sign;
	bool operator==(const TrafficKey &) const = default;
	bool operator<(const TrafficKey &other) const { return sign < other.sign; }
};
struct SpeedLimitKey
{
	std::uint16_t value;
	bool mph;
	SpeedStyle style;
	bool operator==(const SpeedLimitKey &) const = default;
	bool operator<(const SpeedLimitKey &other) const
	{
		return std::tie(value, mph, style) <
			   std::tie(other.value, other.mph, other.style);
	}
};
struct RouteShieldKey
{
	ShieldStyle style;
	std::string text;
	bool operator==(const RouteShieldKey &) const = default;
	bool operator<(const RouteShieldKey &other) const
	{
		return std::tie(style, text) < std::tie(other.style, other.text);
	}
};
struct PosterKey
{
	std::uint8_t variant;
	bool operator==(const PosterKey &) const = default;
	bool operator<(const PosterKey &other) const { return variant < other.variant; }
};
struct ColumnPosterKey
{
	std::uint8_t variant;
	bool operator==(const ColumnPosterKey &) const = default;
	bool operator<(const ColumnPosterKey &other) const { return variant < other.variant; }
};
struct LocalMapKey
{
	int x;
	int z;
	bool operator==(const LocalMapKey &) const = default;
	bool operator<(const LocalMapKey &other) const
	{
		return std::tie(x, z) < std::tie(other.x, other.z);
	}
};

class DecalKey : public std::variant<PictogramKey, TextKey, TrafficKey, SpeedLimitKey,
						 RouteShieldKey, PosterKey, ColumnPosterKey, LocalMapKey>
{
public:
	using variant::variant;
	static DecalKey text(TextStyle style, std::string text, std::uint8_t cols);
	std::pair<std::uint32_t, std::uint32_t> dims() const;
	bool operator==(const DecalKey &other) const
	{
		return static_cast<const variant &>(*this) == static_cast<const variant &>(other);
	}
	bool operator<(const DecalKey &other) const
	{
		return static_cast<const variant &>(*this) < static_cast<const variant &>(other);
	}
};

struct DecalEntry
{
	int base_id{0};
	std::uint32_t cols{0};
	std::uint32_t rows{0};
	int tile_id(std::uint32_t col, std::uint32_t row) const;
	bool operator==(const DecalEntry &) const = default;
};

class DecalRegistry
{
	std::map<DecalKey, DecalEntry> entries_;
	std::vector<DecalKey> ordered_;
	int next_id_{FIRST_ID};

public:
	static constexpr int FIRST_ID = 2;
	static DecalRegistry from_keys(const std::set<DecalKey> &keys);
	std::optional<DecalEntry> get(const DecalKey &key) const;
	bool contains(const DecalKey &key) const;
	std::size_t size() const;
	bool empty() const;
	int max_id() const;
	int tile_count() const;
	const std::vector<DecalKey> &ordered() const;
};
}
