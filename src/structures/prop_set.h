#pragma once

#include <cstdint>
#include <algorithm>
#include <cctype>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>

namespace arnis::structures
{
enum class Prop : std::uint8_t
{
	Boat,
	Car,
	Crane,
	Excavator,
	Fountain,
	Helicopter,
	Jetbridge,
	Landmark,
	Lighthouse,
	Plane,
	Playground,
	Starship,
	Tombstone,
	Tractor,
	Windturbine,
};

class PropSet
{
public:
	static constexpr PropSet all() { return PropSet((1u << 15) - 1); }
	static constexpr PropSet none() { return PropSet(0); }
	constexpr bool has(Prop prop) const
	{
		return (bits_ & (1u << static_cast<unsigned>(prop))) != 0;
	}
	constexpr void set(Prop prop, bool enabled = true)
	{
		const auto bit = 1u << static_cast<unsigned>(prop);
		bits_ = enabled ? bits_ | bit : bits_ & ~bit;
	}
	static bool parse(std::string_view spec, PropSet &out, std::string *error = nullptr)
	{
		const auto trim = [](std::string_view value) {
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.front())))
				value.remove_prefix(1);
			while (!value.empty() &&
					std::isspace(static_cast<unsigned char>(value.back())))
				value.remove_suffix(1);
			return value;
		};
		spec = trim(spec);
		if (spec == "all") {
			out = all();
			return true;
		}
		if (spec == "none") {
			out = none();
			return true;
		}
		out = none();
		while (!spec.empty()) {
			const auto comma = spec.find(',');
			const auto token = trim(spec.substr(0, comma));
			if (!token.empty()) {
				std::string name(token);
				std::transform(
						name.begin(), name.end(), name.begin(), [](unsigned char c) {
							return static_cast<char>(std::tolower(c));
						});
				static constexpr std::pair<std::string_view, Prop> names[] = {
						{"boat", Prop::Boat}, {"car", Prop::Car}, {"crane", Prop::Crane},
						{"excavator", Prop::Excavator}, {"fountain", Prop::Fountain},
						{"helicopter", Prop::Helicopter}, {"jetbridge", Prop::Jetbridge},
						{"landmark", Prop::Landmark}, {"lighthouse", Prop::Lighthouse},
						{"plane", Prop::Plane}, {"playground", Prop::Playground},
						{"starship", Prop::Starship}, {"tombstone", Prop::Tombstone},
						{"tractor", Prop::Tractor}, {"windturbine", Prop::Windturbine}};
				const auto it = std::find_if(std::begin(names), std::end(names),
						[&](const auto &entry) { return entry.first == name; });
				if (it == std::end(names)) {
					if (error)
						*error =
								name +
								": not a prop family (all, none, or a list of boat, car, crane, excavator, fountain, helicopter, jetbridge, landmark, lighthouse, plane, playground, starship, tombstone, tractor, windturbine)";
					return false;
				}
				out.set(it->second);
			}
			if (comma == std::string_view::npos)
				break;
			spec.remove_prefix(comma + 1);
		}
		return true;
	}

	friend constexpr bool operator==(PropSet, PropSet) = default;

private:
	explicit constexpr PropSet(std::uint32_t bits) : bits_(bits) {}
	std::uint32_t bits_{};
};
} // namespace arnis::structures
