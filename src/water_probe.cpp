#include "water_probe.h"

#include <cmath>
#include <limits>
#include <string_view>

namespace arnis::water_probe
{
namespace
{
constexpr int ABOVE = 110, BELOW = 40, FOOT_BELOW = 4, FOOT_ABOVE = 3;

bool ends_with(std::string_view s, std::string_view suffix)
{
	return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
}

bool walk_through(std::string_view name)
{
	static constexpr std::array<std::string_view, 40> open{"air", "cave_air", "void_air",
			"light", "snow", "short_grass", "tall_grass", "short_dry_grass",
			"tall_dry_grass", "fern", "large_fern", "dead_bush", "bush", "firefly_bush",
			"vine", "sunflower", "lilac", "rose_bush", "peony", "dandelion", "poppy",
			"blue_orchid", "allium", "azure_bluet", "oxeye_daisy", "cornflower",
			"lily_of_the_valley", "torchflower", "pink_petals", "wildflowers",
			"leaf_litter", "rail", "powered_rail", "detector_rail", "activator_rail",
			"redstone_wire", "tripwire", "lever", "flower_pot", "lily_pad"};
	if (std::find(open.begin(), open.end(), name) != open.end())
		return true;
	static constexpr std::array<std::string_view, 8> suffixes{"_carpet", "_tulip",
			"_sapling", "_pressure_plate", "_button", "_sign", "_banner", "_torch"};
	return std::any_of(suffixes.begin(), suffixes.end(),
			[&](auto suffix) { return ends_with(name, suffix); });
}

bool too_tall(std::string_view name)
{
	static constexpr std::array<std::string_view, 14> suffixes{"cauldron", "barrel",
			"redstone_block", "composter", "_fence", "_fence_gate", "_wall", "_pane",
			"_bars", "glass_pane", "_log", "_wood", "_stem", "_hyphae"};
	return std::any_of(suffixes.begin(), suffixes.end(),
			[&](auto suffix) { return ends_with(name, suffix); });
}
} // namespace

Sample Sample::make(std::optional<int> water, std::optional<int> overhead)
{
	const auto narrow = [](int y) {
		return static_cast<std::int16_t>(
				std::clamp(y, int(std::numeric_limits<std::int16_t>::min()),
						int(std::numeric_limits<std::int16_t>::max())));
	};
	return {water ? std::optional<std::int16_t>(narrow(*water)) : std::nullopt,
			overhead ? std::optional<std::int16_t>(narrow(*overhead)) : std::nullopt};
}

std::optional<int> Sample::water() const
{
	return water_y ? std::optional<int>(*water_y) : std::nullopt;
}

std::optional<int> Sample::clearance() const
{
	if (!water_y)
		return std::nullopt;
	return overhead_y ? int(*overhead_y) - int(*water_y) - 1 : ABOVE;
}

std::optional<double> Footing::nearest(double y) const
{
	std::optional<double> best;
	for (unsigned k = 0; k < 8; ++k) {
		if (!(levels & (1U << k)))
			continue;
		const double feet = double(base) + k + ((half & (1U << k)) ? 0.5 : 1.0);
		if (!best || std::abs(feet - y) < std::abs(*best - y) ||
				(std::abs(feet - y) == std::abs(*best - y) && feet < *best))
			best = feet;
	}
	return best;
}

bool Footing::post() const
{
	for (int k = FOOT_BELOW - 1; k <= FOOT_BELOW + 1; ++k)
		if (levels & (1U << k))
			return false;
	return is_thin;
}

std::pair<int, int> water_span(int expect)
{
	return {expect - BELOW, expect + ABOVE};
}

Sample water_column(const std::function<Spot(int)> &block, int expect, int top)
{
	std::optional<int> overhead;
	const auto [bottom, upper] = water_span(expect);
	for (int y = std::min(top, upper); y >= bottom; --y) {
		switch (block(y)) {
		case Spot::Open:
			break;
		case Spot::Water:
			return Sample::make(y, overhead);
		case Spot::Solid:
		case Spot::Half:
		case Spot::Tall:
		case Spot::Leaves:
			overhead = y;
			break;
		}
	}
	return Sample::make(std::nullopt, overhead);
}

Footing footing(const std::function<Spot(int)> &block, int expect)
{
	const int base = expect - FOOT_BELOW;
	std::array<Spot, FOOT_BELOW + FOOT_ABOVE + 3> spots{};
	for (std::size_t i = 0; i < spots.size(); ++i)
		spots[i] = block(base + static_cast<int>(i));
	std::uint8_t levels = 0, half = 0;
	for (int k = 0; k <= FOOT_BELOW + FOOT_ABOVE; ++k) {
		const auto floor = spots[k];
		if ((floor == Spot::Solid || floor == Spot::Half) && spots[k + 1] == Spot::Open &&
				(spots[k + 2] == Spot::Open || spots[k + 2] == Spot::Leaves)) {
			levels |= static_cast<std::uint8_t>(1U << k);
			if (floor == Spot::Half)
				half |= static_cast<std::uint8_t>(1U << k);
		}
	}
	const bool thin = std::all_of(spots.begin() + FOOT_BELOW + 2,
			spots.begin() + FOOT_BELOW + 5, [](Spot s) { return s != Spot::Solid; });
	return {static_cast<std::int16_t>(
					std::clamp(base, int(std::numeric_limits<std::int16_t>::min()),
							int(std::numeric_limits<std::int16_t>::max()))),
			levels, half, thin};
}

Spot walk_spot(const std::string &block_name, bool bottom_slab)
{
	std::string_view name(block_name);
	if (const auto colon = name.rfind(':'); colon != std::string_view::npos)
		name.remove_prefix(colon + 1);
	if (walk_through(name))
		return Spot::Open;
	if (ends_with(name, "_slab") && bottom_slab)
		return Spot::Half;
	if (ends_with(name, "_leaves"))
		return Spot::Leaves;
	if (too_tall(name))
		return Spot::Tall;
	return Spot::Solid;
}
} // namespace arnis::water_probe
