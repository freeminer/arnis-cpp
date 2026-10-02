#pragma once

#include "caves_biomes.h"
#include "caves_noise.h"
#include "caves_rng.h"

#include <cstdint>
#include <string_view>

namespace arnis::caves
{
inline constexpr int VANILLA_FLOOR = -64;

enum class CaveTheme
{
	Normal,
	Lush,
	Dripstone,
	DeepDark,
	Mushroom,
	Ice,
	Amethyst,
	Volcanic
};

inline NormalNoise cave_theme_noise(std::int64_t seed, std::string_view name)
{
	auto factory = XoroRandom::from_seed(seed ^ 0x0DEC0DECL).fork_positional();
	auto random = XoroRandom::from_hash_of(factory.first, factory.second, name);
	return NormalNoise::create(random, -7, {1.0});
}

struct ThemeSelector
{
	NormalNoise lush;
	NormalNoise drip;
	NormalNoise sculk;
	NormalNoise shroom;
	NormalNoise ice;
	NormalNoise crystal;
	NormalNoise volcanic;
	BiomeAmounts amounts;
	int y_shift;

	ThemeSelector(std::int64_t seed, BiomeAmounts biome_amounts, int floor_y) :
			lush(cave_theme_noise(seed, "lush_select")),
			drip(cave_theme_noise(seed, "dripstone_select")),
			sculk(cave_theme_noise(seed, "sculk_select")),
			shroom(cave_theme_noise(seed, "shroom_select")),
			ice(cave_theme_noise(seed, "ice_select")),
			crystal(cave_theme_noise(seed, "crystal_select")),
			volcanic(cave_theme_noise(seed, "volcanic_select")), amounts(biome_amounts),
			y_shift(floor_y - VANILLA_FLOOR)
	{
	}

	bool hit(const NormalNoise &noise, double threshold, double amount, double x,
			double z, double scale) const
	{
		return noise.get_value(x * scale, 0.0, z * scale) >
			   BiomeAmounts::effective_threshold(threshold, amount);
	}

	CaveTheme at(int x, int y, int z, int surface_y) const
	{
		constexpr double margin = 0.06;
		const int vanilla_y = y - y_shift;
		const int vanilla_surface = surface_y - y_shift;
		const double scale = vanilla_y < -30 ? 0.72 : 1.15;
		if (vanilla_y <= -40 &&
				hit(volcanic, 0.18 + margin, amounts.volcanic, x, z, scale))
			return CaveTheme::Volcanic;
		if (vanilla_y <= -35 && hit(sculk, 0.17 + margin, amounts.deepdark, x, z, scale))
			return CaveTheme::DeepDark;
		if (vanilla_y >= -36 && vanilla_surface >= 115 &&
				hit(ice, 0.20 + margin, amounts.ice, x, z, scale))
			return CaveTheme::Ice;
		if (hit(lush, 0.17 + margin, amounts.lush, x, z, scale))
			return CaveTheme::Lush;
		if (hit(drip, 0.18 + margin, amounts.dripstone, x, z, scale))
			return CaveTheme::Dripstone;
		if (hit(shroom, 0.21 + margin, amounts.mushroom, x, z, scale))
			return CaveTheme::Mushroom;
		if (hit(crystal, 0.25 + margin, amounts.amethyst, x, z, scale))
			return CaveTheme::Amethyst;
		return CaveTheme::Normal;
	}
};

inline const char *cave_theme_tag(CaveTheme theme)
{
	switch (theme) {
	case CaveTheme::Lush:
		return "lush";
	case CaveTheme::Dripstone:
		return "dripstone";
	case CaveTheme::DeepDark:
		return "deepdark";
	case CaveTheme::Mushroom:
		return "mushroom";
	case CaveTheme::Ice:
		return "ice";
	case CaveTheme::Amethyst:
		return "amethyst";
	case CaveTheme::Volcanic:
		return "volcanic";
	case CaveTheme::Normal:
	default:
		return "any";
	}
}
} // namespace arnis::caves
