#pragma once
#include <string>
#include <string_view>

namespace arnis::caves
{
struct BiomeAmounts
{
	double lush = 1, dripstone = 1, deepdark = 1, mushroom = 1, ice = 1, amethyst = 1,
		   volcanic = 1, coral = 1;
	static BiomeAmounts defaults() { return {}; }
	static BiomeAmounts parse(std::string_view spec, std::string *error = nullptr);
	static double effective_threshold(double base, double amount);
};
// Rust caves::biome_amounts fallback used by generation when validation was
// bypassed by an embedding caller.
BiomeAmounts biome_amounts_for_generation(std::string_view spec);
}
