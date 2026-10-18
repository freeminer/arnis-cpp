#pragma once

#include <optional>

namespace arnis::building_style
{

enum class BuildingRegion
{
	NorthAmerica,
	Europe,
	Other,
};

enum class MaterialRegion
{
	NorthAmerica,
	NorthAmericaSunbelt,
	Canada,
	BritishIsles,
	BrickNorth,
	France,
	Central,
	CentralEast,
	Nordic,
	Mediterranean,
	EastEurope,
	CentralAsia,
	MiddleEast,
	SubSaharanAfrica,
	SouthAsia,
	SoutheastAsia,
	EastAsia,
	Korea,
	Japan,
	LatinAmerica,
	SouthernCone,
	Australasia,
	Other,
};

struct RoofHabit
{
	double house;
	double block;
	double block_max_m;
	double hipped;
};

MaterialRegion material_region_at(double latitude, double longitude);
BuildingRegion building_region(MaterialRegion region);
BuildingRegion building_region_at(double latitude, double longitude);
std::optional<RoofHabit> roof_habit(MaterialRegion region);

} // namespace arnis::building_style
