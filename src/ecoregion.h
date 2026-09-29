#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <memory>
#include <filesystem>
#include <vector>
#include "geo_grid.h"

namespace arnis::ecoregion
{
enum class EcoBiome : std::uint8_t
{
	MoistTropical,
	DryTropical,
	TropicalConifer,
	TemperateBroadleaf,
	TemperateConifer,
	Boreal,
	TropicalGrassland,
	TemperateGrassland,
	Flooded,
	MontaneGrassland,
	Tundra,
	Mediterranean,
	Desert,
	Mangroves
};
enum class Realm : std::uint8_t
{
	Afrotropic,
	Antarctica,
	Australasia,
	Indomalayan,
	Nearctic,
	Neotropic,
	Oceania,
	Palearctic
};
struct Ecoregion
{
	std::uint16_t id = 0;
	EcoBiome biome{};
	Realm realm{};
};

std::optional<Ecoregion> lookup(std::uint16_t id);
std::optional<std::string_view> name(std::uint16_t id);
std::optional<std::pair<std::string_view, std::string_view>> tree_mix(std::uint16_t id);
bool palms_belong(Ecoregion eco, double absolute_latitude);
bool realm_is_americas(Realm realm);

class EcoMap
{
	std::shared_ptr<std::vector<std::uint8_t>> bytes_;
	std::optional<geo_grid::TiledGrid> grid_;
	EcoMap() = default;

public:
	static std::optional<EcoMap> load(const std::filesystem::path &path);
	std::optional<std::uint16_t> id_at(double latitude, double longitude) const;
};
}
