#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <memory>
#include <filesystem>
#include <functional>
#include <vector>
#include <map>
#include "geo_grid.h"

namespace arnis::cartesian
{
struct XZPoint;
}

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
	std::int32_t cell_{0};
	std::int32_t origin_x_{0}, origin_z_{0};
	std::int32_t align_x_{0}, align_z_{0};
	std::size_t width_{0}, height_{0};
	std::vector<std::uint16_t> ids_;
	EcoMap() = default;
	std::uint16_t id_at_local(std::int32_t x, std::int32_t z) const;

public:
	static std::optional<EcoMap> load(const std::filesystem::path &path);
	static std::optional<EcoMap> build(std::size_t world_width, std::size_t world_height,
			std::pair<std::int32_t, std::int32_t> align,
			const std::function<std::pair<double, double>(double, double)> &geo);
	std::optional<std::uint16_t> id_at(double latitude, double longitude) const;
	std::optional<Ecoregion> at(const cartesian::XZPoint &coord) const;
	std::optional<EcoMap> resample(std::size_t world_width, std::size_t world_height,
			const std::function<std::pair<std::int32_t, std::int32_t>(
					std::int32_t, std::int32_t)> &source) const;
	bool is_local() const { return cell_ > 0 && !ids_.empty(); }
	std::pair<std::int32_t, std::int32_t> alignment() const
	{
		return {align_x_, align_z_};
	}
	std::vector<std::pair<std::uint16_t, std::size_t>> by_area() const;
	bool has_gaps() const;
	std::optional<std::string> dominant_tree_pack() const;
};
}
