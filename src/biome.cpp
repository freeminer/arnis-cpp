#include "biome.h"
#include "celestial.h"
#include "ground_decoration.h"
#include "ground_generation.h"
#include "land_cover/land_cover.h"
#include "../../arnis_adapter.h"
#include <bit>
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>
namespace arnis::biome
{
namespace
{
constexpr std::array<std::uint16_t, 9> RED_EARTH_ECOREGIONS{
		207, 208, 209, 210, 211, 213, 214, 215, 429};
constexpr int ROCK_FACE_SLOPE = 4;

std::optional<std::string> cell_mountain_biome(const Cell &cell, Climate climate)
{
	using namespace land_cover;
	if (!cell.above_snow_m)
		return std::nullopt;
	const double above = *cell.above_snow_m;
	if (above >= 0.0) {
		if (cell.slope <= 1)
			return "minecraft:snowy_plains";
		if (cell.slope <= 6)
			return "minecraft:snowy_slopes";
		return "minecraft:jagged_peaks";
	}
	if (above < -ground_decoration::ALPINE_BAND_METRES || climate == Climate::HotDesert ||
			climate == Climate::ColdDesert)
		return std::nullopt;
	switch (cell.land_cover) {
	case LC_GRASSLAND:
	case LC_SHRUBLAND:
	case LC_MOSS:
		return "minecraft:meadow";
	case LC_BARE:
		return "minecraft:stony_peaks";
	case LC_SNOW_ICE:
		return "minecraft:snowy_slopes";
	default:
		return std::nullopt;
	}
}

std::string cell_land_biome(const Cell &cell, Climate climate, double latitude)
{
	using namespace land_cover;
	const double abs_lat = std::abs(latitude);
	const bool tropical = abs_lat < 23.5 && climate == Climate::Temperate;
	if (cell.land_cover == LC_MANGROVES)
		return "minecraft:mangrove_swamp";
	switch (climate) {
	case Climate::HotDesert:
	case Climate::ColdDesert:
		return "minecraft:desert";
	case Climate::HotSteppe:
	case Climate::TropicalSavanna:
	case Climate::DryContinental:
		return cell.land_cover == LC_WETLAND ? "minecraft:swamp" : "minecraft:savanna";
	case Climate::Tundra:
	case Climate::IceCap:
		if (cell.land_cover == LC_TREE_COVER)
			return "minecraft:snowy_taiga";
		return cell.land_cover == LC_BEACH ? "minecraft:snowy_beach"
										   : "minecraft:snowy_plains";
	case Climate::ColdSteppe:
	case Climate::Boreal:
	case Climate::Temperate:
		if (cell.land_cover == LC_TREE_COVER)
			return climate == Climate::Boreal || abs_lat > 55.0 ? "minecraft:taiga"
				   : tropical									? "minecraft:jungle"
																: "minecraft:forest";
		if (cell.land_cover == LC_SHRUBLAND)
			return climate == Climate::Boreal ? "minecraft:taiga"
				   : tropical				  ? "minecraft:sparse_jungle"
											  : "minecraft:plains";
		if (cell.land_cover == LC_MOSS)
			return "minecraft:taiga";
		if (cell.land_cover == LC_BARE && cell.shore)
			return "minecraft:stony_shore";
		if (cell.land_cover == LC_BARE && cell.slope >= ROCK_FACE_SLOPE)
			return "minecraft:stony_peaks";
		if (cell.land_cover == LC_BEACH)
			return "minecraft:beach";
		if (cell.land_cover == LC_SNOW_ICE)
			return "minecraft:snowy_plains";
		if (cell.land_cover == LC_WETLAND)
			return "minecraft:swamp";
		return "minecraft:plains";
	}
	return "minecraft:plains";
}

std::optional<float> game_temperature(std::string_view name)
{
	if (name == "minecraft:stony_shore")
		return .2;
	if (name == "minecraft:taiga")
		return .25;
	if (name == "minecraft:old_growth_pine_taiga")
		return .3;
	if (name == "minecraft:meadow")
		return .5;
	if (name == "minecraft:birch_forest")
		return .6;
	if (name == "minecraft:forest")
		return .7;
	if (name == "minecraft:plains")
		return .8;
	if (name == "minecraft:stony_peaks")
		return 1.0;
	return std::nullopt;
}

static std::optional<int> game_snow_y(std::string_view name)
{
	const auto temperature = game_temperature(name);
	if (!temperature)
		return std::nullopt;
	return -46 + static_cast<int>((*temperature - 0.15f) / 0.00125f);
}

std::string snow_safe(std::string name, int y, bool trees)
{
	const int canopy = y + 8 + (trees ? 24 : 2);
	while (true) {
		const auto snow_y = game_snow_y(name);
		if (!snow_y)
			break;
		if (canopy < *snow_y)
			break;
		if (name == "minecraft:taiga")
			name = "minecraft:old_growth_pine_taiga";
		else if (name == "minecraft:old_growth_pine_taiga" ||
				 name == "minecraft:birch_forest")
			name = "minecraft:forest";
		else if (name == "minecraft:meadow")
			name = "minecraft:plains";
		else if (name == "minecraft:stony_shore")
			name = "minecraft:stony_peaks";
		else
			break;
	}
	return name;
}

Cell classify_cell(const Ground &ground, const XZPoint &coord, const XZPoint &eco_coord,
		int snow_y, const std::optional<int> &sea_max)
{
	using namespace land_cover;
	const auto lc = ground.cover_class(coord);
	const int y = ground.level(coord);
	bool shore = false;
	if (lc == LC_BARE) {
		for (const int radius : {4, 8}) {
			for (const auto &[dx, dz] : std::array<std::pair<int, int>, 8>{{{radius, 0},
						 {-radius, 0}, {0, radius}, {0, -radius}, {radius, radius},
						 {radius, -radius}, {-radius, radius}, {-radius, -radius}}}) {
				if (ground.cover_class({coord.x + dx, coord.z + dz}) == LC_WATER) {
					shore = true;
					break;
				}
			}
			if (shore)
				break;
		}
	}
	Cell cell;
	cell.land_cover = lc;
	cell.inland_water = lc == LC_WATER &&
						(ground.inland_water_mapped(coord) || (sea_max && y > *sea_max));
	cell.shore = shore;
	if (ground.elevation_enabled && snow_y != std::numeric_limits<int>::max())
		cell.above_snow_m = snow_y == std::numeric_limits<int>::min()
									? std::numeric_limits<double>::infinity()
									: double(y - snow_y) / ground.blocks_per_meter();
	cell.slope = ground.slope(coord);
	cell.y = y;
	cell.eco = ground.ecoregion_at(eco_coord);
	return cell;
}
} // namespace

bool climate_is_dry(Climate climate)
{
	return climate == Climate::HotDesert || climate == Climate::ColdDesert ||
		   climate == Climate::HotSteppe || climate == Climate::ColdSteppe ||
		   climate == Climate::DryContinental;
}

std::pair<int, int> frayed_ecoregion_offset(int x, int z)
{
	constexpr std::uint32_t salt = 0xF4A73D19u;
	const auto shift = [x, z](std::uint32_t value) {
		return static_cast<int>(
				(ground_generation::value_noise_salted(x, z, 32, value) - .5) * 80.0);
	};
	const auto add_wrapped = [](int a, int b) {
		return std::bit_cast<std::int32_t>(
				static_cast<std::uint32_t>(a) + static_cast<std::uint32_t>(b));
	};
	return {add_wrapped(x, shift(salt)),
			add_wrapped(z, shift((salt << 11) | (salt >> 21)))};
}

bool red_earth(Climate climate, const std::optional<ecoregion::Ecoregion> &eco)
{
	return climate_is_dry(climate) && eco &&
		   std::find(RED_EARTH_ECOREGIONS.begin(), RED_EARTH_ECOREGIONS.end(), eco->id) !=
				   RED_EARTH_ECOREGIONS.end();
}

std::string cell_biome(const Cell &cell, Climate climate, double latitude)
{
	using namespace land_cover;
	if ((climate == Climate::Tundra || climate == Climate::IceCap) &&
			std::abs(latitude) < 60.0)
		climate = Climate::Boreal;
	std::string name;
	if (cell.land_cover == LC_WATER) {
		const bool frozen = climate == Climate::Tundra || climate == Climate::IceCap ||
							(cell.above_snow_m && *cell.above_snow_m >= 0.0);
		if (cell.inland_water)
			name = frozen ? "minecraft:frozen_river" : "minecraft:river";
		else if (frozen)
			name = "minecraft:frozen_ocean";
		else if (std::abs(latitude) < 23.5 || climate == Climate::HotDesert ||
				 climate == Climate::HotSteppe || climate == Climate::TropicalSavanna)
			name = "minecraft:warm_ocean";
		else if (std::abs(latitude) < 45.0)
			name = "minecraft:lukewarm_ocean";
		else
			name = "minecraft:cold_ocean";
		return name;
	}
	if (const auto mountain = cell_mountain_biome(cell, climate))
		name = *mountain;
	else if (const auto eco = ecoregion_biome(cell.land_cover, climate, cell.eco))
		name = *eco;
	else
		name = cell_land_biome(cell, climate, latitude);
	if (cell.above_snow_m)
		name = snow_safe(std::move(name), cell.y, cell.land_cover == LC_TREE_COVER);
	return name;
}

std::optional<std::string> ecoregion_biome(
		std::uint8_t lc, Climate climate, const std::optional<ecoregion::Ecoregion> &eco)
{
	using namespace land_cover;
	const bool open = lc == LC_TREE_COVER || lc == LC_SHRUBLAND || lc == LC_GRASSLAND ||
					  lc == LC_BARE;
	// Keep the public classifier in step with Rust's ecoregion_biome(): the
	// Australian/Colorado red-earth override does not require a warm ecoregion
	// biome, but does require both a dry climate and one of the red-earth IDs.
	if (open && red_earth(climate, eco))
		return lc == LC_TREE_COVER ? "minecraft:wooded_badlands" : "minecraft:badlands";
	if (!eco || climate == Climate::HotDesert || climate == Climate::ColdDesert ||
			climate == Climate::Boreal || climate == Climate::Tundra ||
			climate == Climate::IceCap)
		return std::nullopt;
	using ecoregion::EcoBiome;
	const bool dry_warm = climate != Climate::ColdSteppe;
	switch (eco->biome) {
	case EcoBiome::MoistTropical:
		if (lc == LC_TREE_COVER)
			return "minecraft:jungle";
		if (lc == LC_SHRUBLAND)
			return "minecraft:sparse_jungle";
		break;
	case EcoBiome::DryTropical:
		if (lc == LC_TREE_COVER)
			return "minecraft:sparse_jungle";
		if (lc == LC_SHRUBLAND)
			return "minecraft:sparse_jungle";
		break;
	case EcoBiome::TropicalConifer:
	case EcoBiome::TemperateBroadleaf:
	case EcoBiome::TemperateGrassland:
		if (lc == LC_TREE_COVER)
			return "minecraft:forest";
		break;
	case EcoBiome::Mediterranean:
		if (lc == LC_TREE_COVER)
			return "minecraft:forest";
		if (lc == LC_SHRUBLAND && dry_warm)
			return "minecraft:savanna";
		break;
	case EcoBiome::MontaneGrassland:
		if (lc == LC_TREE_COVER)
			return "minecraft:forest";
		if (lc == LC_SHRUBLAND || lc == LC_GRASSLAND)
			return "minecraft:meadow";
		break;
	case EcoBiome::TemperateConifer:
	case EcoBiome::Boreal:
		if (lc == LC_TREE_COVER)
			return "minecraft:taiga";
		if (lc == LC_SHRUBLAND)
			return "minecraft:taiga";
		break;
	case EcoBiome::Tundra:
		if (lc == LC_SHRUBLAND)
			return "minecraft:taiga";
		break;
	case EcoBiome::TropicalGrassland:
	case EcoBiome::Desert:
		if (dry_warm && (lc == LC_TREE_COVER || lc == LC_SHRUBLAND || lc == LC_GRASSLAND))
			return "minecraft:savanna";
		break;
	default:
		break;
	}
	return std::nullopt;
}

std::optional<std::string> mountain_biome(std::uint8_t lc, Climate climate,
		std::uint8_t water_distance, double above_m, int slope)
{
	using namespace land_cover;
	if (lc == LC_WATER)
		return (above_m >= 0.0 && water_distance < 8)
					   ? std::optional<std::string>{"minecraft:frozen_river"}
					   : std::nullopt;
	if (above_m >= 0.0) {
		if (slope <= 1)
			return "minecraft:snowy_plains";
		if (slope <= 6)
			return "minecraft:snowy_slopes";
		return "minecraft:jagged_peaks";
	}
	if (above_m < -ground_decoration::ALPINE_BAND_METRES ||
			climate == Climate::HotDesert || climate == Climate::ColdDesert)
		return std::nullopt;
	switch (lc) {
	case LC_GRASSLAND:
	case LC_SHRUBLAND:
	case LC_MOSS:
		return "minecraft:meadow";
	case LC_BARE:
		return "minecraft:stony_peaks";
	case LC_SNOW_ICE:
		return "minecraft:snowy_slopes";
	default:
		return std::nullopt;
	}
}

std::string biome_for_class(std::uint8_t lc, Climate c, double lat, std::uint8_t wd)
{
	// Keep the simple sampler API on the same climate/cover rules as Rust's
	// cell_biome(). Its water-distance input is the only inland-water hint it
	// has; the active ground-backed path uses the richer Cell classification.
	Cell cell;
	cell.land_cover = lc;
	cell.inland_water = lc == land_cover::LC_WATER && wd < 8;
	return cell_biome(cell, c, lat);
}

unsigned biome_bits_per_index(std::size_t palette_size)
{
	if (palette_size <= 1)
		return 0;
	unsigned bits = 0;
	for (std::size_t n = palette_size - 1; n; n >>= 1)
		++bits;
	return bits;
}
std::vector<std::int64_t> pack_chunk_biome_indices(
		const std::array<std::uint8_t, 16> &indices, unsigned bits)
{
	if (bits == 0 || bits > 6)
		return {};
	const std::size_t per_long = 64 / bits, words = (64 + per_long - 1) / per_long;
	std::vector<std::uint64_t> packed(words, 0);
	const std::uint64_t mask = (std::uint64_t(1) << bits) - 1;
	for (std::size_t cell = 0; cell < 64; ++cell) {
		const auto word = cell / per_long, shift = (cell % per_long) * bits;
		packed[word] |= (std::uint64_t(indices[cell % 16]) & mask) << shift;
	}
	return {packed.begin(), packed.end()};
}
ChunkBiomeData build_chunk_biomes(int chunk_x, int chunk_z, const BiomeSampler &sample,
		Climate climate, double latitude)
{
	ChunkBiomeData out;
	std::array<std::string, 16> names{};
	for (int z = 0; z < 4; ++z)
		for (int x = 0; x < 4; ++x) {
			const auto value =
					sample ? sample(chunk_x * 16 + x * 4 + 2, chunk_z * 16 + z * 4 + 2)
						   : BiomeSample{};
			names[std::size_t(z * 4 + x)] = biome_for_class(
					value.land_cover, climate, latitude, value.water_distance);
		}
	for (std::size_t i = 0; i < names.size(); ++i) {
		auto it = std::find(out.palette.begin(), out.palette.end(), names[i]);
		if (it == out.palette.end()) {
			out.horizontal_indices[i] = std::uint8_t(out.palette.size());
			out.palette.push_back(names[i]);
		} else
			out.horizontal_indices[i] =
					std::uint8_t(std::distance(out.palette.begin(), it));
	}
	out.bits_per_index = biome_bits_per_index(out.palette.size());
	if (out.bits_per_index)
		out.packed_indices =
				pack_chunk_biome_indices(out.horizontal_indices, out.bits_per_index);
	return out;
}

std::array<std::string, 16> chunk_biome_names(int chunk_x, int chunk_z,
		const ::arnis::Ground *ground, double center_latitude, int world_origin_x,
		int world_origin_z)
{
	std::array<std::string, 16> names;
	names.fill("minecraft:plains");
	if (!ground)
		return names;
	if (!ground->is_earth()) {
		names.fill(std::string(celestial_body_biome(ground->body())));
		return names;
	}
	const auto climate = ground->climate();
	const int snow_y = ground->elevation_enabled ? ground->snow_threshold_y()
												 : std::numeric_limits<int>::max();
	const auto sea_max = ground->sea_surface_max_y();
	for (int zi = 0; zi < 4; ++zi)
		for (int xi = 0; xi < 4; ++xi) {
			const int world_x = chunk_x * 16 + xi * 4 + 2;
			const int world_z = chunk_z * 16 + zi * 4 + 2;
			const XZPoint local{world_x - world_origin_x, world_z - world_origin_z};
			XZPoint eco_coord = local;
			if (climate_is_dry(climate)) {
				const auto [sample_x, sample_z] =
						frayed_ecoregion_offset(world_x, world_z);
				eco_coord = {sample_x - world_origin_x, sample_z - world_origin_z};
			}
			const auto cell = classify_cell(*ground, local, eco_coord, snow_y, sea_max);
			auto name = cell_biome(cell, climate, center_latitude);
			if (ground->winter())
				name = winter_biome(name);
			names[std::size_t(zi * 4 + xi)] = std::move(name);
		}
	return names;
}

std::string winter_biome(const std::string &name)
{
	if (name == "minecraft:river")
		return "minecraft:frozen_river";
	if (name == "minecraft:beach" || name == "minecraft:stony_shore")
		return "minecraft:snowy_beach";
	if (name == "minecraft:ocean" || name == "minecraft:lukewarm_ocean" ||
			name == "minecraft:warm_ocean")
		return "minecraft:cold_ocean";
	if (name == "minecraft:forest" || name == "minecraft:taiga" ||
			name == "minecraft:old_growth_pine_taiga" ||
			name == "minecraft:wooded_badlands" || name == "minecraft:birch_forest" ||
			name == "minecraft:dark_forest" || name == "minecraft:jungle" ||
			name == "minecraft:sparse_jungle" || name == "minecraft:mangrove_swamp" ||
			name == "minecraft:swamp")
		return "minecraft:snowy_taiga";
	if (name == "minecraft:meadow" || name == "minecraft:stony_peaks")
		return "minecraft:snowy_slopes";
	if (name.find("snowy") != std::string::npos ||
			name.find("frozen") != std::string::npos ||
			name.find("cold") != std::string::npos || name.ends_with("_peaks") ||
			name == "minecraft:grove" || name == "minecraft:ice_spikes")
		return name;
	return "minecraft:snowy_plains";
}

ChunkBiomeData build_chunk_biomes_for_ground(int chunk_x, int chunk_z,
		const ::arnis::Ground *ground, double center_latitude, int world_origin_x,
		int world_origin_z)
{
	if (!ground)
		return build_chunk_biomes(
				chunk_x, chunk_z, {}, Climate::Temperate, center_latitude);
	const auto names = chunk_biome_names(
			chunk_x, chunk_z, ground, center_latitude, world_origin_x, world_origin_z);
	ChunkBiomeData out;
	for (std::size_t i = 0; i < names.size(); ++i) {
		auto it = std::find(out.palette.begin(), out.palette.end(), names[i]);
		if (it == out.palette.end()) {
			out.horizontal_indices[i] = static_cast<std::uint8_t>(out.palette.size());
			out.palette.push_back(names[i]);
		} else {
			out.horizontal_indices[i] =
					static_cast<std::uint8_t>(std::distance(out.palette.begin(), it));
		}
	}
	out.bits_per_index = biome_bits_per_index(out.palette.size());
	if (out.bits_per_index)
		out.packed_indices =
				pack_chunk_biome_indices(out.horizontal_indices, out.bits_per_index);
	return out;
}
}
