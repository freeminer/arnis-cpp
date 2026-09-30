#include "biome.h"
#include "celestial.h"
#include "land_cover/land_cover.h"
#include "../../arnis_adapter.h"
#include <cmath>
#include <algorithm>
#include <limits>
namespace arnis::biome
{
std::optional<std::string> ecoregion_biome(
		std::uint8_t lc, Climate climate, const std::optional<ecoregion::Ecoregion> &eco)
{
	if (!eco || climate == Climate::HotDesert || climate == Climate::ColdDesert ||
			climate == Climate::Boreal || climate == Climate::Tundra ||
			climate == Climate::IceCap)
		return std::nullopt;
	using namespace land_cover;
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
	case EcoBiome::Mediterranean:
		if (lc == LC_TREE_COVER)
			return "minecraft:forest";
		break;
	case EcoBiome::MontaneGrassland:
		if (lc == LC_TREE_COVER)
			return "minecraft:forest";
		if (lc == LC_SHRUBLAND || lc == LC_GRASSLAND)
			return "minecraft:meadow";
		break;
	case EcoBiome::TemperateConifer:
	case EcoBiome::Boreal:
		if (lc == LC_TREE_COVER || lc == LC_MOSS)
			return "minecraft:taiga";
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
		std::uint8_t water_distance, double above_m, int slope, double alpine_band_metres)
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
	if (above_m < -alpine_band_metres || climate == Climate::HotDesert ||
			climate == Climate::ColdDesert)
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
	const double a = std::abs(lat);
	if (lc == land_cover::LC_WATER) {
		// Rust's temperate baseline keeps water as river/ocean; the
		// latitude-based warm/lukewarm/cold variants are for other climates.
		if (c == Climate::Temperate)
			return wd >= 8 ? "minecraft:ocean" : "minecraft:river";
		bool cold = c == Climate::IceCap || c == Climate::Tundra || c == Climate::Boreal;
		if (wd < 8)
			return cold ? "minecraft:frozen_river" : "minecraft:river";
		if (cold)
			return wd >= 12 ? "minecraft:deep_frozen_ocean" : "minecraft:frozen_ocean";
		if (a < 23.5 || c == Climate::HotDesert || c == Climate::HotSteppe ||
				c == Climate::TropicalSavanna)
			return "minecraft:warm_ocean";
		if (a < 45)
			return wd >= 12 ? "minecraft:deep_lukewarm_ocean"
							: "minecraft:lukewarm_ocean";
		return wd >= 12 ? "minecraft:deep_cold_ocean" : "minecraft:cold_ocean";
	}
	switch (c) {
	case Climate::HotDesert:
	case Climate::ColdDesert:
		return "minecraft:desert";
	case Climate::HotSteppe:
	case Climate::TropicalSavanna:
	case Climate::DryContinental:
		return "minecraft:savanna";
	case Climate::ColdSteppe:
		return "minecraft:plains";
	case Climate::Tundra:
	case Climate::IceCap:
		return "minecraft:snowy_plains";
	case Climate::Boreal:
		return lc == land_cover::LC_WETLAND ? "minecraft:swamp"
			   : lc == land_cover::LC_BEACH
					   ? "minecraft:snowy_beach"
					   : (lc == land_cover::LC_TREE_COVER || lc == land_cover::LC_MOSS
										 ? "minecraft:taiga"
										 : "minecraft:snowy_plains");
	case Climate::Temperate:
		break;
	}
	if (lc == land_cover::LC_TREE_COVER)
		return a > 55 ? "minecraft:taiga"
					  : (a < 23.5 ? "minecraft:jungle" : "minecraft:forest");
	if (lc == land_cover::LC_SHRUBLAND)
		return a < 23.5 ? "minecraft:sparse_jungle" : "minecraft:savanna";
	if (lc == land_cover::LC_GRASSLAND || lc == land_cover::LC_CROPLAND ||
			lc == land_cover::LC_BUILT_UP)
		return "minecraft:plains";
	if (lc == land_cover::LC_BARE)
		return "minecraft:desert";
	if (lc == land_cover::LC_BEACH)
		return "minecraft:beach";
	if (lc == land_cover::LC_SNOW_ICE)
		return "minecraft:snowy_plains";
	if (lc == land_cover::LC_WETLAND)
		return "minecraft:swamp";
	if (lc == land_cover::LC_MANGROVES)
		return "minecraft:mangrove_swamp";
	if (lc == land_cover::LC_MOSS)
		return "minecraft:taiga";
	return "minecraft:plains";
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
		names.fill(std::string(celestial_body_biome(ground->celestial_body())));
		return names;
	}
	for (int zi = 0; zi < 4; ++zi)
		for (int xi = 0; xi < 4; ++xi) {
			const int world_x = chunk_x * 16 + xi * 4 + 2;
			const int world_z = chunk_z * 16 + zi * 4 + 2;
			const XZPoint local{world_x - world_origin_x, world_z - world_origin_z};
			const auto lc = ground->cover_class(local);
			const auto climate = ground->climate();
			const auto eco = ground->ecoregion_at(local);
			const auto water_distance = ground->water_distance(local);
			std::optional<std::string> selected;
			const auto bpm = ground->blocks_per_meter();
			if (ground->snow_threshold() != std::numeric_limits<int>::max() && bpm > 0.0)
				selected = mountain_biome(lc, climate, water_distance,
						(ground->level_exact(local) - ground->snow_threshold()) / bpm,
						ground->slope(local));
			if (!selected)
				selected = ecoregion_biome(lc, climate, eco);
			names[std::size_t(zi * 4 + xi)] = selected.value_or(
					biome_for_class(lc, climate, center_latitude, water_distance));
		}
	return names;
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
